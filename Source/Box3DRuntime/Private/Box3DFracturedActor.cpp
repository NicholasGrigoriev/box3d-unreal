#include "Box3DFracturedActor.h"

#include "Box3DConversion.h"
#include "Box3DCooking.h"
#include "Box3DTypes.h"
#include "Box3DRuntime.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "PhysicsEngine/BodySetup.h"
#include "ProceduralMeshComponent.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	using namespace Box3D::Fracture;

	/// Convex hull of a cm point cloud as a fracture proxy, faces wound CCW seen
	/// from outside (the b3 half-edge convention matches the proxy convention).
	/// Runs through b3CreateHull at the usual cm -> m seam; scale must already be
	/// baked into the points — hulling after scaling also keeps windings correct
	/// under negative (mirroring) scale, because faces are derived fresh.
	bool BuildConvexProxy(const TArray<FVector>& PointsCm, FFractureProxy& OutProxy)
	{
		if (PointsCm.Num() < 4)
		{
			return false;
		}

		TArray<b3Vec3> Points;
		Points.Reserve(PointsCm.Num());
		for (const FVector& Point : PointsCm)
		{
			Points.Add(Box3D::ToB3(Point));
		}

		// 64 matches the cooking hull budget; hull indices are uint8 anyway.
		b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), 64);
		if (Hull == nullptr)
		{
			return false;
		}

		const b3Vec3* HullPoints = b3GetHullPoints(Hull);
		const b3HullHalfEdge* Edges = b3GetHullEdges(Hull);
		const b3HullFace* Faces = b3GetHullFaces(Hull);

		FFractureProxy Proxy;
		Proxy.Vertices.Reserve(Hull->vertexCount);
		for (int32 Index = 0; Index < Hull->vertexCount; ++Index)
		{
			Proxy.Vertices.Add(Box3D::ToUE(HullPoints[Index]));
		}

		Proxy.Faces.Reserve(Hull->faceCount);
		for (int32 FaceIndex = 0; FaceIndex < Hull->faceCount; ++FaceIndex)
		{
			TArray<int32> Loop;
			const uint8 FirstEdge = Faces[FaceIndex].edge;
			uint8 EdgeIndex = FirstEdge;
			int32 Guard = 0;
			do
			{
				Loop.Add(Edges[EdgeIndex].origin);
				EdgeIndex = Edges[EdgeIndex].next;
			} while (EdgeIndex != FirstEdge && ++Guard <= Hull->edgeCount);

			if (Loop.Num() >= 3)
			{
				Proxy.Faces.Add(MoveTemp(Loop));
			}
		}
		b3DestroyHull(Hull);

		if (Proxy.Vertices.Num() < 4 || Proxy.Faces.Num() < 4)
		{
			return false;
		}
		OutProxy = MoveTemp(Proxy);
		return true;
	}

	/// Point cloud of the authored convex elements, elem transforms applied.
	void GatherConvexElemPoints(const FKAggregateGeom& AggGeom, const FVector& Scale, TArray<FVector>& OutPoints)
	{
		for (const FKConvexElem& Elem : AggGeom.ConvexElems)
		{
			const FTransform ElemTransform = Elem.GetTransform();
			for (const FVector& Vertex : Elem.VertexData)
			{
				OutPoints.Add(ElemTransform.TransformPosition(Vertex) * Scale);
			}
		}
	}

	/// Point cloud approximating the simple-collision primitives: box corners
	/// plus axis extremes of spheres and capsule hemispheres (inscribed for the
	/// round shapes — a proxy, not an exact bound).
	void GatherSimpleCollisionPoints(const FKAggregateGeom& AggGeom, const FVector& Scale, TArray<FVector>& OutPoints)
	{
		for (const FKBoxElem& Elem : AggGeom.BoxElems)
		{
			const FTransform ElemTransform = Elem.GetTransform();
			const FVector Half(Elem.X * 0.5, Elem.Y * 0.5, Elem.Z * 0.5);
			for (int32 Corner = 0; Corner < 8; ++Corner)
			{
				const FVector Local((Corner & 1) ? Half.X : -Half.X, (Corner & 2) ? Half.Y : -Half.Y,
					(Corner & 4) ? Half.Z : -Half.Z);
				OutPoints.Add(ElemTransform.TransformPosition(Local) * Scale);
			}
		}

		for (const FKSphereElem& Elem : AggGeom.SphereElems)
		{
			const FVector Center(Elem.Center);
			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				FVector Offset = FVector::ZeroVector;
				Offset[Axis] = Elem.Radius;
				OutPoints.Add((Center + Offset) * Scale);
				OutPoints.Add((Center - Offset) * Scale);
			}
		}

		for (const FKSphylElem& Elem : AggGeom.SphylElems)
		{
			const FTransform ElemTransform = Elem.GetTransform();
			for (double End : { -0.5, 0.5 })
			{
				const FVector HemisphereCenter(0.0, 0.0, End * Elem.Length);
				for (int32 Axis = 0; Axis < 3; ++Axis)
				{
					FVector Offset = FVector::ZeroVector;
					Offset[Axis] = Elem.Radius;
					OutPoints.Add(ElemTransform.TransformPosition(HemisphereCenter + Offset) * Scale);
					OutPoints.Add(ElemTransform.TransformPosition(HemisphereCenter - Offset) * Scale);
				}
			}
		}
	}

	/// Geometry buffers for one PMC section, filled face by face. Faces stay
	/// flat-shaded: vertices are duplicated per face with the face normal.
	struct FSectionBatch
	{
		TArray<FVector> Vertices;
		TArray<FVector> Normals;
		TArray<FVector2D> UVs;
		TArray<int32> Triangles;

		void AddFace(const FBox3DFragmentData& Fragment, const FBox3DFragmentFace& Face)
		{
			const TArray<int32>& Loop = Face.VertexIndices;
			if (Loop.Num() < 3)
			{
				return;
			}

			// Newell normal: outward for loops wound CCW seen from outside.
			FVector Normal = FVector::ZeroVector;
			for (int32 Index = 0; Index < Loop.Num(); ++Index)
			{
				const FVector& A = Fragment.Vertices[Loop[Index]];
				const FVector& B = Fragment.Vertices[Loop[(Index + 1) % Loop.Num()]];
				Normal += FVector::CrossProduct(A, B);
			}
			Normal = Normal.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);

			// Planar UVs along the dominant normal axis, tiled per meter.
			int32 AxisU = 0;
			int32 AxisV = 1;
			const FVector AbsNormal = Normal.GetAbs();
			if (AbsNormal.X >= AbsNormal.Y && AbsNormal.X >= AbsNormal.Z)
			{
				AxisU = 1;
				AxisV = 2;
			}
			else if (AbsNormal.Y >= AbsNormal.Z)
			{
				AxisU = 0;
				AxisV = 2;
			}

			const int32 Base = Vertices.Num();
			for (const int32 VertexIndex : Loop)
			{
				const FVector& Position = Fragment.Vertices[VertexIndex];
				Vertices.Add(Position);
				Normals.Add(Normal);
				UVs.Emplace(Position[AxisU] / 100.0, Position[AxisV] / 100.0);
			}

			// Fragment faces wind CCW from outside (the b3 convention); UE renders
			// the reverse (see the cooking index flip), so the fan is emitted flipped.
			for (int32 Index = 1; Index + 1 < Loop.Num(); ++Index)
			{
				Triangles.Append({ Base, Base + Index + 1, Base + Index });
			}
		}
	};
}

ABox3DFracturedActor::ABox3DFracturedActor()
{
	PrimaryActorTick.bCanEverTick = true;

	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("FracturedMesh"));
	Mesh->SetMobility(EComponentMobility::Movable);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetRootComponent(Mesh);
}

void ABox3DFracturedActor::InitializeFragments(TArray<FBox3DFragmentData>&& InFragments,
	UMaterialInterface* SourceMaterial)
{
	DestroyFragmentPhysics();
	SectionGeometry.Reset();

	Fragments = MoveTemp(InFragments);
	Box3D::Destruction::ClassifyFragmentTiers(Fragments, TierThresholds, FragmentTiers);
	Box3D::Destruction::BuildDebrisBurst(Fragments, FragmentTiers, DebrisImpactPoint, DebrisSpeed, DebrisBurst);
	FragmentSections.Init(FIntPoint(INDEX_NONE, INDEX_NONE), Fragments.Num());
	Mesh->ClearAllMeshSections();

	UMaterialInterface* InteriorMaterial = CoreMaterial != nullptr ? CoreMaterial.Get() : SourceMaterial;
	int32 SectionIndex = 0;

	for (int32 FragmentIndex = 0; FragmentIndex < Fragments.Num(); ++FragmentIndex)
	{
		if (FragmentTiers[FragmentIndex] != EBox3DFragmentTier::Body)
		{
			continue;
		}
		const FBox3DFragmentData& Fragment = Fragments[FragmentIndex];
		FSectionBatch Exterior;
		FSectionBatch Interior;
		for (const FBox3DFragmentFace& Face : Fragment.Faces)
		{
			(Face.NeighborIndex == INDEX_NONE ? Exterior : Interior).AddFace(Fragment, Face);
		}

		const auto CreateSection = [&](const FSectionBatch& Batch, UMaterialInterface* Material) -> int32
		{
			if (Batch.Triangles.IsEmpty())
			{
				return INDEX_NONE;
			}
			Mesh->CreateMeshSection_LinearColor(SectionIndex, Batch.Vertices, Batch.Triangles, Batch.Normals,
				Batch.UVs, TArray<FLinearColor>(), TArray<FProcMeshTangent>(), /*bCreateCollision*/ false);
			Mesh->SetMaterial(SectionIndex, Material);

			FSectionGeometry& Geometry = SectionGeometry.AddDefaulted_GetRef();
			Geometry.SectionIndex = SectionIndex;
			Geometry.FragmentIndex = FragmentIndex;
			Geometry.LocalVertices.Reserve(Batch.Vertices.Num());
			for (const FVector& Vertex : Batch.Vertices)
			{
				Geometry.LocalVertices.Add(Vertex - Fragment.Centroid);
			}
			Geometry.LocalNormals = Batch.Normals;
			return SectionIndex++;
		};

		FragmentSections[FragmentIndex].X = CreateSection(Exterior, SourceMaterial);
		FragmentSections[FragmentIndex].Y = CreateSection(Interior, InteriorMaterial);
	}

	BuildFragmentPhysics();
}

void ABox3DFracturedActor::BuildFragmentPhysics()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return;
	}
	const b3WorldId WorldId = Subsystem->GetBox3DWorldId();
	const FTransform ActorTransform = GetActorTransform();

	FragmentBodies.Init(b3BodyId{}, Fragments.Num());
	TArray<b3Vec3> Points;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		if (GetFragmentTier(Index) != EBox3DFragmentTier::Body)
		{
			continue;
		}
		const FBox3DFragmentData& Fragment = Fragments[Index];
		// Hull points are centroid-relative so the body origin is the center of
		// mass and the spawn frame matches the actor frame.
		Points.Reset();
		Points.Reserve(Fragment.Vertices.Num());
		for (const FVector& Vertex : Fragment.Vertices)
		{
			Points.Add(Box3D::ToB3(Vertex - Fragment.Centroid));
		}
		// b3CreateHull convexifies merged (possibly non-convex) fragment unions;
		// 64 matches the cooking hull budget.
		b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), 64);
		if (Hull == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("%s: fragment %d hull cook failed (%d verts); fragment stays visual-only"),
				*GetNameSafe(this), Index, Fragment.Vertices.Num());
			continue;
		}

		b3BodyDef BodyDef = b3DefaultBodyDef();
		BodyDef.type = b3_dynamicBody;
		BodyDef.position = Box3D::ToB3Pos(ActorTransform.TransformPosition(Fragment.Centroid));
		BodyDef.rotation = Box3D::ToB3(ActorTransform.GetRotation());
		BodyDef.isAwake = !bStartAsleep;
		BodyDef.name = "Box3DFragment";
		// userData stays null: Box3D::ResolveComponent casts it to UObject on
		// every query hit, so raw bodies must never carry anything else.
		const b3BodyId Body = b3CreateBody(WorldId, &BodyDef);

		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		ShapeDef.density = FMath::Max(FragmentDensity, 1.0f);
		ShapeDef.filter.categoryBits = 1ull << static_cast<int32>(EBox3DChannel::Debris);
		ShapeDef.filter.maskBits = UINT64_MAX;
		// Hull shapes clone the data, so the temp cook is freed immediately.
		b3CreateHullShape(Body, &ShapeDef, Hull);
		b3DestroyHull(Hull);
		FragmentBodies[Index] = Body;
	}

	// One weld per adjacent pair, walked in fixed order; the lower index owns
	// the pair (Neighbors is symmetric with canonical shared areas). Joint
	// frames anchor at the centroid midpoint, matching ABox3DBreakableActor.
	for (int32 IndexA = 0; IndexA < Fragments.Num(); ++IndexA)
	{
		for (const FBox3DFragmentNeighbor& Neighbor : Fragments[IndexA].Neighbors)
		{
			const int32 IndexB = Neighbor.FragmentIndex;
			if (IndexB <= IndexA)
			{
				continue;
			}
			const b3BodyId BodyA = FragmentBodies[IndexA];
			const b3BodyId BodyB = FragmentBodies.IsValidIndex(IndexB) ? FragmentBodies[IndexB] : b3BodyId{};
			if (!b3Body_IsValid(BodyA) || !b3Body_IsValid(BodyB))
			{
				continue;
			}

			const FTransform WorldA(ActorTransform.GetRotation(),
				ActorTransform.TransformPosition(Fragments[IndexA].Centroid));
			const FTransform WorldB(ActorTransform.GetRotation(),
				ActorTransform.TransformPosition(Fragments[IndexB].Centroid));
			const FVector Midpoint = (WorldA.GetLocation() + WorldB.GetLocation()) * 0.5;

			b3WeldJointDef Def = b3DefaultWeldJointDef();
			Def.base.bodyIdA = BodyA;
			Def.base.bodyIdB = BodyB;
			Def.base.localFrameA = b3Transform{
				Box3D::ToB3(WorldA.InverseTransformPosition(Midpoint)),
				Box3D::ToB3(WorldA.GetRotation().Inverse()) };
			Def.base.localFrameB = b3Transform{
				Box3D::ToB3(WorldB.InverseTransformPosition(Midpoint)),
				Box3D::ToB3(WorldB.GetRotation().Inverse()) };

			const float BreakForce = MaterialToughness > 0.0f
				? float(Neighbor.SharedFaceArea) * MaterialToughness
				: 0.0f;
			Welds.Add({ b3CreateWeldJoint(WorldId, &Def), IndexA, IndexB, BreakForce });
		}
	}
	UE_LOG(LogBox3D, Log, TEXT("%s: %d fragment bodies, %d welds, %d debris, %d dust"), *GetNameSafe(this),
		CountFragmentsInTier(EBox3DFragmentTier::Body), Welds.Num(),
		CountFragmentsInTier(EBox3DFragmentTier::Debris), CountFragmentsInTier(EBox3DFragmentTier::Dust));
}

int32 ABox3DFracturedActor::CountFragmentsInTier(EBox3DFragmentTier Tier) const
{
	int32 Count = 0;
	for (const EBox3DFragmentTier FragmentTier : FragmentTiers)
	{
		Count += FragmentTier == Tier ? 1 : 0;
	}
	return Count;
}

void ABox3DFracturedActor::DestroyFragmentPhysics()
{
	for (const FWeld& Weld : Welds)
	{
		if (b3Joint_IsValid(Weld.Joint))
		{
			b3DestroyJoint(Weld.Joint, /*wakeAttached*/ false);
		}
	}
	Welds.Empty();

	for (const b3BodyId Body : FragmentBodies)
	{
		if (b3Body_IsValid(Body))
		{
			b3DestroyBody(Body);
		}
	}
	FragmentBodies.Empty();
}

void ABox3DFracturedActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyFragmentPhysics();
	Super::EndPlay(EndPlayReason);
}

void ABox3DFracturedActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	CheckWelds();
	SyncFragments();
}

void ABox3DFracturedActor::CheckWelds()
{
	bool bAnyBroke = false;
	for (int32 Index = Welds.Num() - 1; Index >= 0; --Index)
	{
		const FWeld& Weld = Welds[Index];
		if (!b3Joint_IsValid(Weld.Joint))
		{
			Welds.RemoveAtSwap(Index);
			continue;
		}
		if (Weld.BreakForce > 0.0f
			&& Box3D::ToUEDir(b3Joint_GetConstraintForce(Weld.Joint)).Size() > Weld.BreakForce)
		{
			b3DestroyJoint(Weld.Joint, /*wakeAttached*/ true);
			Welds.RemoveAtSwap(Index);
			bAnyBroke = true;
		}
	}
	if (bAnyBroke)
	{
		OnWeldBroken.Broadcast();
	}
}

void ABox3DFracturedActor::GetLiveWeldPairs(TArray<FIntPoint>& OutPairs) const
{
	OutPairs.Reset(Welds.Num());
	for (const FWeld& Weld : Welds)
	{
		OutPairs.Emplace(Weld.FragmentA, Weld.FragmentB);
	}
}

void ABox3DFracturedActor::SyncFragments()
{
	if (SectionGeometry.IsEmpty() || Mesh == nullptr)
	{
		return;
	}

	const FTransform WorldToActor = GetActorTransform().Inverse();
	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	for (const FSectionGeometry& Section : SectionGeometry)
	{
		const b3BodyId Body = FragmentBodies.IsValidIndex(Section.FragmentIndex)
			? FragmentBodies[Section.FragmentIndex]
			: b3BodyId{};
		// Asleep bodies have not moved since their last synced pose (and never
		// need a first sync: the sections start at the spawn pose).
		if (!b3Body_IsValid(Body) || !b3Body_IsAwake(Body))
		{
			continue;
		}

		const b3WorldTransform Transform = b3Body_GetTransform(Body);
		const FTransform Delta =
			FTransform(Box3D::ToUE(Transform.q), Box3D::ToUEPos(Transform.p)) * WorldToActor;

		Vertices.Reset(Section.LocalVertices.Num());
		Normals.Reset(Section.LocalNormals.Num());
		for (int32 Index = 0; Index < Section.LocalVertices.Num(); ++Index)
		{
			Vertices.Add(Delta.TransformPosition(Section.LocalVertices[Index]));
			Normals.Add(Delta.TransformVectorNoScale(Section.LocalNormals[Index]));
		}
		Mesh->UpdateMeshSection_LinearColor(Section.SectionIndex, Vertices, Normals,
			TArray<FVector2D>(), TArray<FLinearColor>(), TArray<FProcMeshTangent>());
	}
}

namespace Box3D
{
	EBox3DFractureProxySource ResolveFractureProxy(const UStaticMeshComponent& Component,
		Fracture::FFractureProxy& OutProxy)
	{
		UStaticMesh* StaticMesh = Component.GetStaticMesh();
		if (StaticMesh == nullptr)
		{
			return EBox3DFractureProxySource::None;
		}

		const FVector Scale = Component.GetComponentScale();
		TArray<FVector> Points;

		if (const UBodySetup* BodySetup = StaticMesh->GetBodySetup())
		{
			if (!BodySetup->AggGeom.ConvexElems.IsEmpty())
			{
				GatherConvexElemPoints(BodySetup->AggGeom, Scale, Points);
				if (BuildConvexProxy(Points, OutProxy))
				{
					return EBox3DFractureProxySource::AuthoredConvex;
				}
				Points.Reset();
			}

			GatherSimpleCollisionPoints(BodySetup->AggGeom, Scale, Points);
			if (BuildConvexProxy(Points, OutProxy))
			{
				return EBox3DFractureProxySource::SimpleCollision;
			}
			Points.Reset();
		}

		if (GetRenderVertexPositions(StaticMesh, Points))
		{
			for (FVector& Point : Points)
			{
				Point *= Scale;
			}
			if (BuildConvexProxy(Points, OutProxy))
			{
				UE_LOG(LogBox3D, Warning,
					TEXT("Box3D fracture proxy for %s: no authored collision, using render vertices "
						 "(editor-only fallback; packaged builds need simple collision)"),
					*GetNameSafe(StaticMesh));
				return EBox3DFractureProxySource::RenderVertices;
			}
		}

		return EBox3DFractureProxySource::None;
	}

	ABox3DFracturedActor* FractureMesh(UStaticMeshComponent* Component, const FBox3DFractureMeshParams& Params)
	{
		if (Component == nullptr || Component->GetWorld() == nullptr)
		{
			return nullptr;
		}

		Fracture::FFractureProxy Proxy;
		if (ResolveFractureProxy(*Component, Proxy) == EBox3DFractureProxySource::None)
		{
			UE_LOG(LogBox3D, Warning, TEXT("Box3D::FractureMesh: no fracture proxy for %s"),
				*GetNameSafe(Component->GetStaticMesh()));
			return nullptr;
		}

		// Fragment space = component space with scale baked into the proxy, so the
		// world seam is rotation + translation only.
		const FTransform ComponentToWorld(Component->GetComponentQuat(), Component->GetComponentLocation());

		Fracture::FFractureParams FractureParams = Params.Fracture;
		FractureParams.ImpactPoint = ComponentToWorld.InverseTransformPosition(Params.Fracture.ImpactPoint);

		TArray<Fracture::FBox3DFragmentData> Fragments;
		if (!Fracture::Fracture(Proxy, FractureParams, Fragments))
		{
			UE_LOG(LogBox3D, Warning, TEXT("Box3D::FractureMesh: fracture produced no fragments for %s"),
				*GetNameSafe(Component->GetStaticMesh()));
			return nullptr;
		}

		UWorld* World = Component->GetWorld();
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		ABox3DFracturedActor* Actor = World->SpawnActor<ABox3DFracturedActor>(
			ABox3DFracturedActor::StaticClass(), ComponentToWorld, SpawnParams);
		if (Actor == nullptr)
		{
			return nullptr;
		}

		Actor->CoreMaterial = Params.CoreMaterial;
		Actor->MaterialToughness = Params.MaterialToughness;
		Actor->FragmentDensity = Params.FragmentDensity;
		Actor->bStartAsleep = Params.bStartAsleep;
		Actor->TierThresholds = Params.Tiers;
		Actor->DebrisSpeed = Params.DebrisSpeed;
		// Fragment/actor space impact, already converted for the fracture core.
		Actor->DebrisImpactPoint = FractureParams.ImpactPoint;
		Actor->InitializeFragments(MoveTemp(Fragments), Component->GetMaterial(0));

		// Swap the source out: the fractured actor owns the visuals from here, and
		// nothing should collide with the intact mesh anymore.
		Component->SetVisibility(false);
		Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (UBox3DWorldSubsystem* Subsystem = World->GetSubsystem<UBox3DWorldSubsystem>())
		{
			if (FBox3DStaticSceneMirror* Mirror = Subsystem->GetStaticMirror())
			{
				Mirror->RemoveComponent(Component);
			}
		}

		return Actor;
	}
}
