#include "Box3DFracturedActor.h"

#include "Box3DConversion.h"
#include "Box3DDeform.h"
#include "Box3DCooking.h"
#include "Box3DTypes.h"
#include "Box3DRuntime.h"
#include "Box3DSettings.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
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
	SpawnDebrisBurst();
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
	InitializeStructure();

	// Join the fragment pool last: registration may evict older fractured actors
	// to make room, and this actor's own footprint must be final by then.
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->RegisterFracturedActor(this);
	}
}

void ABox3DFracturedActor::SpawnDebrisBurst() const
{
	if (DebrisSystem == nullptr || DebrisBurst.Positions.IsEmpty())
	{
		return;
	}

	// The burst arrays are actor-space data; Niagara gets world-space copies so
	// the system needs no transform plumbing.
	const FTransform& ActorToWorld = GetActorTransform();
	TArray<FVector> Positions;
	TArray<FVector> Velocities;
	Positions.Reserve(DebrisBurst.Positions.Num());
	Velocities.Reserve(DebrisBurst.Velocities.Num());
	for (const FVector& Position : DebrisBurst.Positions)
	{
		Positions.Add(ActorToWorld.TransformPosition(Position));
	}
	for (const FVector& Velocity : DebrisBurst.Velocities)
	{
		Velocities.Add(ActorToWorld.TransformVector(Velocity));
	}

	UNiagaraComponent* Burst = UNiagaraFunctionLibrary::SpawnSystemAtLocation(GetWorld(), DebrisSystem,
		GetActorLocation(), GetActorRotation(), FVector::OneVector, /*bAutoDestroy*/ true);
	if (Burst == nullptr)
	{
		return;
	}
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(Burst, TEXT("DebrisPositions"), Positions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(Burst, TEXT("DebrisVelocities"), Velocities);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayFloat(Burst, TEXT("DebrisSizes"), DebrisBurst.Sizes);
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
		// Structural assemblies hold still until connectivity says otherwise —
		// promotion flips unsupported islands to dynamic later.
		BodyDef.type = bStructural ? b3_staticBody : b3_dynamicBody;
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

void ABox3DFracturedActor::InitializeStructure()
{
	bStructureActive = false;
	StructureGraph.Reset();
	StressSolver.Reset();
	PendingStressImpulses.Reset();
	bStressTopologyDirty = false;
	AppliedStressCoarsenThreshold = INDEX_NONE;
	bStressSolveSeeded = false;
	bStressImpulseSolve = false;
	PendingPromotions.Reset();
	if (!bStructural)
	{
		return;
	}

	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return;
	}

	StructureGraph.Build(Fragments);
	const int32 AnchorCount =
		Box3D::Structure::DetectAnchors(StructureGraph, Subsystem->GetBox3DWorldId(), FragmentBodies);
	if (AnchorCount == 0)
	{
		// Nothing holds this assembly to the world — it is not a structure, just
		// rubble. Revert to the plain dynamic behavior instead of promoting
		// everything through the budget one tick late.
		UE_LOG(LogBox3D, Log, TEXT("%s: structural assembly found no anchors; falling back to dynamic rubble"),
			*GetNameSafe(this));
		StructureGraph.Reset();
		for (const b3BodyId Body : FragmentBodies)
		{
			if (b3Body_IsValid(Body))
			{
				b3Body_SetType(Body, b3_dynamicBody);
				if (!bStartAsleep)
				{
					b3Body_SetAwake(Body, true);
				}
			}
		}
		return;
	}
	bStructureActive = true;

	// Bodiless fragments (non-Body tiers, failed hull cooks) exist as graph
	// nodes but cannot carry load — mark them destroyed up front. Any islands
	// this strands are queued like any other unsupported chunks.
	Box3D::Structure::FBox3DStructureIslands Islands;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		if (!FragmentBodies.IsValidIndex(Index) || !b3Body_IsValid(FragmentBodies[Index]))
		{
			StructureGraph.NotifyChunkDestroyed(Index, Islands);
		}
	}
	EnqueueIslands(Islands);
	InitializeStressSolver();

	UE_LOG(LogBox3D, Log, TEXT("%s: structural assembly live — %d chunks, %d bonds, %d anchors"),
		*GetNameSafe(this), StructureGraph.GetNodeCount(), StructureGraph.GetBondCount(), AnchorCount);
}

void ABox3DFracturedActor::EnqueueIslands(const Box3D::Structure::FBox3DStructureIslands& Islands)
{
	for (const TArray<int32>& Island : Islands.Islands)
	{
		PendingPromotions.Append(Island);
	}
}

bool ABox3DFracturedActor::InitializeStressSolver()
{
	if (!bStructureActive)
	{
		StressSolver.Reset();
		return false;
	}
	TArray<double> Masses;
	Masses.Reserve(StructureGraph.GetNodeCount());
	for (int32 Index = 0; Index < StructureGraph.GetNodeCount(); ++Index)
	{
		const b3BodyId Body = GetFragmentBody(Index);
		const double BodyMass = b3Body_IsValid(Body) ? b3Body_GetMass(Body) : 0.0;
		if (BodyMass > UE_SMALL_NUMBER)
		{
			Masses.Add(BodyMass);
		}
		else
		{
			// Static bodies may expose zero solver mass. Recover authored physical
			// mass from kg/m^3 * cm^3 / 1e6 at the UE seam.
			Masses.Add(StructureGraph.GetNodeVolume(Index)
				* FMath::Max(static_cast<double>(FragmentDensity), 0.0) * 1.0e-6);
		}
	}
	const int32 CoarsenThreshold = GetDefault<UBox3DSettings>()->StressCoarsenNodeThreshold;
	bStressTopologyDirty = false;
	AppliedStressCoarsenThreshold = CoarsenThreshold;
	bStressSolveSeeded = false;
	return StressSolver.Initialize(StructureGraph, Masses, CoarsenThreshold);
}

bool ABox3DFracturedActor::BreakStructuralBond(int32 BondIndex)
{
	if (!bStructureActive || BondIndex < 0 || BondIndex >= StructureGraph.GetBondCount()
		|| StructureGraph.GetBond(BondIndex).bBroken)
	{
		return false;
	}
	const Box3D::Structure::FBox3DStructureBond& Bond = StructureGraph.GetBond(BondIndex);
	for (int32 WeldIndex = Welds.Num() - 1; WeldIndex >= 0; --WeldIndex)
	{
		const FWeld& Weld = Welds[WeldIndex];
		if (Weld.FragmentA == Bond.NodeA && Weld.FragmentB == Bond.NodeB)
		{
			if (b3Joint_IsValid(Weld.Joint))
			{
				b3DestroyJoint(Weld.Joint, /*wakeAttached*/ true);
			}
			Welds.RemoveAtSwap(WeldIndex);
			break;
		}
	}
	Box3D::Structure::FBox3DStructureIslands Islands;
	StructureGraph.NotifyBondBroken(BondIndex, Islands);
	EnqueueIslands(Islands);
	bStressTopologyDirty = true;
	return true;
}

void ABox3DFracturedActor::QueueStressImpulse(int32 FragmentIndex, const FVector& WorldImpulse,
	const FVector& WorldApplicationPoint)
{
	if (!bStructureActive || !Fragments.IsValidIndex(FragmentIndex)
		|| StructureGraph.IsChunkDestroyed(FragmentIndex))
	{
		return;
	}
	const FTransform WorldToActor = GetActorTransform().Inverse();
	Box3D::Structure::FBox3DStressImpulse& Event = PendingStressImpulses.AddDefaulted_GetRef();
	Event.NodeIndex = FragmentIndex;
	Event.Impulse = WorldToActor.TransformVector(WorldImpulse);
	Event.ApplicationPoint = WorldToActor.TransformPosition(WorldApplicationPoint);
}

void ABox3DFracturedActor::QueueExplosionStress(const FVector& WorldCenter, float Radius, float Falloff,
	float ImpulsePerArea)
{
	if (!bStructureActive || FMath::IsNearlyZero(ImpulsePerArea))
	{
		return;
	}
	const FTransform ActorToWorld = GetActorTransform();
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		if (StructureGraph.IsChunkDestroyed(Index) || !b3Body_IsValid(GetFragmentBody(Index)))
		{
			continue;
		}
		const FVector Center = ActorToWorld.TransformPosition(Fragments[Index].Centroid);
		const double Distance = FVector::Dist(WorldCenter, Center);
		double Scale = 0.0;
		if (Distance <= Radius)
		{
			Scale = 1.0;
		}
		else if (Falloff > 0.0f && Distance < double(Radius) + Falloff)
		{
			Scale = 1.0 - (Distance - Radius) / Falloff;
		}
		if (Scale <= 0.0)
		{
			continue;
		}
		const FVector Direction = (Center - WorldCenter).GetSafeNormal();
		// Projected blast area is approximated deterministically from volume.
		const double ProjectedArea = FMath::Pow(FMath::Max(Fragments[Index].Volume, 0.0), 2.0 / 3.0);
		QueueStressImpulse(Index, Direction * (ImpulsePerArea * ProjectedArea * Scale), Center);
	}
}

void ABox3DFracturedActor::ProcessStructuralStress(float DeltaSeconds)
{
	if (!bStructureActive || DeltaSeconds <= 0.0f)
	{
		return;
	}
	if ((bStressTopologyDirty
			|| AppliedStressCoarsenThreshold != GetDefault<UBox3DSettings>()->StressCoarsenNodeThreshold)
		&& !InitializeStressSolver())
	{
		return;
	}
	if (!StressSolver.IsInitialized())
	{
		return;
	}

	const UBox3DSettings* Settings = GetDefault<UBox3DSettings>();
	if (!bStressSolveSeeded || !PendingStressImpulses.IsEmpty())
	{
		bStressImpulseSolve = !PendingStressImpulses.IsEmpty();
		const FVector LocalGravity = GetActorTransform().InverseTransformVectorNoScale(Settings->Gravity);
		StressSolver.BeginSolve(LocalGravity, PendingStressImpulses, DeltaSeconds);
		PendingStressImpulses.Reset();
		bStressSolveSeeded = true;
	}
	const Box3D::Structure::FBox3DStressSolveStats Stats =
		StressSolver.Relax(FMath::Max(Settings->StressRelaxationIterationsPerTick, 1));
	// A partially propagated load is not yet a sustained equilibrium. Impulse
	// solves are one-shot once converged; gravity equilibrium remains live.
	if (Stats.ResidualForce > 1.0e-4 || Stats.ResidualMoment > 1.0e-4)
	{
		return;
	}

	Box3D::Structure::FBox3DStressThresholds Thresholds;
	Thresholds.TensionPa = TensionStrengthPa;
	Thresholds.CompressionPa = CompressionStrengthPa;
	Thresholds.ShearPa = ShearStrengthPa;
	int32 BrokenBond = INDEX_NONE;
	for (int32 BondIndex = 0; BondIndex < StructureGraph.GetBondCount(); ++BondIndex)
	{
		const Box3D::Structure::FBox3DStructureBond& Bond = StructureGraph.GetBond(BondIndex);
		if (Bond.bBroken)
		{
			continue;
		}
		const Box3D::Structure::FBox3DBondOverload Overload =
			StressSolver.GetBondOverload(BondIndex, Thresholds);
		if (Overload.Ratio <= 1.0)
		{
			continue;
		}
		OnStructureStressed.Broadcast(BondIndex, static_cast<float>(Overload.Ratio), Bond.Health);
		const float Damage = static_cast<float>((Overload.Ratio - 1.0)
			* FMath::Max(static_cast<double>(SustainedOverloadHealthPerSecond), 0.0)
			* DeltaSeconds);
		if (StructureGraph.ApplyBondDamage(BondIndex, Damage) <= 0.0f)
		{
			BrokenBond = BondIndex;
			break; // Deterministic chain collapse: one topology event per tick.
		}
	}
	const bool bAnyBroke = BrokenBond != INDEX_NONE && BreakStructuralBond(BrokenBond);
	if (bAnyBroke)
	{
		OnWeldBroken.Broadcast();
	}
	if (bStressImpulseSolve)
	{
		bStressSolveSeeded = false;
		bStressImpulseSolve = false;
	}
}

void ABox3DFracturedActor::ProcessPromotions()
{
	if (PendingPromotions.IsEmpty())
	{
		return;
	}
	const int32 BudgetSetting = GetDefault<UBox3DSettings>()->MaxPromotionsPerTick;
	int32 Budget = BudgetSetting > 0 ? BudgetSetting : MAX_int32;

	// Phase 1: flip the whole batch static->dynamic before waking anything, so
	// no weld connects a static body to an awake dynamic one mid-pass.
	TArray<b3BodyId, TInlineAllocator<64>> Promoted;
	int32 Consumed = 0;
	while (Consumed < PendingPromotions.Num() && Budget > 0)
	{
		const b3BodyId Body = GetFragmentBody(PendingPromotions[Consumed++]);
		// Stale entries (destroyed or already-promoted chunks) cost no budget.
		if (!b3Body_IsValid(Body) || b3Body_GetType(Body) != b3_staticBody)
		{
			continue;
		}
		b3Body_SetType(Body, b3_dynamicBody);
		Promoted.Add(Body);
		--Budget;
	}
	PendingPromotions.RemoveAt(0, Consumed);

	// Phase 2: wake the batch and let physics take over.
	for (const b3BodyId Body : Promoted)
	{
		b3Body_SetAwake(Body, true);
	}
}

void ABox3DFracturedActor::DestroyFragment(int32 FragmentIndex)
{
	if (!Fragments.IsValidIndex(FragmentIndex)
		|| (bStructureActive && StructureGraph.IsChunkDestroyed(FragmentIndex)))
	{
		return;
	}

	for (int32 Index = Welds.Num() - 1; Index >= 0; --Index)
	{
		const FWeld& Weld = Welds[Index];
		if (Weld.FragmentA == FragmentIndex || Weld.FragmentB == FragmentIndex)
		{
			if (b3Joint_IsValid(Weld.Joint))
			{
				b3DestroyJoint(Weld.Joint, /*wakeAttached*/ true);
			}
			Welds.RemoveAtSwap(Index);
		}
	}

	if (FragmentBodies.IsValidIndex(FragmentIndex) && b3Body_IsValid(FragmentBodies[FragmentIndex]))
	{
		b3DestroyBody(FragmentBodies[FragmentIndex]);
		FragmentBodies[FragmentIndex] = b3BodyId{};
	}

	if (FragmentSections.IsValidIndex(FragmentIndex))
	{
		for (const int32 Section : { FragmentSections[FragmentIndex].X, FragmentSections[FragmentIndex].Y })
		{
			if (Section != INDEX_NONE)
			{
				Mesh->ClearMeshSection(Section);
			}
		}
		FragmentSections[FragmentIndex] = FIntPoint(INDEX_NONE, INDEX_NONE);
	}
	SectionGeometry.RemoveAll([FragmentIndex](const FSectionGeometry& Section)
	{
		return Section.FragmentIndex == FragmentIndex;
	});

	if (bStructureActive)
	{
		Box3D::Structure::FBox3DStructureIslands Islands;
		StructureGraph.NotifyChunkDestroyed(FragmentIndex, Islands);
		EnqueueIslands(Islands);
		bStressTopologyDirty = true;
	}
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

	bStructureActive = false;
	StructureGraph.Reset();
	StressSolver.Reset();
	PendingStressImpulses.Empty();
	bStressTopologyDirty = false;
	AppliedStressCoarsenThreshold = INDEX_NONE;
	bStressSolveSeeded = false;
	bStressImpulseSolve = false;
	PendingPromotions.Empty();
}

void ABox3DFracturedActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->UnregisterFracturedActor(this);
	}
	DestroyFragmentPhysics();
	Super::EndPlay(EndPlayReason);
}

void ABox3DFracturedActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	CheckWelds();
	ProcessStructuralStress(DeltaSeconds);
	ProcessPromotions();
	SyncFragments();
}

void ABox3DFracturedActor::CheckWelds()
{
	bool bAnyBroke = false;
	TArray<FIntPoint, TInlineAllocator<8>> BrokenPairs;
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
			if (bStructureActive)
			{
				BrokenPairs.Emplace(Weld.FragmentA, Weld.FragmentB);
			}
			Welds.RemoveAtSwap(Index);
			bAnyBroke = true;
		}
	}

	// Weld breaks are the bond-break events of the structure graph: each snap
	// floods the affected neighborhood and queues stranded islands. The joint is
	// already gone, so route topology through the same helper used by stress.
	for (const FIntPoint& Pair : BrokenPairs)
	{
		const int32 BondIndex = StructureGraph.FindBond(Pair.X, Pair.Y);
		if (BondIndex != INDEX_NONE && !StructureGraph.GetBond(BondIndex).bBroken)
		{
			Box3D::Structure::FBox3DStructureIslands Islands;
			StructureGraph.NotifyBondBroken(BondIndex, Islands);
			EnqueueIslands(Islands);
			bStressTopologyDirty = true;
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

int32 ABox3DFracturedActor::ApplyVertexDent(FVector WorldImpactPoint,
	FVector WorldImpactNormal, float RadiusCm, float MaxDepthCm, float FalloffExponent)
{
	if (SectionGeometry.IsEmpty() || Mesh == nullptr)
	{
		return 0;
	}

	const FTransform WorldToActor = GetActorTransform().Inverse();
	const FVector ActorImpactPoint = WorldToActor.TransformPosition(WorldImpactPoint);
	const FVector ActorImpactNormal = WorldToActor.TransformVectorNoScale(WorldImpactNormal).GetSafeNormal();
	if (ActorImpactNormal.IsNearlyZero())
	{
		return 0;
	}

	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	int32 DisplacedCount = 0;
	for (FSectionGeometry& Section : SectionGeometry)
	{
		FTransform Delta(FQuat::Identity,
			Fragments.IsValidIndex(Section.FragmentIndex)
				? Fragments[Section.FragmentIndex].Centroid
				: FVector::ZeroVector);
		const b3BodyId Body = FragmentBodies.IsValidIndex(Section.FragmentIndex)
			? FragmentBodies[Section.FragmentIndex]
			: b3BodyId{};
		if (b3Body_IsValid(Body))
		{
			const b3WorldTransform Transform = b3Body_GetTransform(Body);
			Delta = FTransform(Box3D::ToUE(Transform.q), Box3D::ToUEPos(Transform.p)) * WorldToActor;
		}

		Box3D::Deform::FBox3DVertexDent Dent;
		Dent.ImpactPoint = Delta.InverseTransformPosition(ActorImpactPoint);
		Dent.ImpactNormal = Delta.InverseTransformVectorNoScale(ActorImpactNormal);
		Dent.RadiusCm = RadiusCm;
		Dent.MaxDepthCm = MaxDepthCm;
		Dent.FalloffExponent = FalloffExponent;
		const int32 SectionDisplaced = Box3D::Deform::ApplyVertexDent(Section.LocalVertices, Dent);
		if (SectionDisplaced == 0)
		{
			continue;
		}
		DisplacedCount += SectionDisplaced;

		Vertices.Reset(Section.LocalVertices.Num());
		Normals.Reset(Section.LocalNormals.Num());
		for (int32 VertexIndex = 0; VertexIndex < Section.LocalVertices.Num(); ++VertexIndex)
		{
			Vertices.Add(Delta.TransformPosition(Section.LocalVertices[VertexIndex]));
			Normals.Add(Delta.TransformVectorNoScale(Section.LocalNormals[VertexIndex]));
		}
		Mesh->UpdateMeshSection_LinearColor(Section.SectionIndex, Vertices, Normals,
			TArray<FVector2D>(), TArray<FLinearColor>(), TArray<FProcMeshTangent>());
	}
	return DisplacedCount;
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
		Actor->bStructural = Params.bStructural;
		Actor->TensionStrengthPa = Params.TensionStrengthPa;
		Actor->CompressionStrengthPa = Params.CompressionStrengthPa;
		Actor->ShearStrengthPa = Params.ShearStrengthPa;
		Actor->SustainedOverloadHealthPerSecond = Params.SustainedOverloadHealthPerSecond;
		Actor->TierThresholds = Params.Tiers;
		Actor->DebrisSpeed = Params.DebrisSpeed;
		Actor->DebrisSystem = Params.DebrisSystem;
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
