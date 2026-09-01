#include "Box3DFracturedActor.h"

#include "Box3DConversion.h"
#include "Box3DDeform.h"
#include "Box3DCooking.h"
#include "Box3DTypes.h"
#include "Box3DRuntime.h"
#include "Box3DSettings.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DWorldSubsystem.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "MeshDescription.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraSystem.h"
#include "PhysicsEngine/BodySetup.h"
#include "StaticMeshAttributes.h"
#include "TimerManager.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	using namespace Box3D::Fracture;

	/// Material slot names of every fragment static mesh; sections resolve their
	/// material index by these.
	const FName ExteriorSlotName(TEXT("Exterior"));
	const FName InteriorSlotName(TEXT("Interior"));

	/// Fragment primitives never collide (the b3 hulls do) and never move by
	/// physics of their own.
	void ConfigureFragmentComponent(UStaticMeshComponent& Component)
	{
		Component.SetMobility(EComponentMobility::Movable);
		Component.SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Component.SetGenerateOverlapEvents(false);
		Component.SetCanEverAffectNavigation(false);
	}

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
}

ABox3DFracturedActor::ABox3DFracturedActor()
{
	PrimaryActorTick.bCanEverTick = true;

	FractureRoot = CreateDefaultSubobject<USceneComponent>(TEXT("FractureRoot"));
	FractureRoot->SetMobility(EComponentMobility::Movable);
	SetRootComponent(FractureRoot);

	AttachedMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("AttachedFragments"));
	AttachedMesh->SetupAttachment(FractureRoot);
	ConfigureFragmentComponent(*AttachedMesh);
}

void ABox3DFracturedActor::BuildRenderGeometry(const FBox3DFragmentData& Fragment, FFragmentRenderGeometry& Out)
{
	Out = FFragmentRenderGeometry();
	int32 CornerCount = 0;
	for (const FBox3DFragmentFace& Face : Fragment.Faces)
	{
		CornerCount += Face.VertexIndices.Num() >= 3 ? Face.VertexIndices.Num() : 0;
	}
	Out.Vertices.Reserve(CornerCount);
	Out.Normals.Reserve(CornerCount);
	Out.Tangents.Reserve(CornerCount);
	Out.BinormalSigns.Reserve(CornerCount);
	Out.UVs.Reserve(CornerCount);

	for (const FBox3DFragmentFace& Face : Fragment.Faces)
	{
		const TArray<int32>& Loop = Face.VertexIndices;
		if (Loop.Num() < 3)
		{
			continue;
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

		// Planar UVs along the dominant normal axis, tiled per meter; the tangent
		// frame follows the same two axes so normal maps read the UV gradient.
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
		FVector AxisUDir = FVector::ZeroVector;
		AxisUDir[AxisU] = 1.0;
		FVector AxisVDir = FVector::ZeroVector;
		AxisVDir[AxisV] = 1.0;
		const FVector Tangent =
			(AxisUDir - Normal * FVector::DotProduct(Normal, AxisUDir)).GetSafeNormal(UE_SMALL_NUMBER, AxisUDir);
		const float BinormalSign =
			FVector::DotProduct(FVector::CrossProduct(Normal, Tangent), AxisVDir) >= 0.0 ? 1.0f : -1.0f;

		const int32 Base = Out.Vertices.Num();
		for (const int32 VertexIndex : Loop)
		{
			const FVector& Position = Fragment.Vertices[VertexIndex];
			Out.Vertices.Add(Position - Fragment.Centroid);
			Out.Normals.Add(Normal);
			Out.Tangents.Add(Tangent);
			Out.BinormalSigns.Add(BinormalSign);
			Out.UVs.Emplace(Position[AxisU] / 100.0, Position[AxisV] / 100.0);
		}

		// Fragment faces wind CCW from outside (the b3 convention); UE renders
		// the reverse (see the cooking index flip), so the fan is emitted flipped.
		TArray<int32>& Triangles = Face.NeighborIndex == INDEX_NONE ? Out.ExteriorTriangles : Out.InteriorTriangles;
		for (int32 Index = 1; Index + 1 < Loop.Num(); ++Index)
		{
			Triangles.Append({ Base, Base + Index + 1, Base + Index });
		}
	}
}

void ABox3DFracturedActor::InitializeFragments(TArray<FBox3DFragmentData>&& InFragments,
	UMaterialInterface* SourceMaterial, bool bCreatePhysics)
{
	DestroyFragmentPhysics();
	for (int32 Index = 0; Index < FragmentComponents.Num(); ++Index)
	{
		ReleaseFragmentComponent(Index);
	}

	Fragments = MoveTemp(InFragments);
	Box3D::Destruction::ClassifyFragmentTiers(Fragments, TierThresholds, FragmentTiers);
	Box3D::Destruction::BuildDebrisBurst(Fragments, FragmentTiers, DebrisImpactPoint, DebrisSpeed, DebrisBurst);
	SpawnDebrisBurst();

	ExteriorMaterial = SourceMaterial;
	RenderGeometry.SetNum(Fragments.Num());
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		if (FragmentTiers[Index] == EBox3DFragmentTier::Body)
		{
			BuildRenderGeometry(Fragments[Index], RenderGeometry[Index]);
		}
	}
	FragmentComponents.Init(nullptr, Fragments.Num());
	FragmentRenderStates.Init(EBox3DFragmentRenderState::None, Fragments.Num());
	FragmentDestroyed.Init(false, Fragments.Num());
	ApplyRenderFlags(*AttachedMesh);

	if (bCreatePhysics)
	{
		BuildFragmentPhysics();
		InitializeStructure();
	}

	// Draw after the physics setup: body types decide which fragments share the
	// attached mesh (anchored cladding: all of them; plain rubble: none).
	bRenderDirty = true;
	FlushRenderState();

	if (bCreatePhysics)
	{
		// Join the fragment pool last: registration may evict older fractured actors
		// to make room, and this actor's own footprint must be final by then.
		if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
		{
			Subsystem->RegisterFracturedActor(this);
		}
	}
}

EBox3DFragmentRenderState ABox3DFracturedActor::DesiredRenderState(int32 FragmentIndex) const
{
	if (!Fragments.IsValidIndex(FragmentIndex) || GetFragmentTier(FragmentIndex) != EBox3DFragmentTier::Body
		|| (FragmentDestroyed.IsValidIndex(FragmentIndex) && FragmentDestroyed[FragmentIndex])
		|| !RenderGeometry.IsValidIndex(FragmentIndex) || RenderGeometry[FragmentIndex].Vertices.IsEmpty())
	{
		return EBox3DFragmentRenderState::None;
	}
	if (IsFragmentAlive(FragmentIndex) && b3Body_GetType(FragmentBodies[FragmentIndex]) == b3_dynamicBody)
	{
		return EBox3DFragmentRenderState::Loose;
	}
	return EBox3DFragmentRenderState::Attached;
}

void ABox3DFracturedActor::MarkRenderDirty()
{
	bRenderDirty = true;
	UWorld* World = GetWorld();
	if (World == nullptr || IsActorBeingDestroyed())
	{
		return;
	}
	// Tick normally flushes first; the timer covers actors that do not tick (a
	// skin stops ticking settled cells, yet still destroys their expired chips).
	FTimerManager& Timers = World->GetTimerManager();
	if (!Timers.IsTimerActive(RenderFlushTimer))
	{
		RenderFlushTimer = Timers.SetTimerForNextTick(this, &ABox3DFracturedActor::FlushRenderState);
	}
}

void ABox3DFracturedActor::FlushRenderState()
{
	if (!bRenderDirty || IsActorBeingDestroyed() || AttachedMesh == nullptr)
	{
		return;
	}
	bRenderDirty = false;

	bool bAttachedChanged = false;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		const EBox3DFragmentRenderState Desired = DesiredRenderState(Index);
		const EBox3DFragmentRenderState Current = FragmentRenderStates[Index];
		if (Desired == Current)
		{
			continue;
		}
		bAttachedChanged |= Current == EBox3DFragmentRenderState::Attached || Desired == EBox3DFragmentRenderState::Attached;
		if (Current == EBox3DFragmentRenderState::Loose)
		{
			ReleaseFragmentComponent(Index);
		}
		FragmentRenderStates[Index] = Desired;
		if (Desired == EBox3DFragmentRenderState::Loose)
		{
			MakeFragmentLoose(Index);
		}
	}
	if (bAttachedChanged)
	{
		RebuildAttachedMesh();
	}
}

void ABox3DFracturedActor::RebuildAttachedMesh()
{
	TArray<int32> Attached;
	Attached.Reserve(Fragments.Num());
	for (int32 Index = 0; Index < FragmentRenderStates.Num(); ++Index)
	{
		if (FragmentRenderStates[Index] == EBox3DFragmentRenderState::Attached)
		{
			Attached.Add(Index);
		}
	}
	// Always a fresh mesh: rebuilding one in place waits on the render thread to
	// release its buffers, swapping just recreates this component's proxy.
	AttachedMesh->SetStaticMesh(Attached.IsEmpty() ? nullptr : BuildStaticMesh(Attached, /*bCentroidRelative*/ false));
}

void ABox3DFracturedActor::MakeFragmentLoose(int32 FragmentIndex)
{
	UStaticMesh* ChipMesh = BuildStaticMesh(MakeArrayView(&FragmentIndex, 1), /*bCentroidRelative*/ true);
	if (ChipMesh == nullptr)
	{
		FragmentRenderStates[FragmentIndex] = EBox3DFragmentRenderState::None;
		return;
	}
	UStaticMeshComponent* Component = AcquireFragmentComponent();
	Component->SetStaticMesh(ChipMesh);
	FragmentComponents[FragmentIndex] = Component;
	SyncFragmentComponent(FragmentIndex);
}

UStaticMeshComponent* ABox3DFracturedActor::AcquireFragmentComponent()
{
	UStaticMeshComponent* Component = nullptr;
	while (Component == nullptr && !FreeFragmentComponents.IsEmpty())
	{
		Component = FreeFragmentComponents.Pop();
		if (!IsValid(Component))
		{
			Component = nullptr;
		}
	}
	if (Component == nullptr)
	{
		Component = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transient);
		ConfigureFragmentComponent(*Component);
		ApplyRenderFlags(*Component);
		Component->SetupAttachment(FractureRoot);
		Component->RegisterComponent();
	}
	Component->SetVisibility(true);
	return Component;
}

void ABox3DFracturedActor::ReleaseFragmentComponent(int32 FragmentIndex)
{
	if (!FragmentComponents.IsValidIndex(FragmentIndex))
	{
		return;
	}
	UStaticMeshComponent* Component = FragmentComponents[FragmentIndex];
	FragmentComponents[FragmentIndex] = nullptr;
	if (Component == nullptr)
	{
		return;
	}
	Component->SetStaticMesh(nullptr);
	Component->SetVisibility(false);
	FreeFragmentComponents.Add(Component);
}

void ABox3DFracturedActor::ApplyRenderFlags(UPrimitiveComponent& Component) const
{
	Component.SetCastShadow(bFragmentsCastShadow);
	Component.bAffectDynamicIndirectLighting = bFragmentsAffectIndirectLighting;
	Component.bAffectDistanceFieldLighting = bFragmentsAffectIndirectLighting;
	Component.SetReceivesDecals(bFragmentsReceiveDecals);
	Component.MarkRenderStateDirty();
}

void ABox3DFracturedActor::SyncFragmentComponent(int32 FragmentIndex) const
{
	UStaticMeshComponent* Component = FragmentComponents.IsValidIndex(FragmentIndex) ? FragmentComponents[FragmentIndex].Get() : nullptr;
	if (Component == nullptr)
	{
		return;
	}
	const b3BodyId Body = GetFragmentBody(FragmentIndex);
	if (b3Body_IsValid(Body))
	{
		// Chip geometry is body-local (centroid origin, spawn rotation), so the
		// body pose is the component's world transform.
		const b3WorldTransform Transform = b3Body_GetTransform(Body);
		Component->SetWorldLocationAndRotation(Box3D::ToUEPos(Transform.p), Box3D::ToUE(Transform.q));
	}
	else
	{
		Component->SetWorldLocationAndRotation(
			GetActorTransform().TransformPosition(Fragments[FragmentIndex].Centroid), GetActorQuat());
	}
}

UStaticMesh* ABox3DFracturedActor::BuildStaticMesh(TArrayView<const int32> FragmentIndices, bool bCentroidRelative)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Box3DFracturedActor_BuildStaticMesh);

	int32 CornerTotal = 0;
	int32 TriangleTotal = 0;
	for (const int32 FragmentIndex : FragmentIndices)
	{
		const FFragmentRenderGeometry& Geometry = RenderGeometry[FragmentIndex];
		CornerTotal += Geometry.Vertices.Num();
		TriangleTotal += (Geometry.ExteriorTriangles.Num() + Geometry.InteriorTriangles.Num()) / 3;
	}
	if (TriangleTotal == 0)
	{
		return nullptr;
	}

	FMeshDescription MeshDescription;
	FStaticMeshAttributes Attributes(MeshDescription);
	Attributes.Register();
	TVertexAttributesRef<FVector3f> Positions = Attributes.GetVertexPositions();
	TVertexInstanceAttributesRef<FVector3f> Normals = Attributes.GetVertexInstanceNormals();
	TVertexInstanceAttributesRef<FVector3f> Tangents = Attributes.GetVertexInstanceTangents();
	TVertexInstanceAttributesRef<float> BinormalSigns = Attributes.GetVertexInstanceBinormalSigns();
	TVertexInstanceAttributesRef<FVector2f> UVs = Attributes.GetVertexInstanceUVs();
	UVs.SetNumChannels(1);
	TPolygonGroupAttributesRef<FName> SlotNames = Attributes.GetPolygonGroupMaterialSlotNames();

	// Two groups always, in slot order; the build drops the empty one.
	const FPolygonGroupID ExteriorGroup = MeshDescription.CreatePolygonGroup();
	SlotNames[ExteriorGroup] = ExteriorSlotName;
	const FPolygonGroupID InteriorGroup = MeshDescription.CreatePolygonGroup();
	SlotNames[InteriorGroup] = InteriorSlotName;

	MeshDescription.ReserveNewVertices(CornerTotal);
	MeshDescription.ReserveNewVertexInstances(CornerTotal);
	MeshDescription.ReserveNewTriangles(TriangleTotal);
	MeshDescription.ReserveNewEdges(TriangleTotal * 3);

	TArray<FVertexInstanceID> Instances;
	for (const int32 FragmentIndex : FragmentIndices)
	{
		const FFragmentRenderGeometry& Geometry = RenderGeometry[FragmentIndex];
		const FVector Offset = bCentroidRelative ? FVector::ZeroVector : Fragments[FragmentIndex].Centroid;
		Instances.Reset(Geometry.Vertices.Num());
		for (int32 Corner = 0; Corner < Geometry.Vertices.Num(); ++Corner)
		{
			const FVertexID Vertex = MeshDescription.CreateVertex();
			Positions[Vertex] = FVector3f(Geometry.Vertices[Corner] + Offset);
			const FVertexInstanceID Instance = MeshDescription.CreateVertexInstance(Vertex);
			Normals[Instance] = FVector3f(Geometry.Normals[Corner]);
			Tangents[Instance] = FVector3f(Geometry.Tangents[Corner]);
			BinormalSigns[Instance] = Geometry.BinormalSigns[Corner];
			UVs.Set(Instance, 0, FVector2f(Geometry.UVs[Corner]));
			Instances.Add(Instance);
		}
		const auto AddTriangles = [&](const TArray<int32>& Triangles, FPolygonGroupID Group)
		{
			for (int32 Index = 0; Index + 2 < Triangles.Num(); Index += 3)
			{
				const FVertexInstanceID Triangle[3] = {
					Instances[Triangles[Index]], Instances[Triangles[Index + 1]], Instances[Triangles[Index + 2]] };
				MeshDescription.CreateTriangle(Group, Triangle);
			}
		};
		AddTriangles(Geometry.ExteriorTriangles, ExteriorGroup);
		AddTriangles(Geometry.InteriorTriangles, InteriorGroup);
	}

	UStaticMesh* StaticMesh = NewObject<UStaticMesh>(this, NAME_None, RF_Transient);
	UMaterialInterface* InteriorMaterial = CoreMaterial != nullptr ? CoreMaterial.Get() : ExteriorMaterial.Get();
	StaticMesh->GetStaticMaterials().Add(FStaticMaterial(ExteriorMaterial, ExteriorSlotName));
	StaticMesh->GetStaticMaterials().Add(FStaticMaterial(InteriorMaterial, InteriorSlotName));

	// The fast path builds render data straight from the description — no
	// source model, no DDC — which is also the only path available in packaged
	// builds.
	UStaticMesh::FBuildMeshDescriptionsParams BuildParams;
	BuildParams.bFastBuild = true;
	BuildParams.bCommitMeshDescription = false;
	BuildParams.bMarkPackageDirty = false;
	BuildParams.bBuildSimpleCollision = false;
	StaticMesh->BuildFromMeshDescriptions({ &MeshDescription }, BuildParams);
	++RenderBuildCount;
	return StaticMesh;
}

int32 ABox3DFracturedActor::CountLooseFragments() const
{
	int32 Count = 0;
	for (const EBox3DFragmentRenderState State : FragmentRenderStates)
	{
		Count += State == EBox3DFragmentRenderState::Loose ? 1 : 0;
	}
	return Count;
}

int32 ABox3DFracturedActor::GetRenderPrimitiveCount() const
{
	return (AttachedMesh != nullptr && AttachedMesh->GetStaticMesh() != nullptr ? 1 : 0) + CountLooseFragments();
}

const TArray<FVector>* ABox3DFracturedActor::GetFragmentRenderVertices(int32 FragmentIndex) const
{
	return RenderGeometry.IsValidIndex(FragmentIndex)
			&& GetFragmentRenderState(FragmentIndex) != EBox3DFragmentRenderState::None
		? &RenderGeometry[FragmentIndex].Vertices
		: nullptr;
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
			FVector Local = Vertex - Fragment.Centroid;
			if (HullInsetCm > 0.0f)
			{
				const double Length = Local.Size();
				Local *= Length > HullInsetCm ? (Length - HullInsetCm) / Length : 0.0;
			}
			Points.Add(Box3D::ToB3(Local));
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
	int32 AnchorCount = 0;
	if (bAnchorAllFragments)
	{
		// Cladding glued to an immovable surface: support never depends on what
		// the b3 world happens to hold behind the mesh.
		for (int32 Index = 0; Index < FragmentBodies.Num(); ++Index)
		{
			if (b3Body_IsValid(FragmentBodies[Index]))
			{
				StructureGraph.SetAnchor(Index, true);
				++AnchorCount;
			}
		}
	}
	else
	{
		AnchorCount = Box3D::Structure::DetectAnchors(StructureGraph, Subsystem->GetBox3DWorldId(), FragmentBodies);
	}
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
	if (!Promoted.IsEmpty())
	{
		MarkRenderDirty();
	}
}

void ABox3DFracturedActor::DestroyFragment(int32 FragmentIndex)
{
	if (!Fragments.IsValidIndex(FragmentIndex))
	{
		return;
	}
	// A detached chip is already gone from the structure graph but still has a
	// body and is drawn, so "already destroyed" is judged on those, not the graph.
	const bool bHasBody = FragmentBodies.IsValidIndex(FragmentIndex) && b3Body_IsValid(FragmentBodies[FragmentIndex]);
	const bool bDrawn = FragmentDestroyed.IsValidIndex(FragmentIndex) && !FragmentDestroyed[FragmentIndex]
		&& GetFragmentTier(FragmentIndex) == EBox3DFragmentTier::Body;
	if (!bHasBody && !bDrawn)
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

	if (FragmentDestroyed.IsValidIndex(FragmentIndex))
	{
		FragmentDestroyed[FragmentIndex] = true;
	}
	MarkRenderDirty();

	if (bStructureActive && !StructureGraph.IsChunkDestroyed(FragmentIndex))
	{
		Box3D::Structure::FBox3DStructureIslands Islands;
		StructureGraph.NotifyChunkDestroyed(FragmentIndex, Islands);
		EnqueueIslands(Islands);
		bStressTopologyDirty = true;
	}
}

bool ABox3DFracturedActor::DetachFragment(int32 FragmentIndex, FVector WorldLinearVelocity,
	FVector WorldAngularVelocity, FVector WorldNudgeCm)
{
	if (!FragmentBodies.IsValidIndex(FragmentIndex) || !b3Body_IsValid(FragmentBodies[FragmentIndex]))
	{
		return false;
	}
	const b3BodyId Body = FragmentBodies[FragmentIndex];

	// Neighbours stay asleep: a chip leaving a static cladding must not wake the
	// rest of the assembly into pointless solver work.
	for (int32 Index = Welds.Num() - 1; Index >= 0; --Index)
	{
		const FWeld& Weld = Welds[Index];
		if (Weld.FragmentA == FragmentIndex || Weld.FragmentB == FragmentIndex)
		{
			if (b3Joint_IsValid(Weld.Joint))
			{
				b3DestroyJoint(Weld.Joint, /*wakeAttached*/ false);
			}
			Welds.RemoveAtSwap(Index);
		}
	}

	if (bStructureActive && !StructureGraph.IsChunkDestroyed(FragmentIndex))
	{
		Box3D::Structure::FBox3DStructureIslands Islands;
		StructureGraph.NotifyChunkDestroyed(FragmentIndex, Islands);
		EnqueueIslands(Islands);
		bStressTopologyDirty = true;
	}

	if (b3Body_GetType(Body) != b3_dynamicBody)
	{
		b3Body_SetType(Body, b3_dynamicBody);
	}
	b3Body_SetAwake(Body, true);
	if (!WorldNudgeCm.IsNearlyZero())
	{
		const b3WorldTransform Pose = b3Body_GetTransform(Body);
		b3Body_SetTransform(Body, Box3D::ToB3Pos(Box3D::ToUEPos(Pose.p) + WorldNudgeCm), Pose.q);
	}
	b3Body_SetLinearVelocity(Body, Box3D::ToB3(WorldLinearVelocity));
	b3Body_SetAngularVelocity(Body, Box3D::ToB3Dir(WorldAngularVelocity));
	MarkRenderDirty();
	return true;
}

bool ABox3DFracturedActor::IsFragmentAlive(int32 FragmentIndex) const
{
	return FragmentBodies.IsValidIndex(FragmentIndex) && b3Body_IsValid(FragmentBodies[FragmentIndex]);
}

bool ABox3DFracturedActor::IsFragmentAttached(int32 FragmentIndex) const
{
	return IsFragmentAlive(FragmentIndex) && b3Body_GetType(FragmentBodies[FragmentIndex]) == b3_staticBody;
}

int32 ABox3DFracturedActor::CountAliveFragments() const
{
	int32 Count = 0;
	for (int32 Index = 0; Index < FragmentBodies.Num(); ++Index)
	{
		Count += IsFragmentAlive(Index) ? 1 : 0;
	}
	return Count;
}

int32 ABox3DFracturedActor::CountAttachedFragments() const
{
	int32 Count = 0;
	for (int32 Index = 0; Index < FragmentBodies.Num(); ++Index)
	{
		Count += IsFragmentAttached(Index) ? 1 : 0;
	}
	return Count;
}

FVector ABox3DFracturedActor::GetFragmentWorldCentroid(int32 FragmentIndex) const
{
	if (!Fragments.IsValidIndex(FragmentIndex))
	{
		return GetActorLocation();
	}
	if (IsFragmentAlive(FragmentIndex))
	{
		return Box3D::ToUEPos(b3Body_GetPosition(FragmentBodies[FragmentIndex]));
	}
	return GetActorTransform().TransformPosition(Fragments[FragmentIndex].Centroid);
}

int32 ABox3DFracturedActor::FindFragmentAtPoint(FVector WorldPoint, bool bAttachedOnly) const
{
	constexpr double Tolerance = 0.05;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		if (!IsFragmentAlive(Index) || (bAttachedOnly && !IsFragmentAttached(Index)))
		{
			continue;
		}
		const FBox3DFragmentData& Fragment = Fragments[Index];
		// Hull geometry is actor space at spawn; the body frame sits at the
		// centroid with the actor's spawn rotation, so map the point through the
		// live body pose back into that space.
		const b3WorldTransform Body = b3Body_GetTransform(FragmentBodies[Index]);
		const FTransform BodyToWorld(Box3D::ToUE(Body.q), Box3D::ToUEPos(Body.p));
		const FVector Local = BodyToWorld.InverseTransformPosition(WorldPoint) + Fragment.Centroid;

		bool bInside = true;
		for (const FBox3DFragmentFace& Face : Fragment.Faces)
		{
			if (Face.VertexIndices.Num() < 3)
			{
				continue;
			}
			const FVector& V0 = Fragment.Vertices[Face.VertexIndices[0]];
			const FVector& V1 = Fragment.Vertices[Face.VertexIndices[1]];
			const FVector& V2 = Fragment.Vertices[Face.VertexIndices[2]];
			const FVector Normal = FVector::CrossProduct(V1 - V0, V2 - V0).GetSafeNormal();
			if (FVector::DotProduct(Normal, Local - V0) > Tolerance)
			{
				bInside = false;
				break;
			}
		}
		if (bInside)
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

int32 ABox3DFracturedActor::FindNearestFragment(FVector WorldPoint, float MaxDistanceCm, bool bAttachedOnly) const
{
	int32 Best = INDEX_NONE;
	double BestDistanceSq = double(MaxDistanceCm) * double(MaxDistanceCm);
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		if (!IsFragmentAlive(Index) || (bAttachedOnly && !IsFragmentAttached(Index)))
		{
			continue;
		}
		const double DistanceSq = FVector::DistSquared(GetFragmentWorldCentroid(Index), WorldPoint);
		if (DistanceSq <= BestDistanceSq)
		{
			BestDistanceSq = DistanceSq;
			Best = Index;
		}
	}
	return Best;
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
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(RenderFlushTimer);
		if (UBox3DWorldSubsystem* Subsystem = World->GetSubsystem<UBox3DWorldSubsystem>())
		{
			Subsystem->UnregisterFracturedActor(this);
		}
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
	if (RenderGeometry.IsEmpty() || AttachedMesh == nullptr)
	{
		return 0;
	}
	FlushRenderState();

	const FTransform WorldToActor = GetActorTransform().Inverse();
	const FVector ActorImpactPoint = WorldToActor.TransformPosition(WorldImpactPoint);
	const FVector ActorImpactNormal = WorldToActor.TransformVectorNoScale(WorldImpactNormal).GetSafeNormal();
	if (ActorImpactNormal.IsNearlyZero())
	{
		return 0;
	}

	int32 DisplacedCount = 0;
	bool bAttachedChanged = false;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		const EBox3DFragmentRenderState State = FragmentRenderStates[Index];
		if (State == EBox3DFragmentRenderState::None)
		{
			continue;
		}
		// Body-local -> actor space: the live body pose, or the spawn pose for
		// bodiless fragments.
		FTransform Delta(FQuat::Identity, Fragments[Index].Centroid);
		const b3BodyId Body = GetFragmentBody(Index);
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
		const int32 FragmentDisplaced = Box3D::Deform::ApplyVertexDent(RenderGeometry[Index].Vertices, Dent);
		if (FragmentDisplaced == 0)
		{
			continue;
		}
		DisplacedCount += FragmentDisplaced;

		if (State == EBox3DFragmentRenderState::Attached)
		{
			bAttachedChanged = true;
		}
		else if (UStaticMeshComponent* Component = FragmentComponents[Index])
		{
			Component->SetStaticMesh(BuildStaticMesh(MakeArrayView(&Index, 1), /*bCentroidRelative*/ true));
		}
	}
	if (bAttachedChanged)
	{
		RebuildAttachedMesh();
	}
	return DisplacedCount;
}

void ABox3DFracturedActor::SyncFragments()
{
	if (bRenderDirty)
	{
		FlushRenderState();
	}
	for (int32 Index = 0; Index < FragmentComponents.Num(); ++Index)
	{
		if (FragmentComponents[Index] == nullptr)
		{
			continue;
		}
		// Asleep bodies have not moved since their last synced pose.
		const b3BodyId Body = GetFragmentBody(Index);
		if (!b3Body_IsValid(Body) || !b3Body_IsAwake(Body))
		{
			continue;
		}
		SyncFragmentComponent(Index);
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

	FName GetDestructionMeshId(const UStaticMeshComponent& Component)
	{
		const UStaticMesh* StaticMesh = Component.GetStaticMesh();
		return StaticMesh != nullptr ? FName(*StaticMesh->GetPathName()) : NAME_None;
	}

	static ABox3DFracturedActor* SpawnFracturedActor(UWorld* World, const FTransform& ProxyToWorld,
		const Fracture::FFractureProxy& Proxy, UMaterialInterface* SourceMaterial,
		const FBox3DFractureMeshParams& Params, bool bCreatePhysics, const FString& Context)
	{
		// Fragment space = proxy space (scale baked in), so the world seam is
		// rotation + translation only.
		Fracture::FFractureParams FractureParams = Params.Fracture;
		FractureParams.ImpactPoint = ProxyToWorld.InverseTransformPosition(Params.Fracture.ImpactPoint);

		TArray<Fracture::FBox3DFragmentData> Fragments;
		if (!Fracture::Fracture(Proxy, FractureParams, Fragments))
		{
			UE_LOG(LogBox3D, Warning, TEXT("Box3D fracture produced no fragments for %s"), *Context);
			return nullptr;
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		ABox3DFracturedActor* Actor = World->SpawnActor<ABox3DFracturedActor>(
			ABox3DFracturedActor::StaticClass(), FTransform(ProxyToWorld.GetRotation(), ProxyToWorld.GetLocation()),
			SpawnParams);
		if (Actor == nullptr)
		{
			return nullptr;
		}

		Actor->CoreMaterial = Params.CoreMaterial;
		Actor->MaterialToughness = Params.MaterialToughness;
		Actor->FragmentDensity = Params.FragmentDensity;
		Actor->HullInsetCm = Params.HullInsetCm;
		Actor->bStartAsleep = Params.bStartAsleep;
		Actor->bStructural = Params.bStructural;
		Actor->bAnchorAllFragments = Params.bAnchorAllFragments;
		Actor->TensionStrengthPa = Params.TensionStrengthPa;
		Actor->CompressionStrengthPa = Params.CompressionStrengthPa;
		Actor->ShearStrengthPa = Params.ShearStrengthPa;
		Actor->SustainedOverloadHealthPerSecond = Params.SustainedOverloadHealthPerSecond;
		Actor->TierThresholds = Params.Tiers;
		Actor->DebrisSpeed = Params.DebrisSpeed;
		Actor->DebrisSystem = Params.DebrisSystem;
		Actor->bFragmentsCastShadow = Params.bCastShadow;
		Actor->bFragmentsAffectIndirectLighting = Params.bAffectIndirectLighting;
		Actor->bFragmentsReceiveDecals = Params.bReceivesDecals;
		// Fragment/actor space impact, already converted for the fracture core.
		Actor->DebrisImpactPoint = FractureParams.ImpactPoint;
		Actor->InitializeFragments(MoveTemp(Fragments), SourceMaterial, bCreatePhysics);
		return Actor;
	}

	static ABox3DFracturedActor* FractureMeshInternal(UStaticMeshComponent* Component,
		const FBox3DFractureMeshParams& Params, bool bCreatePhysics)
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

		UWorld* World = Component->GetWorld();
		const FTransform ComponentToWorld(Component->GetComponentQuat(), Component->GetComponentLocation());
		ABox3DFracturedActor* Actor = SpawnFracturedActor(World, ComponentToWorld, Proxy, Component->GetMaterial(0),
			Params, bCreatePhysics, GetNameSafe(Component->GetStaticMesh()));
		if (Actor == nullptr)
		{
			return nullptr;
		}

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

	ABox3DFracturedActor* FractureConvexProxy(UWorld* World, const FTransform& ProxyToWorld,
		const Fracture::FFractureProxy& Proxy, UMaterialInterface* SourceMaterial,
		const FBox3DFractureMeshParams& Params)
	{
		UBox3DWorldSubsystem* Subsystem = World != nullptr ? World->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
		if (Subsystem == nullptr || !Subsystem->IsSimulationAuthority())
		{
			UE_LOG(LogBox3D, Warning, TEXT("Box3D::FractureConvexProxy rejected without simulation authority"));
			return nullptr;
		}
		return SpawnFracturedActor(World, ProxyToWorld, Proxy, SourceMaterial, Params, true, TEXT("convex proxy"));
	}

	ABox3DFracturedActor* FractureMesh(UStaticMeshComponent* Component, const FBox3DFractureMeshParams& Params)
	{
		UBox3DWorldSubsystem* Subsystem = Component != nullptr && Component->GetWorld() != nullptr
			? Component->GetWorld()->GetSubsystem<UBox3DWorldSubsystem>()
			: nullptr;
		if (Subsystem == nullptr || !Subsystem->IsSimulationAuthority())
		{
			UE_LOG(LogBox3D, Warning, TEXT("Box3D::FractureMesh rejected without simulation authority"));
			return nullptr;
		}
		return FractureMeshInternal(Component, Params, true);
	}

	ABox3DFracturedActor* RegenerateFractureVisuals(
		UStaticMeshComponent* Component, const FBox3DFractureMeshParams& Params)
	{
		return FractureMeshInternal(Component, Params, false);
	}
}
