#include "Box3DFracturedActor.h"

#include "Box3DConversion.h"
#include "Box3DCooking.h"
#include "Box3DRuntime.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "PhysicsEngine/BodySetup.h"
#include "ProceduralMeshComponent.h"
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
	PrimaryActorTick.bCanEverTick = false;

	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("FracturedMesh"));
	Mesh->SetMobility(EComponentMobility::Movable);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetRootComponent(Mesh);
}

void ABox3DFracturedActor::InitializeFragments(TArray<FBox3DFragmentData>&& InFragments,
	UMaterialInterface* SourceMaterial)
{
	Fragments = MoveTemp(InFragments);
	FragmentSections.Init(FIntPoint(INDEX_NONE, INDEX_NONE), Fragments.Num());
	Mesh->ClearAllMeshSections();

	UMaterialInterface* InteriorMaterial = CoreMaterial != nullptr ? CoreMaterial.Get() : SourceMaterial;
	int32 SectionIndex = 0;

	for (int32 FragmentIndex = 0; FragmentIndex < Fragments.Num(); ++FragmentIndex)
	{
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
			return SectionIndex++;
		};

		FragmentSections[FragmentIndex].X = CreateSection(Exterior, SourceMaterial);
		FragmentSections[FragmentIndex].Y = CreateSection(Interior, InteriorMaterial);
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
