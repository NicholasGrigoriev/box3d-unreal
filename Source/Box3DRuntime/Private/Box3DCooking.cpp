#include "Box3DCooking.h"

#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Engine/StaticMesh.h"
#include "PhysicsEngine/BodySetup.h"
#include "StaticMeshResources.h"
#include "UObject/ObjectKey.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	// Keyed by FObjectKey so a recycled object address can't alias a stale entry.
	// Game-thread only. Values owned here; freed in FlushCookedDataCaches.
	TMap<FObjectKey, b3MeshData*> GMeshCache;
	TMap<FObjectKey, b3HullData*> GHullCache;

	bool GetLOD0Geometry(UStaticMesh* Mesh, TArray<b3Vec3>& OutVertices, TArray<int32>& OutIndices)
	{
		const FStaticMeshRenderData* RenderData = Mesh->GetRenderData();
		if (RenderData == nullptr || RenderData->LODResources.IsEmpty())
		{
			return false;
		}

		const FStaticMeshLODResources& LOD = RenderData->LODResources[0];
		const FPositionVertexBuffer& Positions = LOD.VertexBuffers.PositionVertexBuffer;
		const int32 VertexCount = Positions.GetNumVertices();
		if (VertexCount < 3)
		{
			return false;
		}

		OutVertices.Reserve(VertexCount);
		for (int32 Index = 0; Index < VertexCount; ++Index)
		{
			OutVertices.Add(Box3D::ToB3(FVector(Positions.VertexPosition(Index))));
		}

		TArray<uint32> RawIndices;
		LOD.IndexBuffer.GetCopy(RawIndices);
		const int32 TriangleCount = RawIndices.Num() / 3;
		if (TriangleCount < 1)
		{
			return false;
		}

		// UE winds triangles clockwise viewed from outside; box3d expects the
		// reverse, so flipped winding turns every face inward (bodies fall through
		// the top and rest on the inside of the bottom). Swap two indices per
		// triangle to flip the normals outward.
		OutIndices.Reserve(TriangleCount * 3);
		for (int32 Triangle = 0; Triangle < TriangleCount; ++Triangle)
		{
			OutIndices.Add(static_cast<int32>(RawIndices[Triangle * 3 + 0]));
			OutIndices.Add(static_cast<int32>(RawIndices[Triangle * 3 + 2]));
			OutIndices.Add(static_cast<int32>(RawIndices[Triangle * 3 + 1]));
		}
		return true;
	}
}

namespace Box3D
{
	int32 CreateShapesFromBodySetup(b3BodyId BodyId, b3ShapeDef& ShapeDef, const UBodySetup& BodySetup,
		const FVector& Scale)
	{
		const FVector AbsScale = Scale.GetAbs();
		int32 Created = 0;

		for (const FKSphereElem& Elem : BodySetup.AggGeom.SphereElems)
		{
			const b3Sphere Sphere{ Box3D::ToB3(FVector(Elem.Center) * Scale),
								   Elem.Radius * AbsScale.GetMin() * Box3D::UEToMeters };
			b3CreateSphereShape(BodyId, &ShapeDef, &Sphere);
			++Created;
		}

		for (const FKSphylElem& Elem : BodySetup.AggGeom.SphylElems)
		{
			const FVector AxisOffset = Elem.Rotation.RotateVector(FVector(0, 0, Elem.Length * 0.5));
			const b3Capsule Capsule{
				Box3D::ToB3((FVector(Elem.Center) - AxisOffset) * Scale),
				Box3D::ToB3((FVector(Elem.Center) + AxisOffset) * Scale),
				Elem.Radius * FMath::Min(AbsScale.X, AbsScale.Y) * Box3D::UEToMeters };
			b3CreateCapsuleShape(BodyId, &ShapeDef, &Capsule);
			++Created;
		}

		for (const FKBoxElem& Elem : BodySetup.AggGeom.BoxElems)
		{
			// b3MakeScaledBoxHull exists for exactly this: editor-scaled, rotated boxes.
			const b3Vec3 HalfWidths{ Elem.X * 0.5f * Box3D::UEToMeters,
									 Elem.Y * 0.5f * Box3D::UEToMeters,
									 Elem.Z * 0.5f * Box3D::UEToMeters };
			const b3Transform LocalTransform{ Box3D::ToB3(FVector(Elem.Center)),
											  Box3D::ToB3(Elem.Rotation.Quaternion()) };
			const b3BoxHull Hull = b3MakeScaledBoxHull(HalfWidths, LocalTransform, Box3D::ToB3Dir(Scale));
			b3CreateHullShape(BodyId, &ShapeDef, &Hull.base);
			++Created;
		}

		for (const FKConvexElem& Elem : BodySetup.AggGeom.ConvexElems)
		{
			const FTransform ElemTransform = Elem.GetTransform();
			TArray<b3Vec3> Points;
			Points.Reserve(Elem.VertexData.Num());
			for (const FVector& Vertex : Elem.VertexData)
			{
				Points.Add(Box3D::ToB3(ElemTransform.TransformPosition(Vertex) * Scale));
			}
			if (Points.Num() < 4)
			{
				continue;
			}
			if (b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), 64))
			{
				// Hull shapes clone the data, so the temp cook is freed immediately.
				b3CreateHullShape(BodyId, &ShapeDef, Hull);
				b3DestroyHull(Hull);
				++Created;
			}
		}

		return Created;
	}

	const b3MeshData* GetOrCreateMeshData(UStaticMesh* Mesh)
	{
		if (Mesh == nullptr)
		{
			return nullptr;
		}

		if (b3MeshData** Found = GMeshCache.Find(FObjectKey(Mesh)))
		{
			return *Found;
		}

		TArray<b3Vec3> Vertices;
		TArray<int32> Indices;
		if (!GetLOD0Geometry(Mesh, Vertices, Indices))
		{
			UE_LOG(LogBox3D, Warning, TEXT("Box3D mesh cook failed for %s: no accessible LOD0 geometry"),
				*GetNameSafe(Mesh));
			return nullptr;
		}

		b3MeshDef Def = {};
		Def.vertices = Vertices.GetData();
		Def.indices = Indices.GetData();
		Def.vertexCount = Vertices.Num();
		Def.triangleCount = Indices.Num() / 3;
		Def.weldVertices = true;
		Def.weldTolerance = 0.001f; // 1 mm: render meshes duplicate vertices per-normal/UV
		Def.identifyEdges = true;   // enables ghost-collision mitigation on shared edges

		b3MeshData* MeshData = b3CreateMesh(&Def, nullptr, 0);
		if (MeshData == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("b3CreateMesh failed for %s (%d verts, %d tris)"),
				*GetNameSafe(Mesh), Vertices.Num(), Indices.Num() / 3);
			return nullptr;
		}

		UE_LOG(LogBox3D, Log, TEXT("Box3D cooked mesh %s: %d verts, %d tris"),
			*GetNameSafe(Mesh), Vertices.Num(), Indices.Num() / 3);
		GMeshCache.Add(FObjectKey(Mesh), MeshData);
		return MeshData;
	}

	const b3HullData* GetOrCreateHullData(UStaticMesh* Mesh)
	{
		if (Mesh == nullptr)
		{
			return nullptr;
		}

		if (b3HullData** Found = GHullCache.Find(FObjectKey(Mesh)))
		{
			return *Found;
		}

		TArray<b3Vec3> Points;

		// Prefer authored convex collision; its point cloud is already lean.
		if (const UBodySetup* BodySetup = Mesh->GetBodySetup();
			BodySetup && !BodySetup->AggGeom.ConvexElems.IsEmpty())
		{
			for (const FVector& Vertex : BodySetup->AggGeom.ConvexElems[0].VertexData)
			{
				Points.Add(Box3D::ToB3(Vertex));
			}
		}

		if (Points.Num() < 4)
		{
			Points.Reset();
			TArray<int32> UnusedIndices;
			if (!GetLOD0Geometry(Mesh, Points, UnusedIndices))
			{
				UE_LOG(LogBox3D, Warning, TEXT("Box3D hull cook failed for %s: no convex elements or render data"),
					*GetNameSafe(Mesh));
				return nullptr;
			}
		}

		b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), 64);
		if (Hull == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("b3CreateHull failed for %s (%d points)"),
				*GetNameSafe(Mesh), Points.Num());
			return nullptr;
		}

		UE_LOG(LogBox3D, Log, TEXT("Box3D cooked hull %s from %d points"), *GetNameSafe(Mesh), Points.Num());
		GHullCache.Add(FObjectKey(Mesh), Hull);
		return Hull;
	}

	void FlushCookedDataCaches()
	{
		for (const TPair<FObjectKey, b3MeshData*>& Pair : GMeshCache)
		{
			b3DestroyMesh(Pair.Value);
		}
		GMeshCache.Empty();

		for (const TPair<FObjectKey, b3HullData*>& Pair : GHullCache)
		{
			b3DestroyHull(Pair.Value);
		}
		GHullCache.Empty();
	}
}
