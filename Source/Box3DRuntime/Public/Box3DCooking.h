#pragma once

#include "CoreMinimal.h"
#include "box3d/id.h"

class UBodySetup;
class UStaticMesh;
typedef struct b3MeshData b3MeshData;
typedef struct b3HullData b3HullData;
typedef struct b3ShapeDef b3ShapeDef;

namespace Box3D
{
	/// One b3 shape per authored collision element of the body setup (sphere, capsule,
	/// box, and convex elements), scale baked per axis (rotation with non-uniform scale
	/// is approximated the same way UE itself does). Returns the number of shapes created.
	BOX3DRUNTIME_API int32 CreateShapesFromBodySetup(b3BodyId BodyId, b3ShapeDef& ShapeDef,
		const UBodySetup& BodySetup, const FVector& Scale);

	/// Cooked triangle mesh for a static mesh asset (LOD0 render triangles),
	/// unscaled — scale is applied at shape creation. Cached per asset; the cache
	/// owns the data and keeps it alive for the module's lifetime because mesh
	/// shapes hold references rather than copies. Returns null if the mesh has no
	/// accessible render data.
	BOX3DRUNTIME_API const b3MeshData* GetOrCreateMeshData(UStaticMesh* Mesh);

	/// Cooked convex hull for a static mesh asset, unscaled. Prefers the first
	/// convex element of the mesh's collision setup, falls back to the render
	/// vertices (hull simplified to 64 vertices). Cached per asset. Hull shapes
	/// clone the data, so this cache is purely a cook-cost saver.
	BOX3DRUNTIME_API const b3HullData* GetOrCreateHullData(UStaticMesh* Mesh);

	/// LOD0 render-vertex positions in UE cm, unscaled, no caching. Returns false
	/// when render data is inaccessible — packaged builds strip CPU vertex data
	/// unless the mesh opts into bAllowCPUAccess, so treat this as editor-only.
	BOX3DRUNTIME_API bool GetRenderVertexPositions(UStaticMesh* Mesh, TArray<FVector>& OutPositions);

	/// Destroy all cooked data. Only safe when no Box3D worlds exist (module shutdown).
	void FlushCookedDataCaches();
}
