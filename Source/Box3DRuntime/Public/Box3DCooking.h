#pragma once

#include "CoreMinimal.h"

class UStaticMesh;
typedef struct b3MeshData b3MeshData;
typedef struct b3HullData b3HullData;

namespace Box3D
{
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

	/// Destroy all cooked data. Only safe when no Box3D worlds exist (module shutdown).
	void FlushCookedDataCaches();
}
