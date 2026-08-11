#pragma once

#include "CoreMinimal.h"
#include "box3d/id.h"

class UBox3DCollisionData;
typedef struct b3MeshData b3MeshData;

/// Instantiates the bodies of baked collision assets (UBox3DCollisionData) into
/// a Box3D world and owns the resulting resources. Owned by UBox3DWorldSubsystem
/// when bUseBakedStaticCollision is enabled; game-thread only.
///
/// Unlike the static scene mirror this holds no link to source components — the
/// bodies are anonymous statics (null userData), exactly like mirror bodies, so
/// queries report hits with a null Component/Actor.
class BOX3DRUNTIME_API FBox3DBakedScene
{
public:
	~FBox3DBakedScene() { Destroy(); }

	/// Create every body of the asset. Returns the number of bodies created.
	/// Call b3World_RebuildStaticTree after the last asset for fast queries.
	int32 Instantiate(b3WorldId WorldId, const UBox3DCollisionData& Data);

	/// Destroy all instantiated bodies and free their mesh data. Safe to call
	/// twice; must run before the b3 world is destroyed.
	void Destroy();

	int32 GetBodyCount() const { return Bodies.Num(); }

private:
	TArray<uint64> Bodies;       // b3StoreBodyId-packed
	TArray<b3MeshData*> Meshes;  // tri-mesh data the shapes reference; freed after bodies
};
