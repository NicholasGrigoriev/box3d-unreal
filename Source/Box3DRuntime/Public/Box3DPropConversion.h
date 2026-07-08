#pragma once

#include "CoreMinimal.h"

class ABox3DPropActor;
class AStaticMeshActor;
class UInstancedStaticMeshComponent;

namespace Box3D
{
	/// Replace a placed static-mesh actor with an ABox3DPropActor at the same
	/// transform (mesh and materials copied). The actor's static-mirror bodies are
	/// removed first, then the original is destroyed. Returns null (and leaves the
	/// actor alone) when it has no mesh or the spawn fails.
	///
	/// Note: an actor destroyed this way respawns if its World Partition cell
	/// unloads and reloads — fine for testing, not a persistence mechanism.
	BOX3DRUNTIME_API ABox3DPropActor* ConvertToProp(AStaticMeshActor* Actor);

	/// Same for a single ISM/HISM instance: the instance is removed from the
	/// component (remaining instances re-mirror, since removal reindexes them) and
	/// a prop spawns at its transform.
	BOX3DRUNTIME_API ABox3DPropActor* ConvertInstanceToProp(UInstancedStaticMeshComponent* Ism, int32 InstanceIndex);

	/// Convert every AStaticMeshActor whose root component has Simulate Physics
	/// authored into a Box3D prop — the pass behind
	/// UBox3DSettings::bConvertSimulatedActors. OnlyLevel restricts the sweep to
	/// one streamed-in level; null sweeps the whole world. Returns conversions.
	BOX3DRUNTIME_API int32 ConvertSimulatedActors(UWorld* World, const ULevel* OnlyLevel = nullptr);
}
