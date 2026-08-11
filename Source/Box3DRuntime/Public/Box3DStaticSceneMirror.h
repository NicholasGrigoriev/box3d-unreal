#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectKey.h"
#include "UObject/WeakObjectPtr.h"
#include "box3d/id.h"

class ULevel;
class UStaticMesh;
class UStaticMeshComponent;
class UWorld;

/// Mirrors qualifying static level geometry into the Box3D world as raw static
/// bodies (no components, null userData), so dynamic Box3D bodies rest on existing
/// maps with zero per-actor setup. Follows level streaming / World Partition via
/// FWorldDelegates. Owned by UBox3DWorldSubsystem; game-thread only.
///
/// Queries that hit mirror bodies report bHit with a null Component/Actor — use
/// FindMirroredComponent for the rare case that needs the source component.
class BOX3DRUNTIME_API FBox3DStaticSceneMirror
{
public:
	~FBox3DStaticSceneMirror() { Shutdown(); }

	/// Registers level-streaming delegates. Call once, after the b3 world exists.
	void Initialize(UWorld* InWorld, b3WorldId InWorldId);

	/// Unregisters delegates and destroys all mirror bodies. Safe to call twice;
	/// must run before the b3 world is destroyed.
	void Shutdown();

	/// Mirror every level currently in the world (subsystem OnWorldBeginPlay).
	void MirrorInitialLevels();

	/// Queue a level's qualifying components, then drain within the configured
	/// budget. Idempotent per level. Streamed-in levels arrive here via delegate.
	void MirrorLevel(ULevel* Level);

	/// Destroy the level's mirror bodies and drop its queued work (stream-out).
	void UnmirrorLevel(ULevel* Level);

	bool HasPendingWork() const { return !PendingQueue.IsEmpty(); }

	/// Cook/create queued bodies until the queue empties or the budget (ms) runs
	/// out. BudgetMs <= 0 drains fully. Called from the subsystem's Tick.
	void DrainQueue(float BudgetMs);

	/// Remove all mirror bodies for a component (every ISM instance). Used by prop
	/// conversion, and by game code when it destroys a mirrored static at runtime.
	/// Returns false if the component was not mirrored.
	bool RemoveComponent(const UStaticMeshComponent* Component);

	/// Remove and immediately re-mirror one component. Needed after ISM instance
	/// removal, which reindexes the remaining instances.
	void RemirrorComponent(UStaticMeshComponent* Component);

	/// The source component of a mirror body, or null (destroyed or not a mirror body).
	UStaticMeshComponent* FindMirroredComponent(b3BodyId BodyId) const;

	int32 GetBodyCount() const { return BodyToComponent.Num(); }

	/// Print totals and per-level counts to the log (backs box3d.MirrorStats).
	void LogStats() const;

	/// Whether a component qualifies for mirroring under the current settings.
	/// Static so the editor bake commandlet applies the exact same filter the
	/// runtime would (docs/BAKED_COLLISION.md).
	static bool ShouldMirror(const UStaticMeshComponent& Component);

private:
	struct FLevelEntry
	{
		/// Bodies per source component, ids packed with b3StoreBodyId.
		TMap<FObjectKey, TArray<uint64>> ComponentBodies;

		FString LevelName;
		int32 PendingCount = 0;  // queued components not yet drained
		int32 ComponentCount = 0;
		int32 BodyCount = 0;
		int32 SkippedCount = 0;  // filtered at discovery (spline/no-collision/too small)
		int32 NewCooks = 0;      // meshes this mirror saw for the first time
		double MirrorSeconds = 0.0;
	};

	struct FPendingEntry
	{
		FObjectKey LevelKey;
		TWeakObjectPtr<UStaticMeshComponent> Component;
	};

	/// Create the body/bodies for one component (N for ISM instances).
	void MirrorComponent(UStaticMeshComponent& Component, FLevelEntry& Entry);

	/// One static body at the given transform. Invalid id if no geometry cooked.
	/// bEnableHitEvents turns on hit events for the body's shapes — set for
	/// destructible-marked components so impacts reach the damage pipeline.
	b3BodyId CreateStaticBody(const FTransform& InstanceToWorld, UStaticMesh& Mesh, const char* DebugName,
		bool bEnableHitEvents);

	void OnLevelAdded(ULevel* Level, UWorld* OwningWorld);
	void OnLevelRemoved(ULevel* Level, UWorld* OwningWorld);

	TMap<FObjectKey /*ULevel*/, FLevelEntry> Levels;
	TMap<uint64 /*b3StoreBodyId*/, TWeakObjectPtr<UStaticMeshComponent>> BodyToComponent;
	TArray<FPendingEntry> PendingQueue;

	/// Mesh assets already requested from the cook cache (new-cook accounting).
	TSet<FObjectKey> SeenMeshes;

	TWeakObjectPtr<UWorld> World;
	b3WorldId WorldId = {};
	FDelegateHandle LevelAddedHandle;
	FDelegateHandle LevelRemovedHandle;
};
