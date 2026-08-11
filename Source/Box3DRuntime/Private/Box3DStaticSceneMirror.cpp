#include "Box3DStaticSceneMirror.h"

#include "Box3DConversion.h"
#include "Box3DCooking.h"
#include "Box3DDestructibleComponent.h"
#include "Box3DRuntime.h"
#include "Box3DSettings.h"
#include "Box3DTypes.h"
#include "Box3DWorldSubsystem.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodySetup.h"
#include "box3d/box3d.h"

void FBox3DStaticSceneMirror::Initialize(UWorld* InWorld, b3WorldId InWorldId)
{
	World = InWorld;
	WorldId = InWorldId;

	LevelAddedHandle = FWorldDelegates::LevelAddedToWorld.AddRaw(this, &FBox3DStaticSceneMirror::OnLevelAdded);
	LevelRemovedHandle = FWorldDelegates::LevelRemovedFromWorld.AddRaw(this, &FBox3DStaticSceneMirror::OnLevelRemoved);
}

void FBox3DStaticSceneMirror::Shutdown()
{
	FWorldDelegates::LevelAddedToWorld.Remove(LevelAddedHandle);
	FWorldDelegates::LevelRemovedFromWorld.Remove(LevelRemovedHandle);
	LevelAddedHandle.Reset();
	LevelRemovedHandle.Reset();

	for (const TPair<uint64, TWeakObjectPtr<UStaticMeshComponent>>& Pair : BodyToComponent)
	{
		const b3BodyId BodyId = b3LoadBodyId(Pair.Key);
		if (b3Body_IsValid(BodyId))
		{
			b3DestroyBody(BodyId);
		}
	}
	BodyToComponent.Empty();
	Levels.Empty();
	PendingQueue.Empty();
	WorldId = b3WorldId{};
}

void FBox3DStaticSceneMirror::MirrorInitialLevels()
{
	const UWorld* WorldPtr = World.Get();
	if (WorldPtr == nullptr)
	{
		return;
	}

	for (ULevel* Level : WorldPtr->GetLevels())
	{
		MirrorLevel(Level);
	}
}

void FBox3DStaticSceneMirror::MirrorLevel(ULevel* Level)
{
	if (Level == nullptr || !b3World_IsValid(WorldId) || Levels.Contains(FObjectKey(Level)))
	{
		return;
	}

	const FObjectKey LevelKey(Level);
	FLevelEntry& Entry = Levels.Add(LevelKey);
	Entry.LevelName = GetNameSafe(Level->GetOuter());

	for (AActor* Actor : Level->Actors)
	{
		if (Actor == nullptr)
		{
			continue;
		}
		Actor->ForEachComponent<UStaticMeshComponent>(/*bIncludeFromChildActors*/ false,
			[this, &Entry, &LevelKey](UStaticMeshComponent* Component)
			{
				if (Component->GetStaticMesh() == nullptr ||
					Component->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
				{
					return; // decorative; not worth counting as skipped
				}
				if (ShouldMirror(*Component))
				{
					PendingQueue.Add({ LevelKey, Component });
					++Entry.PendingCount;
				}
				else
				{
					++Entry.SkippedCount;
				}
			});
	}

	if (Entry.PendingCount == 0)
	{
		UE_LOG(LogBox3D, Log, TEXT("Box3D mirror: %s — nothing to mirror (%d skipped)"),
			*Entry.LevelName, Entry.SkippedCount);
		return;
	}

	DrainQueue(GetDefault<UBox3DSettings>()->MirrorTimeBudgetMs);
}

void FBox3DStaticSceneMirror::UnmirrorLevel(ULevel* Level)
{
	const FObjectKey LevelKey(Level);
	FLevelEntry Entry;
	if (!Levels.RemoveAndCopyValue(LevelKey, Entry))
	{
		return;
	}

	int32 Destroyed = 0;
	for (const TPair<FObjectKey, TArray<uint64>>& Component : Entry.ComponentBodies)
	{
		for (const uint64 PackedId : Component.Value)
		{
			const b3BodyId BodyId = b3LoadBodyId(PackedId);
			if (b3Body_IsValid(BodyId))
			{
				b3DestroyBody(BodyId);
				++Destroyed;
			}
			BodyToComponent.Remove(PackedId);
		}
	}

	PendingQueue.RemoveAll([&LevelKey](const FPendingEntry& Pending) { return Pending.LevelKey == LevelKey; });

	UE_LOG(LogBox3D, Log, TEXT("Box3D mirror: %s streamed out — %d bodies destroyed"),
		*Entry.LevelName, Destroyed);
}

void FBox3DStaticSceneMirror::DrainQueue(float BudgetMs)
{
	if (PendingQueue.IsEmpty() || !b3World_IsValid(WorldId))
	{
		return;
	}

	const double StartSeconds = FPlatformTime::Seconds();
	const double Deadline = BudgetMs > 0.0f ? StartSeconds + BudgetMs * 0.001 : TNumericLimits<double>::Max();

	int32 Processed = 0;
	for (; Processed < PendingQueue.Num(); ++Processed)
	{
		if (FPlatformTime::Seconds() >= Deadline)
		{
			break;
		}

		const FPendingEntry& Pending = PendingQueue[Processed];
		FLevelEntry* Entry = Levels.Find(Pending.LevelKey);
		if (Entry == nullptr)
		{
			continue; // level streamed out while queued (UnmirrorLevel purges, but be safe)
		}

		const double ComponentStart = FPlatformTime::Seconds();
		if (UStaticMeshComponent* Component = Pending.Component.Get())
		{
			MirrorComponent(*Component, *Entry);
		}
		Entry->MirrorSeconds += FPlatformTime::Seconds() - ComponentStart;

		if (--Entry->PendingCount == 0)
		{
			// Last queued component of this level: one static-tree rebuild, one log line.
			b3World_RebuildStaticTree(WorldId);
			UE_LOG(LogBox3D, Log, TEXT("Box3D mirror: %s — %d components, %d bodies, %d new cooks, %d skipped in %.1f ms"),
				*Entry->LevelName, Entry->ComponentCount, Entry->BodyCount, Entry->NewCooks,
				Entry->SkippedCount, Entry->MirrorSeconds * 1000.0);
		}
	}

	PendingQueue.RemoveAt(0, Processed);
}

bool FBox3DStaticSceneMirror::ShouldMirror(const UStaticMeshComponent& Component)
{
	const UBox3DSettings* Settings = GetDefault<UBox3DSettings>();

	if (Component.Mobility != EComponentMobility::Static)
	{
		return false; // movable statics belong to the prop path, not the mirror
	}

	// Spline meshes deform their source mesh; the cooked geometry would be wrong.
	if (Component.IsA<USplineMeshComponent>())
	{
		return false;
	}

	if (Component.IsA<UInstancedStaticMeshComponent>() && !Settings->bMirrorInstancedMeshes)
	{
		return false;
	}

	const ECollisionEnabled::Type Collision = Component.GetCollisionEnabled();
	const bool bPhysicsEnabled =
		Collision == ECollisionEnabled::QueryAndPhysics || Collision == ECollisionEnabled::PhysicsOnly;
	if (!bPhysicsEnabled && !(Settings->bMirrorQueryOnlyComponents && Collision == ECollisionEnabled::QueryOnly))
	{
		return false; // triggers and overlap volumes are query-only
	}

	if (Settings->MinMirrorBoundsRadius > 0.0f)
	{
		const float ScaledRadius = Component.CalcLocalBounds().SphereRadius *
			Component.GetComponentTransform().GetScale3D().GetAbsMax();
		if (ScaledRadius < Settings->MinMirrorBoundsRadius)
		{
			return false;
		}
	}

	return true;
}

void FBox3DStaticSceneMirror::MirrorComponent(UStaticMeshComponent& Component, FLevelEntry& Entry)
{
	UStaticMesh* Mesh = Component.GetStaticMesh();
	if (Mesh == nullptr)
	{
		return;
	}

	if (!SeenMeshes.Contains(FObjectKey(Mesh)))
	{
		SeenMeshes.Add(FObjectKey(Mesh));
		++Entry.NewCooks;
	}

	const FTCHARToUTF8 NameUtf8(*Component.GetName());
	TArray<uint64> Bodies;

	// Destructible-marked meshes need their mirror shapes to report hit events:
	// that is the D3 damage pipeline's impact intake (Box3DWorldSubsystem routes
	// component-less hit shapes back through FindMirroredComponent).
	const bool bEnableHitEvents = Component.GetOwner() != nullptr
		&& Component.GetOwner()->FindComponentByClass<UBox3DDestructibleComponent>() != nullptr;

	if (const UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(&Component))
	{
		const int32 InstanceCount = Ism->GetInstanceCount();
		Bodies.Reserve(InstanceCount);
		for (int32 Index = 0; Index < InstanceCount; ++Index)
		{
			FTransform InstanceToWorld;
			if (Ism->GetInstanceTransform(Index, InstanceToWorld, /*bWorldSpace*/ true))
			{
				const b3BodyId BodyId = CreateStaticBody(InstanceToWorld, *Mesh, NameUtf8.Get(), bEnableHitEvents);
				if (b3Body_IsValid(BodyId))
				{
					Bodies.Add(b3StoreBodyId(BodyId));
				}
			}
		}
	}
	else
	{
		const b3BodyId BodyId = CreateStaticBody(Component.GetComponentTransform(), *Mesh, NameUtf8.Get(), bEnableHitEvents);
		if (b3Body_IsValid(BodyId))
		{
			Bodies.Add(b3StoreBodyId(BodyId));
		}
	}

	if (Bodies.IsEmpty())
	{
		++Entry.SkippedCount; // mesh had no cookable geometry
		return;
	}

	for (const uint64 PackedId : Bodies)
	{
		BodyToComponent.Add(PackedId, &Component);
	}
	Entry.BodyCount += Bodies.Num();
	++Entry.ComponentCount;
	Entry.ComponentBodies.Add(FObjectKey(&Component), MoveTemp(Bodies));
}

b3BodyId FBox3DStaticSceneMirror::CreateStaticBody(const FTransform& InstanceToWorld, UStaticMesh& Mesh,
	const char* DebugName, bool bEnableHitEvents)
{
	b3BodyDef BodyDef = b3DefaultBodyDef();
	BodyDef.type = b3_staticBody;
	BodyDef.position = Box3D::ToB3Pos(InstanceToWorld.GetLocation());
	BodyDef.rotation = Box3D::ToB3(InstanceToWorld.GetRotation());
	BodyDef.name = DebugName;
	// Queries resolve userData straight to UObject; mirror bodies have no UObject,
	// so this MUST stay null (hits report a null component, which is correct).
	BodyDef.userData = nullptr;

	const b3BodyId BodyId = b3CreateBody(WorldId, &BodyDef);

	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	ShapeDef.filter.categoryBits = Box3D::ToB3Bits(1 << static_cast<int32>(EBox3DChannel::WorldStatic));
	ShapeDef.filter.maskBits = UINT64_MAX;
	ShapeDef.enableSensorEvents = false;
	ShapeDef.enableContactEvents = false;
	ShapeDef.enableHitEvents = bEnableHitEvents;

	const FVector Scale = InstanceToWorld.GetScale3D();
	int32 Created = 0;

	if (GetDefault<UBox3DSettings>()->MirrorGeometry == EBox3DMirrorGeometry::PreferSimpleCollision)
	{
		if (const UBodySetup* BodySetup = Mesh.GetBodySetup();
			BodySetup && BodySetup->AggGeom.GetElementCount() > 0)
		{
			Created = Box3D::CreateShapesFromBodySetup(BodyId, ShapeDef, *BodySetup, Scale);
		}
	}

	if (Created == 0)
	{
		if (const b3MeshData* MeshData = Box3D::GetOrCreateMeshData(&Mesh))
		{
			b3CreateMeshShape(BodyId, &ShapeDef, MeshData, Box3D::ToB3Dir(Scale));
			Created = 1;
		}
	}

	if (Created == 0)
	{
		b3DestroyBody(BodyId);
		return b3BodyId{};
	}
	return BodyId;
}

bool FBox3DStaticSceneMirror::RemoveComponent(const UStaticMeshComponent* Component)
{
	if (Component == nullptr)
	{
		return false;
	}

	const FObjectKey ComponentKey(Component);
	bool bRemoved = false;

	for (TPair<FObjectKey, FLevelEntry>& Level : Levels)
	{
		TArray<uint64> Bodies;
		if (!Level.Value.ComponentBodies.RemoveAndCopyValue(ComponentKey, Bodies))
		{
			continue;
		}
		for (const uint64 PackedId : Bodies)
		{
			const b3BodyId BodyId = b3LoadBodyId(PackedId);
			if (b3Body_IsValid(BodyId))
			{
				b3DestroyBody(BodyId);
			}
			BodyToComponent.Remove(PackedId);
		}
		Level.Value.BodyCount -= Bodies.Num();
		--Level.Value.ComponentCount;
		bRemoved = true;
		break;
	}

	// Not drained yet still counts as mirrored: drop the queued work too.
	const int32 Purged = PendingQueue.RemoveAll([Component](const FPendingEntry& Pending)
		{ return Pending.Component.Get() == Component; });
	if (Purged > 0)
	{
		for (TPair<FObjectKey, FLevelEntry>& Level : Levels)
		{
			// PendingCount bookkeeping is per level; a purged entry belongs to the
			// level that queued it.
			if (Level.Key == FObjectKey(Component->GetComponentLevel()))
			{
				Level.Value.PendingCount = FMath::Max(0, Level.Value.PendingCount - Purged);
			}
		}
		bRemoved = true;
	}

	return bRemoved;
}

void FBox3DStaticSceneMirror::RemirrorComponent(UStaticMeshComponent* Component)
{
	if (Component == nullptr)
	{
		return;
	}

	RemoveComponent(Component);

	if (!ShouldMirror(*Component))
	{
		return;
	}
	FLevelEntry* Entry = Levels.Find(FObjectKey(Component->GetComponentLevel()));
	if (Entry == nullptr)
	{
		return; // level not mirrored (streamed out or mirroring disabled at load)
	}
	MirrorComponent(*Component, *Entry);
	b3World_RebuildStaticTree(WorldId);
}

UStaticMeshComponent* FBox3DStaticSceneMirror::FindMirroredComponent(b3BodyId BodyId) const
{
	const TWeakObjectPtr<UStaticMeshComponent>* Found = BodyToComponent.Find(b3StoreBodyId(BodyId));
	return Found ? Found->Get() : nullptr;
}

void FBox3DStaticSceneMirror::LogStats() const
{
	UE_LOG(LogBox3D, Log, TEXT("Box3D mirror: %d bodies across %d levels (%d queued)"),
		BodyToComponent.Num(), Levels.Num(), PendingQueue.Num());
	for (const TPair<FObjectKey, FLevelEntry>& Level : Levels)
	{
		UE_LOG(LogBox3D, Log, TEXT("  %s: %d components, %d bodies, %d pending, %d skipped, %.1f ms"),
			*Level.Value.LevelName, Level.Value.ComponentCount, Level.Value.BodyCount,
			Level.Value.PendingCount, Level.Value.SkippedCount, Level.Value.MirrorSeconds * 1000.0);
	}
}

void FBox3DStaticSceneMirror::OnLevelAdded(ULevel* Level, UWorld* OwningWorld)
{
	if (Level != nullptr && OwningWorld == World.Get())
	{
		MirrorLevel(Level);
	}
}

void FBox3DStaticSceneMirror::OnLevelRemoved(ULevel* Level, UWorld* OwningWorld)
{
	if (Level != nullptr && OwningWorld == World.Get())
	{
		UnmirrorLevel(Level);
	}
}

#if !UE_BUILD_SHIPPING
static FAutoConsoleCommandWithWorldAndArgs GBox3DMirrorStatsCmd(
	TEXT("box3d.MirrorStats"),
	TEXT("Log static scene mirror totals and per-level body counts."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(
		[](const TArray<FString>&, UWorld* World)
		{
			const UBox3DWorldSubsystem* Subsystem = World ? World->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
			if (Subsystem == nullptr || Subsystem->GetStaticMirror() == nullptr)
			{
				UE_LOG(LogBox3D, Log, TEXT("box3d.MirrorStats: static scene mirror is disabled (see Box3D settings)"));
				return;
			}
			Subsystem->GetStaticMirror()->LogStats();
		}));
#endif
