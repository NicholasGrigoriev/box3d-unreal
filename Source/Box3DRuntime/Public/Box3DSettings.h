#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "Box3DSettings.generated.h"

/// Which scheduler runs box3d's solver tasks when WorkerCount > 1.
UENUM()
enum class EBox3DTaskSystem : uint8
{
	/// UE::Tasks — solver tasks run on the engine's shared worker threads.
	UnrealTasks,
	/// box3d's built-in scheduler — dedicated threads owned by the physics world.
	Box3DInternal,
};

/// Geometry the static scene mirror cooks for each mirrored mesh.
UENUM()
enum class EBox3DMirrorGeometry : uint8
{
	/// Exact LOD0 render triangles. Props rest on what the player sees, and one
	/// cooked mesh is shared by reference across every instance of an asset.
	TriangleMesh,
	/// Authored simple collision (sphere/capsule/box/convex elements) when the
	/// mesh has any, TriangleMesh otherwise. Cheaper contacts, coarser geometry.
	PreferSimpleCollision,
};

class UBox3DCollisionData;

/// Project settings for the Box3D physics integration.
/// Edit in Project Settings > Plugins > Box3D, saved to DefaultGame.ini.
UCLASS(config = Game, defaultconfig, meta = (DisplayName = "Box3D"))
class BOX3DRUNTIME_API UBox3DSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UBox3DSettings();

	virtual FName GetCategoryName() const override { return TEXT("Plugins"); }

	/// Fixed simulation timestep in seconds. Box3D is designed for fixed stepping;
	/// the subsystem accumulates frame time and steps at this rate.
	UPROPERTY(EditAnywhere, config, Category = "Stepping", meta = (ClampMin = "0.004", ClampMax = "0.05"))
	float FixedTimeStep = 1.0f / 60.0f;

	/// Solver sub-steps per step. Higher improves stacking and joint stiffness.
	UPROPERTY(EditAnywhere, config, Category = "Stepping", meta = (ClampMin = "1", ClampMax = "16"))
	int32 SubStepCount = 4;

	/// Maximum fixed steps performed in one frame. Prevents the accumulator death
	/// spiral after hitches; excess simulation time is dropped.
	UPROPERTY(EditAnywhere, config, Category = "Stepping", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaxStepsPerTick = 4;

	/// Gravity in UE units (cm/s^2). Default matches UE's -980 on Z.
	UPROPERTY(EditAnywhere, config, Category = "World")
	FVector Gravity = FVector(0.0, 0.0, -980.0);

	/// Allow bodies to sleep when they come to rest.
	UPROPERTY(EditAnywhere, config, Category = "World")
	bool bEnableSleep = true;

	/// Enable continuous collision detection for fast-moving bodies.
	UPROPERTY(EditAnywhere, config, Category = "World")
	bool bEnableContinuous = true;

	/// Impacts faster than this (cm/s) fire OnHit on bodies that enable hit
	/// events. Default 100 cm/s = box3d's 1 m/s.
	UPROPERTY(EditAnywhere, config, Category = "World", meta = (ClampMin = "0"))
	float HitEventThreshold = 100.0f;

	/// Worker threads for the solver. 1 = single-threaded. box3d performs best on
	/// performance cores sharing an L2 cache; going past the physical core count
	/// buys nothing. Run `box3d.Benchmark` to size this for a target machine.
	UPROPERTY(EditAnywhere, config, Category = "Performance", meta = (ClampMin = "1", ClampMax = "16"))
	int32 WorkerCount = 1;

	/// Scheduler used when WorkerCount > 1. UnrealTasks shares the engine's worker
	/// threads (no extra threads, coexists with rendering/audio work); Box3DInternal
	/// spins up dedicated threads that only serve physics.
	UPROPERTY(EditAnywhere, config, Category = "Performance")
	EBox3DTaskSystem TaskSystem = EBox3DTaskSystem::UnrealTasks;

	/// Mirror static level geometry into the Box3D world as static collision, so
	/// dynamic Box3D bodies rest on existing maps without per-actor setup. Covers
	/// static-mobility static mesh components (and ISM/HISM instances), follows
	/// level streaming / World Partition. Landscape, BSP, and spline meshes are
	/// not mirrored.
	UPROPERTY(EditAnywhere, config, Category = "Static Scene Mirror")
	bool bMirrorStaticGeometry = false;

	/// Geometry cooked for mirrored meshes.
	UPROPERTY(EditAnywhere, config, Category = "Static Scene Mirror", meta = (EditCondition = "bMirrorStaticGeometry"))
	EBox3DMirrorGeometry MirrorGeometry = EBox3DMirrorGeometry::TriangleMesh;

	/// Mirror instanced static mesh components (one static body per instance).
	UPROPERTY(EditAnywhere, config, Category = "Static Scene Mirror", meta = (EditCondition = "bMirrorStaticGeometry"))
	bool bMirrorInstancedMeshes = true;

	/// Also mirror components whose Chaos collision is query-only. Off by default:
	/// query-only usually means triggers and volumes, but some kits mark real
	/// floors that way.
	UPROPERTY(EditAnywhere, config, Category = "Static Scene Mirror", meta = (EditCondition = "bMirrorStaticGeometry"))
	bool bMirrorQueryOnlyComponents = false;

	/// Skip components whose local bounds radius (scaled, cm) is below this.
	/// 0 mirrors everything.
	UPROPERTY(EditAnywhere, config, Category = "Static Scene Mirror", meta = (ClampMin = "0", EditCondition = "bMirrorStaticGeometry"))
	float MinMirrorBoundsRadius = 0.0f;

	/// Per-frame budget (ms) for mirroring newly streamed-in geometry. 0 mirrors
	/// each level fully on arrival; set a budget if streaming hitches.
	UPROPERTY(EditAnywhere, config, Category = "Static Scene Mirror", meta = (ClampMin = "0", EditCondition = "bMirrorStaticGeometry"))
	float MirrorTimeBudgetMs = 0.0f;

	/// Global cap on simulated (Body-tier) fracture fragments alive at once — the
	/// D3 fragment pool. When a new fracture pushes the total over the cap, the
	/// oldest fractured actors despawn first until it fits again (the newest
	/// fracture always survives, even alone over the cap). 0 = unlimited.
	UPROPERTY(EditAnywhere, config, Category = "Destruction", meta = (ClampMin = "0"))
	int32 MaxLiveFragments = 256;

	/// Per-tick time budget (ms) for fracturing queued destructible impacts. At
	/// least one impact is processed per tick; the rest of the queue carries over
	/// to following ticks in order. 0 = fracture the whole queue every tick.
	UPROPERTY(EditAnywhere, config, Category = "Destruction", meta = (ClampMin = "0"))
	float FractureTimeBudgetMs = 2.0f;

	/// Convert every placed static-mesh actor whose root simulates Chaos physics
	/// into a Box3D prop (ABox3DPropActor) when a game world starts and when
	/// levels stream in. "Simulate Physics" on a placed actor thereby becomes the
	/// author's opt-in for Box3D ownership; static scenery is untouched (that is
	/// the static mirror's job). Manual alternative: the box3d.MakeProp command.
	UPROPERTY(EditAnywhere, config, Category = "Prop Conversion")
	bool bConvertSimulatedActors = false;

	/// Interpolate body component transforms between fixed steps for smooth motion
	/// on displays faster than the fixed step rate. Adds up to one fixed step of
	/// visual latency to Box3D-driven components.
	UPROPERTY(EditAnywhere, config, Category = "Rendering")
	bool bInterpolateBodyTransforms = false;

	/// Only create and step the Box3D world where the simulation is authoritative:
	/// standalone, listen server, and dedicated server. Pure clients get no world —
	/// body components stay inert and queries return empty (use UE traces there, or
	/// drive visuals from replicated authority state; see docs/NETWORKING.md).
	/// Off by default so single-world projects and local experiments are unchanged.
	/// Evaluated when the world subsystem initializes.
	UPROPERTY(EditAnywhere, config, Category = "Networking")
	bool bAuthorityOnlySimulation = false;

	/// Instantiate pre-baked static collision (UBox3DCollisionData assets, produced
	/// by the Box3DBake commandlet) on world begin-play instead of running the
	/// static scene mirror's initial cook. Baked assets need no runtime cooking and
	/// no CPU-accessible render data, so they are the packaged-build path for
	/// static geometry. Levels streamed in after begin-play still use the runtime
	/// mirror (when enabled). See docs/BAKED_COLLISION.md.
	UPROPERTY(EditAnywhere, config, Category = "Baked Static Collision")
	bool bUseBakedStaticCollision = false;

	/// Baked collision assets to instantiate on world begin-play, on top of
	/// whatever bAutoDiscoverBakedCollision finds (duplicates are ignored).
	UPROPERTY(EditAnywhere, config, Category = "Baked Static Collision",
		meta = (EditCondition = "bUseBakedStaticCollision"))
	TArray<TSoftObjectPtr<UBox3DCollisionData>> BakedCollisionAssets;

	/// Also load the current map's baked asset (BC_<MapName> beside the map)
	/// without it being listed above. On by default: forgetting to list an asset
	/// silently loads a level with no static collision, which looks like a physics
	/// bug rather than a config mistake.
	UPROPERTY(EditAnywhere, config, Category = "Baked Static Collision",
		meta = (EditCondition = "bUseBakedStaticCollision"))
	bool bAutoDiscoverBakedCollision = true;
};
