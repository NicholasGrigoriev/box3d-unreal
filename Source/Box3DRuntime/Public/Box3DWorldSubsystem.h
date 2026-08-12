#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Templates/PimplPtr.h"
#include "box3d/id.h"
#include "Box3DWorldSubsystem.generated.h"

class ABox3DFracturedActor;
class UBox3DBodyComponent;
class UBox3DDestructibleComponent;
class UBox3DJointComponent;
class UBox3DPlasticHingeComponent;
class FBox3DBakedScene;
class FBox3DUETaskPool;
class FBox3DDebugDrawer;
class FBox3DStaticSceneMirror;
struct b3Recording;

/// Fired immediately before every fixed step (after kinematic targets are
/// pushed). The right place to apply continuous forces — wind, buoyancy,
/// attractors — because forces are cleared after each step, so per-frame
/// application over- or under-doses depending on frame rate.
DECLARE_MULTICAST_DELEGATE_OneParam(FBox3DPreStepSignature, float /*FixedDeltaTime*/);

/// Snapshot of box3d's per-step profile and simulation counters, readable from
/// Blueprint. Times describe the most recent fixed step, in milliseconds. The
/// same data feeds `stat box3d`.
USTRUCT(BlueprintType)
struct FBox3DWorldStats
{
	GENERATED_BODY()

	/// Whole b3World_Step, ms.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	float StepMs = 0.0f;

	/// Broad-phase pair generation, ms.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	float PairsMs = 0.0f;

	/// Narrow-phase collision, ms.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	float CollideMs = 0.0f;

	/// Constraint solver, ms.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	float SolveMs = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 BodyCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 ShapeCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 ContactCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 JointCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 IslandCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 AwakeBodyCount = 0;

	/// Solver tasks spawned during the last step (0 when single-threaded).
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 TaskCount = 0;

	/// Bytes of memory held by the physics world.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int64 MemoryBytes = 0;
};

/// Owns one Box3D world per game/PIE UWorld and steps it at a fixed timestep.
///
/// Stepping runs on the game thread from Tick using an accumulator clamped by
/// UBox3DSettings::MaxStepsPerTick. Gameplay objects (body components, queries)
/// reach the b3WorldId through this subsystem.
UCLASS()
class BOX3DRUNTIME_API UBox3DWorldSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	//~ USubsystem
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	//~ UWorldSubsystem
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	//~ FTickableGameObject
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

	/// The Box3D world handle. Null id (check with b3World_IsValid) outside of
	/// Initialize/Deinitialize.
	b3WorldId GetBox3DWorldId() const { return WorldId; }

	/// Number of fixed steps performed since world creation.
	uint64 GetStepCount() const { return StepCount; }

	/// True where Box3D simulates. Always true unless bAuthorityOnlySimulation is
	/// enabled, in which case pure clients get no Box3D world at all — body
	/// components stay inert and queries return empty there. Evaluated once at
	/// subsystem initialization from the world's net mode.
	UFUNCTION(BlueprintPure, Category = "Box3D|Networking")
	bool IsSimulationAuthority() const { return bSimulationAuthority; }

	/// Fraction [0, 1] of the way from the last fixed step to the next, for
	/// interpolating raw-body visuals the way bInterpolateBodyTransforms
	/// interpolates body components.
	float GetFixedStepAlpha() const;

	/// Broadcast before every fixed step; see FBox3DPreStepSignature.
	FBox3DPreStepSignature OnPreStep;

	/// Profile times (last fixed step) and simulation counters. Zeroed when the
	/// physics world does not exist.
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	FBox3DWorldStats GetWorldStats() const;

	/// UE task pool driving the solver, or null when stepping single-threaded /
	/// via box3d's internal scheduler.
	const FBox3DUETaskPool* GetTaskPool() const { return TaskPool.Get(); }

	/// Debug-shape wireframe cache; always registered with the world (populated
	/// lazily on first draw, so it costs nothing until debug draw is used).
	FBox3DDebugDrawer* GetDebugDrawer() const { return DebugDrawer.Get(); }

	/// Static level-geometry mirror, or null when disabled in settings.
	FBox3DStaticSceneMirror* GetStaticMirror() const { return StaticMirror.Get(); }

	//~ Recording (box3d's determinism-validated record/replay) ------------------

	/// Begin recording every world mutation (a seed snapshot plus all API calls
	/// and steps, with embedded state hashes). Restarts the buffer if a recording
	/// is already active. Console: box3d.RecordStart.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Recording")
	bool StartRecording();

	/// Finish the active recording. The buffer stays available for saving or
	/// validation until the next StartRecording or world teardown.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Recording")
	bool StopRecording();

	UFUNCTION(BlueprintPure, Category = "Box3D|Recording")
	bool IsRecording() const { return bRecordingActive; }

	/// Write the last stopped recording to disk (directories are created).
	/// Replay/inspect it with box3d.ValidateReplay <path>.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Recording")
	bool SaveRecordingToFile(const FString& Path) const;

	/// Replay the last stopped recording in a scratch world and check every
	/// embedded state hash — box3d's end-to-end determinism validation.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Recording")
	bool ValidateLastRecording() const;

	/// Raw buffer access for tests/tools (null when nothing recorded).
	const b3Recording* GetRecording() const { return Recording; }

	/// Kinematic bodies receive velocity-based transform targets before each fixed
	/// step (b3Body_SetTargetTransform), so they collide smoothly instead of
	/// teleporting. Body components register themselves on creation.
	void RegisterKinematicBody(UBox3DBodyComponent* Component);
	void UnregisterKinematicBody(UBox3DBodyComponent* Component);

	/// Joints whose bodies were not created yet at their BeginPlay; retried
	/// before each tick's stepping until creation succeeds or attempts run out.
	void AddPendingJoint(UBox3DJointComponent* Joint);

	/// Plastic hinges are sampled in registration order after every fixed step so
	/// yielding and angle-break transitions stay deterministic.
	void RegisterPlasticHinge(UBox3DPlasticHingeComponent* Hinge);
	void UnregisterPlasticHinge(UBox3DPlasticHingeComponent* Hinge);

	/// Destructible markers register themselves on BeginPlay. The hit-event pump
	/// routes mirror-body impacts to them, and Box3DExplode's fracture energy
	/// fans out over this registry.
	void RegisterDestructible(UBox3DDestructibleComponent* Component);
	void UnregisterDestructible(UBox3DDestructibleComponent* Component);
	const TArray<TWeakObjectPtr<UBox3DDestructibleComponent>>& GetDestructibles() const { return Destructibles; }

	/// Queue impact damage against a destructible for the budgeted end-of-tick
	/// drain. The hit-event pump feeds this internally; game code can feed it too
	/// when it prefers budgeted destruction over an immediate ApplyImpact. Queue
	/// order is preserved across ticks, so bursts of events fracture
	/// deterministically even when the per-tick budget spreads them out.
	void QueueDestructibleImpact(UBox3DDestructibleComponent* Destructible, const FVector& WorldLocation,
		float EnergyJoules);

	/// Impacts still queued, waiting on the per-tick fracture budget.
	int32 GetPendingDestructibleImpactCount() const { return PendingDestructibleImpacts.Num(); }

	/// Fractured actors register when their fragments initialize — the D3 fragment
	/// pool, oldest first. Registration enforces UBox3DSettings::MaxLiveFragments:
	/// oldest actors are destroyed until the Body-tier fragment total fits.
	void RegisterFracturedActor(ABox3DFracturedActor* Actor);
	void UnregisterFracturedActor(ABox3DFracturedActor* Actor);
	const TArray<TWeakObjectPtr<ABox3DFracturedActor>>& GetLiveFracturedActors() const { return LiveFracturedActors; }

	/// Body-tier fragments across all live fractured actors — the number the
	/// MaxLiveFragments cap is enforced against.
	int32 GetLiveFragmentCount() const;

	/// Drop a body's interpolation segment after an explicit teleport so the next
	/// segment starts from the teleported pose instead of rubber-banding.
	void InvalidateInterpolation(UBox3DBodyComponent* Component);

#if !UE_BUILD_SHIPPING
	/// Smoke test (box3d.Smoke): drop debug-drawn bodies onto a static ground slab.
	void SpawnSmokeBodies(int32 Count);
	void ClearSmokeBodies();
#endif

private:
	void StepFixed(float FixedDeltaTime, int32 SubSteps);
	void PushKinematicTargets(float FixedDeltaTime);
	void SyncMovedBodies();

	/// Interpolated rendering (bInterpolateBodyTransforms): instead of writing move
	/// events straight to components, record per-body step segments...
	void RecordMovedBodies();
	/// ...and every tick place components at Lerp(prev, curr, Accumulator/FixedDt).
	/// Costs up to one fixed step of visual latency.
	void ApplyInterpolatedTransforms(float FixedDeltaTime);

	/// Retry joints whose bodies were missing at BeginPlay.
	void CreatePendingJoints();

	/// bConvertSimulatedActors on streamed-in levels (begin-play covers the rest).
	void OnLevelAddedToWorld(ULevel* Level, UWorld* OwningWorld);

	/// Dispatch contact/hit/sensor/joint-threshold events to components. Runs
	/// after every fixed step because box3d buffers events per step only.
	void PumpEvents();

	/// Fracture destructibles hit this tick, oldest queue entries first, stopping
	/// once the per-tick fracture budget is spent (the remainder carries over).
	/// Deferred out of PumpEvents because fracturing creates/destroys bodies,
	/// which must not happen while iterating the step's event buffers.
	void DrainDestructibleImpacts();

	/// Destroy oldest fractured actors while the fragment pool exceeds its cap.
	void EnforceFragmentPool();

	/// Publish profile/counter values to `stat box3d` (compiled out without STATS).
	void UpdateStats() const;

	/// CVar-gated b3World_Draw pass (box3d.DebugDraw) using DrawDebugHelpers.
	void DrawDebugWorld() const;

	/// Instantiate pre-baked static collision (settings-gated). Returns the number
	/// of bodies created; > 0 makes OnWorldBeginPlay skip the mirror's initial
	/// cook. See docs/BAKED_COLLISION.md.
	int32 LoadBakedStaticGeometry();

	b3WorldId WorldId = {};
	float Accumulator = 0.0f;
	uint64 StepCount = 0;
	bool bSimulationAuthority = true;
	FDelegateHandle LevelAddedHandle;

	/// Bridges solver tasks onto UE worker threads (Settings: WorkerCount > 1 with
	/// TaskSystem == UnrealTasks). Must outlive the b3 world. TPimplPtr because the
	/// type lives in a private header the UHT-generated code cannot see.
	TPimplPtr<FBox3DUETaskPool> TaskPool;

	/// Owns cached debug wireframes; box3d calls back into it when shapes are
	/// first drawn and when they are destroyed. Must outlive the b3 world.
	TPimplPtr<FBox3DDebugDrawer> DebugDrawer;

	/// Mirrors static level geometry into raw static bodies (settings-gated).
	/// Shut down before the world dies; its bodies live in the b3 world.
	TPimplPtr<FBox3DStaticSceneMirror> StaticMirror;

	/// Bodies instantiated from baked collision assets (settings-gated). Destroyed
	/// before the world dies, like the mirror.
	TPimplPtr<FBox3DBakedScene> BakedScene;

	/// Reusable recording buffer (box3d resets it on each StartRecording).
	/// Destroyed after the world in Deinitialize; plain pointer because the C
	/// struct is opaque and freed via b3DestroyRecording.
	b3Recording* Recording = nullptr;
	bool bRecordingActive = false;

	TArray<TWeakObjectPtr<UBox3DBodyComponent>> KinematicBodies;
	TArray<TWeakObjectPtr<UBox3DJointComponent>> PendingJoints;
	TArray<TWeakObjectPtr<UBox3DPlasticHingeComponent>> PlasticHinges;
	TArray<TWeakObjectPtr<UBox3DDestructibleComponent>> Destructibles;

	/// Live fractured actors in creation order (oldest first) — the fragment pool.
	TArray<TWeakObjectPtr<ABox3DFracturedActor>> LiveFracturedActors;

	/// One queued impact against a destructible, in event order (deterministic).
	struct FPendingDestructibleImpact
	{
		TWeakObjectPtr<UBox3DDestructibleComponent> Destructible;
		FVector Location = FVector::ZeroVector;
		float EnergyJoules = 0.0f;
	};
	TArray<FPendingDestructibleImpact> PendingDestructibleImpacts;

	/// One segment per dynamic body that moved: pose at the previous and latest
	/// fixed step, plus which step produced P1 (stale segments snap and drop).
	struct FInterpState
	{
		FVector P0 = FVector::ZeroVector;
		FVector P1 = FVector::ZeroVector;
		FQuat Q0 = FQuat::Identity;
		FQuat Q1 = FQuat::Identity;
		uint64 Step = 0;
	};
	TMap<TWeakObjectPtr<UBox3DBodyComponent>, FInterpState> InterpStates;

#if !UE_BUILD_SHIPPING
	void DrawSmokeBodies() const;

	struct FSmokeBody
	{
		b3BodyId Id = {};
		FVector HalfExtentUE = FVector::ZeroVector; // box half extents, cm
		float RadiusUE = 0.0f;                      // sphere radius, cm; 0 = box
	};

	TArray<FSmokeBody> SmokeBodies;
	b3BodyId SmokeGroundId = {};
	FVector SmokeGroundCenterUE = FVector::ZeroVector;
	FVector SmokeGroundHalfExtentUE = FVector::ZeroVector;
#endif
};
