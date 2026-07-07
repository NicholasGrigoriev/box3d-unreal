#include "Box3DWorldSubsystem.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DDebugDraw.h"
#include "Box3DJointComponent.h"
#include "Box3DRuntime.h"
#include "Box3DSettings.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DTaskSystem.h"
#include "Box3DTypes.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

// `stat box3d`. Cycle stats time our game-thread work; float/dword counters mirror
// box3d's own per-step profile and world counters.
DECLARE_STATS_GROUP(TEXT("Box3D"), STATGROUP_Box3D, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Tick (game thread)"), STAT_Box3DTick, STATGROUP_Box3D);
DECLARE_FLOAT_COUNTER_STAT(TEXT("Step (ms)"), STAT_Box3DStepMs, STATGROUP_Box3D);
DECLARE_FLOAT_COUNTER_STAT(TEXT("Pairs (ms)"), STAT_Box3DPairsMs, STATGROUP_Box3D);
DECLARE_FLOAT_COUNTER_STAT(TEXT("Collide (ms)"), STAT_Box3DCollideMs, STATGROUP_Box3D);
DECLARE_FLOAT_COUNTER_STAT(TEXT("Solve (ms)"), STAT_Box3DSolveMs, STATGROUP_Box3D);
DECLARE_FLOAT_COUNTER_STAT(TEXT("Tree refit (ms)"), STAT_Box3DRefitMs, STATGROUP_Box3D);
DECLARE_FLOAT_COUNTER_STAT(TEXT("Transforms (ms)"), STAT_Box3DTransformsMs, STATGROUP_Box3D);
DECLARE_FLOAT_COUNTER_STAT(TEXT("Sensors (ms)"), STAT_Box3DSensorsMs, STATGROUP_Box3D);
DECLARE_DWORD_COUNTER_STAT(TEXT("Bodies"), STAT_Box3DBodies, STATGROUP_Box3D);
DECLARE_DWORD_COUNTER_STAT(TEXT("Awake bodies"), STAT_Box3DAwakeBodies, STATGROUP_Box3D);
DECLARE_DWORD_COUNTER_STAT(TEXT("Shapes"), STAT_Box3DShapes, STATGROUP_Box3D);
DECLARE_DWORD_COUNTER_STAT(TEXT("Contacts"), STAT_Box3DContacts, STATGROUP_Box3D);
DECLARE_DWORD_COUNTER_STAT(TEXT("Joints"), STAT_Box3DJoints, STATGROUP_Box3D);
DECLARE_DWORD_COUNTER_STAT(TEXT("Islands"), STAT_Box3DIslands, STATGROUP_Box3D);
DECLARE_DWORD_COUNTER_STAT(TEXT("Solver tasks"), STAT_Box3DTasks, STATGROUP_Box3D);
DECLARE_MEMORY_STAT(TEXT("World memory"), STAT_Box3DMemory, STATGROUP_Box3D);

void UBox3DWorldSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	const UBox3DSettings* Settings = GetDefault<UBox3DSettings>();

	b3WorldDef Def = b3DefaultWorldDef();
	Def.gravity = Box3D::ToB3Accel(Settings->Gravity);
	Def.enableSleep = Settings->bEnableSleep;
	Def.enableContinuous = Settings->bEnableContinuous;
	Def.hitEventThreshold = Settings->HitEventThreshold * Box3D::UEToMeters;
	Def.workerCount = static_cast<uint32>(Settings->WorkerCount);

	if (Settings->WorkerCount > 1 && Settings->TaskSystem == EBox3DTaskSystem::UnrealTasks)
	{
		TaskPool = MakePimpl<FBox3DUETaskPool>();
		TaskPool->ApplyToWorldDef(Def, Settings->WorkerCount);
	}

	// Registered unconditionally: box3d only calls back when a shape is first
	// drawn, so the cache is free until box3d.DebugDraw is turned on.
	DebugDrawer = MakePimpl<FBox3DDebugDrawer>();
	Def.createDebugShape = &FBox3DDebugDrawer::CreateShapeThunk;
	Def.destroyDebugShape = &FBox3DDebugDrawer::DestroyShapeThunk;
	Def.userDebugShapeContext = DebugDrawer.Get();

	WorldId = b3CreateWorld(&Def);
	Accumulator = 0.0f;
	StepCount = 0;

	if (Settings->bMirrorStaticGeometry)
	{
		StaticMirror = MakePimpl<FBox3DStaticSceneMirror>();
		StaticMirror->Initialize(GetWorld(), WorldId);
	}

	UE_LOG(LogBox3D, Log, TEXT("Box3D world created for %s (gravity=%s cm/s^2, workers=%d, scheduler=%s)"),
		*GetNameSafe(GetWorld()), *Settings->Gravity.ToCompactString(), Settings->WorkerCount,
		Settings->WorkerCount <= 1 ? TEXT("serial")
			: (TaskPool ? TEXT("UE tasks") : TEXT("box3d internal")));
}

void UBox3DWorldSubsystem::Deinitialize()
{
#if !UE_BUILD_SHIPPING
	ClearSmokeBodies();
#endif

	// Before the world: destroys mirror bodies and unhooks level delegates while
	// the b3 world can still accept b3DestroyBody.
	if (StaticMirror.IsValid())
	{
		StaticMirror->Shutdown();
	}

	if (b3World_IsValid(WorldId))
	{
		b3DestroyWorld(WorldId);
		UE_LOG(LogBox3D, Log, TEXT("Box3D world destroyed for %s after %llu steps"),
			*GetNameSafe(GetWorld()), StepCount);
	}
	WorldId = b3WorldId{};

	// After the world: destroying it fires destroyDebugShape into the drawer,
	// detaches any active recording, and task handles must not outlive their pool.
	if (Recording != nullptr)
	{
		b3DestroyRecording(Recording);
		Recording = nullptr;
	}
	bRecordingActive = false;
	StaticMirror.Reset();
	DebugDrawer.Reset();
	TaskPool.Reset();

	Super::Deinitialize();
}

bool UBox3DWorldSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UBox3DWorldSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	if (StaticMirror.IsValid())
	{
		StaticMirror->MirrorInitialLevels();
	}
}

void UBox3DWorldSubsystem::Tick(float DeltaTime)
{
	SCOPE_CYCLE_COUNTER(STAT_Box3DTick);
	Super::Tick(DeltaTime);

	if (!b3World_IsValid(WorldId))
	{
		return;
	}

	const UBox3DSettings* Settings = GetDefault<UBox3DSettings>();
	const float FixedDt = Settings->FixedTimeStep;

	CreatePendingJoints();

	// Mirror work left over from a budgeted stream-in drain.
	if (StaticMirror.IsValid() && StaticMirror->HasPendingWork())
	{
		StaticMirror->DrainQueue(Settings->MirrorTimeBudgetMs);
	}

	// Fixed-step accumulator; drop time beyond the per-frame budget so a hitch
	// cannot snowball into ever-longer frames.
	Accumulator = FMath::Min(Accumulator + DeltaTime, Settings->MaxStepsPerTick * FixedDt);
	bool bStepped = false;
	while (Accumulator >= FixedDt)
	{
		PushKinematicTargets(FixedDt);
		StepFixed(FixedDt, Settings->SubStepCount);
		// Events are buffered for the last step only, so a begin/end pair split
		// across two steps of the same tick would vanish if pumped once per tick.
		PumpEvents();
		Accumulator -= FixedDt;
		bStepped = true;
	}

	if (Settings->bInterpolateBodyTransforms)
	{
		if (bStepped)
		{
			RecordMovedBodies();
		}
		// Every tick, stepped or not: Alpha keeps advancing toward the latest step.
		ApplyInterpolatedTransforms(FixedDt);
	}
	else if (bStepped)
	{
		SyncMovedBodies();
	}

	UpdateStats();

	if (FBox3DDebugDrawer::IsDrawEnabled())
	{
		DrawDebugWorld();
	}

#if !UE_BUILD_SHIPPING
	DrawSmokeBodies();
#endif
}

TStatId UBox3DWorldSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UBox3DWorldSubsystem, STATGROUP_Tickables);
}

void UBox3DWorldSubsystem::StepFixed(float FixedDeltaTime, int32 SubSteps)
{
	if (TaskPool.IsValid())
	{
		TaskPool->ResetForStep();
	}
	b3World_Step(WorldId, FixedDeltaTime, SubSteps);
	++StepCount;

	// One-time heartbeat so logs (including headless runs) confirm stepping works.
	if (StepCount == 60)
	{
		UE_LOG(LogBox3D, Log, TEXT("Box3D stepping: 60 fixed steps done, %d awake bodies"),
			b3World_GetAwakeBodyCount(WorldId));
	}
}

void UBox3DWorldSubsystem::RegisterKinematicBody(UBox3DBodyComponent* Component)
{
	KinematicBodies.AddUnique(Component);
}

void UBox3DWorldSubsystem::UnregisterKinematicBody(UBox3DBodyComponent* Component)
{
	KinematicBodies.Remove(Component);
}

void UBox3DWorldSubsystem::PushKinematicTargets(float FixedDeltaTime)
{
	for (int32 Index = KinematicBodies.Num() - 1; Index >= 0; --Index)
	{
		const UBox3DBodyComponent* Component = KinematicBodies[Index].Get();
		if (Component == nullptr || !b3Body_IsValid(Component->GetBodyId()))
		{
			KinematicBodies.RemoveAtSwap(Index);
			continue;
		}

		const b3WorldTransform Target{ Box3D::ToB3Pos(Component->GetComponentLocation()),
									   Box3D::ToB3(Component->GetComponentQuat()) };
		b3Body_SetTargetTransform(Component->GetBodyId(), Target, FixedDeltaTime, /*wake*/ true);
	}
}

void UBox3DWorldSubsystem::AddPendingJoint(UBox3DJointComponent* Joint)
{
	PendingJoints.AddUnique(Joint);
}

void UBox3DWorldSubsystem::CreatePendingJoints()
{
	for (int32 Index = PendingJoints.Num() - 1; Index >= 0; --Index)
	{
		UBox3DJointComponent* Joint = PendingJoints[Index].Get();
		if (Joint == nullptr || Joint->TryCreateJoint())
		{
			PendingJoints.RemoveAtSwap(Index);
		}
		else if (Joint->CreateAttempts > 8)
		{
			UE_LOG(LogBox3D, Warning, TEXT("%s: giving up joint creation after %d attempts (bodies never appeared)"),
				*Joint->GetPathName(), Joint->CreateAttempts);
			PendingJoints.RemoveAtSwap(Index);
		}
	}
}

void UBox3DWorldSubsystem::PumpEvents()
{
	// Contact begin/end/hit. Shape ids in end events may reference destroyed
	// shapes; ResolveComponent validates before touching them.
	const b3ContactEvents Contacts = b3World_GetContactEvents(WorldId);
	for (int32 Index = 0; Index < Contacts.beginCount; ++Index)
	{
		const b3ContactBeginTouchEvent& Event = Contacts.beginEvents[Index];
		UBox3DBodyComponent* A = Box3D::ResolveComponent(Event.shapeIdA);
		UBox3DBodyComponent* B = Box3D::ResolveComponent(Event.shapeIdB);
		if (A != nullptr)
		{
			A->NotifyContactBegin(B);
		}
		if (B != nullptr)
		{
			B->NotifyContactBegin(A);
		}
	}
	for (int32 Index = 0; Index < Contacts.endCount; ++Index)
	{
		const b3ContactEndTouchEvent& Event = Contacts.endEvents[Index];
		UBox3DBodyComponent* A = Box3D::ResolveComponent(Event.shapeIdA);
		UBox3DBodyComponent* B = Box3D::ResolveComponent(Event.shapeIdB);
		if (A != nullptr)
		{
			A->NotifyContactEnd(B);
		}
		if (B != nullptr)
		{
			B->NotifyContactEnd(A);
		}
	}
	for (int32 Index = 0; Index < Contacts.hitCount; ++Index)
	{
		const b3ContactHitEvent& Event = Contacts.hitEvents[Index];
		UBox3DBodyComponent* A = Box3D::ResolveComponent(Event.shapeIdA);
		UBox3DBodyComponent* B = Box3D::ResolveComponent(Event.shapeIdB);
		const FVector Location = Box3D::ToUEPos(Event.point);
		const FVector Normal = Box3D::ToUEDir(Event.normal); // points from A to B
		const float ApproachSpeed = Event.approachSpeed * Box3D::MetersToUE;
		if (A != nullptr)
		{
			A->NotifyHit(B, Location, -Normal, ApproachSpeed);
		}
		if (B != nullptr)
		{
			B->NotifyHit(A, Location, Normal, ApproachSpeed);
		}
	}

	// Sensor overlaps, dispatched on the sensor body.
	const b3SensorEvents Sensors = b3World_GetSensorEvents(WorldId);
	for (int32 Index = 0; Index < Sensors.beginCount; ++Index)
	{
		const b3SensorBeginTouchEvent& Event = Sensors.beginEvents[Index];
		if (UBox3DBodyComponent* Sensor = Box3D::ResolveComponent(Event.sensorShapeId))
		{
			Sensor->NotifySensorBegin(Box3D::ResolveComponent(Event.visitorShapeId));
		}
	}
	for (int32 Index = 0; Index < Sensors.endCount; ++Index)
	{
		const b3SensorEndTouchEvent& Event = Sensors.endEvents[Index];
		if (UBox3DBodyComponent* Sensor = Box3D::ResolveComponent(Event.sensorShapeId))
		{
			Sensor->NotifySensorEnd(Box3D::ResolveComponent(Event.visitorShapeId));
		}
	}

	// Joints whose constraint force/torque exceeded their thresholds. Collect
	// first: handling may destroy joints, invalidating the event array.
	const b3JointEvents JointEvents = b3World_GetJointEvents(WorldId);
	TArray<UBox3DJointComponent*, TInlineAllocator<16>> ThresholdJoints;
	for (int32 Index = 0; Index < JointEvents.count; ++Index)
	{
		UBox3DJointComponent* Joint = static_cast<UBox3DJointComponent*>(JointEvents.jointEvents[Index].userData);
		if (Joint != nullptr && IsValid(Joint))
		{
			ThresholdJoints.Add(Joint);
		}
	}
	for (UBox3DJointComponent* Joint : ThresholdJoints)
	{
		Joint->HandleThresholdExceeded();
	}
}

void UBox3DWorldSubsystem::SyncMovedBodies()
{
	// Move events cover the last step only; called once after the final step of the
	// tick. userData is the owning component, kept valid because bodies are always
	// destroyed in the component's EndPlay. Only dynamic bodies sync back: static
	// and kinematic components drive their bodies, and writing the (lagging)
	// physics pose into an attached kinematic component would corrupt its relative
	// transform (e.g. a pawn proxy hanging off a capsule).
	const b3BodyEvents Events = b3World_GetBodyEvents(WorldId);
	for (int32 Index = 0; Index < Events.moveCount; ++Index)
	{
		const b3BodyMoveEvent& Move = Events.moveEvents[Index];
		UBox3DBodyComponent* Component = static_cast<UBox3DBodyComponent*>(Move.userData);
		if (Component != nullptr && IsValid(Component) && Component->BodyType == EBox3DBodyType::Dynamic)
		{
			Component->SyncTransformFromPhysics(Box3D::ToUEPos(Move.transform.p), Box3D::ToUE(Move.transform.q));
		}
	}
}

void UBox3DWorldSubsystem::RecordMovedBodies()
{
	const b3BodyEvents Events = b3World_GetBodyEvents(WorldId);
	for (int32 Index = 0; Index < Events.moveCount; ++Index)
	{
		const b3BodyMoveEvent& Move = Events.moveEvents[Index];
		UBox3DBodyComponent* Component = static_cast<UBox3DBodyComponent*>(Move.userData);
		if (Component == nullptr || !IsValid(Component) || Component->BodyType != EBox3DBodyType::Dynamic)
		{
			continue;
		}

		FInterpState& State = InterpStates.FindOrAdd(Component);
		if (State.Step == 0)
		{
			// New segment: start from wherever the component renders right now.
			State.P0 = Component->GetComponentLocation();
			State.Q0 = Component->GetComponentQuat();
		}
		else
		{
			State.P0 = State.P1;
			State.Q0 = State.Q1;
		}
		State.P1 = Box3D::ToUEPos(Move.transform.p);
		State.Q1 = Box3D::ToUE(Move.transform.q);
		State.Step = StepCount;
	}
}

void UBox3DWorldSubsystem::ApplyInterpolatedTransforms(float FixedDeltaTime)
{
	const float Alpha = FMath::Clamp(Accumulator / FixedDeltaTime, 0.0f, 1.0f);

	for (auto It = InterpStates.CreateIterator(); It; ++It)
	{
		UBox3DBodyComponent* Component = It.Key().Get();
		if (Component == nullptr || !IsValid(Component))
		{
			It.RemoveCurrent();
			continue;
		}

		const FInterpState& State = It.Value();
		if (State.Step == StepCount)
		{
			Component->SyncTransformFromPhysics(FMath::Lerp(State.P0, State.P1, Alpha),
				FQuat::Slerp(State.Q0, State.Q1, Alpha).GetNormalized());
		}
		else
		{
			// No move event in the latest step: the body settled. Land exactly on
			// the final pose and stop tracking it.
			Component->SyncTransformFromPhysics(State.P1, State.Q1);
			It.RemoveCurrent();
		}
	}
}

void UBox3DWorldSubsystem::InvalidateInterpolation(UBox3DBodyComponent* Component)
{
	InterpStates.Remove(Component);
}

FBox3DWorldStats UBox3DWorldSubsystem::GetWorldStats() const
{
	FBox3DWorldStats Stats;
	if (!b3World_IsValid(WorldId))
	{
		return Stats;
	}

	const b3Profile Profile = b3World_GetProfile(WorldId);
	const b3Counters Counters = b3World_GetCounters(WorldId);

	Stats.StepMs = Profile.step;
	Stats.PairsMs = Profile.pairs;
	Stats.CollideMs = Profile.collide;
	Stats.SolveMs = Profile.solve;
	Stats.BodyCount = Counters.bodyCount;
	Stats.ShapeCount = Counters.shapeCount;
	Stats.ContactCount = Counters.contactCount;
	Stats.JointCount = Counters.jointCount;
	Stats.IslandCount = Counters.islandCount;
	Stats.AwakeBodyCount = b3World_GetAwakeBodyCount(WorldId);
	Stats.TaskCount = Counters.taskCount;
	Stats.MemoryBytes = Counters.byteCount;
	return Stats;
}

void UBox3DWorldSubsystem::UpdateStats() const
{
#if STATS
	const b3Profile Profile = b3World_GetProfile(WorldId);
	const b3Counters Counters = b3World_GetCounters(WorldId);

	SET_FLOAT_STAT(STAT_Box3DStepMs, Profile.step);
	SET_FLOAT_STAT(STAT_Box3DPairsMs, Profile.pairs);
	SET_FLOAT_STAT(STAT_Box3DCollideMs, Profile.collide);
	SET_FLOAT_STAT(STAT_Box3DSolveMs, Profile.solve);
	SET_FLOAT_STAT(STAT_Box3DRefitMs, Profile.refit);
	SET_FLOAT_STAT(STAT_Box3DTransformsMs, Profile.transforms);
	SET_FLOAT_STAT(STAT_Box3DSensorsMs, Profile.sensors);
	SET_DWORD_STAT(STAT_Box3DBodies, Counters.bodyCount);
	SET_DWORD_STAT(STAT_Box3DAwakeBodies, b3World_GetAwakeBodyCount(WorldId));
	SET_DWORD_STAT(STAT_Box3DShapes, Counters.shapeCount);
	SET_DWORD_STAT(STAT_Box3DContacts, Counters.contactCount);
	SET_DWORD_STAT(STAT_Box3DJoints, Counters.jointCount);
	SET_DWORD_STAT(STAT_Box3DIslands, Counters.islandCount);
	SET_DWORD_STAT(STAT_Box3DTasks, Counters.taskCount);
	SET_MEMORY_STAT(STAT_Box3DMemory, Counters.byteCount);
#endif
}

void UBox3DWorldSubsystem::DrawDebugWorld() const
{
	DebugDrawer->Draw(GetWorld(), WorldId);
}

bool UBox3DWorldSubsystem::StartRecording()
{
	if (!b3World_IsValid(WorldId))
	{
		return false;
	}

	if (bRecordingActive)
	{
		b3World_StopRecording(WorldId);
	}
	if (Recording == nullptr)
	{
		Recording = b3CreateRecording(0);
	}

	// box3d resets the buffer, snapshots the world as the replay seed, then
	// records every mutation and step (with embedded state hashes) until stop.
	b3World_StartRecording(WorldId, Recording);
	bRecordingActive = true;
	UE_LOG(LogBox3D, Log, TEXT("Box3D recording started for %s"), *GetNameSafe(GetWorld()));
	return true;
}

bool UBox3DWorldSubsystem::StopRecording()
{
	if (!bRecordingActive || !b3World_IsValid(WorldId))
	{
		return false;
	}

	b3World_StopRecording(WorldId);
	bRecordingActive = false;
	UE_LOG(LogBox3D, Log, TEXT("Box3D recording stopped (%d bytes)"), b3Recording_GetSize(Recording));
	return true;
}

bool UBox3DWorldSubsystem::SaveRecordingToFile(const FString& Path) const
{
	if (Recording == nullptr || bRecordingActive || b3Recording_GetSize(Recording) <= 0)
	{
		UE_LOG(LogBox3D, Warning, TEXT("SaveRecordingToFile: no stopped recording to save"));
		return false;
	}

	// b3SaveRecordingToFile fopens the path directly; make sure the folder exists.
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree*/ true);
	const bool bSaved = b3SaveRecordingToFile(Recording, TCHAR_TO_UTF8(*Path));
	UE_LOG(LogBox3D, Log, TEXT("Box3D recording %s to %s"), bSaved ? TEXT("saved") : TEXT("FAILED to save"), *Path);
	return bSaved;
}

bool UBox3DWorldSubsystem::ValidateLastRecording() const
{
	if (Recording == nullptr || bRecordingActive || b3Recording_GetSize(Recording) <= 0)
	{
		UE_LOG(LogBox3D, Warning, TEXT("ValidateLastRecording: no stopped recording to validate"));
		return false;
	}

	const bool bValid = b3ValidateReplay(b3Recording_GetData(Recording), b3Recording_GetSize(Recording), 1);
	UE_LOG(LogBox3D, Log, TEXT("Box3D replay validation: %s"), bValid ? TEXT("deterministic") : TEXT("DIVERGED"));
	return bValid;
}

#if !UE_BUILD_SHIPPING

void UBox3DWorldSubsystem::SpawnSmokeBodies(int32 Count)
{
	if (!b3World_IsValid(WorldId))
	{
		return;
	}

	ClearSmokeBodies();

	// Place the test in front of the local player's view so it is visible in PIE.
	FVector ViewLocation = FVector::ZeroVector;
	FRotator ViewRotation = FRotator::ZeroRotator;
	if (const APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);
	}
	const FVector Forward = FRotator(0.0, ViewRotation.Yaw, 0.0).Vector();
	const FVector Center = ViewLocation + Forward * 600.0;

	// Static ground slab, 20 m x 20 m x 0.5 m, top surface 3 m below the view.
	SmokeGroundHalfExtentUE = FVector(1000.0, 1000.0, 25.0);
	SmokeGroundCenterUE = FVector(Center.X, Center.Y, ViewLocation.Z - 300.0 - SmokeGroundHalfExtentUE.Z);
	{
		b3BodyDef BodyDef = b3DefaultBodyDef();
		BodyDef.type = b3_staticBody;
		BodyDef.position = Box3D::ToB3Pos(SmokeGroundCenterUE);
		BodyDef.name = "SmokeGround";
		SmokeGroundId = b3CreateBody(WorldId, &BodyDef);

		const b3Vec3 HalfExtents = Box3D::ToB3(SmokeGroundHalfExtentUE);
		const b3BoxHull GroundHull = b3MakeBoxHull(HalfExtents.x, HalfExtents.y, HalfExtents.z);
		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		b3CreateHullShape(SmokeGroundId, &ShapeDef, &GroundHull.base);
	}

	// Dynamic bodies in a loose grid above the slab, alternating boxes and spheres.
	const int32 Columns = FMath::CeilToInt32(FMath::Sqrt(static_cast<float>(Count)));
	const double Spacing = 70.0;
	const double GridOffset = 0.5 * (Columns - 1) * Spacing;
	const FVector GroundTop = SmokeGroundCenterUE + FVector(0, 0, SmokeGroundHalfExtentUE.Z);

	SmokeBodies.Reserve(Count);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const int32 Col = Index % Columns;
		const int32 Row = (Index / Columns) % Columns;
		const int32 Layer = Index / (Columns * Columns);

		// Small jitter so stacks topple visibly instead of balancing forever.
		const FVector Position = GroundTop + FVector(
			Col * Spacing - GridOffset + FMath::FRandRange(-3.0f, 3.0f),
			Row * Spacing - GridOffset + FMath::FRandRange(-3.0f, 3.0f),
			200.0 + Layer * 120.0);

		b3BodyDef BodyDef = b3DefaultBodyDef();
		BodyDef.type = b3_dynamicBody;
		BodyDef.position = Box3D::ToB3Pos(Position);
		BodyDef.rotation = Box3D::ToB3(FRotator(FMath::FRandRange(0.f, 30.f), FMath::FRandRange(0.f, 360.f), 0.f).Quaternion());

		FSmokeBody Smoke;
		Smoke.Id = b3CreateBody(WorldId, &BodyDef);

		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		if (Index % 2 == 0)
		{
			Smoke.HalfExtentUE = FVector(25.0);
			const b3Vec3 HalfExtents = Box3D::ToB3(Smoke.HalfExtentUE);
			const b3BoxHull Hull = b3MakeBoxHull(HalfExtents.x, HalfExtents.y, HalfExtents.z);
			b3CreateHullShape(Smoke.Id, &ShapeDef, &Hull.base);
		}
		else
		{
			Smoke.RadiusUE = 25.0f;
			const b3Sphere Sphere{ b3Vec3{ 0.0f, 0.0f, 0.0f }, Smoke.RadiusUE * Box3D::UEToMeters };
			b3CreateSphereShape(Smoke.Id, &ShapeDef, &Sphere);
		}

		SmokeBodies.Add(Smoke);
	}

	UE_LOG(LogBox3D, Log, TEXT("Smoke test: spawned %d bodies over a %s slab at %s"),
		Count, *SmokeGroundHalfExtentUE.ToCompactString(), *SmokeGroundCenterUE.ToCompactString());
}

void UBox3DWorldSubsystem::ClearSmokeBodies()
{
	for (const FSmokeBody& Smoke : SmokeBodies)
	{
		if (b3Body_IsValid(Smoke.Id))
		{
			b3DestroyBody(Smoke.Id);
		}
	}
	SmokeBodies.Empty();

	if (b3Body_IsValid(SmokeGroundId))
	{
		b3DestroyBody(SmokeGroundId);
	}
	SmokeGroundId = b3BodyId{};
}

void UBox3DWorldSubsystem::DrawSmokeBodies() const
{
#if ENABLE_DRAW_DEBUG
	if (SmokeBodies.IsEmpty())
	{
		return;
	}

	UWorld* World = GetWorld();
	DrawDebugBox(World, SmokeGroundCenterUE, SmokeGroundHalfExtentUE, FQuat::Identity, FColor::Silver);

	for (const FSmokeBody& Smoke : SmokeBodies)
	{
		if (!b3Body_IsValid(Smoke.Id))
		{
			continue;
		}

		const b3WorldTransform Transform = b3Body_GetTransform(Smoke.Id);
		const FVector Position = Box3D::ToUEPos(Transform.p);
		const FQuat Rotation = Box3D::ToUE(Transform.q);
		const FColor Color = b3Body_IsAwake(Smoke.Id) ? FColor::Green : FColor::Cyan;

		if (Smoke.RadiusUE > 0.0f)
		{
			DrawDebugSphere(World, Position, Smoke.RadiusUE, 12, Color);
		}
		else
		{
			DrawDebugBox(World, Position, Smoke.HalfExtentUE, Rotation, Color);
		}
	}
#endif
}

#endif // !UE_BUILD_SHIPPING
