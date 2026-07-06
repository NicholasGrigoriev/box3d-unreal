#include "Box3DWorldSubsystem.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DSettings.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

void UBox3DWorldSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	const UBox3DSettings* Settings = GetDefault<UBox3DSettings>();

	b3WorldDef Def = b3DefaultWorldDef();
	Def.gravity = Box3D::ToB3Accel(Settings->Gravity);
	Def.enableSleep = Settings->bEnableSleep;
	Def.enableContinuous = Settings->bEnableContinuous;
	Def.workerCount = static_cast<uint32>(Settings->WorkerCount);

	WorldId = b3CreateWorld(&Def);
	Accumulator = 0.0f;
	StepCount = 0;

	UE_LOG(LogBox3D, Log, TEXT("Box3D world created for %s (gravity=%s cm/s^2, workers=%d)"),
		*GetNameSafe(GetWorld()), *Settings->Gravity.ToCompactString(), Settings->WorkerCount);
}

void UBox3DWorldSubsystem::Deinitialize()
{
#if !UE_BUILD_SHIPPING
	ClearSmokeBodies();
#endif

	if (b3World_IsValid(WorldId))
	{
		b3DestroyWorld(WorldId);
		UE_LOG(LogBox3D, Log, TEXT("Box3D world destroyed for %s after %llu steps"),
			*GetNameSafe(GetWorld()), StepCount);
	}
	WorldId = b3WorldId{};

	Super::Deinitialize();
}

bool UBox3DWorldSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UBox3DWorldSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!b3World_IsValid(WorldId))
	{
		return;
	}

	const UBox3DSettings* Settings = GetDefault<UBox3DSettings>();
	const float FixedDt = Settings->FixedTimeStep;

	// Fixed-step accumulator; drop time beyond the per-frame budget so a hitch
	// cannot snowball into ever-longer frames.
	Accumulator = FMath::Min(Accumulator + DeltaTime, Settings->MaxStepsPerTick * FixedDt);
	bool bStepped = false;
	while (Accumulator >= FixedDt)
	{
		PushKinematicTargets(FixedDt);
		StepFixed(FixedDt, Settings->SubStepCount);
		Accumulator -= FixedDt;
		bStepped = true;
	}

	if (bStepped)
	{
		SyncMovedBodies();
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

void UBox3DWorldSubsystem::SyncMovedBodies()
{
	// Move events cover the last step only; called once after the final step of the
	// tick. userData is the owning component, kept valid because bodies are always
	// destroyed in the component's EndPlay.
	const b3BodyEvents Events = b3World_GetBodyEvents(WorldId);
	for (int32 Index = 0; Index < Events.moveCount; ++Index)
	{
		const b3BodyMoveEvent& Move = Events.moveEvents[Index];
		UBox3DBodyComponent* Component = static_cast<UBox3DBodyComponent*>(Move.userData);
		if (Component != nullptr && IsValid(Component))
		{
			Component->SyncTransformFromPhysics(Box3D::ToUEPos(Move.transform.p), Box3D::ToUE(Move.transform.q));
		}
	}
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
