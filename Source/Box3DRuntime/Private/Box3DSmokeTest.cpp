// Console hooks for smoke tests:
//   box3d.Smoke [Count]        (M0) debug-drawn raw bodies, no actors involved
//   box3d.SmokeActors [Count]  (M1) real actors with UBox3DBodyComponent roots and
//                              rendered meshes, proving the component sync pipeline
//   box3d.AutoSmokeActors N    CVar: run SmokeActors N automatically when a game
//                              world starts, then log settle state ~9s later.
//                              Enables headless verification via
//                              -ExecCmds="box3d.AutoSmokeActors 12".

#include "Box3DBodyComponent.h"
#include "Box3DQueryLibrary.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "TimerManager.h"

#if !UE_BUILD_SHIPPING

namespace
{
	TArray<TWeakObjectPtr<AActor>> GSmokeActors;

	AActor* SpawnBox3DMeshActor(UWorld* World, UStaticMesh* Mesh, const FTransform& Transform,
		const FVector& MeshScale, EBox3DBodyType BodyType, EBox3DShapeType ShapeType)
	{
		AActor* Actor = World->SpawnActor<AActor>();
		if (Actor == nullptr)
		{
			return nullptr;
		}

		UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(Actor, TEXT("Box3DBody"));
		Body->BodyType = BodyType;
		Body->ShapeType = ShapeType;
		Actor->SetRootComponent(Body);
		Body->SetWorldTransform(Transform);

		UStaticMeshComponent* MeshComponent = NewObject<UStaticMeshComponent>(Actor, TEXT("Mesh"));
		MeshComponent->SetupAttachment(Body);
		MeshComponent->SetStaticMesh(Mesh);
		MeshComponent->SetRelativeScale3D(MeshScale);
		MeshComponent->SetMobility(EComponentMobility::Movable);
		// Box3D owns collision here; keep Chaos entirely out of it.
		MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		// The mesh registers first so the body's BeginPlay (fired by its own
		// RegisterComponent below) can auto-fit the shape from the mesh bounds.
		MeshComponent->RegisterComponent();
		Body->RegisterComponent();
		return Actor;
	}

	void ClearSmokeActors()
	{
		int32 Destroyed = 0;
		for (const TWeakObjectPtr<AActor>& Actor : GSmokeActors)
		{
			if (AActor* Live = Actor.Get())
			{
				Live->Destroy();
				++Destroyed;
			}
		}
		GSmokeActors.Empty();
		UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: destroyed %d actors"), Destroyed);
	}

	void SpawnSmokeActors(UWorld* World, int32 Count)
	{
		Count = FMath::Min(Count, 1024);

		UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
		UStaticMesh* CylinderMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
		if (CubeMesh == nullptr || SphereMesh == nullptr || CylinderMesh == nullptr)
		{
			UE_LOG(LogBox3D, Error, TEXT("box3d.SmokeActors: engine basic shape meshes not found"));
			return;
		}

		FVector ViewLocation = FVector::ZeroVector;
		FRotator ViewRotation = FRotator::ZeroRotator;
		if (const APlayerController* PC = World->GetFirstPlayerController())
		{
			PC->GetPlayerViewPoint(ViewLocation, ViewRotation);
		}
		const FVector Forward = FRotator(0.0, ViewRotation.Yaw, 0.0).Vector();
		const FVector Center = ViewLocation + Forward * 600.0;

		// Static ground slab: 20 m x 20 m x 0.5 m cube, top 3 m below the view.
		// TriangleMesh on purpose: exercises the mesh cooker with non-uniform scale.
		const FVector GroundScale(20.0, 20.0, 0.5);
		const FVector GroundCenter(Center.X, Center.Y, ViewLocation.Z - 300.0 - 25.0);
		if (AActor* Ground = SpawnBox3DMeshActor(World, CubeMesh, FTransform(GroundCenter), GroundScale,
			EBox3DBodyType::Static, EBox3DShapeType::TriangleMesh))
		{
			GSmokeActors.Add(Ground);
		}

		// Dynamic bodies: 50 cm cubes, spheres, and convex-hull cylinders in a
		// jittered grid above the slab.
		const int32 Columns = FMath::CeilToInt32(FMath::Sqrt(static_cast<float>(Count)));
		const double Spacing = 70.0;
		const double GridOffset = 0.5 * (Columns - 1) * Spacing;
		const double GroundTopZ = GroundCenter.Z + 25.0;

		for (int32 Index = 0; Index < Count; ++Index)
		{
			const int32 Col = Index % Columns;
			const int32 Row = (Index / Columns) % Columns;
			const int32 Layer = Index / (Columns * Columns);

			const FVector Position(
				Center.X + (Col * Spacing - GridOffset) + FMath::FRandRange(-3.0f, 3.0f),
				Center.Y + (Row * Spacing - GridOffset) + FMath::FRandRange(-3.0f, 3.0f),
				GroundTopZ + 200.0 + Layer * 120.0);
			const FQuat Rotation = FRotator(FMath::FRandRange(0.f, 30.f), FMath::FRandRange(0.f, 360.f), 0.f).Quaternion();

			UStaticMesh* Mesh = CubeMesh;
			EBox3DShapeType Shape = EBox3DShapeType::Box;
			if (Index % 3 == 1)
			{
				Mesh = SphereMesh;
				Shape = EBox3DShapeType::Sphere;
			}
			else if (Index % 3 == 2)
			{
				Mesh = CylinderMesh;
				Shape = EBox3DShapeType::ConvexHull;
			}
			if (AActor* Actor = SpawnBox3DMeshActor(World, Mesh, FTransform(Rotation, Position), FVector(0.5),
				EBox3DBodyType::Dynamic, Shape))
			{
				GSmokeActors.Add(Actor);
			}
		}

		UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: spawned %d body actors + ground (top Z=%.0f)"),
			Count, GroundTopZ);
	}

	void LogSmokeActorState(UWorld* World)
	{
		int32 Simulating = 0;
		int32 Awake = 0;
		int32 Logged = 0;
		const AActor* SampleActor = nullptr;
		for (int32 Index = 1; Index < GSmokeActors.Num(); ++Index) // skip ground at [0]
		{
			const AActor* Actor = GSmokeActors[Index].Get();
			const UBox3DBodyComponent* Body = Actor ? Actor->FindComponentByClass<UBox3DBodyComponent>() : nullptr;
			if (Body == nullptr || !Body->IsSimulating())
			{
				continue;
			}
			++Simulating;
			Awake += Body->IsAwake() ? 1 : 0;
			if (Logged < 3)
			{
				UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: sample body %d at Z=%.1f (awake=%d, mass=%.1f kg)"),
					Index, Actor->GetActorLocation().Z, Body->IsAwake() ? 1 : 0, Body->GetMass());
				++Logged;
			}
			if (SampleActor == nullptr)
			{
				SampleActor = Actor;
			}
		}
		UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: settle check — %d simulating, %d awake"), Simulating, Awake);

		if (World == nullptr || SampleActor == nullptr)
		{
			return;
		}

		// Query self-test against the settled scene (M2).
		const FVector SampleLocation = SampleActor->GetActorLocation();
		const FBox3DQueryFilter Filter;

		FBox3DHitResult Hit;
		const bool bRayHit = UBox3DQueryLibrary::Box3DRayCast(World,
			SampleLocation + FVector(0, 0, 300), SampleLocation - FVector(0, 0, 300), Filter, Hit);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: query check — ray %s at Z=%.1f on %s"),
			bRayHit ? TEXT("hit") : TEXT("MISSED"), Hit.Location.Z, *GetNameSafe(Hit.Actor));

		// Clear of the grid, straight onto the triangle-mesh ground: exact top Z
		// proves mesh cooking units and non-uniform scale.
		FBox3DHitResult GroundHit;
		const bool bGroundHit = UBox3DQueryLibrary::Box3DRayCast(World,
			SampleLocation + FVector(600, 600, 300), SampleLocation + FVector(600, 600, -300), Filter, GroundHit);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: query check — mesh ground ray %s at Z=%.1f (tri=%d)"),
			bGroundHit ? TEXT("hit") : TEXT("MISSED"), GroundHit.Location.Z, GroundHit.TriangleIndex);

		const TArray<UBox3DBodyComponent*> Overlaps = UBox3DQueryLibrary::Box3DOverlapSphere(World,
			SampleLocation, 200.0f, Filter);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: query check — overlap sphere r=200 found %d bodies"),
			Overlaps.Num());

		const float MoverFraction = UBox3DQueryLibrary::Box3DCastMover(World,
			SampleLocation + FVector(0, 0, 400), FVector(0, 0, -600), 30.0f, 90.0f, Filter);
		int32 PlaneCount = 0;
		const FVector SolvedDelta = UBox3DQueryLibrary::Box3DSolveMoverDelta(World,
			SampleLocation + FVector(0, 0, 60), 30.0f, 90.0f, FVector(0, 0, -50), Filter, PlaneCount);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SmokeActors: query check — mover cast fraction=%.2f, solve planes=%d delta=(%.1f %.1f %.1f)"),
			MoverFraction, PlaneCount, SolvedDelta.X, SolvedDelta.Y, SolvedDelta.Z);
	}

	TAutoConsoleVariable<int32> CVarAutoSmokeActors(
		TEXT("box3d.AutoSmokeActors"), 0,
		TEXT("If > 0, box3d.SmokeActors runs with this count when a game world starts (for headless testing)."));

	FTimerHandle GAutoSmokeSpawnTimer;
	FTimerHandle GAutoSmokeLogTimer;

	// Static registration at module load; fires after a game world's actors initialize.
	struct FAutoSmokeRegistrar
	{
		FAutoSmokeRegistrar()
		{
			FWorldDelegates::OnWorldInitializedActors.AddLambda([](const FActorsInitializedParams& Params)
			{
				UWorld* World = Params.World;
				if (World == nullptr || !World->IsGameWorld())
				{
					return;
				}

				// The CVar is read inside the timer, not here: -ExecCmds from the
				// command line executes after the initial map load, so at this point
				// it may not be applied yet. The delay also lets the player
				// controller spawn so placement in front of the view works.
				World->GetTimerManager().SetTimer(GAutoSmokeSpawnTimer,
					FTimerDelegate::CreateLambda([World]
					{
						const int32 Count = CVarAutoSmokeActors.GetValueOnGameThread();
						if (Count <= 0)
						{
							return;
						}
						SpawnSmokeActors(World, Count);
						World->GetTimerManager().SetTimer(GAutoSmokeLogTimer,
							FTimerDelegate::CreateLambda([World] { LogSmokeActorState(World); }), 8.0f, false);
					}), 1.0f, false);
			});
		}
	};
	FAutoSmokeRegistrar GAutoSmokeRegistrar;
}

static FAutoConsoleCommandWithWorldAndArgs GBox3DSmokeCommand(
	TEXT("box3d.Smoke"),
	TEXT("Spawn falling Box3D test bodies with debug drawing. Usage: box3d.Smoke [Count=32]; 0 clears."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		if (World == nullptr)
		{
			return;
		}

		UBox3DWorldSubsystem* Subsystem = World->GetSubsystem<UBox3DWorldSubsystem>();
		if (Subsystem == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.Smoke: no Box3D world subsystem in %s (game/PIE worlds only)"),
				*GetNameSafe(World));
			return;
		}

		int32 Count = 32;
		if (Args.Num() > 0)
		{
			Count = FCString::Atoi(*Args[0]);
		}

		if (Count <= 0)
		{
			Subsystem->ClearSmokeBodies();
			UE_LOG(LogBox3D, Log, TEXT("box3d.Smoke: cleared"));
		}
		else
		{
			Subsystem->SpawnSmokeBodies(FMath::Min(Count, 4096));
		}
	}));

static FAutoConsoleCommandWithWorldAndArgs GBox3DSmokeActorsCommand(
	TEXT("box3d.SmokeActors"),
	TEXT("Spawn actors whose root is a UBox3DBodyComponent with a rendered mesh. Usage: box3d.SmokeActors [Count=16]; 0 clears."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		if (World == nullptr)
		{
			return;
		}

		int32 Count = 16;
		if (Args.Num() > 0)
		{
			Count = FCString::Atoi(*Args[0]);
		}

		if (Count <= 0)
		{
			ClearSmokeActors();
		}
		else
		{
			SpawnSmokeActors(World, Count);
		}
	}));

#endif // !UE_BUILD_SHIPPING
