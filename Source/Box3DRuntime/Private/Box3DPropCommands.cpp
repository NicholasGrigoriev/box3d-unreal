// Console hooks for the static mesh -> physics prop pipeline:
//   box3d.MakeProp              convert the static mesh actor (or ISM instance)
//                               under the player crosshair into an ABox3DPropActor
//   box3d.MakeProp all [Radius] convert every AStaticMeshActor within Radius cm of
//                               the view (max 200)
//   box3d.AutoDropProps N       CVar: drop N prop cubes above the player shortly
//                               after world start and log a settle line ~8s later.
//                               Pairs with the static scene mirror for headless
//                               verification on real maps:
//                               -ExecCmds="box3d.AutoDropProps 24"

#include "Box3DBodyComponent.h"
#include "Box3DPropActor.h"
#include "Box3DPropConversion.h"
#include "Box3DQueryLibrary.h"
#include "Box3DRuntime.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "Misc/App.h"
#include "TimerManager.h"

#if !UE_BUILD_SHIPPING

namespace
{
	bool GetPlayerView(UWorld* World, FVector& OutLocation, FRotator& OutRotation)
	{
		if (const APlayerController* PC = World->GetFirstPlayerController())
		{
			PC->GetPlayerViewPoint(OutLocation, OutRotation);
			return true;
		}
		return false;
	}

	void MakePropUnderCrosshair(UWorld* World)
	{
		FVector ViewLocation;
		FRotator ViewRotation;
		if (!GetPlayerView(World, ViewLocation, ViewRotation))
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.MakeProp: no player view"));
			return;
		}

		// Chaos trace on purpose: it resolves to actors/components (and ISM instance
		// indices via FHitResult::Item), which the Box3D mirror does not.
		FHitResult Hit;
		const FVector End = ViewLocation + ViewRotation.Vector() * 5000.0;
		if (!World->LineTraceSingleByChannel(Hit, ViewLocation, End, ECC_Visibility))
		{
			UE_LOG(LogBox3D, Log, TEXT("box3d.MakeProp: nothing under the crosshair within 50 m"));
			return;
		}

		if (UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(Hit.GetComponent()))
		{
			if (Hit.Item >= 0)
			{
				Box3D::ConvertInstanceToProp(Ism, Hit.Item);
				return;
			}
		}
		if (AStaticMeshActor* MeshActor = Cast<AStaticMeshActor>(Hit.GetActor()))
		{
			Box3D::ConvertToProp(MeshActor);
			return;
		}

		UE_LOG(LogBox3D, Log, TEXT("box3d.MakeProp: hit %s (%s) — only AStaticMeshActor and ISM instances convert"),
			*GetNameSafe(Hit.GetActor()), *GetNameSafe(Hit.GetComponent()));
	}

	void MakePropsInRadius(UWorld* World, float Radius)
	{
		FVector ViewLocation;
		FRotator ViewRotation;
		if (!GetPlayerView(World, ViewLocation, ViewRotation))
		{
			return;
		}

		constexpr int32 MaxConversions = 200;
		int32 Converted = 0;
		for (TActorIterator<AStaticMeshActor> It(World); It && Converted < MaxConversions; ++It)
		{
			AStaticMeshActor* Actor = *It;
			const UStaticMeshComponent* MeshComponent = Actor->GetStaticMeshComponent();
			if (MeshComponent == nullptr || MeshComponent->GetStaticMesh() == nullptr ||
				MeshComponent->GetCollisionEnabled() == ECollisionEnabled::NoCollision ||
				FVector::DistSquared(Actor->GetActorLocation(), ViewLocation) > FMath::Square(Radius))
			{
				continue;
			}
			if (Box3D::ConvertToProp(Actor) != nullptr)
			{
				++Converted;
			}
		}
		UE_LOG(LogBox3D, Log, TEXT("box3d.MakeProp all: converted %d actors within %.0f cm"), Converted, Radius);
	}

	TArray<TWeakObjectPtr<ABox3DPropActor>> GDroppedProps;

	void SpawnDropProps(UWorld* World, int32 Count)
	{
		UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (CubeMesh == nullptr)
		{
			UE_LOG(LogBox3D, Error, TEXT("box3d.AutoDropProps: engine cube mesh not found"));
			return;
		}

		FVector ViewLocation = FVector::ZeroVector;
		FRotator ViewRotation = FRotator::ZeroRotator;
		GetPlayerView(World, ViewLocation, ViewRotation);
		const FVector Forward = FRotator(0.0, ViewRotation.Yaw, 0.0).Vector();
		const FVector Center = ViewLocation + Forward * 400.0 + FVector(0, 0, 300.0);

		// No synthetic ground here on purpose: the mirrored map floor is the test bed.
		const int32 Columns = FMath::CeilToInt32(FMath::Sqrt(static_cast<float>(Count)));
		const double Spacing = 60.0;
		const double GridOffset = 0.5 * (Columns - 1) * Spacing;

		int32 Spawned = 0;
		for (int32 Index = 0; Index < Count; ++Index)
		{
			const FVector Position = Center + FVector(
				(Index % Columns) * Spacing - GridOffset + FMath::FRandRange(-3.0f, 3.0f),
				((Index / Columns) % Columns) * Spacing - GridOffset + FMath::FRandRange(-3.0f, 3.0f),
				(Index / (Columns * Columns)) * 100.0);

			const FTransform Transform(FQuat::Identity, Position, FVector(0.4));
			if (ABox3DPropActor* Prop = World->SpawnActorDeferred<ABox3DPropActor>(
					ABox3DPropActor::StaticClass(), Transform))
			{
				Prop->SetStaticMesh(CubeMesh);
				Prop->FinishSpawning(Transform);
				GDroppedProps.Add(Prop);
				++Spawned;
			}
		}
		UE_LOG(LogBox3D, Log, TEXT("box3d.AutoDropProps: dropped %d prop cubes around %s"),
			Spawned, *Center.ToCompactString());
	}

	void LogDropPropState(UWorld* World)
	{
		int32 Simulating = 0;
		int32 Awake = 0;
		const ABox3DPropActor* Sample = nullptr;
		for (const TWeakObjectPtr<ABox3DPropActor>& Weak : GDroppedProps)
		{
			const ABox3DPropActor* Prop = Weak.Get();
			const UBox3DBodyComponent* Body = Prop ? Prop->GetBody() : nullptr;
			if (Body == nullptr || !Body->IsSimulating())
			{
				continue;
			}
			++Simulating;
			Awake += Body->IsAwake() ? 1 : 0;
			Sample = Sample ? Sample : Prop;
		}

		float SampleZ = 0.0f;
		float FloorZ = -99999.0f;
		if (Sample != nullptr)
		{
			SampleZ = Sample->GetActorLocation().Z;
			// Straight down past the prop: proves it rests on mirrored map geometry.
			FBox3DHitResult FloorHit;
			if (UBox3DQueryLibrary::Box3DRayCast(World,
					Sample->GetActorLocation() + FVector(0, 0, -50.0),
					Sample->GetActorLocation() - FVector(0, 0, 5000.0), FBox3DQueryFilter{}, FloorHit))
			{
				FloorZ = FloorHit.Location.Z;
			}
		}

		UE_LOG(LogBox3D, Log, TEXT("box3d.AutoDropProps: settle check — %d simulating, %d awake, sample Z=%.1f, floor ray hit Z=%.1f"),
			Simulating, Awake, SampleZ, FloorZ);

		if (FApp::IsUnattended() && GEngine != nullptr)
		{
			GEngine->Exec(World, TEXT("QUIT"));
		}
	}

	TAutoConsoleVariable<int32> CVarAutoDropProps(
		TEXT("box3d.AutoDropProps"), 0,
		TEXT("If > 0, drop this many Box3D prop cubes above the player when a game world starts, "
			 "log a settle line ~8s later, and quit when running -unattended."));

	FTimerHandle GAutoDropSpawnTimer;
	FTimerHandle GAutoDropLogTimer;

	// Static registration at module load; same deferred-CVar pattern as AutoSmokeActors
	// (-ExecCmds applies after the initial map load, so the value is read in the timer).
	struct FAutoDropPropsRegistrar
	{
		FAutoDropPropsRegistrar()
		{
			FWorldDelegates::OnWorldInitializedActors.AddLambda([](const FActorsInitializedParams& Params)
			{
				UWorld* World = Params.World;
				if (World == nullptr || !World->IsGameWorld())
				{
					return;
				}

				World->GetTimerManager().SetTimer(GAutoDropSpawnTimer,
					FTimerDelegate::CreateLambda([World]
					{
						const int32 Count = CVarAutoDropProps.GetValueOnGameThread();
						if (Count <= 0)
						{
							return;
						}
						GDroppedProps.Empty();
						SpawnDropProps(World, FMath::Min(Count, 512));
						World->GetTimerManager().SetTimer(GAutoDropLogTimer,
							FTimerDelegate::CreateLambda([World] { LogDropPropState(World); }), 8.0f, false);
					}), 1.0f, false);
			});
		}
	};
	FAutoDropPropsRegistrar GAutoDropPropsRegistrar;
}

static FAutoConsoleCommandWithWorldAndArgs GBox3DMakePropCommand(
	TEXT("box3d.MakeProp"),
	TEXT("Convert static meshes into Box3D physics props. Usage: box3d.MakeProp (crosshair target) | box3d.MakeProp all [Radius=1500]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		if (World == nullptr)
		{
			return;
		}

		if (Args.Num() > 0 && Args[0].Equals(TEXT("all"), ESearchCase::IgnoreCase))
		{
			const float Radius = Args.Num() > 1 ? FCString::Atof(*Args[1]) : 1500.0f;
			MakePropsInRadius(World, FMath::Clamp(Radius, 100.0f, 20000.0f));
		}
		else
		{
			MakePropUnderCrosshair(World);
		}
	}));

#endif // !UE_BUILD_SHIPPING
