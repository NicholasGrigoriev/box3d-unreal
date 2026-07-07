// Console hooks for the rope/cloth soft-body actors:
//   box3d.SpawnRope [Length=400] [Segments=20]  hang a rope from the surface
//                                               under the crosshair
//   box3d.SpawnCloth [Width=200] [Height=200]   hang a cloth sheet there,
//                                               facing the player

#include "Box3DClothActor.h"
#include "Box3DRopeActor.h"
#include "Box3DRuntime.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"

#if !UE_BUILD_SHIPPING

namespace
{
	/// Crosshair surface point (Chaos visibility trace), or a spot 4 m ahead.
	bool GetSpawnPoint(UWorld* World, FVector& OutLocation, FRotator& OutViewRotation)
	{
		const APlayerController* PC = World->GetFirstPlayerController();
		if (PC == nullptr)
		{
			return false;
		}
		FVector ViewLocation;
		PC->GetPlayerViewPoint(ViewLocation, OutViewRotation);

		FHitResult Hit;
		const FVector End = ViewLocation + OutViewRotation.Vector() * 5000.0;
		OutLocation = World->LineTraceSingleByChannel(Hit, ViewLocation, End, ECC_Visibility)
			? Hit.ImpactPoint
			: ViewLocation + OutViewRotation.Vector() * 400.0;
		return true;
	}
}

static FAutoConsoleCommandWithWorldAndArgs GBox3DSpawnRopeCommand(
	TEXT("box3d.SpawnRope"),
	TEXT("Hang a Box3D rope from the point under the crosshair. Usage: box3d.SpawnRope [Length=400] [Segments=20]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		FVector SpawnPoint;
		FRotator ViewRotation;
		if (World == nullptr || !GetSpawnPoint(World, SpawnPoint, ViewRotation))
		{
			return;
		}

		const FTransform Transform(FQuat::Identity, SpawnPoint);
		ABox3DRopeActor* Rope = World->SpawnActorDeferred<ABox3DRopeActor>(ABox3DRopeActor::StaticClass(), Transform);
		if (Rope == nullptr)
		{
			return;
		}
		Rope->RopeLength = Args.Num() > 0 ? FMath::Clamp(FCString::Atof(*Args[0]), 50.0f, 5000.0f) : 400.0f;
		Rope->NumSegments = Args.Num() > 1 ? FCString::Atoi(*Args[1]) : 20;
		Rope->FinishSpawning(Transform);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SpawnRope: %.0f cm, %d segments at %s"),
			Rope->RopeLength, Rope->GetBodyCount(), *SpawnPoint.ToCompactString());
	}));

static FAutoConsoleCommandWithWorldAndArgs GBox3DSpawnClothCommand(
	TEXT("box3d.SpawnCloth"),
	TEXT("Hang a Box3D cloth sheet at the point under the crosshair, facing the player. Usage: box3d.SpawnCloth [Width=200] [Height=200]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		FVector SpawnPoint;
		FRotator ViewRotation;
		if (World == nullptr || !GetSpawnPoint(World, SpawnPoint, ViewRotation))
		{
			return;
		}

		// Local X spans the width; yaw it perpendicular to the view so the
		// sheet faces the player.
		const FTransform Transform(FRotator(0.0, ViewRotation.Yaw + 90.0, 0.0), SpawnPoint);
		ABox3DClothActor* Cloth = World->SpawnActorDeferred<ABox3DClothActor>(ABox3DClothActor::StaticClass(), Transform);
		if (Cloth == nullptr)
		{
			return;
		}
		Cloth->Width = Args.Num() > 0 ? FMath::Clamp(FCString::Atof(*Args[0]), 50.0f, 1000.0f) : 200.0f;
		Cloth->Height = Args.Num() > 1 ? FMath::Clamp(FCString::Atof(*Args[1]), 50.0f, 1000.0f) : 200.0f;
		Cloth->FinishSpawning(Transform);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SpawnCloth: %.0fx%.0f cm, %d particles at %s"),
			Cloth->Width, Cloth->Height, Cloth->GetParticleCount(), *SpawnPoint.ToCompactString());
	}));

#endif // !UE_BUILD_SHIPPING
