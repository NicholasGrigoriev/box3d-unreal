// Console hooks for the special-actor zoo:
//   box3d.SpawnRope [Length=400] [Segments=20]   hang a rope from the surface
//                                                under the crosshair
//   box3d.SpawnCloth [Width=200] [Height=200]    hang a cloth sheet there,
//                                                facing the player
//   box3d.SpawnBreakable [Cols=4] [Layers=3]     welded cube wall at the
//                                                crosshair — shoot it apart
//   box3d.SpawnWind [mode=directional] [Speed]   wind source at the crosshair
//                                                (directional|turbulence|vortex)
//   box3d.SpawnLiquid [Rate=120] [Max=400]       liquid tap pouring onto the
//                                                point under the crosshair

#include "Box3DBreakableActor.h"
#include "Box3DClothActor.h"
#include "Box3DLiquidSourceActor.h"
#include "Box3DRopeActor.h"
#include "Box3DRuntime.h"
#include "Box3DWindActor.h"
#include "Engine/StaticMesh.h"
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

static FAutoConsoleCommandWithWorldAndArgs GBox3DSpawnBreakableCommand(
	TEXT("box3d.SpawnBreakable"),
	TEXT("Build a welded wall of cube chunks at the point under the crosshair. Usage: box3d.SpawnBreakable [Cols=4] [Layers=3]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		FVector SpawnPoint;
		FRotator ViewRotation;
		if (World == nullptr || !GetSpawnPoint(World, SpawnPoint, ViewRotation))
		{
			return;
		}
		UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (CubeMesh == nullptr)
		{
			return;
		}

		const int32 Cols = FMath::Clamp(Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 4, 1, 10);
		const int32 Layers = FMath::Clamp(Args.Num() > 1 ? FCString::Atoi(*Args[1]) : 3, 1, 10);
		constexpr float ChunkSize = 40.0f; // engine cube (100 cm) at 0.4 scale

		// Wall faces the player: chunks spread along the view-right axis.
		const FTransform Transform(FRotator(0.0, ViewRotation.Yaw + 90.0, 0.0), SpawnPoint);
		ABox3DBreakableActor* Breakable = World->SpawnActorDeferred<ABox3DBreakableActor>(
			ABox3DBreakableActor::StaticClass(), Transform);
		if (Breakable == nullptr)
		{
			return;
		}
		for (int32 Layer = 0; Layer < Layers; ++Layer)
		{
			for (int32 Col = 0; Col < Cols; ++Col)
			{
				const FVector Local(
					(Col - 0.5f * (Cols - 1)) * ChunkSize, 0.0,
					Layer * ChunkSize + 0.5f * ChunkSize);
				Breakable->AddChunk(CubeMesh, FTransform(FQuat::Identity, Local, FVector(0.4)));
			}
		}
		Breakable->FinishSpawning(Transform);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SpawnBreakable: %d chunks, %d welds at %s"),
			Breakable->GetChunkCount(), Breakable->GetLiveWeldCount(), *SpawnPoint.ToCompactString());
	}));

static FAutoConsoleCommandWithWorldAndArgs GBox3DSpawnWindCommand(
	TEXT("box3d.SpawnWind"),
	TEXT("Drop a Box3D wind source at the point under the crosshair, blowing away from you. Usage: box3d.SpawnWind [directional|turbulence|vortex] [Speed=800]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		FVector SpawnPoint;
		FRotator ViewRotation;
		if (World == nullptr || !GetSpawnPoint(World, SpawnPoint, ViewRotation))
		{
			return;
		}

		EBox3DWindMode Mode = EBox3DWindMode::Directional;
		if (Args.Num() > 0)
		{
			if (Args[0].Equals(TEXT("turbulence"), ESearchCase::IgnoreCase))
			{
				Mode = EBox3DWindMode::Turbulence;
			}
			else if (Args[0].Equals(TEXT("vortex"), ESearchCase::IgnoreCase))
			{
				Mode = EBox3DWindMode::Vortex;
			}
		}

		const FTransform Transform(FRotator(0.0, ViewRotation.Yaw, 0.0), SpawnPoint);
		ABox3DWindActor* Wind = World->SpawnActorDeferred<ABox3DWindActor>(ABox3DWindActor::StaticClass(), Transform);
		if (Wind == nullptr)
		{
			return;
		}
		Wind->WindMode = Mode;
		Wind->WindSpeed = Args.Num() > 1 ? FMath::Clamp(FCString::Atof(*Args[1]), 0.0f, 10000.0f) : 800.0f;
		Wind->FinishSpawning(Transform);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SpawnWind: mode %d, %.0f cm/s at %s"),
			static_cast<int32>(Mode), Wind->WindSpeed, *SpawnPoint.ToCompactString());
	}));

static FAutoConsoleCommandWithWorldAndArgs GBox3DSpawnLiquidCommand(
	TEXT("box3d.SpawnLiquid"),
	TEXT("Hang a Box3D liquid tap 150 cm above the point under the crosshair, pouring down. Usage: box3d.SpawnLiquid [Rate=120] [MaxParticles=400]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		FVector SpawnPoint;
		FRotator ViewRotation;
		if (World == nullptr || !GetSpawnPoint(World, SpawnPoint, ViewRotation))
		{
			return;
		}

		// Forward = flow direction; pitch -90 pours onto the crosshair point.
		const FTransform Transform(FRotator(-90.0, 0.0, 0.0), SpawnPoint + FVector(0, 0, 150));
		ABox3DLiquidSourceActor* Liquid = World->SpawnActorDeferred<ABox3DLiquidSourceActor>(
			ABox3DLiquidSourceActor::StaticClass(), Transform);
		if (Liquid == nullptr)
		{
			return;
		}
		Liquid->SpawnRate = Args.Num() > 0 ? FMath::Clamp(FCString::Atof(*Args[0]), 1.0f, 1000.0f) : 120.0f;
		Liquid->MaxParticles = Args.Num() > 1 ? FMath::Clamp(FCString::Atoi(*Args[1]), 1, 2000) : 400;
		Liquid->FinishSpawning(Transform);
		UE_LOG(LogBox3D, Log, TEXT("box3d.SpawnLiquid: %.0f/s, cap %d at %s"),
			Liquid->SpawnRate, Liquid->MaxParticles, *SpawnPoint.ToCompactString());
	}));

#endif // !UE_BUILD_SHIPPING
