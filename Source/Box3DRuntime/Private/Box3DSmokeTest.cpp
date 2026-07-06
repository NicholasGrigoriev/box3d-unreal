// Console hook for the M0 smoke test: `box3d.Smoke [Count]` in PIE drops
// debug-drawn rigid bodies through the full create/step/read-back loop.

#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

#if !UE_BUILD_SHIPPING

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

#endif // !UE_BUILD_SHIPPING
