#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "Box3DSettings.generated.h"

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

	/// Worker threads for the solver. 1 = single-threaded (current default; the
	/// UE task-system hookup is a later milestone). Values above 1 use Box3D's
	/// internal scheduler threads.
	UPROPERTY(EditAnywhere, config, Category = "Performance", meta = (ClampMin = "1", ClampMax = "16"))
	int32 WorkerCount = 1;
};
