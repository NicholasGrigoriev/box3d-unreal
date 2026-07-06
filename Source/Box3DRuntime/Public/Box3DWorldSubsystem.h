#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "box3d/id.h"
#include "Box3DWorldSubsystem.generated.h"

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

	//~ FTickableGameObject
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

	/// The Box3D world handle. Null id (check with b3World_IsValid) outside of
	/// Initialize/Deinitialize.
	b3WorldId GetBox3DWorldId() const { return WorldId; }

	/// Number of fixed steps performed since world creation.
	uint64 GetStepCount() const { return StepCount; }

#if !UE_BUILD_SHIPPING
	/// Smoke test (box3d.Smoke): drop debug-drawn bodies onto a static ground slab.
	void SpawnSmokeBodies(int32 Count);
	void ClearSmokeBodies();
#endif

private:
	void StepFixed(float FixedDeltaTime, int32 SubSteps);

	b3WorldId WorldId = {};
	float Accumulator = 0.0f;
	uint64 StepCount = 0;

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
