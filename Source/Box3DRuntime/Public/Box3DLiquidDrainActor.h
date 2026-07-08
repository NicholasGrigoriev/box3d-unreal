#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "box3d/id.h"
#include "Box3DLiquidDrainActor.generated.h"

class ABox3DLiquidSourceActor;
class UDrawSphereComponent;

/// A liquid sink: a radial suction field that pulls nearby liquid particles
/// toward the actor and swallows the ones that reach the consume core, where
/// they shrink out and despawn. Place it at a grate, a pipe mouth, a whirlpool
/// center — pair with your own mesh; this actor is pure behavior plus two
/// editor-only radius gizmos.
///
/// Suction is a per-fixed-step force with configurable strength and falloff,
/// so pools visibly flow toward the drain rather than vanishing on contact.
/// DrainRate caps swallowing throughput (a slow drain against a fast source
/// makes an overflow); the suction field keeps pulling either way. By default
/// suction wakes sleeping particles so a settled pool next to a drain still
/// empties.
///
/// Works on every ABox3DLiquidSourceActor's particles in the world, or only
/// the sources listed in AffectedSources.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DLiquidDrainActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DLiquidDrainActor();

	/// Master switch — flip at runtime to plug/unplug the drain.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain")
	bool bEnabled = true;

	/// Reach of the suction field in cm.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain", meta = (ClampMin = "10"))
	float SuctionRadius = 250.0f;

	/// Pull acceleration in m/s^2 at the drain center, fading to zero at
	/// SuctionRadius. Gravity is ~10; noticeably higher values dominate it.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain", meta = (ClampMin = "0", ClampMax = "500"))
	float SuctionStrength = 60.0f;

	/// Falloff curve exponent: 1 = linear, higher = pull concentrated near the
	/// core, lower = broad even suction.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain", meta = (ClampMin = "0.1", ClampMax = "8"))
	float FalloffExponent = 1.0f;

	/// Velocity damping (1/s) inside the field. Without it, fast liquid ORBITS
	/// a strong drain like water around a plughole — a central pull adds no way
	/// to shed the swing-around energy; this bleeds it so particles spiral in.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain", meta = (ClampMin = "0", ClampMax = "20"))
	float SuctionDamping = 1.5f;

	/// Particles inside this radius get swallowed (cm).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain", meta = (ClampMin = "1"))
	float ConsumeRadius = 50.0f;

	/// Swallowed particles shrink out over this window instead of popping.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drain", meta = (ClampMin = "0"))
	float ConsumeShrinkSeconds = 0.2f;

	/// Max particles swallowed per second; 0 = unlimited. The suction field
	/// still pulls when the budget is spent, so liquid queues at the core.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain", meta = (ClampMin = "0"))
	float DrainRate = 0.0f;

	/// Wake sleeping particles inside the field so settled pools still drain.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drain")
	bool bWakeParticles = true;

	/// Sources this drain acts on. Empty = every liquid source in the world.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drain")
	TArray<TObjectPtr<ABox3DLiquidSourceActor>> AffectedSources;

	/// Particles swallowed since BeginPlay — fill meters, puzzle triggers.
	UFUNCTION(BlueprintPure, Category = "Drain")
	int32 GetTotalConsumed() const { return TotalConsumed; }

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void PreStep(float FixedDeltaTime);
	void DrainSource(ABox3DLiquidSourceActor& Source);

	/// Editor-only wireframe gizmos for the two radii.
	UPROPERTY()
	TObjectPtr<UDrawSphereComponent> SuctionGizmo;

	UPROPERTY()
	TObjectPtr<UDrawSphereComponent> ConsumeGizmo;

	FDelegateHandle PreStepHandle;
	/// Fractional swallow budget carried between steps (DrainRate accumulator).
	float ConsumeBudget = 0.0f;
	int32 TotalConsumed = 0;
};
