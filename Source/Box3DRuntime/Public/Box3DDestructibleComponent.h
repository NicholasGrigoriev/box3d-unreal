#pragma once

#include "CoreMinimal.h"
#include "Box3DDestruction.h"
#include "Components/ActorComponent.h"
#include "Box3DDestructibleComponent.generated.h"

class ABox3DFracturedActor;
class UMaterialInterface;
class UNiagaraSystem;
class UStaticMeshComponent;

/// Opt-in destructible marker — the designer-facing surface of the D3 damage
/// pipeline. Drop it on an actor whose static mesh should fracture and it wires
/// the whole intake: the static scene mirror enables hit events on the mesh's
/// mirror bodies, the world subsystem converts impacts (approach speed x other
/// body's mass) into energy and routes them here, and Box3DExplode delivers
/// blast energy with distance falloff. Energy maps through EnergyToCells and
/// fractures the mesh via Box3D::FractureMesh with this component's material /
/// tier configuration. Game code can also deal damage directly via ApplyImpact.
///
/// One fracture per component: after the mesh swaps out, further impacts are
/// ignored (the fractured actor's welds carry the story from there).
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DDestructibleComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	/// Impact energy (J) -> fracture cell count.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Damage")
	FBox3DEnergyToCellCurve EnergyToCells;

	/// Weld strength per unit of shared-face area (N/cm^2); <= 0 = unbreakable.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Fracture", meta = (ClampMin = "0"))
	float MaterialToughness = 50.0f;

	/// Fragment density in kg/m^3.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Fracture", meta = (ClampMin = "1"))
	float FragmentDensity = 400.0f;

	/// Material for interior (fracture-cut) faces; null falls back to the source
	/// mesh's material.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Fracture")
	TObjectPtr<UMaterialInterface> CoreMaterial;

	/// Fracture layout seed (deterministic per seed + impact + energy).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Fracture")
	int32 Seed = 42;

	/// Fraction of Voronoi sites clustered around the impact point.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Fracture", meta = (ClampMin = "0", ClampMax = "1"))
	float RadialBias = 0.5f;

	/// Fragments below this volume (cm^3) merge into a neighbor during fracture.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Fracture", meta = (ClampMin = "0"))
	float MinFragmentVolume = 0.0f;

	/// Volume thresholds routing fragments into Body / Debris / Dust tiers.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Tiers")
	FBox3DTierThresholds TierThresholds;

	/// Radial scatter speed (cm/s) of Debris-tier burst fragments.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Tiers", meta = (ClampMin = "0"))
	float DebrisSpeed = 300.0f;

	/// Niagara system spawned with the Debris-tier burst on fracture. Receives
	/// world-space user array parameters DebrisPositions (Vector), DebrisVelocities
	/// (Vector), and DebrisSizes (float, cube edge lengths in cm). Null (the
	/// default) spawns no effect — sub-physics fragments simply vanish.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Tiers")
	TObjectPtr<UNiagaraSystem> DebrisSystem;

	/// Deal impact damage at a world location. Maps the energy (J) through
	/// EnergyToCells and fractures the target mesh when it clears MinEnergy.
	/// Returns the fractured actor, or null when the energy was absorbed, the
	/// mesh already fractured, or no fracturable mesh exists on the owner.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Damage")
	ABox3DFracturedActor* ApplyImpact(FVector WorldLocation, float EnergyJoules);

	UFUNCTION(BlueprintPure, Category = "Box3D|Damage")
	bool IsFractured() const { return bFractured; }

	UFUNCTION(BlueprintPure, Category = "Box3D|Damage")
	ABox3DFracturedActor* GetFracturedActor() const { return FracturedActor.Get(); }

	/// The mesh ApplyImpact fractures: the owner's root static mesh component,
	/// else the first static mesh component on the owner.
	UStaticMeshComponent* ResolveTargetMesh() const;

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	bool bFractured = false;

	TWeakObjectPtr<ABox3DFracturedActor> FracturedActor;
};
