#pragma once

#include "CoreMinimal.h"
#include "Box3DDestruction.h"
#include "Components/ActorComponent.h"
#include "Box3DDestructibleComponent.generated.h"

class ABox3DFracturedActor;
class UMaterialInterface;
class UNiagaraSystem;
class UStaticMeshComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FBox3DDestructionEventGeneratedSignature,
	FBox3DDestructionEvent, Event, int64, LayoutHash);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FBox3DDestructionCorrectionRequiredSignature,
	FName, MeshId, int64, AuthorityLayoutHash, int64, LocalLayoutHash);

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

	/// Keep supported fragments static and enable D4 connectivity + D5 stress.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Structure")
	bool bStructural = false;

	/// Per-material sustained-load capacities in pascals. A non-positive value
	/// disables damage for that load mode.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Structure", meta = (ClampMin = "0"))
	float TensionStrengthPa = 1.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Structure", meta = (ClampMin = "0"))
	float CompressionStrengthPa = 5.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Structure", meta = (ClampMin = "0"))
	float ShearStrengthPa = 1.0e6f;

	/// Health removed per second for each unit by which load exceeds capacity.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Structure", meta = (ClampMin = "0"))
	float SustainedOverloadHealthPerSecond = 1.0f;

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

	/// Authority-only hook emitted after ApplyImpact makes and applies a fracture
	/// decision. Game code forwards Event + LayoutHash through its RPC or GAS
	/// channel; the plugin deliberately has no dependency on either transport.
	UPROPERTY(BlueprintAssignable, Category = "Box3D|Networking")
	FBox3DDestructionEventGeneratedSignature OnDestructionEventGenerated;

	/// A received event regenerated a different layout. Game code should request
	/// server-authoritative fragment state; predicted fragment bodies can feed
	/// those states to Box3D::ReconcileAndReplay.
	UPROPERTY(BlueprintAssignable, Category = "Box3D|Networking")
	FBox3DDestructionCorrectionRequiredSignature OnDestructionCorrectionRequired;

	/// Build the deterministic replication tuple without applying it. This is an
	/// authority-only fracture decision; false means absorbed damage, no target,
	/// an already-fractured target, or no simulation authority.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Networking")
	bool BuildDestructionEvent(FVector WorldLocation, float EnergyJoules,
		FBox3DDestructionEvent& OutEvent) const;

	/// Apply an authority event to this component. Authority instances build b3
	/// fragment bodies; non-authority/no-world clients build the identical PMC
	/// visual set only. AuthorityLayoutHash = 0 skips validation (server path).
	UFUNCTION(BlueprintCallable, Category = "Box3D|Networking",
		meta = (AdvancedDisplay = "AuthorityLayoutHash"))
	ABox3DFracturedActor* ApplyDestructionEvent(const FBox3DDestructionEvent& Event,
		int64 AuthorityLayoutHash, FBox3DDestructionEventResult& OutResult);

	/// Deal impact damage at a world location. Maps the energy (J) through
	/// EnergyToCells, applies the event locally, then emits
	/// OnDestructionEventGenerated for game-side replication.
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
