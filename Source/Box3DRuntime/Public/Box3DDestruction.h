#pragma once

#include "CoreMinimal.h"
#include "Box3DFracture.h"
#include "Box3DDestruction.generated.h"

/// D3 damage pipeline data: impact energy intake, the energy -> cell-count
/// mapping curve, and the volume-threshold tier policy that decides which
/// fragments get bodies, which become a Niagara debris burst, and which are
/// dropped outright (docs/DESTRUCTION_PLAN.md § D3). Everything here is pure
/// data + functions — no actors, no effects — so tier decisions and energy
/// mapping are testable without Niagara or a physics world.

/// Simulation tier of one fracture fragment, decided purely from its volume.
UENUM(BlueprintType)
enum class EBox3DFragmentTier : uint8
{
	/// Full citizen: hull body, welds, and rendered PMC sections.
	Body,
	/// Below PhysicsVolumeThreshold: no body, no welds, no sections — handed to
	/// a Niagara burst (position/velocity/size arrays) instead.
	Debris,
	/// Below RenderVolumeThreshold: does not exist at all.
	Dust,
};

/// Fragment-volume thresholds (cm^3) routing fragments into tiers. Defaults of
/// zero keep every fragment in the Body tier. Dust wins where the thresholds
/// overlap: a fragment below RenderVolumeThreshold is dropped even when
/// RenderVolumeThreshold exceeds PhysicsVolumeThreshold.
USTRUCT(BlueprintType)
struct FBox3DTierThresholds
{
	GENERATED_BODY()

	/// Fragments below this volume (cm^3) skip physics bodies and welds and are
	/// routed to the debris burst.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	double PhysicsVolumeThreshold = 0.0;

	/// Fragments below this volume (cm^3) are dropped entirely.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	double RenderVolumeThreshold = 0.0;
};

/// Analytic impact-energy (J) -> Voronoi cell-count mapping: zero below
/// MinEnergy (the impact is absorbed), MinCellCount at MinEnergy, easing up to
/// MaxCellCount at FullEnergy with a sqrt curve (early joules buy cells fast,
/// later ones saturate), clamped above. Monotonic by construction.
USTRUCT(BlueprintType)
struct FBox3DEnergyToCellCurve
{
	GENERATED_BODY()

	/// Impacts below this energy (J) do not fracture at all.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float MinEnergy = 100.0f;

	/// Energy (J) at which the curve reaches MaxCellCount; higher energy clamps.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float FullEnergy = 20000.0f;

	/// Cell count of the weakest fracturing impact.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "1"))
	int32 MinCellCount = 4;

	/// Cell count cap (also the per-event cell cap of the D3 budget policy).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "1"))
	int32 MaxCellCount = 48;
};

/// Replicated numeric policy accompanying one destruction decision. Presentation
/// assets stay on the local destructible component; every value that can change
/// fragment geometry, tiering, bodies, or structural state travels in the event.
USTRUCT(BlueprintType)
struct FBox3DDestructionEventParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "1"))
	int32 CellCount = 8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	double ImpactRadius = 50.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0", ClampMax = "1"))
	double RadialBias = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	double MinFragmentVolume = 0.0;

	/// Proxy-local axis (0/1/2) whose site coordinate is flattened to the centre so
	/// cells span the full thickness (thin slabs); -1 keeps free 3D sites.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "-1", ClampMax = "2"))
	int32 FlattenAxis = -1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float MaterialToughness = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "1"))
	float FragmentDensity = 400.0f;

	/// Physics-hull inset toward each centroid (cm) so neighbouring hulls never overlap.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float HullInsetCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	bool bStartAsleep = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	FBox3DTierThresholds Tiers;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float DebrisSpeed = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	bool bStructural = false;

	/// Structural only: every fragment is an anchor instead of auto-detecting
	/// support from surrounding static bodies.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	bool bAnchorAllFragments = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float TensionStrengthPa = 1.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float CompressionStrengthPa = 5.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float ShearStrengthPa = 1.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction", meta = (ClampMin = "0"))
	float SustainedOverloadHealthPerSecond = 1.0f;
};

/// Game-side RPC/GAS payload. MeshId is the stable source-mesh asset path; game
/// code still chooses the target actor/component through its own replicated id.
/// Impact is world-space UE centimetres. The authority sends the event plus the
/// layout hash returned by applying it; receivers validate before accepting
/// predicted fragment state.
USTRUCT(BlueprintType)
struct FBox3DDestructionEvent
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	FName MeshId = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	FVector Impact = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	int32 Seed = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D|Destruction")
	FBox3DDestructionEventParams Params;
};

/// Result of applying a destruction event locally. Hashes use int64 only because
/// Blueprint has no uint32 pin; values are always in the uint32 range.
USTRUCT(BlueprintType)
struct FBox3DDestructionEventResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Destruction")
	bool bApplied = false;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Destruction")
	bool bVisualOnly = false;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Destruction")
	bool bLayoutHashValidated = false;

	/// True when game code must request the authority's fragment/body correction.
	/// Predicted fragment bodies can use Box3D::ReconcileAndReplay for that state.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Destruction")
	bool bNeedsServerCorrection = false;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Destruction")
	int64 FractureLayoutHash = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Box3D|Destruction")
	int64 BondHealthHash = 0;
};

/// Per-fragment burst arrays for the Debris tier, in the fractured actor's
/// space (actor transform maps them to world). The Niagara hookup consuming
/// these is a later D3 slice; until then they are inspectable data.
struct FBox3DDebrisBurst
{
	/// Fragment centroids.
	TArray<FVector> Positions;

	/// Radial scatter velocities (cm/s): away from the impact point at the
	/// configured debris speed.
	TArray<FVector> Velocities;

	/// Equivalent cube edge lengths (cm): cbrt(fragment volume).
	TArray<float> Sizes;
};

namespace Box3D::Destruction
{
	/// Kinetic energy (J) of an impact: 1/2 m v^2 with the approach speed taken
	/// as the relative velocity. Mass in kg, speed in UE cm/s (the unit hit
	/// events report).
	BOX3DRUNTIME_API float ImpactEnergyJoules(float MassKg, float ApproachSpeedCmS);

	/// Map impact energy (J) through the curve. Returns 0 (no fracture) below
	/// MinEnergy, otherwise a cell count in [MinCellCount, MaxCellCount].
	BOX3DRUNTIME_API int32 EnergyToCellCount(float EnergyJoules, const FBox3DEnergyToCellCurve& Curve);

	/// Tier of a single fragment volume (cm^3) under the thresholds.
	BOX3DRUNTIME_API EBox3DFragmentTier ClassifyFragmentTier(double VolumeCm3, const FBox3DTierThresholds& Thresholds);

	/// Classify every fragment; OutTiers is parallel to Fragments.
	BOX3DRUNTIME_API void ClassifyFragmentTiers(const TArray<Fracture::FBox3DFragmentData>& Fragments,
		const FBox3DTierThresholds& Thresholds, TArray<EBox3DFragmentTier>& OutTiers);

	/// Collect the Debris-tier fragments into burst arrays. ImpactPoint and the
	/// fragment data share one space (the fractured actor's); DebrisSpeedCmS
	/// scales the radial scatter velocity. A fragment whose centroid coincides
	/// with the impact point scatters upward.
	BOX3DRUNTIME_API void BuildDebrisBurst(const TArray<Fracture::FBox3DFragmentData>& Fragments,
		const TArray<EBox3DFragmentTier>& Tiers, const FVector& ImpactPoint, float DebrisSpeedCmS,
		FBox3DDebrisBurst& OutBurst);

	/// Convert the replicated tuple into pure fracture inputs. ComponentToWorld
	/// must contain rotation + translation only: proxy scale is already baked.
	BOX3DRUNTIME_API Fracture::FFractureParams MakeFractureParams(
		const FBox3DDestructionEvent& Event, const FTransform& ComponentToWorld);

	/// Pure deterministic event expansion used by authority and visual clients.
	/// It has no actor, renderer, or b3-world dependency.
	BOX3DRUNTIME_API bool GenerateEventFragments(const Fracture::FFractureProxy& Proxy,
		const FTransform& ComponentToWorld, const FBox3DDestructionEvent& Event,
		TArray<Fracture::FBox3DFragmentData>& OutFragments);

	/// Initial D4/D5 bond-health digest for an event's fragment graph. This uses
	/// the same structure graph and stress-solver hash as live structural actors,
	/// but needs no physics world or anchor query.
	BOX3DRUNTIME_API uint32 InitialBondHealthHash(
		const TArray<Fracture::FBox3DFragmentData>& Fragments, float FragmentDensity);
}
