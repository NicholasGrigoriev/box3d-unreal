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
}
