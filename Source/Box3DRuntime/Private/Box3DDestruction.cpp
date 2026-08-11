#include "Box3DDestruction.h"

namespace Box3D::Destruction
{
	float ImpactEnergyJoules(float MassKg, float ApproachSpeedCmS)
	{
		const float SpeedMs = ApproachSpeedCmS * 0.01f;
		return 0.5f * FMath::Max(MassKg, 0.0f) * SpeedMs * SpeedMs;
	}

	int32 EnergyToCellCount(float EnergyJoules, const FBox3DEnergyToCellCurve& Curve)
	{
		if (Curve.MaxCellCount < 1 || EnergyJoules < Curve.MinEnergy)
		{
			return 0;
		}
		if (Curve.FullEnergy <= Curve.MinEnergy)
		{
			return Curve.MaxCellCount;
		}
		const float T = FMath::Clamp(
			(EnergyJoules - Curve.MinEnergy) / (Curve.FullEnergy - Curve.MinEnergy), 0.0f, 1.0f);
		const int32 Cells = FMath::RoundToInt32(
			FMath::Lerp(float(Curve.MinCellCount), float(Curve.MaxCellCount), FMath::Sqrt(T)));
		return FMath::Clamp(Cells, 1, Curve.MaxCellCount);
	}

	EBox3DFragmentTier ClassifyFragmentTier(double VolumeCm3, const FBox3DTierThresholds& Thresholds)
	{
		if (VolumeCm3 < Thresholds.RenderVolumeThreshold)
		{
			return EBox3DFragmentTier::Dust;
		}
		if (VolumeCm3 < Thresholds.PhysicsVolumeThreshold)
		{
			return EBox3DFragmentTier::Debris;
		}
		return EBox3DFragmentTier::Body;
	}

	void ClassifyFragmentTiers(const TArray<Fracture::FBox3DFragmentData>& Fragments,
		const FBox3DTierThresholds& Thresholds, TArray<EBox3DFragmentTier>& OutTiers)
	{
		OutTiers.Reset(Fragments.Num());
		for (const Fracture::FBox3DFragmentData& Fragment : Fragments)
		{
			OutTiers.Add(ClassifyFragmentTier(Fragment.Volume, Thresholds));
		}
	}

	void BuildDebrisBurst(const TArray<Fracture::FBox3DFragmentData>& Fragments,
		const TArray<EBox3DFragmentTier>& Tiers, const FVector& ImpactPoint, float DebrisSpeedCmS,
		FBox3DDebrisBurst& OutBurst)
	{
		OutBurst.Positions.Reset();
		OutBurst.Velocities.Reset();
		OutBurst.Sizes.Reset();

		for (int32 Index = 0; Index < Fragments.Num(); ++Index)
		{
			if (!Tiers.IsValidIndex(Index) || Tiers[Index] != EBox3DFragmentTier::Debris)
			{
				continue;
			}
			const Fracture::FBox3DFragmentData& Fragment = Fragments[Index];
			OutBurst.Positions.Add(Fragment.Centroid);
			OutBurst.Velocities.Add(
				(Fragment.Centroid - ImpactPoint).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector)
				* DebrisSpeedCmS);
			OutBurst.Sizes.Add(float(FMath::Pow(FMath::Max(Fragment.Volume, 0.0), 1.0 / 3.0)));
		}
	}
}
