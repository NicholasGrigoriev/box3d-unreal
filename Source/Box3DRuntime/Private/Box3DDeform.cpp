#include "Box3DDeform.h"

#include "box3d/base.h"

namespace Box3D::Deform
{
	namespace
	{
		constexpr double HashQuantizationCm = 0.001;

		uint32 HashInt64(uint32 Hash, int64 Value)
		{
			return b3Hash(Hash, reinterpret_cast<const uint8*>(&Value), sizeof(Value));
		}
	}

	int32 ApplyVertexDent(TArray<FVector>& Vertices, const FBox3DVertexDent& Dent)
	{
		if (Dent.ImpactPoint.ContainsNaN() || Dent.ImpactNormal.ContainsNaN()
			|| !FMath::IsFinite(Dent.RadiusCm) || !FMath::IsFinite(Dent.FalloffExponent)
			|| !FMath::IsFinite(Dent.MaxDepthCm) || Dent.RadiusCm <= UE_SMALL_NUMBER
			|| Dent.MaxDepthCm <= 0.0)
		{
			return 0;
		}

		const FVector Normal = Dent.ImpactNormal.GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			return 0;
		}

		const double RadiusSquared = Dent.RadiusCm * Dent.RadiusCm;
		const double FalloffExponent = FMath::Max(Dent.FalloffExponent, 0.0);
		int32 DisplacedCount = 0;
		for (int32 VertexIndex = 0; VertexIndex < Vertices.Num(); ++VertexIndex)
		{
			FVector& Vertex = Vertices[VertexIndex];
			if (Vertex.ContainsNaN())
			{
				continue;
			}

			const FVector Relative = Vertex - Dent.ImpactPoint;
			const double DistanceSquared = Relative.SizeSquared();
			if (DistanceSquared >= RadiusSquared)
			{
				continue;
			}

			const double Weight = 1.0 - FMath::Sqrt(DistanceSquared) / Dent.RadiusCm;
			const double Falloff = FalloffExponent <= UE_SMALL_NUMBER
				? 1.0
				: FMath::Pow(Weight, FalloffExponent);
			const double Depth = FMath::Clamp(Dent.MaxDepthCm * Falloff, 0.0, Dent.MaxDepthCm);
			if (Depth <= UE_SMALL_NUMBER)
			{
				continue;
			}

			Vertex -= Normal * Depth;
			++DisplacedCount;
		}
		return DisplacedCount;
	}

	uint32 VertexHash(TConstArrayView<FVector> Vertices)
	{
		uint32 Hash = B3_HASH_INIT;
		Hash = HashInt64(Hash, Vertices.Num());
		for (const FVector& Vertex : Vertices)
		{
			Hash = HashInt64(Hash, FMath::RoundToInt64(Vertex.X / HashQuantizationCm));
			Hash = HashInt64(Hash, FMath::RoundToInt64(Vertex.Y / HashQuantizationCm));
			Hash = HashInt64(Hash, FMath::RoundToInt64(Vertex.Z / HashQuantizationCm));
		}
		return Hash;
	}
}
