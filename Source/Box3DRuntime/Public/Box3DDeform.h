#pragma once

#include "CoreMinimal.h"

/// Deterministic cosmetic deformation helpers. Geometry stays in UE centimetres;
/// callers own rendering and deliberately do not rebuild collision.
namespace Box3D::Deform
{
	/// One circular dent stamp in the same space as Vertices. ImpactNormal points
	/// out of the struck surface; vertices move opposite it. FalloffExponent 0
	/// produces a flat stamp, 1 is linear, and larger values tighten the centre.
	struct FBox3DVertexDent
	{
		FVector ImpactPoint = FVector::ZeroVector;
		FVector ImpactNormal = FVector::UpVector;
		double RadiusCm = 10.0;
		double FalloffExponent = 2.0;
		double MaxDepthCm = 2.0;
	};

	/// Apply one dent in fixed vertex-index order. Vertices inside RadiusCm move
	/// inward, and no vertex moves farther than MaxDepthCm for this stamp. Returns
	/// the number of displaced vertices.
	BOX3DRUNTIME_API int32 ApplyVertexDent(TArray<FVector>& Vertices,
		const FBox3DVertexDent& Dent);

	/// Stable digest over vertices quantized to 0.001 cm in array order.
	BOX3DRUNTIME_API uint32 VertexHash(TConstArrayView<FVector> Vertices);
}
