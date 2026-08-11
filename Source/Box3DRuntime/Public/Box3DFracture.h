#pragma once

#include "CoreMinimal.h"

/// Deterministic Voronoi fracture core — pure geometry, no actors, no rendering.
///
/// Fragments are convex: Voronoi cells of seeded sites clipped against a convex
/// proxy polytope by half-space bisector planes (see docs/DESTRUCTION_PLAN.md § D1
/// and docs/DESTRUCTION_RESEARCH.md § 3a). Fracture is a pure function of
/// (proxy, params): sites and bisector-plane offsets are quantized to a fixed grid
/// before clipping, iteration order is fixed, and bisector planes are canonical
/// per site pair, so the same inputs produce hash-identical fragments on every
/// run.
///
/// Everything here operates in UE centimeters. The cm -> m conversion happens at
/// the existing Box3DConversion seam when fragments are handed to box3d
/// (b3CreateHull) by later milestones.
namespace Box3D::Fracture
{
	/// Fixed quantization grid (cm) applied to site coordinates and bisector plane
	/// offsets before clipping, and to fragment data inside FractureLayoutHash.
	inline constexpr double QuantizeStep = 0.01;

	/// Snap a scalar to the QuantizeStep grid.
	FORCEINLINE double Quantize(double Value)
	{
		return FMath::RoundToDouble(Value / QuantizeStep) * QuantizeStep;
	}

	FORCEINLINE FVector Quantize(const FVector& V)
	{
		return FVector(Quantize(V.X), Quantize(V.Y), Quantize(V.Z));
	}

	/// Inputs that fully determine a fracture layout (together with the proxy).
	struct FFractureParams
	{
		/// PRNG seed for site generation.
		int32 Seed = 0;

		/// Number of Voronoi sites to generate. The output fragment count matches
		/// unless degenerate cells collapse (rare) or MinFragmentVolume merging
		/// kicks in (D1 slice 2).
		int32 CellCount = 8;

		/// Impact location (cm) that site density will bias toward (D1 slice 2 —
		/// currently unused, sites are uniform).
		FVector ImpactPoint = FVector::ZeroVector;

		/// Radius (cm) of the impact-biased site cluster (D1 slice 2).
		double ImpactRadius = 50.0;

		/// 0 = uniform site density, 1 = fully impact-clustered (D1 slice 2).
		double RadialBias = 0.0;

		/// Fragments below this volume (cm^3) get merged into a neighbor
		/// (D1 slice 2 — currently unused).
		double MinFragmentVolume = 0.0;
	};

	/// One convex face of a fragment. Vertex indices wind counter-clockwise seen
	/// from outside the fragment.
	struct FBox3DFragmentFace
	{
		TArray<int32> VertexIndices;

		/// Fragment index sharing this face, or INDEX_NONE for exterior faces
		/// inherited from the proxy surface. Later milestones use this to assign
		/// source vs interior materials.
		int32 NeighborIndex = INDEX_NONE;

		/// Face area (cm^2).
		double Area = 0.0;
	};

	/// A fragment's shared-face link to one neighboring fragment.
	struct FBox3DFragmentNeighbor
	{
		int32 FragmentIndex = INDEX_NONE;

		/// Area (cm^2) of the single convex face the two fragments share. Later
		/// milestones scale weld break forces by this. Canonical per pair: both
		/// fragments record the area computed on the lower-indexed side, because
		/// quantized bisector offsets mean the two cells' own face polygons can
		/// differ by sub-QuantizeStep slivers (their Face.Area values agree only
		/// to that tolerance).
		double SharedFaceArea = 0.0;
	};

	/// One convex fragment produced by Fracture(). All quantities in UE cm.
	struct FBox3DFragmentData
	{
		TArray<FVector> Vertices;
		TArray<FBox3DFragmentFace> Faces;
		double Volume = 0.0;
		FVector Centroid = FVector::ZeroVector;

		/// Symmetric adjacency: A lists B iff B lists A, with matching areas.
		/// Derived from Faces; sorted by FragmentIndex ascending.
		TArray<FBox3DFragmentNeighbor> Neighbors;
	};

	/// The convex volume to fracture: a closed convex polytope with faces wound
	/// counter-clockwise seen from outside.
	struct FFractureProxy
	{
		TArray<FVector> Vertices;
		TArray<TArray<int32>> Faces;
	};

	/// Axis-aligned box proxy — the common case for D1 tests and debug commands.
	BOX3DRUNTIME_API FFractureProxy MakeBoxProxy(const FVector& Center, const FVector& HalfExtents);

	/// Fracture Proxy into convex Voronoi fragments. Deterministic: same
	/// (proxy, params) produce bit-identical output. Returns false when inputs are
	/// degenerate (empty proxy, CellCount < 1) or no fragment survives.
	BOX3DRUNTIME_API bool Fracture(const FFractureProxy& Proxy, const FFractureParams& Params,
		TArray<FBox3DFragmentData>& OutFragments);

	/// djb2 digest (b3Hash) over quantized fragment data, folded in stable order —
	/// same layout hashes identically across runs and machines. Mirrors the
	/// Box3DSnapshot.h hashing pattern.
	BOX3DRUNTIME_API uint32 FractureLayoutHash(const TArray<FBox3DFragmentData>& Fragments);
}
