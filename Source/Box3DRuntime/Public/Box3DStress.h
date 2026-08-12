#pragma once

#include "CoreMinimal.h"
#include "Box3DStructure.h"

/// D5 budgeted stress relaxation for structural bond graphs. The solver keeps
/// all state as deterministic plain data and never queries physics joints.
namespace Box3D::Structure
{
	/// One instantaneous impulse applied to a graph node. Units follow the public
	/// UE seam: kg*cm/s at a chunk-local point in centimetres.
	struct FBox3DStressImpulse
	{
		int32 NodeIndex = INDEX_NONE;
		FVector Impulse = FVector::ZeroVector;
		FVector ApplicationPoint = FVector::ZeroVector;
	};

	/// Resolved load on one fine graph bond. Force is oriented NodeA -> NodeB;
	/// positive axial load is compression and negative axial load is tension.
	struct FBox3DBondStress
	{
		FVector Force = FVector::ZeroVector;
		FVector Moment = FVector::ZeroVector;
		double Compression = 0.0;
		double Tension = 0.0;
		double Shear = 0.0;
		double BendingMoment = 0.0;
		double TorsionMoment = 0.0;
	};

	/// Material capacities at the UE-facing seam. Force components are converted
	/// from kg*cm/s^2 over cm^2 to pascals before these thresholds are tested.
	/// A non-positive threshold disables damage for that component.
	struct FBox3DStressThresholds
	{
		double TensionPa = 1.0e6;
		double CompressionPa = 5.0e6;
		double ShearPa = 1.0e6;
	};

	struct FBox3DBondOverload
	{
		double TensionPa = 0.0;
		double CompressionPa = 0.0;
		double ShearPa = 0.0;
		/// Maximum stress/threshold ratio. Damage begins strictly above 1.
		double Ratio = 0.0;
	};

	struct FBox3DStressSolveStats
	{
		int32 Iterations = 0;
		int32 SolveNodeCount = 0;
		int32 SolveBondCount = 0;
		double ResidualForce = 0.0;
		double ResidualMoment = 0.0;
		bool bCoarsened = false;
	};

	/// ExtStress-style graph solver. BeginSolve seeds gravity and impulses;
	/// Relax advances a fixed number of deterministic load-propagation passes, so
	/// callers can amortize a large solve across ticks without changing order.
	class BOX3DRUNTIME_API FBox3DStressSolver
	{
	public:
		/// NodeMassesKg is parallel to Graph nodes. CoarsenNodeThreshold <= 0
		/// disables reduction; otherwise connected same-support-status nodes are
		/// merged deterministically until the graph reaches the threshold or no
		/// legal merge remains.
		bool Initialize(const FBox3DStructureGraph& Graph,
			TConstArrayView<double> NodeMassesKg, int32 CoarsenNodeThreshold = 0);

		/// Convenience initialization from graph volumes and uniform kg/cm^3.
		bool Initialize(const FBox3DStructureGraph& Graph,
			double DensityKgPerCubicCm, int32 CoarsenNodeThreshold = 0);

		void Reset();
		bool IsInitialized() const { return Graph != nullptr; }
		bool IsCoarsened() const { return bCoarsened; }
		int32 GetSolveNodeCount() const { return SolveNodes.Num(); }
		int32 GetSolveBondCount() const { return SolveBonds.Num(); }

		/// Starts a fresh equilibrium solve. Gravity is cm/s^2. Impulses are
		/// converted to average forces over DeltaSeconds and may include off-centre
		/// application points, which seed torque as well as force.
		void BeginSolve(const FVector& GravityCmPerSecondSquared,
			TConstArrayView<FBox3DStressImpulse> Impulses = {},
			double DeltaSeconds = 1.0 / 60.0);

		/// Execute exactly IterationCount fixed-order passes (negative is clamped
		/// to zero) and report the remaining unabsorbed load.
		FBox3DStressSolveStats Relax(int32 IterationCount);

		FBox3DBondStress GetBondStress(int32 BondIndex) const;

		/// Resolve one bond's force components to Pa and compare them with the
		/// material capacities. Bond area is the shared face in cm^2.
		FBox3DBondOverload GetBondOverload(int32 BondIndex,
			const FBox3DStressThresholds& Thresholds) const;

		/// Quantized deterministic digest of fine-bond health and resolved loads.
		/// Slice 2 can use the same digest once overload starts eroding health.
		uint32 BondHealthHash() const;

	private:
		struct FSolveNode
		{
			FVector Centroid = FVector::ZeroVector;
			double MassKg = 0.0;
			bool bAnchor = false;
			FVector ResidualForce = FVector::ZeroVector;
			FVector ResidualMoment = FVector::ZeroVector;
			TArray<int32, TInlineAllocator<8>> BondIndices;
		};

		struct FSolveBond
		{
			int32 NodeA = INDEX_NONE;
			int32 NodeB = INDEX_NONE;
			double Weight = 0.0;
			FVector Centroid = FVector::ZeroVector;
			FVector Normal = FVector::ZeroVector;
			FVector Force = FVector::ZeroVector;
			FVector Moment = FVector::ZeroVector;
		};

		int32 OtherSolveNode(int32 BondIndex, int32 NodeIndex) const;
		void UpdateStats(FBox3DStressSolveStats& Stats) const;

		const FBox3DStructureGraph* Graph = nullptr;
		TArray<FSolveNode> SolveNodes;
		TArray<FSolveBond> SolveBonds;
		TArray<int32> FineNodeToSolveNode;
		TArray<int32> FineBondToSolveBond;
		TArray<int8> FineBondOrientation;
		TArray<double> FineBondWeight;
		bool bCoarsened = false;
	};
}
