#include "Box3DStress.h"

#include "box3d/base.h"

namespace Box3D::Structure
{
	namespace
	{
		constexpr double MinBondWeight = 1.0e-6;
		constexpr double HashQuantization = 1000.0;

		int64 QuantizeForHash(double Value)
		{
			return FMath::RoundToInt64(Value * HashQuantization);
		}

		uint32 HashValue(uint32 Hash, const void* Value, int32 Size)
		{
			return b3Hash(Hash, static_cast<const uint8*>(Value), Size);
		}

		uint32 HashInt64(uint32 Hash, int64 Value)
		{
			return HashValue(Hash, &Value, sizeof(Value));
		}

		uint32 HashVector(uint32 Hash, const FVector& Value)
		{
			Hash = HashInt64(Hash, QuantizeForHash(Value.X));
			Hash = HashInt64(Hash, QuantizeForHash(Value.Y));
			return HashInt64(Hash, QuantizeForHash(Value.Z));
		}
	}

	bool FBox3DStressSolver::Initialize(const FBox3DStructureGraph& InGraph,
		TConstArrayView<double> NodeMassesKg, int32 CoarsenNodeThreshold)
	{
		Reset();
		if (NodeMassesKg.Num() != InGraph.GetNodeCount())
		{
			return false;
		}

		Graph = &InGraph;
		const int32 FineNodeCount = Graph->GetNodeCount();
		const int32 FineBondCount = Graph->GetBondCount();
		FineNodeToSolveNode.Init(INDEX_NONE, FineNodeCount);
		FineBondToSolveBond.Init(INDEX_NONE, FineBondCount);
		FineBondOrientation.Init(0, FineBondCount);
		FineBondWeight.Init(0.0, FineBondCount);

		TArray<int32> Parent;
		TArray<int32> ClusterSize;
		Parent.SetNumUninitialized(FineNodeCount);
		ClusterSize.Init(1, FineNodeCount);
		int32 ActiveClusterCount = 0;
		for (int32 NodeIndex = 0; NodeIndex < FineNodeCount; ++NodeIndex)
		{
			Parent[NodeIndex] = NodeIndex;
			if (Graph->IsChunkDestroyed(NodeIndex))
			{
				ClusterSize[NodeIndex] = 0;
			}
			else
			{
				++ActiveClusterCount;
			}
		}

		const auto FindRoot = [&Parent](int32 NodeIndex)
		{
			int32 Root = NodeIndex;
			while (Parent[Root] != Root)
			{
				Root = Parent[Root];
			}
			while (Parent[NodeIndex] != NodeIndex)
			{
				const int32 Next = Parent[NodeIndex];
				Parent[NodeIndex] = Root;
				NodeIndex = Next;
			}
			return Root;
		};

		const int32 TargetCount = CoarsenNodeThreshold > 0
			? FMath::Max(CoarsenNodeThreshold, 1)
			: ActiveClusterCount;
		while (ActiveClusterCount > TargetCount)
		{
			bool bMergedAny = false;
			for (int32 BondIndex = 0; BondIndex < FineBondCount && ActiveClusterCount > TargetCount; ++BondIndex)
			{
				const FBox3DStructureBond& Bond = Graph->GetBond(BondIndex);
				if (Bond.bBroken || Graph->IsChunkDestroyed(Bond.NodeA) || Graph->IsChunkDestroyed(Bond.NodeB))
				{
					continue;
				}
				int32 RootA = FindRoot(Bond.NodeA);
				int32 RootB = FindRoot(Bond.NodeB);
				if (RootA == RootB || Graph->IsAnchor(RootA) != Graph->IsAnchor(RootB))
				{
					continue;
				}
				if (RootB < RootA)
				{
					Swap(RootA, RootB);
				}
				Parent[RootB] = RootA;
				ClusterSize[RootA] += ClusterSize[RootB];
				ClusterSize[RootB] = 0;
				--ActiveClusterCount;
				bMergedAny = true;
			}
			if (!bMergedAny)
			{
				break;
			}
		}

		TArray<int32> RootToSolveNode;
		RootToSolveNode.Init(INDEX_NONE, FineNodeCount);
		TArray<int32> MemberCounts;
		for (int32 NodeIndex = 0; NodeIndex < FineNodeCount; ++NodeIndex)
		{
			if (Graph->IsChunkDestroyed(NodeIndex))
			{
				continue;
			}
			const int32 Root = FindRoot(NodeIndex);
			if (RootToSolveNode[Root] == INDEX_NONE)
			{
				RootToSolveNode[Root] = SolveNodes.AddDefaulted();
				MemberCounts.Add(0);
			}
			const int32 SolveNodeIndex = RootToSolveNode[Root];
			FineNodeToSolveNode[NodeIndex] = SolveNodeIndex;
			FSolveNode& SolveNode = SolveNodes[SolveNodeIndex];
			const double Mass = FMath::Max(NodeMassesKg[NodeIndex], 0.0);
			SolveNode.MassKg += Mass;
			SolveNode.Centroid += Graph->GetNodeCentroid(NodeIndex) * Mass;
			SolveNode.bAnchor |= Graph->IsAnchor(NodeIndex);
			++MemberCounts[SolveNodeIndex];
		}

		for (int32 SolveNodeIndex = 0; SolveNodeIndex < SolveNodes.Num(); ++SolveNodeIndex)
		{
			FSolveNode& SolveNode = SolveNodes[SolveNodeIndex];
			if (SolveNode.MassKg > UE_SMALL_NUMBER)
			{
				SolveNode.Centroid /= SolveNode.MassKg;
				continue;
			}
			for (int32 FineNodeIndex = 0; FineNodeIndex < FineNodeCount; ++FineNodeIndex)
			{
				if (FineNodeToSolveNode[FineNodeIndex] == SolveNodeIndex)
				{
					SolveNode.Centroid += Graph->GetNodeCentroid(FineNodeIndex);
				}
			}
			SolveNode.Centroid /= FMath::Max(MemberCounts[SolveNodeIndex], 1);
		}

		for (int32 FineBondIndex = 0; FineBondIndex < FineBondCount; ++FineBondIndex)
		{
			const FBox3DStructureBond& FineBond = Graph->GetBond(FineBondIndex);
			if (FineBond.bBroken)
			{
				continue;
			}
			int32 SolveA = FineNodeToSolveNode[FineBond.NodeA];
			int32 SolveB = FineNodeToSolveNode[FineBond.NodeB];
			if (SolveA == INDEX_NONE || SolveB == INDEX_NONE || SolveA == SolveB)
			{
				continue;
			}

			int8 Orientation = 1;
			if (SolveB < SolveA)
			{
				Swap(SolveA, SolveB);
				Orientation = -1;
			}
			int32 SolveBondIndex = INDEX_NONE;
			for (int32 Candidate = 0; Candidate < SolveBonds.Num(); ++Candidate)
			{
				if (SolveBonds[Candidate].NodeA == SolveA && SolveBonds[Candidate].NodeB == SolveB)
				{
					SolveBondIndex = Candidate;
					break;
				}
			}
			if (SolveBondIndex == INDEX_NONE)
			{
				SolveBondIndex = SolveBonds.AddDefaulted();
				SolveBonds[SolveBondIndex].NodeA = SolveA;
				SolveBonds[SolveBondIndex].NodeB = SolveB;
				SolveNodes[SolveA].BondIndices.Add(SolveBondIndex);
				SolveNodes[SolveB].BondIndices.Add(SolveBondIndex);
			}

			const double Weight = FMath::Max(FineBond.Area, MinBondWeight);
			FSolveBond& SolveBond = SolveBonds[SolveBondIndex];
			SolveBond.Weight += Weight;
			SolveBond.Centroid += FineBond.Centroid * Weight;
			SolveBond.Normal += FineBond.Normal * (Weight * Orientation);
			FineBondToSolveBond[FineBondIndex] = SolveBondIndex;
			FineBondOrientation[FineBondIndex] = Orientation;
			FineBondWeight[FineBondIndex] = Weight;
		}

		for (FSolveBond& Bond : SolveBonds)
		{
			Bond.Centroid /= Bond.Weight;
			Bond.Normal = Bond.Normal.GetSafeNormal();
			if (Bond.Normal.IsNearlyZero())
			{
				Bond.Normal = (SolveNodes[Bond.NodeB].Centroid - SolveNodes[Bond.NodeA].Centroid).GetSafeNormal();
			}
		}

		bCoarsened = SolveNodes.Num() < ActiveClusterCount
			|| SolveNodes.Num() < FineNodeCount;
		return true;
	}

	bool FBox3DStressSolver::Initialize(const FBox3DStructureGraph& InGraph,
		double DensityKgPerCubicCm, int32 CoarsenNodeThreshold)
	{
		TArray<double> Masses;
		Masses.Reserve(InGraph.GetNodeCount());
		const double Density = FMath::Max(DensityKgPerCubicCm, 0.0);
		for (int32 NodeIndex = 0; NodeIndex < InGraph.GetNodeCount(); ++NodeIndex)
		{
			Masses.Add(InGraph.GetNodeVolume(NodeIndex) * Density);
		}
		return Initialize(InGraph, Masses, CoarsenNodeThreshold);
	}

	void FBox3DStressSolver::Reset()
	{
		Graph = nullptr;
		SolveNodes.Reset();
		SolveBonds.Reset();
		FineNodeToSolveNode.Reset();
		FineBondToSolveBond.Reset();
		FineBondOrientation.Reset();
		FineBondWeight.Reset();
		bCoarsened = false;
	}

	void FBox3DStressSolver::BeginSolve(const FVector& GravityCmPerSecondSquared,
		TConstArrayView<FBox3DStressImpulse> Impulses, double DeltaSeconds)
	{
		if (Graph == nullptr)
		{
			return;
		}
		for (FSolveNode& Node : SolveNodes)
		{
			Node.ResidualForce = GravityCmPerSecondSquared * Node.MassKg;
			Node.ResidualMoment = FVector::ZeroVector;
		}
		for (FSolveBond& Bond : SolveBonds)
		{
			Bond.Force = FVector::ZeroVector;
			Bond.Moment = FVector::ZeroVector;
		}

		const double SafeDeltaSeconds = FMath::Max(DeltaSeconds, UE_SMALL_NUMBER);
		for (const FBox3DStressImpulse& Impulse : Impulses)
		{
			if (!FineNodeToSolveNode.IsValidIndex(Impulse.NodeIndex))
			{
				continue;
			}
			const int32 SolveNodeIndex = FineNodeToSolveNode[Impulse.NodeIndex];
			if (!SolveNodes.IsValidIndex(SolveNodeIndex))
			{
				continue;
			}
			FSolveNode& Node = SolveNodes[SolveNodeIndex];
			const FVector Force = Impulse.Impulse / SafeDeltaSeconds;
			Node.ResidualForce += Force;
			Node.ResidualMoment += FVector::CrossProduct(Impulse.ApplicationPoint - Node.Centroid, Force);
		}
	}

	FBox3DStressSolveStats FBox3DStressSolver::Relax(int32 IterationCount)
	{
		FBox3DStressSolveStats Stats;
		if (Graph == nullptr)
		{
			return Stats;
		}

		Stats.Iterations = FMath::Max(IterationCount, 0);
		for (int32 Iteration = 0; Iteration < Stats.Iterations; ++Iteration)
		{
			for (int32 NodeIndex = 0; NodeIndex < SolveNodes.Num(); ++NodeIndex)
			{
				FSolveNode& Node = SolveNodes[NodeIndex];
				if (Node.bAnchor)
				{
					Node.ResidualForce = FVector::ZeroVector;
					Node.ResidualMoment = FVector::ZeroVector;
					continue;
				}
				if (Node.BondIndices.IsEmpty())
				{
					continue;
				}

				double TotalWeight = 0.0;
				for (const int32 BondIndex : Node.BondIndices)
				{
					TotalWeight += SolveBonds[BondIndex].Weight;
				}
				const FVector SourceForce = Node.ResidualForce;
				const FVector SourceMoment = Node.ResidualMoment;
				Node.ResidualForce = FVector::ZeroVector;
				Node.ResidualMoment = FVector::ZeroVector;

				for (const int32 BondIndex : Node.BondIndices)
				{
					FSolveBond& Bond = SolveBonds[BondIndex];
					const double Fraction = Bond.Weight / TotalWeight;
					const FVector TransferForce = SourceForce * Fraction;
					const FVector TransferMomentAtNode = SourceMoment * Fraction;
					const int32 OtherIndex = OtherSolveNode(BondIndex, NodeIndex);
					FSolveNode& Other = SolveNodes[OtherIndex];
					const FVector TransferMomentAtBond = TransferMomentAtNode
						+ FVector::CrossProduct(Node.Centroid - Bond.Centroid, TransferForce);
					const double Orientation = NodeIndex == Bond.NodeA ? 1.0 : -1.0;
					Bond.Force += TransferForce * Orientation;
					Bond.Moment += TransferMomentAtBond * Orientation;
					Other.ResidualForce += TransferForce;
					Other.ResidualMoment += TransferMomentAtNode
						+ FVector::CrossProduct(Node.Centroid - Other.Centroid, TransferForce);
				}
			}
		}

		UpdateStats(Stats);
		return Stats;
	}

	FBox3DBondStress FBox3DStressSolver::GetBondStress(int32 BondIndex) const
	{
		FBox3DBondStress Result;
		if (Graph == nullptr || !FineBondToSolveBond.IsValidIndex(BondIndex))
		{
			return Result;
		}
		const int32 SolveBondIndex = FineBondToSolveBond[BondIndex];
		if (!SolveBonds.IsValidIndex(SolveBondIndex))
		{
			return Result;
		}

		const FSolveBond& SolveBond = SolveBonds[SolveBondIndex];
		const double Scale = FineBondOrientation[BondIndex]
			* FineBondWeight[BondIndex] / SolveBond.Weight;
		Result.Force = SolveBond.Force * Scale;
		Result.Moment = SolveBond.Moment * Scale;

		const FVector Normal = Graph->GetBond(BondIndex).Normal;
		const double AxialLoad = FVector::DotProduct(Result.Force, Normal);
		Result.Compression = FMath::Max(AxialLoad, 0.0);
		Result.Tension = FMath::Max(-AxialLoad, 0.0);
		Result.Shear = (Result.Force - Normal * AxialLoad).Length();
		const double Torsion = FVector::DotProduct(Result.Moment, Normal);
		Result.TorsionMoment = FMath::Abs(Torsion);
		Result.BendingMoment = (Result.Moment - Normal * Torsion).Length();
		return Result;
	}

	uint32 FBox3DStressSolver::BondHealthHash() const
	{
		uint32 Hash = B3_HASH_INIT;
		if (Graph == nullptr)
		{
			return Hash;
		}
		const int32 BondCount = Graph->GetBondCount();
		Hash = HashValue(Hash, &BondCount, sizeof(BondCount));
		for (int32 BondIndex = 0; BondIndex < BondCount; ++BondIndex)
		{
			const FBox3DStructureBond& Bond = Graph->GetBond(BondIndex);
			const int64 Health = QuantizeForHash(Bond.Health);
			const uint8 bBroken = Bond.bBroken ? 1 : 0;
			Hash = HashInt64(Hash, Health);
			Hash = HashValue(Hash, &bBroken, sizeof(bBroken));
			const FBox3DBondStress Stress = GetBondStress(BondIndex);
			Hash = HashVector(Hash, Stress.Force);
			Hash = HashVector(Hash, Stress.Moment);
		}
		return Hash;
	}

	int32 FBox3DStressSolver::OtherSolveNode(int32 BondIndex, int32 NodeIndex) const
	{
		const FSolveBond& Bond = SolveBonds[BondIndex];
		return Bond.NodeA == NodeIndex ? Bond.NodeB : Bond.NodeA;
	}

	void FBox3DStressSolver::UpdateStats(FBox3DStressSolveStats& Stats) const
	{
		Stats.SolveNodeCount = SolveNodes.Num();
		Stats.SolveBondCount = SolveBonds.Num();
		Stats.bCoarsened = bCoarsened;
		for (const FSolveNode& Node : SolveNodes)
		{
			if (!Node.bAnchor)
			{
				Stats.ResidualForce += Node.ResidualForce.Length();
				Stats.ResidualMoment += Node.ResidualMoment.Length();
			}
		}
	}
}
