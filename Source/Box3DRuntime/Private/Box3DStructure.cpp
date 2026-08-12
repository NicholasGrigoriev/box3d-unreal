#include "Box3DStructure.h"

#include "Box3DConversion.h"
#include "box3d/box3d.h"

namespace Box3D::Structure
{
	void FBox3DStructureGraph::Build(TConstArrayView<Fracture::FBox3DFragmentData> Fragments)
	{
		Reset();
		Nodes.SetNum(Fragments.Num());
		for (int32 Index = 0; Index < Fragments.Num(); ++Index)
		{
			Nodes[Index].Centroid = Fragments[Index].Centroid;
			Nodes[Index].Volume = Fragments[Index].Volume;
		}

		// Lower index owns each pair (Neighbors is symmetric with canonical
		// areas), so every bond is created exactly once in fixed order. A node's
		// BondIndices come out ascending by neighbor index: bonds to lower
		// neighbors land while those neighbors are scanned, bonds to higher ones
		// while the node itself is.
		for (int32 Index = 0; Index < Fragments.Num(); ++Index)
		{
			for (const Fracture::FBox3DFragmentNeighbor& Neighbor : Fragments[Index].Neighbors)
			{
				if (Neighbor.FragmentIndex <= Index || !Nodes.IsValidIndex(Neighbor.FragmentIndex))
				{
					continue;
				}
				const int32 BondIndex = Bonds.Num();
				FBox3DStructureBond& Bond = Bonds.AddDefaulted_GetRef();
				Bond.NodeA = Index;
				Bond.NodeB = Neighbor.FragmentIndex;
				Bond.Area = Neighbor.SharedFaceArea;
				Bond.Centroid = (Fragments[Index].Centroid + Fragments[Neighbor.FragmentIndex].Centroid) * 0.5;
				Bond.Normal = (Fragments[Neighbor.FragmentIndex].Centroid - Fragments[Index].Centroid).GetSafeNormal();
				Nodes[Index].BondIndices.Add(BondIndex);
				Nodes[Neighbor.FragmentIndex].BondIndices.Add(BondIndex);
			}
		}
	}

	void FBox3DStructureGraph::Reset()
	{
		Nodes.Reset();
		Bonds.Reset();
		CurrentStamp = 0;
	}

	int32 FBox3DStructureGraph::GetLiveBondCount() const
	{
		int32 Count = 0;
		for (const FBox3DStructureBond& Bond : Bonds)
		{
			Count += Bond.bBroken ? 0 : 1;
		}
		return Count;
	}

	FVector FBox3DStructureGraph::GetNodeCentroid(int32 NodeIndex) const
	{
		return Nodes.IsValidIndex(NodeIndex) ? Nodes[NodeIndex].Centroid : FVector::ZeroVector;
	}

	double FBox3DStructureGraph::GetNodeVolume(int32 NodeIndex) const
	{
		return Nodes.IsValidIndex(NodeIndex) ? Nodes[NodeIndex].Volume : 0.0;
	}

	int32 FBox3DStructureGraph::FindBond(int32 NodeA, int32 NodeB) const
	{
		if (!Nodes.IsValidIndex(NodeA) || !Nodes.IsValidIndex(NodeB) || NodeA == NodeB)
		{
			return INDEX_NONE;
		}
		for (const int32 BondIndex : Nodes[NodeA].BondIndices)
		{
			const FBox3DStructureBond& Bond = Bonds[BondIndex];
			if (Bond.NodeA + Bond.NodeB == NodeA + NodeB)
			{
				return BondIndex;
			}
		}
		return INDEX_NONE;
	}

	void FBox3DStructureGraph::SetAnchor(int32 NodeIndex, bool bAnchor)
	{
		if (Nodes.IsValidIndex(NodeIndex))
		{
			Nodes[NodeIndex].bAnchor = bAnchor;
		}
	}

	bool FBox3DStructureGraph::IsAnchor(int32 NodeIndex) const
	{
		return Nodes.IsValidIndex(NodeIndex) && Nodes[NodeIndex].bAnchor;
	}

	int32 FBox3DStructureGraph::GetAnchorCount() const
	{
		int32 Count = 0;
		for (const FNode& Node : Nodes)
		{
			Count += Node.bAnchor ? 1 : 0;
		}
		return Count;
	}

	bool FBox3DStructureGraph::IsChunkDestroyed(int32 NodeIndex) const
	{
		return Nodes.IsValidIndex(NodeIndex) && Nodes[NodeIndex].bDestroyed;
	}

	void FBox3DStructureGraph::NotifyChunkDestroyed(int32 NodeIndex, FBox3DStructureIslands& OutIslands)
	{
		if (!Nodes.IsValidIndex(NodeIndex) || Nodes[NodeIndex].bDestroyed)
		{
			return;
		}
		FNode& Node = Nodes[NodeIndex];
		Node.bDestroyed = true;

		// BondIndices is ascending by neighbor index, so the seeds are too.
		TArray<int32, TInlineAllocator<8>> Seeds;
		for (const int32 BondIndex : Node.BondIndices)
		{
			FBox3DStructureBond& Bond = Bonds[BondIndex];
			if (Bond.bBroken)
			{
				continue;
			}
			Bond.bBroken = true;
			Bond.Health = 0.0f;
			const int32 Other = OtherNode(BondIndex, NodeIndex);
			if (!Nodes[Other].bDestroyed)
			{
				Seeds.Add(Other);
			}
		}
		FloodFrom(Seeds, OutIslands);
	}

	void FBox3DStructureGraph::NotifyBondBroken(int32 BondIndex, FBox3DStructureIslands& OutIslands)
	{
		if (!Bonds.IsValidIndex(BondIndex) || Bonds[BondIndex].bBroken)
		{
			return;
		}
		FBox3DStructureBond& Bond = Bonds[BondIndex];
		Bond.bBroken = true;
		Bond.Health = 0.0f;

		TArray<int32, TInlineAllocator<2>> Seeds;
		for (const int32 Endpoint : { Bond.NodeA, Bond.NodeB })
		{
			if (!Nodes[Endpoint].bDestroyed)
			{
				Seeds.Add(Endpoint);
			}
		}
		FloodFrom(Seeds, OutIslands);
	}

	void FBox3DStructureGraph::FloodFrom(TConstArrayView<int32> Seeds, FBox3DStructureIslands& OutIslands)
	{
		// Stamps at or above this value mark nodes some flood of THIS update
		// touched; older stamps read as unvisited. Touching another flood's
		// region proves support: unsupported islands are enumerated exhaustively,
		// so every node adjacent to an island was absorbed into it — a region a
		// later flood can reach is necessarily a supported one.
		const uint32 UpdateFirstStamp = CurrentStamp + 1;

		TArray<int32> Queue;
		TArray<int32> Region;
		for (const int32 Seed : Seeds)
		{
			FNode& SeedNode = Nodes[Seed];
			if (SeedNode.bDestroyed || SeedNode.VisitStamp >= UpdateFirstStamp)
			{
				continue;
			}
			const uint32 Stamp = ++CurrentStamp;
			SeedNode.VisitStamp = Stamp;
			Queue.Reset();
			Queue.Add(Seed);
			Region.Reset();

			bool bSupported = false;
			for (int32 Head = 0; Head < Queue.Num() && !bSupported; ++Head)
			{
				const int32 NodeIndex = Queue[Head];
				const FNode& Node = Nodes[NodeIndex];
				++OutIslands.VisitCount;
				if (Node.bAnchor)
				{
					bSupported = true;
					break;
				}
				Region.Add(NodeIndex);

				for (const int32 BondIndex : Node.BondIndices)
				{
					if (Bonds[BondIndex].bBroken)
					{
						continue;
					}
					FNode& Other = Nodes[OtherNode(BondIndex, NodeIndex)];
					if (Other.bDestroyed || Other.VisitStamp == Stamp)
					{
						continue;
					}
					if (Other.VisitStamp >= UpdateFirstStamp)
					{
						bSupported = true;
						break;
					}
					Other.VisitStamp = Stamp;
					Queue.Add(OtherNode(BondIndex, NodeIndex));
				}
			}

			if (!bSupported)
			{
				// The BFS exhausted without proving support, so Region is the
				// entire connected component — an island.
				Region.Sort();
				OutIslands.Islands.Add(Region);
			}
		}
	}

	int32 DetectAnchors(FBox3DStructureGraph& Graph, b3WorldId WorldId,
		TConstArrayView<b3BodyId> NodeBodies, float MarginCm)
	{
		if (!b3World_IsValid(WorldId))
		{
			return 0;
		}

		// The assembly's own bodies never anchor it — relevant once slice 2 keeps
		// unbroken chunks static (a static sibling must not read as ground).
		TSet<uint64> OwnBodies;
		for (const b3BodyId Body : NodeBodies)
		{
			if (b3Body_IsValid(Body))
			{
				OwnBodies.Add(b3StoreBodyId(Body));
			}
		}

		struct FAnchorContext
		{
			const TSet<uint64>* OwnBodies;
			bool bAnchored;
		};

		const float MarginM = FMath::Max(MarginCm, 0.0f) * UEToMeters;
		b3QueryFilter Filter = b3DefaultQueryFilter();
		Filter.categoryBits = UINT64_MAX;
		Filter.maskBits = UINT64_MAX;

		int32 AnchorCount = 0;
		const int32 Count = FMath::Min(Graph.GetNodeCount(), NodeBodies.Num());
		for (int32 Index = 0; Index < Count; ++Index)
		{
			const b3BodyId Body = NodeBodies[Index];
			if (!b3Body_IsValid(Body))
			{
				continue;
			}

			b3AABB Bounds = b3Body_ComputeAABB(Body);
			Bounds.lowerBound.x -= MarginM;
			Bounds.lowerBound.y -= MarginM;
			Bounds.lowerBound.z -= MarginM;
			Bounds.upperBound.x += MarginM;
			Bounds.upperBound.y += MarginM;
			Bounds.upperBound.z += MarginM;

			FAnchorContext Context{ &OwnBodies, false };
			b3World_OverlapAABB(WorldId, Bounds, Filter,
				[](b3ShapeId ShapeId, void* RawContext) -> bool
				{
					FAnchorContext* Context = static_cast<FAnchorContext*>(RawContext);
					const b3BodyId HitBody = b3Shape_GetBody(ShapeId);
					if (b3Body_GetType(HitBody) == b3_staticBody
						&& !Context->OwnBodies->Contains(b3StoreBodyId(HitBody)))
					{
						Context->bAnchored = true;
						return false; // one static contact is enough
					}
					return true;
				},
				&Context);

			if (Context.bAnchored)
			{
				Graph.SetAnchor(Index, true);
				++AnchorCount;
			}
		}
		return AnchorCount;
	}
}
