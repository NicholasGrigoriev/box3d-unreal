#pragma once

#include "CoreMinimal.h"
#include "Box3DFracture.h"
#include "box3d/id.h"

/// D4 structural connectivity core — the bond graph of one destructible
/// assembly (docs/DESTRUCTION_PLAN.md § D4). Nodes are chunks (D1 fragments or
/// authored breakable chunks), bonds are shared-face links carrying area and
/// health, anchors are chunks attached to the immovable world. Connectivity is
/// event-driven: nothing is polled — on chunk-destroyed / bond-broken the graph
/// flood-fills only the affected neighborhood and reports the islands that lost
/// their last path to an anchor. Island promotion (static -> dynamic handover)
/// is the consumer's job, not the graph's.
///
/// Everything except DetectAnchors is pure data — no physics world, no actors —
/// so connectivity is testable headlessly and deterministic: fixed build order,
/// ascending seed order, FIFO flood-fill, sorted island output.
namespace Box3D::Structure
{
	/// One shared-face link between two chunks. Alive until bBroken.
	struct FBox3DStructureBond
	{
		/// Chunk indices, NodeA < NodeB (canonical pair order).
		int32 NodeA = INDEX_NONE;
		int32 NodeB = INDEX_NONE;

		/// Shared-face area (cm^2) from fracture adjacency — the bond's strength
		/// scale (weld break forces and D5 stress capacity derive from it).
		double Area = 0.0;

		/// Bond midpoint and canonical normal in chunk-local centimetres. Normal
		/// points NodeA -> NodeB; D5 uses it to split axial and shear load.
		FVector Centroid = FVector::ZeroVector;
		FVector Normal = FVector::ZeroVector;

		/// Remaining health fraction in [0, 1]. D4 events snap it to zero; D5
		/// stress overload erodes it gradually.
		float Health = 1.0f;

		bool bBroken = false;
	};

	/// Result of one connectivity update.
	struct FBox3DStructureIslands
	{
		/// Chunk groups left with no live path to any anchor, each sorted
		/// ascending; groups ordered by their smallest member. The graph does not
		/// modify island nodes — promotion decides their fate.
		TArray<TArray<int32>> Islands;

		/// Nodes the flood-fill dequeued. The locality bound for tests: an update
		/// next to anchors must not walk the whole graph.
		int32 VisitCount = 0;
	};

	/// Bond graph of one destructible assembly.
	///
	/// Support rule: a chunk is supported while a path of unbroken bonds through
	/// non-destroyed chunks reaches any anchor. Updates flood outward from the
	/// event's neighborhood only, stopping a region's flood early the moment it
	/// proves support (an anchor, or a region already proven supported this
	/// update); only genuinely unsupported islands are walked exhaustively —
	/// their members must be enumerated for promotion anyway.
	class BOX3DRUNTIME_API FBox3DStructureGraph
	{
	public:
		/// Build nodes and bonds from fracture adjacency: one node per fragment,
		/// one bond per neighbor pair (lower index owns the pair; Neighbors is
		/// symmetric with canonical areas). No anchors yet — call SetAnchor or
		/// DetectAnchors after. Replaces any previous graph.
		void Build(TConstArrayView<Fracture::FBox3DFragmentData> Fragments);

		void Reset();

		int32 GetNodeCount() const { return Nodes.Num(); }
		int32 GetBondCount() const { return Bonds.Num(); }
		int32 GetLiveBondCount() const;

		/// Bond index linking the two chunks (either argument order), or
		/// INDEX_NONE. Broken bonds are still found — check bBroken.
		int32 FindBond(int32 NodeA, int32 NodeB) const;

		const FBox3DStructureBond& GetBond(int32 BondIndex) const { return Bonds[BondIndex]; }
		FVector GetNodeCentroid(int32 NodeIndex) const;
		double GetNodeVolume(int32 NodeIndex) const;

		void SetAnchor(int32 NodeIndex, bool bAnchor);
		bool IsAnchor(int32 NodeIndex) const;
		int32 GetAnchorCount() const;
		bool IsChunkDestroyed(int32 NodeIndex) const;

		/// The chunk is gone (fractured away, dissolved): mark it destroyed, snap
		/// all its live bonds, then flood from its surviving neighbors. No-op on
		/// an already-destroyed or invalid index.
		void NotifyChunkDestroyed(int32 NodeIndex, FBox3DStructureIslands& OutIslands);

		/// A bond snapped (weld break, D5 overload): mark it broken and flood
		/// from both endpoints. No-op on an already-broken or invalid index.
		void NotifyBondBroken(int32 BondIndex, FBox3DStructureIslands& OutIslands);

	private:
		struct FNode
		{
			FVector Centroid = FVector::ZeroVector;
			double Volume = 0.0;
			bool bDestroyed = false;
			bool bAnchor = false;

			/// Marks the flood that last touched this node (see FloodFrom).
			uint32 VisitStamp = 0;

			/// Bond indices in build order (ascending neighbor index).
			TArray<int32, TInlineAllocator<6>> BondIndices;
		};

		/// Flood the affected neighborhood from event seeds (ascending order).
		/// Per-seed BFS over live bonds with a fresh stamp; a region stops the
		/// moment it dequeues an anchor or touches a node another flood of this
		/// update stamped (touchable regions are always supported: unsupported
		/// islands are walked exhaustively, so anything adjacent to them was
		/// already absorbed into them). Regions that exhaust unsupported are
		/// reported as islands.
		void FloodFrom(TConstArrayView<int32> Seeds, FBox3DStructureIslands& OutIslands);

		int32 OtherNode(int32 BondIndex, int32 NodeIndex) const
		{
			const FBox3DStructureBond& Bond = Bonds[BondIndex];
			return Bond.NodeA == NodeIndex ? Bond.NodeB : Bond.NodeA;
		}

		TArray<FNode> Nodes;
		TArray<FBox3DStructureBond> Bonds;

		/// Monotonic flood stamp. Never reset per update, so stale stamps from
		/// earlier updates read as unvisited without clearing every node.
		uint32 CurrentStamp = 0;
	};

	/// Auto-anchor detection: overlap each chunk body's AABB, inflated by
	/// MarginCm, against the b3 world; chunks touching any *static* body
	/// (mirror, baked, or static body components) become anchors. Conservative
	/// broad-phase test by design — resting contact with a small margin is what
	/// anchoring means here. Invalid body ids (visual-only chunks) are skipped.
	/// NodeBodies is parallel to the graph's nodes. Returns the anchor count.
	BOX3DRUNTIME_API int32 DetectAnchors(FBox3DStructureGraph& Graph, b3WorldId WorldId,
		TConstArrayView<b3BodyId> NodeBodies, float MarginCm = 2.0f);
}
