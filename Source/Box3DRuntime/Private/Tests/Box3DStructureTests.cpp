// Tests for D4 structural connectivity: bond graph build from fracture
// adjacency, anchor auto-detection against mirrored static geometry, the
// event-driven flood-fill that finds unsupported islands while touching only
// the affected neighborhood (visit-counter bound), and two-phase
// static->dynamic island promotion on ABox3DFracturedActor (bridge and tower
// scenarios, per-tick promotion budget with FIFO overflow).

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DFracture.h"
#include "Box3DFracturedActor.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DStructure.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	using namespace Box3D::Fracture;
	using namespace Box3D::Structure;

	/// N chunks in a row, each bonded to the next — pure adjacency, no geometry.
	TArray<FBox3DFragmentData> MakeChainFragments(int32 Count, double Area = 100.0)
	{
		TArray<FBox3DFragmentData> Fragments;
		Fragments.SetNum(Count);
		for (int32 Index = 0; Index + 1 < Count; ++Index)
		{
			Fragments[Index].Neighbors.Add({ Index + 1, Area });
			Fragments[Index + 1].Neighbors.Add({ Index, Area });
		}
		return Fragments;
	}

	/// A 50 cm cube chunk centered at Center — hand-made fragment geometry for
	/// structural assemblies whose shape the test controls exactly (no Voronoi
	/// randomness). No faces: promotion tests need bodies and bonds, not
	/// rendering.
	FBox3DFragmentData MakeBoxChunk(const FVector& Center, double Half = 25.0)
	{
		FBox3DFragmentData Fragment;
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			Fragment.Vertices.Add(Center
				+ FVector((Corner & 1) ? Half : -Half, (Corner & 2) ? Half : -Half, (Corner & 4) ? Half : -Half));
		}
		Fragment.Centroid = Center;
		Fragment.Volume = 8.0 * Half * Half * Half;
		return Fragment;
	}

	/// Symmetric bond between two chunks (full 50x50 cm shared face by default).
	void BondChunks(TArray<FBox3DFragmentData>& Fragments, int32 A, int32 B, double Area = 2500.0)
	{
		Fragments[A].Neighbors.Add({ B, Area });
		Fragments[B].Neighbors.Add({ A, Area });
	}

	/// Spawn a structural fractured actor from hand-made chunks at the world
	/// origin (chunk space = world space).
	ABox3DFracturedActor* SpawnStructuralActor(UWorld* World, TArray<FBox3DFragmentData>&& Fragments)
	{
		ABox3DFracturedActor* Actor = World->SpawnActor<ABox3DFracturedActor>();
		Actor->bStructural = true;
		Actor->InitializeFragments(MoveTemp(Fragments), nullptr);
		return Actor;
	}

	bool ChunkIsStatic(const ABox3DFracturedActor& Actor, int32 Index)
	{
		const b3BodyId Body = Actor.GetFragmentBody(Index);
		return b3Body_IsValid(Body) && b3Body_GetType(Body) == b3_staticBody;
	}

	bool ChunkIsDynamic(const ABox3DFracturedActor& Actor, int32 Index)
	{
		const b3BodyId Body = Actor.GetFragmentBody(Index);
		return b3Body_IsValid(Body) && b3Body_GetType(Body) == b3_dynamicBody;
	}

	/// Mirrored ground plate with its top face at Z = 0 — the anchor surface for
	/// structural assemblies (same pattern as the anchor-detect test).
	void MirrorGroundPlate(Box3DTest::FTestWorld& Test)
	{
		Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
			FTransform(FQuat::Identity, FVector(0, 0, -50), FVector(8.0, 8.0, 1.0)));
		Test.Subsystem().GetStaticMirror()->MirrorLevel(Test.World->PersistentLevel);
	}

	/// Three chunks, every pair bonded.
	TArray<FBox3DFragmentData> MakeTriangleFragments(double Area = 100.0)
	{
		TArray<FBox3DFragmentData> Fragments;
		Fragments.SetNum(3);
		for (int32 A = 0; A < 3; ++A)
		{
			for (int32 B = 0; B < 3; ++B)
			{
				if (A != B)
				{
					Fragments[A].Neighbors.Add({ B, Area });
				}
			}
		}
		return Fragments;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStructureGraphBuildTest,
	"Box3DUnreal.Structure.GraphBuild", BOX3D_TEST_FLAGS)
bool FBox3DStructureGraphBuildTest::RunTest(const FString& Parameters)
{
	// Real fracture adjacency is the graph's raw material.
	const FFractureProxy Proxy = MakeBoxProxy(FVector::ZeroVector, FVector(50.0));
	FFractureParams Params;
	Params.Seed = 7;
	Params.CellCount = 8;
	TArray<FBox3DFragmentData> Fragments;
	TestTrue(TEXT("fracture succeeds"), Fracture(Proxy, Params, Fragments));
	TestTrue(TEXT("multiple fragments"), Fragments.Num() > 1);

	FBox3DStructureGraph Graph;
	Graph.Build(Fragments);

	TestEqual(TEXT("one node per fragment"), Graph.GetNodeCount(), Fragments.Num());

	int32 NeighborLinks = 0;
	bool bAllPairsBonded = true;
	bool bAreasMatch = true;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		for (const FBox3DFragmentNeighbor& Neighbor : Fragments[Index].Neighbors)
		{
			++NeighborLinks;
			const int32 BondIndex = Graph.FindBond(Index, Neighbor.FragmentIndex);
			bAllPairsBonded &= BondIndex != INDEX_NONE;
			// Same bond from either argument order.
			bAllPairsBonded &= Graph.FindBond(Neighbor.FragmentIndex, Index) == BondIndex;
			if (BondIndex != INDEX_NONE)
			{
				const FBox3DStructureBond& Bond = Graph.GetBond(BondIndex);
				bAreasMatch &= FMath::IsNearlyEqual(Bond.Area, Neighbor.SharedFaceArea, UE_KINDA_SMALL_NUMBER);
				bAreasMatch &= Bond.NodeA < Bond.NodeB;
			}
		}
	}
	TestEqual(TEXT("one bond per symmetric neighbor pair"), Graph.GetBondCount(), NeighborLinks / 2);
	TestTrue(TEXT("every adjacency pair has a bond, canonical order"), bAllPairsBonded);
	TestTrue(TEXT("bond areas carry the canonical shared-face areas"), bAreasMatch);
	TestEqual(TEXT("all bonds live after build"), Graph.GetLiveBondCount(), Graph.GetBondCount());
	TestEqual(TEXT("no anchors before detection"), Graph.GetAnchorCount(), 0);
	TestEqual(TEXT("no bond between non-neighbors"), Graph.FindBond(0, 0), (int32)INDEX_NONE);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStructureConnectivityTest,
	"Box3DUnreal.Structure.ConnectivityEvents", BOX3D_TEST_FLAGS)
bool FBox3DStructureConnectivityTest::RunTest(const FString& Parameters)
{
	// Chain 0-1-2-3-4 anchored at 0: cutting 1-2 strands exactly {2,3,4}.
	{
		FBox3DStructureGraph Graph;
		Graph.Build(MakeChainFragments(5));
		Graph.SetAnchor(0, true);

		FBox3DStructureIslands Out;
		Graph.NotifyBondBroken(Graph.FindBond(1, 2), Out);
		TestEqual(TEXT("one island"), Out.Islands.Num(), 1);
		if (Out.Islands.Num() == 1)
		{
			TestTrue(TEXT("island is the unanchored side"), Out.Islands[0] == TArray<int32>({ 2, 3, 4 }));
		}
		TestEqual(TEXT("3 of 4 bonds live"), Graph.GetLiveBondCount(), 3);
		// Anchored side: 1 then anchor 0 = 2 visits; island side exhausts = 3.
		TestEqual(TEXT("flood visited both sides once"), Out.VisitCount, 5);

		// Re-breaking a broken bond is a no-op.
		FBox3DStructureIslands Again;
		Graph.NotifyBondBroken(Graph.FindBond(1, 2), Again);
		TestEqual(TEXT("no islands from a dead bond"), Again.Islands.Num(), 0);
		TestEqual(TEXT("no flood from a dead bond"), Again.VisitCount, 0);

		// Destroying 3 inside the strand splits it; the dead chunk is not reported.
		FBox3DStructureIslands Split;
		Graph.NotifyChunkDestroyed(3, Split);
		TestTrue(TEXT("chunk marked destroyed"), Graph.IsChunkDestroyed(3));
		TestEqual(TEXT("two islands after mid-strand destroy"), Split.Islands.Num(), 2);
		if (Split.Islands.Num() == 2)
		{
			TestTrue(TEXT("left island"), Split.Islands[0] == TArray<int32>({ 2 }));
			TestTrue(TEXT("right island"), Split.Islands[1] == TArray<int32>({ 4 }));
		}
		TestEqual(TEXT("destroy snapped both bonds of the chunk"), Graph.GetLiveBondCount(), 1);

		FBox3DStructureIslands Dead;
		Graph.NotifyChunkDestroyed(3, Dead);
		TestEqual(TEXT("re-destroying is a no-op"), Dead.VisitCount, 0);
	}

	// Redundant path: triangle anchored at 0 — cutting 1-2 strands nothing.
	{
		FBox3DStructureGraph Graph;
		Graph.Build(MakeTriangleFragments());
		Graph.SetAnchor(0, true);

		FBox3DStructureIslands Out;
		Graph.NotifyBondBroken(Graph.FindBond(1, 2), Out);
		TestEqual(TEXT("redundant bond loss strands nothing"), Out.Islands.Num(), 0);
	}

	// No anchors anywhere: every severed component is an island.
	{
		FBox3DStructureGraph Graph;
		Graph.Build(MakeChainFragments(2));

		FBox3DStructureIslands Out;
		Graph.NotifyBondBroken(Graph.FindBond(0, 1), Out);
		TestEqual(TEXT("both halves unsupported"), Out.Islands.Num(), 2);
	}

	// A chunk with no neighbors: destroying it floods nothing.
	{
		FBox3DStructureGraph Graph;
		Graph.Build(MakeChainFragments(1));

		FBox3DStructureIslands Out;
		Graph.NotifyChunkDestroyed(0, Out);
		TestEqual(TEXT("no islands from an isolated chunk"), Out.Islands.Num(), 0);
		TestEqual(TEXT("no flood from an isolated chunk"), Out.VisitCount, 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStructureFloodBoundTest,
	"Box3DUnreal.Structure.FloodFillBound", BOX3D_TEST_FLAGS)
bool FBox3DStructureFloodBoundTest::RunTest(const FString& Parameters)
{
	// A long row where every chunk touches ground (all anchored, e.g. a wall's
	// base course): events in the middle must not walk the whole graph.
	{
		constexpr int32 Count = 1000;
		FBox3DStructureGraph Graph;
		Graph.Build(MakeChainFragments(Count));
		for (int32 Index = 0; Index < Count; ++Index)
		{
			Graph.SetAnchor(Index, true);
		}

		FBox3DStructureIslands Destroy;
		Graph.NotifyChunkDestroyed(Count / 2, Destroy);
		TestEqual(TEXT("anchored row loses nothing"), Destroy.Islands.Num(), 0);
		TestEqual(TEXT("destroy floods only the two seed neighbors"), Destroy.VisitCount, 2);

		FBox3DStructureIslands Break;
		Graph.NotifyBondBroken(Graph.FindBond(10, 11), Break);
		TestEqual(TEXT("bond break floods only the two endpoints"), Break.VisitCount, 2);
	}

	// Cross-flood early-out: once one flood proves a region supported, a later
	// seed touching that region stops immediately instead of re-walking it.
	{
		FBox3DStructureGraph Graph;
		Graph.Build(MakeTriangleFragments());
		Graph.SetAnchor(2, true);

		FBox3DStructureIslands Out;
		Graph.NotifyBondBroken(Graph.FindBond(0, 1), Out);
		TestEqual(TEXT("triangle stays supported"), Out.Islands.Num(), 0);
		// Seed 0: visits 0 then anchor 2. Seed 1: visits 1, touches the region
		// flood 0 stamped, stops. 3 total.
		TestEqual(TEXT("second flood reuses the first flood's proof"), Out.VisitCount, 3);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStructureAnchorDetectTest,
	"Box3DUnreal.Structure.AnchorAutoDetect", BOX3D_TEST_FLAGS)
bool FBox3DStructureAnchorDetectTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();
	TestNotNull(TEXT("cube mesh loads"), Cube);

	// Mirrored ground with its top at Z = 0, a cube resting on it, and a cube
	// floating far above. Mirror first so FractureMesh swaps the sources out.
	Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(0, 0, -50), FVector(8.0, 8.0, 1.0)));
	UStaticMeshComponent* Resting = Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(0, 0, 50)));
	UStaticMeshComponent* Floating = Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(500, 0, 500)));

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	TestNotNull(TEXT("mirror enabled"), Mirror);
	Mirror->MirrorLevel(Test.World->PersistentLevel);

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 11;
	Params.Fracture.CellCount = 12;
	ABox3DFracturedActor* RestingActor = Box3D::FractureMesh(Resting, Params);
	ABox3DFracturedActor* FloatingActor = Box3D::FractureMesh(Floating, Params);
	TestNotNull(TEXT("resting cube fractures"), RestingActor);
	TestNotNull(TEXT("floating cube fractures"), FloatingActor);
	if (RestingActor == nullptr || FloatingActor == nullptr)
	{
		return false;
	}

	const auto GatherBodies = [](const ABox3DFracturedActor& Actor, TArray<b3BodyId>& OutBodies)
	{
		OutBodies.Reset();
		for (int32 Index = 0; Index < Actor.GetFragmentCount(); ++Index)
		{
			OutBodies.Add(Actor.GetFragmentBody(Index));
		}
	};

	// Resting cube: exactly the fragments whose geometry reaches the ground
	// plane anchor. Detection is AABB-based with a 2 cm margin, so assert with
	// slack bands and skip the ambiguous sliver in between.
	{
		FBox3DStructureGraph Graph;
		Graph.Build(RestingActor->GetFragments());
		TArray<b3BodyId> Bodies;
		GatherBodies(*RestingActor, Bodies);
		const int32 AnchorCount = Box3D::Structure::DetectAnchors(Graph, Test.B3World(), Bodies, 2.0f);

		TestEqual(TEXT("return value matches graph anchors"), AnchorCount, Graph.GetAnchorCount());
		TestTrue(TEXT("some fragments touch ground"), AnchorCount > 0);
		TestTrue(TEXT("not every fragment touches ground"), AnchorCount < Graph.GetNodeCount());

		const FTransform ActorToWorld = RestingActor->GetActorTransform();
		bool bBandsRespected = true;
		int32 HighFragments = 0;
		for (int32 Index = 0; Index < RestingActor->GetFragmentCount(); ++Index)
		{
			double MinZ = TNumericLimits<double>::Max();
			for (const FVector& Vertex : RestingActor->GetFragments()[Index].Vertices)
			{
				MinZ = FMath::Min(MinZ, ActorToWorld.TransformPosition(Vertex).Z);
			}
			if (MinZ <= 0.5)
			{
				bBandsRespected &= Graph.IsAnchor(Index);
			}
			else if (MinZ > 10.0)
			{
				++HighFragments;
				bBandsRespected &= !Graph.IsAnchor(Index);
			}
		}
		TestTrue(TEXT("ground-touching fragments anchored, high fragments not"), bBandsRespected);
		TestTrue(TEXT("layout has clearly-off-ground fragments"), HighFragments > 0);
	}

	// Floating cube: nothing touches static geometry, nothing anchors — its own
	// dynamic sibling fragments must not count.
	{
		FBox3DStructureGraph Graph;
		Graph.Build(FloatingActor->GetFragments());
		TArray<b3BodyId> Bodies;
		GatherBodies(*FloatingActor, Bodies);
		TestEqual(TEXT("floating assembly has no anchors"),
			Box3D::Structure::DetectAnchors(Graph, Test.B3World(), Bodies, 2.0f), 0);
		TestEqual(TEXT("graph agrees"), Graph.GetAnchorCount(), 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStructureBridgeTest,
	"Box3DUnreal.Structure.BridgePromotion", BOX3D_TEST_FLAGS)
bool FBox3DStructureBridgeTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FScopedDestructionSettings DestructionSettings(0, 0.0f);
	Box3DTest::FScopedPromotionBudget PromotionBudget(0);
	Box3DTest::FTestWorld Test;
	MirrorGroundPlate(Test);

	// Two anchored piers with a span across them, 50 cm cube chunks:
	//   0 = pier A base (on ground), 1 = pier A top,
	//   2..4 = span, 5 = pier B top, 6 = pier B base (on ground).
	TArray<FBox3DFragmentData> Chunks;
	Chunks.Add(MakeBoxChunk(FVector(0, 0, 25)));
	Chunks.Add(MakeBoxChunk(FVector(0, 0, 75)));
	Chunks.Add(MakeBoxChunk(FVector(50, 0, 75)));
	Chunks.Add(MakeBoxChunk(FVector(100, 0, 75)));
	Chunks.Add(MakeBoxChunk(FVector(150, 0, 75)));
	Chunks.Add(MakeBoxChunk(FVector(200, 0, 75)));
	Chunks.Add(MakeBoxChunk(FVector(200, 0, 25)));
	for (int32 Index = 0; Index < 6; ++Index)
	{
		BondChunks(Chunks, Index, Index + 1);
	}

	ABox3DFracturedActor* Bridge = SpawnStructuralActor(Test.World, MoveTemp(Chunks));
	TestTrue(TEXT("structural mode live"), Bridge->IsStructureActive());
	TestTrue(TEXT("pier bases anchored"),
		Bridge->GetStructureGraph().IsAnchor(0) && Bridge->GetStructureGraph().IsAnchor(6));
	TestEqual(TEXT("only the bases anchor"), Bridge->GetStructureGraph().GetAnchorCount(), 2);
	bool bAllStatic = true;
	for (int32 Index = 0; Index < 7; ++Index)
	{
		bAllStatic &= ChunkIsStatic(*Bridge, Index);
	}
	TestTrue(TEXT("anchored assembly starts fully static"), bAllStatic);

	// Destroy pier A's top chunk: the span is still carried by pier B —
	// redundant support, nothing promotes.
	Bridge->DestroyFragment(1);
	TestEqual(TEXT("redundant support queues nothing"), Bridge->GetPendingPromotionCount(), 0);
	Bridge->ProcessPromotions();
	TestTrue(TEXT("span still static via pier B"),
		ChunkIsStatic(*Bridge, 2) && ChunkIsStatic(*Bridge, 3) && ChunkIsStatic(*Bridge, 4));
	TestFalse(TEXT("destroyed chunk's body is gone"), b3Body_IsValid(Bridge->GetFragmentBody(1)));

	// Destroy pier B's top chunk: the span loses its last anchor path — exactly
	// the island {2,3,4} promotes; the anchored piers stay static.
	Bridge->DestroyFragment(5);
	TestEqual(TEXT("exactly the span is queued"), Bridge->GetPendingPromotionCount(), 3);
	Bridge->ProcessPromotions();
	TestTrue(TEXT("span island promoted to dynamic"),
		ChunkIsDynamic(*Bridge, 2) && ChunkIsDynamic(*Bridge, 3) && ChunkIsDynamic(*Bridge, 4));
	TestTrue(TEXT("anchored side stays static"), ChunkIsStatic(*Bridge, 0) && ChunkIsStatic(*Bridge, 6));
	TestEqual(TEXT("queue drained"), Bridge->GetPendingPromotionCount(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStructureTowerTest,
	"Box3DUnreal.Structure.TowerPromotion", BOX3D_TEST_FLAGS)
bool FBox3DStructureTowerTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FScopedDestructionSettings DestructionSettings(0, 0.0f);
	Box3DTest::FScopedPromotionBudget PromotionBudget(0);
	Box3DTest::FTestWorld Test;
	MirrorGroundPlate(Test);

	// Column of four chunks, only the base on the ground.
	TArray<FBox3DFragmentData> Chunks;
	for (int32 Index = 0; Index < 4; ++Index)
	{
		Chunks.Add(MakeBoxChunk(FVector(0, 0, 25 + Index * 50)));
		if (Index > 0)
		{
			BondChunks(Chunks, Index - 1, Index);
		}
	}

	ABox3DFracturedActor* Tower = SpawnStructuralActor(Test.World, MoveTemp(Chunks));
	TestTrue(TEXT("structural mode live"), Tower->IsStructureActive());
	TestEqual(TEXT("only the base anchors"), Tower->GetStructureGraph().GetAnchorCount(), 1);
	TestTrue(TEXT("base is the anchor"), Tower->GetStructureGraph().IsAnchor(0));

	// Destroy the base: the whole remaining column strands in one event.
	Tower->DestroyFragment(0);
	TestEqual(TEXT("whole column queued by one event"), Tower->GetPendingPromotionCount(), 3);
	Tower->ProcessPromotions();
	TestTrue(TEXT("whole column promoted"),
		ChunkIsDynamic(*Tower, 1) && ChunkIsDynamic(*Tower, 2) && ChunkIsDynamic(*Tower, 3));
	TestEqual(TEXT("queue drained"), Tower->GetPendingPromotionCount(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStructurePromotionBudgetTest,
	"Box3DUnreal.Structure.PromotionBudget", BOX3D_TEST_FLAGS)
bool FBox3DStructurePromotionBudgetTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FScopedDestructionSettings DestructionSettings(0, 0.0f);
	Box3DTest::FScopedPromotionBudget PromotionBudget(1);
	Box3DTest::FTestWorld Test;
	MirrorGroundPlate(Test);

	// Five-chunk tower; destroying the base queues four promotions against a
	// budget of one per pump.
	TArray<FBox3DFragmentData> Chunks;
	for (int32 Index = 0; Index < 5; ++Index)
	{
		Chunks.Add(MakeBoxChunk(FVector(0, 0, 25 + Index * 50)));
		if (Index > 0)
		{
			BondChunks(Chunks, Index - 1, Index);
		}
	}

	ABox3DFracturedActor* Tower = SpawnStructuralActor(Test.World, MoveTemp(Chunks));
	TestTrue(TEXT("structural mode live"), Tower->IsStructureActive());
	Tower->DestroyFragment(0);
	TestEqual(TEXT("four chunks queued"), Tower->GetPendingPromotionCount(), 4);

	// FIFO under budget: exactly one chunk per pump, in island (ascending) order.
	for (int32 Pump = 1; Pump <= 4; ++Pump)
	{
		Tower->ProcessPromotions();
		TestEqual(FString::Printf(TEXT("pump %d leaves %d queued"), Pump, 4 - Pump),
			Tower->GetPendingPromotionCount(), 4 - Pump);
		bool bOrderRespected = true;
		for (int32 Index = 1; Index <= 4; ++Index)
		{
			bOrderRespected &= Index <= Pump ? ChunkIsDynamic(*Tower, Index) : ChunkIsStatic(*Tower, Index);
		}
		TestTrue(FString::Printf(TEXT("pump %d promoted the FIFO head only"), Pump), bOrderRespected);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
