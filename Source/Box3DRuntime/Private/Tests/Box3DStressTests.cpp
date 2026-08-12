// D5 stress-core tests: analytic cantilever equilibrium, axial/shear load
// decomposition, deterministic coarsening, and quantized bond-state hashing.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DFracture.h"
#include "Box3DFracturedActor.h"
#include "Box3DQueryLibrary.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DStress.h"
#include "Tests/Box3DTestEventCounter.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	using namespace Box3D::Fracture;
	using namespace Box3D::Structure;

	TArray<FBox3DFragmentData> MakeBeam(int32 Count, double SpacingCm = 100.0)
	{
		TArray<FBox3DFragmentData> Chunks;
		Chunks.SetNum(Count);
		for (int32 Index = 0; Index < Count; ++Index)
		{
			Chunks[Index].Centroid = FVector(Index * SpacingCm, 0.0, 0.0);
			Chunks[Index].Volume = 1.0;
			if (Index > 0)
			{
				Chunks[Index - 1].Neighbors.Add({ Index, 100.0 });
				Chunks[Index].Neighbors.Add({ Index - 1, 100.0 });
			}
		}
		return Chunks;
	}

	FBox3DStructureGraph MakeCantileverGraph(int32 Count)
	{
		FBox3DStructureGraph Graph;
		Graph.Build(MakeBeam(Count));
		Graph.SetAnchor(0, true);
		return Graph;
	}

	FBox3DFragmentData MakeStressBoxChunk(const FVector& Center, double Half = 25.0)
	{
		FBox3DFragmentData Fragment;
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			Fragment.Vertices.Add(Center + FVector((Corner & 1) ? Half : -Half,
				(Corner & 2) ? Half : -Half, (Corner & 4) ? Half : -Half));
		}
		Fragment.Centroid = Center;
		Fragment.Volume = 8.0 * Half * Half * Half;
		return Fragment;
	}

	void BondStressChunks(TArray<FBox3DFragmentData>& Chunks, int32 A, int32 B, double Area = 2500.0)
	{
		Chunks[A].Neighbors.Add({ B, Area });
		Chunks[B].Neighbors.Add({ A, Area });
	}

	void MirrorGround(Box3DTest::FTestWorld& Test)
	{
		Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
			FTransform(FQuat::Identity, FVector(0, 0, -50), FVector(8.0, 8.0, 1.0)));
		Test.Subsystem().GetStaticMirror()->MirrorLevel(Test.World->PersistentLevel);
	}

	ABox3DFracturedActor* SpawnStressTower(UWorld* World, int32 Count)
	{
		TArray<FBox3DFragmentData> Chunks;
		for (int32 Index = 0; Index < Count; ++Index)
		{
			Chunks.Add(MakeStressBoxChunk(FVector(0, 0, 25 + Index * 50)));
			if (Index > 0)
			{
				BondStressChunks(Chunks, Index - 1, Index);
			}
		}
		ABox3DFracturedActor* Actor = World->SpawnActor<ABox3DFracturedActor>();
		Actor->bStructural = true;
		Actor->FragmentDensity = 400.0f;
		Actor->InitializeFragments(MoveTemp(Chunks), nullptr);
		return Actor;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStressCantileverAnalyticTest,
	"Box3DUnreal.Stress.CantileverAnalytic", BOX3D_TEST_FLAGS)
bool FBox3DStressCantileverAnalyticTest::RunTest(const FString& Parameters)
{
	constexpr int32 Count = 5;
	constexpr double MassKg = 2.0;
	constexpr double Gravity = 980.0;
	FBox3DStructureGraph Graph = MakeCantileverGraph(Count);
	TArray<double> Masses;
	Masses.Init(MassKg, Count);

	FBox3DStressSolver Solver;
	TestTrue(TEXT("solver initializes"), Solver.Initialize(Graph, Masses));
	Solver.BeginSolve(FVector(0.0, 0.0, -Gravity));
	const FBox3DStressSolveStats Stats = Solver.Relax(128);

	const int32 RootBond = Graph.FindBond(0, 1);
	const FBox3DBondStress Root = Solver.GetBondStress(RootBond);
	const double RootX = Graph.GetBond(RootBond).Centroid.X;
	double ExpectedMoment = 0.0;
	for (int32 Index = 1; Index < Count; ++Index)
	{
		ExpectedMoment += MassKg * Gravity * (Graph.GetNodeCentroid(Index).X - RootX);
	}
	const double ExpectedShear = (Count - 1) * MassKg * Gravity;

	TestTrue(TEXT("solve converges within fixed iteration budget"),
		Stats.ResidualForce < ExpectedShear * 1.0e-8);
	TestTrue(TEXT("root bond carries analytic cantilever moment"),
		FMath::IsNearlyEqual(Root.BendingMoment, ExpectedMoment, ExpectedMoment * 1.0e-6));
	TestTrue(TEXT("root bond carries total cantilever shear"),
		FMath::IsNearlyEqual(Root.Shear, ExpectedShear, ExpectedShear * 1.0e-6));
	TestTrue(TEXT("horizontal beam has no axial gravity load"),
		Root.Compression < 1.0e-6 && Root.Tension < 1.0e-6);

	// The same two-node graph is compression when supported below and tension
	// when supported above; this locks the NodeA -> NodeB sign convention.
	{
		TArray<FBox3DFragmentData> Vertical = MakeBeam(2);
		Vertical[0].Centroid = FVector(0.0, 0.0, 0.0);
		Vertical[1].Centroid = FVector(0.0, 0.0, 100.0);
		FBox3DStructureGraph Tower;
		Tower.Build(Vertical);
		Tower.SetAnchor(0, true);
		FBox3DStressSolver TowerSolver;
		TowerSolver.Initialize(Tower, TArray<double>({ MassKg, MassKg }));
		TowerSolver.BeginSolve(FVector(0.0, 0.0, -Gravity));
		TowerSolver.Relax(2);
		const FBox3DBondStress Compression = TowerSolver.GetBondStress(0);
		TestTrue(TEXT("supported column resolves gravity as compression"),
			FMath::IsNearlyEqual(Compression.Compression, MassKg * Gravity, 1.0e-6));
		TestTrue(TEXT("supported column has no tension"), Compression.Tension < 1.0e-6);

		FBox3DStructureGraph Hanging;
		Hanging.Build(Vertical);
		Hanging.SetAnchor(1, true);
		FBox3DStressSolver HangingSolver;
		HangingSolver.Initialize(Hanging, TArray<double>({ MassKg, MassKg }));
		HangingSolver.BeginSolve(FVector(0.0, 0.0, -Gravity));
		HangingSolver.Relax(2);
		const FBox3DBondStress Tension = HangingSolver.GetBondStress(0);
		TestTrue(TEXT("hanging column resolves gravity as tension"),
			FMath::IsNearlyEqual(Tension.Tension, MassKg * Gravity, 1.0e-6));
		TestTrue(TEXT("hanging column has no compression"), Tension.Compression < 1.0e-6);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStressCoarseningTest,
	"Box3DUnreal.Stress.GraphCoarsening", BOX3D_TEST_FLAGS)
bool FBox3DStressCoarseningTest::RunTest(const FString& Parameters)
{
	constexpr int32 Count = 12;
	constexpr int32 Threshold = 4;
	FBox3DStructureGraph Graph = MakeCantileverGraph(Count);
	TArray<double> Masses;
	Masses.Init(1.0, Count);

	FBox3DStressSolver Solver;
	TestTrue(TEXT("coarsened solver initializes"), Solver.Initialize(Graph, Masses, Threshold));
	TestTrue(TEXT("coarsening activates above threshold"), Solver.IsCoarsened());
	TestTrue(TEXT("solve graph respects node threshold"), Solver.GetSolveNodeCount() <= Threshold);
	TestTrue(TEXT("anchor boundary remains a solve bond"), Solver.GetSolveBondCount() > 0);

	Solver.BeginSolve(FVector(0.0, 0.0, -980.0));
	const FBox3DStressSolveStats Stats = Solver.Relax(128);
	const FBox3DBondStress Root = Solver.GetBondStress(Graph.FindBond(0, 1));
	double ExpectedMoment = 0.0;
	const double RootX = Graph.GetBond(Graph.FindBond(0, 1)).Centroid.X;
	for (int32 Index = 1; Index < Count; ++Index)
	{
		ExpectedMoment += 980.0 * (Graph.GetNodeCentroid(Index).X - RootX);
	}
	TestTrue(FString::Printf(TEXT("coarsened cantilever converges (residual %.12g)"), Stats.ResidualForce),
		Stats.ResidualForce < 1.0e-6);
	TestTrue(FString::Printf(TEXT("coarsening preserves aggregate root moment (got %.12g, expected %.12g)"),
		Root.BendingMoment, ExpectedMoment),
		FMath::IsNearlyEqual(Root.BendingMoment, ExpectedMoment, ExpectedMoment * 1.0e-8));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStressOverloadBreakTest,
	"Box3DUnreal.Stress.OverloadBreaksCorrectBond", BOX3D_TEST_FLAGS)
bool FBox3DStressOverloadBreakTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FScopedPromotionBudget PromotionBudget(0);
	Box3DTest::FScopedStressSettings StressSettings(64, 0);
	Box3DTest::FTestWorld Test;
	MirrorGround(Test);
	ABox3DFracturedActor* Tower = SpawnStressTower(Test.World, 4);
	TestTrue(TEXT("structure active"), Tower->IsStructureActive());

	// A vertical column carries maximum compression at the root. Put capacity
	// between the analytic root and second-bond stresses so only bond 0 erodes.
	Tower->TensionStrengthPa = 0.0f;
	Tower->ShearStrengthPa = 0.0f;
	Tower->CompressionStrengthPa = 1000.0f;
	Tower->SustainedOverloadHealthPerSecond = 100.0f;
	UBox3DTestEventCounter* Counter = NewObject<UBox3DTestEventCounter>();
	Tower->OnStructureStressed.AddDynamic(Counter, &UBox3DTestEventCounter::HandleStructureStressed);
	Tower->OnWeldBroken.AddDynamic(Counter, &UBox3DTestEventCounter::HandleWeldBroke);

	for (int32 Pump = 0; Pump < 64 && !Tower->GetStructureGraph().GetBond(0).bBroken; ++Pump)
	{
		Tower->ProcessStructuralStress(1.0f / 60.0f);
	}
	TestTrue(TEXT("creak hook fires before failure"), Counter->StructureStressedCount > 0);
	TestEqual(TEXT("analytically most-loaded root bond breaks first"),
		Tower->GetStructureGraph().GetBond(0).bBroken, true);
	TestFalse(TEXT("next bond remains intact"), Tower->GetStructureGraph().GetBond(1).bBroken);
	TestEqual(TEXT("stress break destroys exactly one weld"), Tower->GetLiveWeldCount(), 2);
	TestEqual(TEXT("stress failure broadcasts weld event once"), Counter->WeldBrokeCount, 1);
	TestEqual(TEXT("unsupported tower queued/promoted through D4 path"), Tower->GetPendingPromotionCount(), 3);
	Tower->ProcessPromotions();
	TestEqual(TEXT("promotion FIFO drains"), Tower->GetPendingPromotionCount(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStressExplosionEventTest,
	"Box3DUnreal.Stress.ExplosionEventPath", BOX3D_TEST_FLAGS)
bool FBox3DStressExplosionEventTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FScopedStressSettings StressSettings(64, 0);
	Box3DTest::FTestWorld Test;
	MirrorGround(Test);
	ABox3DFracturedActor* Tower = SpawnStressTower(Test.World, 4);
	Tower->TensionStrengthPa = 1.0e9f;
	Tower->CompressionStrengthPa = 1.0e9f;
	Tower->ShearStrengthPa = 1000.0f;
	Tower->SustainedOverloadHealthPerSecond = 100.0f;
	UBox3DTestEventCounter* Counter = NewObject<UBox3DTestEventCounter>();
	Tower->OnStructureStressed.AddDynamic(Counter, &UBox3DTestEventCounter::HandleStructureStressed);

	UBox3DQueryLibrary::Box3DExplode(Test.World, FVector(-200, 0, 100), 1000.0f, 0.0f,
		5.0f, FBox3DQueryFilter());
	for (int32 Pump = 0; Pump < 64 && Counter->StructureStressedCount == 0; ++Pump)
	{
		Tower->ProcessStructuralStress(1.0f / 60.0f);
	}
	TestTrue(TEXT("Box3DExplode feeds structural stress without joint-force polling"),
		Counter->StructureStressedCount > 0);
	TestTrue(TEXT("explosion event erodes or breaks the loaded structure"),
		Tower->GetStructureGraph().GetBond(0).Health < 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStressStableGravityTest,
	"Box3DUnreal.Stress.StableGravityNoDamage600Steps", BOX3D_TEST_FLAGS)
bool FBox3DStressStableGravityTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FScopedStressSettings StressSettings(8, 0);
	Box3DTest::FTestWorld Test;
	MirrorGround(Test);
	ABox3DFracturedActor* Tower = SpawnStressTower(Test.World, 4);
	Tower->TensionStrengthPa = 1.0e9f;
	Tower->CompressionStrengthPa = 1.0e9f;
	Tower->ShearStrengthPa = 1.0e9f;
	for (int32 Step = 0; Step < 600; ++Step)
	{
		Tower->ProcessStructuralStress(1.0f / 60.0f);
	}
	bool bPristine = true;
	for (int32 BondIndex = 0; BondIndex < Tower->GetStructureGraph().GetBondCount(); ++BondIndex)
	{
		const FBox3DStructureBond& Bond = Tower->GetStructureGraph().GetBond(BondIndex);
		bPristine &= Bond.Health == 1.0f && !Bond.bBroken;
	}
	TestTrue(TEXT("stable structure has exactly zero damage after 600 steps"), bPristine);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStressDeterminismTest,
	"Box3DUnreal.Stress.BondHealthHashDeterminism", BOX3D_TEST_FLAGS)
bool FBox3DStressDeterminismTest::RunTest(const FString& Parameters)
{
	struct FProgression
	{
		TArray<uint32> Hashes;
		TArray<int32> LiveBonds;
	};
	const auto SolveProgression = [](const FVector& Impulse) -> FProgression
	{
		FBox3DStructureGraph Graph = MakeCantileverGraph(9);
		TArray<double> Masses;
		for (int32 Index = 0; Index < Graph.GetNodeCount(); ++Index)
		{
			Masses.Add(1.0 + Index * 0.125);
		}
		FBox3DStressSolver Solver;
		Solver.Initialize(Graph, Masses, 4);
		FBox3DStressThresholds Thresholds;
		Thresholds.TensionPa = 1.0e9;
		Thresholds.CompressionPa = 1.0e9;
		Thresholds.ShearPa = 250.0;
		FProgression Out;
		for (int32 Step = 0; Step < 8; ++Step)
		{
			FBox3DStressImpulse Event;
			Event.NodeIndex = 7;
			Event.Impulse = Impulse;
			Event.ApplicationPoint = Graph.GetNodeCentroid(7) + FVector(0.0, 25.0, 10.0);
			Solver.BeginSolve(FVector(0.0, 0.0, -980.0), MakeArrayView(&Event, 1), 1.0 / 60.0);
			Solver.Relax(64);
			for (int32 BondIndex = 0; BondIndex < Graph.GetBondCount(); ++BondIndex)
			{
				const FBox3DBondOverload Overload = Solver.GetBondOverload(BondIndex, Thresholds);
				if (Overload.Ratio > 1.0 && Graph.ApplyBondDamage(BondIndex,
					static_cast<float>((Overload.Ratio - 1.0) * 0.02)) <= 0.0f)
				{
					FBox3DStructureIslands Islands;
					Graph.NotifyBondBroken(BondIndex, Islands);
				}
			}
			Out.Hashes.Add(Solver.BondHealthHash());
			Out.LiveBonds.Add(Graph.GetLiveBondCount());
			Solver.Initialize(Graph, Masses, 4);
		}
		return Out;
	};

	const FProgression A = SolveProgression(FVector(125.0, -30.0, 50.0));
	const FProgression B = SolveProgression(FVector(125.0, -30.0, 50.0));
	const FProgression Different = SolveProgression(FVector(126.0, -30.0, 50.0));
	TestTrue(TEXT("same structure and impulses produce identical health erosion hashes"), A.Hashes == B.Hashes);
	TestTrue(TEXT("same structure and impulses break bonds in identical progression"), A.LiveBonds == B.LiveBonds);
	TestFalse(TEXT("hash progression covers impulse/load differences"), A.Hashes == Different.Hashes);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
