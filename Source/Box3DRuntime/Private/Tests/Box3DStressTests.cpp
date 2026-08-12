// D5 stress-core tests: analytic cantilever equilibrium, axial/shear load
// decomposition, deterministic coarsening, and quantized bond-state hashing.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DFracture.h"
#include "Box3DStress.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DStressDeterminismTest,
	"Box3DUnreal.Stress.BondHealthHashDeterminism", BOX3D_TEST_FLAGS)
bool FBox3DStressDeterminismTest::RunTest(const FString& Parameters)
{
	const auto SolveHash = [](const FVector& Impulse) -> uint32
	{
		FBox3DStructureGraph Graph = MakeCantileverGraph(9);
		TArray<double> Masses;
		for (int32 Index = 0; Index < Graph.GetNodeCount(); ++Index)
		{
			Masses.Add(1.0 + Index * 0.125);
		}
		FBox3DStressSolver Solver;
		Solver.Initialize(Graph, Masses, 4);
		FBox3DStressImpulse Event;
		Event.NodeIndex = 7;
		Event.Impulse = Impulse;
		Event.ApplicationPoint = Graph.GetNodeCentroid(7) + FVector(0.0, 25.0, 10.0);
		Solver.BeginSolve(FVector(0.0, 0.0, -980.0), MakeArrayView(&Event, 1), 1.0 / 60.0);
		Solver.Relax(64);
		return Solver.BondHealthHash();
	};

	const uint32 HashA = SolveHash(FVector(125.0, -30.0, 50.0));
	const uint32 HashB = SolveHash(FVector(125.0, -30.0, 50.0));
	const uint32 DifferentImpulseHash = SolveHash(FVector(126.0, -30.0, 50.0));
	TestEqual(TEXT("same structure and impulses produce identical bond-health hash"), HashA, HashB);
	TestNotEqual(TEXT("hash covers resolved impulse load, not health alone"), HashA, DifferentImpulseHash);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
