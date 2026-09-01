// Tests for the deterministic fracture core (Box3DFracture): layout hashing,
// seed sensitivity, volume conservation, hull validity, adjacency symmetry,
// impact-biased site density, MinFragmentVolume merging, and worker-thread
// independence. Pure geometry — no world or subsystem needed.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DConversion.h"
#include "Box3DFracture.h"
#include "Tests/Box3DTestHelpers.h"
#include "box3d/collision.h"

namespace
{
	using namespace Box3D::Fracture;

	/// The standard fixture: a 100 cm cube fractured into 12 cells.
	FFractureParams DefaultParams(int32 Seed = 42)
	{
		FFractureParams Params;
		Params.Seed = Seed;
		Params.CellCount = 12;
		return Params;
	}

	FFractureProxy DefaultProxy()
	{
		return MakeBoxProxy(FVector::ZeroVector, FVector(50.0));
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureDeterminismTest,
	"Box3DUnreal.Fracture.Determinism", BOX3D_TEST_FLAGS)
bool FBox3DFractureDeterminismTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();
	const FFractureParams Params = DefaultParams();

	TArray<FBox3DFragmentData> FirstRun;
	TArray<FBox3DFragmentData> SecondRun;
	TestTrue(TEXT("first fracture succeeds"), Fracture(Proxy, Params, FirstRun));
	TestTrue(TEXT("second fracture succeeds"), Fracture(Proxy, Params, SecondRun));

	TestEqual(TEXT("fragment counts identical"), SecondRun.Num(), FirstRun.Num());
	TestEqual(TEXT("same seed + params -> bit-identical layout hash"),
		FractureLayoutHash(SecondRun), FractureLayoutHash(FirstRun));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureSeedVariationTest,
	"Box3DUnreal.Fracture.SeedVariation", BOX3D_TEST_FLAGS)
bool FBox3DFractureSeedVariationTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();

	TArray<FBox3DFragmentData> SeedA;
	TArray<FBox3DFragmentData> SeedB;
	TestTrue(TEXT("seed 42 fracture succeeds"), Fracture(Proxy, DefaultParams(42), SeedA));
	TestTrue(TEXT("seed 43 fracture succeeds"), Fracture(Proxy, DefaultParams(43), SeedB));

	TestNotEqual(TEXT("different seeds -> different layout hashes"),
		FractureLayoutHash(SeedB), FractureLayoutHash(SeedA));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureVolumeConservationTest,
	"Box3DUnreal.Fracture.VolumeConservation", BOX3D_TEST_FLAGS)
bool FBox3DFractureVolumeConservationTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();
	TArray<FBox3DFragmentData> Fragments;
	TestTrue(TEXT("fracture succeeds"), Fracture(Proxy, DefaultParams(), Fragments));

	const double ProxyVolume = 100.0 * 100.0 * 100.0; // cm^3
	double TotalVolume = 0.0;
	for (const FBox3DFragmentData& Fragment : Fragments)
	{
		TestTrue(TEXT("fragment volume positive"), Fragment.Volume > 0.0);
		TestTrue(TEXT("centroid inside proxy bounds"),
			FBox(FVector(-50.0), FVector(50.0)).IsInsideOrOn(Fragment.Centroid));
		TotalVolume += Fragment.Volume;
	}

	const double Error = FMath::Abs(TotalVolume - ProxyVolume) / ProxyVolume;
	TestTrue(FString::Printf(TEXT("volume conserved within 0.5%% (error %.6f%%)"), Error * 100.0),
		Error < 0.005);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureValidHullsTest,
	"Box3DUnreal.Fracture.ValidHulls", BOX3D_TEST_FLAGS)
bool FBox3DFractureValidHullsTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();
	TArray<FBox3DFragmentData> Fragments;
	TestTrue(TEXT("fracture succeeds"), Fracture(Proxy, DefaultParams(), Fragments));

	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		const FBox3DFragmentData& Fragment = Fragments[Index];
		TestTrue(FString::Printf(TEXT("fragment %d has >= 4 vertices"), Index),
			Fragment.Vertices.Num() >= 4);

		// Same cm -> m seam the physics handoff will use (D2).
		TArray<b3Vec3> Points;
		Points.Reserve(Fragment.Vertices.Num());
		for (const FVector& Vertex : Fragment.Vertices)
		{
			Points.Add(Box3D::ToB3(Vertex));
		}
		b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), 64);
		if (TestNotNull(FString::Printf(TEXT("b3CreateHull accepts fragment %d"), Index), Hull))
		{
			b3DestroyHull(Hull);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureAdjacencyTest,
	"Box3DUnreal.Fracture.Adjacency", BOX3D_TEST_FLAGS)
bool FBox3DFractureAdjacencyTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();
	TArray<FBox3DFragmentData> Fragments;
	TestTrue(TEXT("fracture succeeds"), Fracture(Proxy, DefaultParams(), Fragments));

	int32 TotalLinks = 0;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		TestTrue(FString::Printf(TEXT("fragment %d has at least one neighbor"), Index),
			Fragments[Index].Neighbors.Num() > 0);
		for (const FBox3DFragmentNeighbor& Neighbor : Fragments[Index].Neighbors)
		{
			++TotalLinks;
			TestTrue(TEXT("neighbor index valid"),
				Fragments.IsValidIndex(Neighbor.FragmentIndex) && Neighbor.FragmentIndex != Index);
			TestTrue(TEXT("shared-face area positive"), Neighbor.SharedFaceArea > 0.0);

			// Symmetry: B must list A back with the identical canonical
			// shared-face area.
			const FBox3DFragmentNeighbor* Back = Fragments[Neighbor.FragmentIndex].Neighbors.FindByPredicate(
				[Index](const FBox3DFragmentNeighbor& Candidate)
				{
					return Candidate.FragmentIndex == Index;
				});
			if (TestNotNull(FString::Printf(TEXT("fragment %d listed back by neighbor %d"),
					Index, Neighbor.FragmentIndex), Back))
			{
				TestTrue(FString::Printf(TEXT("canonical shared-face areas identical (%f vs %f)"),
						Neighbor.SharedFaceArea, Back->SharedFaceArea),
					Neighbor.SharedFaceArea == Back->SharedFaceArea);
			}

			// Cross-check the raw geometry: the two cells' own face polygons on
			// the shared bisector plane may differ by quantization slivers
			// (quantized plane offsets shift third-party cut lines), but only
			// within a sub-percent tolerance.
			const FBox3DFragmentFace* FaceAB = Fragments[Index].Faces.FindByPredicate(
				[&Neighbor](const FBox3DFragmentFace& Face)
				{
					return Face.NeighborIndex == Neighbor.FragmentIndex;
				});
			const FBox3DFragmentFace* FaceBA = Fragments[Neighbor.FragmentIndex].Faces.FindByPredicate(
				[Index](const FBox3DFragmentFace& Face)
				{
					return Face.NeighborIndex == Index;
				});
			if (TestNotNull(TEXT("forward face exists"), FaceAB) &&
				TestNotNull(TEXT("backward face exists"), FaceBA))
			{
				const double Tolerance = FMath::Max(0.5, 0.02 * FMath::Max(FaceAB->Area, FaceBA->Area));
				TestTrue(FString::Printf(TEXT("raw face areas agree within slivers (%f vs %f)"),
						FaceAB->Area, FaceBA->Area),
					FMath::Abs(FaceAB->Area - FaceBA->Area) <= Tolerance);
			}
		}
	}
	TestTrue(TEXT("adjacency links exist"), TotalLinks > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureCellCountTest,
	"Box3DUnreal.Fracture.CellCount", BOX3D_TEST_FLAGS)
bool FBox3DFractureCellCountTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();

	for (const int32 CellCount : { 1, 2, 8, 20 })
	{
		FFractureParams Params = DefaultParams();
		Params.CellCount = CellCount;
		TArray<FBox3DFragmentData> Fragments;
		TestTrue(FString::Printf(TEXT("fracture with %d cells succeeds"), CellCount),
			Fracture(Proxy, Params, Fragments));
		// Sites are well-separated at this scale, so no cells collapse and the
		// fragment count matches the request exactly.
		TestEqual(FString::Printf(TEXT("fragment count for CellCount %d"), CellCount),
			Fragments.Num(), CellCount);
	}

	// Degenerate inputs are rejected, not asserted on.
	TArray<FBox3DFragmentData> Fragments;
	FFractureParams Bad = DefaultParams();
	Bad.CellCount = 0;
	TestFalse(TEXT("CellCount 0 rejected"), Fracture(Proxy, Bad, Fragments));
	TestFalse(TEXT("empty proxy rejected"), Fracture(FFractureProxy(), DefaultParams(), Fragments));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureImpactBiasTest,
	"Box3DUnreal.Fracture.ImpactBias", BOX3D_TEST_FLAGS)
bool FBox3DFractureImpactBiasTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();

	FFractureParams Uniform = DefaultParams();
	Uniform.CellCount = 16;
	Uniform.ImpactPoint = FVector::ZeroVector;
	Uniform.ImpactRadius = 20.0;
	Uniform.RadialBias = 0.0;
	FFractureParams Biased = Uniform;
	Biased.RadialBias = 1.0;

	TArray<FBox3DFragmentData> UniformFragments;
	TArray<FBox3DFragmentData> BiasedFragments;
	TestTrue(TEXT("uniform fracture succeeds"), Fracture(Proxy, Uniform, UniformFragments));
	TestTrue(TEXT("biased fracture succeeds"), Fracture(Proxy, Biased, BiasedFragments));
	TestEqual(TEXT("biased fracture yields the requested cell count"),
		BiasedFragments.Num(), Biased.CellCount);

	// All biased sites sit within ImpactRadius of the impact, so cells near
	// the impact are much smaller (denser sites -> tinier cells) and fragment
	// centroids sit closer to the impact on average than the uniform layout.
	auto MinVolume = [](const TArray<FBox3DFragmentData>& Fragments)
	{
		double Min = TNumericLimits<double>::Max();
		for (const FBox3DFragmentData& Fragment : Fragments)
		{
			Min = FMath::Min(Min, Fragment.Volume);
		}
		return Min;
	};
	auto MeanDistance = [](const TArray<FBox3DFragmentData>& Fragments, const FVector& Point)
	{
		double Sum = 0.0;
		for (const FBox3DFragmentData& Fragment : Fragments)
		{
			Sum += FVector::Dist(Fragment.Centroid, Point);
		}
		return Sum / Fragments.Num();
	};
	TestTrue(FString::Printf(TEXT("smallest fragment shrinks when biased (%.0f vs %.0f cm3)"),
			MinVolume(BiasedFragments), MinVolume(UniformFragments)),
		MinVolume(BiasedFragments) < 0.5 * MinVolume(UniformFragments));
	TestTrue(FString::Printf(TEXT("mean centroid distance shrinks when biased (%.1f vs %.1f)"),
			MeanDistance(BiasedFragments, Uniform.ImpactPoint),
			MeanDistance(UniformFragments, Uniform.ImpactPoint)),
		MeanDistance(BiasedFragments, Uniform.ImpactPoint) <
			MeanDistance(UniformFragments, Uniform.ImpactPoint));

	// Bias changes the layout, and the biased path is itself deterministic.
	TestNotEqual(TEXT("bias changes the layout hash"),
		FractureLayoutHash(BiasedFragments), FractureLayoutHash(UniformFragments));
	TArray<FBox3DFragmentData> BiasedRepeat;
	TestTrue(TEXT("biased repeat succeeds"), Fracture(Proxy, Biased, BiasedRepeat));
	TestEqual(TEXT("biased fracture deterministic"),
		FractureLayoutHash(BiasedRepeat), FractureLayoutHash(BiasedFragments));

	// Biased volume is still conserved: the cluster's hull cells stretch to
	// the proxy boundary, they don't leave gaps.
	double TotalVolume = 0.0;
	for (const FBox3DFragmentData& Fragment : BiasedFragments)
	{
		TotalVolume += Fragment.Volume;
	}
	TestTrue(TEXT("biased volume conserved within 0.5%"),
		FMath::Abs(TotalVolume - 1000000.0) / 1000000.0 < 0.005);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureMinVolumeMergeTest,
	"Box3DUnreal.Fracture.MinVolumeMerge", BOX3D_TEST_FLAGS)
bool FBox3DFractureMinVolumeMergeTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = DefaultProxy();

	// Threshold just above the average cell volume (1e6 / 12), so at least one
	// fragment sits below it and merging must fire.
	FFractureParams Params = DefaultParams();
	Params.MinFragmentVolume = 1000000.0 / 12.0 + 1.0;

	TArray<FBox3DFragmentData> Fragments;
	TestTrue(TEXT("fracture with merging succeeds"), Fracture(Proxy, Params, Fragments));
	TestTrue(FString::Printf(TEXT("merging reduced fragment count (%d < 12)"), Fragments.Num()),
		Fragments.Num() < 12);
	TestTrue(TEXT("at least one fragment survives"), Fragments.Num() >= 1);

	double TotalVolume = 0.0;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		const FBox3DFragmentData& Fragment = Fragments[Index];
		TotalVolume += Fragment.Volume;
		TestTrue(FString::Printf(TEXT("fragment %d meets threshold or is isolated (%.0f cm3)"),
				Index, Fragment.Volume),
			Fragment.Volume >= Params.MinFragmentVolume || Fragment.Neighbors.IsEmpty());
		TestTrue(FString::Printf(TEXT("fragment %d has >= 4 vertices"), Index),
			Fragment.Vertices.Num() >= 4);

		// Every vertex is face-referenced (merging compacts orphans) and every
		// face index is in range.
		TArray<bool> Referenced;
		Referenced.Init(false, Fragment.Vertices.Num());
		for (const FBox3DFragmentFace& Face : Fragment.Faces)
		{
			for (const int32 VertexIndex : Face.VertexIndices)
			{
				if (TestTrue(TEXT("face vertex index in range"),
						Fragment.Vertices.IsValidIndex(VertexIndex)))
				{
					Referenced[VertexIndex] = true;
				}
			}
			TestTrue(TEXT("face neighbor index valid or exterior"),
				Face.NeighborIndex == INDEX_NONE ||
					(Fragments.IsValidIndex(Face.NeighborIndex) && Face.NeighborIndex != Index));
		}
		TestFalse(FString::Printf(TEXT("fragment %d has no orphaned vertices"), Index),
			Referenced.Contains(false));

		// Adjacency stays symmetric with identical canonical areas after merging.
		for (const FBox3DFragmentNeighbor& Neighbor : Fragment.Neighbors)
		{
			TestTrue(TEXT("neighbor index valid"),
				Fragments.IsValidIndex(Neighbor.FragmentIndex) && Neighbor.FragmentIndex != Index);
			const FBox3DFragmentNeighbor* Back = Fragments[Neighbor.FragmentIndex].Neighbors.FindByPredicate(
				[Index](const FBox3DFragmentNeighbor& Candidate)
				{
					return Candidate.FragmentIndex == Index;
				});
			if (TestNotNull(TEXT("neighbor lists back after merging"), Back))
			{
				TestTrue(TEXT("merged shared-face areas identical both ways"),
					Neighbor.SharedFaceArea == Back->SharedFaceArea);
			}
		}
	}

	// Merging adds volumes exactly — conservation is unaffected.
	TestTrue(TEXT("merged volume conserved within 0.5%"),
		FMath::Abs(TotalVolume - 1000000.0) / 1000000.0 < 0.005);

	TArray<FBox3DFragmentData> Repeat;
	TestTrue(TEXT("merge repeat succeeds"), Fracture(Proxy, Params, Repeat));
	TestEqual(TEXT("merging deterministic"),
		FractureLayoutHash(Repeat), FractureLayoutHash(Fragments));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureThreadCountDeterminismTest,
	"Box3DUnreal.Fracture.ThreadCountDeterminism", BOX3D_TEST_FLAGS)
bool FBox3DFractureThreadCountDeterminismTest::RunTest(const FString& Parameters)
{
	// Fracture is single-threaded pure geometry by design — it never touches
	// the scheduler — but the plan pins hash stability across worker-thread
	// counts, so prove independence by running under different scheduler
	// settings (the same override the threading tests use), with every fracture
	// feature engaged.
	const FFractureProxy Proxy = DefaultProxy();
	FFractureParams Params = DefaultParams();
	Params.ImpactPoint = FVector(25.0, 0.0, -10.0);
	Params.ImpactRadius = 30.0;
	Params.RadialBias = 0.75;
	Params.MinFragmentVolume = 20000.0;

	TOptional<uint32> ReferenceHash;
	for (const int32 Workers : { 1, 2, 8 })
	{
		Box3DTest::FScopedTaskSettings Settings(Workers, EBox3DTaskSystem::UnrealTasks);
		TArray<FBox3DFragmentData> Fragments;
		TestTrue(FString::Printf(TEXT("fracture succeeds with %d workers"), Workers),
			Fracture(Proxy, Params, Fragments));
		const uint32 Hash = FractureLayoutHash(Fragments);
		if (!ReferenceHash.IsSet())
		{
			ReferenceHash = Hash;
		}
		else
		{
			TestEqual(FString::Printf(TEXT("layout hash identical with %d workers"), Workers),
				Hash, ReferenceHash.GetValue());
		}
	}
	return true;
}

/// Explicit sites replace seeded generation: outside sites are dropped, grid
/// duplicates collapse, FlattenAxis still applies, and the layout is a pure
/// function of the sites.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFractureExplicitSitesTest,
	"Box3DUnreal.Fracture.ExplicitSites", BOX3D_TEST_FLAGS)
bool FBox3DFractureExplicitSitesTest::RunTest(const FString& Parameters)
{
	const FFractureProxy Proxy = MakeBoxProxy(FVector::ZeroVector, FVector(50, 50, 50));
	FFractureParams Params;
	Params.Seed = 99;
	Params.CellCount = 40; // ignored
	Params.FlattenAxis = 2;
	Params.Sites = { FVector(-25, -25, 40), FVector(25, -25, -40), FVector(-25, 25, 0), FVector(25, 25, 10),
		FVector(200, 0, 0), FVector(25.001, 25.001, 0) };

	TArray<FBox3DFragmentData> Fragments;
	TestTrue(TEXT("fracture succeeds"), Fracture(Proxy, Params, Fragments));
	TestEqual(TEXT("four usable sites -> four cells"), Fragments.Num(), 4);
	double Volume = 0.0;
	for (const FBox3DFragmentData& Fragment : Fragments)
	{
		Volume += Fragment.Volume;
		FBox Box(ForceInit);
		for (const FVector& Vertex : Fragment.Vertices)
		{
			Box += Vertex;
		}
		TestTrue(TEXT("flattened cell spans the full height"),
			FMath::IsNearlyEqual(Box.Min.Z, -50.0, 0.05) && FMath::IsNearlyEqual(Box.Max.Z, 50.0, 0.05));
		TestTrue(TEXT("quadrant cell is a 50 x 50 column"),
			FMath::IsNearlyEqual(Box.GetSize().X, 50.0, 0.05) && FMath::IsNearlyEqual(Box.GetSize().Y, 50.0, 0.05));
	}
	TestTrue(TEXT("volume conserved"), FMath::IsNearlyEqual(Volume, 1.0e6, 1.0));

	TArray<FBox3DFragmentData> Again;
	Params.Seed = 1;
	Fracture(Proxy, Params, Again);
	TestEqual(TEXT("seed is irrelevant with explicit sites"), FractureLayoutHash(Again), FractureLayoutHash(Fragments));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
