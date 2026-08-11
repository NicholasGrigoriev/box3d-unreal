// Tests for the deterministic fracture core (Box3DFracture): layout hashing,
// seed sensitivity, volume conservation, hull validity, and adjacency symmetry.
// Pure geometry — no world or subsystem needed.

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

#endif // WITH_DEV_AUTOMATION_TESTS
