// Tests for the D2 fractured actor: proxy resolution fallback order
// (authored convex -> simple collision -> render vertices), fragment layout
// parity between Box3D::FractureMesh and the pure fracture core, per-fragment
// section materials, the source-component swap-out seam (visibility, Chaos
// collision, static mirror body), and the physics half — hull bodies per
// fragment, cell-adjacency welds with area-scaled break forces, rest-state
// sleep, and overload-driven weld snapping with island separation.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DConversion.h"
#include "Box3DFracturedActor.h"
#include "Box3DStaticSceneMirror.h"
#include "Materials/MaterialInterface.h"
#include "PhysicsEngine/BodySetup.h"
#include "ProceduralMeshComponent.h"
#include "Tests/Box3DTestEventCounter.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	using namespace Box3D::Fracture;

	/// A proxy's volume, measured through the core itself: a single-cell
	/// fracture returns the whole proxy as one fragment.
	double MeasureProxyVolume(const FFractureProxy& Proxy)
	{
		FFractureParams Params;
		Params.Seed = 1;
		Params.CellCount = 1;
		TArray<FBox3DFragmentData> Fragments;
		if (!Fracture(Proxy, Params, Fragments) || Fragments.Num() != 1)
		{
			return 0.0;
		}
		return Fragments[0].Volume;
	}

	/// Connected components over fragments, edges = surviving welds.
	int32 CountWeldIslands(const ABox3DFracturedActor& Actor)
	{
		TArray<int32> Parent;
		Parent.SetNum(Actor.GetFragmentCount());
		for (int32 Index = 0; Index < Parent.Num(); ++Index)
		{
			Parent[Index] = Index;
		}
		const auto Find = [&Parent](int32 Node)
		{
			while (Parent[Node] != Node)
			{
				Node = Parent[Node] = Parent[Parent[Node]];
			}
			return Node;
		};
		TArray<FIntPoint> Pairs;
		Actor.GetLiveWeldPairs(Pairs);
		for (const FIntPoint& Pair : Pairs)
		{
			Parent[Find(Pair.X)] = Find(Pair.Y);
		}
		int32 Islands = 0;
		for (int32 Index = 0; Index < Parent.Num(); ++Index)
		{
			Islands += Find(Index) == Index ? 1 : 0;
		}
		return Islands;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorProxyResolutionTest,
	"Box3DUnreal.FracturedActor.ProxyResolution", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorProxyResolutionTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UStaticMesh* Cone = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cone.Cone"));
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();
	if (!TestNotNull(TEXT("engine cone mesh"), Cone) || !TestNotNull(TEXT("engine cube mesh"), Cube))
	{
		return false;
	}

	// Authored convex path: the engine cone ships convex collision.
	if (TestTrue(TEXT("cone has authored convex elems"),
			Cone->GetBodySetup() != nullptr && Cone->GetBodySetup()->AggGeom.ConvexElems.Num() > 0))
	{
		UStaticMeshComponent* ConeComponent = Box3DTest::SpawnSceneMesh(Test.World, Cone, FTransform::Identity,
			EComponentMobility::Movable, ECollisionEnabled::NoCollision);
		FFractureProxy Proxy;
		TestTrue(TEXT("cone resolves from authored convex"),
			Box3D::ResolveFractureProxy(*ConeComponent, Proxy) == EBox3DFractureProxySource::AuthoredConvex);
		TestTrue(TEXT("cone proxy is a closed polytope"), Proxy.Vertices.Num() >= 4 && Proxy.Faces.Num() >= 4);
	}

	// Simple-collision path: the engine cube ships a box elem, no convex elems.
	UBodySetup* CubeSetup = Cube->GetBodySetup();
	if (!TestNotNull(TEXT("cube body setup"), CubeSetup))
	{
		return false;
	}
	TestTrue(TEXT("cube ships box elem collision without convex elems"),
		CubeSetup->AggGeom.BoxElems.Num() > 0 && CubeSetup->AggGeom.ConvexElems.IsEmpty());

	UStaticMeshComponent* CubeComponent = Box3DTest::SpawnSceneMesh(Test.World, Cube, FTransform::Identity,
		EComponentMobility::Movable, ECollisionEnabled::NoCollision);
	FFractureProxy CubeProxy;
	TestTrue(TEXT("cube resolves from simple collision"),
		Box3D::ResolveFractureProxy(*CubeComponent, CubeProxy) == EBox3DFractureProxySource::SimpleCollision);

	const double CubeVolume = MeasureProxyVolume(CubeProxy);
	TestTrue(FString::Printf(TEXT("cube proxy volume ~ 100^3 cm^3 (got %f)"), CubeVolume),
		FMath::IsNearlyEqual(CubeVolume, 1.0e6, 5.0e4));

	// Component scale bakes into the proxy.
	UStaticMeshComponent* ScaledComponent = Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector::ZeroVector, FVector(2.0, 1.0, 1.0)),
		EComponentMobility::Movable, ECollisionEnabled::NoCollision);
	FFractureProxy ScaledProxy;
	TestTrue(TEXT("scaled cube resolves from simple collision"),
		Box3D::ResolveFractureProxy(*ScaledComponent, ScaledProxy) == EBox3DFractureProxySource::SimpleCollision);
	const double ScaledVolume = MeasureProxyVolume(ScaledProxy);
	TestTrue(FString::Printf(TEXT("scale 2x1x1 doubles the proxy volume (got %f)"), ScaledVolume),
		FMath::IsNearlyEqual(ScaledVolume, 2.0 * CubeVolume, 1.0e5));

	// Render-vertex fallback: strip the cube's collision for the duration.
	{
		const FKAggregateGeom SavedGeom = CubeSetup->AggGeom;
		CubeSetup->AggGeom = FKAggregateGeom();

		AddExpectedMessage(TEXT("using render vertices"), ELogVerbosity::Warning,
			EAutomationExpectedMessageFlags::Contains);
		FFractureProxy RenderProxy;
		TestTrue(TEXT("collisionless cube falls back to render vertices"),
			Box3D::ResolveFractureProxy(*CubeComponent, RenderProxy) == EBox3DFractureProxySource::RenderVertices);
		const double RenderVolume = MeasureProxyVolume(RenderProxy);
		TestTrue(FString::Printf(TEXT("render-vert proxy volume ~ 100^3 cm^3 (got %f)"), RenderVolume),
			FMath::IsNearlyEqual(RenderVolume, 1.0e6, 5.0e4));

		CubeSetup->AggGeom = SavedGeom;
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorFragmentLayoutTest,
	"Box3DUnreal.FracturedActor.FragmentLayoutAndMaterials", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorFragmentLayoutTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();
	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Cube, FTransform::Identity,
		EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
	UMaterialInterface* GridMaterial = LoadObject<UMaterialInterface>(nullptr,
		TEXT("/Engine/EngineMaterials/WorldGridMaterial.WorldGridMaterial"));
	if (!TestNotNull(TEXT("core material loads"), GridMaterial))
	{
		return false;
	}

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 42;
	Params.Fracture.CellCount = 12;
	Params.CoreMaterial = GridMaterial;

	// Reference layout straight from the D1 core on the same proxy: the
	// component sits at the origin unrotated, so spaces coincide.
	FFractureProxy Proxy;
	TestTrue(TEXT("proxy resolves"),
		Box3D::ResolveFractureProxy(*Component, Proxy) != EBox3DFractureProxySource::None);
	TArray<FBox3DFragmentData> Expected;
	TestTrue(TEXT("core fracture succeeds"), Fracture(Proxy, Params.Fracture, Expected));

	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor))
	{
		return false;
	}

	TestEqual(TEXT("fragment count matches the D1 layout"), Actor->GetFragmentCount(), Expected.Num());
	TestTrue(TEXT("multiple fragments"), Actor->GetFragmentCount() > 1);
	TestTrue(TEXT("actor spawned at the source transform"),
		Actor->GetActorLocation().Equals(Component->GetComponentLocation(), 0.1));

	UProceduralMeshComponent* Mesh = Actor->GetMesh();
	UMaterialInterface* SourceMaterial = Component->GetMaterial(0);
	TestNotNull(TEXT("source material exists"), SourceMaterial);

	int32 ExpectedSectionCount = 0;
	int32 ExteriorSectionCount = 0;
	for (int32 Index = 0; Index < Actor->GetFragmentCount(); ++Index)
	{
		const FIntPoint Sections = Actor->GetFragmentSections(Index);

		// Every fragment of a 12-cell layout touches at least one neighbor.
		TestTrue(FString::Printf(TEXT("fragment %d has an interior section"), Index), Sections.Y != INDEX_NONE);
		if (Sections.Y != INDEX_NONE)
		{
			++ExpectedSectionCount;
			TestTrue(FString::Printf(TEXT("fragment %d interior faces carry the core material"), Index),
				Mesh->GetMaterial(Sections.Y) == GridMaterial);
		}
		if (Sections.X != INDEX_NONE)
		{
			++ExpectedSectionCount;
			++ExteriorSectionCount;
			TestTrue(FString::Printf(TEXT("fragment %d exterior faces carry the source material"), Index),
				Mesh->GetMaterial(Sections.X) == SourceMaterial);
		}
	}
	TestTrue(TEXT("some fragments expose exterior faces"), ExteriorSectionCount > 0);
	TestEqual(TEXT("one PMC section per non-empty fragment side"), Mesh->GetNumSections(), ExpectedSectionCount);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorSwapOutTest,
	"Box3DUnreal.FracturedActor.SwapOutSeam", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorSwapOutTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;

	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FVector(0, 0, 50)));

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	if (!TestNotNull(TEXT("mirror enabled via settings"), Mirror))
	{
		return false;
	}
	Mirror->MirrorLevel(Test.World->PersistentLevel);
	TestEqual(TEXT("source cube mirrored"), Mirror->GetBodyCount(), 1);

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 7;
	Params.Fracture.CellCount = 8;
	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor))
	{
		return false;
	}

	TestFalse(TEXT("source component hidden"), Component->IsVisible());
	TestTrue(TEXT("source Chaos collision disabled"),
		Component->GetCollisionEnabled() == ECollisionEnabled::NoCollision);
	TestEqual(TEXT("mirror body removed"), Mirror->GetBodyCount(), 0);

	// Null CoreMaterial falls back to the source material on interior faces.
	const FIntPoint Sections = Actor->GetFragmentSections(0);
	if (Sections.Y != INDEX_NONE)
	{
		TestTrue(TEXT("interior falls back to the source material"),
			Actor->GetMesh()->GetMaterial(Sections.Y) == Component->GetMaterial(0));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorWeldAdjacencyTest,
	"Box3DUnreal.FracturedActor.WeldsMatchAdjacency", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorWeldAdjacencyTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform::Identity, EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 42;
	Params.Fracture.CellCount = 12;
	Params.bStartAsleep = true;
	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor))
	{
		return false;
	}

	const TArray<FBox3DFragmentData>& Fragments = Actor->GetFragments();
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		TestTrue(FString::Printf(TEXT("fragment %d has a hull body"), Index),
			b3Body_IsValid(Actor->GetFragmentBody(Index)));
	}

	// Welds are exactly the D1 adjacency pairs, one per pair.
	TSet<FIntPoint> ExpectedPairs;
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		for (const FBox3DFragmentNeighbor& Neighbor : Fragments[Index].Neighbors)
		{
			if (Neighbor.FragmentIndex > Index)
			{
				ExpectedPairs.Add(FIntPoint(Index, Neighbor.FragmentIndex));
			}
		}
	}
	TestTrue(TEXT("layout has adjacency"), ExpectedPairs.Num() > 0);
	TestEqual(TEXT("one weld per adjacent pair"), Actor->GetLiveWeldCount(), ExpectedPairs.Num());

	TArray<FIntPoint> LivePairs;
	Actor->GetLiveWeldPairs(LivePairs);
	for (const FIntPoint& Pair : LivePairs)
	{
		TestTrue(FString::Printf(TEXT("weld (%d, %d) matches an adjacency pair"), Pair.X, Pair.Y),
			ExpectedPairs.Contains(Pair));
	}

	TestEqual(TEXT("welded assembly is a single island"), CountWeldIslands(*Actor), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorRestingAssemblyTest,
	"Box3DUnreal.FracturedActor.WeldedAssemblyAtRest", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorRestingAssemblyTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World, 0.0f);

	// Cube resting exactly on the ground (spans Z 0..100).
	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FVector(0, 0, 50)), EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 42;
	Params.Fracture.CellCount = 8;
	Params.bStartAsleep = true;
	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor))
	{
		return false;
	}
	const int32 InitialWelds = Actor->GetLiveWeldCount();
	TestTrue(TEXT("assembly is welded"), InitialWelds > 0);

	TArray<FVector> InitialPositions;
	for (int32 Index = 0; Index < Actor->GetFragmentCount(); ++Index)
	{
		InitialPositions.Add(Box3D::ToUEPos(b3Body_GetPosition(Actor->GetFragmentBody(Index))));
	}

	Test.Step(60);
	Actor->CheckWelds();

	for (int32 Index = 0; Index < Actor->GetFragmentCount(); ++Index)
	{
		const b3BodyId Body = Actor->GetFragmentBody(Index);
		TestFalse(FString::Printf(TEXT("fragment %d asleep after 60 steps"), Index), b3Body_IsAwake(Body));
		const FVector Position = Box3D::ToUEPos(b3Body_GetPosition(Body));
		TestTrue(FString::Printf(TEXT("fragment %d has zero drift (moved %f cm)"), Index,
					 FVector::Dist(Position, InitialPositions[Index])),
			Position.Equals(InitialPositions[Index], 0.001));
	}
	TestEqual(TEXT("all welds survive rest"), Actor->GetLiveWeldCount(), InitialWelds);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorOverloadBreakTest,
	"Box3DUnreal.FracturedActor.OverloadSnapsWelds", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorOverloadBreakTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World, 0.0f);

	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FVector(0, 0, 50)), EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);

	// Two fragments, one weld: "the loaded welds" is exactly the weld of the
	// impacted fragment, and the break outcome is unambiguous.
	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 42;
	Params.Fracture.CellCount = 2;
	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor) || !TestEqual(TEXT("two fragments"),
			Actor->GetFragmentCount(), 2) || !TestEqual(TEXT("one weld"), Actor->GetLiveWeldCount(), 1))
	{
		return false;
	}

	UBox3DTestEventCounter* Counter = NewObject<UBox3DTestEventCounter>();
	Actor->OnWeldBroken.AddDynamic(Counter, &UBox3DTestEventCounter::HandleWeldBroke);

	// The break threshold in newtons, from the same shared-face area the weld
	// was built from. Loads are applied as sustained forces: the weld's reported
	// constraint force is inv_h x the warm-start impulse accumulator, which
	// decays within a step's substeps for an instantaneous velocity impulse —
	// only loads sustained across substeps (contacts, gravity, applied forces)
	// register, exactly the loads gameplay impacts produce.
	const double SharedArea = Actor->GetFragments()[0].Neighbors[0].SharedFaceArea;
	const float BreakForce = float(SharedArea) * Actor->MaterialToughness;
	TestTrue(TEXT("break force is positive"), BreakForce > 0.0f);

	const b3BodyId Body0 = Actor->GetFragmentBody(0);
	const b3BodyId Body1 = Actor->GetFragmentBody(1);

	Test.Step(30); // settle on the ground
	Actor->CheckWelds();
	TestEqual(TEXT("weld survives settling"), Actor->GetLiveWeldCount(), 1);

	// Below-threshold load: the weld holds and no event fires. The joint sees
	// at most the mass-split share of the applied force plus the gravity load.
	for (int32 Step = 0; Step < 3; ++Step)
	{
		b3Body_ApplyForceToCenter(Body0, b3Vec3{ BreakForce * 0.1f, 0.0f, 0.0f }, /*wake*/ true);
		Test.Step(1);
		Actor->CheckWelds();
	}
	TestEqual(TEXT("small load leaves the weld intact"), Actor->GetLiveWeldCount(), 1);
	TestEqual(TEXT("no break event for the small load"), Counter->WeldBrokeCount, 0);

	// Overload: 40x the threshold dwarfs any mass split, so the loaded weld
	// snaps within a few steps and the event fires once.
	for (int32 Step = 0; Step < 5 && Actor->GetLiveWeldCount() > 0; ++Step)
	{
		b3Body_ApplyForceToCenter(Body0, b3Vec3{ 0.0f, 0.0f, BreakForce * 40.0f }, /*wake*/ true);
		Test.Step(1);
		Actor->CheckWelds();
	}
	TestEqual(TEXT("overload snaps the weld"), Actor->GetLiveWeldCount(), 0);
	TestEqual(TEXT("OnWeldBroken fired once"), Counter->WeldBrokeCount, 1);
	TestEqual(TEXT("assembly separates into two islands"), CountWeldIslands(*Actor), 2);

	// Freed fragment moves away independently.
	const double InitialDistance = FVector::Dist(
		Box3D::ToUEPos(b3Body_GetPosition(Body0)), Box3D::ToUEPos(b3Body_GetPosition(Body1)));
	b3Body_ApplyLinearImpulseToCenter(Body0, b3Vec3{ 500.0f, 0.0f, 0.0f }, /*wake*/ true);
	Test.Step(30);
	Actor->SyncFragments();
	const double FinalDistance = FVector::Dist(
		Box3D::ToUEPos(b3Body_GetPosition(Body0)), Box3D::ToUEPos(b3Body_GetPosition(Body1)));
	TestTrue(FString::Printf(TEXT("freed fragment separates (%f -> %f cm)"), InitialDistance, FinalDistance),
		FinalDistance > InitialDistance + 50.0);

	// The PMC sections follow the bodies: fragment 0 flew away from its spawn
	// pose, so its synced section vertices sit far from the original geometry.
	const FIntPoint Sections = Actor->GetFragmentSections(0);
	const int32 SectionIndex = Sections.Y != INDEX_NONE ? Sections.Y : Sections.X;
	const FProcMeshSection* Section = Actor->GetMesh()->GetProcMeshSection(SectionIndex);
	if (TestNotNull(TEXT("fragment 0 has a PMC section"), Section) && Section->ProcVertexBuffer.Num() > 0)
	{
		const FVector SectionVertex(Section->ProcVertexBuffer[0].Position);
		const FVector BodyPosition = Actor->GetActorTransform().InverseTransformPosition(
			Box3D::ToUEPos(b3Body_GetPosition(Body0)));
		TestTrue(TEXT("synced section rides the fragment body"),
			FVector::Dist(SectionVertex, BodyPosition) < 200.0);
	}
	return true;
}

/// Anchor-all cladding: no ground anywhere, yet the structural assembly is live
/// and holds still; a detached fragment becomes a free body that flies while
/// its neighbours stay put, point lookups find the right fragment, and a
/// detached chip can still be destroyed (it is already "gone" in the graph).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorAnchorAllDetachTest,
	"Box3DUnreal.FracturedActor.AnchorAllAndDetach", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorAnchorAllDetachTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// A 100 x 100 x 10 slab floating in mid-air: nothing to anchor to.
	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FQuat::Identity, FVector(0, 0, 500), FVector(1.0, 1.0, 0.1)),
		EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 7;
	Params.Fracture.CellCount = 8;
	Params.bStartAsleep = true;
	Params.bStructural = true;
	Params.bAnchorAllFragments = true;
	Params.MaterialToughness = 0.0f;
	Params.Fracture.FlattenAxis = 2;
	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor))
	{
		return false;
	}
	TestTrue(TEXT("structure live without any static neighbour"), Actor->IsStructureActive());
	for (int32 Index = 0; Index < Actor->GetFragmentCount(); ++Index)
	{
		// Flattened sites: every cell is a prism through the whole 10 cm thickness.
		FBox Box(ForceInit);
		for (const FVector& Vertex : Actor->GetFragments()[Index].Vertices)
		{
			Box += Vertex;
		}
		TestTrue(FString::Printf(TEXT("fragment %d spans the full thickness (%f..%f)"), Index, Box.Min.Z, Box.Max.Z),
			FMath::IsNearlyEqual(Box.Min.Z, -5.0, 0.05) && FMath::IsNearlyEqual(Box.Max.Z, 5.0, 0.05));
	}
	const int32 Alive = Actor->CountAliveFragments();
	TestTrue(TEXT("fragments got bodies"), Alive >= 4);
	TestEqual(TEXT("every fragment starts attached"), Actor->CountAttachedFragments(), Alive);

	Test.Step(30);
	for (int32 Index = 0; Index < Actor->GetFragmentCount(); ++Index)
	{
		const FVector Expected = Actor->GetActorTransform().TransformPosition(Actor->GetFragments()[Index].Centroid);
		TestTrue(FString::Printf(TEXT("fragment %d holds still in mid-air"), Index),
			Actor->GetFragmentWorldCentroid(Index).Equals(Expected, 0.01));
	}

	// Point lookup: each centroid maps to its own fragment.
	int32 Target = INDEX_NONE;
	for (int32 Index = 0; Index < Actor->GetFragmentCount() && Target == INDEX_NONE; ++Index)
	{
		if (Actor->IsFragmentAttached(Index))
		{
			Target = Index;
		}
	}
	if (!TestTrue(TEXT("found an attached fragment"), Target != INDEX_NONE))
	{
		return false;
	}
	const FVector TargetCentroid = Actor->GetFragmentWorldCentroid(Target);
	TestEqual(TEXT("centroid lookup hits its own fragment"), Actor->FindFragmentAtPoint(TargetCentroid), Target);
	TestEqual(TEXT("nearest lookup agrees"), Actor->FindNearestFragment(TargetCentroid, 5.0f), Target);
	TestEqual(TEXT("far point finds nothing"), Actor->FindFragmentAtPoint(TargetCentroid + FVector(0, 0, 300)), (int32)INDEX_NONE);

	TestTrue(TEXT("detach succeeds"), Actor->DetachFragment(Target, FVector(0, 0, 300), FVector(0, 0, 2)));
	TestFalse(TEXT("detached fragment is no longer attached"), Actor->IsFragmentAttached(Target));
	TestTrue(TEXT("detached fragment is still alive"), Actor->IsFragmentAlive(Target));
	TestEqual(TEXT("attached count dropped by one"), Actor->CountAttachedFragments(), Alive - 1);
	TestEqual(TEXT("moving chip is excluded from attached lookups"),
		Actor->FindFragmentAtPoint(TargetCentroid, /*bAttachedOnly*/ true), (int32)INDEX_NONE);
	TestFalse(TEXT("detaching an invalid index fails"), Actor->DetachFragment(999, FVector::ZeroVector));

	Test.Step(30);
	const FVector After = Actor->GetFragmentWorldCentroid(Target);
	TestTrue(FString::Printf(TEXT("chip flew (moved %f cm)"), FVector::Dist(After, TargetCentroid)),
		FVector::Dist(After, TargetCentroid) > 5.0);
	for (int32 Index = 0; Index < Actor->GetFragmentCount(); ++Index)
	{
		if (Index == Target || !Actor->IsFragmentAlive(Index))
		{
			continue;
		}
		const FVector Expected = Actor->GetActorTransform().TransformPosition(Actor->GetFragments()[Index].Centroid);
		TestTrue(FString::Printf(TEXT("fragment %d unaffected by the detach"), Index),
			Actor->GetFragmentWorldCentroid(Index).Equals(Expected, 0.01));
	}
	TestEqual(TEXT("no promotions queued: neighbours are anchors themselves"), Actor->GetPendingPromotionCount(), 0);

	Actor->DestroyFragment(Target);
	TestFalse(TEXT("detached chip can be destroyed"), Actor->IsFragmentAlive(Target));
	TestEqual(TEXT("alive count reflects the destroy"), Actor->CountAliveFragments(), Alive - 1);
	Actor->DestroyFragment(Target);
	TestEqual(TEXT("double destroy is a no-op"), Actor->CountAliveFragments(), Alive - 1);
	return true;
}

/// Proxy entry point: a bare convex prism becomes an anchored fractured actor
/// at the given pose with no source component involved, and a nudged detach
/// moves the chip before it flies.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorConvexProxyTest,
	"Box3DUnreal.FracturedActor.ConvexProxyEntry", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorConvexProxyTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	const FFractureProxy Proxy = MakeBoxProxy(FVector(25, 25, 5), FVector(25, 25, 5));
	const FTransform ProxyToWorld(FQuat::Identity, FVector(100, 200, 300));

	FBox3DFractureMeshParams Params;
	Params.Fracture.Sites = { FVector(10, 10, 5), FVector(40, 10, 5), FVector(25, 40, 5) };
	Params.Fracture.FlattenAxis = 2;
	Params.Fracture.ImpactPoint = ProxyToWorld.TransformPosition(FVector(10, 10, 5));
	Params.bStartAsleep = true;
	Params.bStructural = true;
	Params.bAnchorAllFragments = true;
	Params.MaterialToughness = 0.0f;
	Params.HullInsetCm = 0.2f;
	ABox3DFracturedActor* Actor = Box3D::FractureConvexProxy(Test.World, ProxyToWorld, Proxy, nullptr, Params);
	if (!TestNotNull(TEXT("actor spawned from a bare proxy"), Actor))
	{
		return false;
	}
	TestEqual(TEXT("three sites -> three fragments"), Actor->GetFragmentCount(), 3);
	TestTrue(TEXT("placed at the proxy pose"), Actor->GetActorLocation().Equals(FVector(100, 200, 300), 0.01));
	TestTrue(TEXT("anchored structure"), Actor->IsStructureActive());
	TestEqual(TEXT("all attached"), Actor->CountAttachedFragments(), 3);
	TestTrue(TEXT("sections rendered"), Actor->GetMesh()->GetNumSections() >= 3);
	const FVector Expected = ProxyToWorld.TransformPosition(Actor->GetFragments()[0].Centroid);
	TestTrue(TEXT("fragment centroid lands in world space"), Actor->GetFragmentWorldCentroid(0).Equals(Expected, 0.01));

	TestTrue(TEXT("nudged detach"), Actor->DetachFragment(0, FVector::ZeroVector, FVector::ZeroVector, FVector(0, 0, 7)));
	TestTrue(TEXT("chip moved by the nudge before any step"),
		Actor->GetFragmentWorldCentroid(0).Equals(Expected + FVector(0, 0, 7), 0.05));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
