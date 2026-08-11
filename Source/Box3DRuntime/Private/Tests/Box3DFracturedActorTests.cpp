// Tests for the fractured-actor rendering half of D2: proxy resolution
// fallback order (authored convex -> simple collision -> render vertices),
// fragment layout parity between Box3D::FractureMesh and the pure fracture
// core, per-fragment section materials, and the source-component swap-out
// seam (visibility, Chaos collision, static mirror body).

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DFracturedActor.h"
#include "Box3DStaticSceneMirror.h"
#include "Materials/MaterialInterface.h"
#include "PhysicsEngine/BodySetup.h"
#include "ProceduralMeshComponent.h"
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

#endif // WITH_DEV_AUTOMATION_TESTS
