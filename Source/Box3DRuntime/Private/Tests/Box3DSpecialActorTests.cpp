// Breakable assemblies, conveyor belts, and wind sources — the special-actor
// zoo built on raw bodies, weld joints, surface materials, and the pre-step
// force hook.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DBreakableActor.h"
#include "Box3DConveyorActor.h"
#include "Box3DWindActor.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSpecialBreakableShattersTest,
	"Box3DUnreal.Special.BreakableShatters", BOX3D_TEST_FLAGS)
bool FBox3DSpecialBreakableShattersTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;
	Box3DTest::SpawnGround(TestWorld.World, 0.0f);

	// Two 30 cm cube chunks stacked on the ground, welded.
	const FTransform Transform = FTransform::Identity;
	ABox3DBreakableActor* Breakable = TestWorld.World->SpawnActorDeferred<ABox3DBreakableActor>(
		ABox3DBreakableActor::StaticClass(), Transform);
	Breakable->BreakForce = 100000.0f;
	Breakable->bStartAsleep = false;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();
	Breakable->AddChunk(Cube, FTransform(FQuat::Identity, FVector(0.0, 0.0, 15.0), FVector(0.3)));
	Breakable->AddChunk(Cube, FTransform(FQuat::Identity, FVector(0.0, 0.0, 45.0), FVector(0.3)));
	Breakable->FinishSpawning(Transform);

	TestEqual(TEXT("Two chunks built"), Breakable->GetChunkCount(), 2);
	TestEqual(TEXT("Touching chunks welded"), Breakable->GetLiveWeldCount(), 1);

	TestWorld.Step(120);
	// Component enumeration order is not creation order: identify by height.
	UBox3DBodyComponent* First = Breakable->GetChunkBody(0);
	UBox3DBodyComponent* Second = Breakable->GetChunkBody(1);
	if (!TestNotNull(TEXT("Chunk 0 body exists"), First) || !TestNotNull(TEXT("Chunk 1 body exists"), Second))
	{
		return false;
	}
	UBox3DBodyComponent* Top = First->GetComponentLocation().Z > Second->GetComponentLocation().Z ? First : Second;
	UBox3DBodyComponent* Bottom = Top == First ? Second : First;
	TestEqual(TEXT("Sturdy weld holds the stack"), Top->GetComponentLocation().Z, 45.0, 8.0);

	// Drop the threshold below the top chunk's resting weight (~106 N). The
	// settled stack is asleep and sleeping joints report zero force, so wake
	// it and let a step measure the load before checking.
	Breakable->BreakForce = 50.0f;
	Top->SetAwake(true);
	TestWorld.Step(2);
	Breakable->CheckWelds();
	TestEqual(TEXT("Overloaded weld snaps"), Breakable->GetLiveWeldCount(), 0);

	// Freed chunk slides away independently.
	Top->AddImpulse(FVector(30000.0, 0.0, 0.0));
	TestWorld.Step(60);
	TestTrue(TEXT("Freed chunk moves away from its neighbor"),
		Top->GetComponentLocation().X > Bottom->GetComponentLocation().X + 40.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSpecialConveyorDragsTest,
	"Box3DUnreal.Special.ConveyorDrags", BOX3D_TEST_FLAGS)
bool FBox3DSpecialConveyorDragsTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;

	// Default belt: 400x150x20 cm, surface at Z=10, running along +X.
	ABox3DConveyorActor* Conveyor = TestWorld.World->SpawnActor<ABox3DConveyorActor>(
		ABox3DConveyorActor::StaticClass(), FTransform::Identity);
	if (!TestNotNull(TEXT("Conveyor spawned"), Conveyor))
	{
		return false;
	}

	UBox3DBodyComponent* Box = Box3DTest::SpawnBody(TestWorld.World, FVector(-150.0, 0.0, 40.0),
		[](UBox3DBodyComponent& Body)
		{
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(15.0);
		});

	TestWorld.Step(120); // 2 s: land on the belt and get dragged
	TestTrue(TEXT("Belt drags the box along +X"), Box->GetComponentLocation().X > -50.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSpecialWindBlowsTest,
	"Box3DUnreal.Special.WindBlows", BOX3D_TEST_FLAGS)
bool FBox3DSpecialWindBlowsTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;

	const FTransform Transform = FTransform::Identity; // forward = +X
	ABox3DWindActor* Wind = TestWorld.World->SpawnActorDeferred<ABox3DWindActor>(
		ABox3DWindActor::StaticClass(), Transform);
	Wind->WindMode = EBox3DWindMode::Directional;
	Wind->WindSpeed = 1000.0f;
	Wind->Radius = 5000.0f;
	Wind->Drag = 30.0f;
	Wind->GustAmount = 0.0f;
	Wind->FinishSpawning(Transform);

	// Gravity-free sphere in range: the drag force should push it downwind.
	UBox3DBodyComponent* Ball = Box3DTest::SpawnBody(TestWorld.World, FVector(200.0, 0.0, 0.0),
		[](UBox3DBodyComponent& Body)
		{
			Body.ShapeType = EBox3DShapeType::Sphere;
			Body.SphereRadius = 25.0f;
			Body.GravityScale = 0.0f;
		});

	TestWorld.Step(120); // 2 s of wind
	TestTrue(TEXT("Wind accelerates the body downwind"), Ball->GetLinearVelocity().X > 100.0);
	TestTrue(TEXT("Body was carried downwind"), Ball->GetComponentLocation().X > 250.0);
	TestTrue(TEXT("Crosswind drift stays negligible"), FMath::Abs(Ball->GetComponentLocation().Y) < 10.0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
