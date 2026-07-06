// Tests for UBox3DWorldSubsystem: world lifecycle, fixed-step accumulator
// semantics, and gravity integration (M0).

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DConversion.h"
#include "Tests/Box3DTestHelpers.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DWorldLifecycleTest,
	"Box3DUnreal.World.SubsystemLifecycle", BOX3D_TEST_FLAGS)
bool FBox3DWorldLifecycleTest::RunTest(const FString& Parameters)
{
	{
		Box3DTest::FTestWorld Test;

		UBox3DWorldSubsystem* Subsystem = Test.World->GetSubsystem<UBox3DWorldSubsystem>();
		TestNotNull(TEXT("game worlds get the subsystem"), Subsystem);
		if (Subsystem == nullptr)
		{
			return false;
		}

		TestTrue(TEXT("Box3D world is valid"), b3World_IsValid(Subsystem->GetBox3DWorldId()));
		TestEqual(TEXT("no steps before first tick"), static_cast<int32>(Subsystem->GetStepCount()), 0);

		const FVector Gravity = Box3D::ToUE(b3World_GetGravity(Subsystem->GetBox3DWorldId()));
		TestTrue(FString::Printf(TEXT("world gravity matches settings (%s)"), *Gravity.ToCompactString()),
			Gravity.Equals(GetDefault<UBox3DSettings>()->Gravity, 0.01));
	} // fixture teardown destroys the world; a leak or double-destroy would assert here

	// Editor worlds must not get a Box3D world.
	if (GWorld != nullptr && GWorld->WorldType == EWorldType::Editor)
	{
		TestNull(TEXT("editor worlds have no subsystem"), GWorld->GetSubsystem<UBox3DWorldSubsystem>());
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DWorldAccumulatorTest,
	"Box3DUnreal.World.FixedStepAccumulator", BOX3D_TEST_FLAGS)
bool FBox3DWorldAccumulatorTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UBox3DWorldSubsystem& Subsystem = Test.Subsystem();
	const float FixedDt = Box3DTest::FTestWorld::FixedDt();

	// Half a step of frame time: nothing happens yet.
	Subsystem.Tick(0.5f * FixedDt);
	TestEqual(TEXT("half a step accumulates without stepping"), static_cast<int32>(Subsystem.GetStepCount()), 0);

	// Another 0.6 steps crosses the threshold exactly once.
	Subsystem.Tick(0.6f * FixedDt);
	TestEqual(TEXT("crossing one fixed step steps once"), static_cast<int32>(Subsystem.GetStepCount()), 1);

	// A huge hitch is clamped to MaxStepsPerTick, not simulated in full. Allow
	// one step of slack: the clamped accumulator drains by repeated float
	// subtraction, so the final step can round either way.
	const int32 MaxSteps = GetDefault<UBox3DSettings>()->MaxStepsPerTick;
	Subsystem.Tick(100.0f * FixedDt);
	const int32 HitchSteps = static_cast<int32>(Subsystem.GetStepCount()) - 1;
	TestTrue(FString::Printf(TEXT("hitch clamps to MaxStepsPerTick (%d steps for a 100-step hitch)"), HitchSteps),
		HitchSteps >= MaxSteps - 1 && HitchSteps <= MaxSteps);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DWorldFreeFallTest,
	"Box3DUnreal.World.FreeFallGravity", BOX3D_TEST_FLAGS)
bool FBox3DWorldFreeFallTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	const FVector Start(0.0, 0.0, 100000.0);
	UBox3DBodyComponent* Body = Box3DTest::SpawnBody(Test.World, Start, [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Dynamic;
		B.ShapeType = EBox3DShapeType::Box;
		B.BoxHalfExtent = FVector(25.0);
	});

	// One second of free fall: semi-implicit integration gives v = g*t exactly.
	Test.Step(60);
	const float GravityZ = GetDefault<UBox3DSettings>()->Gravity.Z; // cm/s^2
	const FVector Velocity = Body->GetLinearVelocity();
	TestEqual(TEXT("free-fall velocity is g*t after 1 s"), static_cast<float>(Velocity.Z), GravityZ, FMath::Abs(GravityZ) * 0.01f);

	// The component transform followed the body down (physics -> UE sync).
	const double Dropped = Start.Z - Body->GetComponentLocation().Z;
	TestTrue(FString::Printf(TEXT("component dropped ~0.5*g*t^2 (%f cm)"), Dropped),
		Dropped > 400.0 && Dropped < 600.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
