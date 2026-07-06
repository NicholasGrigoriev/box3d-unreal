// Tests for the M4 gameplay events: contact begin/end, hit events with
// approach speed, and sensor overlaps — counted through dynamic delegates on
// a UObject sink, exactly the way Blueprint would consume them.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Tests/Box3DTestEventCounter.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	UBox3DBodyComponent* SpawnEventBox(Box3DTest::FTestWorld& Test, const FVector& Location,
		TFunctionRef<void(UBox3DBodyComponent&)> Extra)
	{
		return Box3DTest::SpawnBody(Test.World, Location, [&](UBox3DBodyComponent& B)
		{
			B.BodyType = EBox3DBodyType::Dynamic;
			B.ShapeType = EBox3DShapeType::Box;
			B.BoxHalfExtent = FVector(25.0);
			Extra(B);
		});
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DEventContactTest,
	"Box3DUnreal.Events.ContactBeginEnd", BOX3D_TEST_FLAGS)
bool FBox3DEventContactTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UBox3DBodyComponent* Ground = Box3DTest::SpawnGround(Test.World);

	UBox3DTestEventCounter* Counter = NewObject<UBox3DTestEventCounter>();
	UBox3DBodyComponent* Box = SpawnEventBox(Test, FVector(0, 0, 100),
		[](UBox3DBodyComponent& B) { B.bEnableContactEvents = true; });
	Box->OnContactBegin.AddDynamic(Counter, &UBox3DTestEventCounter::HandleContactBegin);
	Box->OnContactEnd.AddDynamic(Counter, &UBox3DTestEventCounter::HandleContactEnd);

	// Control body without the flag must stay silent.
	UBox3DTestEventCounter* ControlCounter = NewObject<UBox3DTestEventCounter>();
	UBox3DBodyComponent* Control = SpawnEventBox(Test, FVector(300, 0, 100), [](UBox3DBodyComponent&) {});
	Control->OnContactBegin.AddDynamic(ControlCounter, &UBox3DTestEventCounter::HandleContactBegin);

	Test.Step(60); // both land on the ground
	TestTrue(TEXT("contact begin fired on landing"), Counter->ContactBeginCount >= 1);
	TestTrue(TEXT("begin reports the ground as the other body"), Counter->LastContactOther == Ground);
	TestEqual(TEXT("no contact end while resting"), Counter->ContactEndCount, 0);
	TestEqual(TEXT("body without the flag gets no events"), ControlCounter->ContactBeginCount, 0);

	// Launch the box up: the touching pair separates and end fires.
	Box->SetLinearVelocity(FVector(0, 0, 500));
	Test.Step(30);
	TestTrue(TEXT("contact end fired on separation"), Counter->ContactEndCount >= 1);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DEventHitTest,
	"Box3DUnreal.Events.HitApproachSpeed", BOX3D_TEST_FLAGS)
bool FBox3DEventHitTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	// Drop from 225 cm of clearance: impact speed sqrt(2*980*225) = ~664 cm/s,
	// far above the 100 cm/s default hit threshold.
	UBox3DTestEventCounter* Counter = NewObject<UBox3DTestEventCounter>();
	UBox3DBodyComponent* Box = SpawnEventBox(Test, FVector(0, 0, 250),
		[](UBox3DBodyComponent& B) { B.bEnableHitEvents = true; });
	Box->OnHit.AddDynamic(Counter, &UBox3DTestEventCounter::HandleHit);

	Test.Step(60);
	TestTrue(TEXT("hit event fired on impact"), Counter->HitCount >= 1);
	TestEqual(TEXT("approach speed matches free-fall impact"), Counter->LastApproachSpeed, 664.0f, 80.0f);
	TestTrue(FString::Printf(TEXT("hit normal points from the ground into the box (n=%s)"),
		*Counter->LastHitNormal.ToCompactString()), Counter->LastHitNormal.Z > 0.9);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DEventSensorTest,
	"Box3DUnreal.Events.SensorOverlap", BOX3D_TEST_FLAGS)
bool FBox3DEventSensorTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// A static sensor volume; a sphere falls straight through it (sensors have
	// no collision response) and must produce one begin and one end.
	UBox3DTestEventCounter* Counter = NewObject<UBox3DTestEventCounter>();
	UBox3DBodyComponent* Sensor = Box3DTest::SpawnBody(Test.World, FVector(0, 0, 100), [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Static;
		B.ShapeType = EBox3DShapeType::Box;
		B.BoxHalfExtent = FVector(50.0);
		B.bIsSensor = true;
	});
	Sensor->OnSensorBegin.AddDynamic(Counter, &UBox3DTestEventCounter::HandleSensorBegin);
	Sensor->OnSensorEnd.AddDynamic(Counter, &UBox3DTestEventCounter::HandleSensorEnd);

	UBox3DBodyComponent* Visitor = Box3DTest::SpawnBody(Test.World, FVector(0, 0, 300), [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Dynamic;
		B.ShapeType = EBox3DShapeType::Sphere;
		B.SphereRadius = 25.0f;
	});

	Test.Step(120); // 2 s: enters at ~0.5 s, exits at ~0.75 s, keeps falling
	TestTrue(TEXT("sensor begin fired"), Counter->SensorBeginCount >= 1);
	TestTrue(TEXT("sensor end fired"), Counter->SensorEndCount >= 1);
	TestTrue(TEXT("visitor resolved to the falling body"), Counter->LastVisitor == Visitor);
	TestTrue(FString::Printf(TEXT("sensor produced no collision response (Z=%.1f)"),
		Visitor->GetComponentLocation().Z), Visitor->GetComponentLocation().Z < 0.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
