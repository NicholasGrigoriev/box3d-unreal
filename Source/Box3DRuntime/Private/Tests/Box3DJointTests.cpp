// Tests for the M4 joint components: each joint type's constraint behavior,
// limits, motors, springs, and breaking — against hand-placed bodies so the
// expected geometry is analytic.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DJointComponent.h"
#include "Tests/Box3DTestEventCounter.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	template <typename TJoint>
	TJoint* SpawnJoint(UBox3DBodyComponent* Body, const FTransform& JointWorld,
		TFunctionRef<void(TJoint&)> Setup)
	{
		TJoint* Joint = NewObject<TJoint>(Body->GetOwner());
		Joint->SetupAttachment(Body);
		Joint->SetWorldTransform(JointWorld);
		Setup(*Joint);
		Joint->RegisterComponent();
		return Joint;
	}

	UBox3DBodyComponent* SpawnSphereBody(Box3DTest::FTestWorld& Test, const FVector& Location, float Radius,
		float GravityScale = 1.0f)
	{
		return Box3DTest::SpawnBody(Test.World, Location, [&](UBox3DBodyComponent& B)
		{
			B.BodyType = EBox3DBodyType::Dynamic;
			B.ShapeType = EBox3DShapeType::Sphere;
			B.SphereRadius = Radius;
			B.GravityScale = GravityScale;
		});
	}

	UBox3DBodyComponent* SpawnBoxBody(Box3DTest::FTestWorld& Test, const FVector& Location, float HalfExtent,
		float GravityScale = 1.0f)
	{
		return Box3DTest::SpawnBody(Test.World, Location, [&](UBox3DBodyComponent& B)
		{
			B.BodyType = EBox3DBodyType::Dynamic;
			B.ShapeType = EBox3DShapeType::Box;
			B.BoxHalfExtent = FVector(HalfExtent);
			B.GravityScale = GravityScale;
		});
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DJointDistanceTest,
	"Box3DUnreal.Joints.DistanceRopeAndBreak", BOX3D_TEST_FLAGS)
bool FBox3DJointDistanceTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// A rigid 100 cm rope to a world anchor: the bob hangs exactly 100 below.
	UBox3DBodyComponent* Bob = SpawnSphereBody(Test, FVector(0, 0, 400), 25.0f);
	UBox3DDistanceJointComponent* Rope = SpawnJoint<UBox3DDistanceJointComponent>(
		Bob, FTransform(FVector(0, 0, 400)), [](UBox3DDistanceJointComponent& J)
	{
		J.bAutoLength = false;
		J.Length = 100.0f;
	});
	TestTrue(TEXT("rope joint created on BeginPlay"), Rope->IsJointActive());

	Test.Step(180); // 3 s to settle
	const FVector BobLocation = Bob->GetComponentLocation();
	TestEqual(TEXT("bob hangs at rope length below the anchor"),
		static_cast<float>(BobLocation.Z), 300.0f, 3.0f);
	TestTrue(TEXT("bob stays on the anchor vertical"),
		FMath::Abs(BobLocation.X) < 1.0 && FMath::Abs(BobLocation.Y) < 1.0);

	// The rope carries exactly the bob's weight: |F| = m*g in newtons.
	const float Weight = Bob->GetMass() * 9.8f;
	const float RopeForce = static_cast<float>(Rope->GetConstraintForce().Size());
	TestEqual(TEXT("constraint force equals the hanging weight"), RopeForce, Weight, Weight * 0.15f);

	// A weak breakable rope holding a 1000 kg box must snap and report it.
	UBox3DTestEventCounter* Counter = NewObject<UBox3DTestEventCounter>();
	UBox3DBodyComponent* Heavy = SpawnBoxBody(Test, FVector(300, 0, 400), 50.0f);
	UBox3DDistanceJointComponent* Weak = SpawnJoint<UBox3DDistanceJointComponent>(
		Heavy, FTransform(FVector(300, 0, 400)), [](UBox3DDistanceJointComponent& J)
	{
		J.bAutoLength = false;
		J.Length = 50.0f;
		J.bBreakable = true;
		J.BreakForce = 100.0f; // the box weighs ~9800 N
	});
	Weak->OnJointBroke.AddDynamic(Counter, &UBox3DTestEventCounter::HandleJointBroke);

	Test.Step(120);
	TestFalse(TEXT("overloaded joint broke"), Weak->IsJointActive());
	TestEqual(TEXT("OnJointBroke fired exactly once"), Counter->JointBrokeCount, 1);
	TestTrue(FString::Printf(TEXT("freed body fell past the rope length (Z=%.1f)"),
		Heavy->GetComponentLocation().Z), Heavy->GetComponentLocation().Z < 250.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DJointRevoluteTest,
	"Box3DUnreal.Joints.RevoluteHingeAndMotor", BOX3D_TEST_FLAGS)
bool FBox3DJointRevoluteTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Hinge axis = world Y (joint local Z rotated onto Y): a pendulum arm in
	// the XZ plane, pivoting about a world anchor.
	const FQuat HingeRot = FQuat::FindBetweenNormals(FVector::UpVector, FVector::RightVector);
	const FVector Pivot(0, 0, 300);
	UBox3DBodyComponent* Arm = SpawnBoxBody(Test, Pivot + FVector(100, 0, 0), 10.0f);
	SpawnJoint<UBox3DRevoluteJointComponent>(Arm, FTransform(HingeRot, Pivot),
		[](UBox3DRevoluteJointComponent&) {});

	for (int32 Sample = 0; Sample < 3; ++Sample)
	{
		Test.Step(15);
		const double Distance = FVector::Dist(Arm->GetComponentLocation(), Pivot);
		TestEqual(FString::Printf(TEXT("arm stays at hinge radius (sample %d)"), Sample),
			static_cast<float>(Distance), 100.0f, 2.5f);
		TestTrue(TEXT("arm stays in the hinge plane"), FMath::Abs(Arm->GetComponentLocation().Y) < 1.5);
	}
	TestTrue(TEXT("arm swung downward under gravity"), Arm->GetComponentLocation().Z < 295.0);

	// Motor drive without gravity: the arm spins up to the motor speed.
	const FVector Pivot2(500, 0, 300);
	UBox3DBodyComponent* Rotor = SpawnBoxBody(Test, Pivot2 + FVector(100, 0, 0), 10.0f, /*gravity*/ 0.0f);
	SpawnJoint<UBox3DRevoluteJointComponent>(Rotor, FTransform(HingeRot, Pivot2),
		[](UBox3DRevoluteJointComponent& J)
	{
		J.bEnableMotor = true;
		J.MotorSpeed = 90.0f; // deg/s
		J.MaxMotorTorque = 100000.0f;
	});

	Test.Step(60);
	const FVector Omega = Rotor->GetAngularVelocity();
	TestEqual(TEXT("motor reaches 90 deg/s"), static_cast<float>(Omega.Size()), UE_HALF_PI, UE_HALF_PI * 0.1f);
	TestTrue(FString::Printf(TEXT("spin axis is the hinge axis (w=%s)"), *Omega.ToCompactString()),
		FMath::Abs(Omega.Y) / FMath::Max(Omega.Size(), 0.001) > 0.95);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DJointPrismaticTest,
	"Box3DUnreal.Joints.PrismaticSlideAndLimits", BOX3D_TEST_FLAGS)
bool FBox3DJointPrismaticTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Slide axis = world Z (joint local X rotated onto Z): a vertical elevator
	// rail with 50 cm of downward travel.
	const FQuat SlideRot = FQuat::FindBetweenNormals(FVector::ForwardVector, FVector::UpVector);
	const FVector Start(0, 0, 500);
	UBox3DBodyComponent* Slider = SpawnBoxBody(Test, Start, 25.0f);
	UBox3DPrismaticJointComponent* Rail = SpawnJoint<UBox3DPrismaticJointComponent>(
		Slider, FTransform(SlideRot, Start), [](UBox3DPrismaticJointComponent& J)
	{
		J.bEnableLimit = true;
		J.LowerTranslation = -50.0f;
		J.UpperTranslation = 0.0f;
	});

	Test.Step(120);
	const FVector End = Slider->GetComponentLocation();
	TestEqual(TEXT("slider rests on the lower limit"), static_cast<float>(End.Z), 450.0f, 1.5f);
	TestTrue(TEXT("no lateral drift"), FMath::Abs(End.X) < 0.5 && FMath::Abs(End.Y) < 0.5);
	TestEqual(TEXT("GetTranslation reports the limit"), Rail->GetTranslation(), -50.0f, 1.5f);
	TestTrue(TEXT("prismatic locks rotation"),
		Slider->GetComponentQuat().Equals(FQuat::Identity, 0.01f));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DJointSphericalTest,
	"Box3DUnreal.Joints.SphericalPendulum", BOX3D_TEST_FLAGS)
bool FBox3DJointSphericalTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Ball-and-socket at a world anchor, body origin 100 cm away: a pendulum
	// free to swing in any direction but pinned to the radius.
	const FVector Pivot(0, 0, 300);
	UBox3DBodyComponent* Bob = SpawnSphereBody(Test, Pivot + FVector(100, 0, 0), 10.0f);
	SpawnJoint<UBox3DSphericalJointComponent>(Bob, FTransform(Pivot),
		[](UBox3DSphericalJointComponent&) {});

	for (int32 Sample = 0; Sample < 3; ++Sample)
	{
		Test.Step(15);
		const double Distance = FVector::Dist(Bob->GetComponentLocation(), Pivot);
		TestEqual(FString::Printf(TEXT("bob stays on the socket radius (sample %d)"), Sample),
			static_cast<float>(Distance), 100.0f, 2.5f);
	}
	TestTrue(TEXT("bob swung downward under gravity"), Bob->GetComponentLocation().Z < 295.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DJointWeldTest,
	"Box3DUnreal.Joints.WeldRigid", BOX3D_TEST_FLAGS)
bool FBox3DJointWeldTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Two welded boxes; an impulse along the line of centers moves both without
	// changing their relative pose.
	UBox3DBodyComponent* A = SpawnBoxBody(Test, FVector(0, 0, 300), 25.0f, /*gravity*/ 0.0f);
	UBox3DBodyComponent* B = SpawnBoxBody(Test, FVector(100, 0, 300), 25.0f, /*gravity*/ 0.0f);
	UBox3DWeldJointComponent* Weld = SpawnJoint<UBox3DWeldJointComponent>(
		B, FTransform(FVector(50, 0, 300)), [&](UBox3DWeldJointComponent& J)
	{
		J.ConnectedActor = A->GetOwner();
	});
	TestTrue(TEXT("weld created"), Weld->IsJointActive());

	A->AddImpulse(FVector(static_cast<double>(A->GetMass()) * 150.0, 0, 0));
	Test.Step(60);

	const FVector Delta = B->GetComponentLocation() - A->GetComponentLocation();
	TestTrue(FString::Printf(TEXT("welded pair keeps its relative offset (delta=%s)"), *Delta.ToCompactString()),
		Delta.Equals(FVector(100, 0, 0), 3.0));
	TestTrue(TEXT("the pair actually moved"), A->GetComponentLocation().X > 30.0);
	TestTrue(TEXT("no relative rotation"),
		A->GetComponentQuat().Equals(B->GetComponentQuat(), 0.02f));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DJointMotorTest,
	"Box3DUnreal.Joints.MotorJointVelocity", BOX3D_TEST_FLAGS)
bool FBox3DJointMotorTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Motor joint against a world anchor drives the body to the target velocity.
	UBox3DBodyComponent* Body = SpawnBoxBody(Test, FVector(0, 0, 300), 25.0f, /*gravity*/ 0.0f);
	SpawnJoint<UBox3DMotorJointComponent>(Body, FTransform(FVector(0, 0, 300)),
		[](UBox3DMotorJointComponent& J)
	{
		J.LinearVelocity = FVector(100, 0, 0); // cm/s
		J.MaxVelocityForce = 1.0e6f;
	});

	Test.Step(60);
	TestTrue(FString::Printf(TEXT("motor joint reaches the target velocity (v=%s)"),
		*Body->GetLinearVelocity().ToCompactString()),
		Body->GetLinearVelocity().Equals(FVector(100, 0, 0), 5.0));
	TestTrue(TEXT("body advanced along the drive direction"), Body->GetComponentLocation().X > 50.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DJointWheelTest,
	"Box3DUnreal.Joints.WheelSuspensionAndSpin", BOX3D_TEST_FLAGS)
bool FBox3DJointWheelTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Wheel frame: suspension axis (local X) up, spin axis (local Z) along
	// world Y — a wheel hanging off a fixed chassis point on a spring.
	const FMatrix FrameMatrix(FVector(0, 0, 1), FVector(1, 0, 0), FVector(0, 1, 0), FVector::ZeroVector);
	const FQuat WheelRot = FrameMatrix.ToQuat();
	const FVector Mount(0, 0, 300);

	UBox3DBodyComponent* Wheel = SpawnSphereBody(Test, Mount, 25.0f);
	UBox3DWheelJointComponent* Suspension = SpawnJoint<UBox3DWheelJointComponent>(
		Wheel, FTransform(WheelRot, Mount), [](UBox3DWheelJointComponent& J)
	{
		J.SuspensionHertz = 2.0f;
		J.SuspensionDampingRatio = 0.7f;
		J.LowerSuspensionLimit = -50.0f;
		J.UpperSuspensionLimit = 0.0f;
		J.bEnableSpinMotor = true;
		J.SpinSpeed = 180.0f; // deg/s
		J.MaxSpinTorque = 10000.0f;
	});
	TestTrue(TEXT("wheel joint created"), Suspension->IsJointActive());

	Test.Step(300); // 5 s: spring settles, motor spins up
	const FVector End = Wheel->GetComponentLocation();
	// Analytic sag: g / (2*pi*hertz)^2 = 9.8/157.9 m = ~6.2 cm below the mount.
	TestEqual(TEXT("suspension sags to the spring equilibrium"),
		static_cast<float>(End.Z), 300.0f - 6.2f, 3.0f);
	TestTrue(TEXT("wheel stays on the suspension axis"),
		FMath::Abs(End.X) < 1.0 && FMath::Abs(End.Y) < 1.0);

	const FVector Omega = Wheel->GetAngularVelocity();
	TestEqual(TEXT("spin motor reaches 180 deg/s"), static_cast<float>(Omega.Size()), UE_PI, UE_PI * 0.15f);
	TestTrue(FString::Printf(TEXT("spin axis is the wheel axle (w=%s)"), *Omega.ToCompactString()),
		FMath::Abs(Omega.Y) / FMath::Max(Omega.Size(), 0.001) > 0.95);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
