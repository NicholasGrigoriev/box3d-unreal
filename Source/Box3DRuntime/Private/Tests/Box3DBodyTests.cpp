// Tests for UBox3DBodyComponent (M1): lifecycle, analytic masses, body flags,
// the velocity/force/impulse API and its unit scaling, transform ownership per
// body type, kinematic target tracking, collision filtering, and materials.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DConversion.h"
#include "Box3DQueryLibrary.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	UBox3DBodyComponent* SpawnTestBox(Box3DTest::FTestWorld& Test, const FVector& Location,
		float HalfExtentCm = 25.0f, TFunction<void(UBox3DBodyComponent&)> Extra = nullptr)
	{
		return Box3DTest::SpawnBody(Test.World, Location, [&](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Dynamic;
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(HalfExtentCm);
			if (Extra)
			{
				Extra(Body);
			}
		});
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyLifecycleTest,
	"Box3DUnreal.Body.CreateDestroy", BOX3D_TEST_FLAGS)
bool FBox3DBodyLifecycleTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UBox3DBodyComponent* Body = SpawnTestBox(Test, FVector(0, 0, 500));
	TestTrue(TEXT("body simulates after BeginPlay"), Body->IsSimulating());
	TestTrue(TEXT("body id valid"), b3Body_IsValid(Body->GetBodyId()));
	TestTrue(TEXT("userData points back at the component"),
		b3Body_GetUserData(Body->GetBodyId()) == Body);

	AActor* Owner = Body->GetOwner();
	Owner->Destroy();
	TestFalse(TEXT("body destroyed with the actor"), Body->IsSimulating());

	// The world keeps stepping fine after the body is gone.
	Test.Step(2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyMassTest,
	"Box3DUnreal.Body.MassAnalytic", BOX3D_TEST_FLAGS)
bool FBox3DBodyMassTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Density 1000 kg/m^3 throughout (the component default).
	UBox3DBodyComponent* Box = SpawnTestBox(Test, FVector(0, 0, 500), 50.0f);
	TestEqual(TEXT("1 m box weighs 1000 kg"), Box->GetMass(), 1000.0f, 1.0f);

	UBox3DBodyComponent* Sphere = Box3DTest::SpawnBody(Test.World, FVector(300, 0, 500), [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Dynamic;
		B.ShapeType = EBox3DShapeType::Sphere;
		B.SphereRadius = 50.0f;
	});
	TestEqual(TEXT("r=0.5 m sphere weighs 4/3*pi*r^3*rho"), Sphere->GetMass(), 523.6f, 2.0f);

	// Capsule r=25, UE half height 75 -> segment half 50 cm: cylinder 1 m + sphere caps.
	UBox3DBodyComponent* Capsule = Box3DTest::SpawnBody(Test.World, FVector(600, 0, 500), [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Dynamic;
		B.ShapeType = EBox3DShapeType::Capsule;
		B.CapsuleRadius = 25.0f;
		B.CapsuleHalfHeight = 75.0f;
	});
	TestEqual(TEXT("capsule mass = cylinder + caps"), Capsule->GetMass(), 261.8f, 2.0f);

	UBox3DBodyComponent* Static = Box3DTest::SpawnBody(Test.World, FVector(900, 0, 500), [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Static;
		B.ShapeType = EBox3DShapeType::Box;
		B.BoxHalfExtent = FVector(25.0);
	});
	TestEqual(TEXT("static bodies have no mass"), Static->GetMass(), 0.0f, KINDA_SMALL_NUMBER);

	// Component world scale applies to explicit extents: x2 scale -> x8 mass.
	UBox3DBodyComponent* Scaled = Box3DTest::SpawnBody(Test.World, FVector(0, 300, 500), [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Dynamic;
		B.ShapeType = EBox3DShapeType::Box;
		B.BoxHalfExtent = FVector(25.0);
	}, FQuat::Identity, FVector(2.0));
	TestEqual(TEXT("x2 component scale gives x8 mass"), Scaled->GetMass(), 8.0f * 125.0f, 1.0f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyGravityAndLocksTest,
	"Box3DUnreal.Body.GravityScaleAndMotionLocks", BOX3D_TEST_FLAGS)
bool FBox3DBodyGravityAndLocksTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UBox3DBodyComponent* NoGravity = SpawnTestBox(Test, FVector(0, 0, 500), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.GravityScale = 0.0f;
	});
	UBox3DBodyComponent* HalfGravity = SpawnTestBox(Test, FVector(300, 0, 500), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.GravityScale = 0.5f;
	});
	UBox3DBodyComponent* LockedZ = SpawnTestBox(Test, FVector(600, 0, 500), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.MotionLocks.bLinearZ = true;
	});
	UBox3DBodyComponent* LockedSpin = SpawnTestBox(Test, FVector(900, 0, 500), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.GravityScale = 0.0f;
		B.MotionLocks.bAngularX = true;
		B.MotionLocks.bAngularY = true;
		B.MotionLocks.bAngularZ = true;
	});
	LockedSpin->AddTorque(FVector(1.0e9, 0, 0));

	Test.Step(30); // 0.5 s
	const float GravityZ = GetDefault<UBox3DSettings>()->Gravity.Z;

	TestTrue(TEXT("gravity scale 0 body hovers"),
		NoGravity->GetLinearVelocity().IsNearlyZero(0.1) &&
		FMath::IsNearlyEqual(NoGravity->GetComponentLocation().Z, 500.0, 0.1));
	TestEqual(TEXT("gravity scale 0.5 falls at half rate"),
		static_cast<float>(HalfGravity->GetLinearVelocity().Z), 0.25f * GravityZ, FMath::Abs(GravityZ) * 0.005f);
	TestTrue(TEXT("linear Z lock holds altitude under gravity"),
		FMath::IsNearlyEqual(LockedZ->GetComponentLocation().Z, 500.0, 0.1) &&
		FMath::IsNearlyZero(LockedZ->GetLinearVelocity().Z, 0.1));
	TestTrue(TEXT("angular locks stop torque from spinning the body"),
		LockedSpin->GetAngularVelocity().IsNearlyZero(0.001));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyVelocityImpulseTest,
	"Box3DUnreal.Body.VelocityForceImpulse", BOX3D_TEST_FLAGS)
bool FBox3DBodyVelocityImpulseTest::RunTest(const FString& Parameters)
{
	// Exact-position assertions need non-interpolated component sync; the project
	// config may enable interpolation, which lags transforms by one fixed step.
	Box3DTest::FScopedInterpolationSettings NoInterpolation(false);
	Box3DTest::FTestWorld Test;
	const float FixedDt = Box3DTest::FTestWorld::FixedDt();

	auto SpawnFloating = [&](const FVector& Location)
	{
		return SpawnTestBox(Test, Location, 25.0f, [](UBox3DBodyComponent& B) { B.GravityScale = 0.0f; });
	};

	// Set/Get round trip and constant-velocity integration.
	UBox3DBodyComponent* Cruiser = SpawnFloating(FVector(0, 0, 500));
	Cruiser->SetLinearVelocity(FVector(123.0, 0, 0));
	TestTrue(TEXT("linear velocity round trip"),
		Cruiser->GetLinearVelocity().Equals(FVector(123.0, 0, 0), 0.01));
	Test.Step(60);
	TestEqual(TEXT("constant velocity moves v*t in 1 s"),
		static_cast<float>(Cruiser->GetComponentLocation().X), 123.0f, 1.0f);

	// Impulse: dv = J / m. J in UE units is kg*cm/s, so J = m * dv directly.
	UBox3DBodyComponent* Kicked = SpawnFloating(FVector(0, 300, 500));
	const float Mass = Kicked->GetMass();
	Kicked->AddImpulse(FVector(0, static_cast<double>(Mass) * 250.0, 0));
	TestEqual(TEXT("impulse of m*250 gives 250 cm/s"),
		static_cast<float>(Kicked->GetLinearVelocity().Y), 250.0f, 1.0f);

	// Force applied before one step: dv = F/m * dt.
	UBox3DBodyComponent* Pushed = SpawnFloating(FVector(0, 600, 500));
	Pushed->AddForce(FVector(static_cast<double>(Pushed->GetMass()) * 980.0, 0, 0));
	Test.Step(1);
	TestEqual(TEXT("force of m*980 for one step gives 980*dt cm/s"),
		static_cast<float>(Pushed->GetLinearVelocity().X), 980.0f * FixedDt, 980.0f * FixedDt * 0.02f);

	// Angular: sphere inertia I = 2/5 m r^2 is analytic. Angular impulse L in UE
	// units is kg*cm^2/s; expected omega = L * 1e-4 / I.
	UBox3DBodyComponent* Spun = Box3DTest::SpawnBody(Test.World, FVector(0, 900, 500), [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Dynamic;
		B.ShapeType = EBox3DShapeType::Sphere;
		B.SphereRadius = 50.0f;
		B.GravityScale = 0.0f;
	});
	const float Inertia = 0.4f * Spun->GetMass() * 0.5f * 0.5f; // kg*m^2
	Spun->AddAngularImpulse(FVector(static_cast<double>(Inertia) * 1.0e4, 0, 0)); // -> 1 rad/s
	TestEqual(TEXT("angular impulse of I/TorqueScale gives 1 rad/s"),
		static_cast<float>(Spun->GetAngularVelocity().X), 1.0f, 0.02f);

	Spun->SetAngularVelocity(FVector(0, 0, 2.5));
	TestTrue(TEXT("angular velocity round trip"),
		Spun->GetAngularVelocity().Equals(FVector(0, 0, 2.5), 0.01));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodySleepEnableTest,
	"Box3DUnreal.Body.SleepWakeEnable", BOX3D_TEST_FLAGS)
bool FBox3DBodySleepEnableTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	UBox3DBodyComponent* Sleeper = SpawnTestBox(Test, FVector(0, 0, 500), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.bStartAwake = false;
		B.GravityScale = 0.0f;
	});
	TestFalse(TEXT("bStartAwake=false spawns asleep"), Sleeper->IsAwake());
	Sleeper->SetAwake(true);
	TestTrue(TEXT("SetAwake wakes the body"), Sleeper->IsAwake());

	// A body resting exactly on the ground goes to sleep on its own.
	UBox3DBodyComponent* Rester = SpawnTestBox(Test, FVector(300, 0, 25.0), 25.0f);
	Test.Step(120); // 2 s; sleep threshold is 0.5 s of rest
	TestFalse(TEXT("resting body falls asleep"), Rester->IsAwake());

	// Disabled bodies neither simulate nor collide.
	UBox3DBodyComponent* Disabled = SpawnTestBox(Test, FVector(600, 0, 500));
	Disabled->SetBodyEnabled(false);
	TestFalse(TEXT("IsBodyEnabled reflects disable"), Disabled->IsBodyEnabled());
	Test.Step(30);
	TestEqual(TEXT("disabled body does not fall"),
		static_cast<float>(Disabled->GetComponentLocation().Z), 500.0f, 0.1f);
	Disabled->SetBodyEnabled(true);
	TestTrue(TEXT("IsBodyEnabled reflects re-enable"), Disabled->IsBodyEnabled());
	Test.Step(30);
	TestTrue(TEXT("re-enabled body resumes falling"), Disabled->GetComponentLocation().Z < 450.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyTransformOwnershipTest,
	"Box3DUnreal.Body.TransformOwnership", BOX3D_TEST_FLAGS)
bool FBox3DBodyTransformOwnershipTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Static bodies follow the component unconditionally.
	UBox3DBodyComponent* Static = Box3DTest::SpawnBody(Test.World, FVector::ZeroVector, [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Static;
		B.ShapeType = EBox3DShapeType::Box;
		B.BoxHalfExtent = FVector(50.0);
	});
	Static->SetWorldLocation(FVector(300, 0, 0));
	const b3Pos StaticPos = b3Body_GetPosition(Static->GetBodyId());
	TestTrue(TEXT("static body follows a plain component move"),
		Box3D::ToUEPos(StaticPos).Equals(FVector(300, 0, 0), 0.1));

	// Dynamic bodies ignore plain moves and honor explicit teleports.
	UBox3DBodyComponent* Dynamic = SpawnTestBox(Test, FVector(0, 300, 100), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.GravityScale = 0.0f;
	});
	Dynamic->SetWorldLocation(FVector(0, 300, 300)); // no teleport flag
	TestTrue(TEXT("plain move does not push a dynamic body"),
		Box3D::ToUEPos(b3Body_GetPosition(Dynamic->GetBodyId())).Equals(FVector(0, 300, 100), 0.1));

	Dynamic->SetWorldLocationAndRotation(FVector(0, 300, 400), FQuat::Identity,
		/*bSweep*/ false, nullptr, ETeleportType::TeleportPhysics);
	TestTrue(TEXT("teleport pushes a dynamic body"),
		Box3D::ToUEPos(b3Body_GetPosition(Dynamic->GetBodyId())).Equals(FVector(0, 300, 400), 0.1));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyKinematicTest,
	"Box3DUnreal.Body.KinematicTargetTracking", BOX3D_TEST_FLAGS)
bool FBox3DBodyKinematicTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UBox3DBodyComponent* Kinematic = Box3DTest::SpawnBody(Test.World, FVector::ZeroVector, [](UBox3DBodyComponent& B)
	{
		B.BodyType = EBox3DBodyType::Kinematic;
		B.ShapeType = EBox3DShapeType::Box;
		B.BoxHalfExtent = FVector(50.0);
	});
	TestEqual(TEXT("kinematic bodies have no mass"), Kinematic->GetMass(), 0.0f, KINDA_SMALL_NUMBER);

	// Each moved component transform becomes a velocity-based target reached by
	// the next fixed step.
	Kinematic->SetWorldLocation(FVector(120, 0, 0));
	Test.Step(1);
	TestTrue(TEXT("kinematic body reaches its target in one step"),
		Box3D::ToUEPos(b3Body_GetPosition(Kinematic->GetBodyId())).Equals(FVector(120, 0, 0), 0.5));
	TestTrue(TEXT("component transform not disturbed by the sync-back"),
		Kinematic->GetComponentLocation().Equals(FVector(120, 0, 0), 0.5));

	Kinematic->SetWorldLocation(FVector(120, 80, 40));
	Test.Step(1);
	TestTrue(TEXT("kinematic body keeps tracking"),
		Box3D::ToUEPos(b3Body_GetPosition(Kinematic->GetBodyId())).Equals(FVector(120, 80, 40), 0.5));

	// Destroying the actor unregisters it; the next step must not crash.
	Kinematic->GetOwner()->Destroy();
	Test.Step(1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyFilterTest,
	"Box3DUnreal.Body.CollisionFilterAndGroup", BOX3D_TEST_FLAGS)
bool FBox3DBodyFilterTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World); // category WorldStatic (bit 0), top at Z=0

	// Mask that excludes WorldStatic: the body must fall straight through.
	UBox3DBodyComponent* Ghost = SpawnTestBox(Test, FVector(0, 0, 200), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.Filter.MaskBits = ~(1 << static_cast<int32>(EBox3DChannel::WorldStatic));
	});
	// Default mask: rests on the ground.
	UBox3DBodyComponent* Solid = SpawnTestBox(Test, FVector(300, 0, 200), 25.0f);

	Test.Step(90); // 1.5 s
	TestTrue(FString::Printf(TEXT("mask excluding ground falls through (Z=%.1f)"),
		Ghost->GetComponentLocation().Z), Ghost->GetComponentLocation().Z < -100.0);
	TestEqual(TEXT("default mask rests on the ground"),
		static_cast<float>(Solid->GetComponentLocation().Z), 25.0f, 2.0f);

	// Same negative group: overlapping bodies are never depenetrated.
	auto SpawnGroupSphere = [&](const FVector& Location, int32 Group)
	{
		return Box3DTest::SpawnBody(Test.World, Location, [Group](UBox3DBodyComponent& B)
		{
			B.BodyType = EBox3DBodyType::Dynamic;
			B.ShapeType = EBox3DShapeType::Sphere;
			B.SphereRadius = 25.0f;
			B.GravityScale = 0.0f;
			B.Filter.GroupIndex = Group;
		});
	};
	UBox3DBodyComponent* GhostA = SpawnGroupSphere(FVector(500, 500, 300), -7);
	UBox3DBodyComponent* GhostB = SpawnGroupSphere(FVector(500, 500, 325), -7);
	UBox3DBodyComponent* PushA = SpawnGroupSphere(FVector(-500, 500, 300), 0);
	UBox3DBodyComponent* PushB = SpawnGroupSphere(FVector(-500, 500, 325), 0);

	// The solver depenetrates through relaxed bias velocity, which moves bodies
	// apart without leaving residual velocity — so assert separation, not speed.
	Test.Step(30);
	const double GhostGap = FVector::Dist(GhostA->GetComponentLocation(), GhostB->GetComponentLocation());
	const double PushGap = FVector::Dist(PushA->GetComponentLocation(), PushB->GetComponentLocation());
	TestTrue(FString::Printf(TEXT("same negative group stays overlapped (gap=%.1f)"), GhostGap),
		FMath::IsNearlyEqual(GhostGap, 25.0, 1.0));
	TestTrue(FString::Printf(TEXT("ungrouped overlapping bodies get pushed apart (gap=%.1f)"), PushGap),
		PushGap > 35.0 && PushGap < 70.0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBodyMaterialTest,
	"Box3DUnreal.Body.MaterialsAndUserMaterialId", BOX3D_TEST_FLAGS)
bool FBox3DBodyMaterialTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UBox3DBodyComponent* Plain = SpawnTestBox(Test, FVector(0, 0, 150), 25.0f, [](UBox3DBodyComponent& B)
	{
		B.GravityScale = 0.0f;
		B.Friction = 0.9f;
		B.Restitution = 0.3f;
		B.UserMaterialId = 42;
	});

	b3ShapeId ShapeId{};
	TestEqual(TEXT("one shape on the body"), b3Body_GetShapeCount(Plain->GetBodyId()), 1);
	b3Body_GetShapes(Plain->GetBodyId(), &ShapeId, 1);
	TestEqual(TEXT("friction reaches the shape"), b3Shape_GetFriction(ShapeId), 0.9f, KINDA_SMALL_NUMBER);
	TestEqual(TEXT("restitution reaches the shape"), b3Shape_GetRestitution(ShapeId), 0.3f, KINDA_SMALL_NUMBER);
	TestTrue(TEXT("user material id reaches the shape"),
		b3Shape_GetSurfaceMaterial(ShapeId).userMaterialId == 42ull);
	TestTrue(TEXT("category bits widen onto the shape"),
		b3Shape_GetFilter(ShapeId).categoryBits == Box3D::ToB3Bits(Plain->Filter.CategoryBits));

	// A UE physical material overrides the loose friction/restitution values.
	UPhysicalMaterial* Material = NewObject<UPhysicalMaterial>(GetTransientPackage());
	Material->Friction = 0.15f;
	Material->Restitution = 0.85f;
	UBox3DBodyComponent* Overridden = SpawnTestBox(Test, FVector(300, 0, 150), 25.0f, [&](UBox3DBodyComponent& B)
	{
		B.GravityScale = 0.0f;
		B.Friction = 0.9f;
		B.Restitution = 0.3f;
		B.PhysicalMaterial = Material;
	});
	b3ShapeId OverriddenShape{};
	b3Body_GetShapes(Overridden->GetBodyId(), &OverriddenShape, 1);
	TestEqual(TEXT("physical material overrides friction"),
		b3Shape_GetFriction(OverriddenShape), 0.15f, KINDA_SMALL_NUMBER);
	TestEqual(TEXT("physical material overrides restitution"),
		b3Shape_GetRestitution(OverriddenShape), 0.85f, KINDA_SMALL_NUMBER);

	// Queries surface the user material id and resolve back to component/actor.
	FBox3DHitResult Hit;
	const bool bHit = UBox3DQueryLibrary::Box3DRayCast(Test.World,
		FVector(0, 0, 400), FVector(0, 0, 0), FBox3DQueryFilter(), Hit);
	TestTrue(TEXT("ray hits the body"), bHit);
	TestEqual(TEXT("hit reports the top face"), static_cast<float>(Hit.Location.Z), 175.0f, 0.5f);
	TestTrue(TEXT("hit carries the user material id"), Hit.UserMaterialId == 42);
	TestTrue(TEXT("hit resolves the component"), Hit.Component == Plain);
	TestTrue(TEXT("hit resolves the actor"), Hit.Actor == Plain->GetOwner());

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
