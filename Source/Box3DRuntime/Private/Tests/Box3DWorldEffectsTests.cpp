// M6 world effects: radial explosions (b3World_Explode) and continuous
// collision for fast movers (world continuous + per-body bullet flag).

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DQueryLibrary.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DExplosionImpulseTest,
	"Box3DUnreal.World.ExplosionImpulse", BOX3D_TEST_FLAGS)
bool FBox3DExplosionImpulseTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Floating spheres (gravity off) so velocities are purely explosion-driven.
	// For a sphere: dv = ImpulsePerArea * (pi r^2) / (4/3 pi r^3 rho) = 3 I / (4 r rho),
	// so r=25cm, rho=1000, I=10 kg*cm/s per cm^2 gives exactly 300 cm/s at full scale.
	auto SpawnFloatingSphere = [&Test](const FVector& Location, int32 CategoryBits = 0)
	{
		return Box3DTest::SpawnBody(Test.World, Location, [CategoryBits](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Dynamic;
			Body.ShapeType = EBox3DShapeType::Sphere;
			Body.SphereRadius = 25.0f;
			Body.GravityScale = 0.0f;
			if (CategoryBits != 0)
			{
				Body.Filter.CategoryBits = CategoryBits;
			}
		});
	};

	UBox3DBodyComponent* Inside = SpawnFloatingSphere(FVector(100, 0, 100));   // surface 75cm from blast
	UBox3DBodyComponent* Fringe = SpawnFloatingSphere(FVector(-175, 0, 100));  // surface 150cm: half falloff
	UBox3DBodyComponent* Beyond = SpawnFloatingSphere(FVector(0, 400, 100),
		1 << static_cast<int32>(EBox3DChannel::Custom7));                      // out of range entirely

	// Radius 100 + falloff 100, impulse 10 kg*cm/s per cm^2, default filter.
	UBox3DQueryLibrary::Box3DExplode(Test.World, FVector(0, 0, 100), 100.0f, 100.0f, 10.0f, FBox3DQueryFilter());

	const FVector InsideVelocity = Inside->GetLinearVelocity();
	TestTrue(FString::Printf(TEXT("full-scale sphere blasted +X at 300 cm/s (got %s)"), *InsideVelocity.ToCompactString()),
		FMath::IsNearlyEqual(InsideVelocity.X, 300.0, 3.0) && InsideVelocity.Y == 0.0 && InsideVelocity.Z == 0.0);
	TestTrue(TEXT("radial impulse through the center imparts no spin"),
		Inside->GetAngularVelocity().IsNearlyZero(0.01));

	const FVector FringeVelocity = Fringe->GetLinearVelocity();
	TestTrue(FString::Printf(TEXT("half-falloff sphere blasted -X at 150 cm/s (got %s)"), *FringeVelocity.ToCompactString()),
		FMath::IsNearlyEqual(FringeVelocity.X, -150.0, 2.0));

	TestTrue(TEXT("sphere beyond radius+falloff untouched"), Beyond->GetLinearVelocity().IsNearlyZero());

	// Mask filtering: a blast centered on the Custom7 sphere that only targets
	// Custom6 must not move it; the same blast with the default mask must.
	FBox3DQueryFilter Custom6Only;
	Custom6Only.MaskBits = 1 << static_cast<int32>(EBox3DChannel::Custom6);
	UBox3DQueryLibrary::Box3DExplode(Test.World, FVector(0, 400, 100), 100.0f, 0.0f, 10.0f, Custom6Only);
	TestTrue(TEXT("masked-out sphere ignores the blast"), Beyond->GetLinearVelocity().IsNearlyZero());

	UBox3DQueryLibrary::Box3DExplode(Test.World, FVector(0, 400, 100), 100.0f, 0.0f, 10.0f, FBox3DQueryFilter());
	TestTrue(FString::Printf(TEXT("default mask hits it (got %s)"), *Beyond->GetLinearVelocity().ToCompactString()),
		FMath::IsNearlyEqual(Beyond->GetLinearVelocity().Size(), 300.0, 3.0));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DContinuousFastBodyTest,
	"Box3DUnreal.World.ContinuousFastBody", BOX3D_TEST_FLAGS)
bool FBox3DContinuousFastBodyTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// A 150 m/s body crosses 250 cm per fixed step — vastly more than a 4 cm
	// plate plus its own diameter, so only continuous collision can stop it.

	// Case 1: fast NON-bullet sphere vs a thin STATIC plate. World-level
	// continuous collision (enabled by default) must catch it.
	Box3DTest::SpawnBody(Test.World, FVector(0, 0, 0), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Static;
		Body.ShapeType = EBox3DShapeType::Box;
		Body.BoxHalfExtent = FVector(200, 200, 2);
	});
	UBox3DBodyComponent* FastSphere = Box3DTest::SpawnBody(Test.World, FVector(0, 0, 150), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Dynamic;
		Body.ShapeType = EBox3DShapeType::Sphere;
		Body.SphereRadius = 10.0f;
	});
	FastSphere->SetLinearVelocity(FVector(0, 0, -15000));

	// Case 2: fast BULLET sphere vs a thin floating DYNAMIC plate — the bullet
	// flag extends continuous collision to dynamic-vs-dynamic.
	UBox3DBodyComponent* Plate = Box3DTest::SpawnBody(Test.World, FVector(500, 0, 100), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Dynamic;
		Body.ShapeType = EBox3DShapeType::Box;
		Body.BoxHalfExtent = FVector(100, 100, 2);
		Body.GravityScale = 0.0f;
	});
	UBox3DBodyComponent* Bullet = Box3DTest::SpawnBody(Test.World, FVector(500, 0, 250), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Dynamic;
		Body.ShapeType = EBox3DShapeType::Sphere;
		Body.SphereRadius = 10.0f;
		Body.bIsBullet = true;
		Body.GravityScale = 0.0f;
	});
	Bullet->SetLinearVelocity(FVector(0, 0, -15000));

	TestFalse(TEXT("plain sphere is not a bullet"), b3Body_IsBullet(FastSphere->GetBodyId()));
	TestTrue(TEXT("bIsBullet reached box3d"), b3Body_IsBullet(Bullet->GetBodyId()));

	Test.Step(30);

	const double FastZ = FastSphere->GetComponentLocation().Z;
	TestTrue(FString::Printf(TEXT("fast sphere rests on the static plate (z=%f)"), FastZ),
		FastZ > 6.0 && FastZ < 20.0);

	// The bullet must never pass the plate, and the plate must have absorbed its
	// momentum (proof the hit was resolved rather than tunneled through).
	const double BulletZ = Bullet->GetComponentLocation().Z;
	const double PlateZ = Plate->GetComponentLocation().Z;
	TestTrue(FString::Printf(TEXT("bullet stayed above the dynamic plate (bullet z=%f, plate z=%f)"), BulletZ, PlateZ),
		BulletZ > PlateZ);
	TestTrue(FString::Printf(TEXT("plate received the impact momentum (vz=%f)"), Plate->GetLinearVelocity().Z),
		Plate->GetLinearVelocity().Z < -10.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
