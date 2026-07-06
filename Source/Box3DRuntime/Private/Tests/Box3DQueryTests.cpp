// Tests for UBox3DQueryLibrary (M2): ray casts (closest + multi), shape casts,
// overlaps, query filtering, and the character mover helpers — all against a
// hand-placed static scene so every expected value is analytic.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DQueryLibrary.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	/// Ground (top Z=0, WorldStatic) + two 1 m static boxes on the ray X axis:
	/// Box1 spans X 150..250 (category Custom6), Box2 spans X 350..450 (Custom7),
	/// both spanning Z 0..100.
	struct FQueryScene
	{
		Box3DTest::FTestWorld Test;
		UBox3DBodyComponent* Ground = nullptr;
		UBox3DBodyComponent* Box1 = nullptr;
		UBox3DBodyComponent* Box2 = nullptr;

		FQueryScene()
		{
			Ground = Box3DTest::SpawnGround(Test.World);
			auto SpawnStaticBox = [&](double X, EBox3DChannel Channel)
			{
				return Box3DTest::SpawnBody(Test.World, FVector(X, 0, 50), [&](UBox3DBodyComponent& Body)
				{
					Body.BodyType = EBox3DBodyType::Static;
					Body.ShapeType = EBox3DShapeType::Box;
					Body.BoxHalfExtent = FVector(50.0);
					Body.Filter.CategoryBits = 1 << static_cast<int32>(Channel);
				});
			};
			Box1 = SpawnStaticBox(200.0, EBox3DChannel::Custom6);
			Box2 = SpawnStaticBox(400.0, EBox3DChannel::Custom7);
		}
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DQueryRayTest,
	"Box3DUnreal.Query.RayClosestAndMulti", BOX3D_TEST_FLAGS)
bool FBox3DQueryRayTest::RunTest(const FString& Parameters)
{
	FQueryScene Scene;
	const FVector Start(0, 0, 50);
	const FVector End(600, 0, 50);
	const FBox3DQueryFilter Everything;

	FBox3DHitResult Hit;
	TestTrue(TEXT("closest ray hits"), UBox3DQueryLibrary::Box3DRayCast(Scene.Test.World, Start, End, Everything, Hit));
	TestEqual(TEXT("closest hit is Box1's near face"), static_cast<float>(Hit.Location.X), 150.0f, 0.5f);
	TestEqual(TEXT("closest fraction"), Hit.Fraction, 0.25f, 0.002f);
	TestTrue(TEXT("closest normal faces the ray"), Hit.Normal.Equals(FVector(-1, 0, 0), 0.01));
	TestTrue(TEXT("closest hit resolves Box1"), Hit.Component == Scene.Box1);
	TestEqual(TEXT("hull hits have no triangle index"), Hit.TriangleIndex, -1);

	const TArray<FBox3DHitResult> Hits = UBox3DQueryLibrary::Box3DRayCastMulti(Scene.Test.World, Start, End, Everything);
	TestEqual(TEXT("multi ray reports both boxes"), Hits.Num(), 2);
	if (Hits.Num() == 2)
	{
		TestTrue(TEXT("multi hits sorted near to far"), Hits[0].Fraction < Hits[1].Fraction);
		TestEqual(TEXT("first entry at Box1"), static_cast<float>(Hits[0].Location.X), 150.0f, 0.5f);
		TestEqual(TEXT("second entry at Box2"), static_cast<float>(Hits[1].Location.X), 350.0f, 0.5f);
		TestTrue(TEXT("components resolved in order"), Hits[0].Component == Scene.Box1 && Hits[1].Component == Scene.Box2);
	}

	// A miss above the whole scene returns false and leaves the hit empty.
	FBox3DHitResult Miss;
	TestFalse(TEXT("ray above the scene misses"), UBox3DQueryLibrary::Box3DRayCast(Scene.Test.World,
		FVector(0, 0, 500), FVector(600, 0, 500), Everything, Miss));
	TestFalse(TEXT("miss leaves bHit false"), Miss.bHit);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DQueryShapeCastTest,
	"Box3DUnreal.Query.SphereAndCapsuleCast", BOX3D_TEST_FLAGS)
bool FBox3DQueryShapeCastTest::RunTest(const FString& Parameters)
{
	FQueryScene Scene;
	const FBox3DQueryFilter Everything;

	// Sphere r=25 dropped over Box1 (top Z=100): center stops at 125.
	FBox3DHitResult SphereHit;
	TestTrue(TEXT("sphere cast hits"), UBox3DQueryLibrary::Box3DSphereCast(Scene.Test.World,
		FVector(200, 0, 300), FVector(200, 0, 0), 25.0f, Everything, SphereHit));
	TestEqual(TEXT("sphere cast fraction stops the center at top+radius"),
		SphereHit.Fraction, (300.0f - 125.0f) / 300.0f, 0.005f);
	TestEqual(TEXT("sphere contact point on the top face"), static_cast<float>(SphereHit.Location.Z), 100.0f, 1.0f);
	TestTrue(TEXT("sphere contact normal points up"), SphereHit.Normal.Z > 0.99);

	// Capsule r=25, half height 75 (UE tip convention): bottom tip at center-75.
	FBox3DHitResult CapsuleHit;
	TestTrue(TEXT("capsule cast hits"), UBox3DQueryLibrary::Box3DCapsuleCast(Scene.Test.World,
		FVector(200, 0, 300), FVector(200, 0, 0), 25.0f, 75.0f, Everything, CapsuleHit));
	TestEqual(TEXT("capsule cast fraction stops the center at top+halfheight"),
		CapsuleHit.Fraction, (300.0f - 175.0f) / 300.0f, 0.005f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DQueryFilterOverlapTest,
	"Box3DUnreal.Query.OverlapAndFilter", BOX3D_TEST_FLAGS)
bool FBox3DQueryFilterOverlapTest::RunTest(const FString& Parameters)
{
	FQueryScene Scene;

	// Unfiltered overlap around Box2 dips into the ground: both are found.
	const TArray<UBox3DBodyComponent*> All = UBox3DQueryLibrary::Box3DOverlapSphere(Scene.Test.World,
		FVector(400, 0, 50), 80.0f, FBox3DQueryFilter());
	TestTrue(TEXT("unfiltered overlap finds Box2 and the ground"),
		All.Contains(Scene.Box2) && All.Contains(Scene.Ground));
	TestFalse(TEXT("overlap does not reach Box1"), All.Contains(Scene.Box1));

	// Masked to Custom7 only: the ground and Box1 disappear.
	FBox3DQueryFilter Custom7Only;
	Custom7Only.MaskBits = 1 << static_cast<int32>(EBox3DChannel::Custom7);
	const TArray<UBox3DBodyComponent*> Filtered = UBox3DQueryLibrary::Box3DOverlapSphere(Scene.Test.World,
		FVector(400, 0, 50), 80.0f, Custom7Only);
	TestEqual(TEXT("masked overlap finds exactly Box2"), Filtered.Num(), 1);
	TestTrue(TEXT("masked overlap resolves Box2"), Filtered.Num() == 1 && Filtered[0] == Scene.Box2);

	// The same mask makes a ray pass through Box1 and hit Box2 behind it.
	FBox3DHitResult Hit;
	TestTrue(TEXT("masked ray hits"), UBox3DQueryLibrary::Box3DRayCast(Scene.Test.World,
		FVector(0, 0, 50), FVector(600, 0, 50), Custom7Only, Hit));
	TestTrue(TEXT("masked ray skips Box1 for Box2"), Hit.Component == Scene.Box2);
	TestEqual(TEXT("masked ray enters at Box2's near face"), static_cast<float>(Hit.Location.X), 350.0f, 0.5f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DQueryMoverTest,
	"Box3DUnreal.Query.MoverCastAndSolve", BOX3D_TEST_FLAGS)
bool FBox3DQueryMoverTest::RunTest(const FString& Parameters)
{
	FQueryScene Scene;
	const FBox3DQueryFilter Everything;
	const float Radius = 30.0f;
	const float HalfHeight = 90.0f;

	// Capsule mover dropped on open ground (top Z=0): the bottom tip is at
	// center-90, so a 200 cm descent from Z=200 is safe for 110 cm.
	const float Fraction = UBox3DQueryLibrary::Box3DCastMover(Scene.Test.World,
		FVector(-300, 0, 200), FVector(0, 0, -200), Radius, HalfHeight, Everything);
	TestEqual(TEXT("mover cast stops at the ground"), Fraction, 110.0f / 200.0f, 0.01f);

	// Free air: full translation allowed.
	const float FreeFraction = UBox3DQueryLibrary::Box3DCastMover(Scene.Test.World,
		FVector(-300, 0, 500), FVector(0, 0, -200), Radius, HalfHeight, Everything);
	TestEqual(TEXT("unobstructed mover cast returns 1"), FreeFraction, 1.0f, KINDA_SMALL_NUMBER);

	// Slightly interpenetrating the ground, pushing further down: the plane
	// solver must refuse the descent (and typically pops the capsule out).
	int32 PlaneCount = 0;
	const FVector Blocked = UBox3DQueryLibrary::Box3DSolveMoverDelta(Scene.Test.World,
		FVector(-300, 0, 85), Radius, HalfHeight, FVector(0, 0, -50), Everything, PlaneCount);
	TestTrue(TEXT("solver found the ground plane"), PlaneCount >= 1);
	TestTrue(FString::Printf(TEXT("descent into the ground is rejected (Z=%.1f)"), Blocked.Z),
		Blocked.Z > -1.0 && Blocked.Z < 25.0);

	// Diagonal move against the ground: vertical part clipped, lateral preserved
	// (collide-and-slide, not a hard stop).
	int32 SlidePlanes = 0;
	const FVector Slid = UBox3DQueryLibrary::Box3DSolveMoverDelta(Scene.Test.World,
		FVector(-300, 0, 88), Radius, HalfHeight, FVector(100, 0, -50), Everything, SlidePlanes);
	TestTrue(TEXT("slide solver found the ground plane"), SlidePlanes >= 1);
	TestTrue(FString::Printf(TEXT("lateral motion preserved while sliding (X=%.1f)"), Slid.X), Slid.X > 80.0);
	TestTrue(FString::Printf(TEXT("vertical motion clipped while sliding (Z=%.1f)"), Slid.Z), Slid.Z > -5.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
