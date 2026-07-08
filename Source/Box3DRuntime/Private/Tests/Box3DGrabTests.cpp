// Tests for UBox3DGrabComponent: mass-gated pickup, velocity-tracking carry,
// throw as impulse/mass, pawn-filter juggling, and the break-distance bail.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DGrabComponent.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	/// 50 cm cube: volume 0.125 m^3, so density 80 -> 10 kg, density 2000 -> 250 kg.
	UBox3DBodyComponent* SpawnCrate(UWorld* World, const FVector& Location, float Density)
	{
		return Box3DTest::SpawnBody(World, Location, [Density](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Dynamic;
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(25.0);
			Body.Density = Density;
		});
	}

	UBox3DGrabComponent* SpawnGrabber(UWorld* World)
	{
		AActor* Actor = World->SpawnActor<AActor>();
		UBox3DGrabComponent* Grab = NewObject<UBox3DGrabComponent>(Actor, TEXT("Grab"));
		Grab->RegisterComponent();
		return Grab;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DGrabHoldTest,
	"Box3DUnreal.Grab.MassLimitAndCarry", BOX3D_TEST_FLAGS)
bool FBox3DGrabHoldTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	UBox3DBodyComponent* Light = SpawnCrate(Test.World, FVector(0, 0, 25), 80.0f);
	UBox3DBodyComponent* Heavy = SpawnCrate(Test.World, FVector(200, 0, 25), 2000.0f);
	UBox3DGrabComponent* Grab = SpawnGrabber(Test.World);

	TestFalse(TEXT("250 kg crate refuses a 60 kg grip"), Grab->GrabBody(Heavy));
	TestTrue(TEXT("10 kg crate grabbed"), Grab->GrabBody(Light));
	TestFalse(TEXT("one object per grip"), Grab->GrabBody(Heavy));
	TestEqual(TEXT("held mass reported"), Grab->GetHeldMassKg(), 10.0f, 0.5f);

	// Carry it up and sideways against gravity.
	const FVector Target(150, 100, 200);
	Grab->SetHoldTarget(Target);
	Test.Step(90);
	const float CarryError = FVector::Dist(Light->GetComponentLocation(), Target);
	TestTrue(FString::Printf(TEXT("carried to the hold target (err=%f cm)"), CarryError),
		CarryError < 30.0f);

	// Release: the grip force is gone, gravity brings it back down.
	Grab->Release();
	TestFalse(TEXT("released"), Grab->IsHolding());
	Test.Step(90);
	TestTrue(FString::Printf(TEXT("falls after release (z=%f)"), Light->GetComponentLocation().Z),
		Light->GetComponentLocation().Z < 100.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DGrabThrowTest,
	"Box3DUnreal.Grab.ThrowAndFilterRestore", BOX3D_TEST_FLAGS)
bool FBox3DGrabThrowTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	UBox3DBodyComponent* Crate = SpawnCrate(Test.World, FVector(0, 0, 25), 80.0f); // 10 kg
	const int32 SavedMask = Crate->Filter.MaskBits;
	UBox3DGrabComponent* Grab = SpawnGrabber(Test.World);
	Grab->ThrowImpulseKgCmS = 6000.0f;

	TestTrue(TEXT("grabbed"), Grab->GrabBody(Crate));

	// Held bodies must ignore pawns (no wedging against the holder's proxy).
	{
		b3ShapeId Shapes[4];
		b3Body_GetShapes(Crate->GetBodyId(), Shapes, 4);
		const b3Filter Held = b3Shape_GetFilter(Shapes[0]);
		TestEqual(TEXT("pawn bit stripped while held"),
			Held.maskBits & (1ull << static_cast<int32>(EBox3DChannel::Pawn)), 0ull);
	}

	TestTrue(TEXT("thrown"), Grab->Throw(FVector(1, 0, 0)));
	TestFalse(TEXT("grip empty after throw"), Grab->IsHolding());

	// v = J/m: 6000 kg*cm/s over 10 kg = 600 cm/s.
	TestEqual(TEXT("throw speed = impulse/mass"), float(Crate->GetLinearVelocity().X), 600.0f, 20.0f);

	b3ShapeId Shapes[4];
	b3Body_GetShapes(Crate->GetBodyId(), Shapes, 4);
	TestEqual(TEXT("original filter restored on release"),
		b3Shape_GetFilter(Shapes[0]).maskBits, Box3D::ToB3Bits(SavedMask));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DGrabBreakTest,
	"Box3DUnreal.Grab.BreakDistance", BOX3D_TEST_FLAGS)
bool FBox3DGrabBreakTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	UBox3DBodyComponent* Crate = SpawnCrate(Test.World, FVector(0, 0, 25), 80.0f);
	UBox3DGrabComponent* Grab = SpawnGrabber(Test.World);
	Grab->BreakDistance = 500.0f;

	TestTrue(TEXT("grabbed"), Grab->GrabBody(Crate));
	// Yank the target far beyond the break distance: the grip must let go
	// instead of slingshotting the crate across the map.
	Grab->SetHoldTarget(FVector(5000, 0, 25));
	Test.Step(2);
	TestFalse(TEXT("grip broke at distance"), Grab->IsHolding());
	TestTrue(FString::Printf(TEXT("crate not slingshot (vx=%f)"), Crate->GetLinearVelocity().X),
		Crate->GetLinearVelocity().X < 1500.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
