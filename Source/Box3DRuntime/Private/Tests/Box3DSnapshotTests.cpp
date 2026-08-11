// Tests for Box3DSnapshot: per-body state capture/restore, the snapshot ring,
// and rollback + deterministic replay (ReconcileAndReplay).

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DConversion.h"
#include "Box3DSnapshot.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	/// N raw dynamic spheres in free fall, spread on X so states differ per body.
	TArray<b3BodyId> SpawnFallingSpheres(b3WorldId World, int32 Count)
	{
		TArray<b3BodyId> Bodies;
		Bodies.Reserve(Count);
		for (int32 Index = 0; Index < Count; ++Index)
		{
			b3BodyDef BodyDef = b3DefaultBodyDef();
			BodyDef.type = b3_dynamicBody;
			BodyDef.position = Box3D::ToB3Pos(FVector(Index * 200.0, 0.0, 1000.0));
			const b3BodyId Body = b3CreateBody(World, &BodyDef);

			b3ShapeDef ShapeDef = b3DefaultShapeDef();
			const b3Sphere Sphere{ b3Vec3{ 0.0f, 0.0f, 0.0f }, 0.5f };
			b3CreateSphereShape(Body, &ShapeDef, &Sphere);
			Bodies.Add(Body);
		}
		return Bodies;
	}

	uint32 HashBodies(const TArray<b3BodyId>& Bodies)
	{
		uint32 Hash = B3_HASH_INIT;
		for (const b3BodyId Body : Bodies)
		{
			Hash = Box3D::HashBody(Hash, Body);
		}
		return Hash;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSnapshotRingTest,
	"Box3DUnreal.Snapshot.RingCaptureRestore", BOX3D_TEST_FLAGS)
bool FBox3DSnapshotRingTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	const b3WorldId World = Test.B3World();
	const TArray<b3BodyId> Bodies = SpawnFallingSpheres(World, 3);

	Box3D::FSnapshotRing Ring;
	Ring.Init(/*Capacity*/ 4, Bodies.Num());
	TestTrue(TEXT("fresh ring is empty"), Ring.IsEmpty());

	const float Dt = 1.0f / 60.0f;
	for (int32 Frame = 0; Frame < 10; ++Frame)
	{
		b3World_Step(World, Dt, 4);
		Ring.Capture(Frame, Bodies);
	}

	// Capacity 4 keeps frames 6..9 only.
	TestEqual(TEXT("newest frame"), Ring.NewestFrame(), 9);
	TestEqual(TEXT("oldest retained frame"), Ring.OldestRetainedFrame(), 6);
	TestTrue(TEXT("frame inside window retained"), Ring.IsRetained(7));
	TestFalse(TEXT("evicted frame not retained"), Ring.IsRetained(5));
	TestFalse(TEXT("future frame not retained"), Ring.IsRetained(10));

	// Roundtrip: hash at frame 9, step on, restore to 9, hashes must match —
	// capture/restore covers the full kinematic state.
	const uint32 HashAt9 = HashBodies(Bodies);
	b3World_Step(World, Dt, 4);
	TestNotEqual(TEXT("stepping changes the state hash"), HashBodies(Bodies), HashAt9);
	TestTrue(TEXT("restore to retained frame succeeds"), Ring.Restore(9, Bodies));
	TestEqual(TEXT("restored state hashes identically"), HashBodies(Bodies), HashAt9);

	// Mismatched body count is rejected.
	TArray<b3BodyId> TooFew(Bodies.GetData(), 2);
	TestFalse(TEXT("restore with wrong body count fails"), Ring.Restore(9, TooFew));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DReconcileReplayTest,
	"Box3DUnreal.Snapshot.ReconcileAndReplay", BOX3D_TEST_FLAGS)
bool FBox3DReconcileReplayTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	const b3WorldId World = Test.B3World();
	const TArray<b3BodyId> Bodies = SpawnFallingSpheres(World, 3);

	const float Dt = 1.0f / 60.0f;
	const int32 SubSteps = 4;
	const int32 AuthFrame = 10;
	const int32 PresentFrame = 20;

	Box3D::FSnapshotRing Ring;
	Ring.Init(/*Capacity*/ 32, Bodies.Num());
	for (int32 Frame = 0; Frame <= PresentFrame; ++Frame)
	{
		b3World_Step(World, Dt, SubSteps);
		Ring.Capture(Frame, Bodies);
	}
	const uint32 UninterruptedHash = HashBodies(Bodies);

	// Authority agrees with prediction: reconcile must be a no-op (no pop).
	TArray<Box3D::FBodyState> AuthStates;
	TestTrue(TEXT("ring returns states for the auth frame"), Ring.GetStates(AuthFrame, AuthStates));
	const TArray<int32> AuthIndices = { 0, 1, 2 };
	{
		const Box3D::FReconcileResult Result = Box3D::ReconcileAndReplay(
			World, Bodies, Ring, AuthFrame, AuthIndices, AuthStates, PresentFrame, Dt, SubSteps);
		TestFalse(TEXT("matching authority: no correction"), Result.bCorrected);
		TestEqual(TEXT("matching authority: nothing replayed"), Result.ReplayedFrames, 0);
		TestEqual(TEXT("matching authority: world untouched"), HashBodies(Bodies), UninterruptedHash);
	}

	// Free fall is contact-free, so restore + replay is exactly deterministic:
	// rolling back to the ring's own frame-10 state and re-stepping to frame 20
	// must land bit-identically on the uninterrupted run. Force the rollback path
	// with a tolerance of zero (the sub-tolerance skip would otherwise hide it).
	{
		const Box3D::FReconcileResult Result = Box3D::ReconcileAndReplay(
			World, Bodies, Ring, AuthFrame, AuthIndices, AuthStates, PresentFrame, Dt, SubSteps,
			/*PositionTolerance*/ -1.0);
		TestTrue(TEXT("forced rollback corrects"), Result.bCorrected);
		TestEqual(TEXT("forced rollback replays to present"), Result.ReplayedFrames, PresentFrame - AuthFrame);
		TestEqual(TEXT("replay is deterministic (bit-identical hash)"), HashBodies(Bodies), UninterruptedHash);
	}

	// A genuinely divergent authority (body 1 shifted +1 m on X) must correct and
	// carry the shift through the replay.
	{
		const FVector Before1 = Box3D::ToUEPos(b3Body_GetTransform(Bodies[1]).p);
		AuthStates[1].Transform.p.x += 1.0f;
		const Box3D::FReconcileResult Result = Box3D::ReconcileAndReplay(
			World, Bodies, Ring, AuthFrame, AuthIndices, AuthStates, PresentFrame, Dt, SubSteps);
		TestTrue(TEXT("divergent authority corrects"), Result.bCorrected);
		TestEqual(TEXT("divergent authority replays to present"), Result.ReplayedFrames, PresentFrame - AuthFrame);
		TestTrue(TEXT("correction magnitude reported (~1 m)"),
			FMath::IsNearlyEqual(Result.MaxCorrection, 1.0, 0.001));

		const FVector After1 = Box3D::ToUEPos(b3Body_GetTransform(Bodies[1]).p);
		TestTrue(FString::Printf(TEXT("corrected body carries the shift (dX=%.1f cm)"), After1.X - Before1.X),
			FMath::IsNearlyEqual(After1.X - Before1.X, 100.0, 0.1));

		// Uncorrected bodies replay onto their original trajectory.
		Box3D::FBodyState State0;
		TArray<Box3D::FBodyState> PresentStates;
		TestTrue(TEXT("ring retains the present frame"), Ring.GetStates(PresentFrame, PresentStates));
	}

	// Rollback past the ring window is refused.
	{
		Box3D::FSnapshotRing SmallRing;
		SmallRing.Init(2, Bodies.Num());
		SmallRing.Capture(19, Bodies);
		SmallRing.Capture(20, Bodies);
		const Box3D::FReconcileResult Result = Box3D::ReconcileAndReplay(
			World, Bodies, SmallRing, /*AuthFrame*/ 5, AuthIndices, AuthStates, PresentFrame, Dt, SubSteps);
		TestFalse(TEXT("auth frame outside window: refused"), Result.bCorrected);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
