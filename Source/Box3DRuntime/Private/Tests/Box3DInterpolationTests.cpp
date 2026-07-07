// Render interpolation (bInterpolateBodyTransforms): components sit at
// Lerp(prev step, latest step, Accumulator/FixedDt), one fixed step behind the
// simulation, and settle exactly on the final pose when a body stops moving.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DInterpolationMidpointTest,
	"Box3DUnreal.Interpolation.Midpoint", BOX3D_TEST_FLAGS)
bool FBox3DInterpolationMidpointTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedInterpolationSettings InterpSettings;
	Box3DTest::FTestWorld Test;
	const float FixedDt = Box3DTest::FTestWorld::FixedDt();

	// Gravity-free glider at a constant 100 cm/s along X: every step advances
	// exactly 100 * dt, making the interpolated positions closed-form.
	UBox3DBodyComponent* Body = Box3DTest::SpawnBody(Test.World, FVector::ZeroVector,
		[](UBox3DBodyComponent& InBody)
		{
			InBody.BodyType = EBox3DBodyType::Dynamic;
			InBody.ShapeType = EBox3DShapeType::Sphere;
			InBody.SphereRadius = 25.0f;
			InBody.GravityScale = 0.0f;
			InBody.bEnableSleep = false;
		});
	Body->SetLinearVelocity(FVector(100, 0, 0));

	const double StepDistance = 100.0 * FixedDt;

	// One full step: the segment spans spawn -> step-1 pose, and with an empty
	// accumulator the component renders at its start (one step of latency).
	Test.Subsystem().Tick(FixedDt);
	TestEqual(TEXT("alpha=0 renders the segment start"),
		Body->GetComponentLocation().X, 0.0, 0.05);

	// Half a step of frame time, no new physics step: alpha=0.5, midpoint.
	Test.Subsystem().Tick(FixedDt * 0.5f);
	TestEqual(TEXT("alpha=0.5 renders the segment midpoint"),
		Body->GetComponentLocation().X, StepDistance * 0.5, 0.05);

	// The other half: step 2 fires, a fresh segment begins at the step-1 pose.
	Test.Subsystem().Tick(FixedDt * 0.5f);
	TestEqual(TEXT("new segment starts at the previous step's pose"),
		Body->GetComponentLocation().X, StepDistance, 0.05);

	// Freeze the body: the next step produces no move event for a stopped body
	// eventually, and the component must land exactly on the final physics pose.
	Body->SetLinearVelocity(FVector::ZeroVector);
	Test.Step(5);
	const double FinalX = Body->GetComponentLocation().X;
	Test.Step(2);
	TestEqual(TEXT("settled component stops advancing"), Body->GetComponentLocation().X, FinalX, 0.05);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
