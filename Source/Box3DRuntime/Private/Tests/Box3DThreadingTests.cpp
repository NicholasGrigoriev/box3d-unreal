// M5 threading: the UE::Tasks hookup and box3d's internal scheduler must fan work
// out to worker threads while producing the same simulation as a serial step.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DTaskSystem.h"
#include "box3d/collision.h"

namespace
{
	/// Deterministic contact-heavy pile built with the raw b3 API (no components:
	/// this test is about stepping, and 180 actors would slow the suite for
	/// nothing). Returns the dynamic body ids in creation order.
	TArray<b3BodyId> SpawnRawPile(b3WorldId WorldId)
	{
		{
			b3BodyDef GroundDef = b3DefaultBodyDef();
			GroundDef.type = b3_staticBody;
			GroundDef.position = b3Pos{ 0.0f, 0.0f, -0.25f };
			const b3BodyId GroundId = b3CreateBody(WorldId, &GroundDef);
			const b3BoxHull GroundHull = b3MakeBoxHull(20.0f, 20.0f, 0.25f);
			b3ShapeDef ShapeDef = b3DefaultShapeDef();
			b3CreateHullShape(GroundId, &ShapeDef, &GroundHull.base);
		}

		TArray<b3BodyId> Bodies;
		const b3BoxHull Hull = b3MakeBoxHull(0.25f, 0.25f, 0.25f);
		constexpr int32 Columns = 6;
		constexpr int32 Layers = 5;
		constexpr float Spacing = 0.6f;
		constexpr float GridOffset = 0.5f * (Columns - 1) * Spacing;
		for (int32 Index = 0; Index < Columns * Columns * Layers; ++Index)
		{
			const int32 Col = Index % Columns;
			const int32 Row = (Index / Columns) % Columns;
			const int32 Layer = Index / (Columns * Columns);

			b3BodyDef BodyDef = b3DefaultBodyDef();
			BodyDef.type = b3_dynamicBody;
			BodyDef.position = b3Pos{ Col * Spacing - GridOffset, Row * Spacing - GridOffset, 0.3f + Layer * 0.55f };
			Bodies.Add(b3CreateBody(WorldId, &BodyDef));
			b3ShapeDef ShapeDef = b3DefaultShapeDef();
			b3CreateHullShape(Bodies.Last(), &ShapeDef, &Hull.base);
		}
		return Bodies;
	}

	/// Step the pile for a fixed number of frames under the given scheduler
	/// settings and return the final body positions (box3d meters).
	TArray<FVector> RunPile(int32 Workers, EBox3DTaskSystem System, int64* OutTasksEnqueued = nullptr)
	{
		Box3DTest::FScopedTaskSettings Settings(Workers, System);
		Box3DTest::FTestWorld Test;

		const TArray<b3BodyId> Bodies = SpawnRawPile(Test.B3World());
		Test.Step(60);

		if (OutTasksEnqueued != nullptr)
		{
			const FBox3DUETaskPool* Pool = Test.Subsystem().GetTaskPool();
			*OutTasksEnqueued = Pool != nullptr ? Pool->GetTotalEnqueued() : -1;
		}

		TArray<FVector> Positions;
		Positions.Reserve(Bodies.Num());
		for (const b3BodyId& Body : Bodies)
		{
			const b3WorldTransform Transform = b3Body_GetTransform(Body);
			Positions.Add(FVector(Transform.p.x, Transform.p.y, Transform.p.z));
		}
		return Positions;
	}

	double MaxPositionDelta(const TArray<FVector>& A, const TArray<FVector>& B)
	{
		double MaxDelta = 0.0;
		for (int32 Index = 0; Index < A.Num(); ++Index)
		{
			MaxDelta = FMath::Max(MaxDelta, (A[Index] - B[Index]).GetAbsMax());
		}
		return MaxDelta;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DThreadingSchedulerEquivalenceTest,
	"Box3DUnreal.Threading.SchedulerEquivalence", BOX3D_TEST_FLAGS)
bool FBox3DThreadingSchedulerEquivalenceTest::RunTest(const FString& Parameters)
{
	// Baseline: serial stepping.
	int64 SerialTasks = 0;
	const TArray<FVector> SerialPositions = RunPile(1, EBox3DTaskSystem::UnrealTasks, &SerialTasks);
	TestEqual(TEXT("180 dynamic bodies simulated"), SerialPositions.Num(), 180);
	TestTrue(TEXT("no task pool exists when WorkerCount is 1"), SerialTasks == -1);

	// Sanity: the pile actually fell and settled onto the slab (top of the ground
	// is z=0, so every box center must sit in [0.2, 3] m and not tunnel through).
	for (const FVector& Position : SerialPositions)
	{
		if (Position.Z < 0.2 || Position.Z > 3.0)
		{
			AddError(FString::Printf(TEXT("serial pile body ended at unexpected z=%f m"), Position.Z));
			break;
		}
	}

	// UE::Tasks scheduler: work must actually reach UE workers, and the result
	// must match the serial run (box3d's solve is deterministic across worker
	// counts; divergence here means the task glue corrupted the step).
	int64 UETasks = 0;
	const TArray<FVector> UETasksPositions = RunPile(4, EBox3DTaskSystem::UnrealTasks, &UETasks);
	TestTrue(TEXT("UE task pool enqueued solver tasks"), UETasks > 0);
	const double UEDelta = MaxPositionDelta(SerialPositions, UETasksPositions);
	TestTrue(FString::Printf(TEXT("UE-tasks run matches serial (max delta %e m)"), UEDelta), UEDelta < 1.0e-3);

	// box3d's internal scheduler: same expectation, no UE pool involved.
	int64 InternalTasks = 0;
	const TArray<FVector> InternalPositions = RunPile(4, EBox3DTaskSystem::Box3DInternal, &InternalTasks);
	TestTrue(TEXT("no UE task pool with the internal scheduler"), InternalTasks == -1);
	const double InternalDelta = MaxPositionDelta(SerialPositions, InternalPositions);
	TestTrue(FString::Printf(TEXT("internal-scheduler run matches serial (max delta %e m)"), InternalDelta),
		InternalDelta < 1.0e-3);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
