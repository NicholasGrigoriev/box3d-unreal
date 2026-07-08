// Perf diagnostic (no assertions): measures wall time per fixed step (physics)
// and per visual Tick (ISM update) for a default liquid tap pouring onto the
// ground, then compares SubStepCount 8 vs 4 at the full pool. Numbers land in
// the automation log — the liquid budget guidance in the actor docs came from
// here.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DLiquidSourceActor.h"
#include "Box3DSettings.h"
#include "HAL/PlatformTime.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	/// SubStepCount is read from the settings CDO every subsystem Tick, so a
	/// scope around a batch of steps is enough.
	struct FScopedSubSteps
	{
		int32 Saved;
		explicit FScopedSubSteps(int32 SubSteps)
			: Saved(GetMutableDefault<UBox3DSettings>()->SubStepCount)
		{
			GetMutableDefault<UBox3DSettings>()->SubStepCount = SubSteps;
		}
		~FScopedSubSteps()
		{
			GetMutableDefault<UBox3DSettings>()->SubStepCount = Saved;
		}
	};

	double MeasureSteps(const Box3DTest::FTestWorld& Test, int32 Steps)
	{
		const double T0 = FPlatformTime::Seconds();
		Test.Step(Steps);
		return 1000.0 * (FPlatformTime::Seconds() - T0) / Steps;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DLiquidPerfProbe,
	"Box3DUnreal.Liquid.PerfProbe", BOX3D_TEST_FLAGS)
bool FBox3DLiquidPerfProbe::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	const FTransform Transform(FRotator(-90.0, 0.0, 0.0), FVector(0, 0, 250));
	ABox3DLiquidSourceActor* Liquid = Test.World->SpawnActorDeferred<ABox3DLiquidSourceActor>(
		ABox3DLiquidSourceActor::StaticClass(), Transform);
	// Stock defaults — the reported setup.
	Liquid->FinishSpawning(Transform);

	const float Dt = Box3DTest::FTestWorld::FixedDt();
	const int32 StepsPerSecond = FMath::RoundToInt(1.0f / Dt);

	for (int32 Second = 1; Second <= 10; ++Second)
	{
		double PhysicsSeconds = 0.0;
		double TickSeconds = 0.0;
		for (int32 Step = 0; Step < StepsPerSecond; ++Step)
		{
			const double T0 = FPlatformTime::Seconds();
			Test.Step(1);
			const double T1 = FPlatformTime::Seconds();
			Liquid->Tick(Dt);
			const double T2 = FPlatformTime::Seconds();
			PhysicsSeconds += T1 - T0;
			TickSeconds += T2 - T1;
		}

		const FBox3DWorldStats Stats = Test.Subsystem().GetWorldStats();
		AddInfo(FString::Printf(
			TEXT("t=%ds particles=%d awake=%d contacts=%d | physics avg %.3f ms/step (solve %.3f, collide %.3f, pairs %.3f) | visual tick avg %.3f ms"),
			Second, Liquid->GetParticleCount(), Stats.AwakeBodyCount, Stats.ContactCount,
			1000.0 * PhysicsSeconds / StepsPerSecond,
			Stats.SolveMs, Stats.CollideMs, Stats.PairsMs,
			1000.0 * TickSeconds / StepsPerSecond));
	}

	// Sub-step cost at the full pool: solver work scales with SubStepCount,
	// and the project config may run higher than the plugin default of 4.
	{
		FScopedSubSteps Eight(8);
		AddInfo(FString::Printf(TEXT("full pool, SubStepCount=8: %.3f ms/step"), MeasureSteps(Test, 120)));
	}
	{
		FScopedSubSteps Four(4);
		AddInfo(FString::Printf(TEXT("full pool, SubStepCount=4: %.3f ms/step"), MeasureSteps(Test, 120)));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
