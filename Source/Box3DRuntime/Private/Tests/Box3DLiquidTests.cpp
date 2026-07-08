// Tests for the liquid source actor: emission rate and the particle budget,
// lifetime despawn, pouring/spreading on the ground, and the cohesion pass.
// All simulation runs on the fixed step, so FTestWorld (which never ticks
// actors) exercises everything except the purely-visual instanced mesh.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DLiquidSourceActor.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	ABox3DLiquidSourceActor* SpawnLiquidSource(UWorld* World, const FTransform& Transform,
		TFunctionRef<void(ABox3DLiquidSourceActor&)> Setup)
	{
		ABox3DLiquidSourceActor* Liquid = World->SpawnActorDeferred<ABox3DLiquidSourceActor>(
			ABox3DLiquidSourceActor::StaticClass(), Transform);
		Setup(*Liquid);
		Liquid->FinishSpawning(Transform);
		return Liquid;
	}

	/// Pointing straight down: forward (+X) rotated onto -Z.
	FTransform PouringDownAt(const FVector& Location)
	{
		return FTransform(FRotator(-90.0, 0.0, 0.0), Location);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DLiquidEmissionTest,
	"Box3DUnreal.Liquid.EmissionRateAndBudget", BOX3D_TEST_FLAGS)
bool FBox3DLiquidEmissionTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	ABox3DLiquidSourceActor* Liquid = SpawnLiquidSource(Test.World, PouringDownAt(FVector(0, 0, 300)),
		[](ABox3DLiquidSourceActor& Source)
		{
			Source.SpawnRate = 600.0f;
			Source.MaxParticles = 35;
			Source.ParticleLifetime = 0.0f;
			Source.bCohesion = false;
			Source.InitialSpeed = 100.0f;
		});

	const float PerStep = 600.0f * Box3DTest::FTestWorld::FixedDt();
	Test.Step(3);
	const int32 Expected = FMath::FloorToInt(3.0f * PerStep);
	TestEqual(TEXT("emission follows the configured rate"), Liquid->GetParticleCount(), Expected);

	// Well past the cap: recycling holds the count at MaxParticles exactly.
	Test.Step(5);
	TestEqual(TEXT("budget caps the swarm"), Liquid->GetParticleCount(), 35);

	Liquid->StopFlow();
	Test.Step(3);
	TestEqual(TEXT("stopping the flow keeps the pool (no lifetime)"), Liquid->GetParticleCount(), 35);

	Liquid->DespawnAll();
	TestEqual(TEXT("DespawnAll clears the swarm"), Liquid->GetParticleCount(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DLiquidLifetimeTest,
	"Box3DUnreal.Liquid.LifetimeDespawn", BOX3D_TEST_FLAGS)
bool FBox3DLiquidLifetimeTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	const float Dt = Box3DTest::FTestWorld::FixedDt();
	ABox3DLiquidSourceActor* Liquid = SpawnLiquidSource(Test.World, PouringDownAt(FVector(0, 0, 100)),
		[Dt](ABox3DLiquidSourceActor& Source)
		{
			Source.bAutoStart = false;
			Source.ParticleLifetime = 6.0f * Dt;
			Source.DespawnShrinkSeconds = 3.0f * Dt;
			Source.bCohesion = false;
		});

	Liquid->SpawnBurst(15);
	TestEqual(TEXT("burst spawns immediately"), Liquid->GetParticleCount(), 15);

	Test.Step(5);
	TestEqual(TEXT("alive before the lifetime elapses"), Liquid->GetParticleCount(), 15);

	// Lifetime + shrink window fully elapsed (ages advance one step at a time).
	Test.Step(6);
	TestEqual(TEXT("expired particles despawn"), Liquid->GetParticleCount(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DLiquidPourTest,
	"Box3DUnreal.Liquid.PoursAndSpreads", BOX3D_TEST_FLAGS)
bool FBox3DLiquidPourTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);

	ABox3DLiquidSourceActor* Liquid = SpawnLiquidSource(Test.World, PouringDownAt(FVector(0, 0, 250)),
		[](ABox3DLiquidSourceActor& Source)
		{
			Source.SpawnRate = 300.0f;
			Source.MaxParticles = 60;
			Source.ParticleLifetime = 0.0f;
			Source.InitialSpeed = 200.0f;
		});

	// Fill the budget, then close the tap — while flowing, recycling keeps
	// teleporting the oldest particles back to the nozzle, so nothing settles.
	Test.Step(60);
	TestEqual(TEXT("pool at the particle budget"), Liquid->GetParticleCount(), 60);
	Liquid->StopFlow();
	Test.Step(180);

	float MaxHorizontal = 0.0f;
	int32 InPuddle = 0;
	float MinZ = TNumericLimits<float>::Max();
	float MaxZ = -TNumericLimits<float>::Max();
	for (const FVector& Location : Liquid->GetParticleLocations())
	{
		const float Horizontal = FVector2D(Location.X, Location.Y).Size();
		MaxHorizontal = FMath::Max(MaxHorizontal, Horizontal);
		InPuddle += Horizontal < 250.0f ? 1 : 0;
		MinZ = FMath::Min(MinZ, float(Location.Z));
		MaxZ = FMath::Max(MaxZ, float(Location.Z));
	}

	TestTrue(FString::Printf(TEXT("puddle spread beyond the nozzle footprint (%f cm)"), MaxHorizontal),
		MaxHorizontal > 25.0f);
	// The bulk must pool; the occasional splash droplet skittering off is
	// physical, so the far tail is only bounded loosely (still on the ground).
	TestTrue(FString::Printf(TEXT("bulk of the pool held together (%d/60 within 250 cm)"), InPuddle),
		InPuddle >= 54);
	TestTrue(FString::Printf(TEXT("no runaway particle (%f cm)"), MaxHorizontal),
		MaxHorizontal < 1000.0f);
	TestTrue(FString::Printf(TEXT("no particle fell through the ground (minZ=%f)"), MinZ),
		MinZ > 0.0f);
	TestTrue(FString::Printf(TEXT("pool rests near the ground, not exploded upward (maxZ=%f)"), MaxZ),
		MaxZ < 200.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DLiquidCohesionTest,
	"Box3DUnreal.Liquid.CohesionAttracts", BOX3D_TEST_FLAGS)
bool FBox3DLiquidCohesionTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Free fall, no ground: gravity cancels out of relative distances.
	ABox3DLiquidSourceActor* Liquid = SpawnLiquidSource(Test.World, FTransform(FVector(0, 0, 1000)),
		[](ABox3DLiquidSourceActor& Source)
		{
			Source.bAutoStart = false;
			Source.ParticleLifetime = 0.0f;
			Source.CohesionRadiusScale = 4.0f;   // radius 6 -> neighbours within 24 cm
			Source.CohesionStrength = 60.0f;
		});

	// One pair inside the neighbour radius, one control pair far outside it.
	Liquid->SpawnParticleAt(FVector(0, 0, 1000), FVector::ZeroVector);
	Liquid->SpawnParticleAt(FVector(20, 0, 1000), FVector::ZeroVector);
	Liquid->SpawnParticleAt(FVector(1000, 0, 1000), FVector::ZeroVector);
	Liquid->SpawnParticleAt(FVector(1200, 0, 1000), FVector::ZeroVector);
	TestEqual(TEXT("four particles placed"), Liquid->GetParticleCount(), 4);

	Test.Step(20);

	const TArray<FVector> Locations = Liquid->GetParticleLocations();
	const float PairDistance = FVector::Dist(Locations[0], Locations[1]);
	const float ControlDistance = FVector::Dist(Locations[2], Locations[3]);

	TestTrue(FString::Printf(TEXT("cohesion pulled the close pair together (%f cm)"), PairDistance),
		PairDistance < 19.0f);
	TestTrue(FString::Printf(TEXT("cohesion did not fling the pair apart (%f cm)"), PairDistance),
		PairDistance > 2.0f);
	TestEqual(TEXT("distant pair unaffected"), ControlDistance, 200.0f, 1.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
