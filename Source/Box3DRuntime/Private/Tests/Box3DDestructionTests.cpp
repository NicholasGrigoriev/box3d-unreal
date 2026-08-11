// Tests for the D3 damage pipeline intake, tier policy, and global budgets:
// impact energy from approach speed x mass, the monotonic clamped energy ->
// cell-count curve, volume-threshold tier routing (Body / Debris / Dust) as
// pure data through the fractured actor, hit-event intake against
// destructible-marked mirror bodies, Box3DExplode's fracture-energy fan-out
// with distance falloff, the fragment pool cap with oldest-first eviction, and
// the per-tick fracture budget with cross-tick impact queueing.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DDestructibleComponent.h"
#include "Box3DDestruction.h"
#include "Box3DFracturedActor.h"
#include "Box3DQueryLibrary.h"
#include "Box3DStaticSceneMirror.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	using namespace Box3D::Destruction;

	UBox3DDestructibleComponent* AddDestructible(UStaticMeshComponent* Mesh)
	{
		UBox3DDestructibleComponent* Destructible =
			NewObject<UBox3DDestructibleComponent>(Mesh->GetOwner(), TEXT("Destructible"));
		Destructible->RegisterComponent();
		return Destructible;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionEnergyMappingTest,
	"Box3DUnreal.Destruction.EnergyMapping", BOX3D_TEST_FLAGS)
bool FBox3DDestructionEnergyMappingTest::RunTest(const FString& Parameters)
{
	// 1/2 m v^2 across the cm/s -> m/s seam: 10 kg at 500 cm/s = 125 J.
	TestTrue(TEXT("impact energy is half m v squared in J"),
		FMath::IsNearlyEqual(ImpactEnergyJoules(10.0f, 500.0f), 125.0f, 0.01f));
	TestEqual(TEXT("massless impact carries no energy"), ImpactEnergyJoules(0.0f, 1000.0f), 0.0f);

	const FBox3DEnergyToCellCurve Curve; // defaults: 100 J -> 4 cells, 20 kJ -> 48

	TestEqual(TEXT("zero energy maps to no fracture"), EnergyToCellCount(0.0f, Curve), 0);
	TestEqual(TEXT("energy below MinEnergy is absorbed"), EnergyToCellCount(Curve.MinEnergy - 0.1f, Curve), 0);
	TestEqual(TEXT("MinEnergy maps to MinCellCount"), EnergyToCellCount(Curve.MinEnergy, Curve), Curve.MinCellCount);
	TestEqual(TEXT("FullEnergy maps to MaxCellCount"), EnergyToCellCount(Curve.FullEnergy, Curve), Curve.MaxCellCount);
	TestEqual(TEXT("clamped above FullEnergy"), EnergyToCellCount(Curve.FullEnergy * 1000.0f, Curve),
		Curve.MaxCellCount);

	// Monotonic over a dense sweep of the whole range.
	int32 Previous = 0;
	bool bMonotonic = true;
	for (float Energy = 0.0f; Energy <= Curve.FullEnergy * 1.5f; Energy += Curve.FullEnergy / 400.0f)
	{
		const int32 Cells = EnergyToCellCount(Energy, Curve);
		bMonotonic &= Cells >= Previous;
		Previous = Cells;
	}
	TestTrue(TEXT("cell count is monotonic in energy"), bMonotonic);

	// Degenerate curve (FullEnergy <= MinEnergy) still clamps sanely.
	FBox3DEnergyToCellCurve Degenerate;
	Degenerate.MinEnergy = 100.0f;
	Degenerate.FullEnergy = 100.0f;
	TestEqual(TEXT("degenerate curve absorbs below MinEnergy"), EnergyToCellCount(99.0f, Degenerate), 0);
	TestEqual(TEXT("degenerate curve jumps to MaxCellCount"), EnergyToCellCount(100.0f, Degenerate),
		Degenerate.MaxCellCount);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionTierRoutingTest,
	"Box3DUnreal.Destruction.TierRouting", BOX3D_TEST_FLAGS)
bool FBox3DDestructionTierRoutingTest::RunTest(const FString& Parameters)
{
	// Pure classification: strict less-than thresholds, Dust wins overlaps.
	{
		FBox3DTierThresholds Thresholds;
		Thresholds.PhysicsVolumeThreshold = 10.0;
		Thresholds.RenderVolumeThreshold = 3.0;
		TestTrue(TEXT("large fragment -> Body"),
			ClassifyFragmentTier(10.0, Thresholds) == EBox3DFragmentTier::Body);
		TestTrue(TEXT("mid fragment -> Debris"),
			ClassifyFragmentTier(5.0, Thresholds) == EBox3DFragmentTier::Debris);
		TestTrue(TEXT("tiny fragment -> Dust"),
			ClassifyFragmentTier(2.0, Thresholds) == EBox3DFragmentTier::Dust);
		TestTrue(TEXT("at-threshold volume stays in the higher tier"),
			ClassifyFragmentTier(3.0, Thresholds) == EBox3DFragmentTier::Debris);
		TestTrue(TEXT("zero thresholds keep everything Body"),
			ClassifyFragmentTier(0.001, FBox3DTierThresholds()) == EBox3DFragmentTier::Body);

		FBox3DTierThresholds Inverted;
		Inverted.PhysicsVolumeThreshold = 3.0;
		Inverted.RenderVolumeThreshold = 10.0;
		TestTrue(TEXT("Dust wins where thresholds overlap"),
			ClassifyFragmentTier(5.0, Inverted) == EBox3DFragmentTier::Dust);
	}

	Box3DTest::FTestWorld Test;
	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform::Identity, EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 42;
	Params.Fracture.CellCount = 24;
	Params.bStartAsleep = true;

	// Thresholds picked from the deterministic reference layout's own volume
	// distribution (quartile / median midpoints), so each tier is non-empty and
	// no fragment sits exactly on a boundary.
	Box3D::Fracture::FFractureProxy Proxy;
	if (!TestTrue(TEXT("proxy resolves"),
			Box3D::ResolveFractureProxy(*Component, Proxy) != EBox3DFractureProxySource::None))
	{
		return false;
	}
	TArray<Box3D::Fracture::FBox3DFragmentData> Reference;
	if (!TestTrue(TEXT("reference fracture succeeds"), Box3D::Fracture::Fracture(Proxy, Params.Fracture, Reference))
		|| !TestTrue(TEXT("enough fragments to tier"), Reference.Num() >= 8))
	{
		return false;
	}
	TArray<double> Volumes;
	for (const Box3D::Fracture::FBox3DFragmentData& Fragment : Reference)
	{
		Volumes.Add(Fragment.Volume);
	}
	Volumes.Sort();
	const int32 DustCount = Volumes.Num() / 4;
	const int32 SubPhysicsCount = Volumes.Num() / 2;
	Params.Tiers.RenderVolumeThreshold = (Volumes[DustCount - 1] + Volumes[DustCount]) * 0.5;
	Params.Tiers.PhysicsVolumeThreshold = (Volumes[SubPhysicsCount - 1] + Volumes[SubPhysicsCount]) * 0.5;
	Params.DebrisSpeed = 250.0f;

	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor))
	{
		return false;
	}

	TestEqual(TEXT("all fragments kept in the data (tiers do not reindex)"),
		Actor->GetFragmentCount(), Reference.Num());
	TestEqual(TEXT("quartile of fragments routed to Dust"),
		Actor->CountFragmentsInTier(EBox3DFragmentTier::Dust), DustCount);
	TestEqual(TEXT("quartile of fragments routed to Debris"),
		Actor->CountFragmentsInTier(EBox3DFragmentTier::Debris), SubPhysicsCount - DustCount);
	TestEqual(TEXT("rest routed to Body"), Actor->CountFragmentsInTier(EBox3DFragmentTier::Body),
		Reference.Num() - SubPhysicsCount);

	const TArray<Box3D::Fracture::FBox3DFragmentData>& Fragments = Actor->GetFragments();
	for (int32 Index = 0; Index < Fragments.Num(); ++Index)
	{
		const EBox3DFragmentTier Tier = Actor->GetFragmentTier(Index);
		TestTrue(FString::Printf(TEXT("fragment %d tier matches its volume"), Index),
			Tier == ClassifyFragmentTier(Fragments[Index].Volume, Params.Tiers));

		const FIntPoint Sections = Actor->GetFragmentSections(Index);
		const bool bHasBody = b3Body_IsValid(Actor->GetFragmentBody(Index));
		const bool bHasSection = Sections.X != INDEX_NONE || Sections.Y != INDEX_NONE;
		if (Tier == EBox3DFragmentTier::Body)
		{
			TestTrue(FString::Printf(TEXT("Body fragment %d has a hull body"), Index), bHasBody);
			TestTrue(FString::Printf(TEXT("Body fragment %d has PMC sections"), Index), bHasSection);
		}
		else
		{
			TestFalse(FString::Printf(TEXT("non-Body fragment %d has no body"), Index), bHasBody);
			TestFalse(FString::Printf(TEXT("non-Body fragment %d has no sections"), Index), bHasSection);
		}
	}

	// Welds only ever join two Body-tier fragments.
	TArray<FIntPoint> Pairs;
	Actor->GetLiveWeldPairs(Pairs);
	TestTrue(TEXT("body tier is welded"), Pairs.Num() > 0);
	for (const FIntPoint& Pair : Pairs)
	{
		TestTrue(FString::Printf(TEXT("weld (%d, %d) joins Body-tier fragments"), Pair.X, Pair.Y),
			Actor->GetFragmentTier(Pair.X) == EBox3DFragmentTier::Body
				&& Actor->GetFragmentTier(Pair.Y) == EBox3DFragmentTier::Body);
	}

	// Debris burst arrays: one entry per Debris fragment, cube-equivalent
	// sizes, radial velocities at the configured speed.
	const FBox3DDebrisBurst& Burst = Actor->GetDebrisBurst();
	const int32 DebrisCount = Actor->CountFragmentsInTier(EBox3DFragmentTier::Debris);
	TestEqual(TEXT("one burst position per Debris fragment"), Burst.Positions.Num(), DebrisCount);
	TestEqual(TEXT("one burst velocity per Debris fragment"), Burst.Velocities.Num(), DebrisCount);
	TestEqual(TEXT("one burst size per Debris fragment"), Burst.Sizes.Num(), DebrisCount);
	for (int32 Index = 0; Index < Burst.Sizes.Num(); ++Index)
	{
		TestTrue(FString::Printf(TEXT("burst size %d is a positive edge length"), Index), Burst.Sizes[Index] > 0.0f);
		TestTrue(FString::Printf(TEXT("burst velocity %d has the debris speed"), Index),
			FMath::IsNearlyEqual(Burst.Velocities[Index].Size(), Params.DebrisSpeed, 0.1));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionHitIntakeTest,
	"Box3DUnreal.Destruction.HitEventIntake", BOX3D_TEST_FLAGS)
bool FBox3DDestructionHitIntakeTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;

	// Destructible cube resting at Z 0..100, mirrored AFTER the marker exists
	// so its mirror body carries hit events.
	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FVector(0, 0, 50)));
	UBox3DDestructibleComponent* Destructible = AddDestructible(Component);

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	if (!TestNotNull(TEXT("mirror enabled"), Mirror))
	{
		return false;
	}
	Mirror->MirrorLevel(Test.World->PersistentLevel);
	TestEqual(TEXT("destructible cube mirrored"), Mirror->GetBodyCount(), 1);

	// 125 kg box slammed down at 5 m/s: ~1.5 kJ on impact, far above the
	// default 100 J fracture threshold and the hit-event speed threshold.
	UBox3DBodyComponent* Impactor = Box3DTest::SpawnBody(Test.World, FVector(0, 0, 160),
		[](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Dynamic;
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(25.0);
		});
	Impactor->SetLinearVelocity(FVector(0, 0, -500));

	for (int32 Step = 0; Step < 60 && !Destructible->IsFractured(); ++Step)
	{
		Test.Step(1);
	}

	TestTrue(TEXT("impact fractures the destructible"), Destructible->IsFractured());
	ABox3DFracturedActor* Actor = Destructible->GetFracturedActor();
	if (!TestNotNull(TEXT("fractured actor exists"), Actor))
	{
		return false;
	}
	TestTrue(TEXT("impact energy maps to multiple cells"), Actor->GetFragmentCount() >= 2);
	TestTrue(TEXT("cell cap respected"),
		Actor->GetFragmentCount() <= Destructible->EnergyToCells.MaxCellCount);
	TestFalse(TEXT("source mesh swapped out"), Component->IsVisible());
	TestEqual(TEXT("mirror body removed by the swap"), Mirror->GetBodyCount(), 0);
	TestNull(TEXT("second impact on a fractured mesh is ignored"),
		Destructible->ApplyImpact(FVector(0, 0, 100), 1.0e6f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionExplosionIntakeTest,
	"Box3DUnreal.Destruction.ExplosionIntake", BOX3D_TEST_FLAGS)
bool FBox3DDestructionExplosionIntakeTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	UStaticMeshComponent* NearMesh = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FVector(0, 0, 50)), EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
	UBox3DDestructibleComponent* Near = AddDestructible(NearMesh);

	UStaticMeshComponent* FarMesh = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FVector(2000, 0, 50)), EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
	UBox3DDestructibleComponent* Far = AddDestructible(FarMesh);

	// No fracture energy: the blast stays a pure impulse effect.
	UBox3DQueryLibrary::Box3DExplode(Test.World, FVector(0, 0, 150), 100.0f, 100.0f, 1.0f, FBox3DQueryFilter());
	TestFalse(TEXT("zero fracture energy leaves the near destructible intact"), Near->IsFractured());
	TestFalse(TEXT("zero fracture energy leaves the far destructible intact"), Far->IsFractured());

	// 5 kJ blast 50 cm above the near cube: inside the radius, full energy.
	// The far cube sits ~1.8 km beyond Radius + Falloff and must be untouched.
	UBox3DQueryLibrary::Box3DExplode(Test.World, FVector(0, 0, 150), 100.0f, 100.0f, 1.0f, FBox3DQueryFilter(),
		5000.0f);
	TestTrue(TEXT("blast fractures the near destructible"), Near->IsFractured());
	TestFalse(TEXT("blast falloff spares the far destructible"), Far->IsFractured());

	ABox3DFracturedActor* Actor = Near->GetFracturedActor();
	if (TestNotNull(TEXT("near fractured actor exists"), Actor))
	{
		// 5 kJ through the default curve: well between the min and the cap.
		TestTrue(TEXT("blast energy maps to multiple cells"), Actor->GetFragmentCount() >= 2);
		TestTrue(TEXT("cell cap respected"), Actor->GetFragmentCount() <= Near->EnergyToCells.MaxCellCount);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionFragmentPoolTest,
	"Box3DUnreal.Destruction.FragmentPool", BOX3D_TEST_FLAGS)
bool FBox3DDestructionFragmentPoolTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Identical cubes, seeds, and energies: every fracture yields the same
	// Body-tier fragment count, measured once with the pool unlimited.
	float NextX = 0.0f;
	const auto FractureCube = [&]() -> ABox3DFracturedActor*
	{
		const float X = NextX;
		NextX += 300.0f;
		UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
			FTransform(FVector(X, 0, 50)), EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
		return AddDestructible(Component)->ApplyImpact(FVector(X, 0, 100), 5000.0f);
	};

	int32 PerActor = 0;
	ABox3DFracturedActor* First = nullptr;
	{
		Box3DTest::FScopedDestructionSettings Unlimited(0, 0.0f);
		First = FractureCube();
		if (!TestNotNull(TEXT("first fracture succeeds"), First))
		{
			return false;
		}
		PerActor = First->CountFragmentsInTier(EBox3DFragmentTier::Body);
		TestTrue(TEXT("fracture yields multiple body fragments"), PerActor >= 2);
		TestEqual(TEXT("pool tracks live body fragments"), Test.Subsystem().GetLiveFragmentCount(), PerActor);
	}

	// Cap = two actors' worth: the third fracture must evict exactly the oldest.
	Box3DTest::FScopedDestructionSettings Capped(PerActor * 2, 0.0f);
	ABox3DFracturedActor* Second = FractureCube();
	if (!TestNotNull(TEXT("second fracture succeeds"), Second))
	{
		return false;
	}
	TestTrue(TEXT("at the cap nothing is evicted"), IsValid(First) && IsValid(Second));
	TestEqual(TEXT("pool holds two actors"), Test.Subsystem().GetLiveFracturedActors().Num(), 2);

	ABox3DFracturedActor* Third = FractureCube();
	if (!TestNotNull(TEXT("third fracture succeeds"), Third))
	{
		return false;
	}
	TestFalse(TEXT("over the cap the oldest actor is evicted"), IsValid(First));
	TestTrue(TEXT("younger actors survive eviction"), IsValid(Second) && IsValid(Third));
	TestTrue(TEXT("pool back within the cap"), Test.Subsystem().GetLiveFragmentCount() <= PerActor * 2);

	ABox3DFracturedActor* Fourth = FractureCube();
	if (!TestNotNull(TEXT("fourth fracture succeeds"), Fourth))
	{
		return false;
	}
	TestFalse(TEXT("eviction is strictly oldest-first"), IsValid(Second));
	TestTrue(TEXT("newest actors survive"), IsValid(Third) && IsValid(Fourth));

	// A fracture larger than the whole cap still spawns: the newest actor is
	// never evicted, everything older goes.
	{
		Box3DTest::FScopedDestructionSettings Tiny(1, 0.0f);
		ABox3DFracturedActor* Fifth = FractureCube();
		if (!TestNotNull(TEXT("over-cap fracture still spawns"), Fifth))
		{
			return false;
		}
		TestTrue(TEXT("newest actor is never evicted"), IsValid(Fifth));
		TestFalse(TEXT("all older actors evicted"), IsValid(Third) || IsValid(Fourth));
		TestEqual(TEXT("pool holds only the newest"), Test.Subsystem().GetLiveFracturedActors().Num(), 1);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionFractureBudgetTest,
	"Box3DUnreal.Destruction.FractureBudget", BOX3D_TEST_FLAGS)
bool FBox3DDestructionFractureBudgetTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UBox3DWorldSubsystem& Subsystem = Test.Subsystem();

	const auto SpawnDestructibleCube = [&](float X) -> UBox3DDestructibleComponent*
	{
		UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
			FTransform(FVector(X, 0, 50)), EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
		return AddDestructible(Component);
	};

	UBox3DDestructibleComponent* Targets[3];
	for (int32 Index = 0; Index < 3; ++Index)
	{
		Targets[Index] = SpawnDestructibleCube(Index * 300.0f);
	}

	// A near-zero budget degenerates to the guaranteed minimum of one impact per
	// tick — frame-exact on any machine, however fast the fracture itself is.
	{
		Box3DTest::FScopedDestructionSettings OnePerTick(0, KINDA_SMALL_NUMBER);
		for (int32 Index = 0; Index < 3; ++Index)
		{
			Subsystem.QueueDestructibleImpact(Targets[Index], FVector(Index * 300.0f, 0, 100), 5000.0f);
		}
		TestEqual(TEXT("three impacts queued"), Subsystem.GetPendingDestructibleImpactCount(), 3);

		Test.Step(1);
		TestTrue(TEXT("tick 1 fractures the first target"), Targets[0]->IsFractured());
		TestFalse(TEXT("tick 1 leaves the second queued"), Targets[1]->IsFractured());
		TestFalse(TEXT("tick 1 leaves the third queued"), Targets[2]->IsFractured());
		TestEqual(TEXT("two impacts carry over"), Subsystem.GetPendingDestructibleImpactCount(), 2);

		Test.Step(1);
		TestTrue(TEXT("tick 2 fractures the second target"), Targets[1]->IsFractured());
		TestFalse(TEXT("tick 2 leaves the third queued"), Targets[2]->IsFractured());
		TestEqual(TEXT("one impact carries over"), Subsystem.GetPendingDestructibleImpactCount(), 1);

		Test.Step(1);
		TestTrue(TEXT("tick 3 fractures the third target"), Targets[2]->IsFractured());
		TestEqual(TEXT("queue fully drained"), Subsystem.GetPendingDestructibleImpactCount(), 0);
	}

	// Zero budget = unlimited: the whole queue drains in one tick.
	{
		UBox3DDestructibleComponent* ExtraA = SpawnDestructibleCube(900.0f);
		UBox3DDestructibleComponent* ExtraB = SpawnDestructibleCube(1200.0f);
		Box3DTest::FScopedDestructionSettings Unlimited(0, 0.0f);
		Subsystem.QueueDestructibleImpact(ExtraA, FVector(900, 0, 100), 5000.0f);
		Subsystem.QueueDestructibleImpact(ExtraB, FVector(1200, 0, 100), 5000.0f);
		Test.Step(1);
		TestTrue(TEXT("unlimited budget drains the whole queue in one tick"),
			ExtraA->IsFractured() && ExtraB->IsFractured());
		TestEqual(TEXT("nothing carries over"), Subsystem.GetPendingDestructibleImpactCount(), 0);
	}

	// Fractured and null targets are rejected at the queue door.
	Subsystem.QueueDestructibleImpact(Targets[0], FVector(0, 0, 100), 5000.0f);
	Subsystem.QueueDestructibleImpact(nullptr, FVector::ZeroVector, 5000.0f);
	TestEqual(TEXT("fractured and null targets are not queued"),
		Subsystem.GetPendingDestructibleImpactCount(), 0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
