// D7 destruction replication contract tests: event-stream determinism across
// independent worlds and visual-only regeneration on an authority-gated client
// path with no b3 world.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DDestructibleComponent.h"
#include "Box3DFracturedActor.h"
#include "ProceduralMeshComponent.h"
#include "Tests/Box3DTestHelpers.h"
#include "box3d/box3d.h"

namespace
{
	UBox3DDestructibleComponent* AddReplicationDestructible(UStaticMeshComponent* Mesh)
	{
		UBox3DDestructibleComponent* Destructible =
			NewObject<UBox3DDestructibleComponent>(Mesh->GetOwner(), TEXT("ReplicationDestructible"));
		Destructible->RegisterComponent();
		return Destructible;
	}

	FBox3DDestructionEvent MakeEvent(const UStaticMeshComponent& Component,
		int32 Seed, int32 CellCount, const FVector& Impact)
	{
		FBox3DDestructionEvent Event;
		Event.MeshId = Box3D::GetDestructionMeshId(Component);
		Event.Impact = Impact;
		Event.Seed = Seed;
		Event.Params.CellCount = CellCount;
		Event.Params.ImpactRadius = 45.0;
		Event.Params.RadialBias = 0.65;
		Event.Params.MinFragmentVolume = 20.0;
		Event.Params.MaterialToughness = 75.0f;
		Event.Params.FragmentDensity = 550.0f;
		Event.Params.bStartAsleep = true;
		return Event;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionEventStreamDeterminismTest,
	"Box3DUnreal.Destruction.Replication.EventStreamDeterminism", BOX3D_TEST_FLAGS)
bool FBox3DDestructionEventStreamDeterminismTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedDestructionSettings UnlimitedPool(0, 0.0f);
	Box3DTest::FTestWorld ContextA(TEXT("Box3DEventContextA"));
	Box3DTest::FTestWorld ContextB(TEXT("Box3DEventContextB"));
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();
	if (!TestNotNull(TEXT("cube mesh loads"), Cube))
	{
		return false;
	}

	const TArray<int32> Seeds = { 17, 42, 901 };
	for (int32 EventIndex = 0; EventIndex < Seeds.Num(); ++EventIndex)
	{
		const FVector Location(EventIndex * 250.0, -35.0, 70.0);
		const FTransform Transform(FRotator(0.0, EventIndex * 13.0, 0.0), Location);
		UStaticMeshComponent* MeshA = Box3DTest::SpawnSceneMesh(ContextA.World, Cube, Transform,
			EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
		UStaticMeshComponent* MeshB = Box3DTest::SpawnSceneMesh(ContextB.World, Cube, Transform,
			EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
		UBox3DDestructibleComponent* DestructibleA = AddReplicationDestructible(MeshA);
		UBox3DDestructibleComponent* DestructibleB = AddReplicationDestructible(MeshB);

		const FBox3DDestructionEvent Event = MakeEvent(
			*MeshA, Seeds[EventIndex], 7 + EventIndex * 3, Location + FVector(15.0, -5.0, 20.0));
		FBox3DDestructionEventResult ResultA;
		ABox3DFracturedActor* ActorA = DestructibleA->ApplyDestructionEvent(Event, 0, ResultA);
		if (!TestNotNull(FString::Printf(TEXT("event %d applies in context A"), EventIndex), ActorA))
		{
			return false;
		}

		FBox3DDestructionEventResult ResultB;
		ABox3DFracturedActor* ActorB = DestructibleB->ApplyDestructionEvent(
			Event, ResultA.FractureLayoutHash, ResultB);
		if (!TestNotNull(FString::Printf(TEXT("event %d applies in context B"), EventIndex), ActorB))
		{
			return false;
		}

		TestFalse(TEXT("authority contexts build physics"), ResultA.bVisualOnly || ResultB.bVisualOnly);
		TestTrue(TEXT("authority layout hash is validated"), ResultB.bLayoutHashValidated);
		TestFalse(TEXT("identical stream needs no correction"), ResultB.bNeedsServerCorrection);
		TestEqual(FString::Printf(TEXT("event %d layout hash matches"), EventIndex),
			ResultA.FractureLayoutHash, ResultB.FractureLayoutHash);
		TestEqual(FString::Printf(TEXT("event %d bond-health hash matches"), EventIndex),
			ResultA.BondHealthHash, ResultB.BondHealthHash);
		TestEqual(TEXT("actor layouts match their event results"),
			static_cast<int64>(Box3D::Fracture::FractureLayoutHash(ActorA->GetFragments())),
			ResultA.FractureLayoutHash);
		TestEqual(TEXT("independent actors have equal fragment counts"),
			ActorA->GetFragmentCount(), ActorB->GetFragmentCount());

		if (EventIndex == 0)
		{
			UStaticMeshComponent* CorrectionMesh = Box3DTest::SpawnSceneMesh(ContextB.World, Cube, Transform,
				EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
			UBox3DDestructibleComponent* CorrectionTarget = AddReplicationDestructible(CorrectionMesh);
			FBox3DDestructionEventResult CorrectionResult;
			TestNotNull(TEXT("mismatched hash still regenerates the authority event"),
				CorrectionTarget->ApplyDestructionEvent(
					Event, ResultA.FractureLayoutHash + 1, CorrectionResult));
			TestTrue(TEXT("layout mismatch requests server correction"),
				CorrectionResult.bNeedsServerCorrection);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDestructionClientVisualEventTest,
	"Box3DUnreal.Destruction.Replication.ClientVisualWithoutWorld", BOX3D_TEST_FLAGS)
bool FBox3DDestructionClientVisualEventTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test(TEXT("Box3DVisualClientContext"));
	UStaticMeshComponent* Mesh = Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FVector(25.0, -10.0, 80.0)), EComponentMobility::Movable,
		ECollisionEnabled::QueryAndPhysics);
	UBox3DDestructibleComponent* Destructible = AddReplicationDestructible(Mesh);
	const FBox3DDestructionEvent Event = MakeEvent(*Mesh, 73, 11, FVector(35.0, -5.0, 105.0));

	Box3D::Fracture::FFractureProxy Proxy;
	if (!TestTrue(TEXT("client target proxy resolves"),
		Box3D::ResolveFractureProxy(*Mesh, Proxy) != EBox3DFractureProxySource::None))
	{
		return false;
	}
	const FTransform ComponentToWorld(Mesh->GetComponentQuat(), Mesh->GetComponentLocation());
	TArray<Box3D::Fracture::FBox3DFragmentData> ExpectedFragments;
	if (!TestTrue(TEXT("event expands without physics"), Box3D::Destruction::GenerateEventFragments(
			Proxy, ComponentToWorld, Event, ExpectedFragments)))
	{
		return false;
	}
	const int64 ExpectedHash = static_cast<int64>(
		Box3D::Fracture::FractureLayoutHash(ExpectedFragments));

	// A pure authority-only client has no b3 world. Invalidating this synthetic
	// world's handle reproduces that production seam while leaving the UE render
	// world alive for the PMC assertion below.
	b3DestroyWorld(Test.B3World());
	TestFalse(TEXT("client has no b3 world"), b3World_IsValid(Test.B3World()));
	TestFalse(TEXT("no-world context is not simulation authority"),
		Test.Subsystem().IsSimulationAuthority());

	FBox3DDestructionEvent RejectedLocalEvent;
	TestFalse(TEXT("client cannot make a local fracture decision"),
		Destructible->BuildDestructionEvent(Event.Impact, 5000.0f, RejectedLocalEvent));

	FBox3DDestructionEventResult Result;
	ABox3DFracturedActor* Actor = Destructible->ApplyDestructionEvent(Event, ExpectedHash, Result);
	if (!TestNotNull(TEXT("received event builds client visuals"), Actor))
	{
		return false;
	}
	TestTrue(TEXT("received event used visual-only path"), Result.bVisualOnly);
	TestTrue(TEXT("client layout validated"), Result.bLayoutHashValidated);
	TestFalse(TEXT("matching client layout needs no correction"), Result.bNeedsServerCorrection);
	TestEqual(TEXT("client visual layout matches pure event expansion"),
		Result.FractureLayoutHash, ExpectedHash);
	TestEqual(TEXT("all event fragments retained"), Actor->GetFragmentCount(), ExpectedFragments.Num());
	TestEqual(TEXT("visual-only actor creates no welds"), Actor->GetLiveWeldCount(), 0);

	int32 VisibleSectionCount = 0;
	bool bAnyBody = false;
	for (int32 FragmentIndex = 0; FragmentIndex < Actor->GetFragmentCount(); ++FragmentIndex)
	{
		const FIntPoint Sections = Actor->GetFragmentSections(FragmentIndex);
		VisibleSectionCount += Sections.X != INDEX_NONE ? 1 : 0;
		VisibleSectionCount += Sections.Y != INDEX_NONE ? 1 : 0;
		bAnyBody |= b3Body_IsValid(Actor->GetFragmentBody(FragmentIndex));
	}
	TestTrue(TEXT("PMC visual sections were built"), VisibleSectionCount > 0);
	TestFalse(TEXT("visual fragment set has no b3 bodies"), bAnyBody);
	TestFalse(TEXT("received event swaps out intact source"), Mesh->IsVisible());
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
