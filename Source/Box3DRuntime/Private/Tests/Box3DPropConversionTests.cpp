// Prop conversion: replacing placed static-mesh actors (and ISM instances) with
// ABox3DPropActor, including static-mirror handoff and Chaos query parity.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DPropActor.h"
#include "Box3DPropConversion.h"
#include "Box3DQueryLibrary.h"
#include "Box3DStaticSceneMirror.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"

namespace
{
	AStaticMeshActor* SpawnPlacedMeshActor(UWorld* World, UStaticMesh* Mesh, const FTransform& Transform)
	{
		AStaticMeshActor* Actor = World->SpawnActor<AStaticMeshActor>(
			AStaticMeshActor::StaticClass(), Transform);
		// Static mobility rejects mesh assignment in a begun-play world; flip to
		// Movable for the assignment, then back (placed actors load pre-assigned).
		UStaticMeshComponent* Component = Actor->GetStaticMeshComponent();
		Component->SetMobility(EComponentMobility::Movable);
		Component->SetStaticMesh(Mesh);
		Component->SetMobility(EComponentMobility::Static);
		return Actor;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DConvertSimulatedActorsTest,
	"Box3DUnreal.Props.ConvertSimulatedActors", BOX3D_TEST_FLAGS)
bool FBox3DConvertSimulatedActorsTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();

	// A Chaos-simulating board (the author's opt-in for Box3D ownership)...
	AStaticMeshActor* Simulated = Test.World->SpawnActor<AStaticMeshActor>(
		AStaticMeshActor::StaticClass(), FTransform(FVector(0, 0, 200)));
	UStaticMeshComponent* SimulatedComponent = Simulated->GetStaticMeshComponent();
	SimulatedComponent->SetMobility(EComponentMobility::Movable);
	SimulatedComponent->SetStaticMesh(Cube);
	SimulatedComponent->BodyInstance.bSimulatePhysics = true;

	// ...and plain static scenery that must stay untouched.
	AStaticMeshActor* Scenery = SpawnPlacedMeshActor(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(500, 0, 200), FVector::OneVector));

	TestEqual(TEXT("only the simulating actor converts"), Box3D::ConvertSimulatedActors(Test.World), 1);
	TestTrue(TEXT("simulating actor destroyed"), !IsValid(Simulated) || Simulated->IsActorBeingDestroyed());
	TestTrue(TEXT("scenery untouched"), IsValid(Scenery) && !Scenery->IsActorBeingDestroyed());

	int32 PropCount = 0;
	ABox3DPropActor* Prop = nullptr;
	for (TActorIterator<ABox3DPropActor> It(Test.World); It; ++It)
	{
		++PropCount;
		Prop = *It;
	}
	TestEqual(TEXT("exactly one prop spawned"), PropCount, 1);
	if (Prop != nullptr)
	{
		TestTrue(TEXT("prop body live in Box3D"), Prop->GetBody()->IsSimulating());
	}

	TestEqual(TEXT("second pass finds nothing left"), Box3D::ConvertSimulatedActors(Test.World), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DPropConvertActorTest,
	"Box3DUnreal.Props.ConvertActor", BOX3D_TEST_FLAGS)
bool FBox3DPropConvertActorTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();

	// Mirrored floor (top Z=0) and a placed 50 cm crate hovering above it.
	Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(0, 0, -25), FVector(8.0, 8.0, 0.5)));
	AStaticMeshActor* Crate = SpawnPlacedMeshActor(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(0, 0, 150), FVector(0.5)));

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	Mirror->MirrorLevel(Test.World->PersistentLevel);
	TestEqual(TEXT("floor and crate mirrored"), Mirror->GetBodyCount(), 2);

	ABox3DPropActor* Prop = Box3D::ConvertToProp(Crate);
	TestNotNull(TEXT("conversion produced a prop"), Prop);
	if (Prop == nullptr)
	{
		return false;
	}

	TestEqual(TEXT("crate's mirror body removed"), Mirror->GetBodyCount(), 1);
	TestTrue(TEXT("original actor destroyed"), !IsValid(Crate) || Crate->IsActorBeingDestroyed());
	TestTrue(TEXT("prop body live"), Prop->GetBody()->IsSimulating());
	TestEqual(TEXT("prop is dynamic"), static_cast<int32>(Prop->GetBody()->BodyType),
		static_cast<int32>(EBox3DBodyType::Dynamic));

	// Falls 125 cm onto the mirrored floor and rests (half extent 25 at scale 0.5).
	Test.Step(180);
	TestEqual(TEXT("prop rests on mirrored floor"), Prop->GetActorLocation().Z, 25.0, 2.0);

	// Chaos query parity: the QueryOnly mesh still blocks engine traces.
	FHitResult ChaosHit;
	const bool bChaosHit = Test.World->LineTraceSingleByChannel(ChaosHit,
		Prop->GetActorLocation() + FVector(0, 0, 200), Prop->GetActorLocation() - FVector(0, 0, 200),
		ECC_Visibility);
	TestTrue(TEXT("engine trace hits the prop"), bChaosHit);
	TestEqual(TEXT("trace resolved to the prop actor"), ChaosHit.GetActor(), static_cast<AActor*>(Prop));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DPropConvertInstanceTest,
	"Box3DUnreal.Props.ConvertInstance", BOX3D_TEST_FLAGS)
bool FBox3DPropConvertInstanceTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();

	AActor* Actor = Test.World->SpawnActor<AActor>();
	UInstancedStaticMeshComponent* Ism = NewObject<UInstancedStaticMeshComponent>(Actor, TEXT("Ism"));
	Ism->SetMobility(EComponentMobility::Static);
	Ism->SetStaticMesh(Cube);
	Actor->SetRootComponent(Ism);
	for (int32 Index = 0; Index < 3; ++Index)
	{
		Ism->AddInstance(FTransform(FVector(Index * 300.0, 0, 50)), /*bWorldSpace*/ false);
	}
	Ism->RegisterComponent();

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	Mirror->MirrorLevel(Test.World->PersistentLevel);
	TestEqual(TEXT("three instance bodies"), Mirror->GetBodyCount(), 3);

	// Convert the middle instance; removal reindexes, so the component re-mirrors.
	ABox3DPropActor* Prop = Box3D::ConvertInstanceToProp(Ism, 1);
	TestNotNull(TEXT("instance conversion produced a prop"), Prop);
	if (Prop == nullptr)
	{
		return false;
	}

	TestEqual(TEXT("instance removed from ISM"), Ism->GetInstanceCount(), 2);
	TestEqual(TEXT("two instance bodies remain"), Mirror->GetBodyCount(), 2);
	TestEqual(TEXT("prop spawned at the instance transform"), Prop->GetActorLocation().X, 300.0, 1.0);
	TestTrue(TEXT("prop body live"), Prop->GetBody()->IsSimulating());

	// The re-mirrored survivors are still solid at their original spots.
	for (const double X : { 0.0, 600.0 })
	{
		FBox3DHitResult Hit;
		const bool bHit = UBox3DQueryLibrary::Box3DRayCast(Test.World,
			FVector(X, 0, 300), FVector(X, 0, -100), FBox3DQueryFilter{}, Hit);
		TestTrue(FString::Printf(TEXT("survivor at X=%.0f hit"), X), bHit);
		TestEqual(FString::Printf(TEXT("survivor at X=%.0f top"), X), Hit.Location.Z, 100.0, 1.0);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
