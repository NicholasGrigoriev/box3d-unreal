// Static scene mirror: discovery filtering, raw-body bookkeeping across
// mirror/unmirror cycles, ISM instances, mirrored (negative) scale, and dynamic
// bodies resting on mirrored geometry. FTestWorld never fires OnWorldBeginPlay,
// so every test calls MirrorLevel on the persistent level explicitly.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DQueryLibrary.h"
#include "Box3DStaticSceneMirror.h"
#include "Components/InstancedStaticMeshComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DMirrorDiscoveryTest,
	"Box3DUnreal.Mirror.Discovery", BOX3D_TEST_FLAGS)
bool FBox3DMirrorDiscoveryTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();
	TestNotNull(TEXT("engine cube mesh"), Cube);

	// Floor 800x800x50 cm, top at Z=0: the only component that qualifies.
	Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(0, 0, -25), FVector(8.0, 8.0, 0.5)));
	// Query-only "trigger" and a movable mesh: both filtered out.
	Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(0, 0, 100)),
		EComponentMobility::Static, ECollisionEnabled::QueryOnly);
	Box3DTest::SpawnSceneMesh(Test.World, Cube,
		FTransform(FQuat::Identity, FVector(300, 0, 100)),
		EComponentMobility::Movable);

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	TestNotNull(TEXT("mirror enabled via settings"), Mirror);
	Mirror->MirrorLevel(Test.World->PersistentLevel);

	TestEqual(TEXT("only the static physics-enabled floor mirrored"), Mirror->GetBodyCount(), 1);

	// Rays resolve to no component (raw bodies, null userData) but must hit.
	FBox3DHitResult Hit;
	const bool bHit = UBox3DQueryLibrary::Box3DRayCast(Test.World,
		FVector(0, 0, 200), FVector(0, 0, -200), FBox3DQueryFilter{}, Hit);
	TestTrue(TEXT("ray hits mirrored floor"), bHit);
	TestEqual(TEXT("hit at floor top"), Hit.Location.Z, 0.0, 1.0);
	TestNull(TEXT("mirror hits carry no component"), Hit.Component.Get());
	TestNull(TEXT("mirror hits carry no actor"), Hit.Actor.Get());

	// The floor sits on the WorldStatic channel.
	FBox3DQueryFilter StaticOnly;
	StaticOnly.MaskBits = 1 << static_cast<int32>(EBox3DChannel::WorldStatic);
	FBox3DQueryFilter DynamicOnly;
	DynamicOnly.MaskBits = 1 << static_cast<int32>(EBox3DChannel::WorldDynamic);
	FBox3DHitResult ChannelHit;
	TestTrue(TEXT("WorldStatic mask hits"), UBox3DQueryLibrary::Box3DRayCast(Test.World,
		FVector(0, 0, 200), FVector(0, 0, -200), StaticOnly, ChannelHit));
	TestFalse(TEXT("WorldDynamic mask misses"), UBox3DQueryLibrary::Box3DRayCast(Test.World,
		FVector(0, 0, 200), FVector(0, 0, -200), DynamicOnly, ChannelHit));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DMirrorBookkeepingTest,
	"Box3DUnreal.Mirror.StreamingBookkeeping", BOX3D_TEST_FLAGS)
bool FBox3DMirrorBookkeepingTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();

	for (int32 Index = 0; Index < 3; ++Index)
	{
		Box3DTest::SpawnSceneMesh(Test.World, Cube,
			FTransform(FQuat::Identity, FVector(Index * 300.0, 0, 0)));
	}

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	ULevel* Level = Test.World->PersistentLevel;

	Mirror->MirrorLevel(Level);
	TestEqual(TEXT("three bodies after mirror"), Mirror->GetBodyCount(), 3);
	const int32 WorldBodies = Test.Subsystem().GetWorldStats().BodyCount;
	TestEqual(TEXT("world body count matches"), WorldBodies, 3);

	// Re-mirroring a mirrored level is a no-op (delegate + initial scan overlap).
	Mirror->MirrorLevel(Level);
	TestEqual(TEXT("idempotent"), Mirror->GetBodyCount(), 3);

	// Stream-out destroys everything; stream-in brings it back.
	Mirror->UnmirrorLevel(Level);
	TestEqual(TEXT("bodies destroyed on unmirror"), Mirror->GetBodyCount(), 0);
	TestEqual(TEXT("b3 world empty again"), Test.Subsystem().GetWorldStats().BodyCount, 0);

	Mirror->MirrorLevel(Level);
	TestEqual(TEXT("re-mirrored"), Mirror->GetBodyCount(), 3);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DMirrorInstancedTest,
	"Box3DUnreal.Mirror.InstancedMeshes", BOX3D_TEST_FLAGS)
bool FBox3DMirrorInstancedTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;
	UStaticMesh* Cube = Box3DTest::LoadCubeMesh();

	// One ISM, three instances: cubes centered at Z=50, tops at Z=100.
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
	TestEqual(TEXT("one body per instance"), Mirror->GetBodyCount(), 3);

	// Every instance is individually solid.
	for (int32 Index = 0; Index < 3; ++Index)
	{
		FBox3DHitResult Hit;
		const bool bHit = UBox3DQueryLibrary::Box3DRayCast(Test.World,
			FVector(Index * 300.0, 0, 300), FVector(Index * 300.0, 0, -100), FBox3DQueryFilter{}, Hit);
		TestTrue(FString::Printf(TEXT("instance %d hit"), Index), bHit);
		TestEqual(FString::Printf(TEXT("instance %d top"), Index), Hit.Location.Z, 100.0, 1.0);
	}

	TestTrue(TEXT("RemoveComponent removes all instance bodies"), Mirror->RemoveComponent(Ism));
	TestEqual(TEXT("no bodies left"), Mirror->GetBodyCount(), 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DMirrorScaleTest,
	"Box3DUnreal.Mirror.MirroredScale", BOX3D_TEST_FLAGS)
bool FBox3DMirrorScaleTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;

	// Negative Y scale mirrors the mesh; the cooked trimesh must still face outward
	// (box3d supports negative mesh scale — this pins that down at the plugin seam).
	Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FQuat::Identity, FVector(0, 0, -25), FVector(4.0, -4.0, 0.5)));

	FBox3DStaticSceneMirror* Mirror = Test.Subsystem().GetStaticMirror();
	Mirror->MirrorLevel(Test.World->PersistentLevel);
	TestEqual(TEXT("mirrored-scale mesh cooked"), Mirror->GetBodyCount(), 1);

	FBox3DHitResult Hit;
	const bool bHit = UBox3DQueryLibrary::Box3DRayCast(Test.World,
		FVector(0, 0, 200), FVector(0, 0, -200), FBox3DQueryFilter{}, Hit);
	TestTrue(TEXT("ray hits mirrored-scale floor"), bHit);
	TestEqual(TEXT("top face height"), Hit.Location.Z, 0.0, 1.0);
	TestTrue(TEXT("normal points up (winding survived the mirror)"), Hit.Normal.Z > 0.9);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DMirrorPropRestTest,
	"Box3DUnreal.Mirror.PropRest", BOX3D_TEST_FLAGS)
bool FBox3DMirrorPropRestTest::RunTest(const FString& Parameters)
{
	Box3DTest::FScopedMirrorSettings MirrorSettings;
	Box3DTest::FTestWorld Test;

	Box3DTest::SpawnSceneMesh(Test.World, Box3DTest::LoadCubeMesh(),
		FTransform(FQuat::Identity, FVector(0, 0, -25), FVector(8.0, 8.0, 0.5)));
	Test.Subsystem().GetStaticMirror()->MirrorLevel(Test.World->PersistentLevel);

	// A dynamic box dropped onto the mirrored floor must settle on it, not fall
	// through: the end-to-end point of the whole feature.
	UBox3DBodyComponent* Box = Box3DTest::SpawnBody(Test.World, FVector(0, 0, 200),
		[](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Dynamic;
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(25.0);
		});

	Test.Step(180);

	TestEqual(TEXT("box rests on mirrored floor"),
		Box->GetComponentLocation().Z, 25.0, 2.0);
	TestFalse(TEXT("box asleep or settling, not tunneled"), Box->GetComponentLocation().Z < 0.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
