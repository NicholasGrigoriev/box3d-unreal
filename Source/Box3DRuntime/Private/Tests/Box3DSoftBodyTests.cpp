// Rope and cloth soft-body actors: chains/lattices of raw bodies behind a
// purely visual skin. These tests assert on the physics (hang lengths, sag,
// pinning, teardown) — the spline/procedural mesh skins are checked for
// existence and vertex count, not appearance.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DClothActor.h"
#include "Box3DRopeActor.h"
#include "Components/SplineMeshComponent.h"
#include "ProceduralMeshComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSoftBodyRopeHangsTest,
	"Box3DUnreal.SoftBody.RopeHangs", BOX3D_TEST_FLAGS)
bool FBox3DSoftBodyRopeHangsTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;
	const int32 BaselineBodies = TestWorld.Subsystem().GetWorldStats().BodyCount;

	const FTransform Transform(FQuat::Identity, FVector(0.0, 0.0, 300.0));
	ABox3DRopeActor* Rope = TestWorld.World->SpawnActorDeferred<ABox3DRopeActor>(
		ABox3DRopeActor::StaticClass(), Transform);
	Rope->RopeLength = 200.0f;
	Rope->NumSegments = 8;
	Rope->FinishSpawning(Transform);

	TestEqual(TEXT("Segment bodies plus the kinematic anchor exist"),
		TestWorld.Subsystem().GetWorldStats().BodyCount, BaselineBodies + 8 + 1);

	TInlineComponentArray<USplineMeshComponent*> Skins(Rope);
	TestEqual(TEXT("One spline mesh per segment"), Skins.Num(), 8);

	TestWorld.Step(300); // 5 s: hang and settle

	const FVector End = Rope->GetEndLocation();
	TestEqual(TEXT("Rope hangs to its length"), End.Z, 100.0, 12.0);
	TestEqual(TEXT("Free-hanging rope stays vertical (X)"), End.X, 0.0, 5.0);
	TestEqual(TEXT("Free-hanging rope stays vertical (Y)"), End.Y, 0.0, 5.0);

	TestWorld.World->DestroyActor(Rope);
	TestEqual(TEXT("Destroying the rope releases every body"),
		TestWorld.Subsystem().GetWorldStats().BodyCount, BaselineBodies);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSoftBodyRopePinnedSagsTest,
	"Box3DUnreal.SoftBody.RopePinnedSags", BOX3D_TEST_FLAGS)
bool FBox3DSoftBodyRopePinnedSagsTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;

	// 300 cm of rope across a 240 cm horizontal span: a catenary ~70 cm deep.
	const FTransform Transform(FQuat::Identity, FVector(0.0, 0.0, 300.0));
	ABox3DRopeActor* Rope = TestWorld.World->SpawnActorDeferred<ABox3DRopeActor>(
		ABox3DRopeActor::StaticClass(), Transform);
	Rope->RopeLength = 300.0f;
	Rope->NumSegments = 10;
	Rope->bPinEnd = true;
	Rope->EndPinLocation = FVector(240.0, 0.0, 0.0);
	Rope->FinishSpawning(Transform);

	TestWorld.Step(400);

	const FVector End = Rope->GetEndLocation();
	TestEqual(TEXT("End stays at the pin (X)"), End.X, 240.0, 15.0);
	TestEqual(TEXT("End stays at the pin (Z)"), End.Z, 300.0, 15.0);

	const FVector Middle = Rope->GetSegmentLocation(5);
	TestTrue(TEXT("Slack rope sags below the endpoints"), Middle.Z < 270.0);
	TestTrue(TEXT("Pinned rope does not fall"), Middle.Z > 150.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSoftBodyClothDrapesTest,
	"Box3DUnreal.SoftBody.ClothDrapes", BOX3D_TEST_FLAGS)
bool FBox3DSoftBodyClothDrapesTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;
	const int32 BaselineBodies = TestWorld.Subsystem().GetWorldStats().BodyCount;

	const FTransform Transform(FQuat::Identity, FVector(0.0, 0.0, 300.0));
	ABox3DClothActor* Cloth = TestWorld.World->SpawnActorDeferred<ABox3DClothActor>(
		ABox3DClothActor::StaticClass(), Transform);
	Cloth->Width = 100.0f;
	Cloth->Height = 100.0f;
	Cloth->Columns = 6;
	Cloth->Rows = 6;
	Cloth->PinMode = EBox3DClothPin::TopEdge;
	Cloth->FinishSpawning(Transform);

	TestEqual(TEXT("Particle bodies plus the kinematic anchor exist"),
		TestWorld.Subsystem().GetWorldStats().BodyCount, BaselineBodies + 36 + 1);

	const FProcMeshSection* Section = Cloth->GetClothMesh()->GetProcMeshSection(0);
	if (TestNotNull(TEXT("Cloth skin section exists"), Section))
	{
		TestEqual(TEXT("One skin vertex per particle"), Section->ProcVertexBuffer.Num(), 36);
	}

	TestWorld.Step(400);

	// Top edge pinned: corner particles hold their spots.
	const FVector TopLeft = Cloth->GetParticleLocation(0, 0);
	TestEqual(TEXT("Pinned corner stays put (X)"), TopLeft.X, -50.0, 3.0);
	TestEqual(TEXT("Pinned corner stays put (Z)"), TopLeft.Z, 300.0, 3.0);

	// Bottom edge drapes to roughly the cloth height below the top.
	const FVector BottomMiddle = Cloth->GetParticleLocation(5, 3);
	TestTrue(TEXT("Bottom edge hangs down"), BottomMiddle.Z < 225.0);
	TestTrue(TEXT("Bottom edge does not stretch away"), BottomMiddle.Z > 175.0);

	// The lattice held its width: bottom corners stay spread apart.
	const double BottomSpread = Cloth->GetParticleLocation(5, 5).X - Cloth->GetParticleLocation(5, 0).X;
	TestTrue(TEXT("Bottom row keeps most of its width"), BottomSpread > 60.0);

	TestWorld.World->DestroyActor(Cloth);
	TestEqual(TEXT("Destroying the cloth releases every body"),
		TestWorld.Subsystem().GetWorldStats().BodyCount, BaselineBodies);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
