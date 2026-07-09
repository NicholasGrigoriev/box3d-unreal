// Rope and cloth soft-body actors: chains/lattices of raw bodies behind a
// purely visual skin. These tests assert on the physics (hang lengths, sag,
// pinning, teardown) — the spline/procedural mesh skins are checked for
// existence and vertex count, not appearance.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DClothActor.h"
#include "Box3DConversion.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSoftBodyRopeCutsTest,
	"Box3DUnreal.SoftBody.RopeCuts", BOX3D_TEST_FLAGS)
bool FBox3DSoftBodyRopeCutsTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;

	const FTransform Transform(FQuat::Identity, FVector(0.0, 0.0, 300.0));
	ABox3DRopeActor* Rope = TestWorld.World->SpawnActorDeferred<ABox3DRopeActor>(
		ABox3DRopeActor::StaticClass(), Transform);
	Rope->RopeLength = 200.0f;
	Rope->NumSegments = 8;
	Rope->FinishSpawning(Transform);

	TestWorld.Step(60);
	TestEqual(TEXT("All links alive while hanging"), Rope->GetLiveLinkCount(), 8);
	TestFalse(TEXT("Rope starts uncut"), Rope->IsCut());

	// Explicit cut mid-chain: the lower half free-falls.
	Rope->CutLink(4);
	TestTrue(TEXT("CutLink marks the rope cut"), Rope->IsCut());
	TestEqual(TEXT("CutLink removed one link"), Rope->GetLiveLinkCount(), 7);
	TestWorld.Step(90);
	TestTrue(TEXT("Severed lower half falls away"), Rope->GetEndLocation().Z < 40.0);

	// Force path: with a 1 N threshold, the hanging upper chain's own weight
	// snaps more links on the next poll.
	Rope->LinkBreakForce = 1.0f;
	TestWorld.Step(1);
	Rope->PollLinkBreaks();
	TestTrue(TEXT("Overloaded links snap via LinkBreakForce"), Rope->GetLiveLinkCount() < 7);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSoftBodyRopeWinchTest,
	"Box3DUnreal.SoftBody.RopeWinchesDeployedLength", BOX3D_TEST_FLAGS)
bool FBox3DSoftBodyRopeWinchTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;

	const FTransform Transform(FQuat::Identity, FVector(0.0, 0.0, 400.0));
	ABox3DRopeActor* Rope = TestWorld.World->SpawnActorDeferred<ABox3DRopeActor>(
		ABox3DRopeActor::StaticClass(), Transform);
	Rope->RopeLength = 320.0f;
	Rope->NumSegments = 16;
	Rope->FinishSpawning(Transform);

	TestEqual(TEXT("A fresh rope is fully paid out"), Rope->GetDeployedLength(), 320.0f, 1.0f);

	// Reel in half: the spooled tail disables, the live end hangs at half length.
	Rope->SetDeployedLength(160.0f);
	TestEqual(TEXT("Reeling quantizes to whole segments"), Rope->GetDeployedLength(), 160.0f, 20.0f);
	TestWorld.Step(240);
	TestEqual(TEXT("Half-deployed rope hangs to half length"), Rope->GetEndLocation().Z, 240.0, 20.0);

	// Floor quantization: a mid-link request must never deploy MORE chain than
	// asked — extra links read as slack on a freshly connected grapple rope.
	Rope->SetDeployedLength(170.0f);
	TestTrue(TEXT("Quantization never deploys past the requested length"),
		Rope->GetDeployedLength() <= 170.0f + UE_KINDA_SMALL_NUMBER);

	// Pay back out: b3Body_Disable only deactivated the chain joints, so the
	// revived tail must hang connected — a snapped chain would free-fall away.
	Rope->SetDeployedLength(320.0f);
	TestEqual(TEXT("Payout restores the full length"), Rope->GetDeployedLength(), 320.0f, 1.0f);
	TestWorld.Step(300);
	TestEqual(TEXT("Re-deployed rope hangs connected to full length"),
		Rope->GetEndLocation().Z, 80.0, 25.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSoftBodyRopeTensionTest,
	"Box3DUnreal.SoftBody.RopeEndTensionReadsHangingLoad", BOX3D_TEST_FLAGS)
bool FBox3DSoftBodyRopeTensionTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;

	const FTransform Transform(FQuat::Identity, FVector(0.0, 0.0, 300.0));
	ABox3DRopeActor* Rope = TestWorld.World->SpawnActorDeferred<ABox3DRopeActor>(
		ABox3DRopeActor::StaticClass(), Transform);
	Rope->RopeLength = 200.0f;
	Rope->NumSegments = 8;
	Rope->FinishSpawning(Transform);

	// 50 cm cube at density 80 = 10 kg, hung just below the rope end (Z = 100).
	UBox3DBodyComponent* Load = Box3DTest::SpawnBody(TestWorld.World, FVector(0.0, 0.0, 70.0),
		[](UBox3DBodyComponent& Body)
		{
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(25.0);
			Body.Density = 80.0f;
		});
	TestEqual(TEXT("Load weighs 10 kg"), Load->GetMass(), 10.0f, 0.5f);

	TestTrue(TEXT("Load attaches to the rope end"), Rope->AttachBodyToEnd(Load));
	TestTrue(TEXT("End attachment registers"), Rope->HasEndAttachment());

	TestWorld.Step(600);

	TestTrue(TEXT("Load hangs instead of falling"), Load->GetComponentLocation().Z > -50.0);
	const FVector Tension = Rope->GetEndConstraintForce();
	TestEqual(TEXT("End joint carries the load's weight (~98 N)"),
		static_cast<float>(Tension.Size()), 98.0f, 25.0f);
	TestTrue(TEXT("Tension is carried vertically"),
		FMath::Abs(Tension.Z) > Tension.Size() * 0.9);
	return true;
}

namespace
{
	struct FRopeFilterScan
	{
		bool bFoundRopeShape = false;
		bool bAnyPawnMask = false;
	};

	bool CollectRopeFilters(b3ShapeId ShapeId, void* Context)
	{
		FRopeFilterScan* Scan = static_cast<FRopeFilterScan*>(Context);
		const b3Filter Filter = b3Shape_GetFilter(ShapeId);
		if (Filter.categoryBits & (1ull << static_cast<int32>(EBox3DChannel::Debris)))
		{
			Scan->bFoundRopeShape = true;
			Scan->bAnyPawnMask |= (Filter.maskBits & (1ull << static_cast<int32>(EBox3DChannel::Pawn))) != 0;
		}
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSoftBodyRopeBuildDirectionTest,
	"Box3DUnreal.SoftBody.RopeBuildsAlongDirectionWithoutPawnCollision", BOX3D_TEST_FLAGS)
bool FBox3DSoftBodyRopeBuildDirectionTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld TestWorld;

	const FTransform Transform(FQuat::Identity, FVector(0.0, 0.0, 300.0));
	ABox3DRopeActor* Rope = TestWorld.World->SpawnActorDeferred<ABox3DRopeActor>(
		ABox3DRopeActor::StaticClass(), Transform);
	Rope->RopeLength = 200.0f;
	Rope->NumSegments = 8;
	Rope->BuildDirection = FVector(1.0, 0.0, 0.0);
	Rope->bCollideWithPawns = false;
	Rope->FinishSpawning(Transform);

	// Before any step: the chain lies exactly along the build direction.
	const FVector End = Rope->GetEndLocation();
	TestEqual(TEXT("Chain builds along +X"), End.X, 200.0, 2.0);
	TestEqual(TEXT("Chain stays level at build"), End.Z, 300.0, 2.0);

	FRopeFilterScan Scan;
	const b3AABB Bounds{ Box3D::ToB3(FVector(-50.0, -50.0, 250.0)), Box3D::ToB3(FVector(250.0, 50.0, 350.0)) };
	b3World_OverlapAABB(TestWorld.B3World(), Bounds, b3DefaultQueryFilter(), &CollectRopeFilters, &Scan);
	TestTrue(TEXT("Overlap sweep found the rope shapes"), Scan.bFoundRopeShape);
	TestFalse(TEXT("bCollideWithPawns=false strips the pawn mask bit"), Scan.bAnyPawnMask);
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
	const FProcMeshSection* BackSection = Cloth->GetClothMesh()->GetProcMeshSection(1);
	if (TestNotNull(TEXT("Double-sided cloth has a back section"), BackSection))
	{
		TestEqual(TEXT("Back section mirrors every vertex"), BackSection->ProcVertexBuffer.Num(), 36);
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
