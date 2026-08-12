// D6 deterministic vertex-denting tests: fixed-order displacement hashing,
// radial falloff/depth clamp, and the fractured-actor render/collision seam.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DDeform.h"
#include "Box3DFracture.h"
#include "Box3DFracturedActor.h"
#include "ProceduralMeshComponent.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	TArray<FVector> MakeDentVertices()
	{
		return {
			FVector(0.0, 0.0, 0.0),
			FVector(5.0, 0.0, 0.0),
			FVector(10.0, 0.0, 0.0),
			FVector(0.0, 11.0, 0.0),
			FVector(-3.0, 4.0, 0.0),
		};
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DVertexDentDeterminismTest,
	"Box3DUnreal.Deform.VertexDentDeterminism", BOX3D_TEST_FLAGS)
bool FBox3DVertexDentDeterminismTest::RunTest(const FString& Parameters)
{
	Box3D::Deform::FBox3DVertexDent Dent;
	Dent.ImpactPoint = FVector::ZeroVector;
	Dent.ImpactNormal = FVector(0.0, 0.0, 7.0);
	Dent.RadiusCm = 10.0;
	Dent.FalloffExponent = 2.0;
	Dent.MaxDepthCm = 4.0;

	const TArray<FVector> Original = MakeDentVertices();
	TArray<FVector> First = Original;
	TArray<FVector> Second = Original;
	const int32 FirstCount = Box3D::Deform::ApplyVertexDent(First, Dent);
	const int32 SecondCount = Box3D::Deform::ApplyVertexDent(Second, Dent);

	TestEqual(TEXT("fixed input displaces the same vertex count"), SecondCount, FirstCount);
	TestEqual(TEXT("same impact produces an identical displaced-vertex hash"),
		Box3D::Deform::VertexHash(Second), Box3D::Deform::VertexHash(First));
	TestNotEqual(TEXT("dent changes the vertex hash"),
		Box3D::Deform::VertexHash(First), Box3D::Deform::VertexHash(Original));
	TestTrue(TEXT("fixed-order output is element-identical"), First == Second);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DVertexDentDepthClampTest,
	"Box3DUnreal.Deform.VertexDentDepthClamp", BOX3D_TEST_FLAGS)
bool FBox3DVertexDentDepthClampTest::RunTest(const FString& Parameters)
{
	Box3D::Deform::FBox3DVertexDent Dent;
	Dent.ImpactNormal = FVector::UpVector;
	Dent.RadiusCm = 10.0;
	Dent.FalloffExponent = 2.0;
	Dent.MaxDepthCm = 4.0;

	const TArray<FVector> Original = MakeDentVertices();
	TArray<FVector> Dented = Original;
	TestEqual(TEXT("only vertices strictly inside the radius move"),
		Box3D::Deform::ApplyVertexDent(Dented, Dent), 3);
	TestTrue(TEXT("centre reaches but does not exceed max depth"),
		Dented[0].Equals(FVector(0.0, 0.0, -4.0), 1.0e-6));
	TestTrue(TEXT("quadratic half-radius falloff gives quarter depth"),
		Dented[1].Equals(FVector(5.0, 0.0, -1.0), 1.0e-6));
	TestTrue(TEXT("radius boundary remains unchanged"), Dented[2].Equals(Original[2], 1.0e-6));
	TestTrue(TEXT("outside radius remains unchanged"), Dented[3].Equals(Original[3], 1.0e-6));

	for (int32 Index = 0; Index < Dented.Num(); ++Index)
	{
		TestTrue(FString::Printf(TEXT("vertex %d respects per-stamp depth clamp"), Index),
			FVector::Dist(Dented[Index], Original[Index]) <= Dent.MaxDepthCm + 1.0e-6);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFracturedActorVertexDentTest,
	"Box3DUnreal.Deform.FracturedActorVertexDent", BOX3D_TEST_FLAGS)
bool FBox3DFracturedActorVertexDentTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UStaticMeshComponent* Component = Box3DTest::SpawnSceneMesh(Test.World,
		Box3DTest::LoadCubeMesh(), FTransform(FVector(25.0, -10.0, 80.0)),
		EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = 61;
	Params.Fracture.CellCount = 4;
	Params.bStartAsleep = true;
	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Component, Params);
	if (!TestNotNull(TEXT("fractured actor spawned"), Actor))
	{
		return false;
	}

	int32 SectionIndex = INDEX_NONE;
	for (int32 FragmentIndex = 0; FragmentIndex < Actor->GetFragmentCount() && SectionIndex == INDEX_NONE;
		++FragmentIndex)
	{
		const FIntPoint Sections = Actor->GetFragmentSections(FragmentIndex);
		SectionIndex = Sections.X != INDEX_NONE ? Sections.X : Sections.Y;
	}
	FProcMeshSection* Section = Actor->GetMesh()->GetProcMeshSection(SectionIndex);
	if (!TestNotNull(TEXT("actor has render geometry"), Section) || Section->ProcVertexBuffer.IsEmpty())
	{
		return false;
	}

	const FVector Before(Section->ProcVertexBuffer[0].Position);
	const uint32 CollisionGeometryHash = Box3D::Fracture::FractureLayoutHash(Actor->GetFragments());
	TestFalse(TEXT("fragment section starts without PMC collision"), Section->bEnableCollision);

	const FVector WorldImpact = Actor->GetActorTransform().TransformPosition(Before);
	const FVector WorldNormal = Actor->GetActorTransform().TransformVectorNoScale(FVector::UpVector);
	TestTrue(TEXT("dent displaces at least the struck render vertex"),
		Actor->ApplyVertexDent(WorldImpact, WorldNormal, 0.25f, 3.0f, 0.0f) > 0);

	Section = Actor->GetMesh()->GetProcMeshSection(SectionIndex);
	if (!TestNotNull(TEXT("section remains available after update"), Section))
	{
		return false;
	}
	const FVector After(Section->ProcVertexBuffer[0].Position);
	TestTrue(TEXT("struck vertex moves inward by the clamped depth"),
		After.Equals(Before - FVector::UpVector * 3.0, 1.0e-4));
	TestFalse(TEXT("render update does not enable PMC collision"), Section->bEnableCollision);
	TestEqual(TEXT("render dent leaves fragment hull source geometry untouched"),
		Box3D::Fracture::FractureLayoutHash(Actor->GetFragments()), CollisionGeometryHash);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
