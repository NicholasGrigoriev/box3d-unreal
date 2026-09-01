// D6 deterministic vertex-denting tests: fixed-order displacement hashing,
// radial falloff/depth clamp, and the fractured-actor render/collision seam.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DDeform.h"
#include "Box3DDentMapComponent.h"
#include "Box3DFracture.h"
#include "Box3DFracturedActor.h"
#include "Engine/TextureRenderTarget2D.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDentMapPingPongTest,
	"Box3DUnreal.Deform.DentMapPingPong", BOX3D_TEST_FLAGS)
bool FBox3DDentMapPingPongTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	UStaticMeshComponent* Mesh = Box3DTest::SpawnSceneMesh(Test.World,
		Box3DTest::LoadCubeMesh(), FTransform::Identity,
		EComponentMobility::Movable, ECollisionEnabled::QueryAndPhysics);
	const ECollisionEnabled::Type CollisionBefore = Mesh->GetCollisionEnabled();
	const UBodySetup* BodySetupBefore = Mesh->GetStaticMesh()->GetBodySetup();

	UBox3DDentMapComponent* DentMap = NewObject<UBox3DDentMapComponent>(Mesh->GetOwner());
	DentMap->DentMapResolution = 64;
	DentMap->RegisterComponent();
	if (!TestTrue(TEXT("dent map initializes on a registered mesh"), DentMap->InitializeDentMap(Mesh)))
	{
		return false;
	}

	UTextureRenderTarget2D* Initial = DentMap->GetDentMap();
	UTextureRenderTarget2D* Other = DentMap->GetInactiveDentMap();
	TestNotNull(TEXT("active render target allocated"), Initial);
	TestNotNull(TEXT("inactive render target allocated"), Other);
	TestNotEqual(TEXT("ping-pong targets are distinct"), Initial, Other);
	TestEqual(TEXT("configured resolution is preserved"), Initial->SizeX, 64);
	TestEqual(TEXT("map uses single-channel cosmetic format"),
		Initial->RenderTargetFormat.GetValue(), RTF_R8);

	TestTrue(TEXT("first valid stamp succeeds"), DentMap->StampDentUV(FVector2D(0.5), 0.2f, 0.4f));
	TestEqual(TEXT("first stamp swaps to the other target"), DentMap->GetDentMap(), Other);
	TestEqual(TEXT("first stamp increments count"), DentMap->GetDentStampCount(), 1);
	TestTrue(TEXT("second valid stamp succeeds"), DentMap->StampDentUV(FVector2D(0.25, 0.75), 0.1f, 1.0f));
	TestEqual(TEXT("second stamp swaps back"), DentMap->GetDentMap(), Initial);
	TestEqual(TEXT("second stamp increments count"), DentMap->GetDentStampCount(), 2);

	TestFalse(TEXT("invalid radius is rejected"), DentMap->StampDentUV(FVector2D(0.5), 0.0f, 1.0f));
	TestEqual(TEXT("invalid stamp does not swap"), DentMap->GetDentMap(), Initial);
	TestEqual(TEXT("invalid stamp does not increment count"), DentMap->GetDentStampCount(), 2);

	TestEqual(TEXT("material contract texture parameter name"),
		UBox3DDentMapComponent::DentMapParameterName, FName(TEXT("Box3D_DentMap")));
	TestEqual(TEXT("material contract normal strength parameter name"),
		UBox3DDentMapComponent::DentNormalStrengthParameterName, FName(TEXT("Box3D_DentNormalStrength")));
	TestEqual(TEXT("material contract texel-size parameter name"),
		UBox3DDentMapComponent::DentMapTexelSizeParameterName, FName(TEXT("Box3D_DentMapTexelSize")));
	TestTrue(TEXT("mesh materials are wrapped in dynamic instances"),
		!DentMap->GetMaterialInstances().IsEmpty());
	for (UMaterialInstanceDynamic* Instance : DentMap->GetMaterialInstances())
	{
		TestTrue(TEXT("active dent map is published to each shared material instance"),
			Instance->K2_GetTextureParameterValue(UBox3DDentMapComponent::DentMapParameterName)
				== DentMap->GetDentMap());
	}

	DentMap->ClearDentMap();
	TestEqual(TEXT("clear restores deterministic active side"), DentMap->GetDentMap(), Initial);
	TestEqual(TEXT("clear resets stamp count"), DentMap->GetDentStampCount(), 0);
	TestEqual(TEXT("dent map leaves collision mode unchanged"), Mesh->GetCollisionEnabled(), CollisionBefore);
	TestTrue(TEXT("dent map leaves collision source unchanged"),
		Mesh->GetStaticMesh()->GetBodySetup() == BodySetupBefore);
#if !UE_BUILD_SHIPPING
	TestNotNull(TEXT("visual dent test command is registered"),
		IConsoleManager::Get().FindConsoleObject(TEXT("box3d.DentTest")));
#endif
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

	int32 FragmentIndex = INDEX_NONE;
	for (int32 Index = 0; Index < Actor->GetFragmentCount() && FragmentIndex == INDEX_NONE; ++Index)
	{
		if (Actor->GetFragmentRenderState(Index) != EBox3DFragmentRenderState::None)
		{
			FragmentIndex = Index;
		}
	}
	const TArray<FVector>* Vertices = Actor->GetFragmentRenderVertices(FragmentIndex);
	if (!TestNotNull(TEXT("actor has render geometry"), Vertices) || Vertices->IsEmpty())
	{
		return false;
	}
	// Asleep dynamic rubble: the fragment draws through its own component, still
	// at the spawn pose, so body-local + centroid is actor space.
	UStaticMeshComponent* Chip = Actor->GetFragmentComponent(FragmentIndex);
	if (!TestNotNull(TEXT("fragment draws through its own component"), Chip))
	{
		return false;
	}
	UStaticMesh* MeshBefore = Chip->GetStaticMesh();
	const FVector Before = (*Vertices)[0];
	const FVector Centroid = Actor->GetFragments()[FragmentIndex].Centroid;
	const uint32 CollisionGeometryHash = Box3D::Fracture::FractureLayoutHash(Actor->GetFragments());
	TestTrue(TEXT("fragment primitive starts without collision"), Chip->GetCollisionEnabled() == ECollisionEnabled::NoCollision);

	const FVector WorldImpact = Actor->GetActorTransform().TransformPosition(Before + Centroid);
	const FVector WorldNormal = Actor->GetActorTransform().TransformVectorNoScale(FVector::UpVector);
	TestTrue(TEXT("dent displaces at least the struck render vertex"),
		Actor->ApplyVertexDent(WorldImpact, WorldNormal, 0.25f, 3.0f, 0.0f) > 0);

	Vertices = Actor->GetFragmentRenderVertices(FragmentIndex);
	if (!TestNotNull(TEXT("render geometry remains available after update"), Vertices))
	{
		return false;
	}
	const FVector After = (*Vertices)[0];
	TestTrue(TEXT("struck vertex moves inward by the clamped depth"),
		After.Equals(Before - FVector::UpVector * 3.0, 1.0e-4));
	TestTrue(TEXT("dent rebuilt the chip mesh"), Chip->GetStaticMesh() != nullptr && Chip->GetStaticMesh() != MeshBefore);
	TestTrue(TEXT("render update leaves collision off"), Chip->GetCollisionEnabled() == ECollisionEnabled::NoCollision);
	TestEqual(TEXT("render dent leaves fragment hull source geometry untouched"),
		Box3D::Fracture::FractureLayoutHash(Actor->GetFragments()), CollisionGeometryHash);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
