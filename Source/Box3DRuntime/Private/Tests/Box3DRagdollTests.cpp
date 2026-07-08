// Tests for the ragdoll builder (bodies/joints replicated from a UPhysicsAsset)
// and the kinematic ground-weight transfer on UBox3DBodyComponent.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DConversion.h"
#include "Box3DRagdoll.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "ReferenceSkeleton.h"
#include "Tests/Box3DTestHelpers.h"
#include "UObject/UObjectIterator.h"

namespace
{
	FVector BodyLocation(const Box3D::FRagdollBone& Bone)
	{
		return Box3D::ToUEPos(b3Body_GetTransform(Bone.BodyId).p);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DRagdollBuildTest,
	"Box3DUnreal.Ragdoll.BuildFromPhysicsAsset", BOX3D_TEST_FLAGS)
bool FBox3DRagdollBuildTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Three-bone chain along Z; only spine and head carry physics bodies, the
	// way real assets skip the root.
	FReferenceSkeleton RefSkeleton;
	{
		FReferenceSkeletonModifier Modifier(RefSkeleton, nullptr);
		Modifier.Add(FMeshBoneInfo(FName(TEXT("root")), TEXT("root"), INDEX_NONE), FTransform::Identity);
		Modifier.Add(FMeshBoneInfo(FName(TEXT("spine")), TEXT("spine"), 0), FTransform(FVector(0, 0, 50)));
		Modifier.Add(FMeshBoneInfo(FName(TEXT("head")), TEXT("head"), 1), FTransform(FVector(0, 0, 50)));
	}

	UPhysicsAsset* PhysAsset = NewObject<UPhysicsAsset>(GetTransientPackage());
	auto AddBody = [PhysAsset](const FName BoneName) -> USkeletalBodySetup*
	{
		USkeletalBodySetup* Setup = NewObject<USkeletalBodySetup>(PhysAsset);
		Setup->BoneName = BoneName;
		Setup->AggGeom.SphylElems.Add(FKSphylElem(10.0f, 30.0f));
		PhysAsset->SkeletalBodySetups.Add(Setup);
		return Setup;
	};
	AddBody(TEXT("spine"));
	// Asset-authored Mass (kg) override — the way real assets tune ragdoll weight.
	AddBody(TEXT("head"))->DefaultInstance.SetMassOverride(200.0f, true);
	AddBody(TEXT("missing_bone")); // must be skipped, not crash

	UPhysicsConstraintTemplate* Constraint = NewObject<UPhysicsConstraintTemplate>(PhysAsset);
	Constraint->DefaultInstance.ConstraintBone1 = TEXT("head");  // child
	Constraint->DefaultInstance.ConstraintBone2 = TEXT("spine"); // parent
	// Pivot at the head bone origin: identity in child space, +50 Z in the
	// parent's (like a real asset authors it — default zero frames would weld
	// the two bone origins together).
	Constraint->DefaultInstance.SetRefPosition(EConstraintFrame::Frame2, FVector(0, 0, 50));
	Constraint->DefaultInstance.SetAngularSwing1Limit(ACM_Limited, 30.0f);
	Constraint->DefaultInstance.SetAngularSwing2Limit(ACM_Limited, 30.0f);
	Constraint->DefaultInstance.SetAngularTwistLimit(ACM_Limited, 15.0f);
	PhysAsset->ConstraintSetup.Add(Constraint);

	const auto BoneWorld = [](int32 BoneIndex)
	{
		return FTransform(FVector(0, 0, 500 + 50.0 * BoneIndex));
	};

	TArray<Box3D::FRagdollBone> Bones;
	TArray<b3JointId> Joints;
	const Box3D::FRagdollBuildParams Params;
	const int32 Built = Box3D::BuildRagdoll(Test.B3World(), RefSkeleton, *PhysAsset, BoneWorld,
		FVector::OneVector, Params, nullptr, Bones, Joints);

	TestEqual(TEXT("one body per resolvable body setup"), Built, 2);
	TestEqual(TEXT("one joint per resolvable constraint"), Joints.Num(), 1);
	if (Built != 2)
	{
		return false;
	}

	// Bones come back in physics-asset order: spine (slot 0), head (slot 1).
	TestTrue(TEXT("bodies valid"), b3Body_IsValid(Bones[0].BodyId) && b3Body_IsValid(Bones[1].BodyId));
	TestEqual(TEXT("head inherits the asset's Mass (kg) override"),
		b3Body_GetMass(Bones[1].BodyId), 200.0f, 0.5f);
	// UE's power-curve mass for the small sphyl lands under the 5% ratio clamp
	// (200 kg * 0.05 = 10 kg), so the spine must be lifted exactly to it.
	const float SpineMass = b3Body_GetMass(Bones[0].BodyId);
	TestTrue(FString::Printf(TEXT("spine ratio-clamped to >= 5%% of heaviest (%f kg)"), SpineMass),
		SpineMass >= 200.0f * Params.MinMassFraction - 0.01f && SpineMass < 100.0f);
	TestTrue(TEXT("joint valid"), b3Joint_IsValid(Joints[0]));

	const float StartZ0 = BodyLocation(Bones[0]).Z;
	Test.Step(30);

	TestTrue(TEXT("ragdoll falls under gravity"), BodyLocation(Bones[0]).Z < StartZ0 - 50.0f);
	const float BoneDistance = FVector::Dist(BodyLocation(Bones[0]), BodyLocation(Bones[1]));
	TestEqual(TEXT("spherical joint keeps the chain together"), BoneDistance, 50.0f, 15.0f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DGroundWeightTest,
	"Box3DUnreal.Body.GroundWeight", BOX3D_TEST_FLAGS)
bool FBox3DGroundWeightTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	// Gravity-free dynamic boxes isolate the weight force: any downward velocity
	// they gain can only come from the pawn pressing on them.
	const auto SpawnPlank = [&Test](float X)
	{
		return Box3DTest::SpawnBody(Test.World, FVector(X, 0, 100), [](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Dynamic;
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(50.0);
			Body.GravityScale = 0.0f;
		});
	};
	const auto SpawnPawn = [&Test](float X, float WeightKg, float GapCm)
	{
		// The probe (bottom + slack) must bridge GapCm above the plank top.
		return Box3DTest::SpawnBody(Test.World, FVector(X, 0, 150 + 90 + GapCm), [WeightKg](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Kinematic;
			Body.ShapeType = EBox3DShapeType::Capsule;
			Body.CapsuleRadius = 30.0f;
			Body.CapsuleHalfHeight = 90.0f;
			Body.GroundWeightKg = WeightKg;
			Body.Filter.CategoryBits = 1 << static_cast<int32>(EBox3DChannel::Pawn);
			Body.Filter.MaskBits = 1 << static_cast<int32>(EBox3DChannel::WorldDynamic);
		});
	};

	UBox3DBodyComponent* Plank = SpawnPlank(0.0f);
	SpawnPawn(0.0f, 80.0f, 5.0f);

	UBox3DBodyComponent* ControlPlank = SpawnPlank(500.0f);
	SpawnPawn(500.0f, 0.0f, 5.0f);

	// The real standing case: capsule bottom exactly touching the plank top —
	// the contact with the infinite-mass kinematic must not eat the weight force.
	UBox3DBodyComponent* TouchedPlank = SpawnPlank(1000.0f);
	SpawnPawn(1000.0f, 80.0f, 0.0f);

	Test.Step(10);

	TestTrue(FString::Printf(TEXT("weighted pawn presses its plank down (vz=%f)"), Plank->GetLinearVelocity().Z),
		Plank->GetLinearVelocity().Z < -1.0f);
	TestEqual(TEXT("weightless pawn leaves its plank alone"), ControlPlank->GetLinearVelocity().Z, 0.0, 0.01);
	TestTrue(FString::Printf(TEXT("weight still presses through standing contact (vz=%f)"), TouchedPlank->GetLinearVelocity().Z),
		TouchedPlank->GetLinearVelocity().Z < -1.0f);

	// Weight stops when the body is disabled (dead pawns must not keep pressing).
	Plank->SetLinearVelocity(FVector::ZeroVector);
	for (TObjectIterator<UBox3DBodyComponent> It; It; ++It)
	{
		if (It->GetWorld() == Test.World && It->GroundWeightKg > 0.0f)
		{
			It->SetBodyEnabled(false);
		}
	}
	Test.Step(5);
	TestEqual(TEXT("disabled pawn stops pressing"), Plank->GetLinearVelocity().Z, 0.0, 0.01);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DKinematicSpeedCapTest,
	"Box3DUnreal.Body.KinematicTargetSpeedCap", BOX3D_TEST_FLAGS)
bool FBox3DKinematicSpeedCapTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;

	const auto SpawnPawn = [&Test](float X, float MaxSpeed)
	{
		return Box3DTest::SpawnBody(Test.World, FVector(X, 0, 100), [MaxSpeed](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Kinematic;
			Body.ShapeType = EBox3DShapeType::Capsule;
			Body.CapsuleRadius = 30.0f;
			Body.CapsuleHalfHeight = 90.0f;
			Body.MaxKinematicTargetSpeed = MaxSpeed;
		});
	};
	UBox3DBodyComponent* Capped = SpawnPawn(0.0f, 500.0f);
	UBox3DBodyComponent* Uncapped = SpawnPawn(1000.0f, 0.0f);

	// Teleport both 400 cm in one tick: 24000 cm/s of implied velocity.
	Capped->SetWorldLocation(FVector(400, 0, 100));
	Uncapped->SetWorldLocation(FVector(1400, 0, 100));
	Test.Step(1);

	TestTrue(FString::Printf(TEXT("capped body snaps without velocity (%f cm/s)"), Capped->GetLinearVelocity().Size()),
		Capped->GetLinearVelocity().Size() < 50.0f);
	TestTrue(FString::Printf(TEXT("uncapped body carries teleport velocity (%f cm/s)"), Uncapped->GetLinearVelocity().Size()),
		Uncapped->GetLinearVelocity().Size() > 10000.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
