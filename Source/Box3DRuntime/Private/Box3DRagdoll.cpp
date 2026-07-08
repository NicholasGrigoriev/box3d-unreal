#include "Box3DRagdoll.h"

#include "Box3DConversion.h"
#include "Box3DCooking.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Animation/AnimNodeBase.h"
#include "Components/SkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "ReferenceSkeleton.h"
#include "box3d/box3d.h"

static TAutoConsoleVariable<bool> CVarBox3DDebugRagdoll(
	TEXT("box3d.DebugRagdoll"), false,
	TEXT("Draw ragdoll bodies with bone names, per-body masses and joint links."));

namespace
{
	// Unity builds merge anonymous namespaces across the module's cpps, so this
	// cannot share UBox3DBodyComponent.cpp's ForceScale name.
	constexpr float RagdollImpulseScale = 0.01f; // kg*cm/s -> kg*m/s

	void ScaleMassData(b3MassData& MassData, float Factor)
	{
		MassData.mass *= Factor;
		MassData.inertia.cx = b3Vec3{ MassData.inertia.cx.x * Factor, MassData.inertia.cx.y * Factor, MassData.inertia.cx.z * Factor };
		MassData.inertia.cy = b3Vec3{ MassData.inertia.cy.x * Factor, MassData.inertia.cy.y * Factor, MassData.inertia.cy.z * Factor };
		MassData.inertia.cz = b3Vec3{ MassData.inertia.cz.x * Factor, MassData.inertia.cz.y * Factor, MassData.inertia.cz.z * Factor };
	}

	/// UE constraint frames put the twist axis on X; box3d puts cone and twist on
	/// the frame Z. Cyclic re-basis (Xb3=Yue, Yb3=Zue, Zb3=Xue) keeps handedness.
	b3Transform MakeJointFrame(const FTransform& UEFrame, const FVector& Scale)
	{
		const FMatrix M = FRotationMatrix::Make(UEFrame.GetRotation());
		const FMatrix Rebased(M.GetUnitAxis(EAxis::Y), M.GetUnitAxis(EAxis::Z), M.GetUnitAxis(EAxis::X),
			FVector::ZeroVector);
		return b3Transform{ Box3D::ToB3(UEFrame.GetTranslation() * Scale), Box3D::ToB3(Rebased.ToQuat()) };
	}

	/// Effective half-angle of one swing axis in degrees: Free is unlimited,
	/// Locked contributes nothing.
	float EffectiveSwingDeg(EAngularConstraintMotion Motion, float LimitDeg)
	{
		switch (Motion)
		{
		case ACM_Free: return 180.0f;
		case ACM_Locked: return 0.0f;
		default: return LimitDeg;
		}
	}
}

int32 Box3D::BuildRagdoll(b3WorldId WorldId, const FReferenceSkeleton& RefSkeleton,
	const UPhysicsAsset& PhysAsset, TFunctionRef<FTransform(int32)> BoneWorldTransform,
	const FVector& MeshScale, const FRagdollBuildParams& Params,
	const UPrimitiveComponent* MassContext,
	TArray<FRagdollBone>& OutBones, TArray<b3JointId>& OutJoints)
{
	OutBones.Reset();
	OutJoints.Reset();
	if (!b3World_IsValid(WorldId))
	{
		return 0;
	}

	const int32 GroupIndex = Params.Filter.GroupIndex != 0
		? Params.Filter.GroupIndex
		: AllocateSelfCollisionGroup();

	TMap<FName, int32> BoneNameToSlot;
	for (const USkeletalBodySetup* BodySetup : PhysAsset.SkeletalBodySetups)
	{
		const int32 BoneIndex = BodySetup ? RefSkeleton.FindBoneIndex(BodySetup->BoneName) : INDEX_NONE;
		if (BoneIndex == INDEX_NONE)
		{
			continue;
		}

		const FTransform BoneWorld = BoneWorldTransform(BoneIndex);

		b3BodyDef BodyDef = b3DefaultBodyDef();
		BodyDef.type = b3_dynamicBody;
		BodyDef.position = ToB3Pos(BoneWorld.GetLocation());
		BodyDef.rotation = ToB3(BoneWorld.GetRotation());
		BodyDef.linearDamping = Params.LinearDamping;
		BodyDef.angularDamping = Params.AngularDamping;
		BodyDef.name = "Box3DRagdoll";
		// userData stays null — see the raw-body rule in the header.
		const b3BodyId BodyId = b3CreateBody(WorldId, &BodyDef);

		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		ShapeDef.density = 1000.0f; // placeholder; mass is overridden below
		ShapeDef.baseMaterial.friction = Params.Friction;
		ShapeDef.filter.categoryBits = ToB3Bits(Params.Filter.CategoryBits);
		ShapeDef.filter.maskBits = ToB3Bits(Params.Filter.MaskBits);
		ShapeDef.filter.groupIndex = GroupIndex;

		if (CreateShapesFromBodySetup(BodyId, ShapeDef, *BodySetup, MeshScale) == 0)
		{
			b3DestroyBody(BodyId);
			continue;
		}

		// Keep box3d's volume-derived inertia shape and center of mass, but match
		// the mass the asset authors: the per-body Mass (kg) override on the
		// setup's DefaultInstance wins. CalculateMass(Component) can NOT be
		// trusted for overrides — on a mesh without live per-bone physics
		// instances (query-only corpses) it silently falls back to the
		// component's body instance and computes volume * density.
		const FBodyInstance& AssetInstance = BodySetup->DefaultInstance;
		const float UEMass = (AssetInstance.bOverrideMass
			? AssetInstance.GetMassOverride()
			: BodySetup->CalculateMass(MassContext)) * Params.MassScale;
		b3MassData MassData = b3Body_GetMassData(BodyId);
		if (UEMass > UE_KINDA_SMALL_NUMBER && MassData.mass > UE_KINDA_SMALL_NUMBER)
		{
			ScaleMassData(MassData, UEMass / MassData.mass);
			b3Body_SetMassData(BodyId, MassData);
		}
		UE_LOG(LogBox3D, Verbose, TEXT("Ragdoll body %s: %.1f kg (%s)"),
			*BodySetup->BoneName.ToString(), b3Body_GetMass(BodyId),
			AssetInstance.bOverrideMass ? TEXT("asset override") : TEXT("computed"));

		BoneNameToSlot.Add(BodySetup->BoneName, OutBones.Num());
		FRagdollBone& Bone = OutBones.AddDefaulted_GetRef();
		Bone.BoneIndex = BoneIndex;
		Bone.BodyId = BodyId;
		Bone.P0 = Bone.P1 = BoneWorld.GetLocation();
		Bone.Q0 = Bone.Q1 = BoneWorld.GetRotation();
	}

	// Rein in extreme mass ratios: a 300 kg torso against a 2 kg hand makes the
	// solver chain jitter and never sleep. Same discipline UE recommends for
	// asset masses, applied automatically.
	if (Params.MinMassFraction > 0.0f)
	{
		float MaxMass = 0.0f;
		for (const FRagdollBone& Bone : OutBones)
		{
			MaxMass = FMath::Max(MaxMass, b3Body_GetMass(Bone.BodyId));
		}
		const float MinMass = MaxMass * Params.MinMassFraction;
		for (const FRagdollBone& Bone : OutBones)
		{
			b3MassData MassData = b3Body_GetMassData(Bone.BodyId);
			if (MassData.mass > UE_KINDA_SMALL_NUMBER && MassData.mass < MinMass)
			{
				ScaleMassData(MassData, MinMass / MassData.mass);
				b3Body_SetMassData(Bone.BodyId, MassData);
			}
		}
	}

	for (const UPhysicsConstraintTemplate* Template : PhysAsset.ConstraintSetup)
	{
		if (Template == nullptr)
		{
			continue;
		}
		const FConstraintInstance& CI = Template->DefaultInstance;
		// Bone1 is the child body, Bone2 the parent — box3d's cone lives on frame
		// A (parent) and twist on frame B (child), matching UE's convention.
		const int32* ChildSlot = BoneNameToSlot.Find(CI.ConstraintBone1);
		const int32* ParentSlot = BoneNameToSlot.Find(CI.ConstraintBone2);
		if (ChildSlot == nullptr || ParentSlot == nullptr)
		{
			continue;
		}

		b3SphericalJointDef Def = b3DefaultSphericalJointDef();
		Def.base.bodyIdA = OutBones[*ParentSlot].BodyId;
		Def.base.bodyIdB = OutBones[*ChildSlot].BodyId;
		Def.base.localFrameA = MakeJointFrame(CI.GetRefFrame(EConstraintFrame::Frame2), MeshScale);
		Def.base.localFrameB = MakeJointFrame(CI.GetRefFrame(EConstraintFrame::Frame1), MeshScale);
		Def.base.collideConnected = false;
		if (Params.ConstraintHertz > 0.0f)
		{
			Def.base.constraintHertz = Params.ConstraintHertz;
			Def.base.constraintDampingRatio = FMath::Max(Params.ConstraintDampingRatio, 0.0f);
		}

		if (Params.bUseConstraintLimits)
		{
			// box3d's cone is symmetric; approximate UE's swing1/swing2 pair with
			// the wider of the two so limbs never end up tighter than authored.
			const float Swing1 = EffectiveSwingDeg(CI.GetAngularSwing1Motion(), CI.GetAngularSwing1Limit());
			const float Swing2 = EffectiveSwingDeg(CI.GetAngularSwing2Motion(), CI.GetAngularSwing2Limit());
			if (Swing1 < 179.0f || Swing2 < 179.0f)
			{
				Def.enableConeLimit = true;
				Def.coneAngle = FMath::Clamp(FMath::DegreesToRadians(FMath::Max(Swing1, Swing2)), 0.0f, PI);
			}

			const EAngularConstraintMotion TwistMotion = CI.GetAngularTwistMotion();
			if (TwistMotion != ACM_Free)
			{
				const float TwistRad = TwistMotion == ACM_Locked
					? 0.0f
					: FMath::Clamp(FMath::DegreesToRadians(CI.GetAngularTwistLimit()), 0.0f, 0.98f * PI);
				Def.enableTwistLimit = true;
				Def.lowerTwistAngle = -TwistRad;
				Def.upperTwistAngle = TwistRad;
			}
		}

		OutJoints.Add(b3CreateSphericalJoint(WorldId, &Def));
	}

	return OutBones.Num();
}

UBox3DRagdollComponent::UBox3DRagdollComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	Filter.CategoryBits = 1 << static_cast<int32>(EBox3DChannel::Debris);
}

bool UBox3DRagdollComponent::StartRagdoll(USkeletalMeshComponent* Mesh, FVector InitialVelocity)
{
	if (IsRagdollActive())
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: StartRagdoll called while already active"), *GetPathName());
		return false;
	}

	UBox3DWorldSubsystem* WorldSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	USkeletalMesh* MeshAsset = Mesh ? Mesh->GetSkeletalMeshAsset() : nullptr;
	UPhysicsAsset* PhysAsset = Mesh ? Mesh->GetPhysicsAsset() : nullptr;
	if (WorldSubsystem == nullptr || !b3World_IsValid(WorldSubsystem->GetBox3DWorldId())
		|| MeshAsset == nullptr || PhysAsset == nullptr)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: StartRagdoll needs a Box3D world and a mesh with a physics asset (%s)"),
			*GetPathName(), *GetNameSafe(Mesh));
		return false;
	}

	const FReferenceSkeleton& RefSkeleton = MeshAsset->GetRefSkeleton();

	// Death pose. Bone-space transforms can be empty on a never-ticked mesh;
	// fall back to the reference pose.
	BaseLocalPose = Mesh->GetBoneSpaceTransforms();
	if (BaseLocalPose.Num() != RefSkeleton.GetNum())
	{
		BaseLocalPose = RefSkeleton.GetRefBonePose();
	}
	BaseComponentSpacePose.SetNum(RefSkeleton.GetNum());
	{
		const TArray<FTransform>& ComponentSpace = Mesh->GetComponentSpaceTransforms();
		for (int32 BoneIndex = 0; BoneIndex < RefSkeleton.GetNum(); ++BoneIndex)
		{
			if (ComponentSpace.IsValidIndex(BoneIndex))
			{
				BaseComponentSpacePose[BoneIndex] = ComponentSpace[BoneIndex];
			}
			else
			{
				const int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);
				BaseComponentSpacePose[BoneIndex] = ParentIndex != INDEX_NONE
					? BaseLocalPose[BoneIndex] * BaseComponentSpacePose[ParentIndex]
					: BaseLocalPose[BoneIndex];
			}
		}
	}

	Box3D::FRagdollBuildParams Params;
	Params.Filter = Filter;
	Params.LinearDamping = LinearDamping;
	Params.AngularDamping = AngularDamping;
	Params.Friction = Friction;
	Params.MassScale = MassScale;
	Params.MinMassFraction = MinBodyMassFraction;
	Params.bUseConstraintLimits = bUseConstraintLimits;
	Params.ConstraintHertz = ConstraintHertz;
	Params.ConstraintDampingRatio = ConstraintDampingRatio;

	Box3D::BuildRagdoll(WorldSubsystem->GetBox3DWorldId(), RefSkeleton, *PhysAsset,
		[Mesh](int32 BoneIndex) { return Mesh->GetBoneTransform(BoneIndex); },
		Mesh->GetComponentTransform().GetScale3D().GetAbs(), Params, Mesh, Bones, Joints);
	if (Bones.IsEmpty())
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: no ragdoll bodies could be built from %s"),
			*GetPathName(), *GetNameSafe(PhysAsset));
		return false;
	}

	// Parent-first order so Bones[0] is the root-most body (mesh re-centering).
	Bones.Sort([](const Box3D::FRagdollBone& A, const Box3D::FRagdollBone& B) { return A.BoneIndex < B.BoneIndex; });
	BoneToBody.Init(INDEX_NONE, RefSkeleton.GetNum());
	for (int32 Slot = 0; Slot < Bones.Num(); ++Slot)
	{
		BoneToBody[Bones[Slot].BoneIndex] = Slot;
		b3Body_SetLinearVelocity(Bones[Slot].BodyId, Box3D::ToB3(InitialVelocity));
	}

	Subsystem = WorldSubsystem;
	TargetMesh = Mesh;
	LastStep = WorldSubsystem->GetStepCount();

	// The corpse must keep refreshing bones offscreen: the Chaos query bodies
	// (weapon traces) and the Box3D-driven pose both hang off anim updates.
	Mesh->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;
	Mesh->SetAnimInstanceClass(UBox3DRagdollAnimInstance::StaticClass());

	UE_LOG(LogBox3D, Log, TEXT("%s: ragdoll from %s — %d bodies, %d joints, total %.1f kg"),
		*GetPathName(), *GetNameSafe(PhysAsset), Bones.Num(), Joints.Num(), GetTotalMass());
	return true;
}

void UBox3DRagdollComponent::StopRagdoll()
{
	for (const b3JointId Joint : Joints)
	{
		if (b3Joint_IsValid(Joint))
		{
			b3DestroyJoint(Joint, /*wakeAttached*/ false);
		}
	}
	Joints.Reset();

	for (const Box3D::FRagdollBone& Bone : Bones)
	{
		if (b3Body_IsValid(Bone.BodyId))
		{
			b3DestroyBody(Bone.BodyId);
		}
	}
	Bones.Reset();
	BoneToBody.Reset();

	// Freeze the mesh in its last pose; the driver instance has nothing to read.
	if (USkeletalMeshComponent* Mesh = TargetMesh.Get())
	{
		Mesh->SetAnimInstanceClass(nullptr);
	}
	TargetMesh.Reset();
}

void UBox3DRagdollComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopRagdoll();
	Super::EndPlay(EndPlayReason);
}

void UBox3DRagdollComponent::SampleBodies()
{
	const UBox3DWorldSubsystem* WorldSubsystem = Subsystem.Get();
	if (WorldSubsystem == nullptr)
	{
		return;
	}
	const uint64 Step = WorldSubsystem->GetStepCount();
	if (Step == LastStep)
	{
		return;
	}
	for (Box3D::FRagdollBone& Bone : Bones)
	{
		if (!b3Body_IsValid(Bone.BodyId))
		{
			continue;
		}
		const b3WorldTransform T = b3Body_GetTransform(Bone.BodyId);
		Bone.P0 = Bone.P1;
		Bone.Q0 = Bone.Q1;
		Bone.P1 = Box3D::ToUEPos(T.p);
		Bone.Q1 = Box3D::ToUE(T.q);
	}
	LastStep = Step;
}

void UBox3DRagdollComponent::BuildLocalPose(TArray<FTransform>& OutLocalPose)
{
	USkeletalMeshComponent* Mesh = TargetMesh.Get();
	const UBox3DWorldSubsystem* WorldSubsystem = Subsystem.Get();
	if (Mesh == nullptr || WorldSubsystem == nullptr || Bones.IsEmpty())
	{
		return;
	}

	SampleBodies();
	const float Alpha = WorldSubsystem->GetFixedStepAlpha();

	auto BodyWorld = [Alpha](const Box3D::FRagdollBone& Bone)
	{
		return FTransform(FQuat::Slerp(Bone.Q0, Bone.Q1, Alpha).GetNormalized(),
			FMath::Lerp(Bone.P0, Bone.P1, Alpha));
	};

	// Re-center the component under the root-most body so skeletal bounds follow
	// the corpse instead of staying at the death spot.
	const Box3D::FRagdollBone& RootBone = Bones[0];
	const FTransform RootWorld = BodyWorld(RootBone);
	{
		const FTransform& CompToWorld = Mesh->GetComponentTransform();
		const FVector NewLocation = RootWorld.GetLocation()
			- CompToWorld.TransformVector(BaseComponentSpacePose[RootBone.BoneIndex].GetTranslation());
		if (!NewLocation.Equals(CompToWorld.GetLocation(), 0.01))
		{
			Mesh->SetWorldLocation(NewLocation, /*bSweep*/ false, nullptr, ETeleportType::TeleportPhysics);
		}
	}

	const FReferenceSkeleton& RefSkeleton = Mesh->GetSkeletalMeshAsset()->GetRefSkeleton();
	const FTransform WorldToComp = Mesh->GetComponentTransform().Inverse();
	const int32 NumBones = FMath::Min(RefSkeleton.GetNum(), BaseLocalPose.Num());

	TArray<FTransform> ComponentSpace;
	ComponentSpace.SetNumUninitialized(NumBones);
	OutLocalPose.SetNumUninitialized(NumBones);

	for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
	{
		const int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);
		const int32 Slot = BoneToBody.IsValidIndex(BoneIndex) ? BoneToBody[BoneIndex] : INDEX_NONE;

		FTransform CS;
		if (Slot != INDEX_NONE)
		{
			CS = BodyWorld(Bones[Slot]) * WorldToComp;
			CS.SetScale3D(BaseComponentSpacePose[BoneIndex].GetScale3D());
		}
		else
		{
			CS = ParentIndex != INDEX_NONE ? BaseLocalPose[BoneIndex] * ComponentSpace[ParentIndex]
										   : BaseLocalPose[BoneIndex];
		}
		ComponentSpace[BoneIndex] = CS;
		OutLocalPose[BoneIndex] = ParentIndex != INDEX_NONE
			? CS.GetRelativeTransform(ComponentSpace[ParentIndex])
			: CS;
	}

#if ENABLE_DRAW_DEBUG
	if (CVarBox3DDebugRagdoll.GetValueOnGameThread())
	{
		UWorld* World = GetWorld();
		for (const Box3D::FRagdollBone& Bone : Bones)
		{
			const FVector Location = FMath::Lerp(Bone.P0, Bone.P1, Alpha);
			DrawDebugString(World, Location + FVector(0, 0, 6),
				FString::Printf(TEXT("%s %.1f kg"),
					*RefSkeleton.GetBoneName(Bone.BoneIndex).ToString(),
					b3Body_GetMass(Bone.BodyId)),
				nullptr, b3Body_IsAwake(Bone.BodyId) ? FColor::Yellow : FColor::Cyan, 0.0f, true);
			const int32 ParentSlot = FindBodySlotForBone(RefSkeleton.GetParentIndex(Bone.BoneIndex));
			if (ParentSlot != INDEX_NONE)
			{
				DrawDebugLine(World, Location,
					FMath::Lerp(Bones[ParentSlot].P0, Bones[ParentSlot].P1, Alpha),
					FColor::Orange, false, 0.0f, SDPG_Foreground, 0.5f);
			}
		}
		DrawDebugString(World, RootWorld.GetLocation() + FVector(0, 0, 40),
			FString::Printf(TEXT("total %.1f kg"), GetTotalMass()),
			nullptr, FColor::Green, 0.0f, true);
	}
#endif
}

int32 UBox3DRagdollComponent::FindBodySlotForBone(int32 BoneIndex) const
{
	const USkeletalMeshComponent* Mesh = TargetMesh.Get();
	const USkeletalMesh* MeshAsset = Mesh ? Mesh->GetSkeletalMeshAsset() : nullptr;
	if (MeshAsset == nullptr)
	{
		return INDEX_NONE;
	}
	const FReferenceSkeleton& RefSkeleton = MeshAsset->GetRefSkeleton();
	while (BoneIndex != INDEX_NONE)
	{
		if (BoneToBody.IsValidIndex(BoneIndex) && BoneToBody[BoneIndex] != INDEX_NONE)
		{
			return BoneToBody[BoneIndex];
		}
		BoneIndex = RefSkeleton.GetParentIndex(BoneIndex);
	}
	return INDEX_NONE;
}

void UBox3DRagdollComponent::AddImpulse(FVector Impulse)
{
	const float TotalMass = GetTotalMass();
	if (TotalMass <= UE_KINDA_SMALL_NUMBER)
	{
		return;
	}
	for (const Box3D::FRagdollBone& Bone : Bones)
	{
		if (b3Body_IsValid(Bone.BodyId))
		{
			const float Share = b3Body_GetMass(Bone.BodyId) / TotalMass;
			b3Body_ApplyLinearImpulseToCenter(Bone.BodyId, Box3D::ToB3Dir(Impulse * Share * RagdollImpulseScale), /*wake*/ true);
		}
	}
}

void UBox3DRagdollComponent::AddImpulseAtLocation(FVector Impulse, FVector Location, FName BoneName)
{
	int32 Slot = INDEX_NONE;
	if (BoneName != NAME_None)
	{
		const USkeletalMeshComponent* Mesh = TargetMesh.Get();
		const USkeletalMesh* MeshAsset = Mesh ? Mesh->GetSkeletalMeshAsset() : nullptr;
		if (MeshAsset != nullptr)
		{
			Slot = FindBodySlotForBone(MeshAsset->GetRefSkeleton().FindBoneIndex(BoneName));
		}
	}
	if (Slot == INDEX_NONE)
	{
		float BestDistSq = TNumericLimits<float>::Max();
		for (int32 Index = 0; Index < Bones.Num(); ++Index)
		{
			const float DistSq = FVector::DistSquared(Bones[Index].P1, Location);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				Slot = Index;
			}
		}
	}
	if (Slot != INDEX_NONE && b3Body_IsValid(Bones[Slot].BodyId))
	{
		b3Body_ApplyLinearImpulse(Bones[Slot].BodyId, Box3D::ToB3Dir(Impulse * RagdollImpulseScale),
			Box3D::ToB3Pos(Location), /*wake*/ true);
	}
}

float UBox3DRagdollComponent::GetTotalMass() const
{
	float TotalMass = 0.0f;
	for (const Box3D::FRagdollBone& Bone : Bones)
	{
		if (b3Body_IsValid(Bone.BodyId))
		{
			TotalMass += b3Body_GetMass(Bone.BodyId);
		}
	}
	return TotalMass;
}

FAnimInstanceProxy* UBox3DRagdollAnimInstance::CreateAnimInstanceProxy()
{
	return new FBox3DRagdollAnimInstanceProxy(this);
}

void FBox3DRagdollAnimInstanceProxy::PreUpdate(UAnimInstance* InAnimInstance, float DeltaSeconds)
{
	FAnimInstanceProxy::PreUpdate(InAnimInstance, DeltaSeconds);

	const AActor* Owner = InAnimInstance->GetOwningActor();
	UBox3DRagdollComponent* Ragdoll = Owner ? Owner->FindComponentByClass<UBox3DRagdollComponent>() : nullptr;
	if (Ragdoll && Ragdoll->IsRagdollActive() && Ragdoll->GetTargetMesh() == InAnimInstance->GetOwningComponent())
	{
		Ragdoll->BuildLocalPose(LocalPose);
	}
}

bool FBox3DRagdollAnimInstanceProxy::Evaluate(FPoseContext& Output)
{
	if (LocalPose.IsEmpty())
	{
		Output.ResetToRefPose();
		return true;
	}

	const FBoneContainer& BoneContainer = Output.Pose.GetBoneContainer();
	for (FCompactPoseBoneIndex BoneIndex : Output.Pose.ForEachBoneIndex())
	{
		const int32 MeshIndex = BoneContainer.MakeMeshPoseIndex(BoneIndex).GetInt();
		if (LocalPose.IsValidIndex(MeshIndex))
		{
			Output.Pose[BoneIndex] = LocalPose[MeshIndex];
		}
	}
	Output.Pose.NormalizeRotations();
	return true;
}
