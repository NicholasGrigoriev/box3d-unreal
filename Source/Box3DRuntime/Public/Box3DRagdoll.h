#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "Animation/AnimInstanceProxy.h"
#include "Components/ActorComponent.h"
#include "Box3DTypes.h"
#include "box3d/id.h"
#include "Box3DRagdoll.generated.h"

class UBox3DWorldSubsystem;
class UPhysicsAsset;
class UPrimitiveComponent;
class USkeletalMeshComponent;
struct FReferenceSkeleton;

namespace Box3D
{
	/// One ragdoll body bound to a skeleton bone, with its interpolation segment
	/// (same scheme as the rope/cloth actors: pose at the previous and latest
	/// fixed step, lerped by the subsystem's fixed-step alpha).
	struct FRagdollBone
	{
		int32 BoneIndex = INDEX_NONE;
		b3BodyId BodyId = {};
		FVector P0 = FVector::ZeroVector;
		FVector P1 = FVector::ZeroVector;
		FQuat Q0 = FQuat::Identity;
		FQuat Q1 = FQuat::Identity;
	};

	struct FRagdollBuildParams
	{
		/// Filter for every ragdoll shape. Give GroupIndex 0 to have a fresh
		/// negative self-collision group allocated per ragdoll.
		FBox3DFilter Filter;

		float LinearDamping = 0.05f;
		float AngularDamping = 1.0f;
		float Friction = 0.7f;

		/// Scales the UE-computed per-body masses.
		float MassScale = 1.0f;

		/// Map the physics asset's cone/twist limits onto the joints. Off: free
		/// spherical joints (position-only, floppy).
		bool bUseConstraintLimits = true;
	};

	/// Replicate a UPhysicsAsset in the Box3D world: one dynamic body per skeletal
	/// body setup (shapes via CreateShapesFromBodySetup, mass matched to UE's
	/// CalculateMass) and one spherical joint per constraint template. UE's
	/// asymmetric swing pair becomes box3d's symmetric cone (the larger of the two
	/// angles); twist maps directly. Bones missing from RefSkeleton are skipped.
	///
	/// BoneWorldTransform supplies the starting pose (scale ignored — pass the
	/// component scale through MeshScale for shape geometry instead). MassContext
	/// feeds UBodySetup::CalculateMass and may be null (unit scale).
	///
	/// Bodies carry no userData (raw-body rule: Box3D::ResolveComponent casts
	/// userData to UObject on every query hit). Returns the number of bodies.
	BOX3DRUNTIME_API int32 BuildRagdoll(b3WorldId WorldId, const FReferenceSkeleton& RefSkeleton,
		const UPhysicsAsset& PhysAsset, TFunctionRef<FTransform(int32 BoneIndex)> BoneWorldTransform,
		const FVector& MeshScale, const FRagdollBuildParams& Params,
		const UPrimitiveComponent* MassContext,
		TArray<FRagdollBone>& OutBones, TArray<b3JointId>& OutJoints);
}

/// Replaces the engine ragdoll with a Box3D one, built on the fly from the
/// skeletal mesh's physics asset.
///
/// StartRagdoll spawns raw Box3D bodies/joints at the mesh's current bone pose
/// and swaps the mesh's anim instance for UBox3DRagdollAnimInstance, which pulls
/// the simulated pose back onto the bones every frame (interpolated between
/// fixed steps). Bones without a physics body keep their death-pose local
/// transforms, exactly like an engine ragdoll.
///
/// The mesh component itself is re-centered under the root-most body each frame
/// so skeletal bounds stay honest when the corpse slides or gets blasted away.
/// Keep the mesh's Chaos collision in QueryOnly mode if weapon traces should
/// still hit the corpse — kinematic Chaos bodies follow the driven bones, and
/// hits can be forwarded here via AddImpulseAtLocation with the hit bone name.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DRagdollComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBox3DRagdollComponent();

	/// Filter for the ragdoll shapes. Defaults to Debris (explosions fling
	/// corpses, pawn proxies shove them). GroupIndex 0 auto-allocates a negative
	/// self-collision group so the ragdoll never fights itself.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Ragdoll")
	FBox3DFilter Filter;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Ragdoll", meta = (ClampMin = "0"))
	float LinearDamping = 0.05f;

	/// Ragdolls want noticeable angular damping; limbs spin unnaturally without it.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Ragdoll", meta = (ClampMin = "0"))
	float AngularDamping = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Ragdoll", meta = (ClampMin = "0"))
	float Friction = 0.7f;

	/// Scales the physics asset's per-body masses.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Ragdoll", meta = (ClampMin = "0.01"))
	float MassScale = 1.0f;

	/// Map the physics asset's cone/twist limits onto the joints.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Ragdoll")
	bool bUseConstraintLimits = true;

	/// Build the ragdoll from the mesh's physics asset at its current pose and
	/// take over the mesh's animation. Every body starts with InitialVelocity
	/// (cm/s) — pass the dying actor's velocity so momentum carries through.
	/// False if the mesh has no physics asset or no body could be built.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Ragdoll")
	bool StartRagdoll(USkeletalMeshComponent* Mesh, FVector InitialVelocity);

	/// Destroy the ragdoll bodies. The mesh keeps its last pose.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Ragdoll")
	void StopRagdoll();

	UFUNCTION(BlueprintPure, Category = "Box3D|Ragdoll")
	bool IsRagdollActive() const { return Bones.Num() > 0; }

	/// Impulse (kg*cm/s) distributed over all bodies by mass — a uniform kick.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Ragdoll")
	void AddImpulse(FVector Impulse);

	/// Impulse (kg*cm/s) at a world location (cm). BoneName picks the struck
	/// body (walking up the skeleton to the nearest simulated bone, the way
	/// engine ragdolls resolve hits); NAME_None falls back to the nearest body.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Ragdoll")
	void AddImpulseAtLocation(FVector Impulse, FVector Location, FName BoneName = NAME_None);

	/// Total ragdoll mass in kg.
	UFUNCTION(BlueprintPure, Category = "Box3D|Ragdoll")
	float GetTotalMass() const;

	int32 GetBodyCount() const { return Bones.Num(); }
	int32 GetJointCount() const { return Joints.Num(); }

	/// Interpolated full local-space pose for the anim instance (game thread).
	/// Also re-centers the mesh component under the root-most body.
	void BuildLocalPose(TArray<FTransform>& OutLocalPose);

	USkeletalMeshComponent* GetTargetMesh() const { return TargetMesh.Get(); }

	//~ UActorComponent
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/// Advance the per-body interpolation segments if fixed steps happened.
	void SampleBodies();

	/// Bones index for the nearest simulated bone at or above BoneIndex.
	int32 FindBodySlotForBone(int32 BoneIndex) const;

	TWeakObjectPtr<USkeletalMeshComponent> TargetMesh;
	TWeakObjectPtr<UBox3DWorldSubsystem> Subsystem;

	TArray<Box3D::FRagdollBone> Bones;
	TArray<b3JointId> Joints;

	/// Death pose captured at StartRagdoll: local-space transforms for bones
	/// without a body, component-space transforms for scale preservation and
	/// mesh re-centering.
	TArray<FTransform> BaseLocalPose;
	TArray<FTransform> BaseComponentSpacePose;

	/// Bone index -> Bones slot (INDEX_NONE for undriven bones).
	TArray<int32> BoneToBody;

	uint64 LastStep = 0;
};

/// Minimal native anim instance that outputs the pose UBox3DRagdollComponent
/// computes from the Box3D bodies. Installed by StartRagdoll; has no anim graph.
UCLASS(NotBlueprintable, Transient)
class BOX3DRUNTIME_API UBox3DRagdollAnimInstance : public UAnimInstance
{
	GENERATED_BODY()

protected:
	virtual FAnimInstanceProxy* CreateAnimInstanceProxy() override;
};

struct FBox3DRagdollAnimInstanceProxy : public FAnimInstanceProxy
{
	FBox3DRagdollAnimInstanceProxy() = default;
	explicit FBox3DRagdollAnimInstanceProxy(UAnimInstance* InInstance) : FAnimInstanceProxy(InInstance) {}

	//~ FAnimInstanceProxy
	virtual void PreUpdate(UAnimInstance* InAnimInstance, float DeltaSeconds) override;
	virtual bool Evaluate(FPoseContext& Output) override;

private:
	/// Full local-space pose, copied on the game thread in PreUpdate and consumed
	/// by Evaluate (possibly on a worker thread).
	TArray<FTransform> LocalPose;
};
