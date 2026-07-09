#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineMeshComponent.h"
#include "box3d/id.h"
#include "Box3DRopeActor.generated.h"

class UBox3DBodyComponent;
class USplineComponent;
class UStaticMesh;
class UMaterialInterface;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FBox3DRopeCutSignature, int32, LinkIndex);

/// A physically simulated rope: a chain of raw Box3D capsule bodies linked by
/// spherical joints, rendered as spline mesh segments. The physics bodies are
/// never drawn — only the spline skin, re-fitted to the chain every frame
/// (interpolated between fixed steps, so it stays smooth at any frame rate).
/// The straight rest shape previews in the editor viewport.
///
/// The rope hangs from a kinematic anchor at the actor location, so moving the
/// actor drags the rope. Optionally pin the far end to a world spot, or attach
/// it to another Box3D body at runtime (AttachActorToEnd). Segment bodies use
/// the Debris channel: explosions fling the rope and pawn proxies push it.
/// Set LinkBreakForce to make the rope snap under load (or call CutLink).
///
/// Bodies and joints are built on BeginPlay; the layout properties below are
/// creation-time only.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DRopeActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DRopeActor();

	/// Total rope length in cm.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10"))
	float RopeLength = 300.0f;

	/// Simulation links. More segments bend smoother and cost more.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "64"))
	int32 NumSegments = 16;

	/// Collision (and default visual) radius in cm.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0.5"))
	float RopeRadius = 2.0f;

	/// Segment density in kg/m^3 (rope mass = volume * density).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "1"))
	float Density = 500.0f;

	/// Air-drag feel; also calms solver jitter on long chains.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0"))
	float LinearDamping = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0"))
	float AngularDamping = 0.5f;

	/// Direction the chain is laid out along at build time when the end is not
	/// pinned (grapple ropes build toward their holder instead of hanging down).
	/// Normalized at use; zero falls back to straight down.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope")
	FVector BuildDirection = FVector(0.0, 0.0, -1.0);

	/// Links collide with pawn proxies (default). Grapple ropes turn this off so
	/// the chain cannot tangle on the character holding it.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope")
	bool bCollideWithPawns = true;

	/// Pin the far end to EndPinLocation with a static anchor (clothesline,
	/// power cable). Unpinned ropes hang free.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope")
	bool bPinEnd = false;

	/// Far-end pin location, relative to the actor (drag the widget in the viewport).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (EditCondition = "bPinEnd", MakeEditWidget))
	FVector EndPinLocation = FVector(0.0, 0.0, -300.0);

	/// Link tension (newtons) above which a joint snaps and the rope is cut.
	/// 0 = unbreakable. A hanging rope's top link carries roughly the weight
	/// below it (~10 N per kg), so leave generous headroom.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0"))
	float LinkBreakForce = 0.0f;

	/// Mesh stretched along each spline segment. Default: the engine cylinder.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	TObjectPtr<UStaticMesh> RopeMesh;

	/// Optional material override for the rope mesh.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	TObjectPtr<UMaterialInterface> RopeMaterial;

	/// Mesh axis that runs along the rope (engine cylinder: Z).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	TEnumAsByte<ESplineMeshAxis::Type> RopeMeshAxis = ESplineMeshAxis::Z;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	bool bCastShadow = true;

	/// A link snapped — via LinkBreakForce or an explicit CutLink. Link 0 is
	/// the anchor link, link N sits between segments N-1 and N.
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FBox3DRopeCutSignature OnRopeCut;

	/// World location of the rope's free end (last joint point).
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetEndLocation() const;

	/// World location of a segment body's center (0 = at the anchor).
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetSegmentLocation(int32 SegmentIndex) const;

	/// Cut the rope at a link (0 = anchor link, N = between segments N-1 and N,
	/// NumSegments = the end pin link when bPinEnd).
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void CutLink(int32 LinkIndex);

	/// Joints still holding the chain together.
	UFUNCTION(BlueprintPure, Category = "Rope")
	int32 GetLiveLinkCount() const { return LinkJoints.Num(); }

	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsCut() const { return bChainBroken; }

	/// Hang another actor's Box3D body off the rope end with a spherical joint.
	/// The body must exist already (call after BeginPlay). Replaces a previous
	/// end attachment. Returns false if the actor has no live Box3D body.
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool AttachActorToEnd(AActor* ActorToAttach);

	/// AttachActorToEnd's component form: tie a specific live body to the rope
	/// end. The attachment survives SetDeployedLength re-rigging.
	/// bSnapToBodyOrigin ties the rope end to the body's ORIGIN instead of
	/// capturing the current offset between them: a chain reeled shorter than
	/// the gap is then immediately under tension, and the tie point does not
	/// orbit the body as it rotates.
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool AttachBodyToEnd(UBox3DBodyComponent* Body, bool bSnapToBodyOrigin = false);

	/// Release whatever AttachActorToEnd tied on.
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void DetachEnd();

	/// Winch API: set how much rope is paid out, quantized to whole segments and
	/// clamped to [one segment, RopeLength]. Reeling in spools tail links into a
	/// disabled, hidden reserve; paying out feeds them back in at the end tip.
	/// The end attachment, if any, is re-tied to the new end link. No-op on a
	/// cut chain.
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void SetDeployedLength(float LengthCm);

	UFUNCTION(BlueprintPure, Category = "Rope")
	float GetDeployedLength() const;

	/// Constraint force (newtons, world space) currently carried by the end
	/// attachment joint — a load hanging off the end reads roughly its weight
	/// (~10 N per kg). Zero when nothing is attached. This is the winch's
	/// tension signal for grapple coupling.
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetEndConstraintForce() const;

	UFUNCTION(BlueprintPure, Category = "Rope")
	bool HasEndAttachment() const;

	/// Links currently simulating (the rest are spooled by SetDeployedLength).
	UFUNCTION(BlueprintPure, Category = "Rope")
	int32 GetActiveSegmentCount() const { return ActiveSegments; }

	/// Kick the rope at its free end (impulse in kg*cm/s).
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void AddImpulseAtEnd(FVector Impulse);

	/// Snap any link whose constraint force exceeds LinkBreakForce. Runs from
	/// Tick; public so headless tests can pump it without ticking actors.
	void PollLinkBreaks();

	/// The kinematic body the rope hangs from (collides with nothing).
	UBox3DBodyComponent* GetAnchorBody() const { return AnchorBody; }

	int32 GetBodyCount() const { return Bodies.Num(); }

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildRope();
	void DestroyRope();
	/// (Re)create the spline mesh segments. Runs in the editor via
	/// OnConstruction (straight preview) and again from BuildRope.
	void BuildSkin();
	/// Fit the spline and segment meshes through world-space joint points.
	void ApplyPointsToSkin(const TArray<FVector>& WorldPoints);
	void UpdateRopeVisual();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBox3DBodyComponent> AnchorBody;

	UPROPERTY()
	TObjectPtr<USplineComponent> Spline;

	UPROPERTY()
	TArray<TObjectPtr<USplineMeshComponent>> SegmentMeshes;

	void CreateEndAttachJoint();

	TArray<b3BodyId> Bodies;
	b3BodyId EndPinBodyId = {};
	b3JointId EndAttachJointId = {};
	b3BodyId EndAttachBodyId = {};
	/// Attachment point in the attached body's local space, captured once by
	/// AttachBodyToEnd and reused when re-tying after a deployed-length change.
	FVector EndAttachLocalPoint = FVector::ZeroVector;
	/// Links currently simulating; Bodies[ActiveSegments..] are spooled (disabled).
	int32 ActiveSegments = 0;

	struct FRopeLink
	{
		b3JointId Joint = {};
		int32 Index = 0;
	};
	TArray<FRopeLink> LinkJoints;
	bool bChainBroken = false;

	/// Per-body pose segment for between-step visual interpolation (same scheme
	/// as the subsystem's bInterpolateBodyTransforms, applied to raw bodies).
	struct FBodyInterp
	{
		FVector P0 = FVector::ZeroVector;
		FVector P1 = FVector::ZeroVector;
		FQuat Q0 = FQuat::Identity;
		FQuat Q1 = FQuat::Identity;
	};
	TArray<FBodyInterp> Interp;
	uint64 LastStep = 0;
	bool bAllAsleep = false;

	float SegmentLength = 0.0f;
};
