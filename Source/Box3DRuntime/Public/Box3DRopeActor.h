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

/// A physically simulated rope: a chain of raw Box3D capsule bodies linked by
/// spherical joints, rendered as spline mesh segments. The physics bodies are
/// never drawn — only the spline skin, re-fitted to the chain every frame
/// (interpolated between fixed steps, so it stays smooth at any frame rate).
///
/// The rope hangs from a kinematic anchor at the actor location, so moving the
/// actor drags the rope. Optionally pin the far end to a world spot, or attach
/// it to another Box3D body at runtime (AttachActorToEnd). Segment bodies use
/// the Debris channel: explosions fling the rope and pawn proxies push it.
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

	/// Pin the far end to EndPinLocation with a static anchor (clothesline,
	/// power cable). Unpinned ropes hang free.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope")
	bool bPinEnd = false;

	/// Far-end pin location, relative to the actor (drag the widget in the viewport).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (EditCondition = "bPinEnd", MakeEditWidget))
	FVector EndPinLocation = FVector(0.0, 0.0, -300.0);

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

	/// World location of the rope's free end (last joint point).
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetEndLocation() const;

	/// World location of a segment body's center (0 = at the anchor).
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetSegmentLocation(int32 SegmentIndex) const;

	/// Hang another actor's Box3D body off the rope end with a spherical joint.
	/// The body must exist already (call after BeginPlay). Replaces a previous
	/// end attachment. Returns false if the actor has no live Box3D body.
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool AttachActorToEnd(AActor* ActorToAttach);

	/// Release whatever AttachActorToEnd tied on.
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void DetachEnd();

	/// Kick the rope at its free end (impulse in kg*cm/s).
	UFUNCTION(BlueprintCallable, Category = "Rope")
	void AddImpulseAtEnd(FVector Impulse);

	/// The kinematic body the rope hangs from (collides with nothing).
	UBox3DBodyComponent* GetAnchorBody() const { return AnchorBody; }

	int32 GetBodyCount() const { return Bodies.Num(); }

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildRope();
	void DestroyRope();
	void UpdateRopeVisual();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBox3DBodyComponent> AnchorBody;

	UPROPERTY()
	TObjectPtr<USplineComponent> Spline;

	UPROPERTY()
	TArray<TObjectPtr<USplineMeshComponent>> SegmentMeshes;

	TArray<b3BodyId> Bodies;
	b3BodyId EndPinBodyId = {};
	b3JointId EndAttachJointId = {};

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
