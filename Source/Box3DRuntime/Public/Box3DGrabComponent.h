#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Box3DTypes.h"
#include "box3d/id.h"
#include "Box3DGrabComponent.generated.h"

class UBox3DBodyComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FBox3DGrabbedSignature, UBox3DBodyComponent*, Body);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FBox3DReleasedSignature, UBox3DBodyComponent*, Body, bool, bThrown);

/// Grab, carry, and throw dynamic Box3D bodies — physics hands, no tractor
/// beams. The held body stays fully dynamic: it is dragged toward the hold
/// target by a velocity-tracking force each fixed step, so it still collides,
/// still has momentum, and visibly sags and lags when heavy. Everything obeys
/// the mass: bodies over MaxGrabMassKg refuse to be picked up, and a throw
/// spends a fixed impulse, so v = J/m — light things fly, heavy things lob.
///
/// The owner steers the carry by calling SetHoldTarget every frame (typically
/// camera location + forward * hold distance). If the body gets snagged or
/// blocked beyond BreakDistance, the grip breaks automatically.
///
/// While held, the body's shapes stop colliding with the Pawn channel so it
/// cannot wedge against its holder's kinematic proxy; original filters are
/// restored on release.
///
/// Doubles as a winch: grab a body far away and it reels toward the hold
/// target at up to MaxHoldSpeed — a physics grapple's "pull the light thing
/// to me" branch is just GrabBody + a nearby hold target.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DGrabComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBox3DGrabComponent();

	/// Heaviest body this grip can hold, kg. 0 = unlimited.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "0"))
	float MaxGrabMassKg = 60.0f;

	/// How eagerly the body chases the hold target: target speed is
	/// (distance x stiffness), in 1/s. Higher = snappier carry.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "1", ClampMax = "60"))
	float HoldStiffness = 12.0f;

	/// Carry/reel speed cap in cm/s — also the winch speed for far grabs.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "50"))
	float MaxHoldSpeed = 1200.0f;

	/// Fraction of the velocity error corrected per fixed step. 1 = rigid
	/// (snaps to target velocity), lower = softer, springier grip.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "0.05", ClampMax = "1"))
	float TrackingSoftness = 0.6f;

	/// Spin bleed-off while held, 1/s — carried objects settle instead of
	/// twirling forever.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "0"))
	float AngularDampingRate = 8.0f;

	/// Grip breaks when the body ends up this far (cm) from the hold target —
	/// snagged on geometry, blasted away, or the target teleported.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "50"))
	float BreakDistance = 600.0f;

	/// Throw impulse budget in kg*cm/s: release speed is impulse/mass, so a
	/// 5 kg crate flies at 1200 cm/s while a 50 kg one lobs at 120.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "0"))
	float ThrowImpulseKgCmS = 6000.0f;

	/// Release-speed ceiling for featherweight objects, cm/s.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab", meta = (ClampMin = "0"))
	float MaxThrowSpeed = 2500.0f;

	/// What GrabAlongRay may latch onto. Default: WorldDynamic and Debris
	/// (props and loose chunks), never static world or pawns.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grab")
	FBox3DQueryFilter GrabRayFilter;

	UPROPERTY(BlueprintAssignable, Category = "Grab")
	FBox3DGrabbedSignature OnGrabbed;

	UPROPERTY(BlueprintAssignable, Category = "Grab")
	FBox3DReleasedSignature OnReleased;

	/// Take hold of a dynamic body. Fails on static/kinematic/disabled bodies
	/// and anything over the mass limit. The hold target starts at the body's
	/// current position — call SetHoldTarget to start moving it.
	UFUNCTION(BlueprintCallable, Category = "Grab")
	bool GrabBody(UBox3DBodyComponent* Body);

	/// Ray-grab: closest component-backed dynamic body along the ray (raw
	/// bodies like cloth particles have no component and are skipped).
	UFUNCTION(BlueprintCallable, Category = "Grab")
	bool GrabAlongRay(FVector Start, FVector Direction, float MaxDistance);

	/// Where the held body is dragged toward, world cm. Call every frame while
	/// carrying (e.g. camera + forward * arm length).
	UFUNCTION(BlueprintCallable, Category = "Grab")
	void SetHoldTarget(FVector WorldLocation) { HoldTarget = WorldLocation; }

	/// Let go without imparting anything beyond the carry momentum.
	UFUNCTION(BlueprintCallable, Category = "Grab")
	void Release();

	/// Hurl the held body: impulse of ThrowImpulseKgCmS along Direction
	/// (speed capped at MaxThrowSpeed), then release.
	UFUNCTION(BlueprintCallable, Category = "Grab")
	bool Throw(FVector Direction);

	UFUNCTION(BlueprintPure, Category = "Grab")
	bool IsHolding() const;

	UFUNCTION(BlueprintPure, Category = "Grab")
	UBox3DBodyComponent* GetHeldBody() const { return HeldBody.Get(); }

	UFUNCTION(BlueprintPure, Category = "Grab")
	float GetHeldMassKg() const { return HeldMassKg; }

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void PreStep(float FixedDeltaTime);
	void ReleaseInternal(bool bThrown);
	void RestoreHeldFilters();

	TWeakObjectPtr<UBox3DBodyComponent> HeldBody;
	/// Original per-shape filter mask bits, restored on release (the Pawn bit
	/// is stripped while held; category/group are never touched).
	TArray<uint64> SavedMaskBits;
	FVector HoldTarget = FVector::ZeroVector;
	float HeldMassKg = 0.0f;
	FDelegateHandle PreStepHandle;
};
