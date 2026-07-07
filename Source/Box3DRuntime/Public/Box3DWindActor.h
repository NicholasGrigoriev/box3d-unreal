#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Box3DWindActor.generated.h"

class UArrowComponent;
class USplineComponent;
class UBox3DWorldSubsystem;

/// The shape of the wind velocity field.
UENUM(BlueprintType)
enum class EBox3DWindMode : uint8
{
	/// Uniform wind along the actor's forward (X) axis.
	Directional,
	/// Scrolling Perlin field: direction and strength vary over space and
	/// time. Gusty, chaotic — leaves and loose cloth.
	Turbulence,
	/// Wind follows the actor's spline: at any point it blows along the
	/// tangent of the nearest spline location. A wind tunnel on a path.
	Spline,
	/// Swirls around the actor's up axis: tangential flow plus optional
	/// inward pull and updraft. A tornado.
	Vortex,
};

/// A wind source for Box3D bodies. Every fixed step it finds dynamic shapes in
/// range and applies a drag force F = Drag * Area * (WindVelocity - BodyVelocity),
/// so light things (cloth, ropes, debris) stream and flutter while heavy props
/// barely notice — and everything settles once it reaches wind speed.
///
/// Forces are applied per fixed step via the subsystem's OnPreStep hook, so
/// the strength is frame-rate independent. All knobs are live-editable.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DWindActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DWindActor();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
	EBox3DWindMode WindMode = EBox3DWindMode::Directional;

	/// Wind speed in cm/s. 300 = breeze, 1000 = storm, 3000 = hurricane.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "0"))
	float WindSpeed = 600.0f;

	/// Range in cm (from the actor, or from the spline in Spline mode).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "50"))
	float Radius = 1500.0f;

	/// Strength falloff toward the edge of the range. 0 = no falloff,
	/// 1 = linear, 2 = quadratic.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "0", ClampMax = "4"))
	float FalloffExponent = 1.0f;

	/// Air grip: force per m^2 of body cross-section per m/s of relative flow.
	/// Higher = bodies reach wind speed faster.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "0"))
	float Drag = 15.0f;

	/// Slow speed oscillation on top of the base wind (0 = steady).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "0", ClampMax = "1"))
	float GustAmount = 0.3f;

	/// Gust cycles per second.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "0"))
	float GustFrequency = 0.4f;

	/// Wake sleeping bodies in range. On: gusts reach settled cloth (which then
	/// stays awake while the wind blows). Off: wind only steers what already moves.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
	bool bWakeBodies = true;

	/// Turbulence: spatial wavelength of the noise field in cm.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Turbulence", meta = (ClampMin = "10", EditCondition = "WindMode == EBox3DWindMode::Turbulence"))
	float TurbulenceScale = 400.0f;

	/// Turbulence: how fast the field scrolls/churns over time.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Turbulence", meta = (ClampMin = "0", EditCondition = "WindMode == EBox3DWindMode::Turbulence"))
	float TurbulenceSpeed = 1.0f;

	/// Vortex: upward flow along the actor's up axis, cm/s.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Vortex", meta = (EditCondition = "WindMode == EBox3DWindMode::Vortex"))
	float UpdraftSpeed = 300.0f;

	/// Vortex: inward pull toward the axis, cm/s. Negative blows outward.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind|Vortex", meta = (EditCondition = "WindMode == EBox3DWindMode::Vortex"))
	float InwardSpeed = 150.0f;

	/// Wind velocity (cm/s) the field produces at a world position, falloff and
	/// gusts included. Zero outside the range.
	UFUNCTION(BlueprintPure, Category = "Wind")
	FVector GetWindVelocityAt(FVector Position) const;

	/// The path the wind follows in Spline mode. Edit its points in the viewport.
	USplineComponent* GetWindSpline() const { return WindSpline; }

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/// OnPreStep: apply drag forces for this step.
	void ApplyWind(float FixedDeltaTime);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wind", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USplineComponent> WindSpline;

	UPROPERTY()
	TObjectPtr<UArrowComponent> DirectionArrow;

	FDelegateHandle PreStepHandle;

	/// Deterministic phase clock: advanced per fixed step, not per frame.
	float WindTime = 0.0f;
};
