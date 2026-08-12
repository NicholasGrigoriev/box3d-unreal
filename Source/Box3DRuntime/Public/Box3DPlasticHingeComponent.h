#pragma once

#include "CoreMinimal.h"
#include "Box3DJointComponent.h"
#include "Box3DPlasticHingeComponent.generated.h"

class USplineMeshComponent;

/// Result of one rate-independent elastic-perfectly-plastic hinge update.
enum class EBox3DPlasticHingeTransition : uint8
{
	None,
	Yielded,
	Broke,
};

/// Pure-data constitutive state for an elastic-perfectly-plastic hinge.
/// Angles are degrees and torque/stiffness are N*m and N*m/degree. When the
/// elastic offset exceeds YieldTorque / ElasticStiffness, the excess becomes a
/// permanent rest-angle increment. Break is measured from the initial rest.
struct BOX3DRUNTIME_API FBox3DPlasticHingeResponse
{
	float InitialRestAngleDegrees = 0.0f;
	float RestAngleDegrees = 0.0f;
	float YieldTorqueNm = 100.0f;
	float ElasticStiffnessNmPerDegree = 20.0f;
	float BreakAngleDegrees = 45.0f;
	bool bBroken = false;

	/// Advance the constitutive state at a measured hinge angle. The returned
	/// plastic increment is exact for the ideal elastic-perfectly-plastic model.
	EBox3DPlasticHingeTransition Advance(float CurrentAngleDegrees,
		float& OutPlasticIncrementDegrees, float& OutRestoringTorqueNm);
};

/// Bend-then-break girder hinge: a two-body revolute joint whose spring target
/// advances permanently after yield. An optional spline mesh is bent in the
/// joint frame to visualise the measured angle; no mesh asset is required.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DPlasticHingeComponent : public UBox3DRevoluteJointComponent
{
	GENERATED_BODY()

public:
	UBox3DPlasticHingeComponent();

	/// Elastic yield torque in newton-metres.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Plastic Hinge", meta = (ClampMin = "0"))
	float YieldTorque = 100.0f;

	/// Constitutive stiffness used to convert YieldTorque to an elastic angle.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Plastic Hinge", meta = (ClampMin = "0.0001"))
	float ElasticStiffness = 20.0f;

	/// Absolute bend from the initial rest angle that breaks the joint. 0 disables angle breaking.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Plastic Hinge", meta = (ClampMin = "0"))
	float BreakAngle = 45.0f;

	/// Optional asset-free visual seam. The component stores spline geometry even
	/// without a StaticMesh; projects may assign their own girder mesh/material.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Plastic Hinge")
	TObjectPtr<USplineMeshComponent> VisualSpline;

	/// Length of each half of the visual girder, in centimetres.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Plastic Hinge", meta = (ClampMin = "0"))
	float VisualHalfLength = 100.0f;

	UFUNCTION(BlueprintPure, Category = "Box3D|Plastic Hinge")
	float GetPermanentRestAngle() const { return Response.RestAngleDegrees; }

	UFUNCTION(BlueprintPure, Category = "Box3D|Plastic Hinge")
	bool HasAngleBroken() const { return Response.bBroken; }

	/// Deterministic data/event seam used by fixed-step integration and tests.
	/// Runtime callers normally let the world subsystem feed the measured angle.
	void UpdatePlasticityAtAngle(float CurrentAngleDegrees);

	/// Called by UBox3DWorldSubsystem once after each Box3D fixed step.
	void HandlePostBox3DStep();

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void UpdateSplineVisual(float CurrentAngleDegrees);

	FBox3DPlasticHingeResponse Response;
};
