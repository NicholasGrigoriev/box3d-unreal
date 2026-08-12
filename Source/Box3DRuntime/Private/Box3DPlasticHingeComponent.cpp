#include "Box3DPlasticHingeComponent.h"

#include "Box3DWorldSubsystem.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/World.h"
#include "box3d/box3d.h"

EBox3DPlasticHingeTransition FBox3DPlasticHingeResponse::Advance(float CurrentAngleDegrees,
	float& OutPlasticIncrementDegrees, float& OutRestoringTorqueNm)
{
	OutPlasticIncrementDegrees = 0.0f;
	OutRestoringTorqueNm = 0.0f;
	if (bBroken || !FMath::IsFinite(CurrentAngleDegrees))
	{
		return EBox3DPlasticHingeTransition::None;
	}

	if (BreakAngleDegrees > 0.0f
		&& FMath::Abs(CurrentAngleDegrees - InitialRestAngleDegrees) >= BreakAngleDegrees)
	{
		bBroken = true;
		return EBox3DPlasticHingeTransition::Broke;
	}

	if (!FMath::IsFinite(YieldTorqueNm) || !FMath::IsFinite(ElasticStiffnessNmPerDegree)
		|| ElasticStiffnessNmPerDegree <= UE_SMALL_NUMBER || YieldTorqueNm < 0.0f)
	{
		return EBox3DPlasticHingeTransition::None;
	}

	const float ElasticOffset = CurrentAngleDegrees - RestAngleDegrees;
	const float YieldAngle = YieldTorqueNm / ElasticStiffnessNmPerDegree;
	if (FMath::Abs(ElasticOffset) > YieldAngle)
	{
		const float PreviousRestAngle = RestAngleDegrees;
		RestAngleDegrees = CurrentAngleDegrees - FMath::Sign(ElasticOffset) * YieldAngle;
		OutPlasticIncrementDegrees = RestAngleDegrees - PreviousRestAngle;
		OutRestoringTorqueNm = -FMath::Sign(ElasticOffset) * YieldTorqueNm;
		return EBox3DPlasticHingeTransition::Yielded;
	}

	OutRestoringTorqueNm = -ElasticOffset * ElasticStiffnessNmPerDegree;
	return EBox3DPlasticHingeTransition::None;
}

UBox3DPlasticHingeComponent::UBox3DPlasticHingeComponent()
{
	bEnableSpring = true;
}

void UBox3DPlasticHingeComponent::BeginPlay()
{
	Response.InitialRestAngleDegrees = TargetAngle;
	Response.RestAngleDegrees = TargetAngle;
	Response.YieldTorqueNm = YieldTorque;
	Response.ElasticStiffnessNmPerDegree = ElasticStiffness;
	Response.BreakAngleDegrees = BreakAngle;
	Response.bBroken = false;

	Super::BeginPlay();
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->RegisterPlasticHinge(this);
	}
	UpdateSplineVisual(TargetAngle);
}

void UBox3DPlasticHingeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->UnregisterPlasticHinge(this);
	}
	Super::EndPlay(EndPlayReason);
}

void UBox3DPlasticHingeComponent::HandlePostBox3DStep()
{
	if (IsJointActive())
	{
		UpdatePlasticityAtAngle(GetJointAngle());
	}
}

void UBox3DPlasticHingeComponent::UpdatePlasticityAtAngle(float CurrentAngleDegrees)
{
	Response.YieldTorqueNm = YieldTorque;
	Response.ElasticStiffnessNmPerDegree = ElasticStiffness;
	Response.BreakAngleDegrees = BreakAngle;

	float PlasticIncrement = 0.0f;
	float RestoringTorque = 0.0f;
	const EBox3DPlasticHingeTransition Transition = Response.Advance(
		CurrentAngleDegrees, PlasticIncrement, RestoringTorque);

	UpdateSplineVisual(CurrentAngleDegrees);
	if (Transition == EBox3DPlasticHingeTransition::Broke)
	{
		BreakJoint();
		return;
	}

	if (Transition == EBox3DPlasticHingeTransition::Yielded && b3Joint_IsValid(JointId))
	{
		TargetAngle = Response.RestAngleDegrees;
		b3RevoluteJoint_SetTargetAngle(JointId, FMath::DegreesToRadians(TargetAngle));
	}
}

void UBox3DPlasticHingeComponent::UpdateSplineVisual(float CurrentAngleDegrees)
{
	if (VisualSpline == nullptr || !FMath::IsFinite(CurrentAngleDegrees) || VisualHalfLength <= 0.0f)
	{
		return;
	}

	const float BendRadians = FMath::DegreesToRadians(CurrentAngleDegrees - Response.InitialRestAngleDegrees);
	const FVector2D BentDirection(FMath::Cos(BendRadians), FMath::Sin(BendRadians));
	const FVector StartPosition(-VisualHalfLength, 0.0f, 0.0f);
	const FVector EndPosition(VisualHalfLength * BentDirection.X,
		VisualHalfLength * BentDirection.Y, 0.0f);
	const FVector StartTangent(VisualHalfLength, 0.0f, 0.0f);
	const FVector EndTangent(VisualHalfLength * BentDirection.X,
		VisualHalfLength * BentDirection.Y, 0.0f);
	VisualSpline->SetStartAndEnd(StartPosition, StartTangent, EndPosition, EndTangent, true);
}
