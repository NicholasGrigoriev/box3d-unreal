#pragma once

// Event sink for automation tests: dynamic delegates need UFUNCTION targets, so
// this tiny UObject counts broadcasts. Not guarded by WITH_DEV_AUTOMATION_TESTS
// because UHT-generated code must compile unconditionally; it is never
// instantiated outside tests.

#include "CoreMinimal.h"
#include "Box3DBodyComponent.h"
#include "Box3DJointComponent.h"
#include "Box3DTestEventCounter.generated.h"

UCLASS()
class UBox3DTestEventCounter : public UObject
{
	GENERATED_BODY()

public:
	int32 ContactBeginCount = 0;
	int32 ContactEndCount = 0;
	int32 HitCount = 0;
	int32 SensorBeginCount = 0;
	int32 SensorEndCount = 0;
	int32 JointBrokeCount = 0;
	int32 WeldBrokeCount = 0;

	UPROPERTY()
	TObjectPtr<UBox3DBodyComponent> LastContactOther;

	UPROPERTY()
	TObjectPtr<UBox3DBodyComponent> LastVisitor;

	FVector LastHitNormal = FVector::ZeroVector;
	float LastApproachSpeed = 0.0f;

	UFUNCTION()
	void HandleContactBegin(UBox3DBodyComponent* OtherBody, AActor* OtherActor)
	{
		++ContactBeginCount;
		LastContactOther = OtherBody;
	}

	UFUNCTION()
	void HandleContactEnd(UBox3DBodyComponent* OtherBody, AActor* OtherActor)
	{
		++ContactEndCount;
	}

	UFUNCTION()
	void HandleHit(UBox3DBodyComponent* OtherBody, FVector Location, FVector Normal, float ApproachSpeed)
	{
		++HitCount;
		LastHitNormal = Normal;
		LastApproachSpeed = ApproachSpeed;
	}

	UFUNCTION()
	void HandleSensorBegin(UBox3DBodyComponent* VisitorBody, AActor* VisitorActor)
	{
		++SensorBeginCount;
		LastVisitor = VisitorBody;
	}

	UFUNCTION()
	void HandleSensorEnd(UBox3DBodyComponent* VisitorBody, AActor* VisitorActor)
	{
		++SensorEndCount;
	}

	UFUNCTION()
	void HandleJointBroke(UBox3DJointComponent* Joint)
	{
		++JointBrokeCount;
	}

	UFUNCTION()
	void HandleWeldBroke()
	{
		++WeldBrokeCount;
	}
};
