#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "box3d/id.h"
#include "Box3DJointComponent.generated.h"

class UBox3DBodyComponent;
class UBox3DJointComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FBox3DJointBrokeSignature, UBox3DJointComponent*, Joint);

/// Base class for Box3D joint components.
///
/// The component's own transform at creation defines the joint frame (pivot,
/// axes). Body B is the nearest UBox3DBodyComponent on the owning actor — put
/// the joint on the body that moves (the wheel, the pendulum bob, the slider).
/// Body A comes from ConnectedActor (the chassis/parent side); leave it unset
/// to pin against a fixed world anchor created at the joint's start transform.
///
/// Joint axes follow box3d conventions in the joint frame: revolute rotates
/// about local Z, prismatic slides along local A-frame X, wheels spin about
/// B-frame Z with suspension along A-frame X. Orient the component to aim them.
///
/// Forces are in newtons and torques in newton-meters (box3d's units); lengths
/// are cm and angles degrees, converted at the boundary.
UCLASS(Abstract, ClassGroup = (Physics))
class BOX3DRUNTIME_API UBox3DJointComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	/// The actor whose body becomes side A of the joint (chassis/parent/anchor).
	/// Unset: the joint connects to a static world anchor at the joint transform.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Joint")
	TObjectPtr<AActor> ConnectedActor;

	/// Allow the two connected bodies to collide with each other.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Joint")
	bool bCollideConnected = false;

	/// Destroy the joint (and broadcast OnJointBroke) when the constraint force
	/// or torque exceeds the thresholds below.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Joint")
	bool bBreakable = false;

	/// Constraint force that breaks the joint, in newtons. 0 = force never breaks it.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Joint", meta = (EditCondition = "bBreakable", ClampMin = "0"))
	float BreakForce = 0.0f;

	/// Constraint torque that breaks the joint, in newton-meters. 0 = torque never breaks it.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Joint", meta = (EditCondition = "bBreakable", ClampMin = "0"))
	float BreakTorque = 0.0f;

	/// Fired when the joint breaks — via thresholds or an explicit BreakJoint call.
	UPROPERTY(BlueprintAssignable, Category = "Box3D|Joint")
	FBox3DJointBrokeSignature OnJointBroke;

	/// True while a live Box3D joint backs this component.
	UFUNCTION(BlueprintPure, Category = "Box3D|Joint")
	bool IsJointActive() const;

	/// Destroy the joint now and broadcast OnJointBroke.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Joint")
	void BreakJoint();

	/// Current constraint force in newtons (world space).
	UFUNCTION(BlueprintPure, Category = "Box3D|Joint")
	FVector GetConstraintForce() const;

	/// Current constraint torque in newton-meters (world space).
	UFUNCTION(BlueprintPure, Category = "Box3D|Joint")
	FVector GetConstraintTorque() const;

	b3JointId GetJointId() const { return JointId; }

	/// Attempt joint creation; false while either body has no Box3D body yet.
	/// Retried by the world subsystem until it succeeds or attempts run out.
	bool TryCreateJoint();

	/// Called by the subsystem when box3d reports the joint's force/torque
	/// threshold exceeded during a step.
	void HandleThresholdExceeded();

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	int32 CreateAttempts = 0;

protected:
	/// Create the concrete b3 joint. Implementations fill their def from their
	/// default def + FillBaseDef and the component properties.
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
		PURE_VIRTUAL(UBox3DJointComponent::CreateConcreteJoint, return b3JointId{};);

	/// Fill the shared base definition: bodies, local frames from this
	/// component's world transform, collision flag, break thresholds, userData.
	void FillBaseDef(struct b3JointDef& Base, b3BodyId BodyA, b3BodyId BodyB) const;

	/// Local frame of this component's world transform in the given body's space.
	struct b3Transform MakeLocalFrame(b3BodyId BodyId) const;

	/// The body on the owning actor this joint drives (side B).
	UBox3DBodyComponent* ResolveOwnBody() const;

	b3JointId JointId = {};

	/// Internal static body when ConnectedActor is unset.
	b3BodyId AnchorBodyId = {};

private:
	void DestroyJointInternal();
};

/// Distance joint: keeps the joint's start location and the owning body's
/// origin connected by a rope/rod/spring of the given length.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DDistanceJointComponent : public UBox3DJointComponent
{
	GENERATED_BODY()

public:
	/// Derive the rest length from the distance between the joint transform and
	/// the body origin at creation.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance")
	bool bAutoLength = true;

	/// Rest length in cm (when not auto-derived).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance", meta = (ClampMin = "0", EditCondition = "!bAutoLength"))
	float Length = 100.0f;

	/// Behave like a spring instead of a rigid rod.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance")
	bool bEnableSpring = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringHertz = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringDampingRatio = 0.7f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance")
	bool bEnableLimit = false;

	/// Length limits in cm (spring mode only; a rigid joint ignores them).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance", meta = (EditCondition = "bEnableLimit", ClampMin = "0"))
	float MinLength = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance", meta = (EditCondition = "bEnableLimit", ClampMin = "0"))
	float MaxLength = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance")
	bool bEnableMotor = false;

	/// Desired length change speed in cm/s.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance", meta = (EditCondition = "bEnableMotor"))
	float MotorSpeed = 0.0f;

	/// Maximum motor force in newtons.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Distance", meta = (EditCondition = "bEnableMotor", ClampMin = "0"))
	float MaxMotorForce = 1000.0f;

	/// Set the rest length (cm) at runtime.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Distance")
	void SetLength(float LengthCm);

	/// Current distance between the attachment points in cm.
	UFUNCTION(BlueprintPure, Category = "Box3D|Distance")
	float GetCurrentLength() const;

	UFUNCTION(BlueprintCallable, Category = "Box3D|Distance")
	void SetMotorSpeed(float CmPerSec);

protected:
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB) override;
};

/// Revolute joint: hinge about the joint frame's Z axis.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DRevoluteJointComponent : public UBox3DJointComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute")
	bool bEnableSpring = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringHertz = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringDampingRatio = 0.7f;

	/// Spring target angle in degrees.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute", meta = (EditCondition = "bEnableSpring"))
	float TargetAngle = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute")
	bool bEnableLimit = false;

	/// Angular limits in degrees, at most +-178.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute", meta = (EditCondition = "bEnableLimit", ClampMin = "-178", ClampMax = "178"))
	float LowerAngle = -45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute", meta = (EditCondition = "bEnableLimit", ClampMin = "-178", ClampMax = "178"))
	float UpperAngle = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute")
	bool bEnableMotor = false;

	/// Desired motor speed in degrees per second.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute", meta = (EditCondition = "bEnableMotor"))
	float MotorSpeed = 0.0f;

	/// Maximum motor torque in newton-meters.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Revolute", meta = (EditCondition = "bEnableMotor", ClampMin = "0"))
	float MaxMotorTorque = 1000.0f;

	/// Current hinge angle in degrees.
	UFUNCTION(BlueprintPure, Category = "Box3D|Revolute")
	float GetJointAngle() const;

	UFUNCTION(BlueprintCallable, Category = "Box3D|Revolute")
	void SetMotorSpeed(float DegreesPerSec);

protected:
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB) override;
};

/// Prismatic joint: body B slides along the A-frame X axis, no relative rotation.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DPrismaticJointComponent : public UBox3DJointComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic")
	bool bEnableSpring = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringHertz = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringDampingRatio = 0.7f;

	/// Spring target translation along the axis in cm.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic", meta = (EditCondition = "bEnableSpring"))
	float TargetTranslation = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic")
	bool bEnableLimit = false;

	/// Translation limits along the axis in cm.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic", meta = (EditCondition = "bEnableLimit"))
	float LowerTranslation = -100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic", meta = (EditCondition = "bEnableLimit"))
	float UpperTranslation = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic")
	bool bEnableMotor = false;

	/// Desired motor speed in cm/s.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic", meta = (EditCondition = "bEnableMotor"))
	float MotorSpeed = 0.0f;

	/// Maximum motor force in newtons.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Prismatic", meta = (EditCondition = "bEnableMotor", ClampMin = "0"))
	float MaxMotorForce = 1000.0f;

	/// Current translation along the joint axis in cm.
	UFUNCTION(BlueprintPure, Category = "Box3D|Prismatic")
	float GetTranslation() const;

	UFUNCTION(BlueprintCallable, Category = "Box3D|Prismatic")
	void SetMotorSpeed(float CmPerSec);

protected:
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB) override;
};

/// Spherical (ball-and-socket) joint: pins the joint's start location on both
/// bodies together, allowing rotation about it. Optional cone/twist limits.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DSphericalJointComponent : public UBox3DJointComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical")
	bool bEnableSpring = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringHertz = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableSpring", ClampMin = "0"))
	float SpringDampingRatio = 0.7f;

	/// Spring target rotation of frame B relative to frame A.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableSpring"))
	FRotator TargetRotation = FRotator::ZeroRotator;

	/// Cone limit about the A-frame Z axis.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical")
	bool bEnableConeLimit = false;

	/// Cone half angle in degrees [0, 180].
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableConeLimit", ClampMin = "0", ClampMax = "180"))
	float ConeAngle = 45.0f;

	/// Twist limit about the B-frame Z axis.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical")
	bool bEnableTwistLimit = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableTwistLimit", ClampMin = "-178", ClampMax = "178"))
	float LowerTwistAngle = -45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableTwistLimit", ClampMin = "-178", ClampMax = "178"))
	float UpperTwistAngle = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical")
	bool bEnableMotor = false;

	/// Desired motor angular velocity in degrees per second (joint space).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableMotor"))
	FVector MotorVelocity = FVector::ZeroVector;

	/// Maximum motor torque in newton-meters.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Spherical", meta = (EditCondition = "bEnableMotor", ClampMin = "0"))
	float MaxMotorTorque = 1000.0f;

protected:
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB) override;
};

/// Weld joint: locks the two bodies together rigidly (or softly via the
/// hertz/damping springs).
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DWeldJointComponent : public UBox3DJointComponent
{
	GENERATED_BODY()

public:
	/// Linear stiffness in Hertz; 0 = fully rigid.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Weld", meta = (ClampMin = "0"))
	float LinearHertz = 0.0f;

	/// Angular stiffness in Hertz; 0 = fully rigid.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Weld", meta = (ClampMin = "0"))
	float AngularHertz = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Weld", meta = (ClampMin = "0"))
	float LinearDampingRatio = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Weld", meta = (ClampMin = "0"))
	float AngularDampingRatio = 1.0f;

protected:
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB) override;
};

/// Motor joint: drives the relative velocity (and optionally position via
/// springs) between two bodies. Useful for animated platforms and grabbing.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DMotorJointComponent : public UBox3DJointComponent
{
	GENERATED_BODY()

public:
	/// Desired relative linear velocity in cm/s.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor")
	FVector LinearVelocity = FVector::ZeroVector;

	/// Maximum force used to reach the linear velocity, in newtons.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float MaxVelocityForce = 10000.0f;

	/// Desired relative angular velocity in degrees per second.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor")
	FVector AngularVelocity = FVector::ZeroVector;

	/// Maximum torque used to reach the angular velocity, in newton-meters.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float MaxVelocityTorque = 10000.0f;

	/// Position spring: 0 disables position control.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float LinearHertz = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float LinearDampingRatio = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float MaxSpringForce = 10000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float AngularHertz = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float AngularDampingRatio = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Motor", meta = (ClampMin = "0"))
	float MaxSpringTorque = 10000.0f;

	/// Set the desired relative linear velocity (cm/s) at runtime.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Motor")
	void SetLinearVelocityTarget(FVector CmPerSec);

	/// Set the desired relative angular velocity (deg/s) at runtime.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Motor")
	void SetAngularVelocityTarget(FVector DegreesPerSec);

protected:
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB) override;
};

/// Wheel joint: body A (ConnectedActor) is the chassis, the owning body is the
/// wheel. Spin about the B-frame Z axis, suspension along the A-frame X axis,
/// optional steering about the A-frame X axis.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DWheelJointComponent : public UBox3DJointComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel")
	bool bEnableSuspensionSpring = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSuspensionSpring", ClampMin = "0"))
	float SuspensionHertz = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSuspensionSpring", ClampMin = "0"))
	float SuspensionDampingRatio = 0.7f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel")
	bool bEnableSuspensionLimit = true;

	/// Suspension travel limits along the axis in cm.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSuspensionLimit"))
	float LowerSuspensionLimit = -25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSuspensionLimit"))
	float UpperSuspensionLimit = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel")
	bool bEnableSpinMotor = false;

	/// Desired spin speed in degrees per second.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSpinMotor"))
	float SpinSpeed = 0.0f;

	/// Maximum spin torque in newton-meters.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSpinMotor", ClampMin = "0"))
	float MaxSpinTorque = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel")
	bool bEnableSteering = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSteering", ClampMin = "0"))
	float SteeringHertz = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSteering", ClampMin = "0"))
	float SteeringDampingRatio = 1.0f;

	/// Target steering angle in degrees.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSteering"))
	float TargetSteeringAngle = 0.0f;

	/// Maximum steering torque in newton-meters.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSteering", ClampMin = "0"))
	float MaxSteeringTorque = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel")
	bool bEnableSteeringLimit = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSteeringLimit", ClampMin = "-178", ClampMax = "178"))
	float LowerSteeringLimit = -30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Wheel", meta = (EditCondition = "bEnableSteeringLimit", ClampMin = "-178", ClampMax = "178"))
	float UpperSteeringLimit = 30.0f;

	/// Set the desired spin speed (deg/s) at runtime.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Wheel")
	void SetSpinSpeed(float DegreesPerSec);

	/// Set the target steering angle (degrees) at runtime.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Wheel")
	void SetTargetSteeringAngle(float Degrees);

protected:
	virtual b3JointId CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB) override;
};
