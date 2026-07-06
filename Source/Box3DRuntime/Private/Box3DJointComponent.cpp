#include "Box3DJointComponent.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "box3d/box3d.h"

namespace
{
	constexpr b3Transform IdentityFrame{ { 0.0f, 0.0f, 0.0f }, { { 0.0f, 0.0f, 0.0f }, 1.0f } };
}

void UBox3DJointComponent::BeginPlay()
{
	Super::BeginPlay();

	if (!TryCreateJoint())
	{
		// Bodies on other actors may not have created their Box3D bodies yet;
		// the subsystem retries pending joints before each tick's stepping.
		if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
		{
			Subsystem->AddPendingJoint(this);
		}
	}
}

void UBox3DJointComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyJointInternal();
	Super::EndPlay(EndPlayReason);
}

bool UBox3DJointComponent::TryCreateJoint()
{
	if (b3Joint_IsValid(JointId))
	{
		return true;
	}
	++CreateAttempts;

	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return false;
	}
	const b3WorldId WorldId = Subsystem->GetBox3DWorldId();

	UBox3DBodyComponent* OwnBody = ResolveOwnBody();
	if (OwnBody == nullptr)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: no UBox3DBodyComponent on the owning actor; joint not created"), *GetPathName());
		return false;
	}
	if (!OwnBody->IsSimulating())
	{
		return false; // body not created yet, retry later
	}

	b3BodyId BodyA{};
	if (ConnectedActor != nullptr)
	{
		const UBox3DBodyComponent* Other = ConnectedActor->FindComponentByClass<UBox3DBodyComponent>();
		if (Other == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("%s: ConnectedActor %s has no UBox3DBodyComponent; joint not created"),
				*GetPathName(), *GetNameSafe(ConnectedActor));
			return false;
		}
		if (!Other->IsSimulating())
		{
			return false; // retry later
		}
		BodyA = Other->GetBodyId();
	}
	else
	{
		// Pin against the world: a shapeless static anchor at the joint transform.
		b3BodyDef AnchorDef = b3DefaultBodyDef();
		AnchorDef.type = b3_staticBody;
		AnchorDef.position = Box3D::ToB3Pos(GetComponentLocation());
		AnchorDef.rotation = Box3D::ToB3(GetComponentQuat());
		AnchorDef.name = "Box3DJointAnchor";
		AnchorBodyId = b3CreateBody(WorldId, &AnchorDef);
		BodyA = AnchorBodyId;
	}

	JointId = CreateConcreteJoint(WorldId, BodyA, OwnBody->GetBodyId());
	return b3Joint_IsValid(JointId);
}

void UBox3DJointComponent::FillBaseDef(b3JointDef& Base, b3BodyId BodyA, b3BodyId BodyB) const
{
	Base.bodyIdA = BodyA;
	Base.bodyIdB = BodyB;
	Base.localFrameA = MakeLocalFrame(BodyA);
	Base.localFrameB = MakeLocalFrame(BodyB);
	Base.collideConnected = bCollideConnected;
	Base.userData = const_cast<UBox3DJointComponent*>(this);
	if (bBreakable)
	{
		if (BreakForce > 0.0f)
		{
			Base.forceThreshold = BreakForce;
		}
		if (BreakTorque > 0.0f)
		{
			Base.torqueThreshold = BreakTorque;
		}
	}
}

b3Transform UBox3DJointComponent::MakeLocalFrame(b3BodyId BodyId) const
{
	const b3WorldTransform BodyTransform = b3Body_GetTransform(BodyId);
	const FTransform BodyWorld(Box3D::ToUE(BodyTransform.q), Box3D::ToUEPos(BodyTransform.p));
	const FTransform JointWorld(GetComponentQuat(), GetComponentLocation());
	const FTransform Local = JointWorld.GetRelativeTransform(BodyWorld);
	return b3Transform{ Box3D::ToB3(Local.GetLocation()), Box3D::ToB3(Local.GetRotation()) };
}

UBox3DBodyComponent* UBox3DJointComponent::ResolveOwnBody() const
{
	// Nearest body up the attach chain first, then anything on the actor.
	for (USceneComponent* Ancestor = GetAttachParent(); Ancestor != nullptr; Ancestor = Ancestor->GetAttachParent())
	{
		if (UBox3DBodyComponent* Body = Cast<UBox3DBodyComponent>(Ancestor))
		{
			return Body;
		}
	}
	return GetOwner() ? GetOwner()->FindComponentByClass<UBox3DBodyComponent>() : nullptr;
}

void UBox3DJointComponent::DestroyJointInternal()
{
	if (b3Joint_IsValid(JointId))
	{
		b3DestroyJoint(JointId, /*wakeAttached*/ true);
	}
	JointId = b3JointId{};

	if (b3Body_IsValid(AnchorBodyId))
	{
		b3DestroyBody(AnchorBodyId);
	}
	AnchorBodyId = b3BodyId{};
}

bool UBox3DJointComponent::IsJointActive() const
{
	return b3Joint_IsValid(JointId);
}

void UBox3DJointComponent::BreakJoint()
{
	if (!b3Joint_IsValid(JointId))
	{
		return;
	}
	DestroyJointInternal();
	OnJointBroke.Broadcast(this);
}

void UBox3DJointComponent::HandleThresholdExceeded()
{
	if (bBreakable)
	{
		BreakJoint();
	}
}

FVector UBox3DJointComponent::GetConstraintForce() const
{
	return b3Joint_IsValid(JointId) ? Box3D::ToUEDir(b3Joint_GetConstraintForce(JointId)) : FVector::ZeroVector;
}

FVector UBox3DJointComponent::GetConstraintTorque() const
{
	return b3Joint_IsValid(JointId) ? Box3D::ToUEDir(b3Joint_GetConstraintTorque(JointId)) : FVector::ZeroVector;
}

//~ Distance --------------------------------------------------------------------

b3JointId UBox3DDistanceJointComponent::CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
{
	b3DistanceJointDef Def = b3DefaultDistanceJointDef();
	FillBaseDef(Def.base, BodyA, BodyB);

	// Rope semantics: attach to the owning body's origin rather than the joint
	// location, so "joint at the mount, body below" gives a sensible length.
	Def.base.localFrameB = IdentityFrame;

	const b3WorldTransform BodyTransform = b3Body_GetTransform(BodyB);
	const float CurrentLength = static_cast<float>(
		FVector::Dist(GetComponentLocation(), Box3D::ToUEPos(BodyTransform.p)));
	Def.length = (bAutoLength ? CurrentLength : Length) * Box3D::UEToMeters;

	Def.enableSpring = bEnableSpring;
	Def.hertz = SpringHertz;
	Def.dampingRatio = SpringDampingRatio;
	Def.enableLimit = bEnableLimit;
	Def.minLength = MinLength * Box3D::UEToMeters;
	Def.maxLength = MaxLength * Box3D::UEToMeters;
	Def.enableMotor = bEnableMotor;
	Def.maxMotorForce = MaxMotorForce;
	Def.motorSpeed = MotorSpeed * Box3D::UEToMeters;
	return b3CreateDistanceJoint(WorldId, &Def);
}

void UBox3DDistanceJointComponent::SetLength(float LengthCm)
{
	Length = LengthCm;
	if (b3Joint_IsValid(JointId))
	{
		b3DistanceJoint_SetLength(JointId, LengthCm * Box3D::UEToMeters);
	}
}

float UBox3DDistanceJointComponent::GetCurrentLength() const
{
	return b3Joint_IsValid(JointId) ? b3DistanceJoint_GetCurrentLength(JointId) * Box3D::MetersToUE : 0.0f;
}

void UBox3DDistanceJointComponent::SetMotorSpeed(float CmPerSec)
{
	MotorSpeed = CmPerSec;
	if (b3Joint_IsValid(JointId))
	{
		b3DistanceJoint_SetMotorSpeed(JointId, CmPerSec * Box3D::UEToMeters);
	}
}

//~ Revolute --------------------------------------------------------------------

b3JointId UBox3DRevoluteJointComponent::CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
{
	b3RevoluteJointDef Def = b3DefaultRevoluteJointDef();
	FillBaseDef(Def.base, BodyA, BodyB);
	Def.targetAngle = FMath::DegreesToRadians(TargetAngle);
	Def.enableSpring = bEnableSpring;
	Def.hertz = SpringHertz;
	Def.dampingRatio = SpringDampingRatio;
	Def.enableLimit = bEnableLimit;
	Def.lowerAngle = FMath::DegreesToRadians(LowerAngle);
	Def.upperAngle = FMath::DegreesToRadians(UpperAngle);
	Def.enableMotor = bEnableMotor;
	Def.maxMotorTorque = MaxMotorTorque;
	Def.motorSpeed = FMath::DegreesToRadians(MotorSpeed);
	return b3CreateRevoluteJoint(WorldId, &Def);
}

float UBox3DRevoluteJointComponent::GetJointAngle() const
{
	return b3Joint_IsValid(JointId) ? FMath::RadiansToDegrees(b3RevoluteJoint_GetAngle(JointId)) : 0.0f;
}

void UBox3DRevoluteJointComponent::SetMotorSpeed(float DegreesPerSec)
{
	MotorSpeed = DegreesPerSec;
	if (b3Joint_IsValid(JointId))
	{
		b3RevoluteJoint_SetMotorSpeed(JointId, FMath::DegreesToRadians(DegreesPerSec));
	}
}

//~ Prismatic -------------------------------------------------------------------

b3JointId UBox3DPrismaticJointComponent::CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
{
	b3PrismaticJointDef Def = b3DefaultPrismaticJointDef();
	FillBaseDef(Def.base, BodyA, BodyB);
	Def.enableSpring = bEnableSpring;
	Def.hertz = SpringHertz;
	Def.dampingRatio = SpringDampingRatio;
	Def.targetTranslation = TargetTranslation * Box3D::UEToMeters;
	Def.enableLimit = bEnableLimit;
	Def.lowerTranslation = LowerTranslation * Box3D::UEToMeters;
	Def.upperTranslation = UpperTranslation * Box3D::UEToMeters;
	Def.enableMotor = bEnableMotor;
	Def.maxMotorForce = MaxMotorForce;
	Def.motorSpeed = MotorSpeed * Box3D::UEToMeters;
	return b3CreatePrismaticJoint(WorldId, &Def);
}

float UBox3DPrismaticJointComponent::GetTranslation() const
{
	return b3Joint_IsValid(JointId) ? b3PrismaticJoint_GetTranslation(JointId) * Box3D::MetersToUE : 0.0f;
}

void UBox3DPrismaticJointComponent::SetMotorSpeed(float CmPerSec)
{
	MotorSpeed = CmPerSec;
	if (b3Joint_IsValid(JointId))
	{
		b3PrismaticJoint_SetMotorSpeed(JointId, CmPerSec * Box3D::UEToMeters);
	}
}

//~ Spherical -------------------------------------------------------------------

b3JointId UBox3DSphericalJointComponent::CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
{
	b3SphericalJointDef Def = b3DefaultSphericalJointDef();
	FillBaseDef(Def.base, BodyA, BodyB);
	Def.enableSpring = bEnableSpring;
	Def.hertz = SpringHertz;
	Def.dampingRatio = SpringDampingRatio;
	Def.targetRotation = Box3D::ToB3(TargetRotation.Quaternion());
	Def.enableConeLimit = bEnableConeLimit;
	Def.coneAngle = FMath::DegreesToRadians(ConeAngle);
	Def.enableTwistLimit = bEnableTwistLimit;
	Def.lowerTwistAngle = FMath::DegreesToRadians(LowerTwistAngle);
	Def.upperTwistAngle = FMath::DegreesToRadians(UpperTwistAngle);
	Def.enableMotor = bEnableMotor;
	Def.maxMotorTorque = MaxMotorTorque;
	Def.motorVelocity = Box3D::ToB3Dir(FVector(
		FMath::DegreesToRadians(MotorVelocity.X),
		FMath::DegreesToRadians(MotorVelocity.Y),
		FMath::DegreesToRadians(MotorVelocity.Z)));
	return b3CreateSphericalJoint(WorldId, &Def);
}

//~ Weld ------------------------------------------------------------------------

b3JointId UBox3DWeldJointComponent::CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
{
	b3WeldJointDef Def = b3DefaultWeldJointDef();
	FillBaseDef(Def.base, BodyA, BodyB);
	Def.linearHertz = LinearHertz;
	Def.angularHertz = AngularHertz;
	Def.linearDampingRatio = LinearDampingRatio;
	Def.angularDampingRatio = AngularDampingRatio;
	return b3CreateWeldJoint(WorldId, &Def);
}

//~ Motor -----------------------------------------------------------------------

b3JointId UBox3DMotorJointComponent::CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
{
	b3MotorJointDef Def = b3DefaultMotorJointDef();
	FillBaseDef(Def.base, BodyA, BodyB);
	Def.linearVelocity = Box3D::ToB3(LinearVelocity);
	Def.maxVelocityForce = MaxVelocityForce;
	Def.angularVelocity = Box3D::ToB3Dir(FVector(
		FMath::DegreesToRadians(AngularVelocity.X),
		FMath::DegreesToRadians(AngularVelocity.Y),
		FMath::DegreesToRadians(AngularVelocity.Z)));
	Def.maxVelocityTorque = MaxVelocityTorque;
	Def.linearHertz = LinearHertz;
	Def.linearDampingRatio = LinearDampingRatio;
	Def.maxSpringForce = MaxSpringForce;
	Def.angularHertz = AngularHertz;
	Def.angularDampingRatio = AngularDampingRatio;
	Def.maxSpringTorque = MaxSpringTorque;
	return b3CreateMotorJoint(WorldId, &Def);
}

void UBox3DMotorJointComponent::SetLinearVelocityTarget(FVector CmPerSec)
{
	LinearVelocity = CmPerSec;
	if (b3Joint_IsValid(JointId))
	{
		b3MotorJoint_SetLinearVelocity(JointId, Box3D::ToB3(CmPerSec));
	}
}

void UBox3DMotorJointComponent::SetAngularVelocityTarget(FVector DegreesPerSec)
{
	AngularVelocity = DegreesPerSec;
	if (b3Joint_IsValid(JointId))
	{
		b3MotorJoint_SetAngularVelocity(JointId, Box3D::ToB3Dir(FVector(
			FMath::DegreesToRadians(DegreesPerSec.X),
			FMath::DegreesToRadians(DegreesPerSec.Y),
			FMath::DegreesToRadians(DegreesPerSec.Z))));
	}
}

//~ Wheel -----------------------------------------------------------------------

b3JointId UBox3DWheelJointComponent::CreateConcreteJoint(b3WorldId WorldId, b3BodyId BodyA, b3BodyId BodyB)
{
	b3WheelJointDef Def = b3DefaultWheelJointDef();
	FillBaseDef(Def.base, BodyA, BodyB);
	Def.enableSuspensionSpring = bEnableSuspensionSpring;
	Def.suspensionHertz = SuspensionHertz;
	Def.suspensionDampingRatio = SuspensionDampingRatio;
	Def.enableSuspensionLimit = bEnableSuspensionLimit;
	Def.lowerSuspensionLimit = LowerSuspensionLimit * Box3D::UEToMeters;
	Def.upperSuspensionLimit = UpperSuspensionLimit * Box3D::UEToMeters;
	Def.enableSpinMotor = bEnableSpinMotor;
	Def.maxSpinTorque = MaxSpinTorque;
	Def.spinSpeed = FMath::DegreesToRadians(SpinSpeed);
	Def.enableSteering = bEnableSteering;
	Def.steeringHertz = SteeringHertz;
	Def.steeringDampingRatio = SteeringDampingRatio;
	Def.targetSteeringAngle = FMath::DegreesToRadians(TargetSteeringAngle);
	Def.maxSteeringTorque = MaxSteeringTorque;
	Def.enableSteeringLimit = bEnableSteeringLimit;
	Def.lowerSteeringLimit = FMath::DegreesToRadians(LowerSteeringLimit);
	Def.upperSteeringLimit = FMath::DegreesToRadians(UpperSteeringLimit);
	return b3CreateWheelJoint(WorldId, &Def);
}

void UBox3DWheelJointComponent::SetSpinSpeed(float DegreesPerSec)
{
	SpinSpeed = DegreesPerSec;
	if (b3Joint_IsValid(JointId))
	{
		b3WheelJoint_SetSpinMotorSpeed(JointId, FMath::DegreesToRadians(DegreesPerSec));
	}
}

void UBox3DWheelJointComponent::SetTargetSteeringAngle(float Degrees)
{
	TargetSteeringAngle = Degrees;
	if (b3Joint_IsValid(JointId))
	{
		b3WheelJoint_SetTargetSteeringAngle(JointId, FMath::DegreesToRadians(Degrees));
	}
}
