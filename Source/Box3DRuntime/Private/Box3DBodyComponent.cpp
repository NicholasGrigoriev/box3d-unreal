#include "Box3DBodyComponent.h"

#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	// UE gameplay units -> box3d SI: one factor of 0.01 per length dimension.
	constexpr float ForceScale = 0.01f;   // kg*cm/s^2 -> N, kg*cm/s -> kg*m/s
	constexpr float TorqueScale = 0.0001f; // kg*cm^2/s^2 -> N*m, kg*cm^2/s -> kg*m^2/s
}

UBox3DBodyComponent::UBox3DBodyComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UBox3DBodyComponent::BeginPlay()
{
	Super::BeginPlay();
	CreateBody();
}

void UBox3DBodyComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyBody();
	Super::EndPlay(EndPlayReason);
}

void UBox3DBodyComponent::CreateBody()
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return;
	}

	const FTCHARToUTF8 NameUtf8(*GetNameSafe(GetOwner()));

	b3BodyDef BodyDef = b3DefaultBodyDef();
	BodyDef.type = static_cast<b3BodyType>(BodyType);
	BodyDef.position = Box3D::ToB3Pos(GetComponentLocation());
	BodyDef.rotation = Box3D::ToB3(GetComponentQuat());
	BodyDef.linearDamping = LinearDamping;
	BodyDef.angularDamping = AngularDamping;
	BodyDef.gravityScale = GravityScale;
	BodyDef.enableSleep = bEnableSleep;
	BodyDef.isAwake = bStartAwake;
	BodyDef.isBullet = bIsBullet;
	BodyDef.motionLocks = { MotionLocks.bLinearX, MotionLocks.bLinearY, MotionLocks.bLinearZ,
							MotionLocks.bAngularX, MotionLocks.bAngularY, MotionLocks.bAngularZ };
	BodyDef.name = NameUtf8.Get();
	BodyDef.userData = this;

	BodyId = b3CreateBody(Subsystem->GetBox3DWorldId(), &BodyDef);

	// Auto-fitted extents are already in world units; explicit extents still need
	// the component's world scale applied.
	const bool bFitted = bAutoFitShape && TryAutoFitShape();
	CreateShape(bFitted ? FVector::OneVector : GetComponentTransform().GetScale3D());

	if (BodyType == EBox3DBodyType::Kinematic)
	{
		Subsystem->RegisterKinematicBody(this);
	}
}

void UBox3DBodyComponent::DestroyBody()
{
	if (UWorld* World = GetWorld())
	{
		if (UBox3DWorldSubsystem* Subsystem = World->GetSubsystem<UBox3DWorldSubsystem>())
		{
			Subsystem->UnregisterKinematicBody(this);
		}
	}

	if (b3Body_IsValid(BodyId))
	{
		b3DestroyBody(BodyId);
	}
	BodyId = b3BodyId{};
}

void UBox3DBodyComponent::CreateShape(const FVector& WorldScale)
{
	const FVector AbsScale = WorldScale.GetAbs();

	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	ShapeDef.density = Density;
	ShapeDef.baseMaterial.friction = Friction;
	ShapeDef.baseMaterial.restitution = Restitution;

	switch (ShapeType)
	{
	case EBox3DShapeType::Box:
	{
		const b3Vec3 Half = Box3D::ToB3(BoxHalfExtent * AbsScale);
		const b3BoxHull Hull = b3MakeBoxHull(Half.x, Half.y, Half.z);
		b3CreateHullShape(BodyId, &ShapeDef, &Hull.base);
		break;
	}
	case EBox3DShapeType::Sphere:
	{
		const float Radius = SphereRadius * AbsScale.GetMax() * Box3D::UEToMeters;
		const b3Sphere Sphere{ b3Vec3{ 0.0f, 0.0f, 0.0f }, Radius };
		b3CreateSphereShape(BodyId, &ShapeDef, &Sphere);
		break;
	}
	case EBox3DShapeType::Capsule:
	{
		const float Radius = CapsuleRadius * FMath::Max(AbsScale.X, AbsScale.Y) * Box3D::UEToMeters;
		const float HalfHeight = CapsuleHalfHeight * AbsScale.Z * Box3D::UEToMeters;
		// UE half height spans to the hemisphere tip; box3d wants the segment ends.
		const float SegmentHalf = FMath::Max(HalfHeight - Radius, 0.0f);
		const b3Capsule Capsule{ b3Vec3{ 0.0f, 0.0f, -SegmentHalf }, b3Vec3{ 0.0f, 0.0f, SegmentHalf }, Radius };
		b3CreateCapsuleShape(BodyId, &ShapeDef, &Capsule);
		break;
	}
	}
}

bool UBox3DBodyComponent::TryAutoFitShape()
{
	// Nearest attached primitive: first child, else the attach parent. Any local
	// offset or rotation between it and this component is ignored in M1.
	const UPrimitiveComponent* Primitive = nullptr;
	for (const USceneComponent* Child : GetAttachChildren())
	{
		if (const UPrimitiveComponent* ChildPrim = Cast<UPrimitiveComponent>(Child))
		{
			Primitive = ChildPrim;
			break;
		}
	}
	if (Primitive == nullptr)
	{
		Primitive = Cast<UPrimitiveComponent>(GetAttachParent());
	}
	if (Primitive == nullptr)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: bAutoFitShape found no attached primitive, using explicit extents"),
			*GetPathName());
		return false;
	}

	const FVector WorldExtent =
		Primitive->CalcLocalBounds().BoxExtent * Primitive->GetComponentTransform().GetScale3D().GetAbs();

	switch (ShapeType)
	{
	case EBox3DShapeType::Box:
		BoxHalfExtent = WorldExtent;
		break;
	case EBox3DShapeType::Sphere:
		SphereRadius = WorldExtent.GetMax();
		break;
	case EBox3DShapeType::Capsule:
		CapsuleRadius = FMath::Max(WorldExtent.X, WorldExtent.Y);
		CapsuleHalfHeight = WorldExtent.Z;
		break;
	}
	return true;
}

void UBox3DBodyComponent::OnUpdateTransform(EUpdateTransformFlags UpdateTransformFlags, ETeleportType Teleport)
{
	Super::OnUpdateTransform(UpdateTransformFlags, Teleport);

	if (bSyncingFromPhysics || !b3Body_IsValid(BodyId))
	{
		return;
	}

	// Dynamic bodies are owned by the simulation: only explicit teleports pass
	// through. Kinematic bodies get smooth velocity-based targets pushed by the
	// subsystem before each step, so only teleports go straight to the body here.
	// Static bodies always follow the component.
	const bool bPushTransform =
		(BodyType == EBox3DBodyType::Static) ||
		(Teleport != ETeleportType::None);

	if (bPushTransform)
	{
		b3Body_SetTransform(BodyId, Box3D::ToB3Pos(GetComponentLocation()), Box3D::ToB3(GetComponentQuat()));
	}
}

void UBox3DBodyComponent::SyncTransformFromPhysics(const FVector& NewLocation, const FQuat& NewRotation)
{
	bSyncingFromPhysics = true;
	SetWorldLocationAndRotation(NewLocation, NewRotation, /*bSweep*/ false, nullptr, ETeleportType::TeleportPhysics);
	bSyncingFromPhysics = false;
}

bool UBox3DBodyComponent::IsSimulating() const
{
	return b3Body_IsValid(BodyId);
}

FVector UBox3DBodyComponent::GetLinearVelocity() const
{
	return b3Body_IsValid(BodyId) ? Box3D::ToUE(b3Body_GetLinearVelocity(BodyId)) : FVector::ZeroVector;
}

void UBox3DBodyComponent::SetLinearVelocity(FVector VelocityCmPerSec)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_SetLinearVelocity(BodyId, Box3D::ToB3(VelocityCmPerSec));
	}
}

FVector UBox3DBodyComponent::GetAngularVelocity() const
{
	return b3Body_IsValid(BodyId) ? Box3D::ToUEDir(b3Body_GetAngularVelocity(BodyId)) : FVector::ZeroVector;
}

void UBox3DBodyComponent::SetAngularVelocity(FVector RadiansPerSec)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_SetAngularVelocity(BodyId, Box3D::ToB3Dir(RadiansPerSec));
	}
}

void UBox3DBodyComponent::AddForce(FVector Force)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_ApplyForceToCenter(BodyId, Box3D::ToB3Dir(Force * ForceScale), /*wake*/ true);
	}
}

void UBox3DBodyComponent::AddImpulse(FVector Impulse)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_ApplyLinearImpulseToCenter(BodyId, Box3D::ToB3Dir(Impulse * ForceScale), /*wake*/ true);
	}
}

void UBox3DBodyComponent::AddTorque(FVector Torque)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_ApplyTorque(BodyId, Box3D::ToB3Dir(Torque * TorqueScale), /*wake*/ true);
	}
}

void UBox3DBodyComponent::AddAngularImpulse(FVector AngularImpulse)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_ApplyAngularImpulse(BodyId, Box3D::ToB3Dir(AngularImpulse * TorqueScale), /*wake*/ true);
	}
}

bool UBox3DBodyComponent::IsAwake() const
{
	return b3Body_IsValid(BodyId) && b3Body_IsAwake(BodyId);
}

void UBox3DBodyComponent::SetAwake(bool bAwake)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_SetAwake(BodyId, bAwake);
	}
}

void UBox3DBodyComponent::SetBodyEnabled(bool bEnabled)
{
	if (b3Body_IsValid(BodyId))
	{
		bEnabled ? b3Body_Enable(BodyId) : b3Body_Disable(BodyId);
	}
}

bool UBox3DBodyComponent::IsBodyEnabled() const
{
	return b3Body_IsValid(BodyId) && b3Body_IsEnabled(BodyId);
}

float UBox3DBodyComponent::GetMass() const
{
	return b3Body_IsValid(BodyId) ? b3Body_GetMass(BodyId) : 0.0f;
}
