#include "Box3DBodyComponent.h"

#include "Box3DConversion.h"
#include "Box3DCooking.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

static TAutoConsoleVariable<bool> CVarBox3DDebugWeight(
	TEXT("box3d.DebugWeight"), false,
	TEXT("Draw GroundWeightKg probes: green = pressing a dynamic body (label shows force and target mass), ")
	TEXT("yellow = hit a non-dynamic body, red = nothing in reach."));

namespace
{
	// UE gameplay units -> box3d SI: one factor of 0.01 per length dimension.
	constexpr float ForceScale = 0.01f;   // kg*cm/s^2 -> N, kg*cm/s -> kg*m/s
	constexpr float TorqueScale = 0.0001f; // kg*cm^2/s^2 -> N*m, kg*cm^2/s -> kg*m^2/s

	// GroundWeightKg probe reach past the shape bottom. Generous enough to bridge
	// the character movement float height and small step-down gaps without pressing
	// on bodies the pawn is merely jumping over.
	constexpr float GroundWeightProbeSlackCm = 20.0f;
}

UBox3DBodyComponent::UBox3DBodyComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	// Without this, USceneComponent never calls OnUpdateTransform and component
	// moves silently stop reaching the physics body.
	bWantsOnUpdateTransform = true;
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
	const FVector ShapeScale = (bFitted ? FVector::OneVector : GetComponentTransform().GetScale3D()).GetAbs();
	CreateShape(ShapeScale);

	switch (ShapeType)
	{
	case EBox3DShapeType::Box:     ShapeBottomExtentCm = BoxHalfExtent.Z * ShapeScale.Z; break;
	case EBox3DShapeType::Sphere:  ShapeBottomExtentCm = SphereRadius * ShapeScale.GetMax(); break;
	case EBox3DShapeType::Capsule: ShapeBottomExtentCm = CapsuleHalfHeight * ShapeScale.Z; break;
	default:
		if (const UPrimitiveComponent* Primitive = FindSourcePrimitive())
		{
			ShapeBottomExtentCm = Primitive->CalcLocalBounds().BoxExtent.Z
				* FMath::Abs(Primitive->GetComponentTransform().GetScale3D().Z);
		}
		break;
	}

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
	ShapeDef.baseMaterial.friction = PhysicalMaterial ? PhysicalMaterial->Friction : Friction;
	ShapeDef.baseMaterial.restitution = PhysicalMaterial ? PhysicalMaterial->Restitution : Restitution;
	ShapeDef.baseMaterial.userMaterialId = static_cast<uint64>(UserMaterialId);
	ShapeDef.filter.categoryBits = Box3D::ToB3Bits(Filter.CategoryBits);
	ShapeDef.filter.maskBits = Box3D::ToB3Bits(Filter.MaskBits);
	ShapeDef.filter.groupIndex = Filter.GroupIndex;
	ShapeDef.isSensor = bIsSensor;
	// Sensor shapes need the flag to emit events; non-sensors need it to be seen.
	ShapeDef.enableSensorEvents = bIsSensor || bDetectableBySensors;
	ShapeDef.enableContactEvents = bEnableContactEvents;
	ShapeDef.enableHitEvents = bEnableHitEvents;

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
	case EBox3DShapeType::ConvexHull:
	case EBox3DShapeType::TriangleMesh:
	case EBox3DShapeType::CollisionAsset:
	{
		const UStaticMeshComponent* MeshComponent = Cast<UStaticMeshComponent>(FindSourcePrimitive());
		UStaticMesh* Mesh = MeshComponent ? MeshComponent->GetStaticMesh() : nullptr;
		if (Mesh == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("%s: mesh-derived shape needs an attached static mesh component; no shape created"),
				*GetPathName());
			return;
		}

		// Geometry comes from the mesh, so use the mesh component's scale. Any
		// relative offset/rotation between it and this component is ignored.
		const FVector MeshScaleUE = MeshComponent->GetComponentTransform().GetScale3D();
		const b3Vec3 MeshScale = Box3D::ToB3Dir(MeshScaleUE);

		if (ShapeType == EBox3DShapeType::CollisionAsset)
		{
			const UBodySetup* BodySetup = Mesh->GetBodySetup();
			if (BodySetup && BodySetup->AggGeom.GetElementCount() > 0)
			{
				const int32 Created = Box3D::CreateShapesFromBodySetup(BodyId, ShapeDef, *BodySetup, MeshScaleUE);
				if (Created > 0)
				{
					break;
				}
			}
			UE_LOG(LogBox3D, Log, TEXT("%s: %s has no simple collision, falling back to %s"),
				*GetPathName(), *GetNameSafe(Mesh),
				BodyType == EBox3DBodyType::Static ? TEXT("TriangleMesh") : TEXT("ConvexHull"));
		}

		bool bUseMesh = BodyType == EBox3DBodyType::Static &&
			(ShapeType == EBox3DShapeType::TriangleMesh || ShapeType == EBox3DShapeType::CollisionAsset);
		if (ShapeType == EBox3DShapeType::TriangleMesh && BodyType != EBox3DBodyType::Static)
		{
			UE_LOG(LogBox3D, Warning, TEXT("%s: TriangleMesh only contacts on static bodies; using ConvexHull instead"),
				*GetPathName());
		}

		if (bUseMesh)
		{
			if (const b3MeshData* MeshData = Box3D::GetOrCreateMeshData(Mesh))
			{
				b3CreateMeshShape(BodyId, &ShapeDef, MeshData, MeshScale);
			}
		}
		else if (const b3HullData* Hull = Box3D::GetOrCreateHullData(Mesh))
		{
			b3CreateTransformedHullShape(BodyId, &ShapeDef, Hull, b3Transform{ { 0, 0, 0 }, { { 0, 0, 0 }, 1.0f } }, MeshScale);
		}
		break;
	}
	}
}

const UPrimitiveComponent* UBox3DBodyComponent::FindSourcePrimitive() const
{
	// Nearest attached primitive: first child, else the attach parent. Any local
	// offset or rotation between it and this component is ignored.
	for (const USceneComponent* Child : GetAttachChildren())
	{
		if (const UPrimitiveComponent* ChildPrim = Cast<UPrimitiveComponent>(Child))
		{
			return ChildPrim;
		}
	}
	return Cast<UPrimitiveComponent>(GetAttachParent());
}

bool UBox3DBodyComponent::TryAutoFitShape()
{
	const UPrimitiveComponent* Primitive = FindSourcePrimitive();
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
	default:
		break; // mesh-derived shapes take geometry, not fitted extents
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

		// A teleported dynamic body must not blend from its pre-teleport segment.
		if (BodyType == EBox3DBodyType::Dynamic && Teleport != ETeleportType::None)
		{
			if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
			{
				Subsystem->InvalidateInterpolation(this);
			}
		}
	}
}

void UBox3DBodyComponent::SyncTransformFromPhysics(const FVector& NewLocation, const FQuat& NewRotation)
{
	bSyncingFromPhysics = true;
	SetWorldLocationAndRotation(NewLocation, NewRotation, /*bSweep*/ false, nullptr, ETeleportType::TeleportPhysics);
	bSyncingFromPhysics = false;
}

void UBox3DBodyComponent::ApplyGroundWeight(b3WorldId WorldId) const
{
	if (GroundWeightKg <= 0.0f || !b3Body_IsValid(BodyId) || !b3Body_IsEnabled(BodyId))
	{
		return;
	}

	const b3Vec3 Gravity = b3World_GetGravity(WorldId);
	const FVector Down = Box3D::ToUEDir(Gravity).GetSafeNormal();
	if (Down.IsNearlyZero())
	{
		return;
	}

	// The probe carries this body's own filter, so its own shape (category not in
	// its own mask for pawn setups) and anything it cannot collide with are skipped.
	b3QueryFilter QueryFilter;
	QueryFilter.categoryBits = Box3D::ToB3Bits(Filter.CategoryBits);
	QueryFilter.maskBits = Box3D::ToB3Bits(Filter.MaskBits);

	const FVector Start = GetComponentLocation();
	const FVector Translation = Down * (ShapeBottomExtentCm + GroundWeightProbeSlackCm);
	const b3RayResult Hit = b3World_CastRayClosest(WorldId,
		Box3D::ToB3Pos(Start), Box3D::ToB3(Translation), QueryFilter);

#if ENABLE_DRAW_DEBUG
	const bool bDebug = CVarBox3DDebugWeight.GetValueOnGameThread();
	if (bDebug && !Hit.hit)
	{
		// Red: nothing to press within reach — either genuinely airborne or the
		// support is not a Box3D body / not in this body's collision mask.
		DrawDebugLine(GetWorld(), Start, Start + Translation, FColor::Red, false, 0.0f, SDPG_Foreground, 0.5f);
	}
#endif
	if (!Hit.hit)
	{
		return;
	}

	const b3BodyId GroundBody = b3Shape_GetBody(Hit.shapeId);
	const bool bDynamic = b3Body_GetType(GroundBody) == b3_dynamicBody;

#if ENABLE_DRAW_DEBUG
	if (bDebug)
	{
		const FVector HitPoint = Box3D::ToUEPos(Hit.point);
		const FColor Color = bDynamic ? FColor::Green : FColor::Yellow;
		DrawDebugLine(GetWorld(), Start, HitPoint, Color, false, 0.0f, SDPG_Foreground, 0.5f);
		DrawDebugPoint(GetWorld(), HitPoint, 10.0f, Color, false, 0.0f, SDPG_Foreground);

		const UBox3DBodyComponent* HitComponent = Box3D::ResolveComponent(Hit.shapeId);
		const FString TargetName = HitComponent
			? GetNameSafe(HitComponent->GetOwner())
			: FString(ANSI_TO_TCHAR(b3Body_GetName(GroundBody)));
		const float GravityMagnitude = Box3D::ToUEDir(Gravity).Size(); // m/s^2
		DrawDebugString(GetWorld(), HitPoint + FVector(0, 0, 15),
			bDynamic
				? FString::Printf(TEXT("%.0f kg -> %.0f N on %s (%.0f kg)"),
					GroundWeightKg, GroundWeightKg * GravityMagnitude, *TargetName, b3Body_GetMass(GroundBody))
				: FString::Printf(TEXT("%s is %s - no weight applied"), *TargetName,
					b3Body_GetType(GroundBody) == b3_staticBody ? TEXT("static") : TEXT("kinematic")),
			nullptr, Color, 0.0f, true);
	}
#endif
	if (!bDynamic)
	{
		return;
	}

	// Weight force in newtons: kg * (m/s^2). Applied at the contact point so
	// standing off-center tips the support. Forces clear after each step, hence
	// the per-fixed-step application.
	const b3Vec3 Force{ Gravity.x * GroundWeightKg, Gravity.y * GroundWeightKg, Gravity.z * GroundWeightKg };
	b3Body_ApplyForce(GroundBody, Force, Hit.point, /*wake*/ true);
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

void UBox3DBodyComponent::AddImpulseAtLocation(FVector Impulse, FVector Location)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_ApplyLinearImpulse(BodyId, Box3D::ToB3Dir(Impulse * ForceScale), Box3D::ToB3Pos(Location),
			/*wake*/ true);
	}
}

void UBox3DBodyComponent::AddForceAtLocation(FVector Force, FVector Location)
{
	if (b3Body_IsValid(BodyId))
	{
		b3Body_ApplyForce(BodyId, Box3D::ToB3Dir(Force * ForceScale), Box3D::ToB3Pos(Location), /*wake*/ true);
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

void UBox3DBodyComponent::NotifyContactBegin(UBox3DBodyComponent* Other)
{
	OnContactBegin.Broadcast(Other, Other ? Other->GetOwner() : nullptr);
}

void UBox3DBodyComponent::NotifyContactEnd(UBox3DBodyComponent* Other)
{
	OnContactEnd.Broadcast(Other, Other ? Other->GetOwner() : nullptr);
}

void UBox3DBodyComponent::NotifyHit(UBox3DBodyComponent* Other, const FVector& Location, const FVector& Normal,
	float ApproachSpeed)
{
	OnHit.Broadcast(Other, Location, Normal, ApproachSpeed);
}

void UBox3DBodyComponent::NotifySensorBegin(UBox3DBodyComponent* Visitor)
{
	OnSensorBegin.Broadcast(Visitor, Visitor ? Visitor->GetOwner() : nullptr);
}

void UBox3DBodyComponent::NotifySensorEnd(UBox3DBodyComponent* Visitor)
{
	OnSensorEnd.Broadcast(Visitor, Visitor ? Visitor->GetOwner() : nullptr);
}
