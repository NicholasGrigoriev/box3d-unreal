#include "Box3DBodyComponent.h"

#include "Box3DConversion.h"
#include "Box3DCooking.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	/// One b3 shape per authored collision element, scale baked per axis (rotation
	/// with non-uniform scale is approximated the same way UE itself does).
	/// Returns the number of shapes created.
	int32 CreateShapesFromBodySetup(b3BodyId BodyId, b3ShapeDef& ShapeDef, const UBodySetup& BodySetup,
		const FVector& Scale)
	{
		const FVector AbsScale = Scale.GetAbs();
		const b3Transform IdentityB3{ { 0, 0, 0 }, { { 0, 0, 0 }, 1.0f } };
		int32 Created = 0;

		for (const FKSphereElem& Elem : BodySetup.AggGeom.SphereElems)
		{
			const b3Sphere Sphere{ Box3D::ToB3(FVector(Elem.Center) * Scale),
								   Elem.Radius * AbsScale.GetMin() * Box3D::UEToMeters };
			b3CreateSphereShape(BodyId, &ShapeDef, &Sphere);
			++Created;
		}

		for (const FKSphylElem& Elem : BodySetup.AggGeom.SphylElems)
		{
			const FVector AxisOffset = Elem.Rotation.RotateVector(FVector(0, 0, Elem.Length * 0.5));
			const b3Capsule Capsule{
				Box3D::ToB3((FVector(Elem.Center) - AxisOffset) * Scale),
				Box3D::ToB3((FVector(Elem.Center) + AxisOffset) * Scale),
				Elem.Radius * FMath::Min(AbsScale.X, AbsScale.Y) * Box3D::UEToMeters };
			b3CreateCapsuleShape(BodyId, &ShapeDef, &Capsule);
			++Created;
		}

		for (const FKBoxElem& Elem : BodySetup.AggGeom.BoxElems)
		{
			// b3MakeScaledBoxHull exists for exactly this: editor-scaled, rotated boxes.
			const b3Vec3 HalfWidths{ Elem.X * 0.5f * Box3D::UEToMeters,
									 Elem.Y * 0.5f * Box3D::UEToMeters,
									 Elem.Z * 0.5f * Box3D::UEToMeters };
			const b3Transform LocalTransform{ Box3D::ToB3(FVector(Elem.Center)),
											  Box3D::ToB3(Elem.Rotation.Quaternion()) };
			const b3BoxHull Hull = b3MakeScaledBoxHull(HalfWidths, LocalTransform, Box3D::ToB3Dir(Scale));
			b3CreateHullShape(BodyId, &ShapeDef, &Hull.base);
			++Created;
		}

		for (const FKConvexElem& Elem : BodySetup.AggGeom.ConvexElems)
		{
			const FTransform ElemTransform = Elem.GetTransform();
			TArray<b3Vec3> Points;
			Points.Reserve(Elem.VertexData.Num());
			for (const FVector& Vertex : Elem.VertexData)
			{
				Points.Add(Box3D::ToB3(ElemTransform.TransformPosition(Vertex) * Scale));
			}
			if (Points.Num() < 4)
			{
				continue;
			}
			if (b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), 64))
			{
				// Hull shapes clone the data, so the temp cook is freed immediately.
				b3CreateHullShape(BodyId, &ShapeDef, Hull);
				b3DestroyHull(Hull);
				++Created;
			}
		}

		return Created;
	}
}

namespace
{
	// UE gameplay units -> box3d SI: one factor of 0.01 per length dimension.
	constexpr float ForceScale = 0.01f;   // kg*cm/s^2 -> N, kg*cm/s -> kg*m/s
	constexpr float TorqueScale = 0.0001f; // kg*cm^2/s^2 -> N*m, kg*cm^2/s -> kg*m^2/s
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
	ShapeDef.baseMaterial.friction = PhysicalMaterial ? PhysicalMaterial->Friction : Friction;
	ShapeDef.baseMaterial.restitution = PhysicalMaterial ? PhysicalMaterial->Restitution : Restitution;
	ShapeDef.baseMaterial.userMaterialId = static_cast<uint64>(UserMaterialId);
	ShapeDef.filter.categoryBits = Box3D::ToB3Bits(Filter.CategoryBits);
	ShapeDef.filter.maskBits = Box3D::ToB3Bits(Filter.MaskBits);
	ShapeDef.filter.groupIndex = Filter.GroupIndex;

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
				const int32 Created = CreateShapesFromBodySetup(BodyId, ShapeDef, *BodySetup, MeshScaleUE);
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
