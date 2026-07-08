#include "Box3DGrabComponent.h"

#include "Box3DBodyComponent.h"
#include "Box3DConversion.h"
#include "Box3DQueryLibrary.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Engine/World.h"
#include "box3d/box3d.h"

namespace
{
	constexpr int32 MaxHeldShapes = 16;
	constexpr uint64 PawnBit = 1ull << static_cast<int32>(EBox3DChannel::Pawn);
}

UBox3DGrabComponent::UBox3DGrabComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	GrabRayFilter.CategoryBits = -1;
	GrabRayFilter.MaskBits = (1 << static_cast<int32>(EBox3DChannel::WorldDynamic))
	                       | (1 << static_cast<int32>(EBox3DChannel::Debris));
}

void UBox3DGrabComponent::BeginPlay()
{
	Super::BeginPlay();
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		PreStepHandle = Subsystem->OnPreStep.AddUObject(this, &UBox3DGrabComponent::PreStep);
	}
}

void UBox3DGrabComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Release();
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->OnPreStep.Remove(PreStepHandle);
	}
	PreStepHandle.Reset();
	Super::EndPlay(EndPlayReason);
}

bool UBox3DGrabComponent::IsHolding() const
{
	return HeldBody.IsValid() && b3Body_IsValid(HeldBody->GetBodyId());
}

bool UBox3DGrabComponent::GrabBody(UBox3DBodyComponent* Body)
{
	if (IsHolding() || Body == nullptr || !b3Body_IsValid(Body->GetBodyId())
		|| b3Body_GetType(Body->GetBodyId()) != b3_dynamicBody || !Body->IsBodyEnabled())
	{
		return false;
	}

	const float MassKg = b3Body_GetMass(Body->GetBodyId());
	if (MaxGrabMassKg > 0.0f && MassKg > MaxGrabMassKg)
	{
		return false;
	}

	// Stop colliding with pawns while held: the body would otherwise wedge
	// against its holder's kinematic proxy and fight the carry force.
	b3ShapeId Shapes[MaxHeldShapes];
	const int32 ShapeCount = b3Body_GetShapes(Body->GetBodyId(), Shapes, MaxHeldShapes);
	SavedMaskBits.Reset();
	for (int32 Index = 0; Index < ShapeCount; ++Index)
	{
		b3Filter Filter = b3Shape_GetFilter(Shapes[Index]);
		SavedMaskBits.Add(Filter.maskBits);
		Filter.maskBits &= ~PawnBit;
		b3Shape_SetFilter(Shapes[Index], Filter, /*invokeContacts*/ true);
	}

	HeldBody = Body;
	HeldMassKg = MassKg;
	HoldTarget = Body->GetComponentLocation();
	b3Body_SetAwake(Body->GetBodyId(), true);
	OnGrabbed.Broadcast(Body);
	return true;
}

bool UBox3DGrabComponent::GrabAlongRay(FVector Start, FVector Direction, float MaxDistance)
{
	if (IsHolding() || GetWorld() == nullptr)
	{
		return false;
	}
	FBox3DHitResult Hit;
	if (!UBox3DQueryLibrary::Box3DRayCast(GetWorld(), Start,
		Start + Direction.GetSafeNormal() * MaxDistance, GrabRayFilter, Hit))
	{
		return false;
	}
	return GrabBody(Hit.Component);
}

void UBox3DGrabComponent::Release()
{
	ReleaseInternal(/*bThrown*/ false);
}

bool UBox3DGrabComponent::Throw(FVector Direction)
{
	if (!IsHolding() || HeldMassKg <= 0.0f)
	{
		return false;
	}
	const float Speed = FMath::Min(ThrowImpulseKgCmS / HeldMassKg, MaxThrowSpeed);
	HeldBody->SetLinearVelocity(Direction.GetSafeNormal() * Speed);
	ReleaseInternal(/*bThrown*/ true);
	return true;
}

void UBox3DGrabComponent::RestoreHeldFilters()
{
	if (!IsHolding())
	{
		return;
	}
	b3ShapeId Shapes[MaxHeldShapes];
	const int32 ShapeCount = b3Body_GetShapes(HeldBody->GetBodyId(), Shapes, MaxHeldShapes);
	for (int32 Index = 0; Index < FMath::Min(ShapeCount, SavedMaskBits.Num()); ++Index)
	{
		b3Filter Filter = b3Shape_GetFilter(Shapes[Index]);
		Filter.maskBits = SavedMaskBits[Index];
		b3Shape_SetFilter(Shapes[Index], Filter, /*invokeContacts*/ true);
	}
}

void UBox3DGrabComponent::ReleaseInternal(bool bThrown)
{
	UBox3DBodyComponent* Body = HeldBody.Get();
	RestoreHeldFilters();
	HeldBody.Reset();
	SavedMaskBits.Reset();
	HeldMassKg = 0.0f;
	if (Body)
	{
		OnReleased.Broadcast(Body, bThrown);
	}
}

void UBox3DGrabComponent::PreStep(float FixedDeltaTime)
{
	if (HeldBody.IsStale())
	{
		// The component died under us; nothing to restore.
		HeldBody.Reset();
		SavedMaskBits.Reset();
		HeldMassKg = 0.0f;
		return;
	}
	if (!IsHolding())
	{
		return;
	}
	UBox3DBodyComponent* Body = HeldBody.Get();
	if (!Body->IsBodyEnabled())
	{
		ReleaseInternal(false);
		return;
	}

	const b3BodyId BodyId = Body->GetBodyId();
	const FVector BodyLocation = Box3D::ToUEPos(b3Body_GetPosition(BodyId));
	const FVector ToTarget = HoldTarget - BodyLocation;
	if (ToTarget.Size() > BreakDistance)
	{
		ReleaseInternal(false);
		return;
	}

	// Velocity tracking: chase (distance x stiffness) capped at the winch
	// speed, correcting a TrackingSoftness fraction of the error per step —
	// geometric convergence, unconditionally stable, and gravity still shows
	// as a heavy object's sag.
	const FVector TargetVelocity = (ToTarget * HoldStiffness).GetClampedToMaxSize(MaxHoldSpeed) * Box3D::UEToMeters;
	const FVector Velocity = Box3D::ToUEDir(b3Body_GetLinearVelocity(BodyId));
	const FVector Force = (TargetVelocity - Velocity) * (HeldMassKg * TrackingSoftness / FixedDeltaTime);
	b3Body_ApplyForceToCenter(BodyId, Box3D::ToB3Dir(Force), /*wake*/ true);

	if (AngularDampingRate > 0.0f)
	{
		const float Keep = FMath::Max(0.0f, 1.0f - AngularDampingRate * FixedDeltaTime);
		const b3Vec3 Omega = b3Body_GetAngularVelocity(BodyId);
		b3Body_SetAngularVelocity(BodyId, b3Vec3{ Omega.x * Keep, Omega.y * Keep, Omega.z * Keep });
	}
}
