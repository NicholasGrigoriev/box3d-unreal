#include "Box3DWindActor.h"

#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/ArrowComponent.h"
#include "Components/SplineComponent.h"
#include "Engine/World.h"
#include "box3d/box3d.h"

ABox3DWindActor::ABox3DWindActor()
{
	PrimaryActorTick.bCanEverTick = false;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Root->SetMobility(EComponentMobility::Movable);

	WindSpline = CreateDefaultSubobject<USplineComponent>(TEXT("WindSpline"));
	WindSpline->SetupAttachment(Root);
	WindSpline->SetMobility(EComponentMobility::Movable);

	// Editor-only direction gizmo (arrows are hidden in game by default).
	DirectionArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("Direction"));
	DirectionArrow->SetupAttachment(Root);
	DirectionArrow->ArrowSize = 3.0f;
	DirectionArrow->ArrowColor = FColor(120, 200, 255);
}

void ABox3DWindActor::BeginPlay()
{
	Super::BeginPlay();
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		PreStepHandle = Subsystem->OnPreStep.AddUObject(this, &ABox3DWindActor::ApplyWind);
	}
}

void ABox3DWindActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->OnPreStep.Remove(PreStepHandle);
	}
	PreStepHandle.Reset();
	Super::EndPlay(EndPlayReason);
}

FVector ABox3DWindActor::GetWindVelocityAt(FVector Position) const
{
	// Gust: slow Perlin wobble on the overall strength, phase-locked to the
	// fixed-step clock so it is deterministic.
	float Speed = WindSpeed;
	if (GustAmount > 0.0f && GustFrequency > 0.0f)
	{
		Speed *= 1.0f + GustAmount * FMath::PerlinNoise1D(WindTime * GustFrequency * 2.0f);
	}

	FVector Direction = FVector::ZeroVector;
	float DistanceForFalloff = FVector::Dist(Position, GetActorLocation());
	switch (WindMode)
	{
	case EBox3DWindMode::Directional:
		Direction = GetActorForwardVector();
		break;

	case EBox3DWindMode::Turbulence:
	{
		const FVector Sample = Position / FMath::Max(TurbulenceScale, 10.0f)
			+ FVector(WindTime * TurbulenceSpeed);
		// Three decorrelated Perlin samples make a swirling vector field.
		Direction = FVector(
			FMath::PerlinNoise3D(Sample),
			FMath::PerlinNoise3D(Sample + FVector(31.7, 0.0, 0.0)),
			FMath::PerlinNoise3D(Sample + FVector(0.0, 67.3, 0.0)));
		Speed *= FMath::Min(Direction.Size() * 1.5, 1.0);
		Direction = Direction.GetSafeNormal(UE_SMALL_NUMBER, GetActorForwardVector());
		break;
	}

	case EBox3DWindMode::Spline:
	{
		const float Key = WindSpline->FindInputKeyClosestToWorldLocation(Position);
		Direction = WindSpline->GetDirectionAtSplineInputKey(Key, ESplineCoordinateSpace::World);
		DistanceForFalloff = FVector::Dist(Position,
			WindSpline->GetLocationAtSplineInputKey(Key, ESplineCoordinateSpace::World));
		break;
	}

	case EBox3DWindMode::Vortex:
	{
		const FVector Axis = GetActorUpVector();
		FVector Radial = Position - GetActorLocation();
		Radial -= Axis * FVector::DotProduct(Radial, Axis);
		const FVector RadialDir = Radial.GetSafeNormal(UE_SMALL_NUMBER, GetActorForwardVector());
		const FVector Tangential = FVector::CrossProduct(Axis, RadialDir);
		return (Tangential * Speed - RadialDir * InwardSpeed + Axis * UpdraftSpeed)
			* FMath::Pow(FMath::Clamp(1.0f - DistanceForFalloff / Radius, 0.0f, 1.0f), FalloffExponent);
	}
	}

	const float Falloff = FMath::Pow(
		FMath::Clamp(1.0f - DistanceForFalloff / Radius, 0.0f, 1.0f), FalloffExponent);
	return Direction * Speed * Falloff;
}

void ABox3DWindActor::ApplyWind(float FixedDeltaTime)
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()) || WindSpeed <= 0.0f)
	{
		return;
	}
	WindTime += FixedDeltaTime;

	// Broad-phase gather of everything the field could reach.
	FBox Range;
	if (WindMode == EBox3DWindMode::Spline)
	{
		Range = WindSpline->Bounds.GetBox().ExpandBy(Radius);
	}
	else
	{
		Range = FBox(GetActorLocation() - FVector(Radius), GetActorLocation() + FVector(Radius));
	}
	b3AABB QueryBox;
	QueryBox.lowerBound = Box3D::ToB3(Range.Min);
	QueryBox.upperBound = Box3D::ToB3(Range.Max);

	using FShapeArray = TArray<b3ShapeId, TInlineAllocator<256>>;
	FShapeArray Shapes;
	b3World_OverlapAABB(Subsystem->GetBox3DWorldId(), QueryBox, b3DefaultQueryFilter(),
		[](b3ShapeId ShapeId, void* Context) -> bool
		{
			static_cast<FShapeArray*>(Context)->Add(ShapeId);
			return true;
		}, &Shapes);

	for (const b3ShapeId ShapeId : Shapes)
	{
		const b3BodyId BodyId = b3Shape_GetBody(ShapeId);
		if (b3Body_GetType(BodyId) != b3_dynamicBody || (!bWakeBodies && !b3Body_IsAwake(BodyId)))
		{
			continue;
		}

		const b3AABB ShapeBox = b3Shape_GetAABB(ShapeId);
		const FVector Center = Box3D::ToUE(b3Vec3{
			0.5f * (ShapeBox.lowerBound.x + ShapeBox.upperBound.x),
			0.5f * (ShapeBox.lowerBound.y + ShapeBox.upperBound.y),
			0.5f * (ShapeBox.lowerBound.z + ShapeBox.upperBound.z) });

		const FVector Wind = GetWindVelocityAt(Center);
		if (Wind.IsNearlyZero())
		{
			continue;
		}

		// Drag model: force chases the relative flow, so bodies asymptote to
		// wind speed instead of accelerating forever. Cross-section from the
		// shape AABB (mean projected area of a box = half its surface / 2).
		const FVector RelativeFlow = (Wind - Box3D::ToUE(b3Body_GetLinearVelocity(BodyId))) * Box3D::UEToMeters;
		const float Ex = 0.5f * (ShapeBox.upperBound.x - ShapeBox.lowerBound.x);
		const float Ey = 0.5f * (ShapeBox.upperBound.y - ShapeBox.lowerBound.y);
		const float Ez = 0.5f * (ShapeBox.upperBound.z - ShapeBox.lowerBound.z);
		const float Area = 2.0f * (Ex * Ey + Ey * Ez + Ez * Ex);

		const FVector ForceNewtons = RelativeFlow * (Drag * Area);
		b3Body_ApplyForceToCenter(BodyId, Box3D::ToB3Dir(ForceNewtons), bWakeBodies);
	}
}
