#include "Box3DLiquidDrainActor.h"

#include "Box3DConversion.h"
#include "Box3DLiquidSourceActor.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/DrawSphereComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"

namespace
{
	/// Shares box3d.DebugLiquid with the source actor's particle overlay.
	IConsoleVariable* DebugLiquidCVar()
	{
		static IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("box3d.DebugLiquid"));
		return CVar;
	}
}

ABox3DLiquidDrainActor::ABox3DLiquidDrainActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Root->SetMobility(EComponentMobility::Movable);

	const auto MakeGizmo = [this](const TCHAR* Name, const FColor Color)
	{
		UDrawSphereComponent* Gizmo = CreateDefaultSubobject<UDrawSphereComponent>(Name);
		Gizmo->SetupAttachment(RootComponent);
		Gizmo->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Gizmo->SetCanEverAffectNavigation(false);
		Gizmo->ShapeColor = Color;
		Gizmo->bDrawOnlyIfSelected = true;
		return Gizmo;
	};
	SuctionGizmo = MakeGizmo(TEXT("SuctionRadius"), FColor(90, 160, 255));
	ConsumeGizmo = MakeGizmo(TEXT("ConsumeRadius"), FColor(255, 90, 90));
}

void ABox3DLiquidDrainActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	SuctionGizmo->SetSphereRadius(SuctionRadius);
	ConsumeGizmo->SetSphereRadius(ConsumeRadius);
}

void ABox3DLiquidDrainActor::BeginPlay()
{
	Super::BeginPlay();
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		PreStepHandle = Subsystem->OnPreStep.AddUObject(this, &ABox3DLiquidDrainActor::PreStep);
	}
}

void ABox3DLiquidDrainActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->OnPreStep.Remove(PreStepHandle);
	}
	PreStepHandle.Reset();
	Super::EndPlay(EndPlayReason);
}

void ABox3DLiquidDrainActor::PreStep(float FixedDeltaTime)
{
	if (!bEnabled)
	{
		return;
	}

	// Swallow budget: fractional carry keeps sub-1-per-step rates exact, the
	// cap stops idle time from banking a mega-gulp.
	if (DrainRate > 0.0f)
	{
		const float PerStep = DrainRate * FixedDeltaTime;
		ConsumeBudget = FMath::Min(ConsumeBudget + PerStep, FMath::Max(1.0f, PerStep));
	}

	if (AffectedSources.Num() > 0)
	{
		for (ABox3DLiquidSourceActor* Source : AffectedSources)
		{
			if (IsValid(Source))
			{
				DrainSource(*Source);
			}
		}
	}
	else
	{
		for (TActorIterator<ABox3DLiquidSourceActor> It(GetWorld()); It; ++It)
		{
			DrainSource(**It);
		}
	}
}

void ABox3DLiquidDrainActor::DrainSource(ABox3DLiquidSourceActor& Source)
{
	const FVector Center = GetActorLocation();
	const float MassKg = Source.GetParticleMassKg();
	const int32 Count = Source.GetParticleCount();

	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FVector ToCenter = Center - Source.GetParticleLocation(Index);
		const float Distance = ToCenter.Size();
		if (Distance > SuctionRadius)
		{
			continue;
		}

		if (Distance <= ConsumeRadius && (DrainRate <= 0.0f || ConsumeBudget >= 1.0f))
		{
			if (Source.ConsumeParticle(Index, ConsumeShrinkSeconds))
			{
				ConsumeBudget -= 1.0f;
				++TotalConsumed;
				continue;
			}
			// Already mid-swallow: fall through so suction keeps it converging.
		}

		if (MassKg > 0.0f && Distance > UE_KINDA_SMALL_NUMBER)
		{
			const float Falloff = FMath::Pow(1.0f - Distance / SuctionRadius, FalloffExponent);
			FVector Force = ToCenter * (SuctionStrength * Falloff * MassKg / Distance);
			if (SuctionDamping > 0.0f)
			{
				Force -= Source.GetParticleVelocity(Index) * Box3D::UEToMeters
					* (SuctionDamping * Falloff * MassKg);
			}
			if (!Force.IsNearlyZero())
			{
				Source.AddForceToParticle(Index, Force, bWakeParticles);
			}
		}
	}
}

void ABox3DLiquidDrainActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
#if !UE_BUILD_SHIPPING
	const IConsoleVariable* CVar = DebugLiquidCVar();
	if (CVar && CVar->GetBool())
	{
		DrawDebugSphere(GetWorld(), GetActorLocation(), SuctionRadius, 24, FColor(90, 160, 255));
		DrawDebugSphere(GetWorld(), GetActorLocation(), ConsumeRadius, 16, FColor(255, 90, 90));
		DrawDebugString(GetWorld(), GetActorLocation() + FVector(0, 0, 30),
			FString::Printf(TEXT("drain: %d consumed%s"), TotalConsumed, bEnabled ? TEXT("") : TEXT(" (disabled)")),
			nullptr, FColor::Green, 0.0f, true);
	}
#endif
}
