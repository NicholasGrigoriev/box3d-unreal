#include "Box3DDestructibleComponent.h"

#include "Box3DFracturedActor.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

bool UBox3DDestructibleComponent::BuildDestructionEvent(FVector WorldLocation,
	float EnergyJoules, FBox3DDestructionEvent& OutEvent) const
{
	OutEvent = FBox3DDestructionEvent();
	const UBox3DWorldSubsystem* Subsystem = GetWorld() != nullptr
		? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>()
		: nullptr;
	if (bFractured || Subsystem == nullptr || !Subsystem->IsSimulationAuthority())
	{
		return false;
	}

	const int32 CellCount = Box3D::Destruction::EnergyToCellCount(EnergyJoules, EnergyToCells);
	UStaticMeshComponent* Target = ResolveTargetMesh();
	if (CellCount < 1 || Target == nullptr)
	{
		return false;
	}

	OutEvent.MeshId = Box3D::GetDestructionMeshId(*Target);
	OutEvent.Impact = WorldLocation;
	OutEvent.Seed = Seed;
	OutEvent.Params.CellCount = CellCount;
	OutEvent.Params.ImpactRadius = FMath::Max(Target->Bounds.SphereRadius * 0.3, 10.0);
	OutEvent.Params.RadialBias = RadialBias;
	OutEvent.Params.MinFragmentVolume = MinFragmentVolume;
	OutEvent.Params.MaterialToughness = MaterialToughness;
	OutEvent.Params.FragmentDensity = FragmentDensity;
	OutEvent.Params.bStartAsleep = false;
	OutEvent.Params.Tiers = TierThresholds;
	OutEvent.Params.DebrisSpeed = DebrisSpeed;
	OutEvent.Params.bStructural = bStructural;
	OutEvent.Params.TensionStrengthPa = TensionStrengthPa;
	OutEvent.Params.CompressionStrengthPa = CompressionStrengthPa;
	OutEvent.Params.ShearStrengthPa = ShearStrengthPa;
	OutEvent.Params.SustainedOverloadHealthPerSecond = SustainedOverloadHealthPerSecond;
	return OutEvent.MeshId != NAME_None;
}

ABox3DFracturedActor* UBox3DDestructibleComponent::ApplyDestructionEvent(
	const FBox3DDestructionEvent& Event, int64 AuthorityLayoutHash,
	FBox3DDestructionEventResult& OutResult)
{
	OutResult = FBox3DDestructionEventResult();
	if (bFractured || Event.Params.CellCount < 1)
	{
		return nullptr;
	}

	UStaticMeshComponent* Target = ResolveTargetMesh();
	if (Target == nullptr || Event.MeshId == NAME_None || Box3D::GetDestructionMeshId(*Target) != Event.MeshId)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: destruction event mesh id does not match target"),
			*GetNameSafe(GetOwner()));
		return nullptr;
	}

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = Event.Seed;
	Params.Fracture.CellCount = Event.Params.CellCount;
	Params.Fracture.ImpactPoint = Event.Impact;
	Params.Fracture.ImpactRadius = Event.Params.ImpactRadius;
	Params.Fracture.RadialBias = Event.Params.RadialBias;
	Params.Fracture.MinFragmentVolume = Event.Params.MinFragmentVolume;
	Params.CoreMaterial = CoreMaterial;
	Params.MaterialToughness = Event.Params.MaterialToughness;
	Params.FragmentDensity = Event.Params.FragmentDensity;
	Params.bStartAsleep = Event.Params.bStartAsleep;
	Params.bStructural = Event.Params.bStructural;
	Params.TensionStrengthPa = Event.Params.TensionStrengthPa;
	Params.CompressionStrengthPa = Event.Params.CompressionStrengthPa;
	Params.ShearStrengthPa = Event.Params.ShearStrengthPa;
	Params.SustainedOverloadHealthPerSecond = Event.Params.SustainedOverloadHealthPerSecond;
	Params.Tiers = Event.Params.Tiers;
	Params.DebrisSpeed = Event.Params.DebrisSpeed;
	Params.DebrisSystem = DebrisSystem;

	const UBox3DWorldSubsystem* Subsystem = GetWorld() != nullptr
		? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>()
		: nullptr;
	OutResult.bVisualOnly = Subsystem == nullptr || !Subsystem->IsSimulationAuthority();
	ABox3DFracturedActor* Actor = OutResult.bVisualOnly
		? Box3D::RegenerateFractureVisuals(Target, Params)
		: Box3D::FractureMesh(Target, Params);
	if (Actor == nullptr)
	{
		return nullptr;
	}

	const uint32 LayoutHash = Box3D::Fracture::FractureLayoutHash(Actor->GetFragments());
	const uint32 BondHash = Box3D::Destruction::InitialBondHealthHash(
		Actor->GetFragments(), Event.Params.FragmentDensity);
	OutResult.bApplied = true;
	OutResult.FractureLayoutHash = static_cast<int64>(LayoutHash);
	OutResult.BondHealthHash = static_cast<int64>(BondHash);
	if (AuthorityLayoutHash > 0)
	{
		OutResult.bLayoutHashValidated = true;
		OutResult.bNeedsServerCorrection = static_cast<uint32>(AuthorityLayoutHash) != LayoutHash;
	}

	bFractured = true;
	FracturedActor = Actor;
	if (OutResult.bNeedsServerCorrection)
	{
		OnDestructionCorrectionRequired.Broadcast(
			Event.MeshId, AuthorityLayoutHash, OutResult.FractureLayoutHash);
	}
	return Actor;
}

ABox3DFracturedActor* UBox3DDestructibleComponent::ApplyImpact(FVector WorldLocation, float EnergyJoules)
{
	FBox3DDestructionEvent Event;
	if (!BuildDestructionEvent(WorldLocation, EnergyJoules, Event))
	{
		return nullptr;
	}

	FBox3DDestructionEventResult Result;
	ABox3DFracturedActor* Actor = ApplyDestructionEvent(Event, 0, Result);
	if (Actor == nullptr)
	{
		return nullptr;
	}

	OnDestructionEventGenerated.Broadcast(Event, Result.FractureLayoutHash);
	UE_LOG(LogBox3D, Log, TEXT("%s: %.0f J impact -> %d cells, %d fragments"),
		*GetNameSafe(GetOwner()), EnergyJoules, Event.Params.CellCount, Actor->GetFragmentCount());
	return Actor;
}

UStaticMeshComponent* UBox3DDestructibleComponent::ResolveTargetMesh() const
{
	const AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return nullptr;
	}
	if (UStaticMeshComponent* Root = Cast<UStaticMeshComponent>(Owner->GetRootComponent()))
	{
		return Root;
	}
	return Owner->FindComponentByClass<UStaticMeshComponent>();
}

void UBox3DDestructibleComponent::BeginPlay()
{
	Super::BeginPlay();
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->RegisterDestructible(this);
	}
}

void UBox3DDestructibleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->UnregisterDestructible(this);
	}
	Super::EndPlay(EndPlayReason);
}
