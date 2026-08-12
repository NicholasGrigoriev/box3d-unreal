#include "Box3DDestructibleComponent.h"

#include "Box3DFracturedActor.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

ABox3DFracturedActor* UBox3DDestructibleComponent::ApplyImpact(FVector WorldLocation, float EnergyJoules)
{
	if (bFractured)
	{
		return nullptr;
	}

	const int32 CellCount = Box3D::Destruction::EnergyToCellCount(EnergyJoules, EnergyToCells);
	if (CellCount < 1)
	{
		return nullptr;
	}

	UStaticMeshComponent* Target = ResolveTargetMesh();
	if (Target == nullptr)
	{
		UE_LOG(LogBox3D, Warning, TEXT("%s: no static mesh component on %s to fracture"),
			*GetNameSafe(this), *GetNameSafe(GetOwner()));
		return nullptr;
	}

	FBox3DFractureMeshParams Params;
	Params.Fracture.Seed = Seed;
	Params.Fracture.CellCount = CellCount;
	Params.Fracture.ImpactPoint = WorldLocation;
	Params.Fracture.ImpactRadius = FMath::Max(Target->Bounds.SphereRadius * 0.3, 10.0);
	Params.Fracture.RadialBias = RadialBias;
	Params.Fracture.MinFragmentVolume = MinFragmentVolume;
	Params.CoreMaterial = CoreMaterial;
	Params.MaterialToughness = MaterialToughness;
	Params.FragmentDensity = FragmentDensity;
	Params.bStructural = bStructural;
	Params.TensionStrengthPa = TensionStrengthPa;
	Params.CompressionStrengthPa = CompressionStrengthPa;
	Params.ShearStrengthPa = ShearStrengthPa;
	Params.SustainedOverloadHealthPerSecond = SustainedOverloadHealthPerSecond;
	Params.Tiers = TierThresholds;
	Params.DebrisSpeed = DebrisSpeed;
	Params.DebrisSystem = DebrisSystem;

	ABox3DFracturedActor* Actor = Box3D::FractureMesh(Target, Params);
	if (Actor == nullptr)
	{
		return nullptr;
	}

	bFractured = true;
	FracturedActor = Actor;
	UE_LOG(LogBox3D, Log, TEXT("%s: %.0f J impact -> %d cells, %d fragments"),
		*GetNameSafe(GetOwner()), EnergyJoules, CellCount, Actor->GetFragmentCount());
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
