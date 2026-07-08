#include "Box3DLiquidSourceActor.h"

#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DWorldSubsystem.h"
#include "Components/ArrowComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"
#include "box3d/box3d.h"

namespace
{
	TAutoConsoleVariable<bool> CVarDebugLiquid(
		TEXT("box3d.DebugLiquid"), false,
		TEXT("Draw liquid particle points (yellow=awake, cyan=asleep) and a per-source particle count."));

	/// Physical radius floor during the shrink-out, as a fraction of full size.
	/// A sphere shrunk to zero would degenerate in the narrow phase.
	constexpr float MinShrinkFraction = 0.15f;
}

ABox3DLiquidSourceActor::ABox3DLiquidSourceActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
	Root->SetMobility(EComponentMobility::Movable);

	ParticleMeshes = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("ParticleMeshes"));
	ParticleMeshes->SetupAttachment(Root);
	ParticleMeshes->SetMobility(EComponentMobility::Movable);
	ParticleMeshes->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ParticleMeshes->SetCanEverAffectNavigation(false);
	ParticleMeshes->SetCastShadow(false);
	ParticleMeshes->NumCustomDataFloats = 1;

	DirectionArrow = CreateDefaultSubobject<UArrowComponent>(TEXT("Direction"));
	DirectionArrow->SetupAttachment(Root);
	DirectionArrow->ArrowSize = 2.0f;
	DirectionArrow->ArrowColor = FColor(90, 160, 255);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereFinder(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (SphereFinder.Succeeded())
	{
		ParticleMesh = SphereFinder.Object;
	}

	Filter.CategoryBits = 1 << static_cast<int32>(EBox3DChannel::Debris);
	Filter.MaskBits = -1;
}

void ABox3DLiquidSourceActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	ParticleMeshes->SetStaticMesh(ParticleMesh);
	ParticleMeshes->SetCastShadow(bCastShadow);
	if (ParticleMaterial)
	{
		ParticleMeshes->SetMaterial(0, ParticleMaterial);
	}
}

void ABox3DLiquidSourceActor::BeginPlay()
{
	Super::BeginPlay();
	Rng.Initialize(RandomSeed);
	bFlowing = bAutoStart;
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		PreStepHandle = Subsystem->OnPreStep.AddUObject(this, &ABox3DLiquidSourceActor::PreStep);
	}
}

void ABox3DLiquidSourceActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr)
	{
		Subsystem->OnPreStep.Remove(PreStepHandle);
	}
	PreStepHandle.Reset();
	DespawnAll();
	Super::EndPlay(EndPlayReason);
}

void ABox3DLiquidSourceActor::SpawnBurst(int32 Count)
{
	for (int32 Index = 0; Index < Count; ++Index)
	{
		if (!SpawnParticleInternal(NozzlePoint(), NozzleVelocity()))
		{
			break;
		}
	}
}

void ABox3DLiquidSourceActor::SpawnParticleAt(FVector Location, FVector Velocity)
{
	SpawnParticleInternal(Location, Velocity);
}

void ABox3DLiquidSourceActor::DespawnAll()
{
	for (const FLiquidParticle& Particle : Particles)
	{
		if (b3Body_IsValid(Particle.Body))
		{
			b3DestroyBody(Particle.Body);
		}
	}
	Particles.Reset();
	SpawnDebt = 0.0f;
	if (ParticleMeshes)
	{
		ParticleMeshes->ClearInstances();
	}
}

TArray<FVector> ABox3DLiquidSourceActor::GetParticleLocations() const
{
	TArray<FVector> Locations;
	Locations.Reserve(Particles.Num());
	for (const FLiquidParticle& Particle : Particles)
	{
		Locations.Add(Box3D::ToUEPos(b3Body_GetPosition(Particle.Body)));
	}
	return Locations;
}

FVector ABox3DLiquidSourceActor::NozzlePoint() const
{
	if (SpawnRadius <= 0.0f)
	{
		return GetActorLocation();
	}
	// Uniform disc perpendicular to the flow.
	const float Radius = SpawnRadius * FMath::Sqrt(Rng.GetFraction());
	const float Theta = Rng.GetFraction() * 2.0f * PI;
	return GetActorLocation()
		+ GetActorRightVector() * Radius * FMath::Cos(Theta)
		+ GetActorUpVector() * Radius * FMath::Sin(Theta);
}

FVector ABox3DLiquidSourceActor::NozzleVelocity() const
{
	if (InitialSpeed <= 0.0f)
	{
		return FVector::ZeroVector;
	}
	const FVector Direction = JitterAngleDeg > 0.0f
		? Rng.VRandCone(GetActorForwardVector(), FMath::DegreesToRadians(JitterAngleDeg))
		: GetActorForwardVector();
	return Direction * InitialSpeed;
}

float ABox3DLiquidSourceActor::ParticleScale(const FLiquidParticle& Particle) const
{
	float Scale = 1.0f;
	if (ParticleLifetime > 0.0f && Particle.Age > ParticleLifetime)
	{
		Scale = DespawnShrinkSeconds <= 0.0f
			? 0.0f
			: 1.0f - (Particle.Age - ParticleLifetime) / DespawnShrinkSeconds;
	}
	if (Particle.ConsumeSeconds > 0.0f)
	{
		Scale = FMath::Min(Scale, 1.0f - Particle.ConsumeAge / Particle.ConsumeSeconds);
	}
	return FMath::Clamp(Scale, 0.0f, 1.0f);
}

bool ABox3DLiquidSourceActor::SpawnParticleInternal(const FVector& Location, const FVector& VelocityCmS)
{
	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return false;
	}

	if (Particles.Num() >= MaxParticles)
	{
		if (!bRecycleOldestWhenFull)
		{
			return false;
		}
		int32 Oldest = 0;
		for (int32 Index = 1; Index < Particles.Num(); ++Index)
		{
			if (Particles[Index].Age > Particles[Oldest].Age)
			{
				Oldest = Index;
			}
		}
		ResetParticle(Oldest, Location, VelocityCmS);
		return true;
	}

	b3BodyDef BodyDef = b3DefaultBodyDef();
	BodyDef.type = b3_dynamicBody;
	BodyDef.position = Box3D::ToB3Pos(Location);
	BodyDef.linearVelocity = Box3D::ToB3(VelocityCmS);
	BodyDef.linearDamping = LinearDamping;
	BodyDef.gravityScale = GravityScale;
	BodyDef.name = "Box3DLiquid";
	// userData stays null: Box3D::ResolveComponent casts it to UObject on every
	// query hit, so raw bodies must never carry anything else.
	const b3BodyId BodyId = b3CreateBody(Subsystem->GetBox3DWorldId(), &BodyDef);

	b3ShapeDef ShapeDef = b3DefaultShapeDef();
	ShapeDef.density = ParticleDensity;
	ShapeDef.baseMaterial.friction = Friction;
	ShapeDef.baseMaterial.restitution = 0.0f;
	ShapeDef.baseMaterial.rollingResistance = RollingResistance;
	ShapeDef.filter.categoryBits = Box3D::ToB3Bits(Filter.CategoryBits);
	ShapeDef.filter.maskBits = Box3D::ToB3Bits(Filter.MaskBits);
	ShapeDef.filter.groupIndex = Filter.GroupIndex;

	b3Sphere Sphere;
	Sphere.center = b3Vec3{ 0.0f, 0.0f, 0.0f };
	Sphere.radius = ParticleRadius * Box3D::UEToMeters;
	const b3ShapeId ShapeId = b3CreateSphereShape(BodyId, &ShapeDef, &Sphere);

	if (ParticleMassKg <= 0.0f)
	{
		ParticleMassKg = b3Body_GetMass(BodyId);
	}

	FLiquidParticle& Particle = Particles.AddDefaulted_GetRef();
	Particle.Body = BodyId;
	Particle.Shape = ShapeId;
	Particle.P0 = Location;
	Particle.P1 = Location;
	return true;
}

void ABox3DLiquidSourceActor::ResetParticle(int32 Index, const FVector& Location, const FVector& VelocityCmS)
{
	FLiquidParticle& Particle = Particles[Index];
	b3Body_SetTransform(Particle.Body, Box3D::ToB3Pos(Location), Box3D::IdentityQuat);
	b3Body_SetLinearVelocity(Particle.Body, Box3D::ToB3(VelocityCmS));
	b3Body_SetAngularVelocity(Particle.Body, b3Vec3{ 0.0f, 0.0f, 0.0f });
	b3Body_SetAwake(Particle.Body, true);

	b3Sphere Sphere;
	Sphere.center = b3Vec3{ 0.0f, 0.0f, 0.0f };
	Sphere.radius = ParticleRadius * Box3D::UEToMeters;
	b3Shape_SetSphere(Particle.Shape, &Sphere);

	Particle.Age = 0.0f;
	Particle.ConsumeSeconds = 0.0f;
	Particle.ConsumeAge = 0.0f;
	Particle.P0 = Location;
	Particle.P1 = Location;
}

FVector ABox3DLiquidSourceActor::GetParticleLocation(int32 Index) const
{
	return Particles.IsValidIndex(Index)
		? Box3D::ToUEPos(b3Body_GetPosition(Particles[Index].Body))
		: FVector::ZeroVector;
}

FVector ABox3DLiquidSourceActor::GetParticleVelocity(int32 Index) const
{
	return Particles.IsValidIndex(Index)
		? Box3D::ToUEDir(b3Body_GetLinearVelocity(Particles[Index].Body)) * Box3D::MetersToUE
		: FVector::ZeroVector;
}

void ABox3DLiquidSourceActor::AddForceToParticle(int32 Index, FVector ForceNewtons, bool bWake)
{
	if (Particles.IsValidIndex(Index))
	{
		b3Body_ApplyForceToCenter(Particles[Index].Body, Box3D::ToB3Dir(ForceNewtons), bWake);
	}
}

bool ABox3DLiquidSourceActor::ConsumeParticle(int32 Index, float ShrinkSeconds)
{
	if (!Particles.IsValidIndex(Index) || Particles[Index].ConsumeSeconds > 0.0f)
	{
		return false;
	}
	// Even "instant" gets a tiny window: the destroy always happens on the next
	// fixed step, so caller loops never see indices shift under them.
	Particles[Index].ConsumeSeconds = FMath::Max(ShrinkSeconds, UE_KINDA_SMALL_NUMBER);
	Particles[Index].ConsumeAge = 0.0f;
	return true;
}

void ABox3DLiquidSourceActor::DestroyParticle(int32 Index)
{
	if (b3Body_IsValid(Particles[Index].Body))
	{
		b3DestroyBody(Particles[Index].Body);
	}
	Particles.RemoveAtSwap(Index);
}

void ABox3DLiquidSourceActor::PreStep(float FixedDeltaTime)
{
	AgeAndDespawn(FixedDeltaTime);
	SpawnFromFlow(FixedDeltaTime);
	ApplyCohesion();
}

void ABox3DLiquidSourceActor::AgeAndDespawn(float FixedDeltaTime)
{
	const float KillZMeters = KillZ * Box3D::UEToMeters;
	for (int32 Index = Particles.Num() - 1; Index >= 0; --Index)
	{
		FLiquidParticle& Particle = Particles[Index];
		Particle.Age += FixedDeltaTime;
		if (Particle.ConsumeSeconds > 0.0f)
		{
			Particle.ConsumeAge += FixedDeltaTime;
		}

		if (b3Body_GetPosition(Particle.Body).z < KillZMeters)
		{
			DestroyParticle(Index);
			continue;
		}

		// Scale only reaches 0 through an expired lifetime or a finished consume.
		const float Scale = ParticleScale(Particle);
		if (Scale <= 0.0f)
		{
			DestroyParticle(Index);
			continue;
		}
		if (Scale < 1.0f)
		{
			// Shrink the physical sphere too, so drying/swallowed particles
			// slip out from under the pile instead of leaving invisible
			// full-size bumps.
			b3Sphere Sphere;
			Sphere.center = b3Vec3{ 0.0f, 0.0f, 0.0f };
			Sphere.radius = FMath::Max(Scale, MinShrinkFraction) * ParticleRadius * Box3D::UEToMeters;
			b3Shape_SetSphere(Particle.Shape, &Sphere);
		}
	}
}

void ABox3DLiquidSourceActor::SpawnFromFlow(float FixedDeltaTime)
{
	if (!bFlowing || SpawnRate <= 0.0f)
	{
		SpawnDebt = 0.0f;
		return;
	}
	SpawnDebt += SpawnRate * FixedDeltaTime;
	while (SpawnDebt >= 1.0f)
	{
		if (!SpawnParticleInternal(NozzlePoint(), NozzleVelocity()))
		{
			// At the cap with recycling off: drop the debt so a freed slot gets
			// one particle, not a banked burst.
			SpawnDebt = 0.0f;
			return;
		}
		SpawnDebt -= 1.0f;
	}
}

void ABox3DLiquidSourceActor::ApplyCohesion()
{
	if (!bCohesion || Particles.Num() < 2 || (CohesionStrength <= 0.0f && ViscosityStrength <= 0.0f))
	{
		return;
	}

	const int32 Count = Particles.Num();
	ScratchPositions.SetNumUninitialized(Count, EAllowShrinking::No);
	ScratchVelocities.SetNumUninitialized(Count, EAllowShrinking::No);
	ScratchAwake.SetNumUninitialized(Count, EAllowShrinking::No);
	ScratchForces.SetNum(Count, EAllowShrinking::No);

	// All pair math runs in box3d units (meters, m/s, Newtons).
	const float NeighborM = CohesionRadiusScale * ParticleRadius * Box3D::UEToMeters;
	const float RestM = 2.0f * ParticleRadius * Box3D::UEToMeters;

	bool bAnyAwake = false;
	NeighborGrid.Reset();
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const b3Pos Position = b3Body_GetPosition(Particles[Index].Body);
		ScratchPositions[Index] = FVector(Position.x, Position.y, Position.z);
		ScratchVelocities[Index] = Box3D::ToUEDir(b3Body_GetLinearVelocity(Particles[Index].Body));
		ScratchAwake[Index] = b3Body_IsAwake(Particles[Index].Body);
		ScratchForces[Index] = FVector::ZeroVector;
		bAnyAwake |= ScratchAwake[Index];

		const FIntVector Cell(
			FMath::FloorToInt(Position.x / NeighborM),
			FMath::FloorToInt(Position.y / NeighborM),
			FMath::FloorToInt(Position.z / NeighborM));
		NeighborGrid.FindOrAdd(Cell).Add(Index);
	}
	if (!bAnyAwake)
	{
		return; // a settled pool costs nothing
	}

	for (int32 IndexA = 0; IndexA < Count; ++IndexA)
	{
		const FVector& PositionA = ScratchPositions[IndexA];
		const FIntVector Cell(
			FMath::FloorToInt(PositionA.X / NeighborM),
			FMath::FloorToInt(PositionA.Y / NeighborM),
			FMath::FloorToInt(PositionA.Z / NeighborM));

		for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
		for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
		for (int32 OffsetZ = -1; OffsetZ <= 1; ++OffsetZ)
		{
			const auto* CellParticles = NeighborGrid.Find(Cell + FIntVector(OffsetX, OffsetY, OffsetZ));
			if (CellParticles == nullptr)
			{
				continue;
			}
			for (const int32 IndexB : *CellParticles)
			{
				// Each pair once; sleeping pairs exert nothing on each other.
				if (IndexB <= IndexA || (!ScratchAwake[IndexA] && !ScratchAwake[IndexB]))
				{
					continue;
				}
				const FVector Delta = ScratchPositions[IndexB] - PositionA;
				const float Distance = Delta.Size();
				if (Distance >= NeighborM || Distance <= UE_KINDA_SMALL_NUMBER)
				{
					continue;
				}

				FVector PairForce = FVector::ZeroVector;
				if (CohesionStrength > 0.0f && Distance > RestM)
				{
					// Linear ramp from zero at touching to full pull at the
					// neighbour radius — surface tension, poor man's edition.
					const float Stretch = (Distance - RestM) / FMath::Max(NeighborM - RestM, UE_KINDA_SMALL_NUMBER);
					PairForce += Delta * (CohesionStrength * Stretch * ParticleMassKg / Distance);
				}
				if (ViscosityStrength > 0.0f)
				{
					PairForce += (ScratchVelocities[IndexB] - ScratchVelocities[IndexA])
						* (0.5f * ViscosityStrength * ParticleMassKg);
				}
				ScratchForces[IndexA] += PairForce;
				ScratchForces[IndexB] -= PairForce;
			}
		}
	}

	for (int32 Index = 0; Index < Count; ++Index)
	{
		// wake=false: cohesion must never keep a settling pool awake.
		if (ScratchAwake[Index] && !ScratchForces[Index].IsNearlyZero())
		{
			b3Body_ApplyForceToCenter(Particles[Index].Body, Box3D::ToB3Dir(ScratchForces[Index]), false);
		}
	}
}

void ABox3DLiquidSourceActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	UBox3DWorldSubsystem* Subsystem = GetWorld() ? GetWorld()->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
	if (Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
	{
		return;
	}

	const uint64 Step = Subsystem->GetStepCount();
	if (Step != LastVisualStep)
	{
		const bool bConsecutive = Step == LastVisualStep + 1;
		bAllAsleep = true;
		for (FLiquidParticle& Particle : Particles)
		{
			const FVector Position = Box3D::ToUEPos(b3Body_GetPosition(Particle.Body));
			Particle.P0 = bConsecutive ? Particle.P1 : Position;
			Particle.P1 = Position;
			// A sleeping particle mid-swallow still animates its shrink.
			bAllAsleep &= !b3Body_IsAwake(Particle.Body) && Particle.ConsumeSeconds <= 0.0f;
		}
		LastVisualStep = Step;
	}
	else if (bAllAsleep && !bFlowing && ParticleLifetime <= 0.0f
		&& ParticleMeshes->GetInstanceCount() == Particles.Num())
	{
		return; // settled and already rendered
	}

	UpdateVisual();

#if !UE_BUILD_SHIPPING
	if (CVarDebugLiquid.GetValueOnGameThread())
	{
		int32 AwakeCount = 0;
		for (const FLiquidParticle& Particle : Particles)
		{
			const bool bAwake = b3Body_IsAwake(Particle.Body);
			AwakeCount += bAwake ? 1 : 0;
			DrawDebugPoint(GetWorld(), Particle.P1, 6.0f, bAwake ? FColor::Yellow : FColor::Cyan, false, -1.0f);
		}
		DrawDebugString(GetWorld(), GetActorLocation() + FVector(0, 0, 30),
			FString::Printf(TEXT("liquid: %d / %d (%d awake)"), Particles.Num(), MaxParticles, AwakeCount),
			nullptr, FColor::Green, 0.0f, true);
	}
#endif
}

void ABox3DLiquidSourceActor::UpdateVisual()
{
	const int32 Count = Particles.Num();
	while (ParticleMeshes->GetInstanceCount() < Count)
	{
		ParticleMeshes->AddInstance(FTransform::Identity, /*bWorldSpace*/ true);
	}
	while (ParticleMeshes->GetInstanceCount() > Count)
	{
		ParticleMeshes->RemoveInstance(ParticleMeshes->GetInstanceCount() - 1);
	}
	if (Count == 0)
	{
		return;
	}

	const float Alpha = GetWorld()->GetSubsystem<UBox3DWorldSubsystem>()->GetFixedStepAlpha();
	const float BaseScale = 2.0f * ParticleRadius * VisualScale / FMath::Max(ParticleMeshDiameter, 1.0f);

	InstanceTransforms.SetNum(Count, EAllowShrinking::No);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const FLiquidParticle& Particle = Particles[Index];
		InstanceTransforms[Index] = FTransform(
			FQuat::Identity,
			FMath::Lerp(Particle.P0, Particle.P1, Alpha),
			FVector(BaseScale * ParticleScale(Particle)));
		// Custom data 0: normalized age for material effects (foam, fade-out).
		ParticleMeshes->SetCustomDataValue(Index, 0,
			ParticleLifetime > 0.0f ? FMath::Clamp(Particle.Age / ParticleLifetime, 0.0f, 1.0f) : 0.0f,
			/*bMarkRenderStateDirty*/ false);
	}
	ParticleMeshes->BatchUpdateInstancesTransforms(0, InstanceTransforms,
		/*bWorldSpace*/ true, /*bMarkRenderStateDirty*/ true, /*bTeleport*/ true);
}
