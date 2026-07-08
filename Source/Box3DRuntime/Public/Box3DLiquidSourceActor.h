#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Box3DTypes.h"
#include "Math/RandomStream.h"
#include "box3d/id.h"
#include "Box3DLiquidSourceActor.generated.h"

class UArrowComponent;
class UInstancedStaticMeshComponent;
class UMaterialInterface;
class UStaticMesh;

/// A liquid emitter that approximates fluid the LiquidFun way: a swarm of small
/// raw Box3D sphere particles (low friction, zero restitution, self-colliding)
/// plus an SPH-lite pass on the fixed step — pairwise cohesion pulls stragglers
/// back into the blob and viscosity smooths relative velocities, so the swarm
/// pours, pools, and spreads instead of scattering like gravel.
///
/// Particles emit from the actor location along its forward vector (rotate the
/// actor to aim the jet; pitch -90 makes a tap). Everything that keeps the
/// budget bounded is configurable: a hard MaxParticles cap with
/// oldest-particle recycling, a lifetime with a shrink-out despawn so pools
/// evaporate instead of popping, and a KillZ for runaway drips. Settled
/// particles fall asleep like any Box3D body, so a resting pool costs almost
/// nothing to simulate or render.
///
/// Rendering is a single instanced static mesh (engine sphere by default) with
/// positions interpolated between fixed steps. Custom data slot 0 carries the
/// particle's normalized age for material effects; GetParticleLocations feeds
/// Niagara or metaball renderers if the ISM look is not enough.
///
/// Particles default to the Debris channel: explosions splash them and pawn
/// proxies wade through puddles.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DLiquidSourceActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DLiquidSourceActor();

	/// Start emitting on BeginPlay; otherwise wait for StartFlow.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Emission")
	bool bAutoStart = true;

	/// Particles per second while flowing.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emission", meta = (ClampMin = "0", ClampMax = "1000"))
	float SpawnRate = 80.0f;

	/// Ejection speed in cm/s along the actor's forward vector.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emission", meta = (ClampMin = "0"))
	float InitialSpeed = 300.0f;

	/// Random cone half-angle around the flow direction, degrees.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emission", meta = (ClampMin = "0", ClampMax = "45"))
	float JitterAngleDeg = 5.0f;

	/// Nozzle disc radius in cm, perpendicular to the flow. Keep it at least
	/// around the particle radius: a true point source spawns consecutive
	/// particles inside each other at pour speeds, and every deep overlap is
	/// pure solver waste.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emission", meta = (ClampMin = "0"))
	float SpawnRadius = 8.0f;

	/// Seed for spawn jitter, so recordings replay deterministically.
	UPROPERTY(EditAnywhere, Category = "Emission")
	int32 RandomSeed = 1234;

	/// Physical particle radius in cm. Smaller = finer liquid, more particles
	/// needed for the same volume. Creation-time only.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Particles", meta = (ClampMin = "2", ClampMax = "50"))
	float ParticleRadius = 6.0f;

	/// Particle density in kg/m^3 (water = 1000). Sets how hard the liquid
	/// shoves props and how explosions toss it.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Particles", meta = (ClampMin = "1"))
	float ParticleDensity = 1000.0f;

	/// Contact friction. Low = slippery flow, but too low turns splash droplets
	/// into ball bearings that glide for meters before stopping.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Particles", meta = (ClampMin = "0", ClampMax = "1"))
	float Friction = 0.15f;

	/// Rolling resistance; this is what actually parks splash droplets and
	/// keeps a settled pool from creeping.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Particles", meta = (ClampMin = "0", ClampMax = "1"))
	float RollingResistance = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Particles", meta = (ClampMin = "0"))
	float LinearDamping = 0.05f;

	/// Gravity multiplier per particle: 1 = water, lower floats like foam,
	/// 0 = weightless blobs.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Particles", meta = (ClampMin = "0", ClampMax = "8"))
	float GravityScale = 1.0f;

	/// Collision filter for every particle. Default: Debris vs everything.
	/// Self-collision must stay on — the particles ARE the liquid volume.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Particles")
	FBox3DFilter Filter;

	/// Pairwise attraction + velocity smoothing between nearby particles. Off =
	/// dry granular flow (sand); on = coherent blobs and puddles.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Liquid")
	bool bCohesion = true;

	/// Neighbour interaction radius in particle radii. Larger binds the blob
	/// tighter but costs more pairs.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Liquid", meta = (EditCondition = "bCohesion", ClampMin = "2.1", ClampMax = "6"))
	float CohesionRadiusScale = 3.0f;

	/// Peak attraction toward neighbours, in m/s^2 at full stretch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Liquid", meta = (EditCondition = "bCohesion", ClampMin = "0", ClampMax = "200"))
	float CohesionStrength = 30.0f;

	/// Relative-velocity damping rate between neighbours, 1/s. High values with
	/// many neighbours can overshoot at 60 Hz; ~5 is honey enough.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Liquid", meta = (EditCondition = "bCohesion", ClampMin = "0", ClampMax = "20"))
	float ViscosityStrength = 4.0f;

	/// Hard particle budget. At the cap the oldest particle is recycled into
	/// each new spawn (bRecycleOldestWhenFull), so flow never stops — the pool
	/// just stops growing.
	///
	/// THE performance dial. Cost is linear: a dense pool runs ~10 contacts
	/// per particle, all solved SubStepCount times per fixed step, and while
	/// the tap flows nothing sleeps (recycling keeps raining). Measured on a
	/// 16-core dev box: 400 particles ~ 3.3 ms/step physics — and once a frame
	/// runs long, the fixed-step accumulator plays catch-up with up to
	/// MaxStepsPerTick steps per frame, multiplying that cost right when you
	/// can least afford it. If you need more visible liquid, raising
	/// ParticleRadius buys volume far cheaper than raising the cap.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Budget", meta = (ClampMin = "1", ClampMax = "2000"))
	int32 MaxParticles = 200;

	/// Seconds a particle lives before despawning. 0 = forever (the cap still
	/// bounds the total).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Budget", meta = (ClampMin = "0"))
	float ParticleLifetime = 20.0f;

	/// Expired particles shrink over this many seconds instead of popping, so
	/// pools visibly dry up. 0 = instant despawn.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Budget", meta = (ClampMin = "0"))
	float DespawnShrinkSeconds = 1.0f;

	/// When the cap is hit while flowing, reuse the oldest particle. Off =
	/// emission stalls until something despawns.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Budget")
	bool bRecycleOldestWhenFull = true;

	/// Particles below this world Z despawn immediately (leaks into the void).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Budget")
	float KillZ = -100000.0f;

	/// Mesh instanced per particle. Default: the engine's 100 cm sphere.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	TObjectPtr<UStaticMesh> ParticleMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> ParticleMaterial;

	/// Unscaled diameter of ParticleMesh in cm, used to size instances.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering", meta = (ClampMin = "1"))
	float ParticleMeshDiameter = 100.0f;

	/// Visual overdraw: instances render this much larger than the physical
	/// sphere so pools read as a continuous surface instead of marbles.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering", meta = (ClampMin = "0.5", ClampMax = "3"))
	float VisualScale = 1.4f;

	/// Off by default: hundreds of shadow-casting instances are not cheap.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	bool bCastShadow = false;

	UFUNCTION(BlueprintCallable, Category = "Liquid")
	void StartFlow() { bFlowing = true; }

	UFUNCTION(BlueprintCallable, Category = "Liquid")
	void StopFlow() { bFlowing = false; }

	UFUNCTION(BlueprintPure, Category = "Liquid")
	bool IsFlowing() const { return bFlowing; }

	/// Emit Count particles immediately from the nozzle — splashes, shatters.
	UFUNCTION(BlueprintCallable, Category = "Liquid")
	void SpawnBurst(int32 Count);

	/// Place one particle at an explicit world location (cm, cm/s) — droplets
	/// at a bullet impact, drips from geometry. Honours the cap.
	UFUNCTION(BlueprintCallable, Category = "Liquid")
	void SpawnParticleAt(FVector Location, FVector Velocity);

	UFUNCTION(BlueprintCallable, Category = "Liquid")
	void DespawnAll();

	UFUNCTION(BlueprintPure, Category = "Liquid")
	int32 GetParticleCount() const { return Particles.Num(); }

	/// Live particle centers in world space — feed Niagara or a metaball
	/// renderer instead of (or on top of) the built-in instanced mesh.
	UFUNCTION(BlueprintPure, Category = "Liquid")
	TArray<FVector> GetParticleLocations() const;

	/// Live world-space center of one particle (cm).
	UFUNCTION(BlueprintPure, Category = "Liquid")
	FVector GetParticleLocation(int32 Index) const;

	/// Live velocity of one particle (cm/s).
	UFUNCTION(BlueprintPure, Category = "Liquid")
	FVector GetParticleVelocity(int32 Index) const;

	/// Mass of one full-size particle in kg (0 until the first spawn).
	UFUNCTION(BlueprintPure, Category = "Liquid")
	float GetParticleMassKg() const { return ParticleMassKg; }

	/// Continuous force in Newtons on one particle — drains, currents,
	/// attractors. Call during the fixed step (OnPreStep) or the force
	/// under- or over-doses with frame rate.
	UFUNCTION(BlueprintCallable, Category = "Liquid")
	void AddForceToParticle(int32 Index, FVector ForceNewtons, bool bWake = true);

	/// Swallow one particle: it shrinks out over ShrinkSeconds and despawns.
	/// Never removes it inside this call (indices stay valid for the caller's
	/// loop); the actual destroy happens on the next fixed step. Returns false
	/// for invalid indices or particles already being consumed.
	UFUNCTION(BlueprintCallable, Category = "Liquid")
	bool ConsumeParticle(int32 Index, float ShrinkSeconds = 0.2f);

	//~ AActor
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	/// All simulation runs on the fixed step (aging/despawn, emission, cohesion)
	/// so behaviour is frame-rate independent and recordings stay deterministic.
	void PreStep(float FixedDeltaTime);
	void AgeAndDespawn(float FixedDeltaTime);
	void SpawnFromFlow(float FixedDeltaTime);
	void ApplyCohesion();

	bool SpawnParticleInternal(const FVector& Location, const FVector& VelocityCmS);
	void ResetParticle(int32 Index, const FVector& Location, const FVector& VelocityCmS);
	void DestroyParticle(int32 Index);
	void UpdateVisual();
	FVector NozzlePoint() const;
	FVector NozzleVelocity() const;

	struct FLiquidParticle
	{
		b3BodyId Body = {};
		b3ShapeId Shape = {};
		float Age = 0.0f;
		/// > 0 while being consumed (drain swallow): shrink window and progress.
		float ConsumeSeconds = 0.0f;
		float ConsumeAge = 0.0f;
		FVector P0 = FVector::ZeroVector;
		FVector P1 = FVector::ZeroVector;
	};

	/// 1 while alive, falling to 0 across the lifetime-shrink and/or consume
	/// window (whichever is smaller).
	float ParticleScale(const FLiquidParticle& Particle) const;
	TArray<FLiquidParticle> Particles;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rendering", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UInstancedStaticMeshComponent> ParticleMeshes;

	UPROPERTY()
	TObjectPtr<UArrowComponent> DirectionArrow;

	FRandomStream Rng;
	FDelegateHandle PreStepHandle;
	float SpawnDebt = 0.0f;
	bool bFlowing = false;
	/// Mass of one full-size particle in kg, cached at first spawn.
	float ParticleMassKg = 0.0f;
	uint64 LastVisualStep = 0;
	bool bAllAsleep = false;
	/// Any particle close enough to its lifetime that shrink animation is (or
	/// is about to be) running — blocks the settled-visual skip.
	bool bAnyNearExpiry = false;

	/// Reused per-step/per-frame buffers.
	TMap<FIntVector, TArray<int32, TInlineAllocator<8>>> NeighborGrid;
	TArray<FVector> ScratchPositions;
	TArray<FVector> ScratchVelocities;
	TArray<FVector> ScratchForces;
	TArray<bool> ScratchAwake;
	TArray<FTransform> InstanceTransforms;
	/// Last age bucket written to each ISM instance's custom data — ages move
	/// slowly, so quantized writes skip ~all of the per-instance update cost.
	TArray<uint8> InstanceAgeBuckets;
};
