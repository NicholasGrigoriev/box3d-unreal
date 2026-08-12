#pragma once

#include "CoreMinimal.h"
#include "Box3DDestruction.h"
#include "Box3DFracture.h"
#include "Box3DStructure.h"
#include "Box3DStress.h"
#include "GameFramework/Actor.h"
#include "box3d/id.h"
#include "Box3DFracturedActor.generated.h"

class UMaterialInterface;
class UNiagaraSystem;
class UProceduralMeshComponent;
class UStaticMeshComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FBox3DFracturedWeldBrokeSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FBox3DStructureStressedSignature,
	int32, BondIndex, float, OverloadRatio, float, BondHealth);

/// Which source ResolveFractureProxy built the convex fracture proxy from, in
/// preference order. Packaged builds must hit AuthoredConvex or SimpleCollision:
/// RenderVertices reads LOD0 CPU vertex data, which cooking normally strips, so
/// meshes meant to fracture in shipping need authored simple collision.
enum class EBox3DFractureProxySource : uint8
{
	/// No usable geometry — fracture is not possible for this component.
	None,
	/// Convex hull of the body setup's authored convex elements (AggGeom).
	AuthoredConvex,
	/// Convex hull of the simple-collision primitives (box/sphere/capsule elems).
	SimpleCollision,
	/// Convex hull of LOD0 render vertices — editor-only fallback, logs a warning.
	RenderVertices,
};

/// Inputs for Box3D::FractureMesh. Unlike the pure-geometry fracture core,
/// Fracture.ImpactPoint here is in WORLD space (cm) — FractureMesh converts it
/// into the source component's space before fracturing.
struct FBox3DFractureMeshParams
{
	Box3D::Fracture::FFractureParams Fracture;

	/// Material for interior (fracture-cut) faces — typically triplanar so the
	/// arbitrary cut geometry needs no UV authoring. Null falls back to the
	/// source component's material.
	UMaterialInterface* CoreMaterial = nullptr;

	/// Weld strength per unit of shared-face area (N/cm^2): each weld's break
	/// force is SharedFaceArea x MaterialToughness. <= 0 makes welds unbreakable.
	float MaterialToughness = 50.0f;

	/// Fragment density in kg/m^3.
	float FragmentDensity = 400.0f;

	/// Spawn the fragment bodies asleep (welded rubble at rest). Defaults off
	/// because fracture usually follows an impact that should scatter the pieces.
	bool bStartAsleep = false;

	/// Volume thresholds routing fragments into Body / Debris / Dust tiers.
	/// Defaults keep every fragment in the Body tier (the D2 behavior).
	FBox3DTierThresholds Tiers;

	/// Radial scatter speed (cm/s) recorded for Debris-tier burst fragments.
	float DebrisSpeed = 300.0f;

	/// Niagara system spawned with the Debris-tier burst arrays (user parameters
	/// DebrisPositions / DebrisVelocities / DebrisSizes, world space). Null skips
	/// the spawn — the burst arrays stay inspectable data either way.
	UNiagaraSystem* DebrisSystem = nullptr;

	/// Anchored structural assembly (D4): fragments spawn as STATIC bodies, a
	/// bond graph tracks connectivity, and auto-detected anchors (chunks touching
	/// static world geometry) hold the assembly up. When a chunk is destroyed or
	/// a weld snaps, islands with no path to an anchor promote static->dynamic
	/// under the per-tick budget and fall as welded clumps. Assemblies with no
	/// anchors at build fall back to plain dynamic rubble (the default behavior).
	bool bStructural = false;

	/// Sustained-load capacities in pascals. Non-positive disables that mode.
	float TensionStrengthPa = 1.0e6f;
	float CompressionStrengthPa = 5.0e6f;
	float ShearStrengthPa = 1.0e6f;

	/// Health removed per second for each unit of overload above capacity.
	float SustainedOverloadHealthPerSecond = 1.0f;
};

/// A fractured static mesh: one ProceduralMeshComponent carrying up to two
/// sections per fragment — exterior faces (inherited from the proxy surface)
/// with the source component's material, interior cut faces with CoreMaterial.
/// Spawned by Box3D::FractureMesh, which also swaps out the source component
/// (hidden, Chaos collision off, static mirror body removed).
///
/// Each fragment gets a dynamic b3 hull body; adjacent fragments (cell
/// adjacency from the fracture core, not bounds proximity) are welded with
/// break force SharedFaceArea x MaterialToughness. Tick snaps overloaded welds
/// (OnWeldBroken) and drives the PMC sections from the body transforms.
UCLASS(BlueprintType, NotBlueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DFracturedActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DFracturedActor();

	/// Material applied to interior (cut) faces. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	TObjectPtr<UMaterialInterface> CoreMaterial;

	/// Weld strength per unit of shared-face area (N/cm^2). Each weld snaps when
	/// its constraint force exceeds SharedFaceArea x MaterialToughness; <= 0
	/// makes welds unbreakable. Sleeping assemblies report zero joint force, so
	/// something must wake the fragments before welds can snap. Set before
	/// InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (ClampMin = "0"))
	float MaterialToughness = 50.0f;

	/// Fragment density in kg/m^3. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (ClampMin = "1"))
	float FragmentDensity = 400.0f;

	/// Spawn the fragment bodies asleep. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	bool bStartAsleep = false;

	/// Anchored structural assembly (D4): see FBox3DFractureMeshParams::
	/// bStructural. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	bool bStructural = false;

	/// Sustained structural tension capacity in pascals; <= 0 disables tension damage.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture|Stress", meta = (ClampMin = "0"))
	float TensionStrengthPa = 1.0e6f;

	/// Sustained structural compression capacity in pascals; <= 0 disables compression damage.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture|Stress", meta = (ClampMin = "0"))
	float CompressionStrengthPa = 5.0e6f;

	/// Sustained structural shear capacity in pascals; <= 0 disables shear damage.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture|Stress", meta = (ClampMin = "0"))
	float ShearStrengthPa = 1.0e6f;

	/// Health removed per second for each unit by which stress exceeds capacity.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture|Stress", meta = (ClampMin = "0"))
	float SustainedOverloadHealthPerSecond = 1.0f;

	/// Volume thresholds routing fragments into tiers. Set before
	/// InitializeFragments; non-Body fragments get no sections, bodies, or welds.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	FBox3DTierThresholds TierThresholds;

	/// Radial scatter speed (cm/s) of Debris-tier burst velocities. Set before
	/// InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (ClampMin = "0"))
	float DebrisSpeed = 300.0f;

	/// Impact point in actor space that debris scatters away from. Set before
	/// InitializeFragments (FractureMesh fills it from the fracture params).
	FVector DebrisImpactPoint = FVector::ZeroVector;

	/// Niagara system handed the Debris-tier burst on InitializeFragments as
	/// world-space user array parameters: DebrisPositions (Position/Vector),
	/// DebrisVelocities (Vector), DebrisSizes (float, cube edge lengths in cm).
	/// Null (the default) spawns nothing. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	TObjectPtr<UNiagaraSystem> DebrisSystem;

	/// One or more welds snapped this check.
	UPROPERTY(BlueprintAssignable, Category = "Fracture")
	FBox3DFracturedWeldBrokeSignature OnWeldBroken;

	/// Fires for each overloaded bond before health erosion and possible failure,
	/// providing a creak-audio / dust-VFX hook.
	UPROPERTY(BlueprintAssignable, Category = "Fracture|Stress")
	FBox3DStructureStressedSignature OnStructureStressed;

	/// Build the mesh sections from fracture output. Fragment geometry is in
	/// actor space (the source component's space with scale baked in, UE cm).
	/// Exterior sections get SourceMaterial, interior sections CoreMaterial
	/// (falling back to SourceMaterial when unset). Visual-only initialization is
	/// the replicated-client path: it deliberately creates no b3 bodies, welds,
	/// structural state, or fragment-pool entry.
	void InitializeFragments(TArray<Box3D::Fracture::FBox3DFragmentData>&& InFragments,
		UMaterialInterface* SourceMaterial, bool bCreatePhysics = true);

	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 GetFragmentCount() const { return Fragments.Num(); }

	/// PMC section indices for a fragment: X = exterior section, Y = interior
	/// section, INDEX_NONE where the fragment has no faces of that kind (fully
	/// interior fragments have no exterior section).
	FIntPoint GetFragmentSections(int32 FragmentIndex) const
	{
		return FragmentSections.IsValidIndex(FragmentIndex) ? FragmentSections[FragmentIndex]
															: FIntPoint(INDEX_NONE, INDEX_NONE);
	}

	/// Fracture geometry this actor renders and simulates (fragment vertices
	/// feed the hulls, Neighbors the welds).
	const TArray<Box3D::Fracture::FBox3DFragmentData>& GetFragments() const { return Fragments; }

	UProceduralMeshComponent* GetMesh() const { return Mesh; }

	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 GetLiveWeldCount() const { return Welds.Num(); }

	/// Simulation tier the fragment was routed to (Body when unclassified).
	EBox3DFragmentTier GetFragmentTier(int32 FragmentIndex) const
	{
		return FragmentTiers.IsValidIndex(FragmentIndex) ? FragmentTiers[FragmentIndex]
														 : EBox3DFragmentTier::Body;
	}

	int32 CountFragmentsInTier(EBox3DFragmentTier Tier) const;

	/// Burst arrays of the Debris-tier fragments, in actor space. The Niagara
	/// hookup consuming these is a later D3 slice.
	const FBox3DDebrisBurst& GetDebrisBurst() const { return DebrisBurst; }

	/// Surviving weld fragment pairs, X < Y. Island counting for tests and later
	/// milestones.
	void GetLiveWeldPairs(TArray<FIntPoint>& OutPairs) const;

	/// The fragment's dynamic hull body (b3_nullBodyId when hull cooking failed
	/// and the fragment stayed visual-only).
	b3BodyId GetFragmentBody(int32 FragmentIndex) const
	{
		return FragmentBodies.IsValidIndex(FragmentIndex) ? FragmentBodies[FragmentIndex] : b3BodyId{};
	}

	/// Snap any weld whose constraint force exceeds its break force. In
	/// structural mode each snapped weld also breaks its bond in the structure
	/// graph and queues any resulting unsupported islands for promotion. Runs
	/// from Tick; public so headless tests can pump it without ticking actors.
	void CheckWelds();

	/// Remove one fragment from the assembly: destroy its body and welds, clear
	/// its mesh sections, and in structural mode notify the structure graph and
	/// queue any unsupported islands for promotion. No-op on invalid or
	/// already-destroyed fragments.
	UFUNCTION(BlueprintCallable, Category = "Fracture")
	void DestroyFragment(int32 FragmentIndex);

	/// Promote queued unsupported chunks static->dynamic, up to
	/// UBox3DSettings::MaxPromotionsPerTick per call (FIFO overflow carries to
	/// the next call). Two-phase: the whole batch flips type first, then wakes,
	/// so intra-island welds never straddle a static body and an awake dynamic
	/// one mid-pass. Runs from Tick; public for headless tests.
	void ProcessPromotions();

	/// Advance structural stress relaxation and sustained-overload erosion.
	/// Runs from Tick; public so headless tests can pump exact fixed steps.
	void ProcessStructuralStress(float DeltaSeconds);

	/// Feed an event-driven impulse into the next stress solve. Inputs are world
	/// space UE units (kg*cm/s and cm); invalid/non-structural fragments are ignored.
	void QueueStressImpulse(int32 FragmentIndex, const FVector& WorldImpulse,
		const FVector& WorldApplicationPoint);

	/// Box3DExplode's structural event path. Builds deterministic per-fragment
	/// impulses without polling physics-joint forces.
	void QueueExplosionStress(const FVector& WorldCenter, float Radius, float Falloff,
		float ImpulsePerArea);

	/// Chunks waiting in the promotion queue.
	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 GetPendingPromotionCount() const { return PendingPromotions.Num(); }

	/// Structural mode is live: the assembly built a bond graph and found
	/// anchors (false when bStructural was off or anchor detection found none).
	UFUNCTION(BlueprintPure, Category = "Fracture")
	bool IsStructureActive() const { return bStructureActive; }

	/// The assembly's bond graph. Empty unless structural mode is live.
	const Box3D::Structure::FBox3DStructureGraph& GetStructureGraph() const { return StructureGraph; }

	/// Deterministic digest of current bond health and last resolved loads.
	uint32 GetStructureStressHash() const { return StressSolver.BondHealthHash(); }

	/// Stamp a deterministic cosmetic dent into every fragment PMC section in
	/// range. ImpactNormal points out of the struck surface; vertices move inward
	/// by the radial falloff, capped at MaxDepthCm for this stamp. The render
	/// geometry changes immediately; fragment hull collision is deliberately
	/// untouched. Returns the number of displaced section vertices.
	UFUNCTION(BlueprintCallable, Category = "Fracture|Deformation",
		meta = (AdvancedDisplay = "FalloffExponent"))
	int32 ApplyVertexDent(FVector WorldImpactPoint, FVector WorldImpactNormal,
		float RadiusCm, float MaxDepthCm, float FalloffExponent = 2.0f);

	/// Re-emit PMC sections from the fragment body transforms (awake bodies
	/// only). Runs from Tick; public for headless tests.
	void SyncFragments();

	//~ AActor
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildFragmentPhysics();
	void DestroyFragmentPhysics();

	/// Build the bond graph and auto-detect anchors (structural mode). With no
	/// anchors the assembly reverts to plain dynamic rubble. Fragments that got
	/// no body (non-Body tiers, failed hull cooks) are marked destroyed in the
	/// graph so they cannot carry support.
	void InitializeStructure();

	/// Queue island chunks for promotion, in island order (FIFO).
	void EnqueueIslands(const Box3D::Structure::FBox3DStructureIslands& Islands);

	/// Destroy the weld matching a graph bond, route its connectivity event, and
	/// enqueue newly unsupported islands. Returns false for an already-dead bond.
	bool BreakStructuralBond(int32 BondIndex);

	/// Rebuild the pure-data stress solve graph after topology changes.
	bool InitializeStressSolver();

	/// Fire-and-forget DebrisSystem spawn carrying the burst arrays, world space.
	void SpawnDebrisBurst() const;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UProceduralMeshComponent> Mesh;

	TArray<Box3D::Fracture::FBox3DFragmentData> Fragments;

	/// Per-fragment tier, parallel to Fragments.
	TArray<EBox3DFragmentTier> FragmentTiers;

	FBox3DDebrisBurst DebrisBurst;

	/// Per-fragment (ExteriorSection, InteriorSection), INDEX_NONE = absent.
	TArray<FIntPoint> FragmentSections;

	/// Per-fragment dynamic hull body, parallel to Fragments.
	TArray<b3BodyId> FragmentBodies;

	struct FWeld
	{
		b3JointId Joint;
		int32 FragmentA;
		int32 FragmentB;
		/// Newtons; <= 0 = unbreakable.
		float BreakForce;
	};
	TArray<FWeld> Welds;

	/// Bond graph of the assembly; built only when structural mode activates.
	Box3D::Structure::FBox3DStructureGraph StructureGraph;

	/// Structural mode passed anchor detection and the graph is live.
	bool bStructureActive = false;

	/// Fragment indices awaiting static->dynamic promotion, FIFO.
	TArray<int32> PendingPromotions;

	Box3D::Structure::FBox3DStressSolver StressSolver;
	TArray<Box3D::Structure::FBox3DStressImpulse> PendingStressImpulses;
	bool bStressTopologyDirty = false;
	int32 AppliedStressCoarsenThreshold = INDEX_NONE;
	bool bStressSolveSeeded = false;
	bool bStressImpulseSolve = false;

	/// Body-local copy of one PMC section's geometry (vertices relative to the
	/// fragment centroid, normals in the spawn frame) so SyncFragments can
	/// re-emit it under the current body transform.
	struct FSectionGeometry
	{
		int32 SectionIndex = INDEX_NONE;
		int32 FragmentIndex = INDEX_NONE;
		TArray<FVector> LocalVertices;
		TArray<FVector> LocalNormals;
	};
	TArray<FSectionGeometry> SectionGeometry;
};

namespace Box3D
{
	/// Resolve the convex proxy to fracture for a static mesh component, in the
	/// component's local space with scale baked in (UE cm). Preference order:
	/// authored convex elements -> hull of simple-collision primitives ->
	/// editor-only hull of LOD0 render vertices (logged warning). Returns the
	/// source used, or None (OutProxy untouched) when no geometry is available.
	BOX3DRUNTIME_API EBox3DFractureProxySource ResolveFractureProxy(
		const UStaticMeshComponent& Component, Fracture::FFractureProxy& OutProxy);

	/// Stable source-geometry id used by FBox3DDestructionEvent. It is the static
	/// mesh asset path, not an actor id; game replication still resolves which
	/// actor/component receives the event.
	BOX3DRUNTIME_API FName GetDestructionMeshId(const UStaticMeshComponent& Component);

	/// Fracture a static mesh component in place. This is a local destruction
	/// decision and therefore refuses to run without simulation authority.
	BOX3DRUNTIME_API ABox3DFracturedActor* FractureMesh(UStaticMeshComponent* Component,
		const FBox3DFractureMeshParams& Params);

	/// Deterministically regenerate only the render fragment set from a received
	/// authority event. This is the pure-client path and never creates b3 state,
	/// even if called in a world that happens to have a Box3D world.
	BOX3DRUNTIME_API ABox3DFracturedActor* RegenerateFractureVisuals(
		UStaticMeshComponent* Component, const FBox3DFractureMeshParams& Params);
}
