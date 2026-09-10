#pragma once

#include "CoreMinimal.h"
#include "Box3DDestruction.h"
#include "Box3DFracture.h"
#include "Box3DStructure.h"
#include "Box3DStress.h"
#include "Engine/TimerHandle.h"
#include "GameFramework/Actor.h"
#include "box3d/id.h"
#include "Box3DFracturedActor.generated.h"

class UMaterialInterface;
class UNiagaraSystem;
class USceneComponent;
class UStaticMesh;
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

/// How ABox3DFracturedActor currently draws one fragment.
UENUM(BlueprintType)
enum class EBox3DFragmentRenderState : uint8
{
	/// Not drawn: non-Body tier, or destroyed.
	None,
	/// Baked into the actor's shared attached mesh (one static mesh for every
	/// fragment still held by the assembly; exterior and interior faces are its
	/// two sections). Rebuilt only when the attached set changes.
	Attached,
	/// A free body: drawn by its own static mesh component, moved by transform
	/// from the fragment body each tick.
	Loose,
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

	/// Pull every fragment hull vertex this far (cm) toward its centroid before
	/// cooking, so neighbouring hulls never overlap. Coincident Voronoi faces
	/// otherwise leave sub-millimetre penetrations whose push-out friction can
	/// pin a fragment detached from a static assembly between its neighbours.
	/// Rendering is unaffected; 0 keeps exact hulls.
	float HullInsetCm = 0.0f;

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

	/// Structural assemblies only: every fragment that gets a body is an anchor,
	/// skipping DetectAnchors. For cladding glued to an immovable surface (a
	/// destructible skin over a real wall) this makes support independent of
	/// what happens to sit behind the mesh; fragments then leave the assembly
	/// only through DetachFragment / DestroyFragment.
	bool bAnchorAllFragments = false;

	/// Sustained-load capacities in pascals. Non-positive disables that mode.
	float TensionStrengthPa = 1.0e6f;
	float CompressionStrengthPa = 5.0e6f;
	float ShearStrengthPa = 1.0e6f;

	/// Health removed per second for each unit of overload above capacity.
	float SustainedOverloadHealthPerSecond = 1.0f;

	/// Render flags for every fragment primitive (the attached mesh and each
	/// loose chip). Cladding over real geometry usually turns all three off:
	/// the wall behind it already shadows, feeds Lumen and takes decals.
	bool bCastShadow = true;
	bool bAffectIndirectLighting = true;
	bool bReceivesDecals = true;
};

/// Optional shared source of loose-chip meshes for a fractured actor. When
/// set (before the first detach), MakeFragmentLoose asks it for a fragment's
/// mesh before building one and hands back what it built, so actors fractured
/// from identical fragment data (cached cladding cells) share one UStaticMesh
/// per chip instead of each building its own. Meshes it stores are created
/// with GetMeshOuter() as outer, so they outlive any single actor; the cache
/// owns their lifetime. Dented actors (ApplyVertexDent) stop consulting it,
/// as their geometry no longer matches the shared set.
class BOX3DRUNTIME_API IBox3DChipMeshCache
{
public:
	virtual ~IBox3DChipMeshCache() = default;
	virtual UStaticMesh* FindChipMesh(int32 FragmentIndex) = 0;
	virtual void StoreChipMesh(int32 FragmentIndex, UStaticMesh* Mesh) = 0;
	virtual UObject* GetMeshOuter() = 0;
};

/// A fractured static mesh. Fragments still held by the assembly (static
/// bodies, or every fragment on the visual-only client path) are baked into ONE
/// runtime static mesh — exterior faces (inherited from the proxy surface) in a
/// section with the source component's material, interior cut faces in a
/// section with CoreMaterial — so a settled ruin costs one static-relevance
/// primitive however many pieces it has. A fragment whose body turns dynamic
/// (DetachFragment, structural promotion, plain rubble) leaves that mesh for
/// its own pooled static mesh component, which Tick moves by transform from
/// the body; vertices are never rewritten per frame. Spawned by
/// Box3D::FractureMesh, which also swaps out the source component (hidden,
/// Chaos collision off, static mirror body removed).
///
/// Each fragment gets a b3 hull body; adjacent fragments (cell adjacency from
/// the fracture core, not bounds proximity) are welded with break force
/// SharedFaceArea x MaterialToughness. Tick snaps overloaded welds
/// (OnWeldBroken). Game code chips pieces off deliberately with DetachFragment
/// (after finding them with FindFragmentAtPoint / FindNearestFragment) and
/// removes them for good with DestroyFragment. Render changes coalesce: they
/// apply on the next SyncFragments (Tick) or, for a non-ticking actor, on a
/// next-tick timer — FlushRenderState applies them at once.
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

	/// Physics-hull inset toward the centroid (cm); see FBox3DFractureMeshParams::
	/// HullInsetCm. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (ClampMin = "0"))
	float HullInsetCm = 0.0f;

	/// Spawn the fragment bodies asleep. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	bool bStartAsleep = false;

	/// Anchored structural assembly (D4): see FBox3DFractureMeshParams::
	/// bStructural. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	bool bStructural = false;

	/// Every fragment with a body is an anchor (see FBox3DFractureMeshParams::
	/// bAnchorAllFragments). Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	bool bAnchorAllFragments = false;

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

	/// Render flags applied to every fragment primitive; see
	/// FBox3DFractureMeshParams. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture|Rendering")
	bool bFragmentsCastShadow = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture|Rendering")
	bool bFragmentsAffectIndirectLighting = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture|Rendering")
	bool bFragmentsReceiveDecals = true;

	/// Material of the exterior faces (the fractured component's material).
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fracture")
	TObjectPtr<UMaterialInterface> ExteriorMaterial;

	/// One or more welds snapped this check.
	UPROPERTY(BlueprintAssignable, Category = "Fracture")
	FBox3DFracturedWeldBrokeSignature OnWeldBroken;

	/// Fires for each overloaded bond before health erosion and possible failure,
	/// providing a creak-audio / dust-VFX hook.
	UPROPERTY(BlueprintAssignable, Category = "Fracture|Stress")
	FBox3DStructureStressedSignature OnStructureStressed;

	/// Build the render geometry and physics from fracture output. Fragment
	/// geometry is in actor space (the source component's space with scale baked
	/// in, UE cm). Exterior faces get SourceMaterial, interior faces CoreMaterial
	/// (falling back to SourceMaterial when unset). Visual-only initialization is
	/// the replicated-client path: it deliberately creates no b3 bodies, welds,
	/// structural state, or fragment-pool entry, and draws every fragment in the
	/// attached mesh.
	void InitializeFragments(TArray<Box3D::Fracture::FBox3DFragmentData>&& InFragments,
		UMaterialInterface* SourceMaterial, bool bCreatePhysics = true);

	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 GetFragmentCount() const { return Fragments.Num(); }

	/// How the fragment is drawn right now (pending changes apply on the next
	/// SyncFragments / FlushRenderState).
	UFUNCTION(BlueprintPure, Category = "Fracture|Rendering")
	EBox3DFragmentRenderState GetFragmentRenderState(int32 FragmentIndex) const
	{
		return FragmentRenderStates.IsValidIndex(FragmentIndex) ? FragmentRenderStates[FragmentIndex]
																: EBox3DFragmentRenderState::None;
	}

	/// The component drawing a Loose fragment; null for Attached / None.
	UFUNCTION(BlueprintPure, Category = "Fracture|Rendering")
	UStaticMeshComponent* GetFragmentComponent(int32 FragmentIndex) const
	{
		return FragmentComponents.IsValidIndex(FragmentIndex) ? FragmentComponents[FragmentIndex].Get() : nullptr;
	}

	/// The shared mesh of every Attached fragment (its static mesh is null while
	/// nothing is attached).
	UStaticMeshComponent* GetAttachedMesh() const { return AttachedMesh; }

	UFUNCTION(BlueprintPure, Category = "Fracture|Rendering")
	int32 CountLooseFragments() const;

	/// Primitives this actor currently submits: the attached mesh (when it has
	/// geometry) plus one per Loose fragment.
	UFUNCTION(BlueprintPure, Category = "Fracture|Rendering")
	int32 GetRenderPrimitiveCount() const;

	/// Static meshes built so far (attached rebuilds + chip meshes); a settled
	/// actor must not grow this from Tick.
	int32 GetRenderBuildCount() const { return RenderBuildCount; }

	/// Loose-chip meshes taken from ChipMeshCache instead of being built.
	int32 GetChipMeshCacheHitCount() const { return ChipMeshCacheHits; }

	/// See IBox3DChipMeshCache. Null (the default) builds every chip mesh here.
	TSharedPtr<IBox3DChipMeshCache> ChipMeshCache;

	/// Render vertices of a Body-tier fragment, relative to its centroid in the
	/// spawn frame (the body frame); null for fragments that are not drawn.
	const TArray<FVector>* GetFragmentRenderVertices(int32 FragmentIndex) const;

	/// Fracture geometry this actor renders and simulates (fragment vertices
	/// feed the hulls, Neighbors the welds).
	const TArray<Box3D::Fracture::FBox3DFragmentData>& GetFragments() const { return Fragments; }

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

	/// Remove one fragment from the assembly: destroy its body and welds, stop
	/// drawing it, and in structural mode notify the structure graph and queue
	/// any unsupported islands for promotion. No-op on invalid or
	/// already-destroyed fragments.
	UFUNCTION(BlueprintCallable, Category = "Fracture")
	void DestroyFragment(int32 FragmentIndex);

	/// Release one fragment from the assembly as a free body: its welds are
	/// destroyed, the structure graph drops it (neighbours re-evaluate support),
	/// the body flips static->dynamic, wakes, is moved by WorldNudgeCm (clears
	/// face-to-face contact with what it leaves behind) and takes the given
	/// velocities (cm/s, rad/s). The fragment keeps rendering and can still be
	/// destroyed later. Returns false for invalid or bodiless fragments.
	UFUNCTION(BlueprintCallable, Category = "Fracture")
	bool DetachFragment(int32 FragmentIndex, FVector WorldLinearVelocity,
		FVector WorldAngularVelocity = FVector::ZeroVector, FVector WorldNudgeCm = FVector::ZeroVector);

	/// The fragment still has a physics body (not destroyed, not visual-only).
	UFUNCTION(BlueprintPure, Category = "Fracture")
	bool IsFragmentAlive(int32 FragmentIndex) const;

	/// Alive and still held static by the structure (never detached or promoted).
	UFUNCTION(BlueprintPure, Category = "Fracture")
	bool IsFragmentAttached(int32 FragmentIndex) const;

	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 CountAliveFragments() const;

	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 CountAttachedFragments() const;

	/// World-space centroid of the fragment's current body pose (spawn pose
	/// for bodiless fragments).
	UFUNCTION(BlueprintPure, Category = "Fracture")
	FVector GetFragmentWorldCentroid(int32 FragmentIndex) const;

	/// Index of the alive fragment whose hull contains the world point, tested
	/// against the fragment's current body pose, or INDEX_NONE. Merged
	/// (non-convex) fragments are tested as the intersection of their face
	/// half-spaces, which can miss points inside a concavity; pair with
	/// FindNearestFragment for a fallback.
	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 FindFragmentAtPoint(FVector WorldPoint, bool bAttachedOnly = true) const;

	/// Alive fragment whose current centroid lies nearest to the world point,
	/// within MaxDistanceCm; INDEX_NONE when none qualifies.
	UFUNCTION(BlueprintPure, Category = "Fracture")
	int32 FindNearestFragment(FVector WorldPoint, float MaxDistanceCm, bool bAttachedOnly = true) const;

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

	/// Stamp a deterministic cosmetic dent into every drawn fragment in range.
	/// ImpactNormal points out of the struck surface; vertices move inward by
	/// the radial falloff, capped at MaxDepthCm for this stamp. The affected
	/// meshes are rebuilt immediately; fragment hull collision is deliberately
	/// untouched. Returns the number of displaced render vertices.
	UFUNCTION(BlueprintCallable, Category = "Fracture|Deformation",
		meta = (AdvancedDisplay = "FalloffExponent"))
	int32 ApplyVertexDent(FVector WorldImpactPoint, FVector WorldImpactNormal,
		float RadiusCm, float MaxDepthCm, float FalloffExponent = 2.0f);

	/// Apply pending render changes, then move every Loose fragment's component
	/// to its body transform (awake bodies only). Runs from Tick; public for
	/// headless tests.
	void SyncFragments();

	/// Apply pending render changes now: fragments whose body turned dynamic get
	/// their own component, destroyed ones stop drawing, and the attached mesh is
	/// rebuilt when its set changed. Idempotent; a no-op when nothing is pending.
	UFUNCTION(BlueprintCallable, Category = "Fracture|Rendering")
	void FlushRenderState();

	//~ AActor
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	/// Flat-shaded render geometry of one Body-tier fragment: vertices relative
	/// to the centroid in the spawn frame (the body frame), one vertex per face
	/// corner; UVs planar in actor space so they stay continuous across the
	/// attached mesh. Triangles index Vertices, split by material.
	struct FFragmentRenderGeometry
	{
		TArray<FVector> Vertices;
		TArray<FVector> Normals;
		TArray<FVector> Tangents;
		TArray<float> BinormalSigns;
		TArray<FVector2D> UVs;
		TArray<int32> ExteriorTriangles;
		TArray<int32> InteriorTriangles;
	};
	static void BuildRenderGeometry(const Box3D::Fracture::FBox3DFragmentData& Fragment, FFragmentRenderGeometry& Out);

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

	/// Render state a fragment should be in given its tier, destroyed flag and
	/// body type.
	EBox3DFragmentRenderState DesiredRenderState(int32 FragmentIndex) const;

	/// Flag the render state stale; FlushRenderState runs from the next
	/// SyncFragments or, for a non-ticking actor, from a next-tick timer.
	void MarkRenderDirty();

	/// Bake the given fragments into a transient static mesh: two material
	/// slots (exterior / interior), empty ones dropped. Actor space unless
	/// bCentroidRelative (chip meshes, body frame). Null when nothing to draw.
	UStaticMesh* BuildStaticMesh(TArrayView<const int32> FragmentIndices, bool bCentroidRelative, UObject* Outer = nullptr);

	/// The chip mesh for one fragment: from ChipMeshCache when it has one, else
	/// built (and stored there). Null when the fragment has nothing to draw.
	UStaticMesh* GetOrBuildChipMesh(int32 FragmentIndex);

	void RebuildAttachedMesh();
	void MakeFragmentLoose(int32 FragmentIndex);
	void ReleaseFragmentComponent(int32 FragmentIndex);
	UStaticMeshComponent* AcquireFragmentComponent();
	void ApplyRenderFlags(UPrimitiveComponent& Component) const;
	void SyncFragmentComponent(int32 FragmentIndex) const;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<USceneComponent> FractureRoot;

	/// Shared mesh of the Attached fragments.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UStaticMeshComponent> AttachedMesh;

	/// Per-fragment chip component, parallel to Fragments; null unless Loose.
	UPROPERTY()
	TArray<TObjectPtr<UStaticMeshComponent>> FragmentComponents;

	/// Chip components released by destroyed fragments, kept hidden for reuse.
	UPROPERTY()
	TArray<TObjectPtr<UStaticMeshComponent>> FreeFragmentComponents;

	TArray<Box3D::Fracture::FBox3DFragmentData> Fragments;

	/// Per-fragment render state, parallel to Fragments.
	TArray<EBox3DFragmentRenderState> FragmentRenderStates;

	/// Set by DestroyFragment, parallel to Fragments.
	TArray<bool> FragmentDestroyed;

	/// Parallel to Fragments; empty for non-Body tiers.
	TArray<FFragmentRenderGeometry> RenderGeometry;

	bool bRenderDirty = false;
	/// ApplyVertexDent changed this actor's render geometry: shared chip meshes no longer match it.
	bool bRenderGeometryDented = false;
	FTimerHandle RenderFlushTimer;
	int32 RenderBuildCount = 0;
	int32 ChipMeshCacheHits = 0;

	/// Per-fragment tier, parallel to Fragments.
	TArray<EBox3DFragmentTier> FragmentTiers;

	FBox3DDebrisBurst DebrisBurst;

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

	/// Fracture an explicit convex proxy (proxy-local cm, scale already baked)
	/// into a fractured actor placed at ProxyToWorld (rotation + translation).
	/// Params.Fracture.ImpactPoint is world space, as for FractureMesh. Nothing
	/// is swapped out: the caller owns whatever rendered the proxy until now.
	/// Authority only; null on failure.
	BOX3DRUNTIME_API ABox3DFracturedActor* FractureConvexProxy(UWorld* World, const FTransform& ProxyToWorld,
		const Fracture::FFractureProxy& Proxy, UMaterialInterface* SourceMaterial,
		const FBox3DFractureMeshParams& Params);

	/// Build a fractured actor from fragments the caller already has (its own
	/// Fracture::Fracture run, typically cached and reused across identical
	/// proxies), placed at ProxyToWorld like FractureConvexProxy. Fragment
	/// geometry is proxy-local cm with scale baked in. Params.Fracture is used
	/// only for its world-space ImpactPoint (debris burst); nothing is fractured
	/// here. Authority only; null on failure.
	BOX3DRUNTIME_API ABox3DFracturedActor* SpawnFracturedActorFromFragments(UWorld* World,
		const FTransform& ProxyToWorld, TArray<Fracture::FBox3DFragmentData>&& Fragments,
		UMaterialInterface* SourceMaterial, const FBox3DFractureMeshParams& Params);

	/// Deterministically regenerate only the render fragment set from a received
	/// authority event. This is the pure-client path and never creates b3 state,
	/// even if called in a world that happens to have a Box3D world.
	BOX3DRUNTIME_API ABox3DFracturedActor* RegenerateFractureVisuals(
		UStaticMeshComponent* Component, const FBox3DFractureMeshParams& Params);
}
