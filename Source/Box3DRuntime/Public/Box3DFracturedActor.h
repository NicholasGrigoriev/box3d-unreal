#pragma once

#include "CoreMinimal.h"
#include "Box3DFracture.h"
#include "GameFramework/Actor.h"
#include "box3d/id.h"
#include "Box3DFracturedActor.generated.h"

class UMaterialInterface;
class UProceduralMeshComponent;
class UStaticMeshComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FBox3DFracturedWeldBrokeSignature);

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

	/// One or more welds snapped this check.
	UPROPERTY(BlueprintAssignable, Category = "Fracture")
	FBox3DFracturedWeldBrokeSignature OnWeldBroken;

	/// Build the mesh sections from fracture output. Fragment geometry is in
	/// actor space (the source component's space with scale baked in, UE cm).
	/// Exterior sections get SourceMaterial, interior sections CoreMaterial
	/// (falling back to SourceMaterial when unset).
	void InitializeFragments(TArray<Box3D::Fracture::FBox3DFragmentData>&& InFragments,
		UMaterialInterface* SourceMaterial);

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

	/// Surviving weld fragment pairs, X < Y. Island counting for tests and later
	/// milestones.
	void GetLiveWeldPairs(TArray<FIntPoint>& OutPairs) const;

	/// The fragment's dynamic hull body (b3_nullBodyId when hull cooking failed
	/// and the fragment stayed visual-only).
	b3BodyId GetFragmentBody(int32 FragmentIndex) const
	{
		return FragmentBodies.IsValidIndex(FragmentIndex) ? FragmentBodies[FragmentIndex] : b3BodyId{};
	}

	/// Snap any weld whose constraint force exceeds its break force. Runs from
	/// Tick; public so headless tests can pump it without ticking actors.
	void CheckWelds();

	/// Re-emit PMC sections from the fragment body transforms (awake bodies
	/// only). Runs from Tick; public for headless tests.
	void SyncFragments();

	//~ AActor
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildFragmentPhysics();
	void DestroyFragmentPhysics();
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UProceduralMeshComponent> Mesh;

	TArray<Box3D::Fracture::FBox3DFragmentData> Fragments;

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

	/// Fracture a static mesh component in place: resolve its proxy, run the
	/// deterministic fracture core, spawn an ABox3DFracturedActor at the
	/// component's transform, then swap the source out (hide it, disable its
	/// Chaos collision, remove its static-mirror body if mirrored). Returns null
	/// when no proxy resolves or fracture produces no fragments.
	BOX3DRUNTIME_API ABox3DFracturedActor* FractureMesh(UStaticMeshComponent* Component,
		const FBox3DFractureMeshParams& Params);
}
