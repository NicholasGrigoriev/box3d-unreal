#pragma once

#include "CoreMinimal.h"
#include "Box3DFracture.h"
#include "GameFramework/Actor.h"
#include "Box3DFracturedActor.generated.h"

class UMaterialInterface;
class UProceduralMeshComponent;
class UStaticMeshComponent;

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
};

/// The rendering half of a fractured static mesh: one ProceduralMeshComponent
/// carrying up to two sections per fragment — exterior faces (inherited from
/// the proxy surface) with the source component's material, interior cut faces
/// with CoreMaterial. Spawned by Box3D::FractureMesh, which also swaps out the
/// source component (hidden, Chaos collision off, static mirror body removed).
///
/// This actor is purely visual for now: fragment physics (b3CreateHull bodies,
/// cell-adjacency welds, break events) arrive with the next D2 slice, driving
/// the stored per-fragment sections.
UCLASS(BlueprintType, NotBlueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DFracturedActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DFracturedActor();

	/// Material applied to interior (cut) faces. Set before InitializeFragments.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Fracture")
	TObjectPtr<UMaterialInterface> CoreMaterial;

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

	/// Fracture geometry this actor renders — the physics slice consumes this
	/// (fragment vertices for hulls, Neighbors for welds).
	const TArray<Box3D::Fracture::FBox3DFragmentData>& GetFragments() const { return Fragments; }

	UProceduralMeshComponent* GetMesh() const { return Mesh; }

private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fracture", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UProceduralMeshComponent> Mesh;

	TArray<Box3D::Fracture::FBox3DFragmentData> Fragments;

	/// Per-fragment (ExteriorSection, InteriorSection), INDEX_NONE = absent.
	TArray<FIntPoint> FragmentSections;
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
