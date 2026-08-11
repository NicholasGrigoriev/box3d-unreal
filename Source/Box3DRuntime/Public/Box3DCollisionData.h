#pragma once

#include "CoreMinimal.h"
#include "Box3DCollisionData.generated.h"

/// Baked static collision: the geometry the static scene mirror would cook at
/// runtime, extracted once in the editor and saved as an asset. Packaged builds
/// instantiate it directly — no runtime cooking, no dependency on CPU-accessible
/// render data (which cooked builds strip unless bAllowCPUAccess is set).
///
/// Concept ported from Antonio Lattanzio's Box3DUnreal
/// (github.com/alattanzio/Box3DUnreal, MIT). See docs/BAKED_COLLISION.md.

/// One shape's kind. Mirrors what the bake extraction produces; the loader
/// switches on it.
UENUM()
enum class EBox3DBakedShapeKind : uint8
{
	/// Convex hull rebuilt at load from Points (b3CreateHull is deterministic).
	Hull,
	/// Static tri-mesh: Points = vertices, Indices = triangles. Winding is
	/// already flipped to box3d's convention at bake time.
	Mesh,
	/// CenterA + Radius.
	Sphere,
	/// CenterA / CenterB (hemisphere centers) + Radius.
	Capsule,
};

/// One box3d shape in its body's local space. All coordinates are UE centimeters
/// with the owning component's scale already baked in (box3d shapes carry no
/// scale); the loader converts through the ordinary Box3D conversion seam.
USTRUCT()
struct FBox3DBakedShape
{
	GENERATED_BODY()

	UPROPERTY()
	EBox3DBakedShapeKind Kind = EBox3DBakedShapeKind::Hull;

	/// Hull point cloud, or Mesh vertices. Empty for sphere/capsule.
	UPROPERTY()
	TArray<FVector3f> Points;

	/// Mesh triangle indices (3 per triangle, box3d winding). Empty unless
	/// Kind == Mesh.
	UPROPERTY()
	TArray<int32> Indices;

	/// Sphere center / capsule first hemisphere center (cm).
	UPROPERTY()
	FVector3f CenterA = FVector3f::ZeroVector;

	/// Capsule second hemisphere center (cm, unused otherwise).
	UPROPERTY()
	FVector3f CenterB = FVector3f::ZeroVector;

	/// Sphere / capsule radius (cm).
	UPROPERTY()
	float Radius = 0.0f;
};

/// One static body: the source component's (or ISM instance's) world transform
/// with scale baked into the shapes. Self-contained — the runtime instantiates
/// it without the source actor.
USTRUCT()
struct FBox3DBakedBody
{
	GENERATED_BODY()

	/// World transform at bake time. Only location + rotation place the body;
	/// scale is baked into the shape geometry.
	UPROPERTY()
	FTransform WorldTransform = FTransform::Identity;

	/// Source actor/component path within its level — metadata for rebake
	/// diffing, not used at load.
	UPROPERTY()
	FString ActorKey;

	UPROPERTY()
	TArray<FBox3DBakedShape> Shapes;
};

/// Cached static collision for one map, produced by the Box3DBake commandlet and
/// loaded on world begin-play by UBox3DWorldSubsystem (which then skips the
/// initial runtime mirror pass). See docs/BAKED_COLLISION.md.
UCLASS(BlueprintType)
class BOX3DRUNTIME_API UBox3DCollisionData : public UObject
{
	GENERATED_BODY()

public:
	/// The asset a map bakes to by convention: BC_<MapName> beside the map.
	/// Shared by the bake commandlet (output) and runtime auto-discovery
	/// (lookup), so the two can never disagree about where a bake lives.
	static FString DeriveAssetPackageName(const FString& MapPackageName);

	/// Map (long package name) this was baked from.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FString SourceLevel;

	/// box3d version at bake time ("major.minor.revision"), so a version bump can
	/// be spotted as stale.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FString Box3DVersion;

	/// True if any body was baked from triangle-mesh geometry.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	bool bContainsTriMesh = false;

	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FDateTime BakeTime = FDateTime(0);

	/// Opaque identity of the source level's files at bake time (see
	/// ComputeSourceFingerprint). Differs from the level's current fingerprint =>
	/// the bake is out of date.
	UPROPERTY(VisibleAnywhere, Category = "Box3D")
	FString SourceFingerprint;

	UPROPERTY()
	TArray<FBox3DBakedBody> Bodies;

#if WITH_EDITOR
	/// Cheap identity of a level on disk: newest modified time + total size +
	/// file count over the .umap AND its external-actors folders. The external
	/// actors matter — under One File Per Actor, moving an actor rewrites its own
	/// package and never touches the .umap, so a .umap-only timestamp would call
	/// an edited world fresh. Empty if the map can't be found.
	///
	/// This is a "something changed" signal, not a geometry hash: re-saving a
	/// level without touching collision also changes it. It only ever warns, so a
	/// false positive costs a re-bake, while a miss would ship wrong collision.
	static FString ComputeSourceFingerprint(const FString& MapPackageName);

	/// True if this bake can no longer be trusted: box3d version bump, source
	/// level edited since the bake, or a bake old enough to predate
	/// fingerprinting.
	bool IsStale(FString& OutReason) const;
#endif
};
