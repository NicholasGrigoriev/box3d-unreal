#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "box3d/id.h"
#include "Box3DClothActor.generated.h"

class UBox3DBodyComponent;
class UMaterialInterface;
class UProceduralMeshComponent;

/// Which cloth particles are pinned to the anchor.
UENUM(BlueprintType)
enum class EBox3DClothPin : uint8
{
	/// The whole top row — a curtain or hanging banner.
	TopEdge,
	/// Only the two top corners — a flag or draped sheet.
	TopCorners,
	/// Nothing pinned — a loose sheet that falls and drapes over the world.
	None,
};

/// A physically simulated cloth sheet: a Rows x Columns lattice of raw Box3D
/// sphere particles (rotation locked — pure point masses) stitched together
/// with rigid distance joints along warp/weft and softer spring joints across
/// the diagonals (shear). The particles are never drawn — a procedural mesh
/// re-skins their positions every frame, normals recomputed, interpolated
/// between fixed steps.
///
/// The cloth hangs in the actor's local X (width) / Z (height, downward) plane
/// from a kinematic anchor, so moving the actor drags pinned cloth. Particles
/// use the Debris channel: explosions blow the cloth around and pawn proxies
/// push through it like a curtain.
///
/// Use a Two Sided material — the sheet is single-layer geometry.
/// Bodies and joints are built on BeginPlay; layout properties are
/// creation-time only.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DClothActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DClothActor();

	/// Cloth width in cm, along the actor's local X.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (ClampMin = "20"))
	float Width = 200.0f;

	/// Cloth height in cm, hanging down the actor's local -Z.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (ClampMin = "20"))
	float Height = 200.0f;

	/// Particles across the width. Higher = finer folds, quadratically more cost.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (ClampMin = "2", ClampMax = "40"))
	int32 Columns = 12;

	/// Particles down the height.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (ClampMin = "2", ClampMax = "40"))
	int32 Rows = 12;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth")
	EBox3DClothPin PinMode = EBox3DClothPin::TopEdge;

	/// Particle density in kg/m^3. Lower = lighter, floatier cloth.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (ClampMin = "1"))
	float Density = 300.0f;

	/// Fakes air drag; the dominant "cloth feel" dial.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (ClampMin = "0"))
	float LinearDamping = 0.2f;

	/// Diagonal spring joints resisting shear. Off = fabric that folds freely
	/// like silk; on = holds its shape more like canvas.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth")
	bool bShearConstraints = true;

	/// Shear spring stiffness in Hz (solver-clamped; ~2-8 is the useful range).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (EditCondition = "bShearConstraints", ClampMin = "0"))
	float ShearHertz = 4.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (EditCondition = "bShearConstraints", ClampMin = "0"))
	float ShearDampingRatio = 0.5f;

	/// Material for the sheet. Enable Two Sided on it, or the back face is invisible.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	TObjectPtr<UMaterialInterface> ClothMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rendering")
	bool bCastShadow = true;

	/// World position of a particle (row 0 = top, column 0 = local -X edge).
	UFUNCTION(BlueprintPure, Category = "Cloth")
	FVector GetParticleLocation(int32 Row, int32 Column) const;

	/// Shove the particle nearest to Location (impulse in kg*cm/s) — an easy
	/// bridge for weapon hits, which trace against Chaos and never see cloth.
	UFUNCTION(BlueprintCallable, Category = "Cloth")
	void AddImpulseAtNearestParticle(FVector Location, FVector Impulse);

	UBox3DBodyComponent* GetAnchorBody() const { return AnchorBody; }
	UProceduralMeshComponent* GetClothMesh() const { return ClothMesh; }
	int32 GetParticleCount() const { return Bodies.Num(); }

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildCloth();
	void DestroyCloth();
	void UpdateClothVisual();
	int32 ParticleIndex(int32 Row, int32 Column) const { return Row * GridColumns + Column; }

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBox3DBodyComponent> AnchorBody;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Cloth", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UProceduralMeshComponent> ClothMesh;

	TArray<b3BodyId> Bodies;
	int32 GridRows = 0;
	int32 GridColumns = 0;

	/// Between-step visual interpolation, positions only (rotation is locked).
	struct FParticleInterp
	{
		FVector P0 = FVector::ZeroVector;
		FVector P1 = FVector::ZeroVector;
	};
	TArray<FParticleInterp> Interp;
	uint64 LastStep = 0;
	bool bAllAsleep = false;

	/// Reused per-frame buffers for the mesh update.
	TArray<FVector> VertexBuffer;
	TArray<FVector> NormalBuffer;
	TArray<int32> Triangles;
};
