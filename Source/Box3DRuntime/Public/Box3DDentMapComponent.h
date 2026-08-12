#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Box3DDentMapComponent.generated.h"

class UMaterialInstanceDynamic;
class UMeshComponent;
class UTexture2D;
class UTextureRenderTarget2D;

/// Per-mesh cosmetic dent-height accumulator. Impacts are stamped in UV space
/// into alternating render targets, so the previous map is never sampled while
/// it is being written. Collision and Box3D simulation are deliberately untouched.
UCLASS(BlueprintType, ClassGroup = (Box3D), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DDentMapComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBox3DDentMapComponent();

	/// Texture parameter sampled by shared surface materials. The red channel is
	/// accumulated dent height in [0, 1]. Derive a tangent-space normal from
	/// neighbouring texels (and optionally WPO) in the material.
	static const FName DentMapParameterName;

	/// Scalar multiplier for the normal reconstructed by the surface material.
	static const FName DentNormalStrengthParameterName;

	/// Vector parameter (1/Width, 1/Height, Width, Height).
	static const FName DentMapTexelSizeParameterName;

	/// Square render-target resolution, clamped to [32, 2048] on initialization.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dent Map", meta = (ClampMin = "32", ClampMax = "2048"))
	int32 DentMapResolution = 256;

	/// Value supplied to Box3D_DentNormalStrength on every bound material instance.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dent Map")
	float DentNormalStrength = 1.0f;

	/// Initialize two transient render targets and bind this component's stable
	/// material parameter contract to dynamic instances on TargetMesh.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Deformation")
	bool InitializeDentMap(UMeshComponent* TargetMesh);

	/// Accumulate a radial dent at normalized UV. Radius is in UV units and
	/// Strength is clamped to [0, 1]. Returns false for invalid input/state.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Deformation")
	bool StampDentUV(FVector2D ImpactUV, float RadiusUV, float Strength = 1.0f);

	/// Clear both ping-pong targets and reset the deterministic active side/count.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Deformation")
	void ClearDentMap();

	/// Recreate dynamic material instances and publish the active map contract.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Deformation")
	void RefreshMaterialBindings();

	UFUNCTION(BlueprintPure, Category = "Box3D|Deformation")
	UTextureRenderTarget2D* GetDentMap() const;

	/// The other side of the ping-pong pair, exposed for diagnostics/tests.
	UTextureRenderTarget2D* GetInactiveDentMap() const;

	UFUNCTION(BlueprintPure, Category = "Box3D|Deformation")
	int32 GetDentStampCount() const { return StampCount; }

	const TArray<TObjectPtr<UMaterialInstanceDynamic>>& GetMaterialInstances() const
	{
		return MaterialInstances;
	}

private:
	bool AllocateResources();
	void ClearTarget(UTextureRenderTarget2D* Target) const;
	void PublishActiveMap();

	UPROPERTY(Transient)
	TObjectPtr<UMeshComponent> BoundMesh;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UTextureRenderTarget2D>> RenderTargets;

	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> StampBrush;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UMaterialInstanceDynamic>> MaterialInstances;

	int32 ActiveTargetIndex = 0;
	int32 StampCount = 0;
};
