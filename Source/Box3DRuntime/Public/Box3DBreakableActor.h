#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "box3d/id.h"
#include "Box3DBreakableActor.generated.h"

class UBox3DBodyComponent;
class UStaticMesh;
class UStaticMeshComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FBox3DWeldBrokeSignature);

/// A destructible assembly: every child static mesh becomes a Box3D chunk at
/// BeginPlay (a dynamic body driving the mesh, Chaos collision query-only so
/// existing traces still hit it), and chunks whose bounds touch are welded
/// together. Constraint force above BreakForce snaps a weld — shoot or blast
/// the object and it breaks apart along the authored seams.
///
/// Author it in Blueprint by adding StaticMeshComponents for the chunks (they
/// preview in the editor as placed), or build one at runtime with AddChunk
/// before FinishSpawning. Try `box3d.SpawnBreakable` for an instant wall.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DBreakableActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DBreakableActor();

	/// Weld strength in newtons. Constraint force above this snaps the weld;
	/// 0 = unbreakable. A stacked 10 kg chunk loads its weld with ~100 N at
	/// rest, so keep an order of magnitude of headroom. Sleeping assemblies
	/// report zero joint force — something must wake the chunks (an impact,
	/// an explosion, an impulse) before welds can snap.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Breakable", meta = (ClampMin = "0"))
	float BreakForce = 20000.0f;

	/// Chunks whose bounds, inflated by this (cm), overlap get welded.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Breakable", meta = (ClampMin = "0"))
	float WeldTolerance = 4.0f;

	/// Chunk density in kg/m^3.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Breakable", meta = (ClampMin = "1"))
	float ChunkDensity = 400.0f;

	/// Chunks sleep until something touches them, so welded walls and stacks
	/// don't spend solver time (or slump) while nothing is happening.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Breakable")
	bool bStartAsleep = true;

	/// A weld snapped (force threshold or BreakAllWelds).
	UPROPERTY(BlueprintAssignable, Category = "Breakable")
	FBox3DWeldBrokeSignature OnWeldBroken;

	/// Snap every weld at once (scripted demolition).
	UFUNCTION(BlueprintCallable, Category = "Breakable")
	void BreakAllWelds();

	UFUNCTION(BlueprintPure, Category = "Breakable")
	int32 GetLiveWeldCount() const { return Welds.Num(); }

	UFUNCTION(BlueprintPure, Category = "Breakable")
	int32 GetChunkCount() const { return ChunkBodies.Num(); }

	/// Add a chunk mesh before FinishSpawning/BeginPlay (runtime construction;
	/// in Blueprint just add StaticMeshComponents instead).
	UStaticMeshComponent* AddChunk(UStaticMesh* Mesh, const FTransform& RelativeTransform);

	/// Snap any weld whose constraint force exceeds BreakForce. Runs from Tick;
	/// public so headless tests can pump it without ticking actors.
	void CheckWelds();

	UBox3DBodyComponent* GetChunkBody(int32 Index) const
	{
		return ChunkBodies.IsValidIndex(Index) ? ChunkBodies[Index] : nullptr;
	}

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void BuildChunks();

	UPROPERTY()
	TArray<TObjectPtr<UBox3DBodyComponent>> ChunkBodies;

	TArray<b3JointId> Welds;
};
