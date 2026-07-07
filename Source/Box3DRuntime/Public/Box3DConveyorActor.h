#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "box3d/id.h"
#include "Box3DConveyorActor.generated.h"

class UStaticMeshComponent;

/// A conveyor belt: a static Box3D box whose surface material carries a
/// tangent velocity (box3d's built-in conveyor feature), so any dynamic Box3D
/// body resting on it gets dragged along the actor's local +X. The mesh is an
/// ordinary static mesh component — visible in the editor, walkable via Chaos.
///
/// Scale the actor/mesh into a belt shape and aim local X down the line.
/// The Box3D collision is the mesh's local bounds as a box.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DConveyorActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DConveyorActor();

	/// Surface speed along local +X in cm/s. Negative reverses the belt.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Conveyor")
	float BeltSpeed = 200.0f;

	/// Belt grip. Low friction lets heavy props slip.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Conveyor", meta = (ClampMin = "0"))
	float Friction = 0.9f;

	/// Change the belt speed at runtime.
	UFUNCTION(BlueprintCallable, Category = "Conveyor")
	void SetBeltSpeed(float CmPerSec);

	UStaticMeshComponent* GetMesh() const { return Mesh; }

	//~ AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void CreateBeltShape();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Conveyor", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UStaticMeshComponent> Mesh;

	b3BodyId BodyId = {};
};
