#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Box3DPropActor.generated.h"

class UBox3DBodyComponent;
class UStaticMesh;
class UStaticMeshComponent;

/// A Box3D-simulated static mesh prop.
///
/// Root is a dynamic UBox3DBodyComponent (CollisionAsset: authored simple collision
/// when the mesh has any, convex hull otherwise); the mesh renders as a child with
/// Chaos collision QueryOnly, so engine traces and character movement still see the
/// prop while Box3D owns its motion. Drop into a level and pick a mesh, or spawn
/// via Box3D::ConvertToProp / the box3d.MakeProp console command.
UCLASS(BlueprintType, Blueprintable, ClassGroup = (Physics))
class BOX3DRUNTIME_API ABox3DPropActor : public AActor
{
	GENERATED_BODY()

public:
	ABox3DPropActor();

	/// Assign the rendered/cooked mesh. Shapes are creation-time: call before
	/// FinishSpawning/BeginPlay (use SpawnActorDeferred when spawning at runtime).
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void SetStaticMesh(UStaticMesh* Mesh);

	UBox3DBodyComponent* GetBody() const { return Body; }
	UStaticMeshComponent* GetMesh() const { return Mesh; }

private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Box3D", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UBox3DBodyComponent> Body;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Box3D", meta = (AllowPrivateAccess = "true"))
	TObjectPtr<UStaticMeshComponent> Mesh;
};
