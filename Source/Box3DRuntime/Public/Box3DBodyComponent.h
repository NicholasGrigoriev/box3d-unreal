#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Box3DTypes.h"
#include "box3d/id.h"
#include "Box3DBodyComponent.generated.h"

class UPhysicalMaterial;

/// Simulation type, mirrors b3BodyType (values must stay in enum order).
UENUM(BlueprintType)
enum class EBox3DBodyType : uint8
{
	/// Never moves under simulation; may be repositioned manually.
	Static,
	/// Moved by setting its transform/velocity; pushes dynamic bodies, unaffected by them.
	Kinematic,
	/// Fully simulated: gravity, forces, collisions.
	Dynamic,
};

/// Collision shape attached to the body.
UENUM(BlueprintType)
enum class EBox3DShapeType : uint8
{
	Box,
	Sphere,
	Capsule,
	/// Convex hull cooked from the attached static mesh (authored convex collision
	/// if present, else simplified render geometry).
	ConvexHull,
	/// Exact triangle mesh from the attached static mesh's LOD0 render geometry.
	/// Static bodies only (box3d mesh collision contacts static bodies only);
	/// dynamic/kinematic bodies fall back to ConvexHull with a warning.
	TriangleMesh,
	/// One Box3D shape per element of the attached mesh's authored collision setup
	/// (sphere, capsule, box, and convex elements). Falls back to TriangleMesh
	/// (static) or ConvexHull (other types) when the mesh has no simple collision.
	CollisionAsset,
};

/// Axes on which the body's motion is locked (world axes).
USTRUCT(BlueprintType)
struct FBox3DMotionLocks
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	bool bLinearX = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	bool bLinearY = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	bool bLinearZ = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	bool bAngularX = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	bool bAngularY = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	bool bAngularZ = false;
};

/// A rigid body simulated by Box3D.
///
/// Use as (or attach as) the transform root of whatever should move: dynamic bodies
/// drive the component transform from physics after each step; static and kinematic
/// bodies push the component transform into physics instead. The body and its shape
/// are created on BeginPlay; shape/material properties are creation-time only in M1.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BOX3DRUNTIME_API UBox3DBodyComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UBox3DBodyComponent();

	/// Simulation type of the body.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body")
	EBox3DBodyType BodyType = EBox3DBodyType::Dynamic;

	/// Scales world gravity for this body. 0 disables gravity.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body")
	float GravityScale = 1.0f;

	/// Linear velocity damping (1/s). Generally keep at 0; large values feel floaty.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body", meta = (ClampMin = "0"))
	float LinearDamping = 0.0f;

	/// Angular velocity damping (1/s).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body", meta = (ClampMin = "0"))
	float AngularDamping = 0.0f;

	/// Lock translation/rotation on specific world axes.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body")
	FBox3DMotionLocks MotionLocks;

	/// Allow this body to sleep when at rest.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body")
	bool bEnableSleep = true;

	/// Start the simulation awake.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body")
	bool bStartAwake = true;

	/// Continuous collision for fast movers. Use sparingly (see box3d docs); prefer
	/// casts for projectiles.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Body")
	bool bIsBullet = false;

	/// Shape attached to the body.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Shape")
	EBox3DShapeType ShapeType = EBox3DShapeType::Box;

	/// Derive shape extents from the nearest attached primitive (first child, else
	/// attach parent) at creation. Falls back to the explicit extents below.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Shape")
	bool bAutoFitShape = true;

	/// Box half extents in cm (used when not auto-fitted).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Shape", meta = (EditCondition = "ShapeType == EBox3DShapeType::Box"))
	FVector BoxHalfExtent = FVector(25.0);

	/// Sphere radius in cm (used when not auto-fitted).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Shape", meta = (ClampMin = "0.1", EditCondition = "ShapeType == EBox3DShapeType::Sphere"))
	float SphereRadius = 25.0f;

	/// Capsule radius in cm (used when not auto-fitted).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Shape", meta = (ClampMin = "0.1", EditCondition = "ShapeType == EBox3DShapeType::Capsule"))
	float CapsuleRadius = 25.0f;

	/// Capsule half height in cm, UE-style: from center to hemisphere tip. The capsule
	/// axis is the component's local Z.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Shape", meta = (ClampMin = "0.1", EditCondition = "ShapeType == EBox3DShapeType::Capsule"))
	float CapsuleHalfHeight = 50.0f;

	/// Density in kg/m^3 (1000 = water). Drives mass from shape volume.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Material", meta = (ClampMin = "0"))
	float Density = 1000.0f;

	/// Coulomb friction coefficient, typically [0, 1].
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Material", meta = (ClampMin = "0"))
	float Friction = 0.6f;

	/// Restitution (bounciness), typically [0, 1].
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Material", meta = (ClampMin = "0"))
	float Restitution = 0.0f;

	/// Optional UE physical material; when set, its friction/restitution override the
	/// values above at body creation.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Material")
	TObjectPtr<UPhysicalMaterial> PhysicalMaterial = nullptr;

	/// Opaque per-shape material id, returned by queries and passed to custom
	/// friction/restitution mixers. Not interpreted by Box3D.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Material")
	int64 UserMaterialId = 0;

	/// Collision filter (categories, mask, group).
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Box3D|Collision")
	FBox3DFilter Filter;

	//~ Runtime API ---------------------------------------------------------------

	/// True while a live Box3D body backs this component.
	UFUNCTION(BlueprintPure, Category = "Box3D")
	bool IsSimulating() const;

	/// Linear velocity of the body origin in cm/s.
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	FVector GetLinearVelocity() const;

	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void SetLinearVelocity(FVector VelocityCmPerSec);

	/// Angular velocity in radians/s.
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	FVector GetAngularVelocity() const;

	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void SetAngularVelocity(FVector RadiansPerSec);

	/// Apply a force in UE units (kg*cm/s^2) at the center of mass. Wakes the body.
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void AddForce(FVector Force);

	/// Apply an instantaneous impulse in UE units (kg*cm/s) at the center of mass.
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void AddImpulse(FVector Impulse);

	/// Apply a torque (kg*cm^2/s^2 around world axes). Wakes the body.
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void AddTorque(FVector Torque);

	/// Apply an angular impulse (kg*cm^2/s).
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void AddAngularImpulse(FVector AngularImpulse);

	UFUNCTION(BlueprintPure, Category = "Box3D")
	bool IsAwake() const;

	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void SetAwake(bool bAwake);

	/// Enable/disable the body in the simulation. Disabled bodies neither move nor collide.
	UFUNCTION(BlueprintCallable, Category = "Box3D")
	void SetBodyEnabled(bool bEnabled);

	UFUNCTION(BlueprintPure, Category = "Box3D")
	bool IsBodyEnabled() const;

	/// Mass in kg (0 for static/kinematic bodies).
	UFUNCTION(BlueprintPure, Category = "Box3D")
	float GetMass() const;

	/// The underlying Box3D handle (C++ only).
	b3BodyId GetBodyId() const { return BodyId; }

	/// Called by the world subsystem after a step moved this body.
	void SyncTransformFromPhysics(const FVector& NewLocation, const FQuat& NewRotation);

	//~ UActorComponent
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
	//~ USceneComponent
	virtual void OnUpdateTransform(EUpdateTransformFlags UpdateTransformFlags, ETeleportType Teleport) override;

private:
	void CreateBody();
	void DestroyBody();
	void CreateShape(const FVector& WorldScale);

	/// Resolve auto-fit extents from the nearest attached primitive into the explicit
	/// extent properties. Returns false if no primitive was found.
	bool TryAutoFitShape();

	/// The nearest attached primitive (first child, else attach parent) that shape
	/// geometry and auto-fit are derived from.
	const UPrimitiveComponent* FindSourcePrimitive() const;

	b3BodyId BodyId = {};

	/// Guards against OnUpdateTransform feeding physics-driven moves back into physics.
	bool bSyncingFromPhysics = false;
};
