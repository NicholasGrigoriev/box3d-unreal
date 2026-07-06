#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Box3DTypes.h"
#include "Box3DQueryLibrary.generated.h"

class UBox3DBodyComponent;

/// Blueprint-facing queries against the Box3D world. All positions/lengths in cm.
UCLASS()
class BOX3DRUNTIME_API UBox3DQueryLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/// Closest-hit ray cast. Returns true on hit.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static bool Box3DRayCast(UObject* WorldContextObject, FVector Start, FVector End,
		const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit);

	/// All hits along a ray, sorted near to far.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static TArray<FBox3DHitResult> Box3DRayCastMulti(UObject* WorldContextObject, FVector Start, FVector End,
		const FBox3DQueryFilter& Filter);

	/// Closest-hit sphere cast from Start to End. Returns true on hit.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static bool Box3DSphereCast(UObject* WorldContextObject, FVector Start, FVector End, float Radius,
		const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit);

	/// Closest-hit vertical-capsule cast from Start to End (capsule axis is world Z,
	/// HalfHeight measured to the hemisphere tip, UE-style). Returns true on hit.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static bool Box3DCapsuleCast(UObject* WorldContextObject, FVector Start, FVector End, float Radius,
		float HalfHeight, const FBox3DQueryFilter& Filter, FBox3DHitResult& OutHit);

	/// All body components with a shape overlapping the sphere.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Query", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static TArray<UBox3DBodyComponent*> Box3DOverlapSphere(UObject* WorldContextObject, FVector Center, float Radius,
		const FBox3DQueryFilter& Filter);

	/// Radial explosion: applies an outward impulse to every dynamic body within
	/// Radius of Center, tapering to zero across Falloff beyond it. ImpulsePerArea
	/// is impulse per unit of shape area facing the blast, in kg·cm/s per cm²;
	/// negative values implode. Velocities change immediately (no step needed).
	/// Only the filter's mask bits apply. Spheres, capsules, and hulls only;
	/// per-shape opt-out via box3d's explosionScale (default 1).
	UFUNCTION(BlueprintCallable, Category = "Box3D|World", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static void Box3DExplode(UObject* WorldContextObject, FVector Center, float Radius, float Falloff,
		float ImpulsePerArea, const FBox3DQueryFilter& Filter);

	//~ Character mover helpers (box3d's kinematic mover toolkit) -----------------

	/// Cast a vertical capsule mover (center at Position) along Translation.
	/// Returns the safe fraction [0, 1] of the translation; slides along surfaces
	/// rather than reporting them — pair with Box3DSolveMoverDelta for contacts.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Mover", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static float Box3DCastMover(UObject* WorldContextObject, FVector Position, FVector Translation, float Radius,
		float HalfHeight, const FBox3DQueryFilter& Filter);

	/// Collide-and-slide: gather contact planes around a capsule mover at Position
	/// and solve DesiredDelta against them (box3d's b3SolvePlanes). Returns the
	/// adjusted delta; OutPlaneCount reports how many contact planes were found.
	/// This is the core of kinematic character movement against the Box3D world.
	UFUNCTION(BlueprintCallable, Category = "Box3D|Mover", meta = (WorldContext = "WorldContextObject", AutoCreateRefTerm = "Filter"))
	static FVector Box3DSolveMoverDelta(UObject* WorldContextObject, FVector Position, float Radius, float HalfHeight,
		FVector DesiredDelta, const FBox3DQueryFilter& Filter, int32& OutPlaneCount);
};
