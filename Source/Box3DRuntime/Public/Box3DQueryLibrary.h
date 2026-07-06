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
};
