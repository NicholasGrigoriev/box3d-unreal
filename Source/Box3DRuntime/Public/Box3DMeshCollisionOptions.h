#pragma once

#include "CoreMinimal.h"
#include "Engine/AssetUserData.h"
#include "Box3DMeshCollisionOptions.generated.h"

/// Asset user data on a static mesh that shapes the Box3D triangle mesh cooked from it (the static scene mirror and
/// any other TriangleMesh use). Sections whose material slot is listed are left out of the cooked collision: a
/// decorative or breakable outer layer (a chippable shell over a core) then does not trap bodies between itself
/// and what lies behind it. Chaos collision and rendering are unaffected.
UCLASS(BlueprintType, EditInlineNew, DefaultToInstanced)
class BOX3DRUNTIME_API UBox3DMeshCollisionOptions : public UAssetUserData
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	TArray<FName> ExcludedMaterialSlots;
};
