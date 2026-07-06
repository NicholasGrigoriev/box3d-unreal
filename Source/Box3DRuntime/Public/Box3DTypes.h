#pragma once

#include "CoreMinimal.h"
#include "box3d/id.h"
#include "Box3DTypes.generated.h"

class AActor;
class UBox3DBodyComponent;

/// Collision channels for Box3D shapes and queries. Values are BIT INDICES into the
/// 64-bit box3d category/mask (editor bitmask widgets treat them as such). The names
/// are conventions only — box3d itself sees raw bits.
UENUM(BlueprintType, meta = (Bitflags))
enum class EBox3DChannel : uint8
{
	WorldStatic = 0,
	WorldDynamic = 1,
	Pawn = 2,
	Projectile = 3,
	Debris = 4,
	Sensor = 5,
	Custom6 = 6,
	Custom7 = 7,
	Custom8 = 8,
	Custom9 = 9,
};

/// Shape collision filter, a thin UE face over b3Filter.
USTRUCT(BlueprintType)
struct BOX3DRUNTIME_API FBox3DFilter
{
	GENERATED_BODY()

	/// What this shape is (its categories).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D",
		meta = (Bitmask, BitmaskEnum = "/Script/Box3DRuntime.EBox3DChannel"))
	int32 CategoryBits = 1 << static_cast<int32>(EBox3DChannel::WorldDynamic);

	/// Which categories this shape collides with. Default: everything.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D",
		meta = (Bitmask, BitmaskEnum = "/Script/Box3DRuntime.EBox3DChannel"))
	int32 MaskBits = -1;

	/// Non-zero group overrides the mask: same positive group always collides, same
	/// negative group never collides (e.g. give a ragdoll a unique negative group).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D")
	int32 GroupIndex = 0;
};

/// Query filter, a thin UE face over b3QueryFilter.
USTRUCT(BlueprintType)
struct BOX3DRUNTIME_API FBox3DQueryFilter
{
	GENERATED_BODY()

	/// What the query itself is. Default: everything.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D",
		meta = (Bitmask, BitmaskEnum = "/Script/Box3DRuntime.EBox3DChannel"))
	int32 CategoryBits = -1;

	/// Which shape categories the query accepts. Default: everything.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box3D",
		meta = (Bitmask, BitmaskEnum = "/Script/Box3DRuntime.EBox3DChannel"))
	int32 MaskBits = -1;
};

/// Result of a Box3D ray/shape cast.
USTRUCT(BlueprintType)
struct BOX3DRUNTIME_API FBox3DHitResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	bool bHit = false;

	/// World hit location (cm).
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	FVector Location = FVector::ZeroVector;

	/// Surface normal at the hit.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	FVector Normal = FVector::UpVector;

	/// Fraction along the cast translation [0, 1].
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	float Fraction = 1.0f;

	/// Body component owning the hit shape (null if the body has no component owner).
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	TObjectPtr<UBox3DBodyComponent> Component = nullptr;

	/// Actor owning the hit component.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	TObjectPtr<AActor> Actor = nullptr;

	/// Shape/triangle user material id at the hit point.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int64 UserMaterialId = 0;

	/// Triangle index for mesh/height-field hits, -1 otherwise.
	UPROPERTY(BlueprintReadOnly, Category = "Box3D")
	int32 TriangleIndex = -1;
};

namespace Box3D
{
	/// int32 bitmask (UE editor-facing) -> 64-bit box3d bits. -1 means "all bits",
	/// including the upper 32 box3d bits UE never names.
	inline uint64 ToB3Bits(int32 Bits)
	{
		return Bits == -1 ? UINT64_MAX : static_cast<uint64>(static_cast<uint32>(Bits));
	}

	/// Resolve a shape hit back to the owning component/actor via body userData.
	BOX3DRUNTIME_API UBox3DBodyComponent* ResolveComponent(b3ShapeId ShapeId);
}
