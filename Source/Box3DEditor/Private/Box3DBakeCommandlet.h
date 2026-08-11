#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "Box3DBakeCommandlet.generated.h"

/// Bakes a map's static Box3D collision into a UBox3DCollisionData asset
/// (BC_<MapName>, saved beside the map) so packaged builds instantiate static
/// geometry without runtime cooking. Extracts exactly what the static scene
/// mirror would cook — same component filter, same geometry choice, same
/// winding — so a baked world collides identically to a mirrored one.
///
/// Usage:
///   UnrealEditor-Cmd.exe <Project>.uproject -run=Box3DBake -Map=/Game/Maps/Foo[,/Game/Maps/Bar]
///
/// Concept ported from Antonio Lattanzio's Box3DUnreal
/// (github.com/alattanzio/Box3DUnreal, MIT). See docs/BAKED_COLLISION.md.
UCLASS()
class UBox3DBakeCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	UBox3DBakeCommandlet();

	virtual int32 Main(const FString& Params) override;

private:
	/// Bake one map. Returns false on any error worth failing the run for.
	bool BakeMap(const FString& MapPackageName);
};
