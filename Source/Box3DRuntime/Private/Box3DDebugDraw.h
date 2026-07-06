#pragma once

#include "CoreMinimal.h"
#include "box3d/types.h"

class UWorld;

/// Persistent debug-draw shape cache plus the b3World_Draw pass.
///
/// box3d draws shapes exclusively through user objects: the world's
/// createDebugShape callback runs the first time a shape is drawn, the returned
/// pointer is cached on the shape, and destroyDebugShape runs when the shape (or
/// world) is destroyed. Without these callbacks drawShapes renders nothing, so the
/// subsystem registers them on every world — they cost nothing until a draw pass
/// actually happens.
///
/// The cached object is a wireframe line list in shape-local box3d meters;
/// drawing transforms it by the body transform and hands line segments to
/// DrawDebugHelpers.
class FBox3DDebugDrawer
{
public:
	/// Line-pair list (Points[0]->Points[1], Points[2]->Points[3], ...) in
	/// shape-local space, box3d meters.
	struct FWireShape
	{
		TArray<FVector3f> Points;
	};

	~FBox3DDebugDrawer();

	/// b3WorldDef::createDebugShape / destroyDebugShape thunks. Context is the
	/// drawer instance, which must outlive the world.
	static void* CreateShapeThunk(const b3DebugShape* DebugShape, void* Context);
	static void DestroyShapeThunk(void* UserShape, void* Context);

	/// Cache statistics, exposed for tests and leak sanity.
	int32 GetCreatedCount() const { return CreatedCount; }
	int32 GetDestroyedCount() const { return DestroyedCount; }
	int32 GetAliveCount() const { return CreatedCount - DestroyedCount; }

	/// Run a b3World_Draw pass into DrawDebugHelpers, configured from the
	/// box3d.DebugDraw.* CVars. Bounds are centered on the local player's view.
	void Draw(const UWorld* World, b3WorldId WorldId) const;

	/// True when the box3d.DebugDraw master switch is on.
	static bool IsDrawEnabled();

private:
	FWireShape* BuildWireShape(const b3DebugShape& DebugShape);

	int32 CreatedCount = 0;
	int32 DestroyedCount = 0;
};
