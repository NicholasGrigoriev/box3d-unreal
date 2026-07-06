// M5 diagnostics: GetWorldStats (the data behind `stat box3d`) and the
// debug-draw wireframe cache driven by box3d's create/destroy shape callbacks.

#include "Box3DTestHelpers.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DDebugDraw.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDiagnosticsWorldStatsTest,
	"Box3DUnreal.Diagnostics.WorldStats", BOX3D_TEST_FLAGS)
bool FBox3DDiagnosticsWorldStatsTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World);
	for (int32 Index = 0; Index < 2; ++Index)
	{
		Box3DTest::SpawnBody(Test.World, FVector(0, 0, 60.0f + Index * 110.0f), [](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Dynamic;
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(50.0);
		});
	}

	// 30 steps: the stack lands on the ground, so contacts must exist and the
	// profile must have timed real solver work.
	Test.Step(30);

	const FBox3DWorldStats Stats = Test.Subsystem().GetWorldStats();
	TestEqual(TEXT("BodyCount"), Stats.BodyCount, 3);
	TestEqual(TEXT("ShapeCount"), Stats.ShapeCount, 3);
	TestEqual(TEXT("JointCount"), Stats.JointCount, 0);
	TestTrue(TEXT("ContactCount > 0 once the stack touches"), Stats.ContactCount > 0);
	TestTrue(TEXT("IslandCount >= 1"), Stats.IslandCount >= 1);
	TestTrue(TEXT("AwakeBodyCount covers the settling boxes"), Stats.AwakeBodyCount >= 1);
	TestTrue(TEXT("StepMs measured"), Stats.StepMs > 0.0f);
	TestTrue(TEXT("SolveMs measured"), Stats.SolveMs > 0.0f);
	TestTrue(TEXT("MemoryBytes > 0"), Stats.MemoryBytes > 0);

	return true;
}

namespace
{
	struct FDrawCounts
	{
		int32 ShapeCalls = 0;
		int32 BoundsCalls = 0;
		TArray<int32> WirePointCounts;
	};

	/// b3World_Draw pass with counting callbacks, exercising the subsystem-owned
	/// shape cache without any DrawDebugHelpers involvement.
	void CountingDraw(b3WorldId WorldId, FDrawCounts& Counts)
	{
		b3DebugDraw Draw = b3DefaultDebugDraw();
		Draw.context = &Counts;
		Draw.drawShapes = true;
		Draw.drawBounds = true;
		Draw.drawingBounds = b3AABB{ b3Vec3{ -100.0f, -100.0f, -100.0f }, b3Vec3{ 100.0f, 100.0f, 100.0f } };

		Draw.DrawShapeFcn = [](void* UserShape, b3WorldTransform, b3HexColor, void* Context) -> bool
		{
			FDrawCounts* C = static_cast<FDrawCounts*>(Context);
			++C->ShapeCalls;
			C->WirePointCounts.Add(static_cast<const FBox3DDebugDrawer::FWireShape*>(UserShape)->Points.Num());
			return true;
		};
		Draw.DrawBoundsFcn = [](b3AABB, b3HexColor, void* Context)
		{
			++static_cast<FDrawCounts*>(Context)->BoundsCalls;
		};
		Draw.DrawSegmentFcn = [](b3Pos, b3Pos, b3HexColor, void*) {};
		Draw.DrawTransformFcn = [](b3WorldTransform, void*) {};
		Draw.DrawPointFcn = [](b3Pos, float, b3HexColor, void*) {};
		Draw.DrawSphereFcn = [](b3Pos, float, b3HexColor, float, void*) {};
		Draw.DrawCapsuleFcn = [](b3Pos, b3Pos, float, b3HexColor, float, void*) {};
		Draw.DrawBoxFcn = [](b3Vec3, b3WorldTransform, b3HexColor, void*) {};
		Draw.DrawStringFcn = [](b3Pos, const char*, b3HexColor, void*) {};

		b3World_Draw(WorldId, &Draw, UINT64_MAX);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DDiagnosticsDebugDrawCacheTest,
	"Box3DUnreal.Diagnostics.DebugDrawCache", BOX3D_TEST_FLAGS)
bool FBox3DDiagnosticsDebugDrawCacheTest::RunTest(const FString& Parameters)
{
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnGround(Test.World); // box -> hull wireframe
	Box3DTest::SpawnBody(Test.World, FVector(0, 0, 100), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Dynamic;
		Body.ShapeType = EBox3DShapeType::Box;
		Body.BoxHalfExtent = FVector(25.0);
	});
	UBox3DBodyComponent* Sphere = Box3DTest::SpawnBody(Test.World, FVector(200, 0, 100), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Dynamic;
		Body.ShapeType = EBox3DShapeType::Sphere;
		Body.SphereRadius = 25.0f;
	});
	Box3DTest::SpawnBody(Test.World, FVector(400, 0, 100), [](UBox3DBodyComponent& Body)
	{
		Body.BodyType = EBox3DBodyType::Dynamic;
		Body.ShapeType = EBox3DShapeType::Capsule;
		Body.CapsuleRadius = 25.0f;
		Body.CapsuleHalfHeight = 75.0f;
	});

	FBox3DDebugDrawer* Drawer = Test.Subsystem().GetDebugDrawer();
	TestNotNull(TEXT("drawer registered with the world"), Drawer);
	TestEqual(TEXT("cache empty before any draw"), Drawer->GetCreatedCount(), 0);

	// First draw lazily builds one wireframe per shape.
	FDrawCounts FirstPass;
	CountingDraw(Test.B3World(), FirstPass);
	TestEqual(TEXT("every shape drawn"), FirstPass.ShapeCalls, 4);
	TestEqual(TEXT("every shape's bounds drawn"), FirstPass.BoundsCalls, 4);
	TestEqual(TEXT("one cache entry per shape"), Drawer->GetCreatedCount(), 4);
	TestEqual(TEXT("all entries alive"), Drawer->GetAliveCount(), 4);

	// Wireframe geometry: box hulls have 12 edges (24 points), the sphere wire is
	// 3 circles x 16 segments (96 points), the capsule 2 rings + 4 sides + 4 arcs
	// (136 points). Both boxes (ground + dynamic) share the 24-point shape.
	FirstPass.WirePointCounts.Sort();
	const TArray<int32> ExpectedCounts = { 24, 24, 96, 136 };
	TestTrue(FString::Printf(TEXT("wireframe point counts per shape type (got %s)"),
			*FString::JoinBy(FirstPass.WirePointCounts, TEXT(","), [](int32 N) { return FString::FromInt(N); })),
		FirstPass.WirePointCounts == ExpectedCounts);

	// Second draw reuses the cache instead of rebuilding.
	FDrawCounts SecondPass;
	CountingDraw(Test.B3World(), SecondPass);
	TestEqual(TEXT("second draw still hits every shape"), SecondPass.ShapeCalls, 4);
	TestEqual(TEXT("no new cache entries on redraw"), Drawer->GetCreatedCount(), 4);

	// Destroying a body destroys its shape, which must free its cache entry.
	Test.World->DestroyActor(Sphere->GetOwner());
	TestEqual(TEXT("destroyed shape released its wireframe"), Drawer->GetDestroyedCount(), 1);
	TestEqual(TEXT("three entries remain"), Drawer->GetAliveCount(), 3);

	FDrawCounts ThirdPass;
	CountingDraw(Test.B3World(), ThirdPass);
	TestEqual(TEXT("remaining shapes drawn after destroy"), ThirdPass.ShapeCalls, 3);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
