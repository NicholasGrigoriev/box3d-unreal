// Tests for mesh cooking (M3): triangle mesh cooking with winding + non-uniform
// scale, convex hull cooking, per-asset caching, and CollisionAsset element
// translation — using engine basic shape meshes as fixtures.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DCooking.h"
#include "Box3DQueryLibrary.h"
#include "PhysicsEngine/BodySetup.h"
#include "Tests/Box3DTestHelpers.h"

namespace
{
	UStaticMesh* LoadBasicShape(const TCHAR* Name)
	{
		return LoadObject<UStaticMesh>(nullptr, *FString::Printf(TEXT("/Engine/BasicShapes/%s.%s"), Name, Name));
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DCookingMeshTest,
	"Box3DUnreal.Cooking.TriangleMeshWindingAndCache", BOX3D_TEST_FLAGS)
bool FBox3DCookingMeshTest::RunTest(const FString& Parameters)
{
	UStaticMesh* Cube = LoadBasicShape(TEXT("Cube"));
	if (!TestNotNull(TEXT("engine cube mesh loads"), Cube))
	{
		return false;
	}

	// Cooked data is cached per asset: the second cook is a pointer hit.
	const b3MeshData* MeshData = Box3D::GetOrCreateMeshData(Cube);
	TestNotNull(TEXT("cube cooks to mesh data"), MeshData);
	TestTrue(TEXT("mesh cook is cached per asset"), Box3D::GetOrCreateMeshData(Cube) == MeshData);

	// A static trimesh ground with non-uniform scale (4, 4, 0.5): the 100 cm cube
	// becomes 400x400x50, top face at Z=25. If winding were wrong, the ray would
	// pass through the top and hit an inward-facing bottom face instead.
	Box3DTest::FTestWorld Test;
	Box3DTest::SpawnMeshBody(Test.World, Cube, FVector::ZeroVector, FVector(4.0, 4.0, 0.5),
		EBox3DBodyType::Static, EBox3DShapeType::TriangleMesh);

	FBox3DHitResult Hit;
	TestTrue(TEXT("ray hits the trimesh"), UBox3DQueryLibrary::Box3DRayCast(Test.World,
		FVector(0, 0, 300), FVector(0, 0, -300), FBox3DQueryFilter(), Hit));
	TestEqual(TEXT("hit lands on the scaled top face"), static_cast<float>(Hit.Location.Z), 25.0f, 0.5f);
	TestTrue(TEXT("top face normal points up (winding correct)"), Hit.Normal.Z > 0.99);
	TestTrue(TEXT("mesh hits carry a triangle index"), Hit.TriangleIndex >= 0);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DCookingHullTest,
	"Box3DUnreal.Cooking.ConvexHullCacheAndMass", BOX3D_TEST_FLAGS)
bool FBox3DCookingHullTest::RunTest(const FString& Parameters)
{
	UStaticMesh* Cylinder = LoadBasicShape(TEXT("Cylinder"));
	if (!TestNotNull(TEXT("engine cylinder mesh loads"), Cylinder))
	{
		return false;
	}

	const b3HullData* HullData = Box3D::GetOrCreateHullData(Cylinder);
	TestNotNull(TEXT("cylinder cooks to hull data"), HullData);
	TestTrue(TEXT("hull cook is cached per asset"), Box3D::GetOrCreateHullData(Cylinder) == HullData);

	// The engine cylinder is r=50 cm, h=100 cm: exact volume pi*r^2*h = 0.785 m^3,
	// so 785 kg at density 1000. A 64-vertex hull comes in slightly under.
	Box3DTest::FTestWorld Test;
	UBox3DBodyComponent* Body = Box3DTest::SpawnMeshBody(Test.World, Cylinder, FVector(0, 0, 500),
		FVector::OneVector, EBox3DBodyType::Dynamic, EBox3DShapeType::ConvexHull);
	TestTrue(TEXT("hull body simulates"), Body->IsSimulating());
	TestTrue(FString::Printf(TEXT("hull mass close under analytic cylinder (%.1f kg)"), Body->GetMass()),
		Body->GetMass() > 740.0f && Body->GetMass() <= 790.0f);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DCookingCollisionAssetTest,
	"Box3DUnreal.Cooking.CollisionAssetElements", BOX3D_TEST_FLAGS)
bool FBox3DCookingCollisionAssetTest::RunTest(const FString& Parameters)
{
	UStaticMesh* Cube = LoadBasicShape(TEXT("Cube"));
	UStaticMesh* Sphere = LoadBasicShape(TEXT("Sphere"));
	if (!TestNotNull(TEXT("engine cube mesh loads"), Cube) || !TestNotNull(TEXT("engine sphere mesh loads"), Sphere))
	{
		return false;
	}

	Box3DTest::FTestWorld Test;

	// The cube's authored collision is a single box element: exact 1 m^3 mass.
	UBox3DBodyComponent* CubeBody = Box3DTest::SpawnMeshBody(Test.World, Cube, FVector(0, 0, 500),
		FVector::OneVector, EBox3DBodyType::Dynamic, EBox3DShapeType::CollisionAsset);
	const int32 CubeElements = Cube->GetBodySetup() ? Cube->GetBodySetup()->AggGeom.GetElementCount() : 0;
	TestTrue(TEXT("cube has authored collision elements"), CubeElements > 0);
	TestEqual(TEXT("one shape per authored element"), b3Body_GetShapeCount(CubeBody->GetBodyId()), CubeElements);
	TestEqual(TEXT("authored box element gives exact mass"), CubeBody->GetMass(), 1000.0f, 5.0f);

	// The sphere's collision setup drives the sphere-element path if present;
	// engine content details may vary, so assert only when the data exists.
	if (Sphere->GetBodySetup() && Sphere->GetBodySetup()->AggGeom.SphereElems.Num() > 0)
	{
		UBox3DBodyComponent* SphereBody = Box3DTest::SpawnMeshBody(Test.World, Sphere, FVector(300, 0, 500),
			FVector::OneVector, EBox3DBodyType::Dynamic, EBox3DShapeType::CollisionAsset);
		TestEqual(TEXT("authored sphere element gives analytic mass"), SphereBody->GetMass(), 523.6f, 10.0f);
	}
	else
	{
		AddInfo(TEXT("engine sphere has no authored sphere element; skipping sphere-element mass check"));
	}

	// Cone ships with convex collision in most engine versions: covers the
	// FKConvexElem -> b3CreateHull path when available.
	if (UStaticMesh* Cone = LoadBasicShape(TEXT("Cone")))
	{
		if (Cone->GetBodySetup() && Cone->GetBodySetup()->AggGeom.ConvexElems.Num() > 0)
		{
			UBox3DBodyComponent* ConeBody = Box3DTest::SpawnMeshBody(Test.World, Cone, FVector(600, 0, 500),
				FVector::OneVector, EBox3DBodyType::Dynamic, EBox3DShapeType::CollisionAsset);
			TestTrue(TEXT("convex element cooks into a shape"), b3Body_GetShapeCount(ConeBody->GetBodyId()) > 0);
			// Analytic cone: 1/3*pi*r^2*h = 0.262 m^3 -> 262 kg; hulls land nearby.
			TestTrue(FString::Printf(TEXT("cone hull mass plausible (%.1f kg)"), ConeBody->GetMass()),
				ConeBody->GetMass() > 200.0f && ConeBody->GetMass() < 320.0f);
		}
		else
		{
			AddInfo(TEXT("engine cone has no convex elements; skipping convex-element check"));
		}
	}

	// Auto-fit from an attached mesh primitive (the M1 smoke path, pinned here):
	// a sphere primitive fitted to the 50 cm-radius sphere mesh.
	UBox3DBodyComponent* Fitted = Box3DTest::SpawnMeshBody(Test.World, Sphere, FVector(900, 0, 500),
		FVector::OneVector, EBox3DBodyType::Dynamic, EBox3DShapeType::Sphere);
	TestEqual(TEXT("auto-fit sphere radius from mesh bounds"), Fitted->GetMass(), 523.6f, 10.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
