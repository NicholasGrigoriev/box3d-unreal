#include "Box3DBakedGeometry.h"

#include "Box3DCollisionData.h"
#include "Box3DConversion.h"
#include "Box3DRuntime.h"
#include "Box3DTypes.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	/// Baked points are UE cm with scale already applied; shapes are placed by
	/// location + rotation only.
	int32 CreateBakedShapes(b3BodyId BodyId, b3ShapeDef& ShapeDef, const FBox3DBakedBody& Body,
		TArray<b3MeshData*>& OutMeshes)
	{
		int32 Created = 0;

		for (const FBox3DBakedShape& Shape : Body.Shapes)
		{
			switch (Shape.Kind)
			{
			case EBox3DBakedShapeKind::Sphere:
			{
				const b3Sphere Sphere{ Box3D::ToB3(FVector(Shape.CenterA)),
									   Shape.Radius * Box3D::UEToMeters };
				b3CreateSphereShape(BodyId, &ShapeDef, &Sphere);
				++Created;
				break;
			}
			case EBox3DBakedShapeKind::Capsule:
			{
				const b3Capsule Capsule{ Box3D::ToB3(FVector(Shape.CenterA)),
										 Box3D::ToB3(FVector(Shape.CenterB)),
										 Shape.Radius * Box3D::UEToMeters };
				b3CreateCapsuleShape(BodyId, &ShapeDef, &Capsule);
				++Created;
				break;
			}
			case EBox3DBakedShapeKind::Hull:
			{
				TArray<b3Vec3> Points;
				Points.Reserve(Shape.Points.Num());
				for (const FVector3f& Point : Shape.Points)
				{
					Points.Add(Box3D::ToB3(FVector(Point)));
				}
				if (Points.Num() < 4)
				{
					break;
				}
				if (b3HullData* Hull = b3CreateHull(Points.GetData(), Points.Num(), 64))
				{
					// Hull shapes clone the data, so the temp hull is freed at once.
					b3CreateHullShape(BodyId, &ShapeDef, Hull);
					b3DestroyHull(Hull);
					++Created;
				}
				break;
			}
			case EBox3DBakedShapeKind::Mesh:
			{
				if (Shape.Points.Num() < 3 || Shape.Indices.Num() < 3)
				{
					break;
				}
				TArray<b3Vec3> Vertices;
				Vertices.Reserve(Shape.Points.Num());
				for (const FVector3f& Point : Shape.Points)
				{
					Vertices.Add(Box3D::ToB3(FVector(Point)));
				}

				// Same cook parameters as the runtime mirror (Box3DCooking.cpp);
				// indices were flipped to box3d winding at bake time.
				b3MeshDef Def = {};
				Def.vertices = Vertices.GetData();
				Def.indices = const_cast<int32*>(Shape.Indices.GetData());
				Def.vertexCount = Vertices.Num();
				Def.triangleCount = Shape.Indices.Num() / 3;
				Def.weldVertices = true;
				Def.weldTolerance = 0.001f;
				Def.identifyEdges = true;

				b3MeshData* MeshData = b3CreateMesh(&Def, nullptr, 0);
				if (MeshData == nullptr)
				{
					UE_LOG(LogBox3D, Warning, TEXT("Baked mesh cook failed for %s (%d verts, %d tris)"),
						*Body.ActorKey, Vertices.Num(), Shape.Indices.Num() / 3);
					break;
				}
				// Mesh shapes reference the data rather than cloning it, so the
				// scene keeps it alive until the bodies are destroyed.
				OutMeshes.Add(MeshData);
				b3CreateMeshShape(BodyId, &ShapeDef, MeshData, b3Vec3{ 1.0f, 1.0f, 1.0f });
				++Created;
				break;
			}
			}
		}

		return Created;
	}
}

int32 FBox3DBakedScene::Instantiate(b3WorldId WorldId, const UBox3DCollisionData& Data)
{
	if (!b3World_IsValid(WorldId))
	{
		return 0;
	}

	const double StartSeconds = FPlatformTime::Seconds();
	int32 CreatedBodies = 0;

	for (const FBox3DBakedBody& Baked : Data.Bodies)
	{
		b3BodyDef BodyDef = b3DefaultBodyDef();
		BodyDef.type = b3_staticBody;
		BodyDef.position = Box3D::ToB3Pos(Baked.WorldTransform.GetLocation());
		BodyDef.rotation = Box3D::ToB3(Baked.WorldTransform.GetRotation());
		// Queries resolve userData straight to UObject; baked bodies have no
		// UObject, so this MUST stay null (hits report a null component).
		BodyDef.userData = nullptr;

		const b3BodyId BodyId = b3CreateBody(WorldId, &BodyDef);

		b3ShapeDef ShapeDef = b3DefaultShapeDef();
		ShapeDef.filter.categoryBits = Box3D::ToB3Bits(1 << static_cast<int32>(EBox3DChannel::WorldStatic));
		ShapeDef.filter.maskBits = UINT64_MAX;
		ShapeDef.enableSensorEvents = false;
		ShapeDef.enableContactEvents = false;
		ShapeDef.enableHitEvents = false;

		if (CreateBakedShapes(BodyId, ShapeDef, Baked, Meshes) == 0)
		{
			b3DestroyBody(BodyId);
			continue;
		}

		Bodies.Add(b3StoreBodyId(BodyId));
		++CreatedBodies;
	}

	UE_LOG(LogBox3D, Log, TEXT("Box3D baked collision: %s — %d bodies instantiated in %.1f ms"),
		*GetNameSafe(&Data), CreatedBodies, (FPlatformTime::Seconds() - StartSeconds) * 1000.0);
	return CreatedBodies;
}

void FBox3DBakedScene::Destroy()
{
	for (const uint64 PackedId : Bodies)
	{
		const b3BodyId BodyId = b3LoadBodyId(PackedId);
		if (b3Body_IsValid(BodyId))
		{
			b3DestroyBody(BodyId);
		}
	}
	Bodies.Empty();

	// After the bodies: mesh shapes reference this data.
	for (b3MeshData* Mesh : Meshes)
	{
		b3DestroyMesh(Mesh);
	}
	Meshes.Empty();
}
