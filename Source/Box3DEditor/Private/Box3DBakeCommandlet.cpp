#include "Box3DBakeCommandlet.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Box3DCollisionData.h"
#include "Box3DRuntime.h"
#include "Box3DSettings.h"
#include "Box3DStaticSceneMirror.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "FileHelpers.h"
#include "GameFramework/Actor.h"
#include "Misc/PackageName.h"
#include "PhysicsEngine/BodySetup.h"
#include "StaticMeshResources.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "box3d/base.h"

UBox3DBakeCommandlet::UBox3DBakeCommandlet()
{
	IsClient = false;
	IsServer = false;
	IsEditor = true;
	LogToConsole = true;
}

namespace
{
	/// LOD0 render triangles in component-local cm with scale baked in and the
	/// winding already flipped to box3d's convention — the exact geometry
	/// Box3DCooking's GetLOD0Geometry cooks at runtime (keep the two in sync).
	bool ExtractMeshShape(UStaticMesh& Mesh, const FVector& Scale, FBox3DBakedShape& OutShape)
	{
		const FStaticMeshRenderData* RenderData = Mesh.GetRenderData();
		if (RenderData == nullptr || RenderData->LODResources.IsEmpty())
		{
			return false;
		}

		const FStaticMeshLODResources& LOD = RenderData->LODResources[0];
		const FPositionVertexBuffer& Positions = LOD.VertexBuffers.PositionVertexBuffer;
		const int32 VertexCount = Positions.GetNumVertices();
		if (VertexCount < 3)
		{
			return false;
		}

		OutShape.Kind = EBox3DBakedShapeKind::Mesh;
		OutShape.Points.Reserve(VertexCount);
		for (int32 Index = 0; Index < VertexCount; ++Index)
		{
			OutShape.Points.Add(FVector3f(FVector(Positions.VertexPosition(Index)) * Scale));
		}

		TArray<uint32> RawIndices;
		LOD.IndexBuffer.GetCopy(RawIndices);
		const int32 TriangleCount = RawIndices.Num() / 3;
		if (TriangleCount < 1)
		{
			return false;
		}

		// UE winds clockwise viewed from outside; box3d expects the reverse.
		OutShape.Indices.Reserve(TriangleCount * 3);
		for (int32 Triangle = 0; Triangle < TriangleCount; ++Triangle)
		{
			OutShape.Indices.Add(static_cast<int32>(RawIndices[Triangle * 3 + 0]));
			OutShape.Indices.Add(static_cast<int32>(RawIndices[Triangle * 3 + 2]));
			OutShape.Indices.Add(static_cast<int32>(RawIndices[Triangle * 3 + 1]));
		}
		return true;
	}

	/// Authored simple-collision elements as baked shapes, mirroring the scale
	/// math of Box3DCooking's CreateShapesFromBodySetup (kept in cm here; the
	/// loader converts through the ordinary seam).
	int32 ExtractBodySetupShapes(const UBodySetup& BodySetup, const FVector& Scale,
		TArray<FBox3DBakedShape>& OutShapes)
	{
		const FVector AbsScale = Scale.GetAbs();
		int32 Created = 0;

		for (const FKSphereElem& Elem : BodySetup.AggGeom.SphereElems)
		{
			FBox3DBakedShape& Shape = OutShapes.AddDefaulted_GetRef();
			Shape.Kind = EBox3DBakedShapeKind::Sphere;
			Shape.CenterA = FVector3f(FVector(Elem.Center) * Scale);
			Shape.Radius = Elem.Radius * AbsScale.GetMin();
			++Created;
		}

		for (const FKSphylElem& Elem : BodySetup.AggGeom.SphylElems)
		{
			const FVector AxisOffset = Elem.Rotation.RotateVector(FVector(0, 0, Elem.Length * 0.5));
			FBox3DBakedShape& Shape = OutShapes.AddDefaulted_GetRef();
			Shape.Kind = EBox3DBakedShapeKind::Capsule;
			Shape.CenterA = FVector3f((FVector(Elem.Center) - AxisOffset) * Scale);
			Shape.CenterB = FVector3f((FVector(Elem.Center) + AxisOffset) * Scale);
			Shape.Radius = Elem.Radius * FMath::Min(AbsScale.X, AbsScale.Y);
			++Created;
		}

		for (const FKBoxElem& Elem : BodySetup.AggGeom.BoxElems)
		{
			// Eight scaled corners; b3CreateHull at load rebuilds the box hull
			// (equivalent to the runtime path's b3MakeScaledBoxHull).
			const FTransform ElemTransform(Elem.Rotation.Quaternion(), FVector(Elem.Center));
			FBox3DBakedShape& Shape = OutShapes.AddDefaulted_GetRef();
			Shape.Kind = EBox3DBakedShapeKind::Hull;
			Shape.Points.Reserve(8);
			for (int32 Corner = 0; Corner < 8; ++Corner)
			{
				const FVector Local(
					((Corner & 1) ? 0.5 : -0.5) * Elem.X,
					((Corner & 2) ? 0.5 : -0.5) * Elem.Y,
					((Corner & 4) ? 0.5 : -0.5) * Elem.Z);
				Shape.Points.Add(FVector3f(ElemTransform.TransformPosition(Local) * Scale));
			}
			++Created;
		}

		for (const FKConvexElem& Elem : BodySetup.AggGeom.ConvexElems)
		{
			if (Elem.VertexData.Num() < 4)
			{
				continue;
			}
			const FTransform ElemTransform = Elem.GetTransform();
			FBox3DBakedShape& Shape = OutShapes.AddDefaulted_GetRef();
			Shape.Kind = EBox3DBakedShapeKind::Hull;
			Shape.Points.Reserve(Elem.VertexData.Num());
			for (const FVector& Vertex : Elem.VertexData)
			{
				Shape.Points.Add(FVector3f(ElemTransform.TransformPosition(Vertex) * Scale));
			}
			++Created;
		}

		return Created;
	}

	/// One baked body per world transform (N for ISM instances).
	void BakeComponent(UStaticMeshComponent& Component, UBox3DCollisionData& Data)
	{
		UStaticMesh* Mesh = Component.GetStaticMesh();
		if (Mesh == nullptr)
		{
			return;
		}

		TArray<FTransform> Transforms;
		if (const UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(&Component))
		{
			const int32 InstanceCount = Ism->GetInstanceCount();
			Transforms.Reserve(InstanceCount);
			for (int32 Index = 0; Index < InstanceCount; ++Index)
			{
				FTransform InstanceToWorld;
				if (Ism->GetInstanceTransform(Index, InstanceToWorld, /*bWorldSpace*/ true))
				{
					Transforms.Add(InstanceToWorld);
				}
			}
		}
		else
		{
			Transforms.Add(Component.GetComponentTransform());
		}

		const bool bPreferSimple =
			GetDefault<UBox3DSettings>()->MirrorGeometry == EBox3DMirrorGeometry::PreferSimpleCollision;

		for (const FTransform& InstanceToWorld : Transforms)
		{
			FBox3DBakedBody Body;
			Body.WorldTransform = InstanceToWorld;
			Body.ActorKey = Component.GetPathName(Component.GetComponentLevel());

			const FVector Scale = InstanceToWorld.GetScale3D();
			int32 Created = 0;

			if (bPreferSimple)
			{
				if (const UBodySetup* BodySetup = Mesh->GetBodySetup();
					BodySetup && BodySetup->AggGeom.GetElementCount() > 0)
				{
					Created = ExtractBodySetupShapes(*BodySetup, Scale, Body.Shapes);
				}
			}

			if (Created == 0)
			{
				FBox3DBakedShape MeshShape;
				if (ExtractMeshShape(*Mesh, Scale, MeshShape))
				{
					Body.Shapes.Add(MoveTemp(MeshShape));
					Data.bContainsTriMesh = true;
					Created = 1;
				}
			}

			if (Created > 0)
			{
				Data.Bodies.Add(MoveTemp(Body));
			}
		}
	}
}

bool UBox3DBakeCommandlet::BakeMap(const FString& MapPackageName)
{
	if (!FPackageName::DoesPackageExist(MapPackageName))
	{
		UE_LOG(LogBox3D, Error, TEXT("Box3DBake: map %s does not exist"), *MapPackageName);
		return false;
	}

	UWorld* World = UEditorLoadingAndSavingUtils::LoadMap(MapPackageName);
	if (World == nullptr)
	{
		UE_LOG(LogBox3D, Error, TEXT("Box3DBake: failed to load %s"), *MapPackageName);
		return false;
	}

	if (World->GetWorldPartition() != nullptr)
	{
		UE_LOG(LogBox3D, Warning,
			TEXT("Box3DBake: %s uses World Partition — only editor-loaded cells are baked. ")
			TEXT("Prefer the runtime static mirror for streamed WP content."), *MapPackageName);
	}

	const FString AssetPackageName = UBox3DCollisionData::DeriveAssetPackageName(MapPackageName);
	UPackage* Package = CreatePackage(*AssetPackageName);
	const FString AssetName = FPackageName::GetShortName(AssetPackageName);

	UBox3DCollisionData* Data = NewObject<UBox3DCollisionData>(
		Package, *AssetName, RF_Public | RF_Standalone);
	Data->SourceLevel = MapPackageName;
	const b3Version Version = b3GetVersion();
	Data->Box3DVersion = FString::Printf(TEXT("%d.%d.%d"), Version.major, Version.minor, Version.revision);
	Data->BakeTime = FDateTime::UtcNow();
	Data->SourceFingerprint = UBox3DCollisionData::ComputeSourceFingerprint(MapPackageName);

	int32 ComponentCount = 0;
	int32 SkippedCount = 0;
	for (ULevel* Level : World->GetLevels())
	{
		for (AActor* Actor : Level->Actors)
		{
			if (Actor == nullptr)
			{
				continue;
			}
			Actor->ForEachComponent<UStaticMeshComponent>(/*bIncludeFromChildActors*/ false,
				[&](UStaticMeshComponent* Component)
				{
					// Same prefilter + filter as FBox3DStaticSceneMirror::MirrorLevel,
					// so a baked world collides identically to a mirrored one.
					if (Component->GetStaticMesh() == nullptr ||
						Component->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
					{
						return;
					}
					if (!FBox3DStaticSceneMirror::ShouldMirror(*Component))
					{
						++SkippedCount;
						return;
					}
					BakeComponent(*Component, *Data);
					++ComponentCount;
				});
		}
	}

	Package->MarkPackageDirty();
	FAssetRegistryModule::AssetCreated(Data);

	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	const FString Filename = FPackageName::LongPackageNameToFilename(
		AssetPackageName, FPackageName::GetAssetPackageExtension());
	if (!UPackage::SavePackage(Package, Data, *Filename, SaveArgs))
	{
		UE_LOG(LogBox3D, Error, TEXT("Box3DBake: failed to save %s"), *Filename);
		return false;
	}

	UE_LOG(LogBox3D, Display,
		TEXT("Box3DBake: %s — %d components baked into %d bodies (%d skipped) -> %s"),
		*MapPackageName, ComponentCount, Data->Bodies.Num(), SkippedCount, *AssetPackageName);
	return true;
}

int32 UBox3DBakeCommandlet::Main(const FString& Params)
{
	TArray<FString> Tokens;
	TArray<FString> Switches;
	ParseCommandLine(*Params, Tokens, Switches);

	// Maps come from -Map= (comma-separated) and/or bare tokens.
	TArray<FString> Maps;
	FString MapValue;
	if (FParse::Value(*Params, TEXT("Map="), MapValue))
	{
		MapValue.ParseIntoArray(Maps, TEXT(","));
	}
	for (const FString& Token : Tokens)
	{
		if (FPackageName::IsValidLongPackageName(Token))
		{
			Maps.AddUnique(Token);
		}
	}

	if (Maps.IsEmpty())
	{
		UE_LOG(LogBox3D, Error,
			TEXT("Box3DBake: no maps given. Use -Map=/Game/Maps/Foo[,/Game/Maps/Bar]."));
		return 1;
	}

	int32 Failures = 0;
	for (const FString& Map : Maps)
	{
		if (!BakeMap(Map))
		{
			++Failures;
		}
	}

	UE_LOG(LogBox3D, Display, TEXT("Box3DBake: %d map(s) baked, %d failure(s)"),
		Maps.Num() - Failures, Failures);
	return Failures == 0 ? 0 : 1;
}
