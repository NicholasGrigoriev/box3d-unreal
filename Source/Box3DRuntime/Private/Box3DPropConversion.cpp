#include "Box3DPropConversion.h"

#include "Box3DPropActor.h"
#include "Box3DRuntime.h"
#include "Box3DStaticSceneMirror.h"
#include "Box3DWorldSubsystem.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"

namespace
{
	FBox3DStaticSceneMirror* GetMirror(const UWorld* World)
	{
		const UBox3DWorldSubsystem* Subsystem = World ? World->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
		return Subsystem ? Subsystem->GetStaticMirror() : nullptr;
	}

	ABox3DPropActor* SpawnProp(UWorld* World, UStaticMesh* Mesh, const UStaticMeshComponent* MaterialSource,
		const FTransform& Transform)
	{
		// Deferred spawn: the mesh must be assigned before BeginPlay cooks the shape.
		ABox3DPropActor* Prop = World->SpawnActorDeferred<ABox3DPropActor>(ABox3DPropActor::StaticClass(), Transform);
		if (Prop == nullptr)
		{
			return nullptr;
		}

		Prop->SetStaticMesh(Mesh);
		if (MaterialSource != nullptr)
		{
			for (int32 Index = 0; Index < MaterialSource->GetNumMaterials(); ++Index)
			{
				Prop->GetMesh()->SetMaterial(Index, MaterialSource->GetMaterial(Index));
			}
		}
		Prop->FinishSpawning(Transform);
		return Prop;
	}
}

namespace Box3D
{
	ABox3DPropActor* ConvertToProp(AStaticMeshActor* Actor)
	{
		UStaticMeshComponent* MeshComponent = Actor ? Actor->GetStaticMeshComponent() : nullptr;
		UStaticMesh* Mesh = MeshComponent ? MeshComponent->GetStaticMesh() : nullptr;
		if (Mesh == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("ConvertToProp: %s has no static mesh"), *GetNameSafe(Actor));
			return nullptr;
		}

		UWorld* World = Actor->GetWorld();
		if (FBox3DStaticSceneMirror* Mirror = GetMirror(World))
		{
			Mirror->RemoveComponent(MeshComponent);
		}

		ABox3DPropActor* Prop = SpawnProp(World, Mesh, MeshComponent, MeshComponent->GetComponentTransform());
		if (Prop == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("ConvertToProp: spawn failed for %s"), *GetNameSafe(Actor));
			return nullptr;
		}

		UE_LOG(LogBox3D, Log, TEXT("ConvertToProp: %s (%s) -> %s"),
			*GetNameSafe(Actor), *GetNameSafe(Mesh), *GetNameSafe(Prop));
		Actor->Destroy();
		return Prop;
	}

	ABox3DPropActor* ConvertInstanceToProp(UInstancedStaticMeshComponent* Ism, int32 InstanceIndex)
	{
		UStaticMesh* Mesh = Ism ? Ism->GetStaticMesh() : nullptr;
		FTransform InstanceToWorld;
		if (Mesh == nullptr || !Ism->GetInstanceTransform(InstanceIndex, InstanceToWorld, /*bWorldSpace*/ true))
		{
			UE_LOG(LogBox3D, Warning, TEXT("ConvertInstanceToProp: invalid instance %d on %s"),
				InstanceIndex, *GetNameSafe(Ism));
			return nullptr;
		}

		UWorld* World = Ism->GetWorld();
		FBox3DStaticSceneMirror* Mirror = GetMirror(World);
		if (Mirror != nullptr)
		{
			Mirror->RemoveComponent(Ism);
		}
		// Removal reindexes the remaining instances, so the component re-mirrors whole.
		Ism->RemoveInstance(InstanceIndex);
		if (Mirror != nullptr)
		{
			Mirror->RemirrorComponent(Ism);
		}

		ABox3DPropActor* Prop = SpawnProp(World, Mesh, Ism, InstanceToWorld);
		UE_LOG(LogBox3D, Log, TEXT("ConvertInstanceToProp: %s[%d] (%s) -> %s"),
			*GetNameSafe(Ism), InstanceIndex, *GetNameSafe(Mesh), *GetNameSafe(Prop));
		return Prop;
	}
}
