// Visual path for the render-only D6 dent-map tier:
//   box3d.DentTest [Resolution=256] [StampCount=12] [SurfaceMaterialPath]
// Spawns a thin panel ahead of the player and accumulates deterministic UV
// stamps. With no material path, an engine emissive texture material previews
// the height map directly; a project material can consume the Box3D_* normal
// reconstruction contract published by UBox3DDentMapComponent.

#include "Box3DDentMapComponent.h"

#include "Box3DRuntime.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"

#if !UE_BUILD_SHIPPING

static FAutoConsoleCommandWithWorldAndArgs GBox3DDentTestCommand(
	TEXT("box3d.DentTest"),
	TEXT("Spawn a UV dent-map panel. Usage: box3d.DentTest [Resolution=256] [StampCount=12] [SurfaceMaterialPath]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		APlayerController* PC = World != nullptr ? World->GetFirstPlayerController() : nullptr;
		if (PC == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.DentTest: no player controller"));
			return;
		}

		FVector ViewLocation;
		FRotator ViewRotation;
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AActor* Panel = World->SpawnActor<AActor>(AActor::StaticClass(),
			ViewLocation + ViewRotation.Vector() * 350.0, ViewRotation, SpawnParams);
		if (Panel == nullptr)
		{
			return;
		}

		UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(Panel, TEXT("DentTestPanel"));
		Panel->SetRootComponent(Mesh);
		Mesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
		Mesh->SetRelativeScale3D(FVector(0.05, 2.0, 2.0));
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		const bool bCustomMaterial = Args.Num() > 2;
		const TCHAR* MaterialPath = bCustomMaterial
			? *Args[2]
			: TEXT("/Engine/EngineMaterials/EmissiveTexturedMaterial.EmissiveTexturedMaterial");
		if (UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, MaterialPath))
		{
			Mesh->SetMaterial(0, Material);
		}
		Mesh->RegisterComponent();

		UBox3DDentMapComponent* DentMap = NewObject<UBox3DDentMapComponent>(Panel, TEXT("DentMap"));
		DentMap->DentMapResolution = FMath::Clamp(Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 256, 32, 2048);
		DentMap->RegisterComponent();
		if (!DentMap->InitializeDentMap(Mesh))
		{
			Panel->Destroy();
			UE_LOG(LogBox3D, Warning, TEXT("box3d.DentTest: render-target initialization failed"));
			return;
		}

		const int32 StampCount = FMath::Clamp(Args.Num() > 1 ? FCString::Atoi(*Args[1]) : 12, 1, 128);
		for (int32 Index = 0; Index < StampCount; ++Index)
		{
			const double Angle = Index * 2.39996322972865332; // Golden angle.
			const double Radius = 0.04 * FMath::Sqrt(static_cast<double>(Index));
			const FVector2D UV(0.5 + FMath::Cos(Angle) * Radius, 0.5 + FMath::Sin(Angle) * Radius);
			DentMap->StampDentUV(UV, 0.10f, 0.35f);
		}

		if (!bCustomMaterial)
		{
			for (UMaterialInstanceDynamic* Instance : DentMap->GetMaterialInstances())
			{
				Instance->SetTextureParameterValue(TEXT("Texture"), DentMap->GetDentMap());
			}
		}

		UE_LOG(LogBox3D, Log,
			TEXT("box3d.DentTest: %d stamps at %dx%d; %s. Material contract: %s, %s, %s"),
			DentMap->GetDentStampCount(), DentMap->DentMapResolution, DentMap->DentMapResolution,
			bCustomMaterial ? TEXT("custom surface material bound") : TEXT("height-map preview (supply a compatible material for normal perturbation)"),
			*UBox3DDentMapComponent::DentMapParameterName.ToString(),
			*UBox3DDentMapComponent::DentNormalStrengthParameterName.ToString(),
			*UBox3DDentMapComponent::DentMapTexelSizeParameterName.ToString());
	}));

#endif // !UE_BUILD_SHIPPING
