// Console hook for eyeballing the fracture core:
//   box3d.FractureDebug [CellCount=12] [Seed=42] [RadialBias=0.5] [MinVolume=0]
// Fractures a 1 m cube resting on the surface under the crosshair (impact
// point = the crosshair hit) and debug-draws the cell wireframes for 20 s,
// one color per fragment.

#include "Box3DFracture.h"
#include "Box3DRuntime.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"

#if !UE_BUILD_SHIPPING

static FAutoConsoleCommandWithWorldAndArgs GBox3DFractureDebugCommand(
	TEXT("box3d.FractureDebug"),
	TEXT("Fracture a 1 m cube at the crosshair and debug-draw the cells. Usage: box3d.FractureDebug [CellCount=12] [Seed=42] [RadialBias=0.5] [MinVolume=0]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		const APlayerController* PC = World != nullptr ? World->GetFirstPlayerController() : nullptr;
		if (PC == nullptr)
		{
			return;
		}
		FVector ViewLocation;
		FRotator ViewRotation;
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);

		using namespace Box3D::Fracture;
		constexpr double HalfSize = 50.0;

		// Rest the cube on the hit surface; the hit point doubles as the
		// impact to bias toward. No hit: float it 4 m ahead, impact at center.
		FHitResult Hit;
		const FVector TraceEnd = ViewLocation + ViewRotation.Vector() * 5000.0;
		FVector ImpactPoint;
		FVector Center;
		if (World->LineTraceSingleByChannel(Hit, ViewLocation, TraceEnd, ECC_Visibility))
		{
			ImpactPoint = Hit.ImpactPoint;
			Center = Hit.ImpactPoint + Hit.ImpactNormal * HalfSize;
		}
		else
		{
			Center = ViewLocation + ViewRotation.Vector() * 400.0;
			ImpactPoint = Center;
		}

		FFractureParams Params;
		Params.CellCount = FMath::Clamp(Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 12, 1, 256);
		Params.Seed = Args.Num() > 1 ? FCString::Atoi(*Args[1]) : 42;
		Params.RadialBias = FMath::Clamp(Args.Num() > 2 ? FCString::Atod(*Args[2]) : 0.5, 0.0, 1.0);
		Params.MinFragmentVolume = FMath::Max(Args.Num() > 3 ? FCString::Atod(*Args[3]) : 0.0, 0.0);
		Params.ImpactPoint = ImpactPoint;
		Params.ImpactRadius = HalfSize;

		TArray<FBox3DFragmentData> Fragments;
		if (!Fracture(MakeBoxProxy(Center, FVector(HalfSize)), Params, Fragments))
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.FractureDebug: fracture failed"));
			return;
		}

		constexpr float LifeTime = 20.0f;
		for (int32 Index = 0; Index < Fragments.Num(); ++Index)
		{
			const FBox3DFragmentData& Fragment = Fragments[Index];
			// Golden-angle hue walk keeps adjacent fragment colors distinct.
			const FColor Color = FLinearColor::MakeFromHSV8(uint8((Index * 41) & 0xFF), 220, 255).ToFColor(false);
			for (const FBox3DFragmentFace& Face : Fragment.Faces)
			{
				const int32 Count = Face.VertexIndices.Num();
				for (int32 Edge = 0; Edge < Count; ++Edge)
				{
					DrawDebugLine(World,
						Fragment.Vertices[Face.VertexIndices[Edge]],
						Fragment.Vertices[Face.VertexIndices[(Edge + 1) % Count]],
						Color, false, LifeTime, 0, 0.75f);
				}
			}
			DrawDebugPoint(World, Fragment.Centroid, 8.0f, Color, false, LifeTime);
		}

		UE_LOG(LogBox3D, Log,
			TEXT("box3d.FractureDebug: %d fragments (seed %d, bias %.2f, min volume %.0f), layout hash 0x%08X at %s"),
			Fragments.Num(), Params.Seed, Params.RadialBias, Params.MinFragmentVolume,
			FractureLayoutHash(Fragments), *Center.ToCompactString());
	}));

#endif // !UE_BUILD_SHIPPING
