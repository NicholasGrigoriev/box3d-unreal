#include "Box3DDentMapComponent.h"

#include "CanvasItem.h"
#include "CanvasTypes.h"
#include "Components/MeshComponent.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "RHIGlobals.h"

const FName UBox3DDentMapComponent::DentMapParameterName(TEXT("Box3D_DentMap"));
const FName UBox3DDentMapComponent::DentNormalStrengthParameterName(TEXT("Box3D_DentNormalStrength"));
const FName UBox3DDentMapComponent::DentMapTexelSizeParameterName(TEXT("Box3D_DentMapTexelSize"));

namespace
{
	constexpr int32 BrushResolution = 64;

	UTexture2D* MakeRadialBrush(UObject* Outer)
	{
		TArray64<uint8> Pixels;
		Pixels.SetNumUninitialized(BrushResolution * BrushResolution * 4);
		for (int32 Y = 0; Y < BrushResolution; ++Y)
		{
			for (int32 X = 0; X < BrushResolution; ++X)
			{
				const double U = (static_cast<double>(X) + 0.5) / BrushResolution * 2.0 - 1.0;
				const double V = (static_cast<double>(Y) + 0.5) / BrushResolution * 2.0 - 1.0;
				const double Falloff = FMath::Square(FMath::Clamp(1.0 - FMath::Sqrt(U * U + V * V), 0.0, 1.0));
				const uint8 Value = static_cast<uint8>(FMath::RoundToInt(Falloff * 255.0));
				const int64 Pixel = (static_cast<int64>(Y) * BrushResolution + X) * 4;
				Pixels[Pixel + 0] = Value;
				Pixels[Pixel + 1] = Value;
				Pixels[Pixel + 2] = Value;
				Pixels[Pixel + 3] = Value;
			}
		}

		UTexture2D* Brush = UTexture2D::CreateTransient(BrushResolution, BrushResolution,
			PF_B8G8R8A8, NAME_None, TConstArrayView64<uint8>(Pixels.GetData(), Pixels.Num()));
		if (Brush != nullptr)
		{
			Brush->Rename(nullptr, Outer);
			Brush->SRGB = false;
			Brush->Filter = TF_Bilinear;
			Brush->AddressX = TA_Clamp;
			Brush->AddressY = TA_Clamp;
			Brush->UpdateResource();
		}
		return Brush;
	}
}

UBox3DDentMapComponent::UBox3DDentMapComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

bool UBox3DDentMapComponent::InitializeDentMap(UMeshComponent* TargetMesh)
{
	if (TargetMesh == nullptr || TargetMesh->GetWorld() == nullptr)
	{
		return false;
	}

	BoundMesh = TargetMesh;
	if (!AllocateResources())
	{
		BoundMesh = nullptr;
		return false;
	}
	RefreshMaterialBindings();
	return true;
}

bool UBox3DDentMapComponent::AllocateResources()
{
	DentMapResolution = FMath::Clamp(DentMapResolution, 32, 2048);
	RenderTargets.Reset(2);
	for (int32 Index = 0; Index < 2; ++Index)
	{
		UTextureRenderTarget2D* Target = NewObject<UTextureRenderTarget2D>(this);
		if (Target == nullptr)
		{
			RenderTargets.Reset();
			return false;
		}
		Target->RenderTargetFormat = RTF_R8;
		Target->ClearColor = FLinearColor::Black;
		Target->bForceLinearGamma = true;
		Target->AddressX = TA_Clamp;
		Target->AddressY = TA_Clamp;
		Target->Filter = TF_Bilinear;
		Target->InitAutoFormat(DentMapResolution, DentMapResolution);
		Target->UpdateResourceImmediate(true);
		RenderTargets.Add(Target);
	}

	StampBrush = MakeRadialBrush(this);
	if (StampBrush == nullptr)
	{
		RenderTargets.Reset();
		return false;
	}
	ClearDentMap();
	return true;
}

void UBox3DDentMapComponent::ClearTarget(UTextureRenderTarget2D* Target) const
{
	if (Target == nullptr || Target->GameThread_GetRenderTargetResource() == nullptr)
	{
		return;
	}
	FCanvas Canvas(Target->GameThread_GetRenderTargetResource(), nullptr, GetWorld(),
		GetWorld()->GetFeatureLevel(), FCanvas::CDM_ImmediateDrawing);
	Canvas.Clear(FLinearColor::Black);
	Canvas.Flush_GameThread(true);
}

bool UBox3DDentMapComponent::StampDentUV(FVector2D ImpactUV, float RadiusUV, float Strength)
{
	if (RenderTargets.Num() != 2 || StampBrush == nullptr || BoundMesh == nullptr
		|| ImpactUV.ContainsNaN() || !FMath::IsFinite(RadiusUV) || !FMath::IsFinite(Strength)
		|| RadiusUV <= UE_SMALL_NUMBER || Strength <= 0.0f)
	{
		return false;
	}

	UTextureRenderTarget2D* Source = RenderTargets[ActiveTargetIndex];
	const int32 DestinationIndex = 1 - ActiveTargetIndex;
	UTextureRenderTarget2D* Destination = RenderTargets[DestinationIndex];
	if (Source == nullptr || Destination == nullptr)
	{
		return false;
	}

	// Commandlets exercise the deterministic helper state under NullRHI; there
	// is intentionally no pixel output there, matching the plan's eyeball-only
	// validation for this cosmetic tier.
	if (GUsingNullRHI)
	{
		ActiveTargetIndex = DestinationIndex;
		++StampCount;
		PublishActiveMap();
		return true;
	}
	if (Source->GetResource() == nullptr || StampBrush->GetResource() == nullptr
		|| Destination->GameThread_GetRenderTargetResource() == nullptr)
	{
		return false;
	}

	FCanvas Canvas(Destination->GameThread_GetRenderTargetResource(), nullptr, GetWorld(),
		GetWorld()->GetFeatureLevel(), FCanvas::CDM_ImmediateDrawing);
	Canvas.Clear(FLinearColor::Black);

	FCanvasTileItem Copy(FVector2D::ZeroVector, Source->GetResource(),
		FVector2D(DentMapResolution), FLinearColor::White);
	Copy.BlendMode = SE_BLEND_Opaque;
	Canvas.DrawItem(Copy);

	const float DiameterPixels = RadiusUV * 2.0f * DentMapResolution;
	const FVector2D StampSize(DiameterPixels);
	const FVector2D StampPosition = ImpactUV * DentMapResolution - StampSize * 0.5;
	const float ClampedStrength = FMath::Clamp(Strength, 0.0f, 1.0f);
	FCanvasTileItem Stamp(StampPosition, StampBrush->GetResource(), StampSize,
		FLinearColor(ClampedStrength, ClampedStrength, ClampedStrength, ClampedStrength));
	Stamp.BlendMode = SE_BLEND_Additive;
	Canvas.DrawItem(Stamp);
	Canvas.Flush_GameThread(true);

	ActiveTargetIndex = DestinationIndex;
	++StampCount;
	PublishActiveMap();
	return true;
}

void UBox3DDentMapComponent::ClearDentMap()
{
	for (UTextureRenderTarget2D* Target : RenderTargets)
	{
		ClearTarget(Target);
	}
	ActiveTargetIndex = 0;
	StampCount = 0;
	PublishActiveMap();
}

void UBox3DDentMapComponent::RefreshMaterialBindings()
{
	MaterialInstances.Reset();
	if (BoundMesh == nullptr)
	{
		return;
	}

	for (int32 MaterialIndex = 0; MaterialIndex < BoundMesh->GetNumMaterials(); ++MaterialIndex)
	{
		if (UMaterialInstanceDynamic* Instance = BoundMesh->CreateDynamicMaterialInstance(MaterialIndex))
		{
			MaterialInstances.Add(Instance);
		}
	}
	PublishActiveMap();
}

void UBox3DDentMapComponent::PublishActiveMap()
{
	UTextureRenderTarget2D* ActiveMap = GetDentMap();
	if (ActiveMap == nullptr)
	{
		return;
	}
	const float ReciprocalSize = 1.0f / FMath::Max(DentMapResolution, 1);
	const FLinearColor TexelSize(ReciprocalSize, ReciprocalSize,
		static_cast<float>(DentMapResolution), static_cast<float>(DentMapResolution));
	for (UMaterialInstanceDynamic* Instance : MaterialInstances)
	{
		if (Instance != nullptr)
		{
			Instance->SetTextureParameterValue(DentMapParameterName, ActiveMap);
			Instance->SetScalarParameterValue(DentNormalStrengthParameterName, DentNormalStrength);
			Instance->SetVectorParameterValue(DentMapTexelSizeParameterName, TexelSize);
		}
	}
}

UTextureRenderTarget2D* UBox3DDentMapComponent::GetDentMap() const
{
	return RenderTargets.IsValidIndex(ActiveTargetIndex) ? RenderTargets[ActiveTargetIndex] : nullptr;
}

UTextureRenderTarget2D* UBox3DDentMapComponent::GetInactiveDentMap() const
{
	const int32 InactiveIndex = 1 - ActiveTargetIndex;
	return RenderTargets.IsValidIndex(InactiveIndex) ? RenderTargets[InactiveIndex] : nullptr;
}
