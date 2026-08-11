// Console hooks for eyeballing the fracture system:
//   box3d.DestructionStress [Count=16]
// Spawns N destructible cubes ahead of the player and queues one fracture
// impact per cube, spread over one second — the D3 budget eyeball: watch
// `stat box3d` while the fracture budget spreads the burst across ticks and
// the fragment pool evicts the oldest rubble.
//   box3d.FractureDebug [CellCount=12] [Seed=42] [RadialBias=0.5] [MinVolume=0]
// Fractures a 1 m cube resting on the surface under the crosshair (impact
// point = the crosshair hit) and debug-draws the cell wireframes for 20 s,
// one color per fragment.
//   box3d.Fracture [CellCount=12] [Seed=42] [Toughness=50]
// Fractures the static mesh under the crosshair for real via
// Box3D::FractureMesh — welded rubble that scatters when welds snap.
//   box3d.SpawnStructure [CellCount=16] [Seed=42] [Toughness=50]
// Spawns an anchored wall + cantilever beam ahead of the player and fractures
// both structurally (D4): chunks stay static until connectivity strands them.
// Needs mirrored/baked static ground for anchoring. Knock chunks out with
// box3d.DestroyChunk.
//   box3d.DestroyChunk
// Destroys the fractured chunk under the crosshair (box3d ray cast) — the
// structural island promotion eyeball: shoot the beam, watch the overhang fall.

#include "Box3DConversion.h"
#include "Box3DDestructibleComponent.h"
#include "Box3DFracture.h"
#include "Box3DFracturedActor.h"
#include "Box3DRuntime.h"
#include "Box3DSettings.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "TimerManager.h"
#include "box3d/box3d.h"

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

static FAutoConsoleCommandWithWorldAndArgs GBox3DFractureCommand(
	TEXT("box3d.Fracture"),
	TEXT("Fracture the static mesh under the crosshair into welded physics fragments. Usage: box3d.Fracture [CellCount=12] [Seed=42] [Toughness=50]"),
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

		FHitResult Hit;
		const FVector TraceEnd = ViewLocation + ViewRotation.Vector() * 10000.0;
		if (!World->LineTraceSingleByChannel(Hit, ViewLocation, TraceEnd, ECC_Visibility))
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.Fracture: nothing under the crosshair"));
			return;
		}
		UStaticMeshComponent* Target = Cast<UStaticMeshComponent>(Hit.GetComponent());
		if (Target == nullptr || Target->GetStaticMesh() == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.Fracture: hit %s, not a static mesh component"),
				*GetNameSafe(Hit.GetComponent()));
			return;
		}

		FBox3DFractureMeshParams Params;
		Params.Fracture.CellCount = FMath::Clamp(Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 12, 1, 256);
		Params.Fracture.Seed = Args.Num() > 1 ? FCString::Atoi(*Args[1]) : 42;
		Params.MaterialToughness = Args.Num() > 2 ? FCString::Atof(*Args[2]) : 50.0f;
		Params.Fracture.ImpactPoint = Hit.ImpactPoint;
		Params.Fracture.ImpactRadius = FMath::Max(Target->Bounds.SphereRadius * 0.3, 10.0);
		Params.Fracture.RadialBias = 0.5;

		if (ABox3DFracturedActor* Actor = Box3D::FractureMesh(Target, Params))
		{
			UE_LOG(LogBox3D, Log, TEXT("box3d.Fracture: %s -> %d fragments, %d welds (seed %d, toughness %.1f)"),
				*GetNameSafe(Target->GetStaticMesh()), Actor->GetFragmentCount(), Actor->GetLiveWeldCount(),
				Params.Fracture.Seed, Params.MaterialToughness);
		}
	}));

static FAutoConsoleCommandWithWorldAndArgs GBox3DDestructionStressCommand(
	TEXT("box3d.DestructionStress"),
	TEXT("Spawn N destructible cubes ahead of the player and queue one fracture impact per cube over one second (budget + pool eyeball). Usage: box3d.DestructionStress [Count=16]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		const APlayerController* PC = World != nullptr ? World->GetFirstPlayerController() : nullptr;
		UBox3DWorldSubsystem* Subsystem = World != nullptr ? World->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
		UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (PC == nullptr || Subsystem == nullptr || Cube == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.DestructionStress: needs a player, a Box3D world, and engine content"));
			return;
		}

		const int32 Count = FMath::Clamp(Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 16, 1, 256);

		FVector ViewLocation;
		FRotator ViewRotation;
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);
		const FVector Forward = FVector(ViewRotation.Vector().X, ViewRotation.Vector().Y, 0.0).GetSafeNormal();
		const FVector Right = FVector::CrossProduct(FVector::UpVector, Forward);

		// Grid of 1 m cubes 6 m ahead, 1.5 m apart, resting at the player's feet Z.
		const int32 Columns = FMath::CeilToInt(FMath::Sqrt(float(Count)));
		const FVector Base = ViewLocation + Forward * 600.0 - FVector(0, 0, 100)
			- Right * (Columns - 1) * 75.0;

		for (int32 Index = 0; Index < Count; ++Index)
		{
			const FVector Center = Base + Right * (Index % Columns) * 150.0 + Forward * (Index / Columns) * 150.0
				+ FVector(0, 0, 50);

			AActor* Actor = World->SpawnActor<AActor>();
			UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(Actor, TEXT("StressCube"));
			Mesh->SetMobility(EComponentMobility::Movable);
			Mesh->SetStaticMesh(Cube);
			Mesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
			Actor->SetRootComponent(Mesh);
			Mesh->SetWorldLocation(Center);
			Mesh->RegisterComponent();

			UBox3DDestructibleComponent* Destructible =
				NewObject<UBox3DDestructibleComponent>(Actor, TEXT("StressDestructible"));
			Destructible->RegisterComponent();

			// One impact per cube, spread across one second — the queue and the
			// per-tick budget take it from here.
			FTimerHandle Handle;
			World->GetTimerManager().SetTimer(Handle,
				FTimerDelegate::CreateWeakLambda(Destructible, [Subsystem, Destructible, Center]()
				{
					Subsystem->QueueDestructibleImpact(Destructible, Center + FVector(0, 0, 50), 5000.0f);
				}),
				(Index + 1) / float(Count), false);
		}

		UE_LOG(LogBox3D, Log,
			TEXT("box3d.DestructionStress: %d destructible cubes queued over 1 s (budget %.2f ms/tick, pool cap %d)"),
			Count, GetDefault<UBox3DSettings>()->FractureTimeBudgetMs, GetDefault<UBox3DSettings>()->MaxLiveFragments);
	}));

static FAutoConsoleCommandWithWorldAndArgs GBox3DSpawnStructureCommand(
	TEXT("box3d.SpawnStructure"),
	TEXT("Spawn an anchored wall + cantilever beam ahead of the player, structurally fractured (D4). Usage: box3d.SpawnStructure [CellCount=16] [Seed=42] [Toughness=50]"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		const APlayerController* PC = World != nullptr ? World->GetFirstPlayerController() : nullptr;
		UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (PC == nullptr || Cube == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.SpawnStructure: needs a player and engine content"));
			return;
		}

		FVector ViewLocation;
		FRotator ViewRotation;
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);
		const FVector Forward = FVector(ViewRotation.Vector().X, ViewRotation.Vector().Y, 0.0).GetSafeNormal();
		const FVector Right = FVector::CrossProduct(FVector::UpVector, Forward);

		// Ground the assembly on whatever is under the spawn spot; fall back to
		// the player's feet if nothing traces.
		const FVector Ahead = ViewLocation + Forward * 500.0;
		FHitResult Hit;
		double GroundZ = ViewLocation.Z - 100.0;
		if (World->LineTraceSingleByChannel(Hit, Ahead + FVector(0, 0, 100), Ahead - FVector(0, 0, 3000),
				ECC_Visibility))
		{
			GroundZ = Hit.ImpactPoint.Z;
		}

		FBox3DFractureMeshParams Params;
		Params.Fracture.CellCount = FMath::Clamp(Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 16, 1, 256);
		Params.Fracture.Seed = Args.Num() > 1 ? FCString::Atoi(*Args[1]) : 42;
		Params.MaterialToughness = Args.Num() > 2 ? FCString::Atof(*Args[2]) : 50.0f;
		Params.bStructural = true;

		// Wall facing the player (long axis along Right), then a beam resting on
		// its top edge, cantilevered toward the player — the beam anchors on the
		// wall's static chunks, the overhang hangs off its bonds.
		const FQuat WallRot = FRotationMatrix::MakeFromXZ(Right, FVector::UpVector).ToQuat();
		const auto SpawnAndFracture = [&](const FTransform& Transform) -> ABox3DFracturedActor*
		{
			AActor* Actor = World->SpawnActor<AActor>();
			UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(Actor, TEXT("StructureMesh"));
			Mesh->SetMobility(EComponentMobility::Movable);
			Mesh->SetStaticMesh(Cube);
			Mesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
			Actor->SetRootComponent(Mesh);
			Mesh->SetWorldTransform(Transform);
			Mesh->RegisterComponent();
			Params.Fracture.ImpactPoint = Transform.GetLocation();
			return Box3D::FractureMesh(Mesh, Params);
		};

		// 300 x 40 x 200 cm wall standing on the ground.
		ABox3DFracturedActor* Wall = SpawnAndFracture(FTransform(WallRot,
			FVector(Ahead.X, Ahead.Y, GroundZ + 100.0), FVector(3.0, 0.4, 2.0)));
		// 300 x 40 x 40 cm beam across the wall top, cantilevered toward the player.
		ABox3DFracturedActor* Beam = SpawnAndFracture(FTransform(
			FRotationMatrix::MakeFromXZ(Forward, FVector::UpVector).ToQuat(),
			FVector(Ahead.X, Ahead.Y, GroundZ + 220.0) - Forward * 100.0, FVector(3.0, 0.4, 0.4)));

		if (Wall == nullptr || Beam == nullptr)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.SpawnStructure: fracture failed"));
			return;
		}
		if (!Wall->IsStructureActive())
		{
			UE_LOG(LogBox3D, Warning,
				TEXT("box3d.SpawnStructure: wall found no anchors — is the ground mirrored or baked "
					 "(bMirrorStaticGeometry / bUseBakedStaticCollision)?"));
		}
		UE_LOG(LogBox3D, Log,
			TEXT("box3d.SpawnStructure: wall %d chunks (structural %d), beam %d chunks (structural %d). "
				 "Knock chunks out with box3d.DestroyChunk."),
			Wall->GetFragmentCount(), Wall->IsStructureActive() ? 1 : 0,
			Beam->GetFragmentCount(), Beam->IsStructureActive() ? 1 : 0);
	}));

static FAutoConsoleCommandWithWorldAndArgs GBox3DDestroyChunkCommand(
	TEXT("box3d.DestroyChunk"),
	TEXT("Destroy the fractured chunk under the crosshair (box3d ray cast). Usage: box3d.DestroyChunk"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		const APlayerController* PC = World != nullptr ? World->GetFirstPlayerController() : nullptr;
		UBox3DWorldSubsystem* Subsystem = World != nullptr ? World->GetSubsystem<UBox3DWorldSubsystem>() : nullptr;
		if (PC == nullptr || Subsystem == nullptr || !b3World_IsValid(Subsystem->GetBox3DWorldId()))
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.DestroyChunk: needs a player and a Box3D world"));
			return;
		}

		FVector ViewLocation;
		FRotator ViewRotation;
		PC->GetPlayerViewPoint(ViewLocation, ViewRotation);

		b3QueryFilter Filter = b3DefaultQueryFilter();
		Filter.categoryBits = UINT64_MAX;
		Filter.maskBits = UINT64_MAX;
		const b3RayResult Result = b3World_CastRayClosest(Subsystem->GetBox3DWorldId(),
			Box3D::ToB3Pos(ViewLocation), Box3D::ToB3(ViewRotation.Vector() * 10000.0), Filter);
		if (!Result.hit)
		{
			UE_LOG(LogBox3D, Warning, TEXT("box3d.DestroyChunk: nothing under the crosshair"));
			return;
		}

		const uint64 HitBody = b3StoreBodyId(b3Shape_GetBody(Result.shapeId));
		for (TActorIterator<ABox3DFracturedActor> It(World); It; ++It)
		{
			for (int32 Index = 0; Index < It->GetFragmentCount(); ++Index)
			{
				const b3BodyId Body = It->GetFragmentBody(Index);
				if (b3Body_IsValid(Body) && b3StoreBodyId(Body) == HitBody)
				{
					It->DestroyFragment(Index);
					UE_LOG(LogBox3D, Log,
						TEXT("box3d.DestroyChunk: %s chunk %d destroyed (%d promotions queued)"),
						*GetNameSafe(*It), Index, It->GetPendingPromotionCount());
					return;
				}
			}
		}
		UE_LOG(LogBox3D, Warning, TEXT("box3d.DestroyChunk: hit body is not a fractured chunk"));
	}));

#endif // !UE_BUILD_SHIPPING
