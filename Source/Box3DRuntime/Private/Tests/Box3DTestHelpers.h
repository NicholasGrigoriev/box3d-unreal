#pragma once

#include "CoreMinimal.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Box3DBodyComponent.h"
#include "Box3DSettings.h"
#include "Box3DWorldSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "Misc/AutomationTest.h"
#include "box3d/box3d.h"

/// Shared flags for all Box3D automation tests. Run with:
///   Automation RunTests Box3DUnreal
#define BOX3D_TEST_FLAGS \
	(EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | \
	 EAutomationTestFlags::CommandletContext | EAutomationTestFlags::ProductFilter)

namespace Box3DTest
{
	/// Temporarily override scheduler settings on the UBox3DSettings CDO. The
	/// subsystem reads settings once in Initialize, so construct this BEFORE the
	/// FTestWorld whose physics world should use the override.
	struct FScopedTaskSettings
	{
		int32 SavedWorkers;
		EBox3DTaskSystem SavedSystem;

		FScopedTaskSettings(int32 Workers, EBox3DTaskSystem System)
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			SavedWorkers = Settings->WorkerCount;
			SavedSystem = Settings->TaskSystem;
			Settings->WorkerCount = Workers;
			Settings->TaskSystem = System;
		}

		~FScopedTaskSettings()
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			Settings->WorkerCount = SavedWorkers;
			Settings->TaskSystem = SavedSystem;
		}
	};

	/// Temporarily enable the static scene mirror on the settings CDO. Construct
	/// BEFORE the FTestWorld (the subsystem creates the mirror in Initialize).
	/// Note FTestWorld never fires OnWorldBeginPlay: tests call MirrorLevel on the
	/// persistent level explicitly after spawning their geometry.
	struct FScopedMirrorSettings
	{
		bool bSavedMirror;
		EBox3DMirrorGeometry SavedGeometry;

		explicit FScopedMirrorSettings(EBox3DMirrorGeometry Geometry = EBox3DMirrorGeometry::TriangleMesh)
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			bSavedMirror = Settings->bMirrorStaticGeometry;
			SavedGeometry = Settings->MirrorGeometry;
			Settings->bMirrorStaticGeometry = true;
			Settings->MirrorGeometry = Geometry;
		}

		~FScopedMirrorSettings()
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			Settings->bMirrorStaticGeometry = bSavedMirror;
			Settings->MirrorGeometry = SavedGeometry;
		}
	};

	/// Temporarily override the D3 destruction budgets (fragment pool cap and
	/// per-tick fracture time budget). Both are read live on use, so the scope
	/// only needs to cover the fractures under test. 0 = unlimited for either.
	struct FScopedDestructionSettings
	{
		int32 SavedMaxLiveFragments;
		float SavedFractureTimeBudgetMs;

		FScopedDestructionSettings(int32 MaxLiveFragments, float FractureTimeBudgetMs)
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			SavedMaxLiveFragments = Settings->MaxLiveFragments;
			SavedFractureTimeBudgetMs = Settings->FractureTimeBudgetMs;
			Settings->MaxLiveFragments = MaxLiveFragments;
			Settings->FractureTimeBudgetMs = FractureTimeBudgetMs;
		}

		~FScopedDestructionSettings()
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			Settings->MaxLiveFragments = SavedMaxLiveFragments;
			Settings->FractureTimeBudgetMs = SavedFractureTimeBudgetMs;
		}
	};

	/// Temporarily override the D4 per-tick promotion budget (0 = unlimited).
	/// Read live in ProcessPromotions, so the scope only needs to cover the
	/// promotion pumps under test.
	struct FScopedPromotionBudget
	{
		int32 Saved;

		explicit FScopedPromotionBudget(int32 MaxPromotionsPerTick)
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			Saved = Settings->MaxPromotionsPerTick;
			Settings->MaxPromotionsPerTick = MaxPromotionsPerTick;
		}

		~FScopedPromotionBudget()
		{
			GetMutableDefault<UBox3DSettings>()->MaxPromotionsPerTick = Saved;
		}
	};

	/// Temporarily override the D5 stress work budget and coarsening threshold.
	/// Both are read live by structural actors.
	struct FScopedStressSettings
	{
		int32 SavedIterations;
		int32 SavedCoarsenThreshold;

		FScopedStressSettings(int32 IterationsPerTick, int32 CoarsenNodeThreshold)
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			SavedIterations = Settings->StressRelaxationIterationsPerTick;
			SavedCoarsenThreshold = Settings->StressCoarsenNodeThreshold;
			Settings->StressRelaxationIterationsPerTick = IterationsPerTick;
			Settings->StressCoarsenNodeThreshold = CoarsenNodeThreshold;
		}

		~FScopedStressSettings()
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			Settings->StressRelaxationIterationsPerTick = SavedIterations;
			Settings->StressCoarsenNodeThreshold = SavedCoarsenThreshold;
		}
	};

	/// Temporarily force interpolated body transforms on or off (read live each
	/// Tick, so the scope only needs to cover the ticks under test). Tests that
	/// assert exact post-step positions must force it OFF: the project config can
	/// enable interpolation, which lags component transforms by up to one step.
	struct FScopedInterpolationSettings
	{
		bool bSaved;

		explicit FScopedInterpolationSettings(bool bEnable = true)
		{
			UBox3DSettings* Settings = GetMutableDefault<UBox3DSettings>();
			bSaved = Settings->bInterpolateBodyTransforms;
			Settings->bInterpolateBodyTransforms = bEnable;
		}

		~FScopedInterpolationSettings()
		{
			GetMutableDefault<UBox3DSettings>()->bInterpolateBodyTransforms = bSaved;
		}
	};

	/// A minimal begun-play game world with a live Box3D subsystem, torn down on
	/// scope exit (which also exercises world destruction each test). Stepping is
	/// driven manually through the subsystem so tests are frame-exact.
	struct FTestWorld
	{
		UWorld* World = nullptr;

		explicit FTestWorld(FName WorldName = TEXT("Box3DTestWorld"))
		{
			// This synthetic world skips actor EndPlay on teardown by design;
			// keep the resulting CleanupWorld warning out of test reports.
			if (FAutomationTestBase* CurrentTest = FAutomationTestFramework::Get().GetCurrentTest())
			{
				CurrentTest->AddExpectedMessage(TEXT("missing call to EndPlay"), ELogVerbosity::Warning,
					EAutomationExpectedMessageFlags::Contains, /*Occurrences*/ -1, /*bIsRegex*/ false);
			}

			World = UWorld::CreateWorld(EWorldType::Game, false, WorldName);
			FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
			Context.SetCurrentWorld(World);

			World->InitializeActorsForPlay(FURL());
			// No game mode in this synthetic world; dispatch BeginPlay directly the
			// way AGameModeBase::StartPlay would.
			World->GetWorldSettings()->NotifyBeginPlay();
			World->GetWorldSettings()->NotifyMatchStarted();
		}

		~FTestWorld()
		{
			GEngine->DestroyWorldContext(World);
			World->RemoveFromRoot();
			World->DestroyWorld(false);
		}

		UBox3DWorldSubsystem& Subsystem() const
		{
			return *World->GetSubsystem<UBox3DWorldSubsystem>();
		}

		b3WorldId B3World() const
		{
			return Subsystem().GetBox3DWorldId();
		}

		static float FixedDt()
		{
			return GetDefault<UBox3DSettings>()->FixedTimeStep;
		}

		/// Advance exactly Count fixed steps (kinematic push + step + component sync).
		void Step(int32 Count = 1) const
		{
			for (int32 Index = 0; Index < Count; ++Index)
			{
				Subsystem().Tick(FixedDt());
			}
		}
	};

	/// Spawn an actor rooted on a body component. Setup runs before registration,
	/// i.e. before BeginPlay creates the Box3D body. Auto-fit is disabled because
	/// these actors carry no primitive; explicit extents are the point.
	inline UBox3DBodyComponent* SpawnBody(UWorld* World, const FVector& Location,
		TFunctionRef<void(UBox3DBodyComponent&)> Setup,
		const FQuat& Rotation = FQuat::Identity, const FVector& Scale = FVector::OneVector)
	{
		AActor* Actor = World->SpawnActor<AActor>();
		UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(Actor, TEXT("Box3DBody"));
		Body->bAutoFitShape = false;
		Setup(*Body);
		Actor->SetRootComponent(Body);
		Body->SetWorldTransform(FTransform(Rotation, Location, Scale));
		Body->RegisterComponent();
		return Body;
	}

	/// Spawn a body actor with an attached static mesh; the mesh registers first so
	/// mesh-derived shapes and auto-fit see it (same pattern as box3d.SmokeActors).
	inline UBox3DBodyComponent* SpawnMeshBody(UWorld* World, UStaticMesh* Mesh, const FVector& Location,
		const FVector& MeshScale, EBox3DBodyType BodyType, EBox3DShapeType ShapeType)
	{
		AActor* Actor = World->SpawnActor<AActor>();
		UBox3DBodyComponent* Body = NewObject<UBox3DBodyComponent>(Actor, TEXT("Box3DBody"));
		Body->BodyType = BodyType;
		Body->ShapeType = ShapeType;
		Actor->SetRootComponent(Body);
		Body->SetWorldLocation(Location);

		UStaticMeshComponent* MeshComponent = NewObject<UStaticMeshComponent>(Actor, TEXT("Mesh"));
		MeshComponent->SetupAttachment(Body);
		MeshComponent->SetStaticMesh(Mesh);
		MeshComponent->SetRelativeScale3D(MeshScale);
		MeshComponent->SetMobility(EComponentMobility::Movable);
		MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		MeshComponent->RegisterComponent();
		Body->RegisterComponent();
		return Body;
	}

	/// The engine's 100 cm basic cube — the standard test mesh (authored box
	/// collision element plus render geometry, so every cook path works).
	inline UStaticMesh* LoadCubeMesh()
	{
		return LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	}

	/// Spawn an actor whose root is a bare UStaticMeshComponent (no Box3D body) —
	/// the raw material of the static scene mirror. Mesh/mobility/collision are set
	/// before registration so Static mobility never trips runtime-change guards.
	inline UStaticMeshComponent* SpawnSceneMesh(UWorld* World, UStaticMesh* Mesh, const FTransform& Transform,
		EComponentMobility::Type Mobility = EComponentMobility::Static,
		ECollisionEnabled::Type Collision = ECollisionEnabled::QueryAndPhysics)
	{
		AActor* Actor = World->SpawnActor<AActor>();
		UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(Actor, TEXT("SceneMesh"));
		Component->SetMobility(Mobility);
		Component->SetStaticMesh(Mesh);
		Component->SetCollisionEnabled(Collision);
		Actor->SetRootComponent(Component);
		Component->SetWorldTransform(Transform);
		Component->RegisterComponent();
		return Component;
	}

	/// Static box ground spanning +-1000 cm in XY with its top face at TopZ.
	inline UBox3DBodyComponent* SpawnGround(UWorld* World, float TopZ = 0.0f,
		int32 CategoryBits = 1 << static_cast<int32>(EBox3DChannel::WorldStatic))
	{
		return SpawnBody(World, FVector(0.0f, 0.0f, TopZ - 25.0f), [&](UBox3DBodyComponent& Body)
		{
			Body.BodyType = EBox3DBodyType::Static;
			Body.ShapeType = EBox3DShapeType::Box;
			Body.BoxHalfExtent = FVector(1000.0, 1000.0, 25.0);
			Body.Filter.CategoryBits = CategoryBits;
		});
	}
}

#endif // WITH_DEV_AUTOMATION_TESTS
