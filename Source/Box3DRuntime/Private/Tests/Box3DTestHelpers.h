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
	/// A minimal begun-play game world with a live Box3D subsystem, torn down on
	/// scope exit (which also exercises world destruction each test). Stepping is
	/// driven manually through the subsystem so tests are frame-exact.
	struct FTestWorld
	{
		UWorld* World = nullptr;

		FTestWorld()
		{
			// This synthetic world skips actor EndPlay on teardown by design;
			// keep the resulting CleanupWorld warning out of test reports.
			if (FAutomationTestBase* CurrentTest = FAutomationTestFramework::Get().GetCurrentTest())
			{
				CurrentTest->AddExpectedMessage(TEXT("missing call to EndPlay"), ELogVerbosity::Warning,
					EAutomationExpectedMessageFlags::Contains, /*Occurrences*/ -1, /*bIsRegex*/ false);
			}

			World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("Box3DTestWorld"));
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
