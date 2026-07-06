// box3d.Benchmark — standalone scheduler comparison for sizing WorkerCount.
//
// Simulates an identical contact-heavy pile in throwaway b3 worlds (independent of
// any UWorld/subsystem) once per scheduler configuration: serial, box3d's internal
// threads, and UE::Tasks. Results go to LogBox3D; run on a target machine to pick
// UBox3DSettings::WorkerCount / TaskSystem.

#if !UE_BUILD_SHIPPING

#include "Box3DRuntime.h"
#include "Box3DTaskSystem.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

namespace
{
	enum class EBenchScheduler
	{
		Serial,
		Internal,
		UETasks,
	};

	struct FBenchResult
	{
		double AvgMs = 0.0;
		double P95Ms = 0.0;
		double MaxMs = 0.0;
		int32 FinalContacts = 0;
		int64 TasksEnqueued = 0;
	};

	// First steps pay one-off costs (tree builds, initial islanding); skip them
	// when averaging so configs are compared on steady-state throughput.
	constexpr int32 WarmupSteps = 10;

	FBenchResult RunBenchConfig(int32 BodyCount, int32 Steps, int32 Workers, EBenchScheduler Scheduler)
	{
		FBox3DUETaskPool TaskPool;

		b3WorldDef Def = b3DefaultWorldDef();
		Def.gravity = b3Vec3{ 0.0f, 0.0f, -9.8f };
		Def.enableSleep = false; // keep every body active for an honest comparison
		switch (Scheduler)
		{
			case EBenchScheduler::Serial:
				Def.workerCount = 1;
				break;
			case EBenchScheduler::Internal:
				Def.workerCount = static_cast<uint32>(Workers);
				break;
			case EBenchScheduler::UETasks:
				TaskPool.ApplyToWorldDef(Def, Workers);
				break;
		}

		const b3WorldId WorldId = b3CreateWorld(&Def);

		// Static ground slab, 100 m x 100 m.
		{
			b3BodyDef BodyDef = b3DefaultBodyDef();
			BodyDef.type = b3_staticBody;
			BodyDef.position = b3Pos{ 0.0f, 0.0f, -0.25f };
			const b3BodyId GroundId = b3CreateBody(WorldId, &BodyDef);
			const b3BoxHull GroundHull = b3MakeBoxHull(50.0f, 50.0f, 0.25f);
			b3ShapeDef ShapeDef = b3DefaultShapeDef();
			b3CreateHullShape(GroundId, &ShapeDef, &GroundHull.base);
		}

		// Deterministic grid of 0.25 m half-extent boxes, 20x20 per layer, packed
		// tightly enough that the pile stays in contact while settling.
		{
			const b3BoxHull BoxHull = b3MakeBoxHull(0.25f, 0.25f, 0.25f);
			constexpr int32 Columns = 20;
			constexpr float Spacing = 0.6f;
			constexpr float GridOffset = 0.5f * (Columns - 1) * Spacing;
			for (int32 Index = 0; Index < BodyCount; ++Index)
			{
				const int32 Col = Index % Columns;
				const int32 Row = (Index / Columns) % Columns;
				const int32 Layer = Index / (Columns * Columns);

				b3BodyDef BodyDef = b3DefaultBodyDef();
				BodyDef.type = b3_dynamicBody;
				BodyDef.position = b3Pos{ Col * Spacing - GridOffset, Row * Spacing - GridOffset, 0.3f + Layer * 0.55f };
				const b3BodyId BodyId = b3CreateBody(WorldId, &BodyDef);
				b3ShapeDef ShapeDef = b3DefaultShapeDef();
				b3CreateHullShape(BodyId, &ShapeDef, &BoxHull.base);
			}
		}

		TArray<double> StepMs;
		StepMs.Reserve(Steps);
		for (int32 Step = 0; Step < Steps; ++Step)
		{
			if (Scheduler == EBenchScheduler::UETasks)
			{
				TaskPool.ResetForStep();
			}
			const double Start = FPlatformTime::Seconds();
			b3World_Step(WorldId, 1.0f / 60.0f, 4);
			StepMs.Add((FPlatformTime::Seconds() - Start) * 1000.0);
		}

		FBenchResult Result;
		Result.FinalContacts = b3World_GetCounters(WorldId).contactCount;
		Result.TasksEnqueued = TaskPool.GetTotalEnqueued();
		b3DestroyWorld(WorldId);

		TArray<double> Sorted(StepMs.GetData() + WarmupSteps, StepMs.Num() - WarmupSteps);
		Sorted.Sort();
		double Total = 0.0;
		for (const double Ms : Sorted)
		{
			Total += Ms;
		}
		Result.AvgMs = Total / Sorted.Num();
		Result.P95Ms = Sorted[FMath::Min(Sorted.Num() - 1, (Sorted.Num() * 95) / 100)];
		Result.MaxMs = Sorted.Last();
		return Result;
	}
}

static FAutoConsoleCommand GBox3DBenchmarkCommand(
	TEXT("box3d.Benchmark"),
	TEXT("Compare box3d scheduler configurations on a contact-heavy pile in standalone physics worlds.\n")
	TEXT("Usage: box3d.Benchmark [Bodies=5000] [Steps=180] [Workers=physical cores]. Blocks the game thread while running."),
	FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args)
	{
		const int32 BodyCount = FMath::Clamp(Args.Num() > 0 ? FCString::Atoi(*Args[0]) : 5000, 1, 100000);
		const int32 Steps = FMath::Clamp(Args.Num() > 1 ? FCString::Atoi(*Args[1]) : 180, WarmupSteps + 1, 100000);
		const int32 Workers = FMath::Clamp(Args.Num() > 2 ? FCString::Atoi(*Args[2]) : FPlatformMisc::NumberOfCores(), 2, 16);

		UE_LOG(LogBox3D, Log, TEXT("Box3D benchmark: %d bodies, %d steps (first %d warmup), 4 substeps, %d workers where applicable"),
			BodyCount, Steps, WarmupSteps, Workers);

		struct FConfig
		{
			const TCHAR* Name;
			EBenchScheduler Scheduler;
		};
		const FConfig Configs[] = {
			{ TEXT("serial (1 thread)"), EBenchScheduler::Serial },
			{ TEXT("box3d internal"), EBenchScheduler::Internal },
			{ TEXT("UE tasks"), EBenchScheduler::UETasks },
		};

		for (const FConfig& Config : Configs)
		{
			const FBenchResult Result = RunBenchConfig(BodyCount, Steps, Workers, Config.Scheduler);
			UE_LOG(LogBox3D, Log, TEXT("  %-18s: avg %6.2f ms  p95 %6.2f ms  max %6.2f ms  | final contacts %d%s"),
				Config.Name, Result.AvgMs, Result.P95Ms, Result.MaxMs, Result.FinalContacts,
				Result.TasksEnqueued > 0 ? *FString::Printf(TEXT("  (%lld tasks enqueued)"), Result.TasksEnqueued) : TEXT(""));
		}
	}));

#endif // !UE_BUILD_SHIPPING
