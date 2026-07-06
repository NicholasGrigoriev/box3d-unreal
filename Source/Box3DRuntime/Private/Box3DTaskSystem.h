#pragma once

#include "CoreMinimal.h"
#include "Tasks/Task.h"
#include "box3d/constants.h"
#include "box3d/types.h"

#include <atomic>

/// Bridges box3d's fork/join task callbacks onto UE::Tasks worker threads.
///
/// b3World_Step forks work via enqueueTask and blocks inside finishTask until each
/// task completes, holding its stack across every fork/join. That contract is safe
/// here because the step runs on the game thread (which may block freely) and
/// UE::Tasks supports waiting from inside a worker by retracting or helping with
/// other tasks — the same strategy box3d's in-tree scheduler uses.
///
/// Task handles live in a fixed array: box3d never enqueues more than B3_MAX_TASKS
/// per step and recommends stable user task pointers. The slot counter is atomic
/// because the solve orchestrator may enqueue follow-up tasks from a worker thread
/// while the game thread is still blocked joining earlier ones.
class FBox3DUETaskPool
{
public:
	/// Point a world definition's task callbacks at this pool. The pool must
	/// outlive the world created from the definition.
	void ApplyToWorldDef(b3WorldDef& Def, int32 WorkerCount)
	{
		Def.workerCount = static_cast<uint32>(FMath::Clamp(WorkerCount, 1, B3_MAX_WORKERS));
		Def.enqueueTask = &FBox3DUETaskPool::EnqueueThunk;
		Def.finishTask = &FBox3DUETaskPool::FinishThunk;
		Def.userTaskContext = this;
	}

	/// Release last step's task handles. Call before each b3World_Step; every task
	/// has been joined by the time the previous step returned, so this only drops
	/// references.
	void ResetForStep()
	{
		const int32 Used = FMath::Min(SlotCount.load(std::memory_order_relaxed), int32(B3_MAX_TASKS));
		for (int32 Index = 0; Index < Used; ++Index)
		{
			Slots[Index] = UE::Tasks::FTask();
		}
		SlotCount.store(0, std::memory_order_relaxed);
	}

	/// Lifetime total of tasks handed to UE workers; proves the hookup actually
	/// fans out (used by tests and the benchmark report).
	int64 GetTotalEnqueued() const { return TotalEnqueued.load(std::memory_order_relaxed); }

private:
	static void* EnqueueThunk(b3TaskCallback* Task, void* TaskContext, void* UserContext, const char* /*TaskName*/)
	{
		FBox3DUETaskPool* Pool = static_cast<FBox3DUETaskPool*>(UserContext);
		const int32 Slot = Pool->SlotCount.fetch_add(1, std::memory_order_relaxed);
		if (Slot >= B3_MAX_TASKS)
		{
			// box3d guarantees at most B3_MAX_TASKS enqueues per step; if that ever
			// breaks, run serially — a nullptr return means "already executed".
			Task(TaskContext);
			return nullptr;
		}

		Pool->TotalEnqueued.fetch_add(1, std::memory_order_relaxed);
		// High priority: the game thread blocks on these inside the step, so any
		// queueing latency is a direct frame-time cost.
		Pool->Slots[Slot] = UE::Tasks::Launch(TEXT("Box3DStepTask"),
			[Task, TaskContext] { Task(TaskContext); },
			UE::Tasks::ETaskPriority::High);
		return &Pool->Slots[Slot];
	}

	static void FinishThunk(void* UserTask, void* /*UserContext*/)
	{
		static_cast<UE::Tasks::FTask*>(UserTask)->Wait();
	}

	UE::Tasks::FTask Slots[B3_MAX_TASKS];
	std::atomic<int32> SlotCount{ 0 };
	std::atomic<int64> TotalEnqueued{ 0 };
};
