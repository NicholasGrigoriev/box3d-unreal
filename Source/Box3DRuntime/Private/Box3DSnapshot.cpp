// Ported from Antonio Lattanzio's Box3DUnreal (github.com/alattanzio/Box3DUnreal, MIT).

#include "Box3DSnapshot.h"

namespace Box3D
{
	void FSnapshotRing::Init(int32 InCapacity, int32 InBodyCount)
	{
		Capacity = FMath::Max(1, InCapacity);
		BodyCount = FMath::Max(0, InBodyCount);
		Newest = -1;
		States.SetNum(Capacity * BodyCount);
		FrameAt.Init(-1, Capacity);
	}

	int32 FSnapshotRing::FindSlot(int32 Frame) const
	{
		if (Capacity == 0 || Frame < 0)
		{
			return INDEX_NONE;
		}
		const int32 Slot = ((Frame % Capacity) + Capacity) % Capacity;
		return (FrameAt.IsValidIndex(Slot) && FrameAt[Slot] == Frame) ? Slot : INDEX_NONE;
	}

	int32 FSnapshotRing::OldestRetainedFrame() const
	{
		if (Newest < 0)
		{
			return -1;
		}
		// The window is the last Capacity frames; the oldest still-valid frame is
		// the lowest one whose slot hasn't been overwritten.
		const int32 Lowest = FMath::Max(0, Newest - Capacity + 1);
		return FindSlot(Lowest) != INDEX_NONE ? Lowest : -1;
	}

	void FSnapshotRing::Capture(int32 Frame, const TArray<b3BodyId>& Bodies)
	{
		if (Capacity == 0 || Frame < 0 || Bodies.Num() != BodyCount)
		{
			return;
		}
		const int32 Slot = ((Frame % Capacity) + Capacity) % Capacity;
		FrameAt[Slot] = Frame;
		FBodyState* Row = &States[Slot * BodyCount];
		for (int32 Index = 0; Index < BodyCount; ++Index)
		{
			Row[Index] = CaptureBodyState(Bodies[Index]);
		}
		Newest = FMath::Max(Newest, Frame);
	}

	bool FSnapshotRing::GetStates(int32 Frame, TArray<FBodyState>& OutStates) const
	{
		const int32 Slot = FindSlot(Frame);
		if (Slot == INDEX_NONE)
		{
			return false;
		}
		OutStates.SetNumUninitialized(BodyCount);
		const FBodyState* Row = &States[Slot * BodyCount];
		for (int32 Index = 0; Index < BodyCount; ++Index)
		{
			OutStates[Index] = Row[Index];
		}
		return true;
	}

	bool FSnapshotRing::Restore(int32 Frame, const TArray<b3BodyId>& Bodies) const
	{
		const int32 Slot = FindSlot(Frame);
		if (Slot == INDEX_NONE || Bodies.Num() != BodyCount)
		{
			return false;
		}
		const FBodyState* Row = &States[Slot * BodyCount];
		for (int32 Index = 0; Index < BodyCount; ++Index)
		{
			RestoreBodyState(Bodies[Index], Row[Index]);
		}
		return true;
	}

	namespace
	{
		double PositionDelta(const b3Pos& A, const b3Pos& B)
		{
			const double X = double(A.x) - double(B.x);
			const double Y = double(A.y) - double(B.y);
			const double Z = double(A.z) - double(B.z);
			return FMath::Sqrt(X * X + Y * Y + Z * Z);
		}
	}

	FReconcileResult ReconcileAndReplay(
		b3WorldId World,
		const TArray<b3BodyId>& Bodies,
		FSnapshotRing& Ring,
		int32 AuthFrame,
		const TArray<int32>& AuthIndices,
		const TArray<FBodyState>& AuthStates,
		int32 PresentFrame,
		float TimeStep,
		int32 SubStepCount,
		double PositionTolerance)
	{
		FReconcileResult Result;

		// Can't roll back past the ring window, or forward.
		if (!Ring.IsRetained(AuthFrame) || AuthFrame > PresentFrame || AuthIndices.Num() != AuthStates.Num())
		{
			return Result;
		}

		// Compare the authority to what the client predicted at that frame. If
		// every corrected body already agrees within tolerance, prediction was
		// right — skip the rollback entirely.
		TArray<FBodyState> Predicted;
		if (!Ring.GetStates(AuthFrame, Predicted))
		{
			return Result;
		}

		double MaxError = 0.0;
		for (int32 Index = 0; Index < AuthIndices.Num(); ++Index)
		{
			const int32 BodyIndex = AuthIndices[Index];
			if (Predicted.IsValidIndex(BodyIndex))
			{
				MaxError = FMath::Max(MaxError,
					PositionDelta(Predicted[BodyIndex].Transform.p, AuthStates[Index].Transform.p));
			}
		}
		if (MaxError <= PositionTolerance)
		{
			return Result; // within tolerance: no correction, no pop
		}

		// Roll the whole world back to the authoritative frame (all bodies from
		// the ring)...
		if (!Ring.Restore(AuthFrame, Bodies))
		{
			return Result;
		}
		// ...then stamp the authoritative bodies over their predicted state.
		for (int32 Index = 0; Index < AuthIndices.Num(); ++Index)
		{
			const int32 BodyIndex = AuthIndices[Index];
			if (Bodies.IsValidIndex(BodyIndex))
			{
				RestoreBodyState(Bodies[BodyIndex], AuthStates[Index]);
			}
		}
		// Overwrite the ring entry at AuthFrame so it reflects the corrected state.
		Ring.Capture(AuthFrame, Bodies);

		// Deterministically replay forward to the present, re-capturing each frame
		// so the ring stays valid for the next correction.
		for (int32 Frame = AuthFrame + 1; Frame <= PresentFrame; ++Frame)
		{
			b3World_Step(World, TimeStep, SubStepCount);
			Ring.Capture(Frame, Bodies);
			++Result.ReplayedFrames;
		}

		Result.bCorrected = true;
		Result.MaxCorrection = MaxError;
		return Result;
	}
}
