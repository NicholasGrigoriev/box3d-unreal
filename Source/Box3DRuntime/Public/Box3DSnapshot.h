#pragma once

#include "CoreMinimal.h"
#include "box3d/box3d.h"

/// Per-body state capture / restore / hash plus the client-side prediction and
/// rollback core built on top of it.
///
/// Ported from Antonio Lattanzio's Box3DUnreal (github.com/alattanzio/Box3DUnreal,
/// MIT) and adapted to this plugin's conventions. See docs/NETWORKING.md.
///
/// box3d exposes no live-world serialize, so a "snapshot" is the kinematic state
/// of each body read back through the public getters: transform + velocity +
/// awake flag, NOT the solver's internal history (warm-start impulses, contact
/// anchors, sleep timers). Restoring then re-simulating is therefore NOT
/// guaranteed bit-identical to a run that never rolled back when contacts are
/// involved; warm starting is the main divergence source (see
/// b3World_EnableWarmStarting). Everything here stays in box3d space — no UE
/// conversion — so a hash compares raw simulation state.
namespace Box3D
{
	/// One dynamic body's restorable state, in box3d space.
	struct FBodyState
	{
		b3WorldTransform Transform{};
		b3Vec3 LinearVelocity{};
		b3Vec3 AngularVelocity{};
		bool bAwake = false;
	};

	FORCEINLINE FBodyState CaptureBodyState(b3BodyId Body)
	{
		FBodyState State;
		State.Transform = b3Body_GetTransform(Body);
		State.LinearVelocity = b3Body_GetLinearVelocity(Body);
		State.AngularVelocity = b3Body_GetAngularVelocity(Body);
		State.bAwake = b3Body_IsAwake(Body);
		return State;
	}

	FORCEINLINE void RestoreBodyState(b3BodyId Body, const FBodyState& State)
	{
		b3Body_SetTransform(Body, State.Transform.p, State.Transform.q);
		b3Body_SetLinearVelocity(Body, State.LinearVelocity);
		b3Body_SetAngularVelocity(Body, State.AngularVelocity);
		// Order matters: setting velocity wakes a body, so apply the sleep flag
		// last to keep a body that was asleep at capture time asleep.
		b3Body_SetAwake(Body, State.bAwake);
	}

	/// Fold one body's state into a running djb2 hash (b3Hash). Feed states in a
	/// stable order so the same world produces the same digest across runs and
	/// peers. Start from B3_HASH_INIT.
	FORCEINLINE uint32 HashBodyState(uint32 Hash, const FBodyState& State)
	{
		Hash = b3Hash(Hash, reinterpret_cast<const uint8_t*>(&State.Transform), sizeof(State.Transform));
		Hash = b3Hash(Hash, reinterpret_cast<const uint8_t*>(&State.LinearVelocity), sizeof(State.LinearVelocity));
		Hash = b3Hash(Hash, reinterpret_cast<const uint8_t*>(&State.AngularVelocity), sizeof(State.AngularVelocity));
		return Hash;
	}

	FORCEINLINE uint32 HashBody(uint32 Hash, b3BodyId Body)
	{
		const FBodyState State = CaptureBodyState(Body);
		return HashBodyState(Hash, State);
	}

	/// Fixed-capacity ring of whole-world snapshots, one entry per fixed step.
	/// Rollback needs the *entire* dynamic body set at the rollback frame
	/// (interacting bodies must all be restored before the replay re-steps), so
	/// this stores every body's FBodyState per frame.
	///
	/// Assumes a stable body set and order across frames — true for persistent
	/// bodies; spawn/despawn mid-window is out of scope for this first cut.
	class BOX3DRUNTIME_API FSnapshotRing
	{
	public:
		/// Size the ring for Capacity frames of BodyCount bodies. Clears any
		/// existing contents.
		void Init(int32 InCapacity, int32 InBodyCount);

		/// Store every body's current state under Frame, evicting the oldest slot
		/// if full.
		void Capture(int32 Frame, const TArray<b3BodyId>& Bodies);

		/// Restore all bodies to their stored state at Frame. False if Frame is no
		/// longer retained (older than the ring window) or the body count differs.
		bool Restore(int32 Frame, const TArray<b3BodyId>& Bodies) const;

		/// Read stored states for Frame without touching the world. False if not
		/// retained.
		bool GetStates(int32 Frame, TArray<FBodyState>& OutStates) const;

		bool IsRetained(int32 Frame) const { return FindSlot(Frame) != INDEX_NONE; }
		int32 NewestFrame() const { return Newest; }
		int32 OldestRetainedFrame() const;
		bool IsEmpty() const { return Newest < 0; }

	private:
		int32 FindSlot(int32 Frame) const;

		int32 Capacity = 0;
		int32 BodyCount = 0;
		int32 Newest = -1;         // highest frame captured, -1 when empty
		TArray<FBodyState> States; // Capacity * BodyCount, row-major by slot
		TArray<int32> FrameAt;     // Capacity; frame in each slot, -1 = empty
	};

	/// Outcome of a reconcile, for logging / smoothing decisions.
	struct FReconcileResult
	{
		bool bCorrected = false;    // the world was rolled back and replayed
		int32 ReplayedFrames = 0;   // how many frames were re-stepped
		double MaxCorrection = 0.0; // largest per-body position change (meters)
	};

	/// Roll World back to AuthFrame, overwrite the bodies named in AuthIndices
	/// with AuthStates, keep every other body at its ring state for that frame,
	/// then re-step to PresentFrame — re-capturing the ring as it goes so it stays
	/// valid. Deterministic replay means the result matches the authority's
	/// forward sim from the same state.
	///
	/// Skips the rollback when the authoritative state already agrees with the
	/// ring within PositionTolerance (meters) for every corrected body — the
	/// common case when prediction was right, so no visible pop.
	///
	/// @param AuthIndices indices into Bodies that AuthStates corresponds to
	///        (partial auth allowed)
	BOX3DRUNTIME_API FReconcileResult ReconcileAndReplay(
		b3WorldId World,
		const TArray<b3BodyId>& Bodies,
		FSnapshotRing& Ring,
		int32 AuthFrame,
		const TArray<int32>& AuthIndices,
		const TArray<FBodyState>& AuthStates,
		int32 PresentFrame,
		float TimeStep,
		int32 SubStepCount,
		double PositionTolerance = 0.02); // 2 cm
}
