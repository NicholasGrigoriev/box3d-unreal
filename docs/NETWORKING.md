# Networking: authority, snapshots, prediction & rollback

Box3D has no built-in replication — and this plugin deliberately does not
replicate bodies for you. What it provides are the three primitives a networked
physics game needs, ported from
[Antonio Lattanzio's Box3DUnreal](https://github.com/alattanzio/Box3DUnreal)
(MIT) and adapted to this plugin:

1. **Authority gating** — simulate only where authoritative.
2. **Snapshots** — capture/restore/hash per-body state (`Box3DSnapshot.h`).
3. **Rollback + deterministic replay** — client-side prediction reconciliation
   (`FSnapshotRing`, `ReconcileAndReplay`).

## Authority gating

`bAuthorityOnlySimulation` (Project Settings → Box3D → Networking, off by
default): when enabled, only standalone, listen-server, and dedicated-server
worlds create a Box3D world. Pure clients get none — body components stay inert
and every query returns false/empty there (use UE traces on clients, or drive
visuals from replicated authority state). Check
`UBox3DWorldSubsystem::IsSimulationAuthority()` (BlueprintPure) before doing
physics work. The mode is evaluated once, when the world subsystem initializes.

Determinism note: for cross-peer reproducibility keep the same `WorkerCount`
and build on both ends. This plugin's solver is deterministic across worker
counts on one build (validated by the threading tests), but a lockstep/rollback
scheme should still pin the configuration — and `box3d.RecordStart` /
`box3d.ValidateReplay` can verify determinism end-to-end on any machine.

## Snapshots (`Box3DSnapshot.h`)

```cpp
Box3D::FBodyState State = Box3D::CaptureBodyState(BodyId);  // transform + velocities + awake
Box3D::RestoreBodyState(BodyId, State);
uint32 Hash = Box3D::HashBody(B3_HASH_INIT, BodyId);        // djb2, box3d-space, cross-peer comparable
```

Everything stays in box3d space (meters, no UE conversion) so hashes compare
raw simulation state. Feed bodies in a stable order — a cross-machine desync
check additionally needs a shared body ordering (network ids).

**Fidelity caveat:** a snapshot is the *kinematic* state read through the public
getters, not the solver's internal history (warm-start impulses, contact
anchors, sleep timers). Restore-then-replay is exactly deterministic for
contact-free motion, and close-but-not-bit-identical once contacts are involved
— warm starting is the main divergence source (see
`b3World_EnableWarmStarting`). The automation tests pin the contact-free case
bit-for-bit.

## Prediction & rollback

The client simulates opted-in bodies locally so they respond with no round-trip
latency. The server periodically sends an authoritative, frame-tagged state.
Because that state is stale by the network delay, the client rolls the world
back to the server's frame, overwrites the corrected bodies, and
deterministically replays forward to the present — landing where the server's
forward sim *will* be, not where it was when it sent the packet.

```cpp
// Once, with a stable body set:
Box3D::FSnapshotRing Ring;
Ring.Init(/*frames retained*/ 64, Bodies.Num());

// Every fixed step, after stepping (GetStepCount() is the frame number):
Ring.Capture(Frame, Bodies);

// When an authoritative update for AuthFrame arrives:
Box3D::FReconcileResult Result = Box3D::ReconcileAndReplay(
    WorldId, Bodies, Ring,
    AuthFrame, AuthIndices, AuthStates,   // partial authority is fine
    PresentFrame, FixedDt, SubStepCount);
// Result.bCorrected / ReplayedFrames / MaxCorrection (meters) for smoothing.
```

`ReconcileAndReplay` skips the rollback entirely when the authority already
agrees with the ring within tolerance (2 cm default) — the common case when
prediction was right, so no visible pop.

**Scope of this first cut:** the ring assumes a stable body set and order
across its window; spawn/despawn mid-window is not handled. Wire the transport
(what to send, how to tag frames, interest management) in game code — see
`Box3DSnapshotTests.cpp` for frame-exact usage.
