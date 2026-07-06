# Box3D Unreal — Design Notes

## Module layout

```
Box3DUnreal/
├── Box3DUnreal.uplugin
├── Source/
│   ├── Box3DCore/            # vendored box3d, compiled from source by UBT
│   │   ├── Box3DCore.Build.cs
│   │   ├── Public/box3d/     # upstream include/box3d/*.h (public API, unmodified)
│   │   └── Private/          # upstream src/* (unmodified) + Box3DCoreModule.cpp
│   └── Box3DRuntime/         # UE integration layer
│       ├── Box3DRuntime.Build.cs
│       ├── Public/
│       └── Private/
└── docs/
```

`Box3DCore` vendors box3d **unmodified** — upgrades are a folder swap (see UPSTREAM.md).
Erin explicitly supports source embedding: `config.h` allows `BOX3D_USER_CONFIG` and
`BOX3D_EXPORT` overrides, and the library has zero dependencies beyond the C runtime.

## Why compile from source instead of prebuilt libs

- One toolchain: UBT/MSVC builds everything; no CMake at build time, no binary drift
  between Debug/Development/Shipping CRT flavors.
- Cross-platform for free later — UBT compiles the same C17 sources for any target.
- box3d is ~90 files of C17 with MSVC atomics via intrinsics (`platform.h`), no C11
  atomics flag needed.

Build.cs specifics for `Box3DCore`:
- `CStandard = CStandardVersion.C17` (box3d needs `_Static_assert`, anonymous unions)
- No PCH, no unity — C sources compile standalone
- DLL exports: modular (editor) builds make each module a DLL, so the C API must be
  exported. UBT's `BOX3DCORE_API` macro cannot be reused — it expands to UE's
  `DLLEXPORT`, defined in `Platform.h`, which C sources never include. Instead the
  Build.cs uses box3d's native scheme when `LinkType == Modular`: `box3d_EXPORTS`
  (private, dllexport — checked first in `base.h`) + `BOX3D_DLL` (public, dllimport
  for consumers). Monolithic builds define neither and link statically
- `B3_ENABLE_ASSERT` defined in non-Shipping so B3_ASSERT routes to our hook in
  Development builds (upstream only enables asserts when `NDEBUG` is unset, and UE
  defines `NDEBUG` in Development)

## Coordinate & unit conventions

**Axes: pass-through. Units: cm ↔ m (×0.01 / ×100).**

Box3D imposes no up-axis or handedness convention ("Box3D has no up-vector defined" —
gravity is a plain vector, quaternion math is standard Hamilton). A simulation fed
consistently with UE's Z-up coordinates is internally consistent; there is no need for
a handedness flip. This keeps conversion trivial and cheap:

- `FVector (cm)` ↔ `b3Vec3 / b3Pos (m)`: scale by `UE_TO_B3 = 0.01` / `B3_TO_UE = 100`
- `FQuat` ↔ `b3Quat`: direct component copy (both are (x,y,z,w) Hamilton quaternions;
  b3Quat stores `{v.x, v.y, v.z, s}`)
- Angular velocity, torque: pass-through (rad/s, N·m in box3d scale)
- Default gravity: UE world gravity Z (−980 cm/s²) → `(0, 0, −9.8) m/s²`

The world-to-meters scale is fixed at 100 (UE default). `AWorldSettings::WorldToMeters`
support can come later if ever needed.

## Precision

box3d builds in single precision (default). UE5's double-precision LWC vectors convert
at the boundary. `BOX3D_DOUBLE_PRECISION` (double `b3Pos`, ABI-affecting) is deferred
to M6 — it must be defined identically in both modules via a shared Build.cs switch.

## World ownership

`UBox3DWorldSubsystem` (a `UTickableWorldSubsystem`) owns exactly one `b3WorldId` per
UWorld (game/PIE worlds only). Stepping uses a fixed timestep (default 1/60 s, 4
substeps) with an accumulator in `Tick`, clamped to avoid spiral-of-death. Interpolation
of render transforms between fixed steps is an M1+ concern.

Global hooks (`b3SetAllocator` → `FMemory::Malloc/Free`, `b3SetAssertFcn`,
`b3SetLogFcn` → `LogBox3D`) are installed once in `FBox3DRuntimeModule::StartupModule`.

## Threading

`b3World_Step` runs on the game thread. With `WorkerCount > 1` it forks solver tasks
through the world's task callbacks and blocks in `finishTask` across every fork/join —
safe here because the game thread may block freely, and the calling thread doubles as
worker 0 (box3d's orchestrator CAS guarantees progress even if the task system runs
tasks late, out of order, or inline).

`FBox3DUETaskPool` (M5) bridges the callbacks onto `UE::Tasks`: enqueue launches a
high-priority task into a fixed `B3_MAX_TASKS` slot array (box3d wants stable task
pointers; the slot counter is atomic because the solve orchestrator enqueues follow-up
tasks from worker threads), finish is `FTask::Wait()` — which, when called from inside
a worker, retracts or helps instead of deadlocking, the same strategy as box3d's
in-tree scheduler. Task names from box3d can live in stack buffers, so they are not
retained. `UBox3DSettings::TaskSystem` picks UE tasks (default — no extra threads) or
box3d's internal scheduler (dedicated threads); `WorkerCount = 1` (default) stays
fully serial. `box3d.Benchmark` compares the three configurations on a target machine.

The step is deterministic across worker counts and schedulers
(`Box3DUnreal.Threading.SchedulerEquivalence` pins this).

## Continuous collision (fast movers)

`UBox3DSettings::bEnableContinuous` (default on) makes box3d sweep fast bodies
against **static** geometry, so projectiles at hundreds of m/s stop on thin walls
with no configuration (`World.ContinuousFastBody` pins 150 m/s vs a 4 cm plate).
Set `bIsBullet` on a body component only when it must not tunnel through **thin
dynamic** objects — bullets are swept against dynamic shapes too, which costs
more, so reserve the flag for genuine projectiles. Speeds are capped by box3d at
400 m/s (`b3WorldDef.maximumLinearSpeed` default); beyond that, prefer ray/shape
casts (`Box3DRayCast`/`Box3DSphereCast`) over simulated projectiles.

## Determinism, recording, replay

box3d steps are deterministic for a given binary across worker counts and
schedulers. The subsystem wraps box3d's recorder: `StartRecording` snapshots the
world and records every mutation with per-step state hashes; `ValidateLastRecording`
(or `box3d.ValidateReplay <file>`) replays in a scratch world and compares hashes,
which is the supported desync/regression check. Replaying at a different worker
count than recorded re-partitions the constraint graph, turning the same check
into a cross-thread determinism test (`Replay.CrossWorkerDeterminism`).

## Naming

- C++ classes: `UBox3D…` / `FBox3D…` prefix, log category `LogBox3D`
- Console commands/CVars: `box3d.*`
