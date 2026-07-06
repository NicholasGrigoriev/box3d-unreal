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

M0/M1 run with `workerCount = 1` (single-threaded stepping on the game thread).
Box3D's internal scheduler (raw `CreateThread` workers) or UE task-system callbacks
come in M5. Note the contract in `types.h`: `b3World_Step` blocks across fork/join, so
if we drive it from a UE worker it must be a thread that can block safely — never a
task-graph job that can't park its stack.

## Naming

- C++ classes: `UBox3D…` / `FBox3D…` prefix, log category `LogBox3D`
- Console commands/CVars: `box3d.*`
