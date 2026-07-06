# Box3D Unreal — Testing

Two layers of verification:

1. **Automation tests** (`Source/Box3DRuntime/Private/Tests/`) — deterministic,
   analytic assertions against synthetic game worlds. The authority on correctness.
2. **Smoke commands** (`box3d.Smoke`, `box3d.SmokeActors`) — visual/integration
   checks in a real PIE session, plus the `box3d.AutoSmokeActors` headless hook.

## Running the automation tests

In the editor: Tools ▸ Test Automation ▸ filter `Box3DUnreal`, or from the console:

```
Automation RunTests Box3DUnreal
```

Headless (CI-style):

```
UnrealEditor-Cmd.exe <project>.uproject -ExecCmds="Automation RunTests Box3DUnreal" ^
  -TestExit="Automation Test Queue Empty" -ReportExportPath=<dir> ^
  -unattended -nullrhi -nop4 -nosplash -log
```

The report directory receives `index.json` with pass/fail per test.

## How the tests work

`Box3DTestHelpers.h` provides `FTestWorld`: a synthetic `EWorldType::Game` world
(created via `UWorld::CreateWorld`, BeginPlay dispatched through
`AWorldSettings::NotifyBeginPlay`) whose Box3D subsystem is stepped **manually** —
`Test.Step(N)` runs exactly N fixed steps, so every assertion is frame-exact and
deterministic. No latent commands, no wall-clock waits.

## Coverage matrix (M0–M3)

| Area | Behavior | Test |
| --- | --- | --- |
| Conversion | cm↔m scaling, `b3Pos` seam, gravity accel | `Conversion.LengthScale` |
| Conversion | directions/angular velocity unscaled | `Conversion.DirectionUnscaled` |
| Conversion | quaternion equivalence (rotate on both sides) | `Conversion.QuaternionEquivalence` |
| Conversion | int32→uint64 filter widening, sign-bit zero-extend | `Conversion.FilterBitsWidening` |
| M0 world | subsystem per game world, gravity from settings, editor worlds excluded | `World.SubsystemLifecycle` |
| M0 world | accumulator: sub-step frames, hitch clamp to MaxStepsPerTick | `World.FixedStepAccumulator` |
| M0 world | free fall v=g·t, physics→component sync | `World.FreeFallGravity` |
| M1 body | create on BeginPlay, userData backref, destroy with actor | `Body.CreateDestroy` |
| M1 body | analytic masses: box/sphere/capsule, static=0, component scale | `Body.MassAnalytic` |
| M1 body | gravity scale 0/0.5, linear + angular motion locks | `Body.GravityScaleAndMotionLocks` |
| M1 body | velocity round trips, impulse dv=J/m, force dv=F/m·dt, angular impulse ω=L/I (unit scales) | `Body.VelocityForceImpulse` |
| M1 body | bStartAwake, SetAwake, auto-sleep at rest, enable/disable | `Body.SleepWakeEnable` |
| M1 body | static follows component; dynamic ignores plain moves, honors teleports | `Body.TransformOwnership` |
| M1 body | kinematic target tracking via SetTargetTransform, unregister on destroy | `Body.KinematicTargetTracking` |
| M2 filter | shape mask vs ground category, negative group never collides | `Body.CollisionFilterAndGroup` |
| M2 material | friction/restitution/userMaterialId on shape, UPhysicalMaterial override, id surfaced in hits | `Body.MaterialsAndUserMaterialId` |
| M2 query | ray closest: location/fraction/normal/component; multi sorted; miss | `Query.RayClosestAndMulti` |
| M2 query | sphere + capsule casts, analytic stop fractions | `Query.SphereAndCapsuleCast` |
| M2 query | overlap membership, query masks on overlap and ray | `Query.OverlapAndFilter` |
| M2 mover | cast fraction analytic, free-air 1.0, plane solve blocks descent, collide-and-slide | `Query.MoverCastAndSolve` |
| M3 cooking | trimesh cook, per-asset cache identity, winding (top-face normal), non-uniform scale, triangle index | `Cooking.TriangleMeshWindingAndCache` |
| M3 cooking | hull cook + cache, hull mass vs analytic cylinder | `Cooking.ConvexHullCacheAndMass` |
| M3 cooking | CollisionAsset box element (exact mass, shape count), sphere/convex elements (data-conditional), auto-fit from mesh bounds | `Cooking.CollisionAssetElements` |

## Known gaps (deliberate)

- **`bIsBullet` / CCD behavior** — passthrough flag; meaningful assertions need
  fast-mover scenarios, deferred to M6 CCD guidance work.
- **Damping decay curves** — passthrough to `b3BodyDef`; exact decay depends on
  solver internals, not asserted.
- **TriangleMesh-on-dynamic fallback warning path** — emits a warning by design;
  exercised manually, not asserted (would require expected-message plumbing).
- **Render-vertex hull fallback for meshes with authored collision** — engine
  basic shapes all have authored collision, so only the authored path and the
  cylinder (no simple collision → render fallback) are covered.
- **Landscape height fields, `b3CreateCompound`** — deferred features (see
  MILESTONES M3 notes); no tests until implemented.
- **Threading (`WorkerCount > 1`)** — M5 scope.

## Smoke commands

Still useful for eyeballing behavior and stressing at scale:

- `box3d.Smoke [N]` — raw debug-drawn bodies (M0 pipeline).
- `box3d.SmokeActors [N]` — full component pipeline with rendered meshes; logs
  settle state + query self-checks ~8 s after spawn when driven by
  `box3d.AutoSmokeActors N` (headless).
