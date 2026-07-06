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

## Coverage matrix

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
| M4 joints | distance: rigid rope length, constraint force = m·g, break threshold + OnJointBroke | `Joints.DistanceRopeAndBreak` |
| M4 joints | revolute: radius pinned, hinge plane held, motor reaches speed on the hinge axis | `Joints.RevoluteHingeAndMotor` |
| M4 joints | prismatic: axis-only slide, lower limit stop, GetTranslation, rotation locked | `Joints.PrismaticSlideAndLimits` |
| M4 joints | spherical: socket radius held while swinging | `Joints.SphericalPendulum` |
| M4 joints | weld: relative pose rigid under impulse | `Joints.WeldRigid` |
| M4 joints | motor joint: drives to target relative velocity | `Joints.MotorJointVelocity` |
| M4 joints | wheel: suspension sag = g/(2πf)², spin motor on the axle axis | `Joints.WheelSuspensionAndSpin` |
| M4 events | contact begin on landing (other resolved), end on separation, silent without the flag | `Events.ContactBeginEnd` |
| M4 events | hit event: free-fall approach speed, normal toward self | `Events.HitApproachSpeed` |
| M4 events | sensor begin/end on pass-through, visitor resolved, no collision response | `Events.SensorOverlap` |
| M5 threading | UE-tasks and internal-scheduler runs match a serial run on a 180-body pile; tasks actually reach UE workers | `Threading.SchedulerEquivalence` |
| M5 diagnostics | GetWorldStats (the `stat box3d` data): profile times measured, body/shape/contact/joint/island/awake counters, memory | `Diagnostics.WorldStats` |
| M5 diagnostics | debug-shape cache: lazy one-entry-per-shape build, exact wireframe point counts per shape type, reuse on redraw, release on shape destroy | `Diagnostics.DebugDrawCache` |
| M6 replay | record (seed + mid-recording spawns), hash-validated replay, file save/load roundtrip validates | `Replay.RecordAndValidate` |
| M6 replay | serial recording replayed at 4 workers: every per-step state hash matches (cross-thread determinism) | `Replay.CrossWorkerDeterminism` |
| M6 world | explosion: dv = 3·I/(4·r·ρ) exact at full scale, half at falloff midpoint, zero beyond, no spin, mask filtering | `World.ExplosionImpulse` |
| M6 world | 150 m/s bodies: continuous collision stops non-bullet vs static thin plate; bullet flag stops dynamic-vs-dynamic, momentum transferred | `World.ContinuousFastBody` |

## Known gaps (deliberate)

- **Spherical cone/twist limits, wheel steering, parallel/filter joints,
  motor-joint position springs** — properties are wired through to box3d but
  their steady states are awkward to pin analytically; covered indirectly by
  the def plumbing being shared with tested paths.
- **`BOX3D_DOUBLE_PRECISION`** — the suite ran 40/40 green under the double ABI
  at M6, but CI-style runs use the default single-precision build; re-run under
  double when bumping upstream (see UPSTREAM.md).
- **Damping decay curves** — passthrough to `b3BodyDef`; exact decay depends on
  solver internals, not asserted.
- **TriangleMesh-on-dynamic fallback warning path** — emits a warning by design;
  exercised manually, not asserted (would require expected-message plumbing).
- **Render-vertex hull fallback for meshes with authored collision** — engine
  basic shapes all have authored collision, so only the authored path and the
  cylinder (no simple collision → render fallback) are covered.
- **Landscape height fields, `b3CreateCompound`** — deferred features (see
  MILESTONES M3 notes); no tests until implemented.
- **`stat box3d` display plumbing** — the SET_*_STAT macros are engine-side; the
  values feeding them are asserted through `GetWorldStats`.
- **The DrawDebugHelpers pass** (`box3d.DebugDraw` CVar path) — line-batcher
  output is eyeball-only; the shape cache and `b3World_Draw` dispatch it rides on
  are asserted with counting callbacks.
- **`box3d.Benchmark`** — a measurement tool, not a test; results vary by machine.

## Smoke commands

Still useful for eyeballing behavior and stressing at scale:

- `box3d.Smoke [N]` — raw debug-drawn bodies (M0 pipeline).
- `box3d.SmokeActors [N]` — full component pipeline with rendered meshes; logs
  settle state + query self-checks ~8 s after spawn when driven by
  `box3d.AutoSmokeActors N` (headless).
