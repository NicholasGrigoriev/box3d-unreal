# Box3D Unreal

Unreal Engine 5.7 plugin integrating [Box3D](https://github.com/erincatto/box3d) —
Erin Catto's 3D rigid body physics engine (the 3D sibling of Box2D v3).

Box3D v0.1.0 is vendored under `Source/Box3DCore` and compiled from source by
UnrealBuildTool; the plugin is fully standalone with no external build steps.

## Status

**Beta / M6 complete — feature roadmap done.** The library compiles inside UE, a
physics world is stepped per game world at a fixed timestep, `UBox3DBodyComponent`
gives actors Box3D rigid bodies with transform sync in both directions — primitive
shapes (box/sphere/capsule with auto-fit), cooked convex hulls, exact triangle
meshes, and `CollisionAsset` (one shape per authored collision element), plus
collision filtering and physical materials. `UBox3DQueryLibrary` exposes ray/shape
casts, overlaps, explosions, and character-mover collide-and-slide helpers to
Blueprint. Seven joint component types
(distance/revolute/prismatic/spherical/weld/motor/wheel) with limits, motors,
springs, and breakage, plus contact/hit/sensor events as Blueprint delegates.
Multithreaded stepping rides UE's task system (or box3d's internal scheduler) —
5.1× on a 16-core 5k-body pile, deterministic across worker counts — with
`stat box3d`, `box3d.DebugDraw` wireframes, and `box3d.Benchmark` for tuning.
Recording/replay with hash-based determinism validation (`box3d.RecordStart/Stop`,
`box3d.ValidateReplay`), continuous collision for fast movers, and an optional
`BOX3D_DOUBLE_PRECISION` large-world build. Landscape height fields are deferred.

Post-M6 world integration: a **static scene mirror** (settings-gated) that cooks
every qualifying static mesh component — ISM/HISM instances included — into raw
Box3D static bodies at level load and follows level streaming / World Partition,
so dynamic bodies rest on existing maps with zero per-actor setup;
`ABox3DPropActor` plus `Box3D::ConvertToProp` / `box3d.MakeProp` to turn any
placed static mesh (or ISM instance) into a live physics prop whose mesh keeps
Chaos query collision for unchanged gameplay traces; `AddImpulseAtLocation` /
`AddForceAtLocation` on the body component; and optional render interpolation
between fixed steps (`bInterpolateBodyTransforms`) for high-refresh displays.
Soft-body actors built from raw bodies behind purely visual skins:
`ABox3DRopeActor` (capsule chain + spherical joints, rendered as spline meshes;
pin the far end, `AttachActorToEnd` at runtime, or snap it via `LinkBreakForce`
/ `CutLink`) and `ABox3DClothActor` (sphere-particle lattice + distance-joint
stitching, rendered as a double-sided procedural mesh with per-frame normals,
material hot-swappable) — both interpolate between fixed steps, preview their
rest shape in the editor, and spawn at the crosshair via `box3d.SpawnRope` /
`box3d.SpawnCloth`. Plus a special-actor zoo: `ABox3DBreakableActor` (child
meshes become welded chunks that shatter past a break force —
`box3d.SpawnBreakable`), `ABox3DConveyorActor` (box3d surface-material tangent
velocity drags resting bodies), and `ABox3DWindActor` (per-fixed-step drag
forces via the subsystem's `OnPreStep` hook; directional, turbulence,
spline-following, or vortex fields — `box3d.SpawnWind`).
All of it is pinned by a deterministic automation suite — 55 tests
(`Automation RunTests Box3DUnreal`, see [docs/TESTING.md](docs/TESTING.md)).
See [docs/MILESTONES.md](docs/MILESTONES.md) for the roadmap,
[docs/DESIGN.md](docs/DESIGN.md) for architecture decisions, and
[UPSTREAM.md](UPSTREAM.md) for the box3d version-bump workflow.

## Install

Drop the `Box3DUnreal` folder into your project's `Plugins/` directory and rebuild.
Requires UE 5.7, C++ project.

## Tests

40 automation tests cover conversion math, world stepping, body components, queries,
mesh cooking, joints, gameplay events, threading, diagnostics, replay determinism,
explosions, and continuous collision with analytic assertions
([docs/TESTING.md](docs/TESTING.md)):

```
Automation RunTests Box3DUnreal
```

## Quick test

Run in PIE, open the console (`` ` ``) and enter:

```
box3d.Smoke
```

A stack of debug-drawn falling boxes and spheres simulated by Box3D appears in front
of the camera. `box3d.Smoke 200` spawns 200 bodies.

For the full component pipeline with rendered meshes (real actors whose root is a
`UBox3DBodyComponent`):

```
box3d.SmokeActors
```

Either command with `0` clears its bodies.

Also useful: `box3d.DebugDraw 1` (wireframes of every physics shape, plus
`box3d.DebugDraw.Contacts/Bounds/Mass/...`), `stat box3d` (step profile and world
counters), `box3d.Benchmark` (scheduler comparison for sizing
`WorkerCount`/`TaskSystem` in Project Settings), and `box3d.RecordStart` /
`box3d.RecordStop` / `box3d.ValidateReplay <file>` (hash-validated record & replay).

## Modules

| Module | Purpose |
| --- | --- |
| `Box3DCore` | Vendored box3d v0.1.0 (C17, unmodified upstream source — see [UPSTREAM.md](UPSTREAM.md)) |
| `Box3DRuntime` | UE integration: world subsystem, unit conversion, settings, hooks |

## Conventions

- Units: box3d works in meters; conversion is a flat ×0.01/×100 at the boundary
- Axes: UE's Z-up coordinates are passed straight through (box3d has no up-axis convention)
- One `b3WorldId` per `UWorld`, owned by `UBox3DWorldSubsystem`

## Licenses

- This plugin: MIT ([LICENSE](LICENSE))
- Box3D: MIT, © Erin Catto ([Source/Box3DCore/box3d.LICENSE](Source/Box3DCore/box3d.LICENSE))
