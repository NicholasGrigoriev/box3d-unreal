# Box3D Unreal

Unreal Engine 5.7 plugin integrating [Box3D](https://github.com/erincatto/box3d) —
Erin Catto's 3D rigid body physics engine (the 3D sibling of Box2D v3).

Box3D v0.1.0 is vendored under `Source/Box3DCore` and compiled from source by
UnrealBuildTool; the plugin is fully standalone with no external build steps.

## Status

**Beta / M6 and deterministic destruction complete.** The library compiles inside UE, a
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
`AddForceAtLocation` on the body component; `UBox3DGrabComponent` (physics
hands: mass-gated grab/carry/throw with a velocity-tracking grip — held bodies
stay dynamic, sag when heavy, throw at impulse/mass — and it doubles as the
winch for pull-the-light-thing-to-me grapples); and optional render
interpolation between fixed steps (`bInterpolateBodyTransforms`) for
high-refresh displays.
Soft-body actors built from raw bodies behind purely visual skins:
`ABox3DRopeActor` (capsule chain + spherical joints, rendered as spline meshes;
pin the far end, `AttachActorToEnd`/`AttachBodyToEnd` at runtime, or snap it via
`LinkBreakForce` / `CutLink`; winch support for grapples — `SetDeployedLength`
spools links in and out, `GetEndConstraintForce` reads the end-joint tension,
`BuildDirection` lays the chain toward its holder, and `bCollideWithPawns`
keeps it off the character carrying it) and `ABox3DClothActor` (sphere-particle lattice + distance-joint
stitching, rendered as a double-sided procedural mesh with per-frame normals,
material hot-swappable) — both interpolate between fixed steps, preview their
rest shape in the editor, and spawn at the crosshair via `box3d.SpawnRope` /
`box3d.SpawnCloth`. Plus a special-actor zoo: `ABox3DBreakableActor` (child
meshes become welded chunks that shatter past a break force —
`box3d.SpawnBreakable`), `ABox3DConveyorActor` (box3d surface-material tangent
velocity drags resting bodies), and `ABox3DWindActor` (per-fixed-step drag
forces via the subsystem's `OnPreStep` hook; directional, turbulence,
spline-following, or vortex fields — `box3d.SpawnWind`).
Post-M6 ports from [Antonio Lattanzio's Box3DUnreal](https://github.com/alattanzio/Box3DUnreal)
(MIT): **baked static collision** — a `Box3DBake` commandlet extracts exactly
what the static mirror would cook into `BC_<MapName>` assets, so packaged
builds instantiate static geometry without runtime cooking or CPU-accessible
render data ([docs/BAKED_COLLISION.md](docs/BAKED_COLLISION.md)); **authority
gating** (`bAuthorityOnlySimulation` — pure clients get no Box3D world); and
**snapshot / prediction / rollback** primitives — per-body capture/restore/hash,
a whole-world `FSnapshotRing`, and `ReconcileAndReplay` for client-side
prediction against an authoritative server
([docs/NETWORKING.md](docs/NETWORKING.md)).

The destruction stack adds seeded, quantized Voronoi fracture; welded rendered
fragments; energy-driven Body/Debris/Dust tiers and global budgets; anchored
connectivity with stress-driven chain collapse; vertex dents, UV dent maps, and
plastic girder hinges. Its transport-neutral multiplayer contract replicates
`(MeshId, Impact, Seed, Params)`, validates layout hashes, regenerates visuals on
no-physics clients, and exposes a server-correction seam
([docs/DESTRUCTION.md](docs/DESTRUCTION.md)).

All of it is pinned by a deterministic automation suite — 110 tests
(`Automation RunTests Box3DUnreal`, see [docs/TESTING.md](docs/TESTING.md)).
See [docs/MILESTONES.md](docs/MILESTONES.md) for the roadmap,
[docs/DESIGN.md](docs/DESIGN.md) for architecture decisions, and
[UPSTREAM.md](UPSTREAM.md) for the box3d version-bump workflow.

## Install

Drop the `Box3DUnreal` folder into your project's `Plugins/` directory and rebuild.
Requires UE 5.7, C++ project.

## Tests

110 automation tests cover conversion math, world stepping, body components, queries,
mesh cooking, joints, gameplay events, threading, diagnostics, replay determinism,
explosions, continuous collision, snapshot/rollback, deterministic fracture,
destruction replication, structures, stress, and deformation with analytic assertions
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

For destruction, start with `box3d.Fracture` or `box3d.FractureDebug`; the
complete setup, multiplayer flow, budgets, structural demos, deformation commands,
and troubleshooting guide is [docs/DESTRUCTION.md](docs/DESTRUCTION.md).

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
| `Box3DEditor` | Editor tooling: `Box3DBake` commandlet for baked static collision |

## Conventions

- Units: box3d works in meters; conversion is a flat ×0.01/×100 at the boundary
- Axes: UE's Z-up coordinates are passed straight through (box3d has no up-axis convention)
- One `b3WorldId` per `UWorld`, owned by `UBox3DWorldSubsystem`

## Contributing

Open source under MIT — bug reports, suggestions, and pull requests are
welcome; see [CONTRIBUTING.md](CONTRIBUTING.md).

## Licenses & credits

- This plugin: MIT ([LICENSE](LICENSE))
- Box3D: MIT, © Erin Catto ([Source/Box3DCore/box3d.LICENSE](Source/Box3DCore/box3d.LICENSE))
- Baked collision and snapshot/rollback concepts ported from
  [Box3DUnreal](https://github.com/alattanzio/Box3DUnreal) by Antonio Lattanzio (MIT)
