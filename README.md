# Box3D Unreal

Unreal Engine 5.7 plugin integrating [Box3D](https://github.com/erincatto/box3d) —
Erin Catto's 3D rigid body physics engine (the 3D sibling of Box2D v3).

Box3D v0.1.0 is vendored under `Source/Box3DCore` and compiled from source by
UnrealBuildTool; the plugin is fully standalone with no external build steps.

## Status

**Alpha / M1 complete.** The library compiles inside UE, a physics world is stepped
per game world at a fixed timestep, and `UBox3DBodyComponent` gives actors Box3D
rigid bodies (box/sphere/capsule, auto-fit shapes, Blueprint force/velocity API)
with transform sync in both directions. Next up: M2 — queries and filtering. See
[docs/MILESTONES.md](docs/MILESTONES.md) for the roadmap and
[docs/DESIGN.md](docs/DESIGN.md) for architecture decisions.

## Install

Drop the `Box3DUnreal` folder into your project's `Plugins/` directory and rebuild.
Requires UE 5.7, C++ project.

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

## Modules

| Module | Purpose |
| --- | --- |
| `Box3DCore` | Vendored box3d v0.1.0 (C17, unmodified upstream source) |
| `Box3DRuntime` | UE integration: world subsystem, unit conversion, settings, hooks |

## Conventions

- Units: box3d works in meters; conversion is a flat ×0.01/×100 at the boundary
- Axes: UE's Z-up coordinates are passed straight through (box3d has no up-axis convention)
- One `b3WorldId` per `UWorld`, owned by `UBox3DWorldSubsystem`

## Licenses

- This plugin: MIT ([LICENSE](LICENSE))
- Box3D: MIT, © Erin Catto ([Source/Box3DCore/box3d.LICENSE](Source/Box3DCore/box3d.LICENSE))
