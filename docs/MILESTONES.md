# Box3D Unreal — Milestones

Tracking document for bringing [Box3D](https://github.com/erincatto/box3d) (v0.1.0, MIT)
into Unreal Engine 5.7 as a standalone plugin.

Status legend: `[ ]` planned · `[~]` in progress · `[x]` done

---

## M0 — Foundation (compiles, world steps)

Goal: the plugin builds inside UE 5.7 and can create, step, and destroy a Box3D world.

- [x] Analyze box3d v0.1.0 source, API, and build requirements
- [~] Vendor box3d `include/` + `src/` into `Box3DCore` module, compiled by UBT as C17
- [~] `Box3DRuntime` module: allocator hook (`FMemory`), assert hook (`ensure`/log), log hook (`UE_LOG`)
- [~] `UBox3DWorldSubsystem`: world lifecycle + fixed-timestep stepping with accumulator
- [~] `UBox3DSettings` developer settings (gravity, substeps, hertz, worker count)
- [~] Smoke test: `box3d.Smoke` console command — falling bodies over a static ground,
      visualized with debug draw, verifying the full create/step/read-back loop
- [~] Private GitHub repo, clean history

## M1 — Rigid bodies as components

Goal: place Box3D-simulated actors in a level without writing C++.

- [ ] `UBox3DBodyComponent` (scene component): body type, mass/damping/gravity scale,
      motion locks, bullet flag; creates body on register, destroys on unregister
- [ ] Shape setup from simple primitives: box, sphere, capsule (explicit extents + auto-fit
      from owner's collision or bounds)
- [ ] Transform sync: box3d → UE for dynamic bodies (post-step), UE → box3d for
      kinematic/static (`b3Body_SetTargetTransform` for kinematics)
- [ ] Velocity/force/impulse Blueprint API on the component
- [ ] Sleep state, enable/disable, body events (`b3World_GetBodyEvents`) driving
      component transform updates only for moved bodies
- [ ] Per-body smoke map/level test in FPS_TEST

## M2 — Queries, filtering, materials

Goal: gameplay code can ask questions of the physics world.

- [ ] Blueprint function library: raycast (closest + multi), shape cast, overlap tests
- [ ] `b3QueryFilter` / `b3Filter` exposure — collision channels mapped to category/mask bits
- [ ] Physical material mapping (friction, restitution, user material IDs)
- [ ] Character mover experiments: `b3World_CastMover` / `b3World_CollideMover` for the
      FPS character (this is the fun one for an FPS project)

## M3 — Complex collision

Goal: real level geometry collides correctly.

- [ ] Convex hull shapes cooked from Static Mesh collision (b3HullData from UBodySetup convex elems)
- [ ] Triangle mesh shapes from Static Mesh render/complex collision data (`b3CreateMesh`)
- [ ] Height field shapes from Landscape (`b3CreateHeightField`)
- [ ] Compound shapes (`b3CreateCompoundShape`) for multi-primitive bodies
- [ ] Asset lifetime strategy: cache cooked b3MeshData/b3HeightFieldData, share across bodies

## M4 — Joints & events

Goal: constraints and gameplay-visible collision events.

- [ ] Joint components: distance, revolute, prismatic, spherical, weld, motor, wheel
- [ ] Limits, motors, springs exposed as UPROPERTYs
- [ ] Contact begin/end + hit events → dynamic multicast delegates on body components
- [ ] Sensor shapes → overlap-style events
- [ ] Joint break events

## M5 — Threading & performance

Goal: production-grade throughput.

- [ ] Hook `enqueueTask`/`finishTask` into UE task system (respecting the fork/join
      blocking contract — dedicated step thread or careful game-thread stepping)
- [ ] Alternative: benchmark box3d's internal scheduler vs UE tasks, pick default
- [ ] `stat box3d` — counters/profile from `b3World_GetProfile` / `b3World_GetCounters`
- [ ] Debug draw via `b3World_Draw` + debug shape callbacks (persistent shape cache)
- [ ] Benchmark map: 5k+ bodies, compare against Chaos equivalent for fun

## M6 — Polish & advanced

- [ ] Determinism validation (`b3Hash`-based), recording/replay tools (`b3Recording`)
- [ ] Optional `BOX3D_DOUBLE_PRECISION` build switch for large worlds (ABI-affecting —
      needs a Build.cs flag + matching define in both modules)
- [ ] CCD/bullet configuration guidance, explosion helper (`b3World_Explode`)
- [ ] Sample content, documentation pass, upstream version bump workflow (UPSTREAM.md)

---

## Non-goals (for now)

- Replacing Chaos wholesale (UE collision channels, physics assets, ragdolls stay Chaos)
- Networked physics replication
- Platforms beyond Win64 (nothing blocks Linux/Mac — just untested here)
