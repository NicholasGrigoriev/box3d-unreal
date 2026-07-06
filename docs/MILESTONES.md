# Box3D Unreal — Milestones

Tracking document for bringing [Box3D](https://github.com/erincatto/box3d) (v0.1.0, MIT)
into Unreal Engine 5.7 as a standalone plugin.

Status legend: `[ ]` planned · `[~]` in progress · `[x]` done

---

## M0 — Foundation (compiles, world steps) ✅ 2026-07-06

Goal: the plugin builds inside UE 5.7 and can create, step, and destroy a Box3D world.

- [x] Analyze box3d v0.1.0 source, API, and build requirements
- [x] Vendor box3d `include/` + `src/` into `Box3DCore` module, compiled by UBT as C17
- [x] `Box3DRuntime` module: allocator hook (`FMemory`), assert hook (`ensure`/log), log hook (`UE_LOG`)
- [x] `UBox3DWorldSubsystem`: world lifecycle + fixed-timestep stepping with accumulator
- [x] `UBox3DSettings` developer settings (gravity, substeps, hertz, worker count)
- [x] Smoke test: `box3d.Smoke` console command — falling bodies over a static ground,
      visualized with debug draw, verifying the full create/step/read-back loop
- [x] Private GitHub repo, clean history

Verified headless (`-game -nullrhi`): world created per game world, transient world
destroyed cleanly, `Box3D stepping: 60 fixed steps done` heartbeat in log. Visual
`box3d.Smoke` check in PIE still worth an eyeball.

## M1 — Rigid bodies as components ✅ 2026-07-06

Goal: place Box3D-simulated actors in a level without writing C++.

- [x] `UBox3DBodyComponent` (scene component): body type, damping, gravity scale,
      motion locks, bullet flag; creates body on BeginPlay, destroys on EndPlay
- [x] Shape setup from simple primitives: box, sphere, capsule (explicit extents + auto-fit
      from the nearest attached primitive's bounds)
- [x] Transform sync: box3d → UE for dynamic bodies (post-step), UE → box3d for
      kinematic/static (`b3Body_SetTargetTransform` targets pushed before each fixed step)
- [x] Velocity/force/impulse Blueprint API on the component
- [x] Sleep state, enable/disable, body events (`b3World_GetBodyEvents`) driving
      component transform updates only for moved bodies
- [x] Smoke test: `box3d.SmokeActors` spawns real actors (body component root + rendered
      mesh); `box3d.AutoSmokeActors` CVar runs it headless

Verified headless: 12 actors dropped 2 m, settled at exactly half-extent above the
static slab (Z=97 on a slab top of 72 with 25 cm half extents), masses analytically
correct (cube 125 kg, sphere 65.4 kg — auto-fit + density both right), all asleep
after 8 s. Not covered yet: render interpolation between fixed steps (M5 territory),
runtime shape/material changes.

## M2 — Queries, filtering, materials ✅ 2026-07-06

Goal: gameplay code can ask questions of the physics world.

- [x] Blueprint function library: raycast (closest + multi), sphere/capsule casts,
      sphere overlap tests
- [x] `b3QueryFilter` / `b3Filter` exposure — `EBox3DChannel` bit indices with editor
      bitmask UI, widened to box3d's 64-bit masks
- [x] Physical material mapping (`UPhysicalMaterial` override, user material IDs
      surfaced in hit results)
- [x] Character mover helpers: `Box3DCastMover` (safe translation fraction) and
      `Box3DSolveMoverDelta` (`b3World_CollideMover` planes → `b3SolvePlanes`
      collide-and-slide) exposed to Blueprint

Verified headless against the settled smoke scene: ray hit a settled body's top face
at exactly Z=122 (center 97 + 25 half-extent), mover cast fraction 0.48 vs 0.475
analytic, plane solver pushed an overlapping capsule up (+4.5 cm) instead of allowing
a -50 cm move. Follow-up for a real FPS character: sweep-then-slide movement component
built on these helpers (gameplay-side, or an M2.5 sample).

## M3 — Complex collision ✅ (core) 2026-07-06

Goal: real level geometry collides correctly.

- [x] Convex hull shapes cooked from Static Mesh collision (convex elems, render-vertex
      fallback simplified to 64 verts)
- [x] Triangle mesh shapes from Static Mesh LOD0 render data (`b3CreateMesh`, welded,
      edge-identified; triangle winding flipped at the cook boundary — UE winds CW)
- [x] `CollisionAsset` shape type: one Box3D shape per authored collision element
      (sphere/capsule/box/convex) — bodies support multiple shapes natively, and this
      works on dynamic bodies where `b3CreateCompoundShape` (static-only) would not
- [x] Asset lifetime strategy: cooked mesh/hull data cached per asset (`FObjectKey`),
      owned by the module (mesh shapes hold references; cache flushed at shutdown)
- [ ] *(deferred)* Height field shapes from Landscape (`b3CreateHeightField`) — no
      landscape in the test project yet; needs the Landscape module dependency
- [ ] *(deferred)* `b3CreateCompoundShape` for very large static multi-primitive
      bodies — per-element shapes cover the common cases

Verified headless, all four paths in one run: triangle-mesh ground (non-uniform scale
20×20×0.5) ray-hit at exact cooked height, CollisionAsset cube (authored box element)
at analytic mass/height, sphere primitive, and convex-hull cylinder (97.5 kg vs 98.2
analytic for a 64-vert hull).

## M3.5 — Test coverage pass ✅ 2026-07-06

Goal: every M0–M3 behavior pinned by a deterministic automation test, not just the
smoke run.

- [x] Automation test suite (`Automation RunTests Box3DUnreal`): 23 tests across
      conversion math, world stepping, bodies, queries, and cooking — synthetic game
      worlds stepped manually, analytic expected values throughout
      (see [TESTING.md](TESTING.md) for the coverage matrix)
- [x] Bug found & fixed: `bWantsOnUpdateTransform` was never set, so component moves
      (static follows, dynamic/kinematic teleports) never reached the physics body
- [x] Bug found & fixed: hull/sphere/capsule hits reported `TriangleIndex 0` instead
      of the documented `-1` (box3d uses 0 for non-mesh shapes; now normalized via
      `b3Shape_GetType`)

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
