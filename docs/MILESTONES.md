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

## M4 — Joints & events ✅ 2026-07-06

Goal: constraints and gameplay-visible collision events.

- [x] Joint components: distance, revolute, prismatic, spherical, weld, motor, wheel
      (`UBox3DJointComponent` base: own transform = joint frame, ConnectedActor =
      side A or implicit static world anchor; subsystem retries joints whose
      bodies appear later)
- [x] Limits, motors, springs exposed as UPROPERTYs (cm/degrees at the editor
      face, newtons/N·m for forces — box3d native)
- [x] Contact begin/end + hit events → dynamic multicast delegates on body
      components (`bEnableContactEvents`/`bEnableHitEvents`; pump runs after
      every fixed step so per-step event buffers are never dropped)
- [x] Sensor shapes → overlap-style events (`bIsSensor` on the body; visitors
      opt out via `bDetectableBySensors`)
- [x] Joint break events (box3d reports force/torque threshold exceedance;
      `bBreakable` joints destroy themselves and broadcast `OnJointBroke`)
- [ ] *(deferred)* Parallel and filter joints (box3d extras outside the classic
      seven; thin wrappers when a use case shows up)

Verified by 10 automation tests (33 total suite green): rope hangs at exact
length with constraint force = m·g and breaks past its threshold; revolute
pins the radius and its motor reaches 90 deg/s on the hinge axis; prismatic
stops on its lower limit with rotation locked; spherical holds the socket
radius; weld keeps relative pose under impulse; motor joint reaches target
velocity; wheel sags to the analytic spring equilibrium g/(2πf)² while the
spin motor holds the axle axis; contact begin/end, hit approach speed √(2gh),
and sensor pass-through all fire and resolve components.

## M5 — Threading & performance ✅ 2026-07-06

Goal: production-grade throughput.

- [x] `enqueueTask`/`finishTask` hooked into UE tasks (`FBox3DUETaskPool`: fixed
      256-slot handle array per box3d's stable-pointer recommendation, atomic slot
      counter because the solve orchestrator enqueues from worker threads,
      `FTask::Wait` for the join — the fork/join contract is safe because the step
      blocks on the game thread, which doubles as box3d's worker 0, and UE workers
      retract-or-help instead of deadlocking)
- [x] Scheduler choice: `UBox3DSettings::TaskSystem` (UnrealTasks default — shares
      engine workers, no extra threads | Box3DInternal — dedicated threads) with
      `WorkerCount` (default 1 = fully serial)
- [x] `stat box3d` — per-step profile times + body/shape/contact/joint/island/
      task/memory counters; `GetWorldStats()` exposes the same data to
      Blueprint and tests without the stats system
- [x] Debug draw via `b3World_Draw`: persistent wireframe cache behind the
      createDebugShape/destroyDebugShape world callbacks (registered on every
      world, lazily populated so it costs nothing until used), DrawDebugHelpers
      output, `box3d.DebugDraw` master CVar + Joints/Bounds/Contacts/Mass/
      Islands/Distance sub-CVars
- [x] `box3d.Benchmark [bodies] [steps] [workers]` — identical 5k-body pile through
      serial / internal / UE-tasks configs in standalone physics worlds
- [ ] *(deferred)* Chaos comparison — needs editor content in a host project;
      the plugin-side command measures box3d only
- [ ] *(deferred)* Render interpolation between fixed steps (carried from M1 notes)

Measured (5000 bodies, 180 steps, ~57.6k contacts, Win64, 16 physical cores):
serial 25.9 ms avg/step; box3d internal ×16 4.45 ms (5.8×); UE tasks ×16 5.12 ms
(5.1×, 8.8k tasks enqueued) — with identical final contact counts across all three
schedulers. Verified by 3 automation tests (36 total green): a 180-body pile lands
within 1 mm of the serial result under UE-tasks ×4 and internal ×4 with tasks
proven to reach UE workers; world stats report measured step/solve times and exact
counters; the debug-shape cache builds one wireframe per shape with exact point
counts, reuses them on redraw, and frees them on shape destruction.

## M6 — Polish & advanced ✅ 2026-07-06

- [x] Recording/replay with built-in determinism validation: subsystem
      `StartRecording`/`StopRecording`/`SaveRecordingToFile`/`ValidateLastRecording`
      (box3d embeds per-step state hashes; validation replays in a scratch world
      and compares them), console commands `box3d.RecordStart` /
      `box3d.RecordStop [file]` (auto-validates + saves to `Saved/Box3D/`) /
      `box3d.ValidateReplay <path>`
- [x] `BOX3D_DOUBLE_PRECISION` build switch: single const in
      `Box3DCore.Build.cs`, propagated to all dependents via PublicDefinitions
      so the ABI always agrees; the conversion seam carries UE's LWC doubles
      through `b3Pos` losslessly in that mode — the full 40-test suite was run
      green under both ABIs (default remains single precision)
- [x] Explosion helper: `UBox3DQueryLibrary::Box3DExplode` (radius + falloff,
      impulse per facing area, mask filtering, immediate velocity change)
- [x] CCD verified and documented: world continuous collision stops 150 m/s
      bodies against thin statics out of the box; `bIsBullet` extends it to
      dynamic-vs-dynamic (guidance in DESIGN.md)
- [x] Documentation pass + `UPSTREAM.md` (vendored-version bump workflow with
      the integration contracts to re-check)
- [ ] *(deferred)* Sample content — the smoke commands (`box3d.Smoke`,
      `box3d.SmokeActors`, `box3d.DebugDraw 1`) double as live samples; packaged
      sample maps need a content project

Verified by 4 automation tests (40 total green): a recorded session (seed
snapshot + mid-recording spawns) replays with every embedded state hash
matching, survives a file roundtrip, and — replayed at 4 workers from a serial
recording — proves cross-thread determinism hash-exactly; explosions match
dv = 3·I/(4·r·ρ) analytically with falloff, range, and mask filtering exact;
continuous collision stops 150 m/s movers per the CCD matrix above.

---

## Post-M6: ports from alattanzio/Box3DUnreal (2026-08)

Evaluated Antonio Lattanzio's independent Box3DUnreal integration (MIT) and
ported the three capabilities it had that we lacked:

- [x] **Baked static collision** — `Box3DBake` commandlet + `UBox3DCollisionData`
      assets (`BC_<MapName>` beside the map, auto-discovered). Same filter and
      geometry as the static mirror; packaged builds no longer depend on
      CPU-accessible render data. OFPA-aware stale-bake fingerprinting warns in
      editor/PIE. New `Box3DEditor` module. (docs/BAKED_COLLISION.md)
- [x] **Authority gating** — `bAuthorityOnlySimulation`: pure clients get no
      Box3D world; `IsSimulationAuthority()` on the subsystem.
- [x] **Snapshot / prediction / rollback** — `Box3DSnapshot.h`: per-body
      capture/restore/djb2-hash, whole-world `FSnapshotRing`,
      `ReconcileAndReplay` with sub-tolerance skip. Contact-free replay is
      bit-identical (test-pinned); contact replay diverges by warm-start design
      — documented. (docs/NETWORKING.md)

Verified by 2 new automation tests (67 total): ring capture/evict/restore
roundtrips hash-exactly; reconcile no-ops on agreement, replays bit-identically
on forced rollback, and carries a 1 m authoritative correction through replay.

---

## Destruction ✅ 2026-08-12 ([DESTRUCTION.md](DESTRUCTION.md))

- [x] **D1 — deterministic fracture core** ✅ 2026-08-11 — `Box3D::Fracture`:
      seeded quantized Voronoi sites (impact-biased via `RadialBias`),
      half-space clipper with canonical per-pair bisector planes,
      `MinFragmentVolume` merging, `FractureLayoutHash`,
      `box3d.FractureDebug`; 9 automation tests (76 total).

- [x] **D2 — fractured actor: rendering + physics** ✅ 2026-08-11 —
      `Box3D::FractureMesh` + `ABox3DFracturedActor`: proxy resolution
      (authored convex → simple collision → render-vert fallback), two
      flat-shaded PMC sections per fragment (source / `CoreMaterial`),
      swap-out seam, per-fragment `b3CreateHull` bodies, cell-adjacency
      welds with `BreakForce = SharedFaceArea × MaterialToughness`,
      `OnWeldBroken`, body-driven section sync, `box3d.Fracture`;
      6 automation tests (82 total).

- [x] **D3 — damage pipeline + tiers + budgets** ✅ 2026-08-12 — impact
      intake (hit-event energy ½mv² via mirror shapes of
      `UBox3DDestructibleComponent`-marked meshes, `Box3DExplode` blast
      falloff), monotonic clamped `FBox3DEnergyToCellCurve` (`MaxCellCount`
      doubles as the per-event cell cap), volume-threshold tier routing
      (Body / Debris / Dust) with debris burst arrays + optional Niagara
      `DebrisSystem` hookup, fragment pool (`MaxLiveFragments`) with
      oldest-first eviction, per-tick fracture budget
      (`FractureTimeBudgetMs`) with cross-tick impact queueing and a public
      `QueueDestructibleImpact` API, `box3d.DestructionStress`;
      6 automation tests (88 total).

- [x] **D4 — structural connectivity (Teardown tier)** ✅ 2026-08-12 —
      `Box3D::Structure`: per-assembly bond graph (nodes = chunks, bonds =
      shared-face area + health, canonical pair order), auto-anchors via
      static-overlap query (`DetectAnchors`, own bodies excluded),
      event-driven flood-fill on chunk-destroyed / bond-broken with
      early-out on anchors or already-proven regions (visit-count bound).
      Structural mode on `ABox3DFracturedActor` (`bStructural`): chunks
      spawn static, weld breaks feed `NotifyBondBroken`,
      `DestroyFragment` feeds `NotifyChunkDestroyed`, unsupported islands
      promote static→dynamic two-phase (flip whole batch, then wake) with
      intra-island welds kept, under `MaxPromotionsPerTick` (FIFO
      overflow); no-anchor assemblies fall back to dynamic rubble.
      `box3d.SpawnStructure` + `box3d.DestroyChunk`;
      7 automation tests (95 total).

- [x] **D5 — stress relaxation (creaks and chain collapses)** ✅ 2026-08-12 —
      budgeted deterministic `FBox3DStressSolver` integrated into structural
      actor ticks with live iteration/coarsening settings, fragment mass and
      configured-world-gravity inputs, event-driven `Box3DExplode` impulses,
      per-bond tension/compression/shear resolution and UE-unit→Pa conversion.
      Per-material Pa capacities and sustained-overload erosion live on
      `UBox3DDestructibleComponent`; `OnStructureStressed` fires before failure,
      and zero-health bonds destroy their matching weld then flow through D4
      connectivity, FIFO promotion, and `OnWeldBroken`. Graph coarsening retains
      canonical ordering and area-weighted fine-bond loads.
      6 stress automation tests (101 total): analytic cantilever, coarsening,
      correct-first overload break with actor integration, explosion event intake,
      600-step stable gravity/no-damage, and deterministic erosion/break hash progression.

- [x] **D6 — metal deformation** ✅ 2026-08-12 — three independent tiers:
      deterministic fixed-order vertex denting on fracture PMC sections with
      radius/falloff/depth clamp and collision left untouched; asset-free
      per-mesh R8 dent-map ping-pong with shared material parameters and
      `box3d.DentTest`; and `UBox3DPlasticHingeComponent`, an ideal
      elastic-perfectly-plastic two-body revolute girder hinge. Hinge response
      commits the analytic excess beyond `YieldTorque / ElasticStiffness` to
      its permanent spring rest angle, updates an optional spline-mesh visual,
      and hands angle failure through the existing exactly-once
      `OnJointBroke` path. 7 automation tests (108 total), plus the required
      dent-map rendered-eyeball path verified offscreen.

- [x] **D7 — multiplayer, polish, and docs** ✅ 2026-08-12 — transport-neutral
      `FBox3DDestructionEvent` tuple `(MeshId, Impact, Seed, Params)`; authority
      gates on every fracture decision path; no-world clients regenerate the
      identical procedural visual set with no Box3D state; layout and initial
      bond-health hashes validate deterministic event expansion; mismatch
      delegates expose the game-side authoritative correction seam, including
      `Box3D::ReconcileAndReplay` for predicted fragment bodies. The complete
      setup, RPC/GAS integration, tier/budget, structure/stress/deformation,
      command, and troubleshooting guide lives in [DESTRUCTION.md](DESTRUCTION.md).
      2 replication automation tests (110 total): matching layout/bond hashes
      after every event in two independent worlds, mismatch correction coverage,
      and a visual-only client path with procedural sections but zero bodies.

---

## Non-goals (for now)

- Replacing Chaos wholesale (UE collision channels, physics assets, ragdolls stay Chaos)
- Networked physics *transport* (what to send, when — the snapshot/rollback
  primitives are here, the wire protocol is game code)
- Platforms beyond Win64 (nothing blocks Linux/Mac — just untested here)
