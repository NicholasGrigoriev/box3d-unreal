# Destruction system — implementation plan

Executable plan for the system designed in
[DESTRUCTION_RESEARCH.md](DESTRUCTION_RESEARCH.md) (read that first: intel,
technique survey, and the *fracture-the-convex-proxy* decision live there).
Branch: `feature/destruction`. Style contract: every milestone lands with
automation tests in the plugin's house style (frame-exact, analytic
assertions, `Automation RunTests Box3DUnreal`), `///` docs, and a console
command for eyeball verification in PIE.

Core decisions (from the research, restated as commitments):

- **Fragments are convex.** Voronoi cells clipped against a convex proxy —
  never boolean-cut render triangles. `b3CreateHull` for physics, fan
  triangulation for render, triplanar interior material.
- **Determinism is a feature.** Fracture is a pure function of
  `(proxy, impact, seed, params)`: quantized sites and plane offsets, stable
  ordering, no transcendentals in the clipper. Same inputs → hash-identical
  fragments on every machine.
- **Physics last, visuals first.** Tiered response: hull bodies for big
  fragments, render-only/Niagara for shrapnel, budgets everywhere.
- **Packaged builds are first-class.** Proxies come from authored simple
  collision or the `BC_`-style bake pipeline — no `bAllowCPUAccess`
  dependency at runtime.

---

## D1 — Deterministic fracture core (geometry only, no UE objects)

New: `Box3DFracture.h/.cpp` (`Box3D::Fracture` namespace). Pure geometry —
no actors, no rendering, testable headless.

Deliverables:
- `FFractureParams { Seed, CellCount, ImpactPoint, ImpactRadius, RadialBias, MinFragmentVolume }`.
- Site generation: seeded PRNG (`FRandomStream` or explicit PCG), density
  biased toward impact, sites + bisector-plane offsets quantized to a fixed
  grid before clipping.
- Half-space clipper: cell = proxy hull polytope clipped by bisector planes
  of neighboring sites. Convex polytope representation with face lists and
  stable vertex ordering.
- Output `FBox3DFragmentData { Vertices, Faces, Volume, Centroid, Neighbors (shared-face area per neighbor) }`.
- `FractureLayoutHash()` — djb2 over quantized fragment data (reuses the
  `Box3DSnapshot.h` hashing pattern).

Tests (`Box3DFractureTests.cpp`):
- Same seed + params → `FractureLayoutHash` **bit-identical** across two runs
  and across worker-thread counts.
- Different seeds → different hashes.
- Volume conservation: Σ fragment volumes ≈ proxy volume (< 0.5 % error).
- Every fragment is a valid hull (`b3CreateHull` succeeds, ≥ 4 verts).
- Adjacency symmetric (A lists B ⇔ B lists A) and shared-face areas match.
- Cell count respects `CellCount` and `MinFragmentVolume` merging.

Eyeball: `box3d.FractureDebug` — fracture a unit cube at the crosshair,
debug-draw the cells.

## D2 — Fractured actor: rendering + physics handoff

New: `ABox3DFracturedActor` + `Box3D::FractureMesh()` entry point.
Evolves the `ABox3DBreakableActor` pattern (welds, break force, events).

Deliverables:
- Proxy resolution: authored convex elements from `AggGeom` (via the
  `Box3DCooking` parse) → else convex hull of simple collision → else
  editor-only render-vert hull with a logged warning (packaged builds
  require the first two; doc the constraint).
- Fragment rendering: one ProceduralMeshComponent per fractured actor,
  one section per fragment (cloth-actor precedent), exterior faces carry the
  source material, interior faces a configurable `CoreMaterial` (triplanar).
- Fragment physics: `b3CreateHull` bodies; neighbors welded using **cell
  adjacency** (not bounds proximity); `BreakForce = SharedFaceArea × MaterialToughness`.
- Swap-out seam: hide source `UStaticMeshComponent`, disable its Chaos
  collision, `FBox3DStaticSceneMirror::RemoveComponent` if mirrored.
- `Box3D::FractureMesh(StaticMeshComponent, Params)` → spawned actor.

Tests (`Box3DFracturedActorTests.cpp`):
- Fragment/weld counts match D1 layout for a known seed.
- Welded assembly at rest: chunks asleep, zero drift over 60 steps.
- Impulse above `BreakForce` snaps exactly the loaded welds
  (`OnWeldBroken` fires); assembly separates into expected island count.
- Mirror body removed; Chaos query-collision off on the source component.

Eyeball: `box3d.Fracture [cells]` — fracture the static mesh under the
crosshair.

## D3 — Damage pipeline + tiers + budgets

Deliverables:
- Impact intake: hit events (approach speed × mass → energy) and
  `Box3DExplode` integration; energy → `CellCount` mapping curve.
- Tier policy: fragments below `PhysicsVolumeThreshold` skip bodies —
  handed to a Niagara system as a burst (position/velocity/size arrays);
  fragments below `RenderVolumeThreshold` don't exist at all.
- Global budget: live-fragment pool with cap, oldest-fragment fade/despawn,
  per-event cell cap, per-tick fracture budget (ms) with event queueing.
- `UBox3DDestructibleComponent`: opt-in marker configuring
  material toughness, tier thresholds, core material — the designer-facing
  surface.

Tests:
- Energy mapping monotonic and clamped; tier thresholds route fragments to
  the correct tier (counts assertable without Niagara: tier decision is
  data).
- Pool cap enforced under repeated fracturing; oldest evicted first.
- Queued events processed within budget across ticks (frame-exact).

Eyeball: `box3d.DestructionStress N` — N fracture events in one second,
`stat box3d` stays sane.

## D4 — Structural connectivity (Teardown tier)

New: `Box3DStructure.h/.cpp` — bond graph per destructible assembly.

Deliverables:
- Graph build: nodes = chunks (authored breakable chunks or D1 fragments),
  bonds = shared-face area + health; auto-anchors = chunks whose bodies
  contact mirror/baked static geometry (overlap query at build).
- Event-driven connectivity: on chunk destroyed/bond broken, flood-fill
  only from the affected neighborhood; islands with no anchor path →
  promote static→dynamic (two-phase promotion), weld intra-island bonds,
  let physics take over.
- Promotion budget: max promotions per tick, FIFO overflow.

Tests (`Box3DStructureTests.cpp`):
- Bridge scenario (two anchored piers + span): destroy one pier chunk →
  exactly the unsupported island promotes; the anchored side stays static.
- Tower scenario: destroy base → whole column promotes in one event.
- Anchor auto-detection: chunk resting on mirrored ground is anchored;
  floating chunk is not.
- Flood-fill touches only the dirty region (visit counter bound).

Eyeball: `box3d.SpawnStructure` — anchored wall+beam assembly; shoot the
beam.

## D5 — Stress relaxation (creaks and chain collapses)

Deliverables:
- Budgeted `ExtStress`-style solver on the bond graph: per-bond load
  decomposed into tension/compression (along bond normal) and shear;
  gravity + explosion impulses as inputs; N relaxation iterations per tick
  (configurable), graph coarsening above a node-count threshold.
- Bond damage from sustained overload → feeds D4 connectivity → chain
  collapse; `OnStructureStressed` event (creak audio/VFX hook).
- Per-material thresholds (pascals) on `UBox3DDestructibleComponent`.

Tests:
- Cantilever analytic: uniform beam of k chunks anchored at one end — root
  bond moment ≈ Σ mᵢ·g·dᵢ within tolerance; solver converges within budget.
- Overload breaks the analytically-correct bond first.
- Stable structure under gravity: zero bond damage after 600 steps.
- Determinism: same structure + impulses → identical bond-health hash.

## D6 — Metal deformation

Deliverables (three independent tiers):
- **Dent maps**: per-component RT stamp on impact → normal perturbation in
  a shared material function; ping-pong RT helper; pure cosmetic.
- **Vertex denting**: radius/falloff/max-depth displacement on fragment
  PMC sections (and optionally a DynamicMeshComponent path for intact hero
  props); deterministic (fixed iteration order); collision untouched.
- **Plastic hinges**: girder = two bodies + revolute joint with yield
  torque; past yield, commit rest-angle (permanent bend), spline-mesh
  visual follows joint angle; past break angle → hand over to D4/weld
  break. Bend-then-break.

Tests:
- Hinge: applied torque below yield → returns to rest; above yield →
  rest-angle advances by the analytic plastic increment; break angle fires
  the joint-break event.
- Vertex denting: same impact → identical displaced-vertex hash; depth
  clamp respected.
- (Dent maps are render-only — eyeball via `box3d.DentTest`.)

## D7 — Multiplayer + polish + docs

Deliverables:
- Replication contract: destruction events as `(MeshId, Impact, Seed, Params)`
  through an RPC/GAS hook (game-side), layout-hash validation with
  server-authoritative fallback (fragment correction piggybacks on
  `Box3DSnapshot`/`ReconcileAndReplay` where bodies are predicted).
- Authority integration: fracture decisions only where
  `IsSimulationAuthority()`; clients regenerate visuals from events.
- `docs/DESTRUCTION.md` user guide; README/MILESTONES/TESTING updates;
  demo commands consolidated.

Tests:
- Two independent worlds fed the same event stream → identical
  `FractureLayoutHash` and bond-health hashes per event.
- Client (no b3 world, authority-gated) can still build the visual fragment
  set from an event (render path has no physics dependency).

---

## Sequencing & estimates

D1 → D2 → D3 → D4 → D5 → D6 → D7. D1+D2 give the visible payoff and
de-risk everything (geometry + handoff). D3 makes it shippable. D4 before
D5 (connectivity alone already reads believably — Teardown). D6 is
independent of D4/D5 and can interleave. D7 last, on proven determinism.

Rough effort: D1 ~2 sessions, D2 ~2, D3 ~1.5, D4 ~1.5, D5 ~2, D6 ~2,
D7 ~1.5. Suite target: ~90 tests total by D7.

Before D1 starts: watch the NZGDC talk (research doc §2) and settle open
questions §4 — PMC-vs-DynamicMesh (plan assumes PMC) and proxy baking
(plan assumes AggGeom-first, bake extension deferred until needed).
