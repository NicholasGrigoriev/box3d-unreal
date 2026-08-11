# Destruction system — investigation & proposed design

**Status: research only, nothing implemented.** Goal: a fast, good-looking
runtime destruction system for small static meshes — deterministic fracture
into Box3D-simulated fragments, structural stress collapse, and metal
bending/denting — with no Chaos Destruction / Geometry Collections, integrated
with this plugin.

Prompted by Antonio Lattanzio's (closed-source) UE5 destruction system
(AD METALLUM). His system is **not** available to study as code; this document
distills what he has said publicly, what the industry has published, and what
we already have, into a concrete proposed architecture.

---

## 1. What we already have (inventory)

The plugin covers a surprising amount of the runway:

| Existing piece | Role in a destruction system |
| --- | --- |
| `ABox3DBreakableActor` | The end-state pattern: chunk bodies auto-welded by bounds proximity, `BreakForce` threshold snaps welds via constraint-force polling, `OnWeldBroken` events. Today chunks are *authored*; fracture would *generate* them. |
| Weld joints (`Box3DJointComponent`, breakable, force+torque thresholds) | Bond layer between fragments during partial collapse. |
| `b3CreateHull(points, n, maxVerts)` | Fragment collision: Voronoi cells are convex polytopes → vertices go straight in. Deterministic hull build. |
| `b3CreateCompoundShape` | Multi-cell chunks (cluster several cells into one body) if body counts need capping. |
| `Box3DExplode`, contact/hit events with approach speed | Damage input: impulse application and impact detection already exist. |
| `FBox3DStaticSceneMirror::RemoveComponent` + `Box3D::ConvertToProp` | The "static world becomes dynamic" seam already exists for whole meshes — destruction extends it to *pieces* of meshes. |
| ProceduralMeshComponent path (`ABox3DClothActor` precedent) | Runtime fragment rendering: `CreateMeshSection` + per-frame transform sync is a solved problem in this codebase. |
| Deterministic fixed-step sim, recording/replay, `FSnapshotRing`/`ReconcileAndReplay` | Multiplayer story: deterministic debris + event replication + rollback primitives. |
| Sleep-by-default assemblies, `stat box3d`, benchmark harness | Performance discipline for thousand-body debris fields. |

Missing, in order of importance: **(A)** deterministic runtime fracture
(generate chunks from a mesh), **(B)** structural integrity layer (anchor
graph + stress), **(C)** metal deformation (denting + bending).

---

## 2. Intel: Lattanzio's system (AD METALLUM)

From his Reddit posts (r/UnrealEngine5: `1ru2kti`, `1ubnxir`, `1sd7xch`,
`1jsljoi`, `1vdhc3d`), two 80.lv articles, LinkedIn posts, and an NZGDC 2025
talk ("Unreal Engine: Beyond Chaos", youtube.com/watch?v=6pCBfNTe13g —
**unwatched, transcript unavailable; worth watching, likely answers open
questions**).

**Confirmed (his words):**
- Voro++ computes Voronoi fracture patterns *at runtime or baked*; he points
  out Voro++ already ships inside UE5 (Chaos itself uses it) — "a huge
  timesaver".
- True runtime geometry cutting (not chunk swaps), on any static *or skeletal*
  mesh, no pre-fracture authoring. He **rewrote UE's MeshBoolean repeatedly**
  because Geometry Script's booleans were too slow for repeated real-time
  cuts. Fracture results are cached and reused.
- Automatic anchor calculation from any static mesh; custom structural logic
  detects compromised supports; removing a beam triggers stress-based
  collapse; chain collapses planned.
- Metal deformation is "real-time tessellation plus mesh deformation,
  explicitly NOT World Position Offset" — real CPU-side vertex changes.
- Collision: prefractured assets use baked collision; runtime breakables use
  "a custom fast collision approach" (no standard UE cooking).
- 3-layer LOD-of-destruction: full sim → "Destruction Decal" (prefractured
  chunks converted to **Niagara particle bursts** driven by hit vector — no
  physics bodies at all) → presumably decals only. "Physics last, visuals
  first."
- Hybrid philosophy: *extends* Chaos-era UE rather than replacing everything;
  engine still handles some physics; his Box3DUnreal plugin later replaces
  debris/prop physics with deterministic Box3D.
- Multiplayer replication works (Lyra+GAS ability pipeline), mechanism
  unpublished; C++ multithreaded; 100–200 FPS on RTX 3090.

**Unknown:** boolean algorithm details, cap/interior UV handling, stress
algorithm, deformation collision refresh, replication payload. **Inferred:**
support-graph + heuristic stress (Family 1 below); deformation is vertex
displacement on his dynamic-mesh pipeline, replicated as impact events.

Takeaway: nothing he described is magic — it is the Blast architecture plus a
fast custom boolean, executed well. We can match the visible result with
simpler geometry (below) and our determinism story is *stronger* than his
Chaos-hybrid era.

---

## 3. Technique survey — what the industry publishes

### 3a. Fracture

- **NVIDIA Blast** (BSD-3, github.com/NVIDIA-Omniverse/PhysX `blast/`; older
  NVIDIAGameWorks/Blast) — the canonical open reference: authored chunk
  hierarchy; **support graph** with bonds (centroid, normal, area, health);
  damage shaders subtract bond health; island detection from anchored nodes;
  islands → new rigid actors; `NvBlastExtStress` solver. **Read this source
  first.**
- **Voronoi by half-space clipping**: a 3D Voronoi cell is the intersection of
  bisector half-spaces against nearby sites — Voro++'s own algorithm
  (math.lbl.gov/voro++). For 10–100 sites this is a ~200-line clipper,
  sub-millisecond, no library dependency, full control over float determinism
  (Voro++'s workspace is also not thread-safe without per-thread instances).
- **Fracture patterns ("stamps")**: precompute cell arrangements in a unit
  cube, transform to the impact point at runtime, clip against the target —
  Blast's authoring/runtime split; zero runtime Voronoi cost, still varied.
- **OpenFracture** (MIT, github.com/dgreenheck/OpenFracture) — readable
  reference for plane-cut + constrained-Delaunay caps + island detection.
- **UE Geometry Script / UDynamicMesh** booleans do run at game runtime but
  are synchronous game-thread and the wrong tool for shatter (Lattanzio's
  rewrite confirms).
- Caps on convex cell faces: planar convex polygons → **fan triangulation**
  is valid (no ear clipping needed); interior material via triplanar/box
  projection, no UV unwrap.
- R6 Siege (GDC 2016, "The Art of Destruction in Rainbow Six Siege"):
  deliberately constrained planar cutting for **deterministic replication**
  under competitive constraints — precedent for replicating cut inputs, not
  geometry.

### 3b. Structural integrity

Three families:
1. **Support-graph flood-fill** (Blast, Teardown): nodes = chunks, edges =
   bonds; a chunk is supported iff connected to an anchor; on removal,
   re-flood only from the removed chunk's neighbors (O(k) event-driven;
   union-find does not handle splits — use dirty BFS). Teardown is *pure*
   connectivity (overhangs float until touched) — proof this alone reads
   believably.
2. **Budgeted bond-stress relaxation** (`NvBlastExtStress`, open source):
   iterative solver *on the bond graph* (not the physics engine); per-bond
   load decomposed into tension/compression/shear vs thresholds in pascals;
   gravity + impulses as inputs; iteration count = cost/quality knob; supports
   graph coarsening. Implementable-from directly.
3. **FEM/modal methods** (Müller 2001, Parker & O'Brien 2009, "Breaking
   Good" fracture modes): fracture *pattern* quality, not runtime integrity;
   overkill here.

Red Faction: Guerrilla is Family 2-ish on beam networks (Game Developer
Oct 2010 archive) but detailed algorithms were never published.

### 3c. Metal deformation

Ranked for our use:
1. **RT dent maps** (render impacts into a per-component render target,
   sample as normal perturbation ± WPO): GPU-trivial, accumulates free,
   no collision change — perfect for bullet dents.
2. **CPU vertex displacement** (radius/falloff push along impact normal,
   clamp depth): UE5.7 tool of choice is `UDynamicMeshComponent` with
   `FastNotifyVerticesUpdated` partial buffer updates (PMC's
   `UpdateMeshSection` re-uploads whole sections). Deterministic if float
   order is fixed. This is almost certainly what Lattanzio does.
3. **Morph-target damage regions** (Saints Row 3, Game Developer Mar 2012):
   artist-authored dents blended per-region — near-zero runtime, trivially
   deterministic, high authoring cost.
4. **Plastic-hinge joints** for girders: elastic torque k·θ until yield
   torque, then permanent rest-angle update (the structural-engineering
   plastic hinge M_p = σ_y·Z_p) — maps directly onto our revolute joint +
   spline-mesh visual bend. No shipped-game citation found; engineering model
   is established.
5. **Node-beam cages** (BeamNG ~2 kHz solver): the real thing, too expensive;
   a 10–50 node cage with plastic yield skinning a render mesh is the cheap
   cousin for crumple props.

Shipped games almost universally **do not update collision after denting**;
refit primitives or swap at discrete damage states when it matters.

---

## 4. Proposed architecture (for discussion — not implemented)

Design principle: **fracture the proxy, not the render mesh.** For "simple
small static meshes", clip Voronoi cells against the mesh's *convex hull* (or
its authored convex pieces), not the render triangles. Every fragment is then
a convex polytope: `b3CreateHull` for physics, fan-triangulated faces for
render, triplanar interior material. No booleans, no caps problem, no CDT, no
Nanite conflict (original mesh is swapped out whole), and — crucially — no
dependency on CPU-accessible render data, so **it works in packaged builds**
(same constraint that motivated baked collision; fracturing render triangles
at runtime would reintroduce the `bAllowCPUAccess` trap). Non-convex props
decompose into a few authored convex pieces (or reuse simple collision
elements from `AggGeom` — already parsed by `Box3DCooking`).

### D1 — Deterministic convex fracture (`Box3D::Fracture`)
- Input: impact point/normal/energy + seed. Sites from seeded PRNG, density
  biased toward impact (radial cluster + sparse far cells — Blast-style
  pattern look). Quantize sites and plane distances to a grid before
  clipping (cross-platform float determinism; see GDC 2024 Pollard
  "Cross-Platform Determinism").
- Half-space clipper: cell = bounding box clipped by bisector planes of
  nearby sites, then by the proxy hull's planes. ~200 lines, per-fracture
  cost sub-ms for ≤64 cells. Optional later: precomputed pattern stamps.
- Output: `FBox3DFragment[] { hull points, volume, centroid, face list }`.
- Render: one `UProceduralMeshComponent` section per fragment (cloth-actor
  pattern), original `UStaticMeshComponent` hidden + mirror-removed
  (`RemoveComponent`) + Chaos collision off. Exterior faces inherit the outer
  material; interior faces get the authored "core" material, triplanar.
- Physics: `ABox3DFracturedActor` (evolution of `ABox3DBreakableActor`):
  fragment bodies from `b3CreateHull`, neighbors welded (cell adjacency is
  known from the clip — no bounds-proximity guess needed), `BreakForce` from
  shared-face area × material toughness. Below a volume threshold fragments
  skip physics entirely → Niagara burst (Lattanzio's "visuals first" tier).
- Replication: replicate `(mesh id, impact, seed, energy)` — both sides
  regenerate. Hash the fragment layout (djb2, same infra as
  `Box3DSnapshot.h`) with server fallback on mismatch.

### D2 — Structural integrity (`Box3DStructure`)
- Per destructible assembly: bond graph (nodes = chunks/fragments, bonds =
  shared-face area + health). Anchors: chunks whose bodies touch mirror/baked
  static geometry (auto-anchor via contact query — matches Lattanzio's
  "automatic anchors").
- Phase 1 connectivity: event-driven flood-fill from anchors on
  chunk-destroyed; unsupported islands promote static→dynamic and get welds
  (two-phase promotion — statics cost nothing until failure).
- Phase 2 stress: budgeted `ExtStress`-style relaxation on the graph
  (tension/compression/shear vs per-material thresholds), a few iterations
  per tick amortized; drives *bond* damage → creaking pre-failure, chain
  collapses. Feed gravity + `Box3DExplode` impulses.
- All state is plain data → replicable, snapshot-able (ring-compatible).

### D3 — Metal deformation (`Box3DDeform`)
- Tier 1 bullets: RT dent map (normal perturbation), pure cosmetic.
- Tier 2 hero dents: CPU vertex displacement with radius/falloff/max-depth,
  on the fragment PMC sections or a `UDynamicMeshComponent` for intact props.
  Collision untouched (established practice); optional hull refit at discrete
  damage states.
- Tier 3 girders: plastic-hinge — revolute joint with yield torque +
  rest-angle commit, spline-mesh visual bend, capsule-chain collision refit
  on settle. Bend-then-break: hinge break threshold hands over to D2.
- All deformation replicates as impact events (point, normal, magnitude).

### Suggested build order
D1 fracture (the visible payoff, exercises everything) → D2 connectivity
(flood-fill only, Teardown-style) → D3 tier 1+3 (cheap wins) → D2 stress →
D3 tier 2. Each stage lands with automation tests in the plugin's style
(fracture determinism: same seed → same hulls hash-exact; graph: island
promotion counts; hinge: yield-angle analytic).

### Open questions (decide before implementing)
1. Fragment render: PMC sections vs one `UDynamicMeshComponent`? (PMC matches
   the cloth precedent; DynamicMesh has better partial-update APIs but no
   instancing either.)
2. Where does the convex proxy come from when a mesh has no authored convex
   collision — `b3CreateHull` over render verts is editor/`bAllowCPUAccess`
   territory; likely answer: extend the **bake commandlet** to also bake
   fracture proxies (fits the existing `BC_` pipeline perfectly).
3. Fragment budget policy: cap per-event cells, global live-fragment pool,
   lifetime/fade rules (Lattanzio pools + tiers; we have sleeping + benchmark
   data to size this).
4. Skeletal meshes: out of scope (he supports them; we shouldn't for v1).
5. Watch the NZGDC talk before D2 — it may reveal his stress model.

---

## 5. Sources

- 80.lv: [No Chaos Involved](https://80.lv/articles/custom-real-time-destruction-system-for-unreal-engine-5-with-no-chaos-involved) · [structural damage follow-up](https://80.lv/articles/antonio-lattanzio-s-destruction-system-gets-satisfying-structural-damage) · [Box3D integration](https://80.lv/articles/try-this-box3d-integration-for-unreal-engine-5)
- Lattanzio Reddit: [1ru2kti](https://old.reddit.com/r/UnrealEngine5/comments/1ru2kti/) · [1ubnxir](https://old.reddit.com/r/UnrealEngine5/comments/1ubnxir/) · [1sd7xch](https://old.reddit.com/r/UnrealEngine5/comments/1sd7xch/) · [1vdhc3d](https://old.reddit.com/r/UnrealEngine5/comments/1vdhc3d/) · NZGDC talk: [youtube.com/watch?v=6pCBfNTe13g](https://www.youtube.com/watch?v=6pCBfNTe13g) (unwatched)
- NVIDIA Blast: [SDK docs](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html) · [ExtStress](https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_stress.html) · [source](https://github.com/NVIDIAGameWorks/Blast) (BSD-3)
- Voro++: [math.lbl.gov/voro++](https://math.lbl.gov/voro++/) (modified BSD) · [multithreaded extension](https://arxiv.org/abs/2209.11606)
- OpenFracture (MIT): [github.com/dgreenheck/OpenFracture](https://github.com/dgreenheck/OpenFracture)
- R6 Siege GDC 2016: [slides](https://media.gdcvault.com/gdc2016/Presentations/LHeureux_Julien_Art_Of_Destruction.pdf)
- Teardown / Gustafsson: [blog.voxagon.se](https://blog.voxagon.se/) (esp. "Cracking destruction", design notes, multiplayer post)
- Red Faction: Guerrilla: [Game Developer Oct 2010](https://media.gdcvault.com/GD_Mag_Archives/GDM_October_2010.pdf)
- Müller, [Real Time Dynamic Fracture (SG2013)](https://matthias-research.github.io/pages/publications/fractureSG2013.pdf) · [Breaking Good fracture modes](https://www.dgp.toronto.edu/projects/breaking-good/)
- Saints Row deformation: [Game Developer Mar 2012](https://media.gdcvault.com/GD_Mag_Archives/GDM_March_2012.pdf)
- BeamNG: [technical whitepaper](https://beamng.tech/blog/2021-06-21-beamng-tech-whitepaper/bng_technical_paper.pdf)
- Determinism: [GDC 2024 Cross-Platform Determinism (Pollard)](https://media.gdcvault.com/gdc2024/Slides/GDC%2Bslide%2Bpresentations/Pollard_Bradley_CrossPlatformDeterminism%2B2024-03-26%2B09.34.19.pdf)
- Fan triangulation validity: [Geometric Tools, ear clipping](https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf)
