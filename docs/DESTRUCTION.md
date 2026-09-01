# Box3D Unreal — Destruction

Box3DUnreal's destruction stack turns an opt-in static mesh into deterministic
Voronoi fragments, welded rigid bodies, budgeted debris, load-bearing structures,
and cosmetic or joint-driven metal deformation. Public inputs and geometry stay
in Unreal centimetres; the existing Box3D conversion seam converts fragment hulls
to metres when bodies are created.

The same source proxy, impact, seed, and numeric policy produce the same
`FractureLayoutHash`. That is the basis of the multiplayer contract: replicate a
small event, regenerate locally, validate the hash, and request authoritative
state only when validation fails.

## Quick setup

1. Add `UBox3DDestructibleComponent` to an actor whose root is a
   `UStaticMeshComponent`, or which has a static mesh component the marker can
   find. A component fractures once; after the intact mesh swaps out, its
   fragments and welds carry subsequent damage.
2. Give meshes intended to fracture in packaged builds authored convex or simple
   collision. `Box3D::ResolveFractureProxy` prefers authored convex, then simple
   collision, then an editor-only LOD0 render-vertex fallback. Cooked builds
   normally strip the fallback's CPU vertex data.
3. Configure the marker's energy curve, seed, material, fragment, tier, and
   structural properties. `CoreMaterial` is used on cut faces; when unset, the
   source material is reused.
4. For automatic rigid-body hit intake, enable the runtime static scene mirror.
   The destructible marker registers with `UBox3DWorldSubsystem`; mirror-body hit
   events are converted to impact energy and queued. Baked static bodies have no
   component identity and do not provide this intake path, so call `ApplyImpact`
   or `QueueDestructibleImpact` for destructibles represented only by a bake.
5. If the assembly should be structural, enable `bStructural` and make sure its
   chunks touch mirrored, baked, or other Box3D static bodies so anchor detection
   can find support.

The main designer-facing settings are under **Project Settings ▸ Plugins ▸
Box3D**. See [BAKED_COLLISION.md](BAKED_COLLISION.md) for packaged static scenes
and [NETWORKING.md](NETWORKING.md) for snapshot/prediction primitives.

## Fracture and damage contracts

### Deterministic fracture

`Box3D::Fracture::Fracture` is a pure convex Voronoi fracture function. Its
`FFractureParams` contain:

- `Seed` and `CellCount`;
- world/local impact position as documented by the caller;
- `ImpactRadius` and `RadialBias` for impact-biased site placement;
- `MinFragmentVolume`, which deterministically merges undersized cells into the
  neighbour with the largest shared face;
- `FlattenAxis`, which snaps every site to the proxy's centre on one local axis so
  thin slabs break into full-thickness prisms rather than layered flakes
  (`INDEX_NONE` keeps free 3D sites and the previous layouts/hashes).
- `Sites`, explicit proxy-local sites that replace seeded generation (a jittered
  grid for cladding, for instance) on the same deterministic clipper.

Sites, bisector planes, and layout-hash inputs are quantized to `0.01 cm`, and
iteration/pair order is fixed. Each `FBox3DFragmentData` contains convex geometry,
volume, centroid, faces, and symmetric shared-face adjacency. Use
`FractureLayoutHash` to compare layouts, not pointer identity or actor order.

`Box3D::FractureConvexProxy` fractures a bare convex proxy at a given pose with no
source component (callers that render their own cladding). `Box3D::FractureMesh` is the authority-only actor handoff. It resolves the source
proxy, creates an `ABox3DFracturedActor`, bakes the fragments into its render
meshes (see Rendering below), hides the intact source mesh, disables its Chaos
collision, and removes its static-mirror body. Body-tier fragments receive Box3D hulls; adjacent cells
receive welds with:

```
BreakForce = SharedFaceAreaCm2 * MaterialToughnessNPerCm2
```

`MaterialToughness <= 0` makes welds unbreakable. Sleeping assemblies report zero
joint force, so a physical weld will not overload until its fragments wake.

`HullInsetCm` pulls every fragment's physics hull a little toward its centroid
(rendering unchanged). Exact Voronoi hulls touch face-to-face and can start
sub-millimetre penetrated; the resulting push-out friction is enough to pin a
fragment detached from a static assembly between its neighbours. A millimetre
or two of inset removes the squeeze.

### Energy intake

`FBox3DEnergyToCellCurve` maps joules to cell count:

- below `MinEnergy`: absorbed, no fracture;
- at `MinEnergy`: `MinCellCount`;
- from `MinEnergy` to `FullEnergy`: monotonic square-root easing;
- at or above `FullEnergy`: `MaxCellCount`.

`MaxCellCount` is also the per-event geometry cap. Automatic hit intake computes
kinetic energy as `0.5 * MassKg * ApproachSpeedMPerS^2`, using the other body's
mass and the hit approach speed. `Box3DExplode` supplies blast energy with radial
falloff and also feeds structural stress impulses. For explicit damage:

```cpp
Destructible->ApplyImpact(WorldImpactPoint, EnergyJoules);
```

This authority-only call builds and applies the event immediately, then broadcasts
`OnDestructionEventGenerated`. To spread work under the global fracture budget,
use `UBox3DWorldSubsystem::QueueDestructibleImpact`; the FIFO queue is drained at
end of tick.

### Event payload

`FBox3DDestructionEvent` is the complete deterministic decision tuple:

```
(MeshId, Impact, Seed, Params)
```

- `MeshId` is the stable static-mesh asset path. It is **not** an actor or
  component identity; game replication must identify the target separately.
- `Impact` is world-space UE centimetres.
- `Seed` controls deterministic site generation.
- `Params` carries every numeric policy that can change geometry, tiers, bodies,
  welds, or structural health: cell count, impact bias/radius, merge threshold,
  toughness, density, sleep state, tier thresholds, debris speed, structural
  mode, stress capacities, and overload erosion rate.

Presentation assets (`CoreMaterial` and `DebrisSystem`) remain local on the target
component. `BuildDestructionEvent` creates the tuple without applying it; it
returns false for absorbed damage, a missing/already-fractured target, or a
non-authority context.

## Multiplayer integration

The plugin deliberately owns no RPC or Gameplay Ability System transport. Game
code wires the transport-neutral seam:

1. On authority, identify the target actor/component through the game's normal
   replicated identity.
2. Call `ApplyImpact`, or call `BuildDestructionEvent` followed by
   `ApplyDestructionEvent(Event, 0, Result)` when transport timing is managed
   manually.
3. Send the target identity, `FBox3DDestructionEvent`, and the authority's
   `Result.FractureLayoutHash` through an RPC, GAS gameplay event, or another
   reliable ordered channel. `ApplyImpact` also exposes the event/hash through
   `OnDestructionEventGenerated`.
4. On each receiver, resolve the target marker and call:

```cpp
FBox3DDestructionEventResult Result;
ABox3DFracturedActor* Fractured = Destructible->ApplyDestructionEvent(
    Event, AuthorityLayoutHash, Result);
```

`ApplyDestructionEvent` rejects a target whose local mesh asset path differs from
`MeshId`. On simulation authority it creates Box3D bodies, welds, and structural
state. On non-authority or no-world clients it uses
`Box3D::RegenerateFractureVisuals`: identical render geometry (every fragment in
the attached mesh) and tier data, but no Box3D bodies, welds, structure solver,
or fragment-pool entry.

Set `bAuthorityOnlySimulation` when pure clients should have no Box3D world.
Standalone, listen-server, and dedicated-server worlds remain authorities;
`UBox3DWorldSubsystem::IsSimulationAuthority()` is the decision gate. Local calls
to `ApplyImpact`, `BuildDestructionEvent`, direct `FractureMesh`, and queued
impacts are all refused outside authority, while received events still regenerate
visuals.

### Validation and correction

Passing a positive authority layout hash enables validation. The result reports:

- `bApplied` and `bVisualOnly`;
- `FractureLayoutHash` and initial `BondHealthHash`;
- `bLayoutHashValidated`;
- `bNeedsServerCorrection` when the local and authority layout hashes differ.

A mismatch still builds the received event's local presentation, then broadcasts
`OnDestructionCorrectionRequired(MeshId, AuthorityHash, LocalHash)`. Game code
must request and apply authoritative fragment/body state; the plugin does not
invent a wire format or actor identity for that fallback.

For predicted fragment bodies, capture a stable, consistently ordered body set in
`Box3D::FSnapshotRing`, request authoritative `FBodyState` values and frame
numbers, then use `Box3D::ReconcileAndReplay`. It restores the whole interacting
body set at the authority frame, overwrites the corrected subset, and replays to
the present. Important limits:

- the ring assumes a stable body set and order across its retained window;
- all interacting bodies must be restored, even if only some receive authority
  corrections;
- snapshot state excludes solver warm-start/contact history, so contact-rich
  rollback is not guaranteed bit-identical to uninterrupted simulation;
- `PositionTolerance` is in Box3D metres (`0.02` by default, or 2 cm).

## Rendering

`ABox3DFracturedActor` draws with static meshes built at runtime
(`UStaticMesh::BuildFromMeshDescriptions`, fast path — the same call packaged
builds use) and never rewrites vertices per frame:

- **Attached** fragments (static bodies — anchored cladding, an unbroken
  structural assembly — and every fragment on the visual-only client path) are
  baked into ONE mesh on the `AttachedFragments` component: section 0 = exterior
  faces with the source material, section 1 = interior cut faces with
  `CoreMaterial`. However many pieces a settled ruin has, it is one
  static-relevance primitive with cached draw commands.
- **Loose** fragments (dynamic bodies: `DetachFragment`, structural promotion,
  plain rubble) each get a pooled `UStaticMeshComponent` carrying that fragment's
  body-local geometry; `SyncFragments` (Tick) moves awake ones with
  `SetWorldLocationAndRotation` — a GPU-scene transform update, no buffer churn.
  `GetFragmentRenderState` / `GetFragmentComponent` / `GetAttachedMesh` expose
  the split; `GetRenderPrimitiveCount` is attached mesh + loose chips.
- Render changes coalesce: `DetachFragment`, `DestroyFragment` and promotions only
  mark the actor dirty; the next `SyncFragments` (or a next-tick timer for a
  non-ticking actor, or an explicit `FlushRenderState`) moves fragments between
  the two forms and rebuilds the attached mesh once. `GetRenderBuildCount` counts
  mesh builds; a settled actor must not grow it from Tick
  (`FracturedActor.RenderBatching` asserts all of this).
- `FBox3DFractureMeshParams::bCastShadow / bAffectIndirectLighting /
  bReceivesDecals` reach every fragment primitive.
- `ApplyVertexDent` edits the stored body-local geometry and rebuilds only the
  meshes it touched.

Why: a procedural mesh section is dynamic relevance — a mesh batch rebuilt per
pass per frame plus a one-frame uniform buffer — and `UpdateMeshSection` is a
staging upload (plus a BLAS rebuild under ray tracing) per chip per frame. With
~140 cracked cladding cells that was 2,840 batches and 31 ms looking at the wall
vs 11 ms looking away (ArcShooter `docs/destructible-skin.md`, profiling notes).

## Fragment tiers and budgets

`FBox3DTierThresholds` classifies each fragment by volume in cm³:

| Tier | Rule | Runtime result |
| --- | --- | --- |
| Dust | below `RenderVolumeThreshold` | dropped completely |
| Debris | not Dust, below `PhysicsVolumeThreshold` | not drawn, no body/weld; added to burst arrays |
| Body | otherwise | drawn, hull body, and eligible welds |

Dust is tested first and therefore wins when thresholds overlap. Zero thresholds
keep every fragment in Body tier.

When `DebrisSystem` is assigned, the actor spawns it with world-space Niagara user
arrays `DebrisPositions`, `DebrisVelocities`, and `DebrisSizes`; sizes are
cube-edge lengths in cm. A null system means sub-physics fragments simply vanish,
although `GetDebrisBurst()` remains available to C++.

Global budgets are read from `UBox3DSettings`:

| Setting | Default | Behaviour |
| --- | ---: | --- |
| `MaxLiveFragments` | 256 | Body-tier pool cap; oldest fractured actors are evicted first. `0` is unlimited; the newest fracture always survives. |
| `FractureTimeBudgetMs` | 2 ms | FIFO queued-impact drain budget. At least one impact runs per tick; `0` drains the whole queue. |
| `MaxPromotionsPerTick` | 64 | Structural static→dynamic chunk promotions per tick, FIFO overflow. `0` is unlimited. |
| `StressRelaxationIterationsPerTick` | 8 | Fixed-order stress passes per structural actor tick. |
| `StressCoarsenNodeThreshold` | 256 | Deterministic stress-graph reduction above this target. `0` disables coarsening. |

## Structures, stress, and deformation

### Connectivity and collapse

With `bStructural`, fragments start as static bodies. A canonical bond graph uses
fragments as nodes and shared faces as bonds. `DetectAnchors` marks chunks whose
inflated AABBs overlap Box3D static bodies; this is deliberately a conservative
broad-phase support test. If no anchors are found, the actor falls back to normal
dynamic rubble and `IsStructureActive()` is false.

Set `bAnchorAllFragments` (params, event params, destructible component) to make
every fragment with a body an anchor instead of running `DetectAnchors`: cladding
glued to an immovable surface then never depends on what the b3 world holds
behind it. Such an assembly only loses pieces through `DetachFragment` — which
destroys the fragment's welds, drops it from the graph, flips it dynamic and
launches it with the given velocities — or `DestroyFragment`, which also removes
detached chips. `FindFragmentAtPoint` / `FindNearestFragment` locate the piece
under a hit; `IsFragmentAttached` / `CountAttachedFragments` report what is still
held by the structure.

`DestroyFragment` and broken welds update connectivity immediately. A fixed-order
flood fill finds islands with no live path to an anchor. Unsupported chunks queue
for two-phase static→dynamic promotion: the batch changes body type first, then
wakes, preserving weld consistency. `OnWeldBroken` reports one or more snapped
welds; `GetPendingPromotionCount` exposes backlog.

### Stress relaxation

Structural actors solve gravity and queued impulses against bond areas without
polling joint forces. Tension, compression, and shear capacities are in pascals;
a non-positive capacity disables that damage mode. Loads above capacity broadcast
`OnStructureStressed(BondIndex, OverloadRatio, BondHealth)`, then erode health by
`SustainedOverloadHealthPerSecond`. Zero-health bonds break their physical weld
and enter the same connectivity/promotion path.

Large graphs can coarsen deterministically. This lowers solve cost but makes local
per-bond stress approximate. Use `GetStructureStressHash()` when a deterministic
bond-health/load digest is needed.

### Metal deformation tiers

The system provides three independent levels rather than rebuilding collision on
every dent:

1. `ABox3DFracturedActor::ApplyVertexDent` permanently moves procedural-mesh
   vertices in fixed order. Radius, falloff, and per-stamp depth are deterministic;
   Box3D hull collision deliberately stays unchanged.
2. `UBox3DDentMapComponent` accumulates R8 UV-space height stamps through
   ping-pong render targets. Call `InitializeDentMap`, then `StampDentUV`. Shared
   materials receive `Box3D_DentMap`, `Box3D_DentNormalStrength`, and
   `Box3D_DentMapTexelSize`; the project material reconstructs normals and may
   apply WPO. This tier is cosmetic and collision-free.
3. `UBox3DPlasticHingeComponent` extends a two-body revolute joint with an ideal
   elastic-perfectly-plastic response. Beyond `YieldTorque / ElasticStiffness`,
   excess angle becomes permanent spring rest angle. `BreakAngle` routes failure
   through the joint's existing exactly-once `OnJointBroke`; `0` disables angle
   breakage. `VisualSpline` is optional and projects assign their own girder mesh.

## Demo and diagnostic commands

These non-shipping commands consolidate the destruction visual checks:

| Command | Purpose |
| --- | --- |
| `box3d.FractureDebug [CellCount=12] [Seed=42] [RadialBias=0.5] [MinVolume=0]` | Draw a deterministic 1 m cube fracture and log its layout hash. |
| `box3d.Fracture [CellCount=12] [Seed=42] [Toughness=50]` | Fracture the static mesh under the crosshair into welded bodies. |
| `box3d.DestructionStress [Count=16]` | Queue many cube impacts to eyeball the fracture-time budget and fragment-pool eviction. |
| `box3d.SpawnStructure [CellCount=16] [Seed=42] [Toughness=50]` | Spawn a structural wall and cantilever; mirrored or baked static ground is needed for anchors. |
| `box3d.DestroyChunk` | Destroy the fractured chunk under the crosshair and exercise island promotion. |
| `box3d.DentTest [Resolution=256] [StampCount=12] [SurfaceMaterialPath]` | Spawn a dent-map panel. Without a material path it previews the height map directly. |

Useful companions are `box3d.DebugDraw 1`, `stat box3d`, and
`box3d.RecordStart` / `box3d.RecordStop` / `box3d.ValidateReplay <path>`.

## Troubleshooting

- **Fracture returns null:** check simulation authority, that the component has
  not already fractured, that energy reaches `MinEnergy`, and that the target is
  a static mesh with a resolvable convex/simple proxy.
- **Works in editor, not packaged:** author simple collision. The render-vertex
  fallback depends on LOD0 CPU data normally stripped by cooking.
- **Received event is rejected:** the target's mesh asset path must equal
  `Event.MeshId`, and target actor identity must be replicated separately.
- **Client has no fragments:** ensure the event reaches the correct marker and
  call `ApplyDestructionEvent`; do not call authority-only `ApplyImpact` on a pure
  client.
- **Layout correction fires:** verify event fields are transmitted losslessly and
  in order, source mesh/collision assets match, and no peer substitutes local
  component policy for `Event.Params`.
- **Structure immediately becomes rubble:** no anchors were detected. Enable
  mirrored or baked static collision, place chunks within the 2 cm anchor margin,
  and inspect `IsStructureActive()`.
- **Welds never break:** sleeping bodies report zero weld force; wake or impact
  the assembly. Also check `MaterialToughness` is positive and not excessive.
- **Collapse is delayed:** `MaxPromotionsPerTick` intentionally spreads large
  failures across ticks; inspect `GetPendingPromotionCount()`.
- **Debris is invisible:** assign a `DebrisSystem` that reads the three documented
  Niagara arrays, or lower the tier thresholds so fragments remain Body tier.
- **Dent map is flat:** the default command material only previews height. A
  project surface material must consume the `Box3D_Dent*` parameter contract to
  reconstruct normals/WPO.

The complete deterministic suite is documented in [TESTING.md](TESTING.md). Run
`Automation RunTests Box3DUnreal`; the current suite contains 110 tests, including
43 dedicated fracture, destruction, structure, stress, and deformation tests.
