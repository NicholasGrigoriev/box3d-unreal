# Baked static collision

The static scene mirror cooks level geometry at runtime from LOD0 render data
(`Box3DCooking.cpp` reads `UStaticMesh::GetRenderData()`). That works everywhere
in the editor and PIE, but **packaged builds strip CPU-side render buffers**
unless each mesh sets `bAllowCPUAccess` — so a shipped game can silently end up
with no static collision, which looks like a physics bug rather than a cooking
problem.

Baked collision solves this offline: a commandlet extracts exactly what the
mirror would cook — same component filter (`FBox3DStaticSceneMirror::ShouldMirror`),
same geometry choice (`MirrorGeometry` setting), same winding — into a
`UBox3DCollisionData` asset saved beside the map. At runtime the subsystem
instantiates those bodies directly: no cooking, no render-data dependency,
deterministic across runs.

Concept ported from [Antonio Lattanzio's Box3DUnreal](https://github.com/alattanzio/Box3DUnreal)
(MIT), adapted to this plugin's mirror pipeline.

## Baking

```
UnrealEditor-Cmd.exe <Project>.uproject -run=Box3DBake -Map=/Game/Maps/Arena
```

Multiple maps: `-Map=/Game/Maps/A,/Game/Maps/B` or bare package-name tokens.
Each map produces `BC_<MapName>` in the map's folder (the convention lives in
`UBox3DCollisionData::DeriveAssetPackageName`, shared by the bake and the
runtime lookup so the two can never disagree).

The bake honours the same Project Settings the mirror does — bake with the
settings you ship (`MirrorGeometry`, `bMirrorInstancedMeshes`,
`bMirrorQueryOnlyComponents`, `MinMirrorBoundsRadius`).

## Loading

Project Settings → Plugins → Box3D → **Baked Static Collision**:

| Setting | Meaning |
| --- | --- |
| `bUseBakedStaticCollision` | Master switch. On world begin-play, baked bodies are instantiated and the mirror's initial cook is skipped. |
| `BakedCollisionAssets` | Explicit assets to load, on top of auto-discovery. |
| `bAutoDiscoverBakedCollision` | Load `BC_<MapName>` beside the current map automatically (PIE prefix handled). On by default. |

Levels streamed in **after** begin-play still use the runtime mirror when
`bMirrorStaticGeometry` is enabled — baked assets cover the world as loaded at
bake time, streaming is the mirror's job.

Baked bodies behave exactly like mirror bodies: anonymous statics on the
`WorldStatic` channel with null userData, so queries report hits with a null
Component/Actor.

## Staleness

Each bake stores the box3d version and a source fingerprint (newest mtime +
total size + file count over the `.umap` **and** its `__ExternalActors__`
folder — under One File Per Actor, editing an actor never touches the `.umap`).
In editor/PIE, loading a stale bake logs a warning naming the reason; a
packaged build can't re-bake, so the check runs where it can still be acted on.
The fingerprint is a "something changed" signal, not a geometry hash: a false
positive costs a re-bake, a miss would ship wrong collision.

## Limitations

- World Partition maps bake only the editor-loaded cells (the commandlet warns).
  Prefer the runtime mirror for WP streaming content.
- ISM instances bake one body per instance with scale baked into the points, so
  tri-mesh baking of large ISM populations produces large assets — prefer
  `PreferSimpleCollision` for ISM-heavy maps.
- Landscape, BSP, and spline meshes are not baked (the mirror skips them too).
