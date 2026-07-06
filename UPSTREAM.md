# Vendored box3d

| | |
| --- | --- |
| Upstream | https://github.com/erincatto/box3d |
| Version | **v0.1.0** |
| License | MIT (`Source/Box3DCore/box3d.LICENSE`) |
| Local modifications | **None** — upstream files are byte-identical |

## Layout mapping

| Upstream | Vendored |
| --- | --- |
| `include/box3d/*.h` | `Source/Box3DCore/Public/box3d/*.h` |
| `src/*.c`, `src/*.h` | `Source/Box3DCore/Private/*` |
| `LICENSE` | `Source/Box3DCore/box3d.LICENSE` |

Everything else (CMake, samples, docs, tests) is intentionally not vendored —
UnrealBuildTool compiles the sources directly (`Box3DCore.Build.cs`: C17, no
PCH/unity, exports via box3d's own `box3d_EXPORTS`/`BOX3D_DLL` scheme).

## Bumping to a new upstream version

1. Fetch the upstream tag and diff it against the vendored copy first — the
   integration relies on several observed contracts, so look specifically for
   changes to:
   - the task callbacks (`b3EnqueueTaskCallback` / `b3FinishTaskCallback`) and
     `B3_MAX_TASKS` / `B3_MAX_WORKERS` (`FBox3DUETaskPool` sizes its slot array
     from these),
   - `b3WorldDef` / `b3BodyDef` / `b3ShapeDef` / joint def layouts (all mirrored
     by `UBox3D*Component` UPROPERTYs),
   - event structs (`b3ContactEvents` / `b3SensorEvents` / `b3JointEvents`) and
     their per-step buffering,
   - debug draw (`b3DebugShape`, `b3DebugDraw`) and the recording API.
2. Replace `Public/box3d/` and `Private/` wholesale per the mapping above (do
   not merge — the copy carries no local edits, keep it that way; integration
   quirks belong in `Box3DRuntime`, never in upstream files).
3. Update the version in this file and in `README.md`.
4. Verify:
   - build (`Build.bat FPS_TESTEditor Win64 Development -project=...`),
   - the automation suite is green (`Automation RunTests Box3DUnreal`,
     see `docs/TESTING.md` for the headless command),
   - build with `bDoublePrecision = true` in `Box3DCore.Build.cs` and run the
     suite under the large-world ABI too (it was 40/40 green at M6), then flip
     it back,
   - `box3d.Benchmark` numbers have not regressed unreasonably,
   - `box3d.Smoke` / `box3d.SmokeActors` eyeball in PIE with
     `box3d.DebugDraw 1`.
5. Note behavioral changes worth surfacing in `docs/MILESTONES.md`.
