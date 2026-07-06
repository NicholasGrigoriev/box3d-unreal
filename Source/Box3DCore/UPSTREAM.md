# Vendored Box3D

- Upstream: https://github.com/erincatto/box3d
- Version: **v0.1.0**
- Commit: `8441b4a06d6d09dcfb0b0f704df4d847d1437b92`
- License: MIT (see `box3d.LICENSE`)

## Layout mapping

| Upstream | Here |
| --- | --- |
| `include/box3d/*.h` | `Public/box3d/` |
| `src/*.c`, `src/*.h`, `src/*.inl`, `src/box3d.natvis` | `Private/` |
| `LICENSE` | `box3d.LICENSE` |

Not vendored: `samples/`, `test/`, `benchmark/`, `docs/`, `extern/`, CMake files.

## Rules

- **Do not edit vendored files.** Compile-time configuration goes through
  `Box3DCore.Build.cs` definitions (or `BOX3D_USER_CONFIG`), never by patching
  upstream source. This keeps upgrades a clean folder swap.
- `Box3DCoreModule.cpp` is ours (UBT module boilerplate), not upstream.

## Upgrading

1. `git clone --branch <tag> https://github.com/erincatto/box3d`
2. Replace `Public/box3d/` with `include/box3d/`, `Private/` (except
   `Box3DCoreModule.cpp`) with `src/` contents, `box3d.LICENSE` with `LICENSE`
3. Update version/commit above; check `src/CMakeLists.txt` diff for new
   compile definitions or flags that `Box3DCore.Build.cs` must mirror
