# Contributing

Bug reports, suggestions, and pull requests are welcome.

## Ground rules

- **Never edit vendored box3d.** `Source/Box3DCore/Public/box3d/` and
  `Source/Box3DCore/Private/` are byte-identical upstream copies (see
  `UPSTREAM.md`). Integration quirks belong in `Box3DRuntime`.
- **Tests are the contract.** Every behavioral change lands with automation
  coverage; the suite must stay green:
  `Automation RunTests Box3DUnreal` (headless command in `docs/TESTING.md`).
- **Match the house style.** UE conventions, tabs, `///` doc comments that
  explain *why*, minimal noise. Read a couple of existing files first.
- **Determinism is a feature.** Anything touching stepping, threading, or body
  state must keep `box3d.ValidateReplay` and the replay/threading tests green,
  including under `BOX3D_DOUBLE_PRECISION` (see `UPSTREAM.md` §Verify).

## Workflow

1. Fork, branch from `master`.
2. Build against UE 5.7 (drop the plugin into a C++ project's `Plugins/`).
3. Run the automation suite; add tests for new behavior.
4. Open a PR with a short rationale — what and, more importantly, why.

## Credits

The baked-collision pipeline and the snapshot/prediction/rollback primitives
were ported from [Antonio Lattanzio's Box3DUnreal](https://github.com/alattanzio/Box3DUnreal)
(MIT). Box3D itself is by [Erin Catto](https://github.com/erincatto/box3d) (MIT).
