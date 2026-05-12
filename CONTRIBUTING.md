# Contributing

Internal lab repo. The conventions below exist so collaborators can extend the env without arguments later.

## Branches

`main` is always working. The smoke test passes and standalone mode runs cleanly. Do not push directly to main.

Feature branches use `feat/<short-name>`. Example: `feat/inspire-hand-fixed-joint`. Bug fixes use `fix/<short-name>`. Docs-only changes use `docs/<short-name>`.

## Commits

Imperative present tense, lowercase first word, no trailing period:

```
add example zmq sender
fix nan in joint vel observation
update teleop protocol doc
```

Group related changes per commit. Avoid the "wip" or "more changes" pattern.

## Pull requests

Before opening a PR:

1. `./isaaclab.sh -p scripts/smoke_test.py` passes locally.
2. `ruff check configs scripts` passes.
3. The README and the docs in `docs/` still match what the code actually does.

The PR description should say what changed, why, and how you tested it. If the change affects the ZMQ wire format or the action dimension, call that out at the top of the description.

## Adding new features

**New observation term.** Add the `ObservationTermCfg` to `configs/dual_g1_env_cfg.py:ObservationsCfg.PolicyCfg`. Mention it in `docs/design_notes.md` if it changes the observation contract for downstream policies.

**New action term.** Add to `configs/dual_g1_env_cfg.py:ActionsCfg`. Update `scripts/dual_g1_teleop.py` to produce the right slice of the action vector. Bump the expected DOF in `docs/teleop_protocol.md` if the external wire format changes.

**New scene asset (manipulation object, fixture, etc.).** Add to `configs/dual_g1_scene_cfg.py:DualG1SceneCfg`. If the asset needs to reset between episodes, add an `EventTermCfg` to `EventCfg`.

**New script.** Goes in `scripts/`. Top of file: docstring explaining what it does, how to launch it, and what it depends on. If it launches the sim, copy the `sys.path` shim and `AppLauncher` pattern from `run_dual_g1.py`.

**New reward or termination.** Goes in `configs/dual_g1_rl_env_cfg.py`. Smoothness and energy rewards already exist as stubs. Task-specific rewards go alongside, not in place of, those.

## Code style

`ruff check` configured in `pyproject.toml`. Line length 100. No type stub requirement, but use type hints on public function signatures. Single-letter variable names only in tight numerical scopes (loop indices, math).

## Writing style for docs

Plain prose. No em-dashes, no rhetorical flourishes, no "comprehensive" or "robust" or "leverage". Engineer voice. If a sentence reads like marketing copy, rewrite it.

## Tests

There is no full test suite. The smoke test is the contract: if it fails on main, that is a release blocker. Add more `scripts/smoke_*.py` files as new subsystems land (camera observations, hand articulation, etc.). Each one should exit 0 on success and nonzero on any failure, and should run headless.

CI runs syntax check and ruff. It does not run the smoke test because GitHub's free runners lack a GPU. Run the smoke test locally before pushing.

## When in doubt

Open an issue or pull `@twilson322` into Slack before doing anything that changes the wire format or the env contract.
