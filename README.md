# SAFE_IsaacLabEnv

Isaac Lab simulation environment with two Unitree G1 humanoids. Built for the Extended Reality Robot Teleoperation project, a collaboration between SAFE Robotics Lab and Prof. Mohsen Moghaddam's group at Georgia Tech.

Two G1 robots stand across a tabletop. Both have their lower bodies fixed to the ground because locomotion is not part of this project. The upper body (waist plus both arms) takes joint position commands over either a keyboard or a ZMQ socket. ZMQ is the interface for external operator stations.

The repo is set up as a standalone project rather than a fork of Isaac Lab. Isaac Lab and the Unitree assets are pulled in as git submodules.

## Status

Both robots load with fixed root links and simulate at 120 Hz physics, 30 Hz control. Standalone mode holds the default pose. Keyboard teleop subscribes to carb events through omni.appwindow and exposes per-joint nudges with active-robot and joint-group switching. ZMQ teleop uses one SUB socket per robot with CONFLATE on, so stale messages get dropped. Joint group indices come from runtime regex matching against joint names, so URDF reordering does not break anything.

An RL variant of the env exists at `configs/dual_g1_rl_env_cfg.py` with stub smoothness rewards and a timeout termination. Task-specific rewards are not in yet.

Inspire dexterous hand attachment is not wired up yet. The URDF-to-USD converter helper is in place, but the wrist fixed joints and the extended action vector are not. See `docs/inspire_hand_setup.md`.

Operator station reference implementations (VR, MoCap) are not in this repo. The wire format is in `docs/teleop_protocol.md`, and a simple Python sender is in `scripts/example_zmq_sender.py`.

## Requirements

Ubuntu 22.04, an NVIDIA GPU with a CUDA driver Isaac Sim accepts, Isaac Sim 4.5.0 or 5.x, and Isaac Lab 2.3 or newer. The 2.3 floor exists because the scene config uses `ArticulationRootPropertiesCfg.fix_root_link`. If the submodule pin is older, that attribute will not exist and import will fail.

Python deps beyond what Isaac Lab installs are in `requirements.txt` and `pyproject.toml`.

## Install

Clone with submodules:

```
git clone --recurse-submodules https://github.com/twilson322/SAFE_IsaacLabEnv.git
cd SAFE_IsaacLabEnv
pip install -e .
```

Then follow the Isaac Lab install in the submodule. If Isaac Lab is already installed system-wide, skip the submodule init and clone normally.

The editable install replaces the older `sys.path` hack so `import configs.dual_g1_env_cfg` works from anywhere. The shim is still in `scripts/run_dual_g1.py` and `scripts/smoke_test.py` as a fallback, so the scripts run even without `pip install -e`.

For development tooling:

```
pip install -e .[dev]
```

This pulls in `ruff` for linting.

## Run

Standalone smoke test (interactive, holds the default pose):

```
./isaaclab.sh -p scripts/run_dual_g1.py
```

Headless smoke test (the one CI would run if it had a GPU):

```
./isaaclab.sh -p scripts/smoke_test.py
```

Keyboard teleop:

```
./isaaclab.sh -p scripts/run_dual_g1.py --teleop
```

ZMQ teleop:

```
./isaaclab.sh -p scripts/run_dual_g1.py --teleop_zmq --left_port 5555 --right_port 5556
```

Vectorized standalone (data collection, no teleop):

```
./isaaclab.sh -p scripts/run_dual_g1.py --num_envs 16
```

## Tools

`scripts/example_zmq_sender.py` publishes a slow sinusoid on the shoulder pitch joints. Run it alongside `run_dual_g1.py --teleop_zmq` to confirm the receive path works without needing the full XR pipeline.

`scripts/record_teleop.py` subscribes to the ZMQ ports and writes timestamped joint targets to a .npz file. Useful for capturing operator trajectories, debugging retargeting offline, and seeding imitation learning data.

`scripts/replay_teleop.py` reads a recording and republishes it. Useful for regression testing and reproducible demos.

`scripts/smoke_test.py` launches the env headless, steps for 100 frames, and exits 0 on success or nonzero on any failure. Run it before pushing.

## Keyboard controls

```
1, 2          switch active robot (left, right)
Q, W          switch joint group to left arm or right arm
E             switch joint group to waist
I, K          joint 0 +/-
J, L          joint 1 +/-
U, O          joint 2 +/-
0             reset all targets to default pose
```

At startup the teleop prints which joint names map to each group for the loaded G1. Use that printout to confirm the mapping before relying on it.

## Repo layout

```
SAFE_IsaacLabEnv/
  README.md
  LICENSE
  CONTRIBUTING.md
  pyproject.toml
  requirements.txt
  .gitignore
  .github/workflows/lint.yml
  docs/
    teleop_protocol.md
    inspire_hand_setup.md
    design_notes.md
  configs/
    __init__.py
    dual_g1_env_cfg.py
    dual_g1_rl_env_cfg.py
    dual_g1_scene_cfg.py
    inspire_hand_cfg.py
  scripts/
    __init__.py
    run_dual_g1.py
    dual_g1_teleop.py
    example_zmq_sender.py
    record_teleop.py
    replay_teleop.py
    smoke_test.py
  assets/
    README.md
```

## Locomotion mode

To unfix the base, open `configs/dual_g1_scene_cfg.py` and set `FIX_BASE = False`. The env config, teleop, and action dimension keep working. The robots will fall over the moment you flip it though, since the keyboard interface does not expose leg control and the default action holds the crouch pose. Adding `left_leg` and `right_leg` patterns back to `DualG1Teleop.GROUP_PATTERNS` is a two-line change.

## Contributing

See `CONTRIBUTING.md` for branch and PR conventions, code style, and what to do when adding a new observation, action, scene asset, or reward.

## License

MIT. See `LICENSE`.
