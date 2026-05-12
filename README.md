# SAFE_IsaacLabEnv

Isaac Lab simulation environment with two Unitree G1 humanoids. Built for the Extended Reality Robot Teleoperation project, a collaboration between SAFE Robotics Lab and Prof. Mohsen Moghaddam's group at Georgia Tech.

Two G1 robots stand across a tabletop. Both have their lower bodies fixed to the ground because locomotion is not part of this project. The upper body (waist plus both arms) takes joint position commands over either a keyboard or a ZMQ socket. ZMQ is the interface for external operator stations.

The repo is set up as a standalone project rather than a fork of Isaac Lab. Isaac Lab and the Unitree assets are pulled in as git submodules.

## Status

Both robots load with fixed root links and simulate at 120 Hz physics, 30 Hz control. Standalone mode holds the default pose. Keyboard teleop subscribes to carb events through omni.appwindow and exposes per-joint nudges with active-robot and joint-group switching. ZMQ teleop uses one SUB socket per robot with CONFLATE on, so stale messages get dropped. Joint group indices come from runtime regex matching against joint names, so URDF reordering does not break anything.

Inspire dexterous hand attachment is not wired up yet. The URDF-to-USD converter helper is in place, but the wrist fixed joints and the extended action vector are not. See docs/inspire_hand_setup.md.

There are no reward or termination terms yet. The env is a ManagerBasedEnv, not ManagerBasedRLEnv.

Operator station reference implementations (VR, MoCap) are not in this repo. The wire format is in docs/teleop_protocol.md.

## Requirements

Ubuntu 22.04, an NVIDIA GPU with a CUDA driver Isaac Sim accepts, Isaac Sim 4.5.0 or 5.x, and Isaac Lab 2.3 or newer. The 2.3 floor exists because the scene config uses ArticulationRootPropertiesCfg.fix_root_link. If the submodule pin is older, that attribute will not exist and import will fail.

Python deps beyond what Isaac Lab installs are in requirements.txt.

## Install

Clone with submodules:

```
git clone --recurse-submodules https://github.com/twilson322/SAFE_IsaacLabEnv.git
cd SAFE_IsaacLabEnv
pip install -r requirements.txt
```

Then follow the Isaac Lab install in the submodule. If Isaac Lab is already installed system-wide, skip the submodule init and clone normally.

The run script adds the repo root to sys.path, so launching from the repo root resolves the configs and scripts packages without needing pip install -e.

## Run

Standalone smoke test:

```
./isaaclab.sh -p scripts/run_dual_g1.py
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
  requirements.txt
  .gitignore
  docs/
    teleop_protocol.md
    inspire_hand_setup.md
    design_notes.md
  configs/
    __init__.py
    dual_g1_env_cfg.py
    dual_g1_scene_cfg.py
    inspire_hand_cfg.py
  scripts/
    __init__.py
    run_dual_g1.py
    dual_g1_teleop.py
  assets/
    README.md
```

## Locomotion mode

To unfix the base, open configs/dual_g1_scene_cfg.py and set FIX_BASE = False. The env config, teleop, and action dimension keep working. The robots will fall over the moment you flip it though, since the keyboard interface does not expose leg control and the default action holds the crouch pose. Adding left_leg and right_leg patterns back to DualG1Teleop.GROUP_PATTERNS is a two-line change.

## License

MIT. See LICENSE.
