"""Headless smoke test for the dual G1 env.

Launches the sim, resets, steps for N frames with the default pose as the
action, and checks for NaN in observations and actions. Exits 0 on success,
nonzero on any failure.

Run from the repo root:

    ./isaaclab.sh -p scripts/smoke_test.py

The script forces headless mode regardless of CLI args, since it is intended for
unattended use (CI, pre-push hook, sanity check after a rebase).
"""

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=int, default=100,
                    help="Number of env.step() calls before declaring success.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# Force headless. Smoke test should never open a window.
args.headless = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402

from isaaclab.envs import ManagerBasedEnv  # noqa: E402

from configs.dual_g1_env_cfg import DualG1EnvCfg  # noqa: E402


def _check_nan(obj, label: str, step: int) -> bool:
    """Return True if NaN was found. Handles tensor and dict-of-tensor."""
    if isinstance(obj, torch.Tensor):
        if torch.isnan(obj).any():
            print(f"FAIL: NaN in {label} at step {step}")
            return True
    elif isinstance(obj, dict):
        for name, value in obj.items():
            if _check_nan(value, f"{label}[{name}]", step):
                return True
    return False


def main() -> int:
    env_cfg = DualG1EnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedEnv(cfg=env_cfg)

    try:
        obs, _ = env.reset()

        left_def = env.scene["robot_left"].data.default_joint_pos[0]
        right_def = env.scene["robot_right"].data.default_joint_pos[0]
        action = torch.cat([left_def, right_def]).unsqueeze(0)

        total_dim = env.action_manager.total_action_dim
        print(f"env up | num_envs=1 | action_dim={total_dim} | running {args.steps} steps")

        for step in range(args.steps):
            obs, _, _, _, _ = env.step(action)
            if _check_nan(obs, "obs", step):
                return 1
            if _check_nan(action, "action", step):
                return 1

        print(f"OK: ran {args.steps} steps with no NaN")
        return 0
    except Exception as e:
        print(f"FAIL: exception during smoke test: {e}")
        return 2
    finally:
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    sys.exit(main())
