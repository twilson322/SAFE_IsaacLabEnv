"""Main launch script for the dual-G1 fixed-base env.

Run from the repo root::

    ./isaaclab.sh -p scripts/run_dual_g1.py                 # standalone hold
    ./isaaclab.sh -p scripts/run_dual_g1.py --teleop        # keyboard
    ./isaaclab.sh -p scripts/run_dual_g1.py --teleop_zmq    # external operator

The script inserts the repo root on ``sys.path`` so ``configs`` and ``scripts``
resolve as packages regardless of where the user CD's from.
"""

import argparse
import sys
from pathlib import Path

# Make ``configs.*`` and ``scripts.*`` resolvable when launched from anywhere.
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# ---------------------------------------------------------------------------
# CLI + Isaac Sim app launch
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--teleop", action="store_true", help="Local keyboard teleop")
parser.add_argument("--teleop_zmq", action="store_true", help="External ZMQ teleop")
parser.add_argument("--left_port", type=int, default=5555)
parser.add_argument("--right_port", type=int, default=5556)
parser.add_argument("--device", type=str, default="cuda:0")

from isaaclab.app import AppLauncher  # noqa: E402

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# ---------------------------------------------------------------------------
# Imports that require the simulation app to already be running
# ---------------------------------------------------------------------------
import torch  # noqa: E402

from isaaclab.envs import ManagerBasedEnv  # noqa: E402

from configs.dual_g1_env_cfg import DualG1EnvCfg  # noqa: E402
from scripts.dual_g1_teleop import (  # noqa: E402
    DualG1Teleop,
    KeyboardSubscription,
    ZMQTeleopReceiver,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _default_action(env, device: str) -> torch.Tensor:
    """Build a ``[num_envs, total_action_dim]`` action that holds the default pose.

    Required because the action term is configured with ``use_default_offset=False``,
    so a zero action would command every joint to 0 rad (legs straight, etc.).
    """
    left_def = env.scene["robot_left"].data.default_joint_pos[0].to(device)
    right_def = env.scene["robot_right"].data.default_joint_pos[0].to(device)
    per_env = torch.cat([left_def, right_def])
    return per_env.unsqueeze(0).expand(env.num_envs, -1).contiguous()


# ---------------------------------------------------------------------------
# Run modes
# ---------------------------------------------------------------------------
def run_standalone(env: ManagerBasedEnv, device: str):
    """Idle hold: every step commands the default pose. Useful as a smoke test."""
    obs, _ = env.reset()
    action = _default_action(env, device)

    total_action_dim = env.action_manager.total_action_dim
    print(f"Environment ready | num_envs={env.num_envs} | action_dim={total_action_dim}")

    step = 0
    while simulation_app.is_running():
        env.step(action)
        step += 1
        if step % 500 == 0:
            print(f"Step {step}")
            env.reset()


def run_teleop_keyboard(env: ManagerBasedEnv, device: str):
    """Local keyboard teleop. Targets persist until the user nudges them."""
    teleop = DualG1Teleop(env, device=device)
    env.reset()

    keyboard_sub = KeyboardSubscription(callback=teleop.process_keyboard)

    print("=" * 64)
    print("DUAL G1 KEYBOARD TELEOPERATION (fixed base — arms + waist only)")
    print("=" * 64)
    print("  1/2      : Switch active robot (left/right)")
    print("  q/w/e    : Switch joint group (left_arm / right_arm / waist)")
    print("  i/k      : Joint 0 +/-")
    print("  j/l      : Joint 1 +/-")
    print("  u/o      : Joint 2 +/-")
    print("  0        : Reset targets to default pose")
    print("=" * 64)

    try:
        while simulation_app.is_running():
            per_env = teleop.get_action()                       # [1, dof_total]
            action = per_env.expand(env.num_envs, -1).contiguous()
            env.step(action)
    finally:
        keyboard_sub.shutdown()


def run_teleop_zmq(env: ManagerBasedEnv, device: str, left_port: int, right_port: int):
    """External-operator teleop. Reads absolute joint targets from two SUB sockets."""
    env.reset()

    left_target = env.scene["robot_left"].data.default_joint_pos[0].clone().to(device)
    right_target = env.scene["robot_right"].data.default_joint_pos[0].clone().to(device)
    num_dof_per_robot = left_target.shape[0]

    receiver = ZMQTeleopReceiver(
        left_port=left_port,
        right_port=right_port,
        expected_dof=num_dof_per_robot,
    )

    print(
        f"ZMQ teleop ready | dof_per_robot={num_dof_per_robot} | "
        f"left=tcp://localhost:{left_port} right=tcp://localhost:{right_port}"
    )

    try:
        while simulation_app.is_running():
            left_msg, right_msg = receiver.receive()
            if left_msg is not None:
                left_target = torch.tensor(left_msg, device=device, dtype=torch.float32)
            if right_msg is not None:
                right_target = torch.tensor(right_msg, device=device, dtype=torch.float32)

            per_env = torch.cat([left_target, right_target]).unsqueeze(0)
            action = per_env.expand(env.num_envs, -1).contiguous()
            env.step(action)
    finally:
        receiver.close()


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
def main():
    if args.teleop and args.teleop_zmq:
        raise SystemExit("Pick one: --teleop or --teleop_zmq, not both.")

    env_cfg = DualG1EnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env = ManagerBasedEnv(cfg=env_cfg)

    try:
        if args.teleop_zmq:
            run_teleop_zmq(env, args.device, args.left_port, args.right_port)
        elif args.teleop:
            run_teleop_keyboard(env, args.device)
        else:
            run_standalone(env, args.device)
    finally:
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
