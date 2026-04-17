import argparse
import torch
import numpy as np

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--teleop", action="store_true")
parser.add_argument("--teleop_zmq", action="store_true")
parser.add_argument("--left_port", type=int, default=5555)
parser.add_argument("--right_port", type=int, default=5556)
parser.add_argument("--device", type=str, default="cuda:0")

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from isaaclab.envs import ManagerBasedEnv
from dual_g1_env_cfg import DualG1EnvCfg
from dual_g1_teleop import DualG1Teleop, ZMQTeleopReceiver


def run_standalone(env, device):
    obs, _ = env.reset()
    num_dof = env.action_manager.total_action_dim
    print(f"Environment ready | Action dim: {num_dof}")

    step = 0
    while simulation_app.is_running():
        action = torch.zeros(env.num_envs, num_dof, device=device)
        obs, _, _, _, _ = env.step(action)
        step += 1

        if step % 500 == 0:
            print(f"Step {step}")
            env.reset()


def run_teleop_keyboard(env, device):
    teleop = DualG1Teleop(env, device)
    obs, _ = env.reset()

    print("=" * 60)
    print("DUAL G1 TELEOPERATION")
    print("=" * 60)
    print("  1/2      : Switch active robot (left/right)")
    print("  q/w/e/r/t: Switch joint group")
    print("  i/k      : Joint 0 +/-")
    print("  j/l      : Joint 1 +/-")
    print("  u/o      : Joint 2 +/-")
    print("  0        : Reset all targets")
    print("=" * 60)

    while simulation_app.is_running():
        action = teleop.get_action()
        obs, _, _, _, _ = env.step(action)


def run_teleop_zmq(env, device, left_port, right_port):
    receiver = ZMQTeleopReceiver(left_port, right_port)
    obs, _ = env.reset()

    num_dof_per_robot = 29
    left_target = torch.zeros(num_dof_per_robot, device=device)
    right_target = torch.zeros(num_dof_per_robot, device=device)

    print(f"Waiting for teleop data on ports {left_port}, {right_port}...")

    while simulation_app.is_running():
        left_joints, right_joints = receiver.receive()

        if left_joints is not None and len(left_joints) == num_dof_per_robot:
            left_target = torch.tensor(left_joints, device=device, dtype=torch.float32)

        if right_joints is not None and len(right_joints) == num_dof_per_robot:
            right_target = torch.tensor(right_joints, device=device, dtype=torch.float32)

        action = torch.cat([left_target, right_target]).unsqueeze(0)
        obs, _, _, _, _ = env.step(action)


def main():
    env_cfg = DualG1EnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env = ManagerBasedEnv(cfg=env_cfg)

    if args.teleop_zmq:
        run_teleop_zmq(env, args.device, args.left_port, args.right_port)
    elif args.teleop:
        run_teleop_keyboard(env, args.device)
    else:
        run_standalone(env, args.device)

    env.close()


if __name__ == "__main__":
    main()
