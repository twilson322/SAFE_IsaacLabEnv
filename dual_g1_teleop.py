import argparse
import numpy as np
import torch
from isaaclab.envs import ManagerBasedEnv
from dual_g1_env_cfg import DualG1EnvCfg


class DualG1Teleop:
    def __init__(self, env, device="cuda:0"):
        self.env = env
        self.device = device
        self.num_dof_per_robot = 29
        self.action_scale = 0.05

        self.left_target = torch.zeros(self.num_dof_per_robot, device=device)
        self.right_target = torch.zeros(self.num_dof_per_robot, device=device)

        self.joint_groups = {
            "left_arm": list(range(13, 20)),
            "right_arm": list(range(20, 27)),
            "left_hand": list(range(27, 29)) if self.num_dof_per_robot > 27 else [],
            "right_hand": [],
            "waist": list(range(0, 3)),
            "left_leg": list(range(3, 8)),
            "right_leg": list(range(8, 13)),
        }

        self.active_group = "left_arm"
        self.active_robot = "left"

    def set_joint_target(self, robot, group_name, delta):
        indices = self.joint_groups.get(group_name, [])
        if not indices:
            return

        if robot == "left":
            for i, idx in enumerate(indices):
                if i < len(delta):
                    self.left_target[idx] += delta[i] * self.action_scale
        else:
            for i, idx in enumerate(indices):
                if i < len(delta):
                    self.right_target[idx] += delta[i] * self.action_scale

    def get_action(self):
        action = torch.cat([self.left_target, self.right_target]).unsqueeze(0)
        return action

    def reset_targets(self):
        self.left_target.zero_()
        self.right_target.zero_()

    def process_keyboard(self, key, pressed):
        if not pressed:
            return

        if key == "1":
            self.active_robot = "left"
            print(f"Active robot: LEFT")
        elif key == "2":
            self.active_robot = "right"
            print(f"Active robot: RIGHT")
        elif key == "q":
            self.active_group = "left_arm"
            print(f"Active group: left_arm")
        elif key == "w":
            self.active_group = "right_arm"
            print(f"Active group: right_arm")
        elif key == "e":
            self.active_group = "waist"
            print(f"Active group: waist")
        elif key == "r":
            self.active_group = "left_leg"
            print(f"Active group: left_leg")
        elif key == "t":
            self.active_group = "right_leg"
            print(f"Active group: right_leg")
        elif key == "0":
            self.reset_targets()
            print("Targets reset to zero")

        delta = np.zeros(7)
        if key == "i":
            delta[0] = 1.0
        elif key == "k":
            delta[0] = -1.0
        elif key == "j":
            delta[1] = 1.0
        elif key == "l":
            delta[1] = -1.0
        elif key == "u":
            delta[2] = 1.0
        elif key == "o":
            delta[2] = -1.0

        if np.any(delta != 0):
            self.set_joint_target(self.active_robot, self.active_group, delta)


class ZMQTeleopReceiver:
    def __init__(self, left_port=5555, right_port=5556):
        import zmq
        self.ctx = zmq.Context()

        self.left_socket = self.ctx.socket(zmq.SUB)
        self.left_socket.connect(f"tcp://localhost:{left_port}")
        self.left_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.left_socket.setsockopt(zmq.RCVTIMEO, 1)

        self.right_socket = self.ctx.socket(zmq.SUB)
        self.right_socket.connect(f"tcp://localhost:{right_port}")
        self.right_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.right_socket.setsockopt(zmq.RCVTIMEO, 1)

    def receive(self):
        left_joints = None
        right_joints = None

        import zmq
        try:
            data = self.left_socket.recv(flags=zmq.NOBLOCK)
            left_joints = np.frombuffer(data, dtype=np.float32)
        except zmq.Again:
            pass

        try:
            data = self.right_socket.recv(flags=zmq.NOBLOCK)
            right_joints = np.frombuffer(data, dtype=np.float32)
        except zmq.Again:
            pass

        return left_joints, right_joints
