"""Teleoperation interfaces for the dual-G1 fixed-base env.

Two interfaces are provided:

* ``DualG1Teleop`` + ``KeyboardSubscription`` — local keyboard control for
  bring-up and debugging. Operator picks an active robot and a joint group,
  then nudges the first three joints in that group with ``i/k``, ``j/l``,
  ``u/o``.

* ``ZMQTeleopReceiver`` — external operator-station interface. One SUB socket
  per robot, expecting raw float32 absolute joint position targets. See
  ``docs/teleop_protocol.md`` for the wire format.

Both interfaces produce absolute joint position targets. The env's action term
is configured with ``use_default_offset=False``, so targets are interpreted as
radians directly (no offset is added).

Since the lower body is fixed (see ``configs/dual_g1_scene_cfg.py``), the
keyboard interface only exposes waist + arm groups. Leg DOF are still part of
the action vector and are simply held at their default crouch pose.
"""

import re

import numpy as np
import torch


class DualG1Teleop:
    """Builds per-robot joint targets from keyboard events.

    Joint-group indices are derived at construction time by regex-matching the
    actual joint names reported by each robot's articulation, so this works
    regardless of the underlying URDF joint ordering.
    """

    # Group name → list of regexes that match member joint names.
    GROUP_PATTERNS = {
        "waist":     [r"waist_.*"],
        "left_arm":  [r"left_shoulder_.*", r"left_elbow.*", r"left_wrist_.*"],
        "right_arm": [r"right_shoulder_.*", r"right_elbow.*", r"right_wrist_.*"],
    }

    def __init__(self, env, device: str = "cuda:0", action_scale: float = 0.05):
        self.env = env
        self.device = device
        self.action_scale = action_scale

        # Pull joint names + defaults from the articulations.
        self._left = env.scene["robot_left"]
        self._right = env.scene["robot_right"]

        self.left_joint_names = list(self._left.joint_names)
        self.right_joint_names = list(self._right.joint_names)
        self.num_dof_left = len(self.left_joint_names)
        self.num_dof_right = len(self.right_joint_names)

        # Default joint positions for env index 0; clone so we own the buffer.
        self.left_default = self._left.data.default_joint_pos[0].clone().to(device)
        self.right_default = self._right.data.default_joint_pos[0].clone().to(device)

        # Active targets start at default pose (sensible neutral).
        self.left_target = self.left_default.clone()
        self.right_target = self.right_default.clone()

        # Build {group_name: [joint_index, ...]} for each robot.
        self.left_groups = self._build_groups(self.left_joint_names)
        self.right_groups = self._build_groups(self.right_joint_names)

        self.active_robot = "left"
        self.active_group = "left_arm"

        self._print_group_summary()

    @classmethod
    def _build_groups(cls, joint_names):
        groups = {}
        for group_name, patterns in cls.GROUP_PATTERNS.items():
            indices = [
                i for i, name in enumerate(joint_names)
                if any(re.match(p, name) for p in patterns)
            ]
            groups[group_name] = indices
        return groups

    def _print_group_summary(self):
        print("Resolved joint groups (left robot):")
        for group, indices in self.left_groups.items():
            names = [self.left_joint_names[i] for i in indices]
            print(f"  {group:10s}: {names}")

    # ------------------------------------------------------------------ actions

    def get_action(self) -> torch.Tensor:
        """Return ``[1, num_dof_left + num_dof_right]`` action for env.step()."""
        return torch.cat([self.left_target, self.right_target]).unsqueeze(0)

    def reset_targets(self):
        self.left_target.copy_(self.left_default)
        self.right_target.copy_(self.right_default)

    def _nudge(self, joint_offset: int, direction: float):
        """Nudge the joint at position ``joint_offset`` within the active group."""
        if self.active_robot == "left":
            groups, target = self.left_groups, self.left_target
        else:
            groups, target = self.right_groups, self.right_target

        indices = groups.get(self.active_group, [])
        if joint_offset >= len(indices):
            return
        idx = indices[joint_offset]
        target[idx] += direction * self.action_scale

    # ----------------------------------------------------------------- keyboard

    def process_keyboard(self, key: str):
        """Handle a single keypress (uppercase carb input name, e.g. 'I', '1')."""
        # Normalize various carb name formats: "KEY_1" → "1", "NUMBER_1" → "1".
        key = key.upper()
        for prefix in ("KEY_", "NUMBER_"):
            if key.startswith(prefix):
                key = key[len(prefix):]

        if key == "1":
            self.active_robot = "left"
            print("Active robot: LEFT")
        elif key == "2":
            self.active_robot = "right"
            print("Active robot: RIGHT")
        elif key == "Q":
            self.active_group = "left_arm"
            print("Active group: left_arm")
        elif key == "W":
            self.active_group = "right_arm"
            print("Active group: right_arm")
        elif key == "E":
            self.active_group = "waist"
            print("Active group: waist")
        elif key == "0":
            self.reset_targets()
            print("Targets reset to default pose")
        elif key == "I":
            self._nudge(0, +1.0)
        elif key == "K":
            self._nudge(0, -1.0)
        elif key == "J":
            self._nudge(1, +1.0)
        elif key == "L":
            self._nudge(1, -1.0)
        elif key == "U":
            self._nudge(2, +1.0)
        elif key == "O":
            self._nudge(2, -1.0)


class KeyboardSubscription:
    """Bridge from Isaac Sim's carb keyboard events to a Python callback.

    Lifetime-managed: call ``shutdown()`` to unsubscribe cleanly. Both KEY_PRESS
    and KEY_REPEAT events trigger the callback so that holding a key produces
    continuous motion.
    """

    def __init__(self, callback):
        import carb
        import omni.appwindow

        self._carb_input = carb.input
        self._callback = callback

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_event
        )

    def _on_event(self, event, *args, **kwargs):
        cei = self._carb_input.KeyboardEventType
        if event.type in (cei.KEY_PRESS, cei.KEY_REPEAT):
            self._callback(event.input.name)
        # Returning True allows the event to propagate to other subscribers.
        return True

    def shutdown(self):
        if self._sub is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub)
            self._sub = None


class ZMQTeleopReceiver:
    """Receives raw float32 joint position targets from a PUB socket per robot.

    Wire format: ``expected_dof`` little-endian float32s per message, no header.
    See ``docs/teleop_protocol.md`` for the full spec.

    ``CONFLATE`` is enabled (and must be set before ``connect``) so the SUB
    socket only ever holds the most recent message — stale targets are useless
    for control.
    """

    def __init__(self, left_port: int = 5555, right_port: int = 5556, expected_dof: int = 29):
        import zmq

        self.expected_dof = expected_dof
        self.expected_bytes = expected_dof * 4  # float32 == 4 bytes
        self.ctx = zmq.Context()

        self.left_socket = self._make_socket(zmq, left_port)
        self.right_socket = self._make_socket(zmq, right_port)

    def _make_socket(self, zmq_mod, port: int):
        sock = self.ctx.socket(zmq_mod.SUB)
        # CONFLATE must be set BEFORE connect, otherwise it is silently ignored.
        sock.setsockopt(zmq_mod.CONFLATE, 1)
        sock.connect(f"tcp://localhost:{port}")
        sock.setsockopt_string(zmq_mod.SUBSCRIBE, "")
        sock.setsockopt(zmq_mod.RCVTIMEO, 1)
        return sock

    def receive(self):
        """Return ``(left_array, right_array)``; either may be ``None``."""
        import zmq

        left = right = None
        try:
            data = self.left_socket.recv(flags=zmq.NOBLOCK)
            if len(data) == self.expected_bytes:
                left = np.frombuffer(data, dtype=np.float32)
        except zmq.Again:
            pass
        try:
            data = self.right_socket.recv(flags=zmq.NOBLOCK)
            if len(data) == self.expected_bytes:
                right = np.frombuffer(data, dtype=np.float32)
        except zmq.Again:
            pass
        return left, right

    def close(self):
        self.left_socket.close()
        self.right_socket.close()
        self.ctx.term()
