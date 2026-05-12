"""Manager-based env config for the dual-G1 fixed-base teleop scene.

Actions are absolute joint position targets in radians (``use_default_offset=False``).
The action vector has length ``num_dof_left + num_dof_right`` — for a stock G1
this is 29 + 29 = 58 — concatenated as ``[left, right]``.
"""

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg, ObservationGroupCfg, ObservationTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from configs.dual_g1_scene_cfg import DualG1SceneCfg


@configclass
class ActionsCfg:
    """Per-robot absolute joint-position action terms."""

    joint_pos_left = mdp.JointPositionActionCfg(
        asset_name="robot_left",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=False,
    )

    joint_pos_right = mdp.JointPositionActionCfg(
        asset_name="robot_right",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=False,
    )


@configclass
class ObservationsCfg:
    """Proprioception + base pose for each robot, plus the last action."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        joint_pos_left = ObservationTermCfg(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot_left")},
        )
        joint_vel_left = ObservationTermCfg(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot_left")},
        )
        base_pos_left = ObservationTermCfg(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("robot_left")},
        )
        base_quat_left = ObservationTermCfg(
            func=mdp.root_quat_w,
            params={"asset_cfg": SceneEntityCfg("robot_left")},
        )

        joint_pos_right = ObservationTermCfg(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot_right")},
        )
        joint_vel_right = ObservationTermCfg(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot_right")},
        )
        base_pos_right = ObservationTermCfg(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("robot_right")},
        )
        base_quat_right = ObservationTermCfg(
            func=mdp.root_quat_w,
            params={"asset_cfg": SceneEntityCfg("robot_right")},
        )

        actions = ObservationTermCfg(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Reset-time randomization of joint positions/velocities.

    The base is fixed (see ``DualG1SceneCfg``), so we do not randomize the root
    pose at reset. Joints get a small uniform offset to avoid identical initial
    conditions across episodes.
    """

    reset_joints_left = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot_left"),
            "position_range": (-0.1, 0.1),
            "velocity_range": (-0.05, 0.05),
        },
    )

    reset_joints_right = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot_right"),
            "position_range": (-0.1, 0.1),
            "velocity_range": (-0.05, 0.05),
        },
    )


@configclass
class DualG1EnvCfg(ManagerBasedEnvCfg):
    """Top-level env cfg consumed by ``ManagerBasedEnv``."""

    scene: DualG1SceneCfg = DualG1SceneCfg(num_envs=1, env_spacing=5.0)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()

    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=1.0 / 120.0,
        render_interval=2,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )

    def __post_init__(self):
        # 120 Hz physics / decimation 4 → 30 Hz control.
        self.decimation = 4
        self.episode_length_s = 30.0
