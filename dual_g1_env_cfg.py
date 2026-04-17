import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg
from isaaclab.managers import ActionTermCfg
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from dual_g1_scene_cfg import DualG1SceneCfg


@configclass
class ActionsCfg:
    joint_pos_left = ActionTermCfg(
        class_type=mdp.JointPositionActionCfg,
        asset_name="robot_left",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=True,
    )

    joint_pos_right = ActionTermCfg(
        class_type=mdp.JointPositionActionCfg,
        asset_name="robot_right",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=True,
    )


@configclass
class ObservationsCfg:
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

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_left = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot_left"),
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)},
            "velocity_range": {},
        },
    )

    reset_right = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot_right"),
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)},
            "velocity_range": {},
        },
    )

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
    scene: DualG1SceneCfg = DualG1SceneCfg(num_envs=1, env_spacing=5.0)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()

    sim = sim_utils.SimulationCfg(
        dt=1.0 / 120.0,
        render_interval=2,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 30.0


import isaaclab.sim as sim_utils
