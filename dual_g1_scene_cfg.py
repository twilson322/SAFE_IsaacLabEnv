import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg, ActionTermCfg
from isaaclab.managers import EventTermCfg, SceneEntityCfg

import isaaclab_assets
from isaaclab_assets.robots.unitree import G1_MINIMAL_CFG

import math


G1_LEFT_CFG = ArticulationCfg(
    spawn=G1_MINIMAL_CFG.spawn,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, -0.5, 1.05),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "left_hip_pitch_joint": -0.2,
            "right_hip_pitch_joint": -0.2,
            "left_knee_joint": 0.4,
            "right_knee_joint": 0.4,
            "left_ankle_pitch_joint": -0.2,
            "right_ankle_pitch_joint": -0.2,
            ".*_shoulder_pitch_joint": 0.3,
            ".*_elbow_pitch_joint": 0.5,
        },
    ),
    actuators=G1_MINIMAL_CFG.actuators,
    soft_joint_pos_limit_factor=0.9,
)

G1_RIGHT_CFG = ArticulationCfg(
    spawn=G1_MINIMAL_CFG.spawn,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.5, 1.05),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "left_hip_pitch_joint": -0.2,
            "right_hip_pitch_joint": -0.2,
            "left_knee_joint": 0.4,
            "right_knee_joint": 0.4,
            "left_ankle_pitch_joint": -0.2,
            "right_ankle_pitch_joint": -0.2,
            ".*_shoulder_pitch_joint": 0.3,
            ".*_elbow_pitch_joint": 0.5,
        },
    ),
    actuators=G1_MINIMAL_CFG.actuators,
    soft_joint_pos_limit_factor=0.9,
)


@configclass
class DualG1SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(
            intensity=2000.0,
            color=(0.75, 0.75, 0.75),
        ),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=(1.2, 0.6, 0.8),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.4, 0.3, 0.2),
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.5, 0.0, 0.4)),
    )

    robot_left = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/RobotLeft",
        spawn=G1_LEFT_CFG.spawn,
        init_state=G1_LEFT_CFG.init_state,
        actuators=G1_LEFT_CFG.actuators,
        soft_joint_pos_limit_factor=G1_LEFT_CFG.soft_joint_pos_limit_factor,
    )

    robot_right = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/RobotRight",
        spawn=G1_RIGHT_CFG.spawn,
        init_state=G1_RIGHT_CFG.init_state,
        actuators=G1_RIGHT_CFG.actuators,
        soft_joint_pos_limit_factor=G1_RIGHT_CFG.soft_joint_pos_limit_factor,
    )
