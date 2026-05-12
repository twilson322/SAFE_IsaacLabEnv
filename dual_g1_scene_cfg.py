"""Scene config: two Unitree G1 humanoids with fixed lower body and a tabletop.

The G1s are spawned with their root link fixed to the world via the articulation
root property ``fix_root_link``. This is intentional: the SAFE Lab + Moghaddam Lab
XR teleop project does not exercise locomotion, so the legs are postured to a
stable crouch and held there by the controller. To restore a free-floating base
for locomotion experiments, set ``FIX_BASE = False`` below.
"""

import copy

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from isaaclab_assets.robots.unitree import G1_MINIMAL_CFG


# ---------------------------------------------------------------------------
# Fixed-base toggle
# ---------------------------------------------------------------------------
FIX_BASE = True


def _make_g1_spawn(fix_base: bool):
    """Return a deep copy of the G1 minimal spawn cfg, optionally fixing the root."""
    spawn = copy.deepcopy(G1_MINIMAL_CFG.spawn)
    if fix_base:
        # ``articulation_props`` is an ArticulationRootPropertiesCfg; create one if
        # the upstream cfg left it as None so we always have a writable handle.
        if spawn.articulation_props is None:
            spawn.articulation_props = sim_utils.ArticulationRootPropertiesCfg()
        spawn.articulation_props.fix_root_link = True
    return spawn


_G1_SPAWN = _make_g1_spawn(FIX_BASE)


# ---------------------------------------------------------------------------
# Initial joint posture (shared between the two robots)
# ---------------------------------------------------------------------------
#
# Slight knee/hip bend keeps the body posed naturally even though the base is
# fixed. Arms forward+slightly bent is a manipulation-ready stance for teleop.
_INIT_JOINT_POS = {
    "left_hip_pitch_joint": -0.2,
    "right_hip_pitch_joint": -0.2,
    "left_knee_joint": 0.4,
    "right_knee_joint": 0.4,
    "left_ankle_pitch_joint": -0.2,
    "right_ankle_pitch_joint": -0.2,
    ".*_shoulder_pitch_joint": 0.3,
    ".*_elbow_pitch_joint": 0.5,
}


# Per-robot cfg templates. ``prim_path`` is patched per-instance in the scene below.
G1_LEFT_CFG = ArticulationCfg(
    spawn=_G1_SPAWN,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, -0.5, 1.05),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=_INIT_JOINT_POS,
    ),
    actuators=G1_MINIMAL_CFG.actuators,
    soft_joint_pos_limit_factor=0.9,
)

G1_RIGHT_CFG = ArticulationCfg(
    spawn=_G1_SPAWN,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.5, 1.05),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=_INIT_JOINT_POS,
    ),
    actuators=G1_MINIMAL_CFG.actuators,
    soft_joint_pos_limit_factor=0.9,
)


@configclass
class DualG1SceneCfg(InteractiveSceneCfg):
    """Two G1s facing across a tabletop with default Isaac Lab lighting and floor."""

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

    robot_left: ArticulationCfg = G1_LEFT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/RobotLeft"
    )

    robot_right: ArticulationCfg = G1_RIGHT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/RobotRight"
    )
