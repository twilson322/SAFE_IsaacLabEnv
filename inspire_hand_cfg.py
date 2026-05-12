"""Inspire dexterous hand articulation configs and a URDF→USD helper.

The USD paths are placeholders. After running ``convert_urdf_to_usd`` below on
the URDF you obtain from Inspire Robotics, paste the resulting USD path into
the two ``usd_path`` fields. See ``docs/inspire_hand_setup.md`` for the full
walkthrough including wrist attachment.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


# Conservative placeholder actuator gains. Re-tune after the hand loads — drive
# a sinusoid into one finger joint and inspect the response (see setup doc).
_FINGER_ACTUATOR = ImplicitActuatorCfg(
    joint_names_expr=[".*"],
    effort_limit=2.0,        # N·m   — placeholder; verify against Inspire datasheet
    velocity_limit=10.0,     # rad/s
    stiffness=20.0,          # N·m/rad
    damping=2.0,             # N·m·s/rad
)


INSPIRE_HAND_LEFT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="REPLACE_WITH_INSPIRE_HAND_LEFT_USD_PATH",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    actuators={"fingers": _FINGER_ACTUATOR},
)


INSPIRE_HAND_RIGHT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="REPLACE_WITH_INSPIRE_HAND_RIGHT_USD_PATH",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    actuators={"fingers": _FINGER_ACTUATOR},
)


def convert_urdf_to_usd(urdf_path: str, output_dir: str) -> str:
    """Convert an Inspire Hand URDF to USD using Isaac Lab's converter.

    Parameters
    ----------
    urdf_path
        Path to the Inspire Hand URDF (with mesh files reachable via the URDF's
        package or relative paths).
    output_dir
        Directory to write the USD output into. Will be created if missing.

    Returns
    -------
    str
        Absolute path to the generated ``.usd`` file. Paste this into the
        ``usd_path`` field of ``INSPIRE_HAND_*_CFG`` above.
    """
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=output_dir,
        force_usd_conversion=True,
        fix_base=False,             # the hand will be attached to the wrist via fixed joint
        merge_fixed_joints=False,   # keep fingertip frames addressable
        make_instanceable=True,     # required for vectorized envs (--num_envs > 1)
    )

    converter = UrdfConverter(cfg)
    return converter.usd_path
