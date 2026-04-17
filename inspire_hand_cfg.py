import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass


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
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit=2.0,
            velocity_limit=10.0,
            stiffness=20.0,
            damping=2.0,
        ),
    },
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
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit=2.0,
            velocity_limit=10.0,
            stiffness=20.0,
            damping=2.0,
        ),
    },
)


def convert_urdf_to_usd(urdf_path, output_dir):
    from isaaclab.sim.converters import UrdfConverterCfg, UrdfConverter

    cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=output_dir,
        force_usd_conversion=True,
        fix_base=False,
        merge_fixed_joints=False,
        make_instanceable=True,
    )

    converter = UrdfConverter(cfg)
    return converter.usd_path
