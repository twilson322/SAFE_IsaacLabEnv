"""RL variant of the dual G1 env.

Same scene, actions, observations, and events as DualG1EnvCfg. Adds reward and
termination managers and inherits from ManagerBasedRLEnvCfg instead of
ManagerBasedEnvCfg. Use ManagerBasedRLEnv to instantiate.

The rewards here are stubs: joint velocity and action rate penalties to give a
trainer something to optimize. Task-specific rewards (object lifted, target
reached, etc.) belong alongside these, not in place of them.
"""

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from isaaclab.utils import configclass

from configs.dual_g1_env_cfg import ActionsCfg, EventCfg, ObservationsCfg
from configs.dual_g1_scene_cfg import DualG1SceneCfg


@configclass
class RewardsCfg:
    """Smoothness / energy rewards only. Add task rewards below these."""

    joint_vel_penalty_left = RewardTermCfg(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot_left")},
    )
    joint_vel_penalty_right = RewardTermCfg(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot_right")},
    )
    action_rate_penalty = RewardTermCfg(
        func=mdp.action_rate_l2,
        weight=-1e-3,
    )


@configclass
class TerminationsCfg:
    """Episode terminations. Task-specific terminations belong here too."""

    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)


@configclass
class DualG1RLEnvCfg(ManagerBasedRLEnvCfg):
    """Dual G1 fixed-base env with RL managers attached."""

    scene: DualG1SceneCfg = DualG1SceneCfg(num_envs=1, env_spacing=5.0)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
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
