# Inspire Hand Setup

This walkthrough covers converting the Inspire dexterous hand from URDF to USD, registering the resulting articulation in the env, and attaching it to the G1 wrist frames. Hand integration is **not yet wired up end-to-end** — `configs/inspire_hand_cfg.py` is a stub with conservative placeholder gains. This document is the path forward.

## Prerequisites

1. Inspire Robotics URDF for the hand model in lab. Confirm the URDF matches the physical hand — Inspire ships several variants and the actuation modes (active vs coupled DOF counts) differ between them.
2. Mesh files referenced by the URDF (typically STL or OBJ in a `meshes/` subdirectory).
3. Isaac Lab installed with the `isaaclab.sim.converters` module available (it is, in 2.3+).

Place the URDF + meshes under `assets/inspire_hand_left/` and `assets/inspire_hand_right/`. If the left and right hands share a single mirrored URDF, run the converter twice with different output dirs and write the mirrored asset on the second pass.

## Step 1: URDF → USD

`configs/inspire_hand_cfg.py` ships a thin wrapper around Isaac Lab's URDF converter:

```python
from configs.inspire_hand_cfg import convert_urdf_to_usd

convert_urdf_to_usd(
    urdf_path="assets/inspire_hand_left/inspire_hand_left.urdf",
    output_dir="assets/inspire_hand_left/usd/",
)
```

The converter is configured with:

- `fix_base=False` — the hand will be attached to the G1 wrist via a fixed joint, so its root must be a free body until that joint is created.
- `merge_fixed_joints=False` — keeps the fingertip frames addressable for contact sensors and IK targets. Set to `True` only if you do not need them and you want a smaller articulation.
- `make_instanceable=True` — required for vectorized envs (`--num_envs > 1`).

The output is a `.usd` file plus a `.usda` description in `output_dir`. The function returns the USD path; paste it into the config in the next step.

## Step 2: Update the Hand Config

Open `configs/inspire_hand_cfg.py` and replace the placeholder strings:

```python
INSPIRE_HAND_LEFT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="assets/inspire_hand_left/usd/inspire_hand_left.usd",   # was REPLACE_WITH_...
        ...
```

Do the same for the right hand. If you keep the assets in a non-default location, use an absolute path or a path resolved from a project-root constant to avoid breakage when launching from different working directories.

## Step 3: Tune Actuator Gains

The defaults in the stub are conservative:

```python
ImplicitActuatorCfg(
    joint_names_expr=[".*"],
    effort_limit=2.0,        # N·m
    velocity_limit=10.0,     # rad/s
    stiffness=20.0,          # N·m/rad
    damping=2.0,             # N·m·s/rad
)
```

These are placeholders, not characterized values. After the hand loads, drive a sinusoid into one finger joint and watch the response — if it overshoots and rings, raise damping; if it lags, raise stiffness; if it clips at the effort ceiling during normal motion, raise `effort_limit`. Inspire publishes nominal torque limits in their datasheet; use those as upper bounds, not setpoints.

If your hand has coupled tendons (one motor driving multiple joints through a linkage), the URDF will represent each coupled joint as its own DOF. You will need to either constrain them in the config (with a `JointPositionActionCfg` that maps one command to several joints) or accept that the simulated hand will be over-actuated relative to the physical one. For the XR teleop project this matters because the operator's hand pose retargeter probably produces motor-space commands, not joint-space ones.

## Step 4: Attach to G1 Wrist Frames

This is the part that does not exist yet. The plan:

1. Add `hand_left` and `hand_right` as `ArticulationCfg` entries in `DualG1SceneCfg`, alongside `robot_left` and `robot_right`. Spawn them with an initial pose roughly at the G1 wrist location — exact values do not matter because step 2 immediately overrides them.
2. After `InteractiveScene` is built, in env `__post_init__` or in a new event term that runs at startup, create a USD `FixedJoint` between the G1 wrist link (`left_wrist_yaw_link` for the left arm, `right_wrist_yaw_link` for the right) and the hand root. Isaac Sim's `omni.isaac.core.utils.stage` and `omni.isaac.core.utils.prims` modules expose helpers for this.
3. The transform between wrist and hand base is the mounting bracket geometry. Measure this off the CAD or the physical mount; do not eyeball it.

A working reference for this pattern is in the Unitree `unitree_sim_isaaclab` repo under their bimanual examples — see how they attach grippers to the H1 wrist. The pattern translates directly to G1 + Inspire.

The fixed-base G1 simplifies this slightly: because the G1 pelvis is welded to the world, the wrist link transforms relative to the world frame are stable, which makes the initial alignment of the hand root much easier to debug than it would be on a balancing robot.

## Step 5: Wire the Hand Into the Action Manager

Add hand action terms in `dual_g1_env_cfg.py`:

```python
joint_pos_hand_left = mdp.JointPositionActionCfg(
    asset_name="hand_left",
    joint_names=[".*"],
    scale=1.0,
    use_default_offset=False,
)
# ...and the same for hand_right
```

The total action dim will grow from `(arm_dof × 2)` to `(arm_dof + hand_dof) × 2`. Bump `expected_dof` in any operator station code accordingly, and decide whether the hand DOF share a ZMQ port with the arm or get their own (see `docs/design_notes.md` open questions).

## Step 6: Verify

A minimal check before adding the hand to teleop:

```bash
./isaaclab.sh -p scripts/run_dual_g1.py --num_envs 1
```

In the viewport, the hand should ride with the wrist when the arm moves and respond to gravity sensibly when the arm holds still. If the hand floats free, the fixed joint did not bind. If the arm jerks or the sim explodes, the fixed joint is fighting the wrist actuator — check that the hand root has `kinematic_enabled=False` and that you are not double-attaching.

## Status Summary

| Step                       | State        |
|----------------------------|--------------|
| URDF converter wrapper     | Done         |
| Stub `INSPIRE_HAND_*_CFG`  | Done         |
| USD paths populated        | **TODO**     |
| Wrist fixed-joint creation | **TODO**     |
| Hand DOF in env action dim | **TODO**     |
| Coupled-tendon handling    | **TODO**     |
| Teleop port for hand DOF   | **TODO**     |
