# Design Notes

This document captures the *why* behind the structure of the env so that future contributors (including future-you) do not have to re-derive the rationale from the code.

## Project Context

This environment is the simulation layer for the **Extended Reality Robot Teleoperation Project**, a collaboration between the SAFE Robotics Lab (Prof. Shreyas Kousik) and Prof. Mohsen Moghaddam's group at Georgia Tech. The downstream pipeline puts a human operator in an XR headset, retargets their pose to one or both G1 humanoids, and streams the resulting joint targets into the simulator (and eventually onto physical hardware).

Locomotion is **explicitly out of scope** for the XR teleop project. The robots are positioned across a workspace, fixed in place, and used as bimanual manipulators. This single fact shapes most of the design decisions below.

## Why two robots in one scene

Putting both robots in a single `InteractiveScene` rather than running two separate envs has three consequences worth being explicit about:

1. **Shared physics context.** Contacts between the two robots are resolved by the same PhysX scene. Two separate envs would not see each other's collision geometry, which makes them useless for any task involving inter-robot interaction (handovers, shared lifts, coordinated assembly).
2. **Shared time.** A single `env.step()` advances both robots by the same `dt`. There is no clock skew. For deployment with two physical G1s this matches reality only when the real machines are time-synchronized; that is a separate engineering problem.
3. **Coupled action and observation dims.** The action vector is the concatenation of both robots' joint targets, and the observation group exposes both robots' proprioception together. This is the "centralized actor, centralized critic" framing — appropriate for teleop (one operator can drive both) and for cooperative MARL with full observability. Less appropriate for decentralized policies.

If you later want decentralized policies, the cleanest path is to keep the env structure but split the action manager into per-robot groups and route them to per-robot policies in the training loop. Do not split the env itself.

## Why the lower body is fixed

The G1 is a full-body humanoid with active legs, but the XR teleop project does not exercise locomotion. Three reasons to fix the base rather than train a stand controller:

1. **Reduced controller failure modes.** A balancing controller adds a whole class of bugs — drift, oscillation, fall recovery — that have nothing to do with the manipulation question the project is trying to answer.
2. **Cleaner sim-to-real transfer for the arms.** When the base does not move, arm joint commands map one-to-one between sim and real. With a moving base, the arm controller has to compensate for body motion, which couples the two problems and makes targeted ablations harder.
3. **Faster iteration.** Fixed base means we can skip the entire crouch-and-balance startup phase and put the operator in front of a manipulation-ready robot the moment the sim launches.

Implementation: `ArticulationRootPropertiesCfg.fix_root_link = True` on the G1 spawn cfg in `configs/dual_g1_scene_cfg.py`. The flag is exposed as `FIX_BASE` at the top of that file so locomotion experiments can flip it without surgery.

## Why ZMQ for teleop

Three things made ZMQ the right pick over the alternatives considered (ROS 2 topics, raw UDP, gRPC, shared memory):

- **Process and language decoupling.** The operator station can be Python, C++, Rust, or anything else with ZMQ bindings. This matters because the project's likely sources of teleop data — VR retargeters running in Unity or Unreal, MoCap pipelines in Python, kinematic mirrors from physical hardware in C++ — are written in different languages by different people. A wire format with no shared schema (just N floats) is the lowest common denominator.
- **No broker.** Unlike message buses that need a running middleware (ROS 2's DDS, MQTT brokers), ZMQ PUB/SUB is point-to-point. Fewer moving parts to start, debug, and clean up.
- **Non-blocking semantics built in.** `RCVTIMEO=1` plus `recv(NOBLOCK)` plus the `CONFLATE` option gives the env loop a clean way to poll without ever stalling, and guarantees we never act on a stale target sitting in a queue.

The cost is that there is no schema, no versioning, no negotiation. If the wire format changes, every operator station breaks silently. The protocol doc (`docs/teleop_protocol.md`) is the only contract; treat it as load-bearing.

## Why absolute joint targets

The action term is configured with `use_default_offset=False` and `scale=1.0`. The operator station sends absolute joint angles in radians and the simulator commands them directly — no scaling, no offset.

The alternative (delta-from-default targets) was rejected because every external retargeter naturally produces absolute joint angles. Forcing them to compute `target - default_pos` requires the operator station to track the default crouch pose, which couples codebases and creates a class of bugs where the two sides disagree on what "default" is. Cleaner to do the subtraction never than to do it twice.

## Why `ManagerBasedEnv` instead of `ManagerBasedRLEnv`

The env currently has no rewards and no terminations. Using `ManagerBasedEnv` instead of `ManagerBasedRLEnv` removes the requirement to define those terms and makes the dependency on reward design explicit: when reward terms are added, the env class swaps to `ManagerBasedRLEnv` and the reward and termination managers are added at that point. This avoids the common failure mode of writing throwaway reward functions just to make the type signature happy and then forgetting they are there.

## Why dynamic joint-group resolution in the teleop

The original teleop hardcoded joint indices (waist = 0..2, left_leg = 3..7, etc.) based on an assumed articulation joint order. Articulation joint order is set by the USD authoring pipeline, not by anything stable, so any change to the G1 asset or the actuator config would silently break the mapping. The current implementation reads `articulation.joint_names` at construction time and builds the index map by regex-matching joint names. The match patterns are the only thing that's hardcoded, and joint naming is much more stable than joint ordering.

## Why the table

The table at `(0.5, 0.0, 0.4)` with size `(1.2, 0.6, 0.8)` is a placeholder workspace. It is `kinematic_enabled=True` so it does not fall and does not respond to contacts — it acts as a static fixture. When real manipulation tasks are added, the table will be replaced or augmented with task-specific objects, and the kinematic flag will flip to `False` for anything the robots are supposed to manipulate.

## Bug Fixes from the Initial Prototype

For history (and because PRs sometimes reintroduce things), these were the bugs in the initial version of this env, all fixed in the current codebase:

1. **`sim_utils` referenced before import.** The `import isaaclab.sim as sim_utils` line was at the bottom of `dual_g1_env_cfg.py`, after the class body that used it. Module evaluation order made this a guaranteed `NameError`. Now imported at the top.
2. **Wrong action term API.** Was `ActionTermCfg(class_type=mdp.JointPositionActionCfg, ...)`. The correct usage is `mdp.JointPositionActionCfg(...)` directly — the Cfg class *is* the term.
3. **ZMQ target convention.** Action term had `use_default_offset=True` while the ZMQ path passed received vectors as absolute targets. The default crouch pose got added on top, producing a constant offset. Now `use_default_offset=False`, ZMQ sends absolute targets.
4. **Keyboard teleop never received key events.** `process_keyboard` was defined but never wired up. `--teleop` mode was functionally identical to standalone. Now subscribes to `carb` keyboard events via `omni.appwindow`.
5. **Wrong joint-group indices.** Hardcoded `left_leg = range(3, 8)` gave 5 indices for a 6-DOF leg. Now derived from joint names at runtime.
6. **Hardcoded DOF count.** Both teleop classes assumed `num_dof_per_robot = 29`. Now read from `articulation.joint_names` and propagated everywhere.
7. **ZMQ stale-message accumulation.** Sockets had no `CONFLATE`, so under sender-faster-than-receiver conditions the queue would grow. Now `CONFLATE = 1` (set before `connect`, which matters).
8. **Standalone action of zeros.** With `use_default_offset=False`, a zero action commands every joint to 0 rad — straight legs, body posture broken. Standalone mode now sends the default pose explicitly.
9. **Broken imports for `configs/` `scripts/` layout.** Flat `from dual_g1_env_cfg import ...` did not survive the move into subdirectories. The launch script now patches `sys.path` to include the repo root, and imports use `from configs.dual_g1_env_cfg import ...`.

## Open Design Questions

- **Hand teleop port assignment.** When the Inspire Hand integration lands, the hand DOF could share the existing port (extending the message size) or use a separate port pair. Separate ports are cleaner for independent operator stations; same port is cleaner for synchronized whole-arm motion. Pick one and document it before writing the operator station.
- **XR retargeting topology.** The XR pipeline currently produces end-effector poses, not joint angles. Either the operator station runs IK and sends joint targets (matches the current ZMQ contract), or we add an IK-target ingress on a separate port. Both are reasonable; the call probably depends on whether we want to control IK solver behavior centrally.
- **Workspace fidelity.** The placeholder table is a single cuboid. The actual XR teleop tasks likely need a more representative workspace (object racks, sorted bins, fixtures). Adding these via a separate `WorkspaceCfg` mixin would keep the env config clean while letting different experiments swap workspaces.
- **Symmetry exploitation.** The two robots are identical articulations spawned at mirrored positions. Future RL work could exploit this either via shared policies or via explicit symmetry rewards. Worth an ablation when training starts.
