# Design Notes

This document captures the *why* behind the structure of the env, so that future contributors (including future-you) do not have to re-derive the rationale from the code. It also lists known issues and open questions that the README intentionally summarizes.

## Why two robots in one scene

The scientific motivation is bimanual and dual-agent humanoid research: handovers, coordinated lifts, contact-rich tasks where two arms (here, two whole humanoids) need to negotiate the same workspace. Putting both robots in a single `InteractiveScene` rather than running two separate envs has three consequences worth being explicit about:

1. **Shared physics context.** Contacts between the two robots are resolved by the same PhysX scene. Two separate envs would not see each other's collision geometry, which makes them useless for any task involving inter-robot interaction.
2. **Shared time.** A single `env.step()` advances both robots by the same `dt`. There is no clock skew. For sim-to-real with two physical G1s this matches reality only when the real machines are time-synchronized; that is a separate engineering problem.
3. **Coupled action and observation dims.** The action vector is the concatenation of both robots' joint targets, and the observation group exposes both robots' proprioception together. This is the "centralized critic, centralized actor" framing — appropriate for teleop and for cooperative MARL with full observability, less appropriate for decentralized policies.

If you later want decentralized policies, the cleanest path is to keep the env structure but split the action manager into per-robot groups and route them to per-robot policies in the training loop. Do not split the env.

## Why ZMQ for teleop

Three things made ZMQ the right pick over the alternatives considered (ROS 2 topics, raw UDP, gRPC, shared memory):

- **Process and language decoupling.** The operator station can be Python, C++, Rust, or anything with ZMQ bindings. This matters because the lab's likely sources of teleop data — VR retargeters, MoCap pipelines, kinematic mirrors from physical hardware — are written in different languages by different people. A wire format with no shared schema (just 29 floats) is the lowest common denominator.
- **No broker.** Unlike message buses that need a running middleware (ROS 2's DDS, MQTT brokers), ZMQ PUB/SUB is point-to-point. Fewer moving parts to start, debug, and clean up.
- **Non-blocking semantics built in.** `RCVTIMEO=1` plus `recv(NOBLOCK)` gives the env loop a clean way to poll without ever stalling. Replicating this on raw sockets is doable but error-prone.

The cost is that there is no schema, no versioning, no negotiation. If the wire format changes, every operator station breaks silently. The protocol doc (`docs/teleop_protocol.md`) is the only contract; treat it as load-bearing.

## Why `ManagerBasedEnv` instead of `ManagerBasedRLEnv`

The env currently has no rewards and no terminations. Using `ManagerBasedEnv` instead of `ManagerBasedRLEnv` removes the requirement to define those terms and makes the dependency on reward design explicit: when reward terms are added, the env class swaps to `ManagerBasedRLEnv` and the reward and termination managers are added at that point. This avoids the common failure mode of writing throwaway reward functions just to make the type signature happy and then forgetting they are there.

## Why the table

The table at `(0.5, 0.0, 0.4)` with size `(1.2, 0.6, 0.8)` is a placeholder workspace. It is `kinematic_enabled=True` so it does not fall and does not respond to contacts — it acts as a static fixture. When real manipulation tasks are added, the table will be replaced or augmented with task-specific objects, and the kinematic flag will likely flip to `False` for any object the robots are supposed to manipulate.

## Why the default crouch pose

The initial joint positions in `dual_g1_scene_cfg.py` (-0.2 hip pitch, 0.4 knee, -0.2 ankle pitch, mild shoulder and elbow flexion) put the robot in a slightly squatted, arms-forward stance. Two reasons:

- **Stability at spawn.** A fully extended G1 spawned 1.05 m off the ground tends to stumble before the first action arrives; the slight crouch broadens the support polygon and lowers the COM.
- **Reasonable starting configuration for arm work.** Arms forward, elbows bent, is closer to a manipulation-ready pose than arms-down-at-sides, which would require a long warmup motion.

This pose is the value of `default_pos` that the action term's `use_default_offset=True` adds to. See "Action target convention" in known issues below.

## Known Issues

These are the bugs and rough edges in the current code that any user cloning the repo will hit.

**1. `sim_utils` is referenced before it is imported in `dual_g1_env_cfg.py`.**
The `import isaaclab.sim as sim_utils` line is at the bottom of the file, but `sim_utils.SimulationCfg(...)` and `sim_utils.RigidBodyMaterialCfg(...)` appear in the class body of `DualG1EnvCfg`. The class body is evaluated at class-definition time, before the bottom-of-file import runs, so this raises `NameError` on import. Move the import to the top of the file.

**2. ZMQ teleop sends absolute targets but the action term applies a default offset.**
`ActionsCfg` sets `use_default_offset=True` and `scale=1.0`. Isaac Lab interprets actions as `target = default_pos + scale * action`. The ZMQ path passes the received vector as `action` directly. If the operator station sends absolute joint angles (the natural convention), the default crouch pose offset gets added on top. Either set `use_default_offset=False` or convert sender output to deltas. See `docs/teleop_protocol.md` for discussion.

**3. Joint-group indices in `DualG1Teleop` are not verified.**
The `joint_groups` dict in `dual_g1_teleop.py` assumes a specific ordering: waist 0–2, left leg 3–7, right leg 8–12, left arm 13–19, right arm 20–26, left hand 27–28. This was written from assumption, not from inspecting `articulation.joint_names` at runtime. Before relying on the keyboard teleop for anything, print the joint names and update the indices.

**4. Keyboard teleop has no actual keyboard hookup.**
`DualG1Teleop.process_keyboard` exists but is never called from the env loop in `run_teleop_keyboard`. The function `run_teleop_keyboard` constructs the teleop and calls `get_action()`, but no key events ever feed into `process_keyboard`. To make the keyboard mode functional, attach Isaac Lab's `Se3Keyboard` or a similar input device, or use Carbonite's input subscriptions. As written, the keyboard mode is equivalent to the standalone idle mode.

**5. ZMQ socket polarity.**
The receiver uses `connect`. Senders therefore must `bind`. This is documented in the teleop protocol doc but is the opposite of what most ZMQ tutorials show, and it is easy to get backwards. If both sides connect, both sides sit silent forever.

**6. Imports break if files move into `configs/` and `scripts/` subdirectories.**
The current files use flat imports (`from dual_g1_env_cfg import DualG1EnvCfg`). Moving them into subdirectories will break those imports. Either keep the files at the repo root, add a `pyproject.toml` and install the package in editable mode, or have each script add its sibling directories to `sys.path` at the top. The scripts in this repo currently assume the first option (flat); update them when you commit the new layout.

**7. Action dimensionality assumption.**
`run_teleop_zmq` and `DualG1Teleop` both hardcode `num_dof_per_robot = 29`. If `G1_MINIMAL_CFG` exposes a different number of joints in your Isaac Lab version, this will silently produce a mismatched action vector. Read it from `env.action_manager` instead: `total_action_dim` divided by 2 robots, or per-term dims if you separate them.

**8. `ManagerBasedEnv` may need additional manager terms.**
Some Isaac Lab versions require at least a `CommandsCfg` or a non-empty event manager for construction. The current event manager has reset terms only, which usually suffices, but if you upgrade Isaac Lab and construction fails, this is a likely culprit.

## Open Design Questions

- **Hand teleop port assignment.** When the Inspire Hand integration lands, the hand DOF could share the existing port (extending the message size) or use a separate port pair. Separate ports are cleaner for independent operator stations; same port is cleaner for synchronized whole-arm motion. Pick one and document it before writing the operator station.
- **Reward shaping for sim-to-real.** The eventual policy training will need rewards that transfer. Position tracking + smoothness penalties is the obvious starting point; whether to add explicit symmetry rewards (since the two robots are identical) is an open question worth a small ablation.
- **Domain randomization scope.** Friction and mass randomization are standard; whether to also randomize the base spawn pose, the table height, or inter-robot spacing depends on what variation the deployed setup needs to handle. The lab's actual deployment constraints should drive this.
