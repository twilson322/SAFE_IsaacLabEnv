# ZMQ Teleop Protocol

This document specifies the wire format used by `ZMQTeleopReceiver` in `scripts/dual_g1_teleop.py`. It is intended for anyone writing an external operator station — a VR rig, Apple Vision Pro retargeter, kinematic mirror from a physical G1, MoCap pipeline, or scripted trajectory player — that needs to drive the simulated robots.

## Architecture

```
  ┌────────────────────────┐                    ┌────────────────────────┐
  │ Operator Station (left)│ ─── tcp:5555 ────▶ │                        │
  └────────────────────────┘                    │   run_dual_g1.py       │
                                                │   --teleop_zmq         │
  ┌────────────────────────┐                    │                        │
  │ Operator Station (right)│ ── tcp:5556 ───▶ │                        │
  └────────────────────────┘                    └────────────────────────┘
```

The simulator runs **two SUB sockets**, one per robot. Each operator station runs a **PUB socket**. This is intentional: the two arms can be driven by independent processes (or by independent humans), and a slow or crashed left-side teleop does not stall the right-side robot. The non-blocking `RCVTIMEO=1` on the simulator side means the env loop never waits more than 1 ms for new targets — if no message has arrived, the most recent target is held.

## Wire Format

Each message is a single ZMQ frame containing 29 little-endian IEEE-754 single-precision floats, packed contiguously, no header.

| Field          | Value                                          |
|----------------|------------------------------------------------|
| Transport      | TCP                                            |
| Pattern        | PUB / SUB                                      |
| Topic filter   | empty (subscribe to all messages)              |
| Message size   | 29 × 4 = 116 bytes                             |
| Element type   | `numpy.float32` (little-endian)                |
| Element units  | radians, joint position targets                |
| Element order  | G1 articulation joint order (see below)        |

If a message of the wrong size arrives, the simulator silently ignores it and reuses the previous target. There is no negative acknowledgement and no version handshake. Operator stations should treat the link as fire-and-forget.

## Joint Order

The simulator uses the joint order returned by Isaac Lab's `Articulation` for a G1 with the `G1_MINIMAL_CFG` actuator set. **This order has not been audited for the configuration in this repo** — you must verify it against your build. After the env is constructed, print:

```python
print(env.scene["robot_left"].joint_names)
```

and copy that ordering into your operator station. The `joint_groups` dict in `DualG1Teleop` is a working assumption, not a verified mapping.

The 29 DOF nominally cover: waist (3), left leg (6), right leg (6), left arm (7), right arm (7). Hand DOF are not included in this 29-vector — the Inspire Hand integration will live on a separate articulation with its own ZMQ port (TBD).

## Sender Reference Implementation

A minimal Python sender for the left robot:

```python
import time
import numpy as np
import zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://*:5555")

# Hold a stable pose for 5 seconds at 100 Hz.
target = np.zeros(29, dtype=np.float32)
target[0] = 0.0          # waist_yaw
target[13] = 0.3         # left_shoulder_pitch (example index — VERIFY)

dt = 0.01
for _ in range(500):
    sock.send(target.tobytes())
    time.sleep(dt)
```

Note `bind` on the sender and `connect` on the receiver: the receiver in this repo is the **subscriber**, so the publisher binds. If you reverse this (sender connects, receiver binds), nothing crashes — both sides will sit silent.

## Target Convention

**This is the most important section to read.**

The action term in `dual_g1_env_cfg.py` is configured with `scale=1.0` and `use_default_offset=True`. With those flags, Isaac Lab interprets each action as a **delta added to the default joint position**: the commanded joint target is `default_pos + scale * action`.

The ZMQ path in `run_dual_g1.py` currently feeds the received vector directly into `env.step` as the action. If your sender computes absolute joint targets in radians (the natural choice for kinematic retargeting), the simulator will add the default crouch pose offset on top, and the robot will not go where you commanded.

There are two acceptable fixes; pick one and document it in your operator station:

1. **Sender sends deltas.** Compute `delta = target_absolute - default_pos` on the operator side and send the delta. This requires the operator station to know the default pose, which couples the two codebases.
2. **Set `use_default_offset=False` in the action term.** The sender can then send absolute targets, which is the convention most external teleop systems expect. This is the recommended fix.

Until one of these is done, expect a constant offset between commanded and actual joint angles.

## Frequency Expectations

The sim loop runs at 120 Hz physics, 30 Hz control after decimation. Sending faster than 30 Hz wastes CPU but is harmless — the SUB socket queues the latest message and the env consumes one per step. Sending slower than 30 Hz means the previous target is held; if the previous target was a midpoint of a fast trajectory the robot will stop there. Aim for sender rate ≥ control rate.

## Failure Modes

| Symptom                                | Likely cause                                                   |
|----------------------------------------|----------------------------------------------------------------|
| Robot snaps to crouch and holds        | Sender stopped publishing; receiver holding last (zero) target |
| Robot drifts slowly to wrong pose      | `use_default_offset` issue above                               |
| Robot ignores commands entirely        | Wrong port, wrong message size, or PUB/SUB bind/connect swap   |
| Both robots move when you only sent left | Wrong port; check for transposed `--left_port` / `--right_port` |
| Joints move but the wrong ones         | Joint-order mismatch; print `joint_names` and verify           |

## Security

There is none. The transport is plain TCP on localhost. Do not expose the ports to a network you do not trust. If you need to run the operator station on a different machine, tunnel the ports through SSH rather than binding to a public interface.
