# ZMQ Teleop Protocol

This document specifies the wire format used by `ZMQTeleopReceiver` in `scripts/dual_g1_teleop.py`. It is the contract between this simulator and any external operator station — XR retargeter, MoCap pipeline, kinematic mirror from a physical G1, scripted trajectory player — that needs to drive the simulated robots.

## Architecture

```
  ┌─────────────────────────────┐                  ┌────────────────────────┐
  │ Operator Station (left)     │ ─── tcp:5555 ──▶ │                        │
  └─────────────────────────────┘                  │   run_dual_g1.py       │
                                                   │   --teleop_zmq         │
  ┌─────────────────────────────┐                  │                        │
  │ Operator Station (right)    │ ─── tcp:5556 ──▶ │                        │
  └─────────────────────────────┘                  └────────────────────────┘
```

The simulator runs **two SUB sockets**, one per robot. Each operator station runs a **PUB socket**. This is intentional: the two arms can be driven by independent processes (or by independent humans), and a slow or crashed left-side teleop does not stall the right-side robot. `RCVTIMEO=1` on the simulator side means the env loop never waits more than 1 ms for new targets — if no message has arrived, the most recent target is held.

`CONFLATE=1` is set on each SUB socket before `connect()`, so the socket buffer only ever holds the most recent message. Stale targets are useless for control and silently dropped.

## Wire Format

Each message is a single ZMQ frame containing N little-endian IEEE-754 single-precision floats, packed contiguously, no header. N is the number of joints in the per-robot articulation (29 for a stock G1).

| Field          | Value                                          |
|----------------|------------------------------------------------|
| Transport      | TCP                                            |
| Pattern        | PUB / SUB                                      |
| Topic filter   | empty (subscribe to all messages)              |
| Message size   | `expected_dof × 4` bytes                       |
| Element type   | `numpy.float32` (little-endian)                |
| Element units  | **radians, absolute joint position targets**   |
| Element order  | G1 articulation joint order (see below)        |

If a message of the wrong size arrives, the simulator silently ignores it and reuses the previous target. There is no negative acknowledgement and no version handshake. Operator stations should treat the link as fire-and-forget.

## Target Convention

The simulator's action term uses `use_default_offset=False, scale=1.0`. The vector you send is treated as the absolute joint position target in radians — no offset is added, no scaling is applied. If you want joint 0 to go to 0.3 rad, send 0.3.

This is the key change from earlier versions of this env. If you have an old operator station that was sending deltas from the default pose, it will now command those deltas as absolute angles and produce nonsense motion. Update the sender to emit absolute targets.

## Joint Order

The simulator uses the joint order returned by Isaac Lab's `Articulation`. **The order depends on your asset build and is not guaranteed stable across Isaac Lab versions.** Print it before you write the sender:

```python
print(env.scene["robot_left"].joint_names)
```

For a stock G1 with `G1_MINIMAL_CFG`, the 29 DOF cover: waist (3), left leg (6), right leg (6), left arm (7), right arm (7). Hand DOF are not part of this 29-vector — the Inspire Hand integration, when it lands, will use a separate articulation with its own teleop ingress (port TBD).

## Fixed-Base Implications

The robots' lower bodies are fixed. Leg joint targets still appear in the action vector (they are part of the articulation), but commanding them does nothing structurally meaningful — the root is welded to the world. For senders, the simplest convention is to send the default crouch values for the leg joints and only move the upper-body indices. The teleop class follows this pattern.

If you flip `FIX_BASE = False` in `dual_g1_scene_cfg.py` to enable locomotion, leg targets become live and the sender becomes responsible for keeping the robot upright.

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
# 29-DOF G1: indices below are illustrative — VERIFY with joint_names printout.
target = np.zeros(29, dtype=np.float32)
target[13] = 0.3          # left_shoulder_pitch (illustrative)
target[14] = 0.0          # left_shoulder_roll
# ... fill remaining indices with default pose values

dt = 0.01
for _ in range(500):
    sock.send(target.tobytes())
    time.sleep(dt)
```

Note `bind` on the sender and `connect` on the receiver: the receiver in this repo is the **subscriber**, so the publisher binds. If you reverse this (sender connects, receiver binds), nothing crashes — both sides will sit silent.

## Frequency Expectations

The sim loop runs at 120 Hz physics, 30 Hz control after decimation. Sending faster than 30 Hz wastes CPU but is harmless — `CONFLATE` makes the SUB socket keep only the latest message and the env consumes one per step. Sending slower than 30 Hz means the previous target is held; if the previous target was a midpoint of a fast trajectory the robot will stop there. Aim for sender rate ≥ control rate.

## Failure Modes

| Symptom                                | Likely cause                                                   |
|----------------------------------------|----------------------------------------------------------------|
| Robot snaps to a weird pose and holds  | Sender publishing absolute zeros — every joint commanded to 0  |
| Robot drifts slowly to wrong pose      | Old sender still subtracting default; remove that subtraction  |
| Robot ignores commands entirely        | Wrong port, wrong message size, or PUB/SUB bind/connect swap   |
| Both robots move when you only sent left | Transposed `--left_port` / `--right_port`                    |
| Joints move but the wrong ones         | Joint-order mismatch; print `joint_names` and verify           |
| Targets seem laggy                     | Sender rate < 30 Hz, or `CONFLATE` not set on sender's own bus |

## Security

There is none. The transport is plain TCP on localhost. Do not expose the ports to a network you do not trust. If the XR operator station runs on a different machine than the simulator, tunnel the ports through SSH rather than binding to a public interface.
