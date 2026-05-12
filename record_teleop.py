"""Record ZMQ teleop traffic to a .npz file for later replay or analysis.

Subscribes to the same ports the simulator subscribes to and snapshots whatever
is on the wire. Useful for debugging retargeting bugs offline, generating
reproducible demos, and capturing operator trajectories for imitation learning.

Run in parallel with an active operator station:

    # Terminal 1: simulator
    ./isaaclab.sh -p scripts/run_dual_g1.py --teleop_zmq

    # Terminal 2: operator station
    python scripts/example_zmq_sender.py

    # Terminal 3: recorder
    python scripts/record_teleop.py --output recordings/test.npz

Ctrl-C to stop. The recorder uses CONFLATE, so it only captures the latest
message per polling tick, which matches the simulator's view of the stream.
"""

import argparse
import os
import time

import numpy as np
import zmq


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left_port", type=int, default=5555)
    parser.add_argument("--right_port", type=int, default=5556)
    parser.add_argument("--output", type=str, default="recording.npz")
    parser.add_argument("--rate_hz", type=float, default=50.0,
                        help="Polling rate. Match or exceed the sender's rate.")
    parser.add_argument("--expected_dof", type=int, default=29)
    args = parser.parse_args()

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    ctx = zmq.Context()

    def make_sub(port: int):
        s = ctx.socket(zmq.SUB)
        s.setsockopt(zmq.CONFLATE, 1)
        s.connect(f"tcp://localhost:{port}")
        s.setsockopt_string(zmq.SUBSCRIBE, "")
        s.setsockopt(zmq.RCVTIMEO, 1)
        return s

    left = make_sub(args.left_port)
    right = make_sub(args.right_port)

    expected_bytes = args.expected_dof * 4
    dt = 1.0 / args.rate_hz

    times_left, joints_left = [], []
    times_right, joints_right = [], []

    print(f"recording from tcp://localhost:{args.left_port} and :{args.right_port}")
    print(f"output: {args.output}")
    print("Ctrl-C to stop")

    start_t = time.time()
    try:
        while True:
            now = time.time() - start_t
            try:
                data = left.recv(flags=zmq.NOBLOCK)
                if len(data) == expected_bytes:
                    times_left.append(now)
                    joints_left.append(np.frombuffer(data, dtype=np.float32).copy())
            except zmq.Again:
                pass
            try:
                data = right.recv(flags=zmq.NOBLOCK)
                if len(data) == expected_bytes:
                    times_right.append(now)
                    joints_right.append(np.frombuffer(data, dtype=np.float32).copy())
            except zmq.Again:
                pass
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        joints_left_arr = (
            np.array(joints_left, dtype=np.float32)
            if joints_left
            else np.zeros((0, args.expected_dof), dtype=np.float32)
        )
        joints_right_arr = (
            np.array(joints_right, dtype=np.float32)
            if joints_right
            else np.zeros((0, args.expected_dof), dtype=np.float32)
        )
        np.savez(
            args.output,
            times_left=np.array(times_left, dtype=np.float64),
            joints_left=joints_left_arr,
            times_right=np.array(times_right, dtype=np.float64),
            joints_right=joints_right_arr,
            expected_dof=args.expected_dof,
        )
        print(f"saved {len(times_left)} left, {len(times_right)} right messages to {args.output}")
        left.close()
        right.close()
        ctx.term()


if __name__ == "__main__":
    main()
