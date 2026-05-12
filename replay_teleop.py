"""Replay a recording made by record_teleop.py back onto the ZMQ ports.

Useful for regression testing the receiver with a known input, debugging
retargeting bugs offline without the operator hardware, and generating
reproducible demos.

    python scripts/replay_teleop.py recordings/test.npz
    python scripts/replay_teleop.py recordings/test.npz --speed 0.5 --loop
"""

import argparse
import time

import numpy as np
import zmq


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", help="Path to .npz file from record_teleop.py")
    parser.add_argument("--left_port", type=int, default=5555)
    parser.add_argument("--right_port", type=int, default=5556)
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier. 1.0 = real time.")
    parser.add_argument("--loop", action="store_true",
                        help="Loop the recording instead of exiting after one pass.")
    args = parser.parse_args()

    data = np.load(args.recording)
    times_l = data["times_left"]
    joints_l = data["joints_left"]
    times_r = data["times_right"]
    joints_r = data["joints_right"]

    last_t = max(
        times_l[-1] if len(times_l) else 0.0,
        times_r[-1] if len(times_r) else 0.0,
    )
    print(f"loaded {len(times_l)} left, {len(times_r)} right messages, {last_t:.1f}s recording")

    ctx = zmq.Context()
    left = ctx.socket(zmq.PUB)
    right = ctx.socket(zmq.PUB)
    left.bind(f"tcp://*:{args.left_port}")
    right.bind(f"tcp://*:{args.right_port}")

    # Brief sleep so SUB sockets on the other end have time to subscribe.
    time.sleep(0.3)

    # Merge into a single timeline so messages go out in the original order.
    events = []
    for t, j in zip(times_l, joints_l):
        events.append((float(t), "left", j))
    for t, j in zip(times_r, joints_r):
        events.append((float(t), "right", j))
    events.sort(key=lambda e: e[0])

    try:
        loop_count = 0
        while True:
            wall_start = time.time()
            for t, side, j in events:
                target_wall = wall_start + t / args.speed
                wait = target_wall - time.time()
                if wait > 0:
                    time.sleep(wait)
                if side == "left":
                    left.send(j.tobytes())
                else:
                    right.send(j.tobytes())
            loop_count += 1
            if not args.loop:
                break
            print(f"loop {loop_count} done, restarting")
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        left.close()
        right.close()
        ctx.term()


if __name__ == "__main__":
    main()
