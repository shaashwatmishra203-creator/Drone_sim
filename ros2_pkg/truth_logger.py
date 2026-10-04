#!/usr/bin/env python3
"""
truth_logger - the drone's TRUE pose, read from Gazebo itself.

Every clearance, collision and "did it fly through the course" figure in the
v2 report is computed from this file, never from PX4's own position estimate.
That independence is the point: before v2, the waypoints, the simulated
sensor and the clearance check all used PX4's frame, all made the same frame
mistake, and agreed with each other while the real aircraft flew somewhere
else. A measurement that shares the system's assumptions cannot catch them.

Writes truth.csv: wall_t, x, y, z (Gazebo world ENU), qw, qx, qy, qz.
"""

import argparse
import sys
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="factory_hall")
    ap.add_argument("--model", default="n360_quad_9x45_pi5_0")
    ap.add_argument("--out", default="/tmp/truth.csv")
    ap.add_argument("--hz", type=float, default=20.0)
    a, _ = ap.parse_known_args()

    f = open(a.out, "w")
    f.write("t,x,y,z,qw,qx,qy,qz\n")
    lock = threading.Lock()
    st = {"last": 0.0, "n": 0}
    period = 1.0 / a.hz

    def cb(msg):
        now = time.time()
        if now - st["last"] < period:
            return
        for p in msg.pose:
            if p.name == a.model:
                o, q = p.position, p.orientation
                with lock:
                    f.write(f"{now:.3f},{o.x:.4f},{o.y:.4f},{o.z:.4f},"
                            f"{q.w:.5f},{q.x:.5f},{q.y:.5f},{q.z:.5f}\n")
                    st["last"] = now
                    st["n"] += 1
                break

    node = Node()
    topic = f"/world/{a.world}/dynamic_pose/info"
    if not node.subscribe(Pose_V, topic, cb):
        print(f"cannot subscribe to {topic}", file=sys.stderr)
        sys.exit(2)
    print(f"truth_logger: {topic} -> {a.out} ({a.model}, {a.hz:.0f} Hz)", flush=True)
    try:
        while True:
            time.sleep(1.0)
            with lock:
                f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        with lock:
            f.close()
        print(f"truth_logger: {st['n']} samples", flush=True)


if __name__ == "__main__":
    main()
