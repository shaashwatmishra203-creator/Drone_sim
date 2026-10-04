#!/usr/bin/env python3
"""
cloud_link - the drone side of cloud object detection.

    Arducam (gz /nav_camera, 15 fps raw RGB)
      -> JPEG encode on the drone (Pi 5 class work)
      -> UPLINK over an emulated factory WiFi link   -> /cloud/frame_in
                                                         (cloud_detector)
      <- DOWNLINK over the same emulated link        <- /cloud/detections_out
      -> /drone/detections  (what the flight code acts on)

The camera is read straight from Gazebo with the gz.transport13 Python
bindings. The apt ros_gz bridge cannot be used: it is built for Gazebo
Fortress (ignition-transport11) and never sees Harmonic's topics. Raw frames
are also never put on the ROS graph: a 640x360 RGB frame is ~690 KB, and
pushing that through rclpy capped delivery at 4.6 fps. A JPEG is ~30-40 KB,
which is also what a real drone would send.

THE LINK IS EMULATED. Capacity, latency, jitter, loss and outages come from
config/factory_hall.yaml and are stated assumptions, not a measured network.
Both the "cloud" and the drone run on this laptop.

Every frame carries its capture time (header.stamp, wall clock) end to end, so
the flight code can look up where the drone was when the picture was taken,
and so round-trip latency is measured rather than assumed.
"""

import heapq
import json
import random
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=5)

# gz.msgs.Image.PixelFormatType (read from the installed proto, not guessed:
# an earlier bridge mapped 3 to RGBA and produced striped garbage)
CHANNELS = {1: 1, 3: 3, 4: 4, 5: 4, 8: 3}


class EmulatedLink:
    """One direction of a WiFi + WAN path: serialisation at a fixed capacity,
    propagation latency with Gaussian jitter, random loss, scripted outages,
    and tail-drop when more than 0.5 s of data is already queued."""

    def __init__(self, mbps, latency_ms, jitter_ms, loss_pct, outages, t0):
        self.bps = mbps * 1e6
        self.lat = latency_ms / 1000.0
        self.jit = jitter_ms / 1000.0
        self.loss = loss_pct / 100.0
        self.outages = [(float(a), float(b)) for a, b in (outages or [])]
        self.t0 = t0
        self.next_free = 0.0
        self.q = []
        self.seq = 0
        self.sent = self.lost = self.dropped_queue = self.dropped_outage = 0
        self.bytes = 0

    def in_outage(self, now):
        t = now - self.t0
        return any(a <= t < b for a, b in self.outages)

    def send(self, item, nbytes, now):
        self.sent += 1
        if self.in_outage(now):
            self.dropped_outage += 1
            return False
        if random.random() < self.loss:
            self.lost += 1
            return False
        start = max(now, self.next_free)
        if start - now > 0.5:
            self.dropped_queue += 1
            return False
        end = start + nbytes * 8.0 / self.bps
        self.next_free = end
        due = end + max(0.0, random.gauss(self.lat, self.jit))
        self.seq += 1
        self.bytes += nbytes
        heapq.heappush(self.q, (due, self.seq, item))
        return True

    def due(self, now):
        out = []
        while self.q and self.q[0][0] <= now:
            out.append(heapq.heappop(self.q)[2])
        return out

    def stats(self):
        return dict(sent=self.sent, lost=self.lost,
                    dropped_queue=self.dropped_queue,
                    dropped_outage=self.dropped_outage, bytes=self.bytes)


class CloudLink(Node):

    def __init__(self):
        super().__init__("cloud_link")
        self.declare_parameter("config", "")
        self.declare_parameter("gz_topic", "/nav_camera")
        self.declare_parameter("out_json", "/tmp/link_stats.json")
        cfg = yaml.safe_load(open(self.get_parameter("config").value))
        L = cfg["link"]
        self.q = int(L.get("jpeg_quality", 70))
        self.period = 1.0 / float(L.get("send_hz", 10.0))
        self.out_json = self.get_parameter("out_json").value
        now = time.time()
        self.up = EmulatedLink(L["uplink_mbps"], L["one_way_latency_ms"],
                               L["jitter_ms"], L["loss_pct"],
                               L.get("outages_s", []), now)
        self.down = EmulatedLink(L["downlink_mbps"], L["one_way_latency_ms"],
                                 L["jitter_ms"], L["loss_pct"],
                                 L.get("outages_s", []), now)
        self.t_start = now

        self.pub_up = self.create_publisher(CompressedImage, "/cloud/frame_in", QOS)
        self.pub_det = self.create_publisher(Detection2DArray, "/drone/detections", QOS)
        self.create_subscription(Detection2DArray, "/cloud/detections_out",
                                 self.cb_cloud_det, QOS)

        self._lock = threading.Lock()
        self._frame = None            # (capture_wall_time, gz Image)
        self.gz_count = 0
        self.rtt = []                 # capture -> detections back on the drone
        self.frame_bytes = []
        self.last_send = 0.0

        self.gz = GzNode()
        topic = self.get_parameter("gz_topic").value
        if not self.gz.subscribe(GzImage, topic, self._on_gz):
            self.get_logger().fatal(f"cannot subscribe to gz {topic}")
            raise SystemExit(2)
        self.create_timer(0.005, self.tick)      # link scheduler, 200 Hz
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"camera {topic} -> JPEG q{self.q} @ {1/self.period:.0f} Hz -> "
            f"emulated link {L['uplink_mbps']} Mbps, {L['one_way_latency_ms']} "
            f"+/- {L['jitter_ms']} ms, {L['loss_pct']}% loss, "
            f"outages {L.get('outages_s', [])}")

    # gz thread: keep only the newest frame
    def _on_gz(self, msg):
        with self._lock:
            self._frame = (time.time(), msg)
            self.gz_count += 1

    def encode(self, msg):
        ch = CHANNELS.get(msg.pixel_format_type)
        if ch is None:
            return None
        a = np.frombuffer(msg.data, dtype=np.uint8)
        a = a.reshape(msg.height, msg.step)[:, :msg.width * ch]
        a = a.reshape(msg.height, msg.width, ch)
        if ch == 3:
            bgr = a[:, :, ::-1] if msg.pixel_format_type == 3 else a
        elif ch == 4:
            bgr = cv2.cvtColor(a, cv2.COLOR_RGBA2BGR if msg.pixel_format_type == 4
                               else cv2.COLOR_BGRA2BGR)
        else:
            bgr = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
        ok, jpg = cv2.imencode(".jpg", np.ascontiguousarray(bgr),
                               [cv2.IMWRITE_JPEG_QUALITY, self.q])
        return jpg.tobytes() if ok else None

    def tick(self):
        now = time.time()
        if now - self.last_send >= self.period:
            with self._lock:
                fr, self._frame = self._frame, None
            if fr is not None:
                self.last_send = now
                t_cap, msg = fr
                jpg = self.encode(msg)
                if jpg:
                    m = CompressedImage()
                    sec = int(t_cap)
                    m.header.stamp.sec = sec
                    m.header.stamp.nanosec = int((t_cap - sec) * 1e9)
                    m.header.frame_id = "nav_camera"
                    m.format = "jpeg"
                    m.data = jpg
                    self.frame_bytes.append(len(jpg))
                    self.up.send(m, len(jpg), now)
        for m in self.up.due(now):
            self.pub_up.publish(m)
        for d in self.down.due(now):
            t_cap = d.header.stamp.sec + d.header.stamp.nanosec * 1e-9
            self.rtt.append(now - t_cap)
            self.pub_det.publish(d)

    def cb_cloud_det(self, msg):
        # ~60 bytes per box plus ~24 per ground-contact point, plus a header
        nb = 64 + sum(60 + 24 * max(0, len(d.results) - 1) for d in msg.detections)
        self.down.send(msg, nb, time.time())

    def report(self):
        el = time.time() - self.t_start
        r = sorted(self.rtt[-200:])
        med = r[len(r) // 2] * 1000 if r else float("nan")
        p95 = r[int(len(r) * 0.95)] * 1000 if r else float("nan")
        fb = self.frame_bytes[-50:]
        kb = (sum(fb) / len(fb) / 1024) if fb else 0
        self.get_logger().info(
            f"camera {self.gz_count / max(el, 1e-6):.1f} fps in | up "
            f"{self.up.sent} sent {self.up.lost} lost {self.up.dropped_outage} "
            f"outage | frame {kb:.0f} KB | round trip median {med:.0f} ms "
            f"p95 {p95:.0f} ms")
        self.write()

    def write(self):
        r = sorted(self.rtt)
        doc = {
            "note": "EMULATED link - parameters are assumptions from "
                    "config/factory_hall.yaml, not a measured network",
            "uplink": self.up.stats(), "downlink": self.down.stats(),
            "frames_encoded": len(self.frame_bytes),
            "mean_frame_kb": (sum(self.frame_bytes) / len(self.frame_bytes) / 1024)
            if self.frame_bytes else None,
            "round_trip_ms": {
                "n": len(r),
                "median": r[len(r) // 2] * 1000 if r else None,
                "p95": r[int(len(r) * 0.95)] * 1000 if r else None,
                "max": r[-1] * 1000 if r else None},
        }
        try:
            with open(self.out_json, "w") as f:
                json.dump(doc, f, indent=2)
        except OSError:
            pass


def main():
    rclpy.init()
    n = CloudLink()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.write()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
