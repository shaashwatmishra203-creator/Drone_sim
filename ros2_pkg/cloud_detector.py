#!/usr/bin/env python3
"""
cloud_detector - object detection running in the "cloud".

    /cloud/frame_in (JPEG from the drone, over the emulated uplink)
      -> decode -> detect -> /cloud/detections_out (vision_msgs/Detection2DArray)
                          -> /cloud/annotated      (JPEG with boxes drawn)

Detection is classical computer vision: HSV colour segmentation per object
class, morphological clean-up, then contours to bounding boxes. Classes and
their colours come from config/factory_hall.yaml, the same file the world is
built from. That is deliberate rather than a shortcut. The hall's objects are
synthetic coloured shapes; a pretrained detector (e.g. YOLO on COCO) has never
seen a "shelf_rack" that is a blue box and would not find them, and training
one is a separate project. This detector is honest about what it does: it
finds coloured regions and labels them by colour.

The cloud does NOT know where the drone is. It returns boxes in pixels, a
class and a confidence. Turning a box into a range and bearing needs the
camera's height and attitude at capture time, and that is done on the drone
(hall_mission) with the IMU-derived attitude. The label drawn here shows only a
rough range from the object's known height when the whole object is in view.
"""

import colorsys
import json
import math
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=5)

MIN_AREA_PX = 120          # smaller blobs are noise or objects beyond ~25 m
HUE_TOL = 10               # OpenCV hue units (0-179)
SAT_MIN = 90
VAL_MIN = 25          # shadowed bases are dark; 35 cut them off and biased range long


class ClassModel:
    def __init__(self, name, rgb, height_m):
        self.name = name
        self.height_m = float(height_m)
        h, s, v = colorsys.rgb_to_hsv(*rgb)
        self.hue = h * 180.0              # OpenCV scale
        r, g, b = rgb
        self.draw = (int(b * 255), int(g * 255), int(r * 255))

    def mask(self, hsv):
        lo = self.hue - HUE_TOL
        hi = self.hue + HUE_TOL
        m = cv2.inRange(hsv, (max(lo, 0), SAT_MIN, VAL_MIN),
                        (min(hi, 179), 255, 255))
        if lo < 0:                        # red wraps around 0
            m |= cv2.inRange(hsv, (180 + lo, SAT_MIN, VAL_MIN), (179, 255, 255))
        if hi > 179:
            m |= cv2.inRange(hsv, (0, SAT_MIN, VAL_MIN), (hi - 180, 255, 255))
        return m


class CloudDetector(Node):

    def __init__(self):
        super().__init__("cloud_detector")
        self.declare_parameter("config", "")
        self.declare_parameter("out_jsonl", "/tmp/detections.jsonl")
        cfg = yaml.safe_load(open(self.get_parameter("config").value))
        self.classes = [ClassModel(n, c["rgb"], c["height_m"])
                        for n, c in cfg["classes"].items()]
        cam = cfg["camera"]
        self.fy = (cam["width"] / 2.0) / math.tan(math.radians(cam["hfov_deg"]) / 2.0)
        self.k_open = np.ones((3, 3), np.uint8)
        self.k_close = np.ones((7, 7), np.uint8)
        self.pub_det = self.create_publisher(Detection2DArray,
                                             "/cloud/detections_out", QOS)
        self.pub_ann = self.create_publisher(CompressedImage, "/cloud/annotated", QOS)
        self.create_subscription(CompressedImage, "/cloud/frame_in", self.cb, QOS)
        self.log = open(self.get_parameter("out_jsonl").value, "w")
        self.n = 0
        self.proc_ms = []
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            "cloud detector ready: classes " +
            ", ".join(f"{c.name}(hue {c.hue:.0f})" for c in self.classes))

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        out = []
        for c in self.classes:
            m = c.mask(hsv)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.k_open)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.k_close)
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            H = m.shape[0]
            for k in cnts:
                area = cv2.contourArea(k)
                if area < MIN_AREA_PX:
                    continue
                x, y, w, h = cv2.boundingRect(k)
                fill = area / float(w * h)
                conf = min(1.0, 1.2 * fill) * min(1.0, math.sqrt(area) / 30.0)
                # Ground contact: in several columns, the lowest pixel of THIS
                # object. Where an object meets the floor is what gives range.
                # One range per box (its lowest point) put long walls seen at
                # an angle too close along their whole length, which smeared
                # phantom obstacles into open floor in run 2.
                blob = np.zeros_like(m)
                cv2.drawContours(blob, [k], -1, 255, -1)
                blob &= m
                prof = []
                n_cols = int(min(12, max(2, w // 12)))
                for u in np.linspace(x + 1, x + w - 2, n_cols).astype(int):
                    rows = np.flatnonzero(blob[y:y + h, u])
                    if rows.size == 0:
                        continue
                    vb = y + int(rows[-1])
                    if vb >= H - 3:
                        continue        # touches the frame bottom: no floor contact
                    prof.append((int(u), vb))
                out.append((c, x, y, w, h, conf, prof))
        return out

    def cb(self, msg):
        t0 = time.time()
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        H, W = bgr.shape[:2]
        dets = self.detect(bgr)

        arr = Detection2DArray()
        arr.header = msg.header           # capture time travels end to end
        rec = []
        for i, (c, x, y, w, h, conf, prof) in enumerate(dets):
            d = Detection2D()
            d.header = msg.header
            d.id = f"{c.name}_{i}"
            d.bbox.center.position.x = x + w / 2.0
            d.bbox.center.position.y = y + h / 2.0
            d.bbox.size_x = float(w)
            d.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = c.name
            hyp.hypothesis.score = float(conf)
            d.results.append(hyp)
            # results[1:] carry the ground-contact pixels: class_id
            # "floor_contact", pose.position = (u, v). vision_msgs has no field
            # for a contact profile; this keeps to one standard message.
            for u, vb in prof:
                p = ObjectHypothesisWithPose()
                p.hypothesis.class_id = "floor_contact"
                p.pose.pose.position.x = float(u)
                p.pose.pose.position.y = float(vb)
                d.results.append(p)
            arr.detections.append(d)
            rec.append([c.name, x, y, x + w, y + h, round(conf, 3)])
        self.pub_det.publish(arr)

        # annotated frame for the operator view
        ann = bgr.copy()
        for c, x, y, w, h, conf, prof in dets:
            for u, vb in prof:
                cv2.circle(ann, (u, vb), 3, (255, 255, 255), -1)
                cv2.circle(ann, (u, vb), 3, c.draw, 1)
            cut = y <= 2 or y + h >= H - 3
            rng = "" if cut else f" ~{self.fy * c.height_m / max(h, 1):.1f}m"
            cv2.rectangle(ann, (x, y), (x + w, y + h), (0, 0, 0), 4)
            cv2.rectangle(ann, (x, y), (x + w, y + h), c.draw, 2)
            label = f"{c.name} {conf:.2f}{rng}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            ly = max(y - 4, th + 4)
            cv2.rectangle(ann, (x, ly - th - 4), (x + tw + 4, ly + 2), c.draw, -1)
            cv2.putText(ann, label, (x + 2, ly - 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)
        t_cap = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = (time.time() - t_cap) * 1000
        cv2.rectangle(ann, (0, 0), (W, 22), (30, 30, 30), -1)
        cv2.putText(ann, f"CLOUD CV  frame {self.n}  objects {len(dets)}  "
                    f"uplink age {age:.0f} ms", (6, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        ok, jpg = cv2.imencode(".jpg", ann, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            a = CompressedImage()
            a.header = msg.header
            a.format = "jpeg"
            a.data = jpg.tobytes()
            self.pub_ann.publish(a)

        self.n += 1
        self.proc_ms.append((time.time() - t0) * 1000)
        self.log.write(json.dumps({"t_cap": t_cap, "dets": rec}) + "\n")

    def report(self):
        p = sorted(self.proc_ms[-100:])
        med = p[len(p) // 2] if p else float("nan")
        self.get_logger().info(f"frames {self.n} | detection {med:.1f} ms/frame")
        self.log.flush()


def main():
    rclpy.init()
    n = CloudDetector()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.log.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
