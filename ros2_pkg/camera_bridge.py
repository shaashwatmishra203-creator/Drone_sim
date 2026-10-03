#!/usr/bin/env python3
"""
camera_bridge — put the Arducam's images onto the ROS 2 graph.

ros_gz_bridge is not installed and installing it needs sudo, but the Gazebo
transport Python bindings (gz.transport13 / gz.msgs10) ARE present, so this
node subscribes to the gz camera topic directly and republishes as
sensor_msgs/Image. Same result, no package install.

    Gazebo  /nav_camera  --(gz transport)-->  this node
            -->  /drone/nav_camera/image_raw   (sensor_msgs/Image)
            -->  /drone/nav_camera/camera_info (sensor_msgs/CameraInfo)

The camera is MONOCULAR. It gives bearing to features, not range. Anything
needing distance has to infer it — motion parallax, learned monocular depth, or
objects of known size.
"""

import math
import os
import sys
import threading

import rclpy
import yaml
from rclpy.node import Node as RclNode
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
except ImportError as e:
    print(f"gz transport python bindings unavailable: {e}", file=sys.stderr)
    raise

# Images are bulky and a dropped frame is not worth retransmitting.
IMG_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=2,
)

# gz Image.pixel_format_type -> ROS encoding
PIXFMT = {1: "l8", 2: "rgb8", 3: "rgba8", 4: "bgra8", 5: "bgr8", 6: "mono16"}


class CameraBridge(RclNode):

    def __init__(self):
        super().__init__("camera_bridge")
        self.declare_parameter("config", "")
        self.declare_parameter("gz_topic", "/nav_camera")
        self.declare_parameter("frame_id", "nav_camera")
        self.declare_parameter("out_topic", "/drone/nav_camera/image_raw")

        cfg_path = self.get_parameter("config").value
        self.cam_cfg = {}
        if cfg_path and os.path.exists(cfg_path):
            self.cam_cfg = yaml.safe_load(open(cfg_path)).get("nav_camera", {})

        self.frame_id = self.get_parameter("frame_id").value
        out = self.get_parameter("out_topic").value

        self.pub_img = self.create_publisher(Image, out, IMG_QOS)
        self.pub_info = self.create_publisher(
            CameraInfo, out.rsplit("/", 1)[0] + "/camera_info", IMG_QOS)

        self._lock = threading.Lock()
        self._latest = None
        self._count = 0
        self._last_report = 0

        self.gz = GzNode()
        gz_topic = self.get_parameter("gz_topic").value
        if not self.gz.subscribe(GzImage, gz_topic, self._on_gz_image):
            self.get_logger().fatal(f"could not subscribe to gz topic {gz_topic}")
            raise SystemExit(2)

        # gz delivers on its own thread; publish from a ROS timer so rclpy is
        # only ever touched from one thread.
        self.create_timer(1.0 / 60.0, self._publish)
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"bridging gz '{gz_topic}' -> '{out}' "
            f"({self.cam_cfg.get('width','?')}x{self.cam_cfg.get('height','?')} "
            f"@ {self.cam_cfg.get('fps','?')} fps, "
            f"{self.cam_cfg.get('hfov_deg','?')} deg HFOV)")

    # -- gz thread ---------------------------------------------------------
    def _on_gz_image(self, msg):
        with self._lock:
            self._latest = msg
            self._count += 1

    # -- ros thread --------------------------------------------------------
    def _publish(self):
        with self._lock:
            msg = self._latest
            self._latest = None
        if msg is None:
            return

        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.frame_id
        img.width = msg.width
        img.height = msg.height
        img.encoding = PIXFMT.get(msg.pixel_format_type, "rgb8")
        img.is_bigendian = 0
        img.step = msg.step
        img.data = msg.data
        self.pub_img.publish(img)
        self.pub_info.publish(self._camera_info(msg))

    def _camera_info(self, msg):
        ci = CameraInfo()
        ci.header.stamp = self.get_clock().now().to_msg()
        ci.header.frame_id = self.frame_id
        ci.width = msg.width
        ci.height = msg.height
        hfov = math.radians(float(self.cam_cfg.get("hfov_deg", 66.0)))
        fx = (msg.width / 2.0) / math.tan(hfov / 2.0)
        fy = fx
        cx, cy = msg.width / 2.0, msg.height / 2.0
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _report(self):
        with self._lock:
            n, self._count = self._count, 0
        self.get_logger().info(f"camera: {n / 5.0:.1f} fps")


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
