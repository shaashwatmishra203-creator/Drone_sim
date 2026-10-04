#!/usr/bin/env python3
"""
vision_view - show what the cloud detector sees, live, and record it.

Displays /cloud/annotated (the drone camera frame with the cloud's bounding
boxes, class labels and confidence drawn on it) in a window next to Gazebo,
and writes the same frames to detections.mp4.
"""


import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

WIN = "Cloud computer vision - drone camera"


class VisionView(Node):

    def __init__(self):
        super().__init__("vision_view")
        self.declare_parameter("out_mp4", "/tmp/detections.mp4")
        self.declare_parameter("show", True)
        self.mp4 = self.get_parameter("out_mp4").value
        self.show = bool(self.get_parameter("show").value)
        self.writer = None
        self.latest = None
        self.n = 0
        self.create_subscription(CompressedImage, "/cloud/annotated", self.cb, 5)
        self.create_timer(1.0 / 30.0, self.draw)
        if self.show:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 960, 540)

    def cb(self, m):
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        self.latest = img
        if self.writer is None:
            h, w = img.shape[:2]
            self.writer = cv2.VideoWriter(self.mp4, cv2.VideoWriter_fourcc(*"mp4v"),
                                          10.0, (w, h))
        self.writer.write(img)
        self.n += 1

    def draw(self):
        if self.show and self.latest is not None:
            cv2.imshow(WIN, self.latest)
        if self.show:
            cv2.waitKey(1)

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.get_logger().info(f"wrote {self.n} frames to {self.mp4}")


def main():
    rclpy.init()
    n = VisionView()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
