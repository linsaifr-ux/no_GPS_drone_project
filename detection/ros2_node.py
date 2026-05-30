#!/usr/bin/env python3
"""
YOLOv8 vehicle detector as a ROS2 node.

Subscribes:
  /drone/camera/image_raw  (sensor_msgs/Image, rgb8, 640×480)
  /drone/pose              (geometry_msgs/PoseStamped, frame_id="wgs84")

Publishes:
  /yolo/detections         (vision_msgs/Detection2DArray)

Run:
  source /opt/ros/jazzy/setup.bash
  python3 detection/ros2_node.py
"""

import os
import sys
import time

_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                              ObjectHypothesisWithPose)

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detection.detector import YOLODetector

MODEL_PT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "yolov8l_visdrone.pt")


class YOLONode(rclpy.node.Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self._det = YOLODetector(MODEL_PT, conf=0.30)

        self._drone_lat = 0.0
        self._drone_lon = 0.0

        self.sub_img  = self.create_subscription(
            Image, "/drone/camera/image_raw", self._cb_image, 1)
        self.sub_pose = self.create_subscription(
            PoseStamped, "/drone/pose", self._cb_pose, 10)

        self.pub = self.create_publisher(
            Detection2DArray, "/yolo/detections", 1)

        self.get_logger().info(
            f"YOLO node ready — model: {os.path.basename(MODEL_PT)}")

    def _cb_pose(self, msg: PoseStamped):
        self._drone_lat = msg.pose.position.x
        self._drone_lon = msg.pose.position.y

    def _cb_image(self, msg: Image):
        from PIL import Image as PILImage
        try:
            pil_img = PILImage.frombytes(
                "RGB", (msg.width, msg.height), bytes(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Image decode error: {e}")
            return

        t0 = time.time()
        detections = self._det.detect(pil_img)
        elapsed_ms = (time.time() - t0) * 1000

        arr = Detection2DArray()
        arr.header = msg.header

        for d in detections:
            det = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d["label"]
            hyp.hypothesis.score    = float(d["conf"])
            det.results.append(hyp)

            cx = (d["x1"] + d["x2"]) / 2.0
            cy = (d["y1"] + d["y2"]) / 2.0
            w  = float(d["x2"] - d["x1"])
            h  = float(d["y2"] - d["y1"])
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            arr.detections.append(det)

        self.pub.publish(arr)

        if detections:
            self.get_logger().info(
                f"{len(detections)} vehicle(s) detected  "
                f"({elapsed_ms:.0f} ms)  "
                f"lat={self._drone_lat:.5f} lon={self._drone_lon:.5f}")


def main():
    rclpy.init()
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
