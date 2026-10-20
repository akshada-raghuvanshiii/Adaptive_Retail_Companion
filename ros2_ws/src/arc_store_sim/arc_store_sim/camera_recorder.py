"""Record a ROS 2 image topic to an mp4, plus a CSV of each frame's sim time.

The CSV lets you line up the video (and the trajectory CSV that
track_and_log_v2.py produces from it) with the scripted ground truth in
scenarios/*_truth.csv, which is indexed by sim time from world start.

    ros2 run arc_store_sim camera_recorder --ros-args \
        -p image_topic:=/store/overhead_cam/image -p output_path:=aisle3.mp4 -p use_sim_time:=true

Needs OpenCV for the system Python (sudo apt install python3-opencv).
"""

import csv

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraRecorder(Node):
    def __init__(self):
        super().__init__("camera_recorder")
        self.topic = self.declare_parameter("image_topic", "/store/overhead_cam/image").value
        self.path = self.declare_parameter("output_path", "store_overhead.mp4").value
        self.fps = float(self.declare_parameter("fps", 15.0).value)
        self.writer = None
        self.frames = 0
        csv_path = self.path.rsplit(".", 1)[0] + "_frames.csv"
        self.csv_file = open(csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["frame", "sim_time_s", "stamp_s"])
        self.create_subscription(Image, self.topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"Recording {self.topic} -> {self.path} (+ {csv_path})")

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"Unsupported encoding {msg.encoding}", throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if self.writer is None:
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"),
                                          self.fps, (msg.width, msg.height))
            if not self.writer.isOpened():
                raise RuntimeError(f"Could not open video writer for {self.path}")
        self.writer.write(img)
        self.frames += 1
        now = self.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        # frame numbering matches track_and_log*.py (1-based)
        self.csv.writerow([self.frames, f"{now:.3f}", f"{stamp:.3f}"])

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.csv_file.close()
        self.get_logger().info(f"Saved {self.frames} frames to {self.path}")


def main():
    rclpy.init()
    node = CameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
