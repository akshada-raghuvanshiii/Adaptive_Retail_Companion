#!/usr/bin/env python3
"""Show the Pi's annotated webcam view (boxes, lock, mode) on the laptop.

    source ~/arc_test_kit/env.sh
    python3 ~/arc_pi/laptop/view_debug.py                       # /arc/perception/debug/compressed (2 Hz)
    python3 ~/arc_pi/laptop/view_debug.py --topic /arc/camera/image/compressed   # raw stream mode
Uses OpenCV from the laptop's ROS install (python3-opencv) or the perception venv. q closes it.
"""
import argparse

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--topic", default="/arc/perception/debug/compressed")
    a = ap.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    rclpy.init()
    node = Node("arc_view_debug")
    st = {"img": None, "n": 0}

    def on_img(m):
        st["img"] = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        st["n"] += 1
    qos = qos_profile_sensor_data if "camera" in a.topic else 1
    node.create_subscription(CompressedImage, a.topic, on_img, qos)
    cv2.namedWindow("ARC Pi camera", cv2.WINDOW_NORMAL)
    print(f"waiting for {a.topic} ... (q to quit)")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if st["img"] is not None:
                cv2.imshow("ARC Pi camera", st["img"])
                st["img"] = None
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
