#!/usr/bin/env python3
"""Webcam on the Pi -> sensor_msgs/CompressedImage (JPEG) for the laptop ("stream" mode).

Use this when YOLO is too slow on the Pi: the Pi only captures and compresses, and the laptop runs
    perception_bridge.py --camera ros:/arc/camera/image/compressed
Publishes /arc/camera/image/compressed (best-effort sensor QoS, frame_id "arc_camera").
A 640x480 JPEG at quality 70 is roughly 30-60 kB, so 10 fps is a few Mbit/s on the Ethernet cable.

    source ~/arc_pi/pi/pi_env.sh
    ~/arc_pi_venv/bin/python ~/arc_pi/pi/camera_pub.py --camera 0 --width 640 --height 480 --fps 10
"""
import argparse
import signal
import sys
import time

import cv2


def _sigterm(*_):
    raise KeyboardInterrupt


def ros_init(rclpy):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


def open_camera(cam, width, height, mjpg=True):
    cap = cv2.VideoCapture(int(cam) if str(cam).isdigit() else cam)
    if mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", default="0")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=10.0, help="publish rate limit")
    ap.add_argument("--quality", type=int, default=70, help="JPEG quality")
    ap.add_argument("--topic", default="/arc/camera/image/compressed")
    ap.add_argument("--no-mjpg", action="store_true")
    a = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _sigterm)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    ros_init(rclpy)
    node = Node("arc_camera_pub")
    pub = node.create_publisher(CompressedImage, a.topic, qos_profile_sensor_data)
    cap = open_camera(a.camera, a.width, a.height, not a.no_mjpg)
    ok, frame = cap.read()
    if not ok:
        node.get_logger().error(f"cannot read camera {a.camera}")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    node.get_logger().info(f"camera {a.camera} {frame.shape[1]}x{frame.shape[0]} -> {a.topic} at <= {a.fps} fps")
    period, sent, t_rep, last = 1.0 / a.fps, 0, time.monotonic(), 0.0
    try:
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                node.get_logger().error("camera read failed; stopping (the laptop bridge then holds the BT in WAIT)")
                break
            now = time.monotonic()
            if now - last < period:
                continue
            last = now
            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), a.quality])
            if not ok:
                continue
            m = CompressedImage()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = "arc_camera"
            m.format = "jpeg"
            m.data = jpg.tobytes()
            pub.publish(m)
            sent += 1
            if now - t_rep >= 10.0:
                node.get_logger().info(f"{sent / (now - t_rep):.1f} fps sent, {len(m.data) / 1024:.0f} kB/frame")
                sent, t_rep = 0, now
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
