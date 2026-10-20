"""Minimal stand-ins for rclpy and ROS message modules so the kit's logic can be
tested without ROS. Only what the kit touches at import time is provided."""
import sys
import types


def _mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


class _Msg:
    def __init__(self, *a, **k):
        pass


class Node:
    def __init__(self, *a, **k):
        pass

    def create_subscription(self, *a, **k):
        return None

    def create_publisher(self, *a, **k):
        return None

    def get_topic_names_and_types(self):
        return []


def install():
    if "rclpy" in sys.modules and getattr(sys.modules["rclpy"], "_is_stub", False):
        return sys.modules["rclpy"]
    rclpy = _mod("rclpy", _is_stub=True, init=lambda *a, **k: None, shutdown=lambda *a, **k: None,
                 spin_once=lambda node, timeout_sec=0.0: getattr(node, "_tick", lambda: None)())
    rclpy.time = _mod("rclpy.time", Time=lambda **k: k)
    rclpy.clock = _mod("rclpy.clock", ClockType=types.SimpleNamespace(ROS_TIME=1))
    rclpy.duration = _mod("rclpy.duration", Duration=lambda **k: k)
    rclpy.node = _mod("rclpy.node", Node=Node)
    rclpy.parameter = _mod("rclpy.parameter", Parameter=type("Parameter", (), {
        "Type": types.SimpleNamespace(BOOL=1), "__init__": lambda self, *a, **k: None}))
    rclpy.qos = _mod("rclpy.qos", DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2),
                     QoSProfile=_Msg, ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1))
    _mod("action_msgs")
    _mod("action_msgs.msg", GoalStatusArray=_Msg)
    _mod("action_msgs.srv", CancelGoal=type("CancelGoal", (), {"Request": _Msg}))
    _mod("geometry_msgs")
    _mod("geometry_msgs.msg", PoseStamped=_Msg, Twist=type("Twist", (_Msg,), {}),
         TwistStamped=type("TwistStamped", (_Msg,), {}))
    _mod("nav_msgs")
    _mod("nav_msgs.msg", OccupancyGrid=_Msg, Path=_Msg)
    _mod("std_msgs")
    _mod("std_msgs.msg", Bool=_Msg, String=_Msg)
    _mod("tf2_msgs")
    _mod("tf2_msgs.msg", TFMessage=_Msg)
    _mod("tf2_ros", Buffer=lambda **k: None, TransformListener=lambda *a, **k: None)
    return rclpy
