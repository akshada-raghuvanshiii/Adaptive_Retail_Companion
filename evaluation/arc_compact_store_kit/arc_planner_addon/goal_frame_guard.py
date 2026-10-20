"""Goal frame guard for the ARC Dijkstra planner.

Problem it fixes: the planner used goal x/y as map coordinates whatever
msg.header.frame_id said, so a goal in "world" (or any other frame) was planned
to the wrong place without any warning.

Rule:
  * goal frame == map frame            -> use as is
  * another frame with a TF to map     -> transform through TF (real, looked-up transform)
  * no TF available / empty frame      -> REJECT with a reason (never guess, never relabel)

Plus a diagnostic of where the goal lands on the occupancy grid, so a failed plan
says "outside the map", "occupied" or "unknown (not yet mapped)" instead of nothing.

The pure functions have no ROS dependency (unit-tested). resolve_goal_for_node()
uses tf2_ros and creates a TF buffer on the node on first use.
"""

from dataclasses import dataclass
from math import atan2, cos, sin
from typing import Callable, Optional, Sequence, Tuple

Transform2D = Tuple[float, float, float]   # tx, ty, yaw  (pose of source frame in target frame)


@dataclass
class GoalResolution:
    ok: bool
    x: Optional[float]
    y: Optional[float]
    yaw: Optional[float]
    reason: str


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def apply_2d(t: Transform2D, x: float, y: float, yaw: float = 0.0) -> Tuple[float, float, float]:
    tx, ty, tyaw = t
    c, s = cos(tyaw), sin(tyaw)
    return tx + c * x - s * y, ty + s * x + c * y, yaw + tyaw


def resolve_goal(frame_id: str, x: float, y: float, yaw: float, map_frame: str,
                 lookup: Callable[[str, str], Transform2D]) -> GoalResolution:
    frame = (frame_id or "").lstrip("/")
    if not frame:
        return GoalResolution(False, None, None, None,
                              f"goal has an empty frame_id; publish goals in '{map_frame}'")
    if frame == map_frame:
        return GoalResolution(True, x, y, yaw, f"goal already in '{map_frame}'")
    try:
        t = lookup(map_frame, frame)
    except Exception as exc:  # tf2 raises several exception types
        return GoalResolution(False, None, None, None,
                              f"no TF from '{frame}' to '{map_frame}' ({type(exc).__name__}). "
                              f"Goal rejected: publish it in '{map_frame}' or provide a real transform.")
    gx, gy, gyaw = apply_2d(t, x, y, yaw)
    return GoalResolution(True, gx, gy, gyaw, f"transformed '{frame}' -> '{map_frame}' via TF")


def goal_cell_status(origin_x: float, origin_y: float, resolution: float, width: int, height: int,
                     data: Sequence[int], x: float, y: float, occupied_threshold: int = 50) -> Tuple[str, Optional[Tuple[int, int]]]:
    """Return ('ok'|'outside'|'occupied'|'unknown', (col, row)) for a map-frame point.
    Assumes an unrotated grid (RTAB-Map's /map has identity origin orientation)."""
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    if not (0 <= col < width and 0 <= row < height):
        return "outside", None
    v = data[row * width + col]
    if v < 0:
        return "unknown", (col, row)
    if v >= occupied_threshold:
        return "occupied", (col, row)
    return "ok", (col, row)


def make_tf_lookup(tf_buffer, timeout_s: float = 0.5):
    """Wrap a tf2_ros.Buffer as lookup(target, source) -> (tx, ty, yaw)."""
    from rclpy.duration import Duration
    from rclpy.time import Time

    def lookup(target: str, source: str) -> Transform2D:
        tf = tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=timeout_s))
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)
    return lookup


def resolve_goal_for_node(node, msg, map_frame: Optional[str] = None) -> GoalResolution:
    """Resolve a geometry_msgs/PoseStamped goal on a running rclpy node."""
    if map_frame is None:
        if not node.has_parameter("map_frame"):
            node.declare_parameter("map_frame", "map")
        map_frame = node.get_parameter("map_frame").value
    if not hasattr(node, "_arc_tf_buffer"):
        from tf2_ros import Buffer, TransformListener
        node._arc_tf_buffer = Buffer()
        node._arc_tf_listener = TransformListener(node._arc_tf_buffer, node)
    q = msg.pose.orientation
    yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
    res = resolve_goal(msg.header.frame_id, msg.pose.position.x, msg.pose.position.y, yaw,
                       map_frame, make_tf_lookup(node._arc_tf_buffer))
    log = node.get_logger()
    (log.info if res.ok else log.error)(
        f"[goal_frame_guard] frame='{msg.header.frame_id}' ({msg.pose.position.x:.2f}, "
        f"{msg.pose.position.y:.2f}) -> {'ACCEPTED' if res.ok else 'REJECTED'}: {res.reason}")
    return res
