"""ROS 2 / Gazebo plumbing shared by the probe and the mission runner.

Read-only towards the ARC stack: it subscribes to existing topics, publishes
goals on /goal_pose (the topic RViz uses), and talks to Gazebo only to read
the true pose and to add or remove test boxes. It never edits ARC packages
or parameters.

Matched to the ARC stack as built (checked against the source, 14 Oct 2026):
- /cmd_vel is geometry_msgs/TwistStamped (arc_motion_gate); plain Twist is
  still accepted if that is what the topic carries.
- planned_path_to_teb publishes /navigation_goal_reached (Bool, latched) when
  FollowPath succeeds, and ignores new paths while a FollowPath is running.
- The FollowPath action is /follow_path; its status topic shows aborts.
- The BT publishes its mode on /arc/behavior_mode (String).
"""
import math
import re
import shutil
import subprocess
import threading
import time

import rclpy
import rclpy.time
from rclpy.clock import ClockType
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
import tf2_ros

GOAL_TOPIC = "/goal_pose"
REACHED_TOPIC = "/navigation_goal_reached"
FOLLOW_PATH_STATUS = "/follow_path/_action/status"
MODE_TOPIC = "/arc/behavior_mode"
ACTIVE_STATES = (1, 2, 3)        # ACCEPTED, EXECUTING, CANCELING
ABORTED, SUCCEEDED, CANCELED = 6, 4, 5


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def path_length(path_msg):
    pts = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])), pts


_NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"


def parse_gz_model_pose(out):
    """(x, y, yaw) from `gz model -m NAME -p` output, as check_frames.py reads it."""
    tr = re.findall(rf"\[\s*({_NUM})\s+({_NUM})\s+({_NUM})\s*\]", out.split("Pose", 1)[-1])
    if len(tr) < 2:
        return None
    return float(tr[0][0]), float(tr[0][1]), float(tr[1][2])


def gz_model_pose(model):
    try:
        out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True,
                             timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    return parse_gz_model_pose(out)


class TruthBridge:
    """Runs ros_gz_bridge for the world's pose topics into /arc_truth/*."""

    def __init__(self, world):
        self.world = world
        self.proc = None

    def start(self):
        dyn = f"/world/{self.world}/dynamic_pose/info"
        allp = f"/world/{self.world}/pose/info"
        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
               f"{dyn}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
               f"{allp}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
               "--ros-args", "-r", f"{dyn}:=/arc_truth/dynamic",
               "-r", f"{allp}:=/arc_truth/all"]
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            print("WARNING: 'ros2' not on PATH; true pose unavailable (source ROS first).")
            self.proc = None

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class GzPoller:
    """Fallback truth source: polls `gz model -p` (what check_frames.py uses).

    The call takes ~0.1-0.3 s, so each reading carries the wall time of the
    request and of the reply; the runner pairs it with the map pose between them.
    """

    def __init__(self, model, period=0.3):
        self.model, self.period = model, period
        self.latest = None        # (x, y, yaw, t_req, t_rep)
        self._stop = False
        self._th = None

    def start(self):
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        while not self._stop:
            t_req = time.time()
            p = gz_model_pose(self.model)
            if p:
                self.latest = (p[0], p[1], p[2], t_req, time.time())
            time.sleep(self.period)

    def stop(self):
        self._stop = True


class Gz:
    """Thin wrapper over the `gz service` CLI for test obstacles."""

    def __init__(self, world):
        self.world = world
        self.ok = shutil.which("gz") is not None

    def _call(self, service, reqtype, req):
        if not self.ok:
            return False
        cmd = ["gz", "service", "-s", f"/world/{self.world}/{service}",
               "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
               "--timeout", "3000", "--req", req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            return "true" in out.stdout.lower()
        except subprocess.TimeoutExpired:
            return False

    def spawn_box(self, name, x, y, size=0.2, height=0.35):
        sdf = (f"<?xml version='1.0'?><sdf version='1.6'><model name='{name}'><static>true</static>"
               f"<pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 0</pose><link name='link'>"
               f"<collision name='c'><geometry><box><size>{size} {size} {height}</size></box></geometry></collision>"
               f"<visual name='v'><geometry><box><size>{size} {size} {height}</size></box></geometry>"
               f"<material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material></visual>"
               f"</link></model></sdf>")
        return self._call("create", "gz.msgs.EntityFactory", f'sdf: "{sdf}" name: "{name}"')

    def remove(self, name):
        return self._call("remove", "gz.msgs.Entity", f'name: "{name}" type: MODEL')

    def model_names(self):
        """Names from `gz model --list`; None if the list could not be read."""
        if not self.ok:
            return None
        try:
            out = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            return None
        names = [m.group(1) for m in re.finditer(r"^\s*-\s*(\S+)\s*$", out, re.M)]
        return names if names else None

    def test_boxes(self, prefix=("arc_test_box", "arc_probe_box")):
        names = self.model_names()
        return None if names is None else [n for n in names if n.startswith(prefix)]

    def remove_verified(self, name, tries=4):
        """Remove a model and confirm it is gone from `gz model --list` (the service reply alone
        is not enough: in A4 on 15 Oct, 12 boxes stayed although removal was requested)."""
        for k in range(tries):
            self.remove(name)
            time.sleep(0.5 + 0.5 * k)
            left = self.test_boxes()
            if left is not None and name not in left:
                return True
        return False

    def sweep_test_boxes(self):
        """Remove every leftover test box; returns the names that could not be removed."""
        left = self.test_boxes()
        if not left:
            return []
        return [n for n in left if not self.remove_verified(n)]


class Watcher(Node):
    """Collects what the tests need: map pose, true pose, commands, paths, map, ARC events."""

    def __init__(self, model="arc", base_frame="base_footprint", map_frame="map",
                 path_topic="/planned_path", cmd_topic="/cmd_vel", map_topic="/rtabmap/map",
                 truth="auto"):
        super().__init__("arc_test_watcher",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.model, self.base, self.mapf = model, base_frame, map_frame
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=20))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.truth_mode = truth          # auto | bridge | gzcli
        self.truth = None                # (x, y, yaw, wall_time, stamp_s or None, src)
        self.truth_names = set()
        self.poller = None
        self.cmd = (0.0, 0.0)
        self.cmd_time = 0.0
        self.cmd_log = []                # (sim_t, v, w) for every /cmd_vel message
        self.cmd_type = None
        self.last_path = None            # (wall_time, length, pts)
        self.map_msg = None
        self.reached_times = []          # wall times of /navigation_goal_reached True
        self.ctrl_status = {}            # goal uuid -> status
        self.ctrl_aborts = []            # wall times of new ABORTED goals
        self.ctrl_succeeded = []
        self.ctrl_status_seen = False
        self.bt_mode, self.bt_mode_time = None, 0.0
        if truth in ("auto", "bridge"):
            self.create_subscription(TFMessage, "/arc_truth/dynamic", self._on_truth, 10)
            self.create_subscription(TFMessage, "/arc_truth/all", self._on_truth_all, 10)
        self._setup_cmd(cmd_topic)
        self.create_subscription(Path, path_topic, self._on_path, 5)
        for dur in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            qos = QoSProfile(depth=1, durability=dur, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, map_topic, self._on_map, qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._t_created = time.time()
        self.create_subscription(Bool, REACHED_TOPIC, self._on_reached, latched)
        self.create_subscription(GoalStatusArray, FOLLOW_PATH_STATUS, self._on_status, latched)
        self.create_subscription(String, MODE_TOPIC, self._on_mode, 10)
        self.goal_pub = None
        self._got_dynamic = False
        if truth == "gzcli":
            self.start_poller()

    # ---- setup
    def _setup_cmd(self, topic):
        """Subscribe with the type the topic actually carries (TwistStamped for ARC)."""
        kind = None
        end = time.time() + 3.0
        while time.time() < end and kind is None:
            for name, types in self.get_topic_names_and_types():
                if name == topic:
                    if "geometry_msgs/msg/TwistStamped" in types:
                        kind = TwistStamped
                    elif "geometry_msgs/msg/Twist" in types:
                        kind = Twist
            if kind is None:
                time.sleep(0.2)
        self.cmd_type = kind or TwistStamped   # ARC's motion gate publishes TwistStamped
        self.create_subscription(self.cmd_type, topic, self._on_cmd, 50)

    def start_poller(self):
        if self.poller is None:
            self.poller = GzPoller(self.model)
            self.poller.start()

    # ---- callbacks
    def _pick(self, msg):
        best = None
        for t in msg.transforms:
            name = t.child_frame_id
            self.truth_names.add(name)
            if name == self.model:
                return t
            if best is None and (name.endswith("::" + self.model) or name.endswith("/" + self.model)):
                best = t
        return best

    def _set_truth(self, t):
        tr = t.transform
        st = t.header.stamp
        stamp = st.sec + st.nanosec * 1e-9 if (st.sec or st.nanosec) else None
        self.truth = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation), time.time(), stamp, "bridge")

    def _on_truth(self, msg):
        t = self._pick(msg)
        if t is not None:
            self._got_dynamic = True
            self._set_truth(t)

    def _on_truth_all(self, msg):
        if self._got_dynamic:
            return
        t = self._pick(msg)
        if t is not None:
            self._set_truth(t)

    def _on_cmd(self, msg):
        tw = msg.twist if hasattr(msg, "twist") else msg
        self.cmd = (tw.linear.x, tw.angular.z)
        self.cmd_time = time.time()
        self.cmd_log.append((self.sim_now(), tw.linear.x, tw.angular.z))

    def _on_path(self, msg):
        if msg.poses:
            length, pts = path_length(msg)
            self.last_path = (time.time(), length, pts)

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_reached(self, msg):
        # The topic is latched: the True from an earlier goal arrives on connect. Ignore it.
        if msg.data and time.time() - self._t_created > 1.5:
            self.reached_times.append(time.time())

    def _on_status(self, msg):
        self.ctrl_status_seen = True
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()
            prev = self.ctrl_status.get(gid)
            # goals already finished when we connected (latched history) are not events
            old = prev is None and st.status in (ABORTED, SUCCEEDED, CANCELED) and \
                time.time() - self._t_created < 3.0
            if prev != st.status and not old:
                if st.status == ABORTED:
                    self.ctrl_aborts.append(time.time())
                if st.status == SUCCEEDED:
                    self.ctrl_succeeded.append(time.time())
            self.ctrl_status[gid] = st.status

    def _on_mode(self, msg):
        self.bt_mode, self.bt_mode_time = msg.data, time.time()

    # ---- helpers
    def controller_active(self):
        return any(s in ACTIVE_STATES for s in self.ctrl_status.values())

    def mode(self):
        return self.bt_mode if time.time() - self.bt_mode_time < 2.0 else None

    def cmd_now(self):
        """Last command, or zero if nothing arrived for 0.5 s (gate publishes only on input)."""
        return self.cmd if time.time() - self.cmd_time < 0.5 else (0.0, 0.0)

    def map_pose(self, stamp_s=None):
        """map -> base_footprint, at the given sim time if the buffer has it."""
        tf = None
        if stamp_s is not None:
            try:
                tf = self.tf_buf.lookup_transform(self.mapf, self.base, rclpy.time.Time(seconds=stamp_s, clock_type=ClockType.ROS_TIME))
            except Exception:
                tf = None
        if tf is None:
            try:
                tf = self.tf_buf.lookup_transform(self.mapf, self.base, rclpy.time.Time())
            except Exception:
                return None
        tr = tf.transform
        return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def tf_pose(self, parent, child):
        try:
            tr = self.tf_buf.lookup_transform(parent, child, rclpy.time.Time()).transform
        except Exception:
            return None
        return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def tf_layers(self):
        """Each layer of map -> icp_odom -> odom -> base_footprint (latest), to see which one failed.

        corr = RTAB-Map's correction (map -> odometry frame), lidar = icp_odom -> base,
        wheel = odom -> base. Without ICP odometry the odometry frame is 'odom'.
        """
        corr = self.tf_pose(self.mapf, "icp_odom")
        if corr is not None:
            return {"corr": corr, "lidar": self.tf_pose("icp_odom", self.base), "wheel": self.tf_pose("odom", self.base)}
        return {"corr": self.tf_pose(self.mapf, "odom"), "lidar": None, "wheel": self.tf_pose("odom", self.base)}

    def fresh_truth(self, max_age=2.0):
        """Newest true pose from the bridge, else from the gz poller."""
        if self.truth and time.time() - self.truth[3] < max_age:
            return self.truth
        if self.poller and self.poller.latest and time.time() - self.poller.latest[4] < max_age:
            x, y, yaw, t_req, t_rep = self.poller.latest
            # gz reads the pose somewhere between request and reply. Give it the sim time of the
            # midpoint so the map pose is looked up at the same instant (sim runs ~1:1 over <1 s).
            stamp = self.sim_now() - (time.time() - 0.5 * (t_req + t_rep))
            return (x, y, yaw, t_rep, stamp if stamp > 0 else None, "gzcli")
        return None

    def pose(self):
        """Best available pose for progress checks: truth if present, else map."""
        t = self.fresh_truth()
        if t:
            return t[:3]
        return self.map_pose()

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def ensure_goal_pub(self, topic):
        if self.goal_pub is None:
            self.goal_pub = self.create_publisher(PoseStamped, topic, 10)
        return self.goal_pub

    def send_goal(self, topic, x, y, yaw):
        pub = self.ensure_goal_pub(topic)
        msg = PoseStamped()
        msg.header.frame_id = self.mapf
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        pub.publish(msg)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def wait_truth(self, timeout=8.0):
        """Bridge first; if it never names the model, fall back to polling gz (check_frames' source)."""
        if self.truth_mode == "gzcli":
            return self.wait_for(lambda: self.fresh_truth() is not None, timeout)
        if self.wait_for(lambda: self.truth is not None, timeout / 2):
            return True
        if self.truth_mode == "auto" and shutil.which("gz"):
            self.start_poller()
            return self.wait_for(lambda: self.fresh_truth() is not None, timeout / 2 + 3)
        return False


def topic_subscribers(node, topic):
    """Node names subscribed to a topic."""
    try:
        return [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in node.get_subscriptions_info_by_topic(topic)]
    except Exception:
        return []


def find_goal_topic(node, default=GOAL_TOPIC):
    """The ARC planner listens to /goal_pose (no frame check, no QoS surprise). Confirm it."""
    subs = topic_subscribers(node, default)
    owner = next((s for s in subs if "dijkstra" in s), None)
    return default, owner


def running_nodes(node):
    return sorted(f"{ns.rstrip('/')}/{n}" for n, ns in node.get_node_names_and_namespaces())


def ros2_param(node_name, param, timeout=8):
    try:
        out = subprocess.run(["ros2", "param", "get", node_name, param], capture_output=True,
                             text=True, timeout=timeout).stdout.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    m = re.search(r"value is:\s*(.+)$", out)
    return m.group(1).strip() if m else None


def stack_config(node):
    """What is actually running: odometry source for RTAB-Map and the controller plugin."""
    nodes = running_nodes(node)
    cfg = {"icp_odometry_running": any("icp_odometry" in n for n in nodes)}
    rtab_odom = None
    for topic in ("/odom_icp", "/odom"):
        if any("rtabmap" in s and "icp_odometry" not in s for s in topic_subscribers(node, topic)):
            rtab_odom = topic
            break
    cfg["rtabmap_odom_topic"] = rtab_odom
    cfg["odometry"] = {"/odom_icp": "icp", "/odom": "wheel"}.get(rtab_odom, "unknown")
    plugin = ros2_param("/controller_server", "TEB.plugin")
    cfg["controller_plugin"] = plugin
    p = (plugin or "").lower()
    cfg["controller"] = "teb" if "teb" in p else "rpp" if "pure_pursuit" in p else "dwb" if "dwb" in p else "unknown"
    r = ros2_param("/controller_server", "progress_checker.required_movement_radius")
    try:
        cfg["progress_radius"] = float(r) if r is not None else None
    except ValueError:
        cfg["progress_radius"] = None
    cfg["progress"] = ("unknown" if cfg["progress_radius"] is None
                       else "default" if abs(cfg["progress_radius"] - 0.5) < 1e-6 else "relaxed")
    cfg["shopping_manager_running"] = any("shopping_manager" in n for n in nodes)
    return cfg


def label_mismatch(label, cfg):
    """Return a reason if the label's tokens contradict the running stack, else None."""
    toks = label.lower().split("_")
    for tok, key, want in (("icp", "odometry", "icp"), ("wheel", "odometry", "wheel"),
                           ("teb", "controller", "teb"), ("rpp", "controller", "rpp")):
        if tok in toks and cfg.get(key) not in (want, "unknown"):
            return f"label '{label}' says {tok} but the running stack has {key}={cfg.get(key)}"
    # phase 2: a 'pc' label must run with the relaxed progress check, any other label without it
    prog = cfg.get("progress", "unknown")
    if "pc" in toks and prog == "default":
        return f"label '{label}' says pc (relaxed progress check) but the controller uses Nav2's default 0.5 m / 10 s"
    if "pc" not in toks and prog == "relaxed" and any(t in toks for t in ("teb", "rpp")):
        return (f"label '{label}' is a baseline but the controller runs a relaxed progress check "
                f"({cfg.get('progress_radius')} m); restart B1 with ARC's own teb_controller.yaml")
    return None
