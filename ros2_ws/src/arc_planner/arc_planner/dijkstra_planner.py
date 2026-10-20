import heapq
import math
from copy import deepcopy

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import tf2_ros


class DijkstraPlanner(Node):

    def __init__(self):

        super().__init__('dijkstra_planner')

        self.get_logger().info(
            'Dijkstra planner started.'
        )

        # ============================================================
        # MAP
        # ============================================================

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.map_callback,
            map_qos
        )

        self.map_msg = None
        self.map_received = False

        # ============================================================
        # INFLATED MAP PUBLISHER
        # ============================================================

        self.inflated_map_pub = self.create_publisher(
            OccupancyGrid,
            '/inflated_map',
            map_qos
        )

        # ============================================================
        # MANUAL RVIZ GOAL
        # ============================================================

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # ============================================================
        # SHOPPING GOAL
        # ============================================================

        shopping_goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.shopping_goal_sub = self.create_subscription(
            PoseStamped,
            '/shopping_goal',
            self.shopping_goal_callback,
            shopping_goal_qos
        )

        # ============================================================
        # PERSON FOLLOW GOAL
        # ============================================================

        person_follow_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.person_follow_goal_sub = self.create_subscription(
            PoseStamped,
            '/person_follow_goal',
            self.person_follow_goal_callback,
            person_follow_qos
        )

        # ============================================================
        # CURRENT GOAL STATE
        # ============================================================

        self.goal_received = False

        self.goal_x = None
        self.goal_y = None

        # Used only for logging/debugging.
        self.goal_source = None

        # ============================================================
        # PATH
        # ============================================================

        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # PLANNER SETTINGS
        # ============================================================

        # RTAB-Map resolution is currently 0.05 m/cell.
        #
        # 4 cells = 0.20 m safety radius.

        self.inflation_radius = 4

        # Search radius for finding a safe start/goal
        # when the requested cell is occupied.

        self.start_search_radius = 20

        # Unknown cells are allowed, but have a higher
        # traversal cost than known free cells.

        self.unknown_cost = 5.0

        # ============================================================
        # PLANNING TIMER
        # ============================================================

        self.timer = self.create_timer(
            2.0,
            self.plan
        )

        self.last_start = None
        self.last_goal = None

    # ================================================================
    # MAP CALLBACK
    # ================================================================

    def map_callback(self, msg):

        self.map_msg = msg

        if not self.map_received:

            self.map_received = True

            self.get_logger().info(
                f'Map received: '
                f'{msg.info.width} x '
                f'{msg.info.height} cells'
            )

            self.get_logger().info(
                f'Resolution: '
                f'{msg.info.resolution:.3f} m/cell'
            )

            self.get_logger().info(
                f'Map origin: '
                f'x={msg.info.origin.position.x:.2f}, '
                f'y={msg.info.origin.position.y:.2f}'
            )

        # Publish visible inflated map.

        self.publish_inflated_map()

    # ================================================================
    # MANUAL RVIZ GOAL CALLBACK
    # ================================================================

    def goal_callback(self, msg):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.goal_received = True

        self.goal_source = 'RViz'

        # Force replanning.

        self.last_start = None
        self.last_goal = None

        self.get_logger().info(
            f'New RViz goal received: '
            f'({self.goal_x:.2f}, '
            f'{self.goal_y:.2f})'
        )

    # ================================================================
    # SHOPPING GOAL CALLBACK
    # ================================================================

    def shopping_goal_callback(self, msg):

        # Shopping Manager goals must be expressed in the map frame.
        # Transform explicitly when a different source frame is supplied;
        # never silently interpret coordinates from another frame as map.
        source_frame = msg.header.frame_id.strip()

        if not source_frame:
            self.get_logger().error(
                'Rejected shopping goal: header.frame_id is empty.'
            )
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        if source_frame != 'map':
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    source_frame,
                    rclpy.time.Time()
                )

                t = transform.transform.translation
                q = transform.transform.rotation

                # Rotate the position by the transform quaternion, then translate.
                x = goal_x
                y = goal_y
                z = msg.pose.position.z

                # Quaternion-vector rotation using q * v * conjugate(q).
                qx, qy, qz, qw = q.x, q.y, q.z, q.w
                tx = 2.0 * (qy * z - qz * y)
                ty = 2.0 * (qz * x - qx * z)
                tz = 2.0 * (qx * y - qy * x)

                goal_x = x + qw * tx + (qy * tz - qz * ty) + t.x
                goal_y = y + qw * ty + (qz * tx - qx * tz) + t.y

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                self.get_logger().error(
                    f'Rejected shopping goal: cannot transform '
                    f'{source_frame!r} to map: {exc}'
                )
                return

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_received = True
        self.goal_source = 'Shopping Manager'

        # Force replanning.
        self.last_start = None
        self.last_goal = None

        self.get_logger().info(
            f'New shopping goal received in map frame: '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    # ================================================================
    # PERSON FOLLOW GOAL CALLBACK
    # ================================================================
    def person_follow_goal_callback(self, msg):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.goal_received = True

        self.goal_source = 'Person Follow'

        # Force replanning immediately when the person's target moves.

        self.last_start = None
        self.last_goal = None

        self.get_logger().info(
            f'New person-follow goal received: '
            f'({self.goal_x:.2f}, '
            f'{self.goal_y:.2f})'
        )

    # ================================================================
    # WORLD -> GRID
    # ================================================================

    def world_to_grid(self, x, y):

        resolution = self.map_msg.info.resolution

        origin_x = (
            self.map_msg.info.origin.position.x
        )

        origin_y = (
            self.map_msg.info.origin.position.y
        )

        gx = int(
            (x - origin_x)
            / resolution
        )

        gy = int(
            (y - origin_y)
            / resolution
        )

        return gx, gy

    # ================================================================
    # GRID -> WORLD
    # ================================================================

    def grid_to_world(self, gx, gy):

        resolution = self.map_msg.info.resolution

        origin_x = (
            self.map_msg.info.origin.position.x
        )

        origin_y = (
            self.map_msg.info.origin.position.y
        )

        x = (
            origin_x
            + (gx + 0.5)
            * resolution
        )

        y = (
            origin_y
            + (gy + 0.5)
            * resolution
        )

        return x, y

    # ================================================================
    # ROBOT POSITION
    # ================================================================

    def get_robot_position(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    rclpy.time.Time()
                )
            )

            x = (
                transform.transform
                .translation.x
            )

            y = (
                transform.transform
                .translation.y
            )

            return x, y

        except Exception as e:

            self.get_logger().warn(
                f'Could not get robot pose '
                f'in map frame: {e}'
            )

            return None

    # ================================================================
    # CELL VALUE
    # ================================================================

    def cell_value(self, gx, gy):

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        if gx < 0 or gx >= width:
            return 100

        if gy < 0 or gy >= height:
            return 100

        index = gy * width + gx

        return self.map_msg.data[index]

    # ================================================================
    # CELL COST
    # ================================================================

    def cell_cost(self, gx, gy):

        value = self.cell_value(
            gx,
            gy
        )

        # Occupied.

        if value >= 50:
            return float('inf')

        # Unknown.

        if value < 0:
            return self.unknown_cost

        # Known free.

        return 1.0

    # ================================================================
    # INFLATED FREE CELL CHECK
    # ================================================================

    def is_free(self, gx, gy):

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        # Outside map.

        if gx < 0 or gx >= width:
            return False

        if gy < 0 or gy >= height:
            return False

        # Check circular inflation radius.

        for dx in range(
            -self.inflation_radius,
            self.inflation_radius + 1
        ):

            for dy in range(
                -self.inflation_radius,
                self.inflation_radius + 1
            ):

                # Circular inflation.

                if (
                    dx * dx
                    + dy * dy
                    >
                    self.inflation_radius
                    * self.inflation_radius
                ):
                    continue

                nx = gx + dx
                ny = gy + dy

                # Map boundary is unsafe.

                if nx < 0 or nx >= width:
                    return False

                if ny < 0 or ny >= height:
                    return False

                value = self.cell_value(
                    nx,
                    ny
                )

                # Occupied nearby = unsafe.

                if value >= 50:
                    return False

        return True

    # ================================================================
    # CREATE AND PUBLISH INFLATED MAP
    # ================================================================

    def publish_inflated_map(self):

        if self.map_msg is None:
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        # Start with a copy of the original map.

        inflated_data = list(
            self.map_msg.data
        )

        occupied_cells = []

        # Find all occupied cells.

        for gy in range(height):

            for gx in range(width):

                index = gy * width + gx

                if self.map_msg.data[index] >= 50:

                    occupied_cells.append(
                        (gx, gy)
                    )

        # Inflate each occupied cell.

        for ox, oy in occupied_cells:

            for dx in range(
                -self.inflation_radius,
                self.inflation_radius + 1
            ):

                for dy in range(
                    -self.inflation_radius,
                    self.inflation_radius + 1
                ):

                    # Circular inflation.

                    if (
                        dx * dx
                        + dy * dy
                        >
                        self.inflation_radius
                        * self.inflation_radius
                    ):
                        continue

                    nx = ox + dx
                    ny = oy + dy

                    if (
                        nx < 0
                        or nx >= width
                        or ny < 0
                        or ny >= height
                    ):
                        continue

                    index = ny * width + nx

                    # Preserve unknown cells.
                    #
                    # Only convert known free cells
                    # into inflated obstacles.

                    if inflated_data[index] >= 0:

                        inflated_data[index] = 100

        # Create OccupancyGrid message.

        inflated_msg = OccupancyGrid()

        inflated_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        inflated_msg.header.frame_id = (
            self.map_msg.header.frame_id
        )

        inflated_msg.info = deepcopy(
            self.map_msg.info
        )

        inflated_msg.data = inflated_data

        self.inflated_map_pub.publish(
            inflated_msg
        )

    # ================================================================
    # FIND NEAREST SAFE CELL
    # ================================================================

    def find_nearest_free_cell(
        self,
        cell,
        max_radius=None
    ):

        if max_radius is None:

            max_radius = (
                self.start_search_radius
            )

        sx, sy = cell

        # Requested cell is already safe.

        if self.is_free(sx, sy):

            return cell

        # Search outward.

        for radius in range(
            1,
            max_radius + 1
        ):

            best_cell = None
            best_distance = float('inf')

            for dx in range(
                -radius,
                radius + 1
            ):

                for dy in range(
                    -radius,
                    radius + 1
                ):

                    # Only inspect current ring.

                    if (
                        abs(dx) != radius
                        and abs(dy) != radius
                    ):
                        continue

                    nx = sx + dx
                    ny = sy + dy

                    if not self.is_free(
                        nx,
                        ny
                    ):
                        continue

                    distance = math.sqrt(
                        dx * dx
                        + dy * dy
                    )

                    if (
                        distance
                        < best_distance
                    ):

                        best_distance = distance

                        best_cell = (
                            nx,
                            ny
                        )

            if best_cell is not None:

                return best_cell

        return None

    # ================================================================
    # DIJKSTRA
    # ================================================================

    def dijkstra(self, start, goal):

        movements = [

            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),

            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2))
        ]

        queue = []

        heapq.heappush(
            queue,
            (
                0.0,
                start[0],
                start[1]
            )
        )

        distances = {
            start: 0.0
        }

        previous = {}

        while queue:

            current_cost, cx, cy = (
                heapq.heappop(queue)
            )

            current = (
                cx,
                cy
            )

            # Ignore outdated entries.

            if current_cost > distances.get(
                current,
                float('inf')
            ):
                continue

            # Goal reached.

            if current == goal:
                break

            for (
                dx,
                dy,
                movement_cost
            ) in movements:

                nx = cx + dx
                ny = cy + dy

                neighbour = (
                    nx,
                    ny
                )

                # Inflated obstacle check.

                if not self.is_free(
                    nx,
                    ny
                ):
                    continue

                # Cost of entering cell.

                terrain_cost = (
                    self.cell_cost(
                        nx,
                        ny
                    )
                )

                if (
                    terrain_cost
                    == float('inf')
                ):
                    continue

                new_cost = (
                    current_cost
                    + movement_cost
                    * terrain_cost
                )

                if new_cost < distances.get(
                    neighbour,
                    float('inf')
                ):

                    distances[
                        neighbour
                    ] = new_cost

                    previous[
                        neighbour
                    ] = current

                    heapq.heappush(
                        queue,
                        (
                            new_cost,
                            nx,
                            ny
                        )
                    )

        # No path.

        if (
            goal not in previous
            and goal != start
        ):

            self.get_logger().error(
                'Dijkstra could not find a path.'
            )

            return []

        # ============================================================
        # RECOVER PATH
        # ============================================================

        path = []

        current = goal

        while current != start:

            path.append(
                current
            )

            current = previous[
                current
            ]

        path.append(
            start
        )

        path.reverse()

        return path

    # ================================================================
    # PUBLISH PATH
    # ================================================================

    def publish_path(self, grid_path):

        path_msg = Path()

        path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        path_msg.header.frame_id = 'map'

        for gx, gy in grid_path:

            x, y = self.grid_to_world(
                gx,
                gy
            )

            pose = PoseStamped()

            pose.header = (
                path_msg.header
            )

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(
                pose
            )

        self.path_pub.publish(
            path_msg
        )

        self.get_logger().info(
            f'Published path with '
            f'{len(grid_path)} cells.'
        )

    # ================================================================
    # PLANNING
    # ================================================================

    def plan(self):

        # Wait for map.

        if not self.map_received:

            self.get_logger().info(
                'Waiting for RTAB-Map '
                'occupancy grid...'
            )

            return

        # Wait for goal.

        if not self.goal_received:

            return

        # Get robot position.

        robot_position = (
            self.get_robot_position()
        )

        if robot_position is None:

            return

        robot_x, robot_y = (
            robot_position
        )

        # Convert to grid.

        raw_start = self.world_to_grid(
            robot_x,
            robot_y
        )

        raw_goal = self.world_to_grid(
            self.goal_x,
            self.goal_y
        )

        self.get_logger().info(
            f'Robot world: '
            f'({robot_x:.2f}, '
            f'{robot_y:.2f})'
        )

        self.get_logger().info(
            f'Robot grid: '
            f'{raw_start}'
        )

        self.get_logger().info(
            f'Goal source: '
            f'{self.goal_source}'
        )

        self.get_logger().info(
            f'Goal world: '
            f'({self.goal_x:.2f}, '
            f'{self.goal_y:.2f})'
        )

        self.get_logger().info(
            f'Goal grid: '
            f'{raw_goal}'
        )

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        # ============================================================
        # CHECK GOAL BOUNDS
        # ============================================================

        if not (
            0 <= raw_goal[0] < width
            and
            0 <= raw_goal[1] < height
        ):

            self.get_logger().error(
                'Goal is outside '
                'the current RTAB-Map map.'
            )

            return

        # ============================================================
        # CHECK ROBOT BOUNDS
        # ============================================================

        if not (
            0 <= raw_start[0] < width
            and
            0 <= raw_start[1] < height
        ):

            self.get_logger().error(
                'Robot is outside '
                'the current map.'
            )

            return

        # ============================================================
        # FIND SAFE START
        # ============================================================

        safe_start = (
            self.find_nearest_free_cell(
                raw_start
            )
        )

        if safe_start is None:

            self.get_logger().error(
                'Could not find a '
                'safe free cell near robot.'
            )

            return

        if safe_start != raw_start:

            self.get_logger().warn(
                f'Robot start cell '
                f'{raw_start} is not safe. '
                f'Using nearby safe cell '
                f'{safe_start}.'
            )

        # ============================================================
        # FIND SAFE GOAL
        # ============================================================

        safe_goal = (
            self.find_nearest_free_cell(
                raw_goal
            )
        )

        if safe_goal is None:

            self.get_logger().error(
                'Could not find a '
                'safe free cell near goal.'
            )

            return

        if safe_goal != raw_goal:

            self.get_logger().warn(
                f'Goal cell {raw_goal} '
                f'is blocked or too close '
                f'to an obstacle. '
                f'Using nearby safe cell '
                f'{safe_goal}.'
            )

        start = safe_start
        goal = safe_goal

        # ============================================================
        # AVOID UNNECESSARY REPLANNING
        # ============================================================

        if (
            self.last_start == start
            and
            self.last_goal == goal
        ):

            return

        self.last_start = start
        self.last_goal = goal

        # ============================================================
        # RUN DIJKSTRA
        # ============================================================

        self.get_logger().info(
            f'Running Dijkstra '
            f'for {self.goal_source} goal...'
        )

        grid_path = self.dijkstra(
            start,
            goal
        )

        if not grid_path:

            return

        self.get_logger().info(
            f'Path found! '
            f'{len(grid_path)} cells.'
        )

        self.publish_path(
            grid_path
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = DijkstraPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
