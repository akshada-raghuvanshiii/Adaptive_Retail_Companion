import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class ReactiveAvoider(Node):

    def __init__(self):
        super().__init__('reactive_avoider')

        # =========================================================
        # REACTIVE AVOIDANCE PARAMETERS
        # =========================================================

        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('resume_distance', 0.55)
        self.declare_parameter('slow_distance', 0.90)

        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('turn_speed', 0.8)

        self.declare_parameter('front_half_width_deg', 20.0)

        self.declare_parameter('side_start_deg', 45.0)
        self.declare_parameter('side_end_deg', 135.0)

        self.stop_distance = self.get_parameter(
            'stop_distance'
        ).value

        self.resume_distance = self.get_parameter(
            'resume_distance'
        ).value

        self.slow_distance = self.get_parameter(
            'slow_distance'
        ).value

        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).value

        self.min_linear_speed = self.get_parameter(
            'min_linear_speed'
        ).value

        self.turn_speed = self.get_parameter(
            'turn_speed'
        ).value

        self.front_half_width_deg = self.get_parameter(
            'front_half_width_deg'
        ).value

        self.side_start_deg = self.get_parameter(
            'side_start_deg'
        ).value

        self.side_end_deg = self.get_parameter(
            'side_end_deg'
        ).value

        # =========================================================
        # LAP DETECTION PARAMETERS
        # =========================================================

        self.declare_parameter('max_laps', 2)

        # Robot must move at least this far away from the starting
        # position before it can be considered to have completed
        # a circuit on returning.
        self.declare_parameter('leave_start_distance', 1.0)

        # Robot must return within this distance of its start
        # position to count as completing a lap.
        self.declare_parameter('return_distance', 0.40)

        self.max_laps = self.get_parameter(
            'max_laps'
        ).value

        self.leave_start_distance = self.get_parameter(
            'leave_start_distance'
        ).value

        self.return_distance = self.get_parameter(
            'return_distance'
        ).value

        # =========================================================
        # REACTIVE STATE
        # =========================================================

        self.avoiding = False

        # =========================================================
        # LAP STATE
        # =========================================================

        self.start_x = None
        self.start_y = None

        self.robot_x = None
        self.robot_y = None

        self.left_start_region = False

        self.lap_count = 0

        self.finished = False

        # =========================================================
        # ROS INTERFACES
        # =========================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.get_logger().info(
            'Reactive avoider started.'
        )

        self.get_logger().info(
            f'Will stop automatically after '
            f'{self.max_laps} laps.'
        )

    # =============================================================
    # ODOMETRY
    # =============================================================

    def odom_callback(self, msg):

        self.robot_x = (
            msg.pose.pose.position.x
        )

        self.robot_y = (
            msg.pose.pose.position.y
        )

        # Record the first valid robot position as the start.
        if self.start_x is None:

            self.start_x = self.robot_x
            self.start_y = self.robot_y

            self.get_logger().info(
                f'Start position recorded: '
                f'({self.start_x:.2f}, '
                f'{self.start_y:.2f})'
            )

            return

        # Once exploration is finished, do not count anything else.
        if self.finished:

            return

        # Distance from starting position.
        dx = (
            self.robot_x
            - self.start_x
        )

        dy = (
            self.robot_y
            - self.start_y
        )

        distance_from_start = math.sqrt(
            dx * dx
            + dy * dy
        )

        # ---------------------------------------------------------
        # Robot has genuinely left the starting region.
        # ---------------------------------------------------------

        if (
            not self.left_start_region
            and distance_from_start
            >= self.leave_start_distance
        ):

            self.left_start_region = True

            self.get_logger().info(
                'Robot has left starting region. '
                'Lap return detection armed.'
            )

        # ---------------------------------------------------------
        # Robot has returned after leaving.
        # ---------------------------------------------------------

        if (
            self.left_start_region
            and distance_from_start
            <= self.return_distance
        ):

            self.lap_count += 1

            self.get_logger().info(
                f'Lap {self.lap_count} completed '
                f'of {self.max_laps}.'
            )

            # Stop permanently after required laps.
            if self.lap_count >= self.max_laps:

                self.finished = True

                self.get_logger().info(
                    'Required laps completed. '
                    'Stopping robot.'
                )

                self.stop_robot()

                return

            # Require robot to leave the start region again before
            # another lap can be counted.
            self.left_start_region = False

    # =============================================================
    # STOP ROBOT
    # =============================================================

    def stop_robot(self):

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # =============================================================
    # CONVERT ANGLE TO LIDAR INDEX
    # =============================================================

    def deg_to_index(self, msg, deg):

        rad = math.radians(deg)

        idx = int(
            round(
                (rad - msg.angle_min)
                / msg.angle_increment
            )
        )

        return idx % len(msg.ranges)

    # =============================================================
    # GET CLOSEST OBSTACLE IN SECTOR
    # =============================================================

    def get_sector_distance(
        self,
        msg,
        start_deg,
        end_deg
    ):

        ranges = msg.ranges
        n = len(ranges)

        start = self.deg_to_index(
            msg,
            start_deg
        )

        end = self.deg_to_index(
            msg,
            end_deg
        )

        steps = (
            end - start
        ) % n

        values = []

        for offset in range(
            steps + 1
        ):

            idx = (
                start + offset
            ) % n

            distance = ranges[idx]

            if (
                math.isfinite(distance)
                and distance > 0.0
            ):

                values.append(
                    distance
                )

        if not values:

            if math.isfinite(
                msg.range_max
            ):

                return msg.range_max

            return 3.5

        return min(values)

    # =============================================================
    # LIDAR CALLBACK
    # =============================================================

    def scan_callback(self, msg):

        # ---------------------------------------------------------
        # STOP IF EXPLORATION FINISHED
        # ---------------------------------------------------------

        if self.finished:

            self.stop_robot()

            return

        front_width = (
            self.front_half_width_deg
        )

        front_left = (
            self.get_sector_distance(
                msg,
                0.0,
                front_width
            )
        )

        front_right = (
            self.get_sector_distance(
                msg,
                -front_width,
                0.0
            )
        )

        left = (
            self.get_sector_distance(
                msg,
                self.side_start_deg,
                self.side_end_deg
            )
        )

        right = (
            self.get_sector_distance(
                msg,
                -self.side_end_deg,
                -self.side_start_deg
            )
        )

        front = min(
            front_left,
            front_right
        )

        # =========================================================
        # HYSTERESIS
        # =========================================================

        if self.avoiding:

            if (
                front
                > self.resume_distance
            ):

                self.avoiding = False

        else:

            if (
                front
                < self.stop_distance
            ):

                self.avoiding = True

        # =========================================================
        # CREATE COMMAND
        # =========================================================

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # =========================================================
        # AVOIDANCE MODE
        # =========================================================

        if self.avoiding:

            cmd.twist.linear.x = 0.0

            if left > right:

                cmd.twist.angular.z = (
                    self.turn_speed
                )

                self.get_logger().info(
                    'Obstacle ahead -> '
                    'turning LEFT'
                )

            else:

                cmd.twist.angular.z = (
                    -self.turn_speed
                )

                self.get_logger().info(
                    'Obstacle ahead -> '
                    'turning RIGHT'
                )

        # =========================================================
        # FORWARD MODE
        # =========================================================

        else:

            if (
                front
                >= self.slow_distance
            ):

                speed = (
                    self.max_linear_speed
                )

            else:

                denominator = (
                    self.slow_distance
                    - self.stop_distance
                )

                ratio = (
                    (
                        front
                        - self.stop_distance
                    )
                    / denominator
                )

                ratio = max(
                    0.0,
                    min(
                        1.0,
                        ratio
                    )
                )

                speed = (
                    self.min_linear_speed
                    + ratio
                    * (
                        self.max_linear_speed
                        - self.min_linear_speed
                    )
                )

            cmd.twist.linear.x = speed
            cmd.twist.angular.z = 0.0

        # =========================================================
        # PUBLISH
        # =========================================================

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ReactiveAvoider()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
