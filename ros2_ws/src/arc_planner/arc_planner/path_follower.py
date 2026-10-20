import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped

import tf2_ros


class PathFollower(Node):

    def __init__(self):

        super().__init__('path_follower')

        self.get_logger().info(
            'Path follower started.'
        )

        # ============================================================
        # PATH SUBSCRIBER
        # ============================================================

        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        # ============================================================
        # VELOCITY PUBLISHER
        # ============================================================

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
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
        # PATH STATE
        # ============================================================

        self.path = None

        # Current progress along the path.
        #
        # This prevents the controller from jumping
        # backwards between nearby path points.

        self.current_index = 0

        # ============================================================
        # SPEEDS
        # ============================================================

        self.max_linear_speed = 0.04

        self.max_angular_speed = 0.30

        # ============================================================
        # CONTROLLER SETTINGS
        # ============================================================

        self.goal_tolerance = 0.15

        self.lookahead_distance = 0.25

        # Distance at which a waypoint is considered passed.

        self.waypoint_tolerance = 0.12

        # Controller frequency.

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

    # ================================================================
    # PATH CALLBACK
    # ================================================================

    def path_callback(self, msg):

        if len(msg.poses) == 0:

            self.path = None

            self.current_index = 0

            self.get_logger().warn(
                'Received empty path.'
            )

            return

        # Only reset progress when receiving
        # a genuinely new path.

        self.path = msg

        self.current_index = 0

        self.get_logger().info(
            f'Received path with '
            f'{len(msg.poses)} poses.'
        )

    # ================================================================
    # GET ROBOT POSE
    # ================================================================

    def get_robot_pose(self):

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

            q = (
                transform.transform
                .rotation
            )

            # Quaternion -> yaw

            siny_cosp = (
                2.0
                * (
                    q.w * q.z
                    + q.x * q.y
                )
            )

            cosy_cosp = (
                1.0
                - 2.0
                * (
                    q.y * q.y
                    + q.z * q.z
                )
            )

            yaw = math.atan2(
                siny_cosp,
                cosy_cosp
            )

            return x, y, yaw

        except Exception as e:

            self.get_logger().warn(
                f'Could not get robot pose: '
                f'{e}'
            )

            return None

    # ================================================================
    # NORMALIZE ANGLE
    # ================================================================

    def normalize_angle(self, angle):

        while angle > math.pi:

            angle -= (
                2.0 * math.pi
            )

        while angle < -math.pi:

            angle += (
                2.0 * math.pi
            )

        return angle

    # ================================================================
    # STOP ROBOT
    # ================================================================

    def stop_robot(self):

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        cmd.header.frame_id = (
            'base_footprint'
        )

        cmd.twist.linear.x = 0.0

        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(
            cmd
        )

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # NO PATH
        # ------------------------------------------------------------

        if self.path is None:

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # ROBOT POSE
        # ------------------------------------------------------------

        robot_pose = (
            self.get_robot_pose()
        )

        if robot_pose is None:

            self.stop_robot()

            return

        robot_x, robot_y, robot_yaw = (
            robot_pose
        )

        poses = (
            self.path.poses
        )

        if len(poses) == 0:

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # GOAL CHECK
        # ------------------------------------------------------------

        goal_pose = (
            poses[-1]
        )

        goal_x = (
            goal_pose
            .pose
            .position
            .x
        )

        goal_y = (
            goal_pose
            .pose
            .position
            .y
        )

        goal_dx = (
            goal_x - robot_x
        )

        goal_dy = (
            goal_y - robot_y
        )

        goal_distance = math.sqrt(
            goal_dx * goal_dx
            +
            goal_dy * goal_dy
        )

        if (
            goal_distance
            < self.goal_tolerance
        ):

            self.get_logger().info(
                'Goal reached. '
                'Stopping robot.'
            )

            self.stop_robot()

            self.path = None

            self.current_index = 0

            return

        # ------------------------------------------------------------
        # ADVANCE PATH PROGRESS
        # ------------------------------------------------------------

        # Move current index forward while
        # waypoints are sufficiently close.

        while (
            self.current_index
            <
            len(poses) - 1
        ):

            waypoint = (
                poses[
                    self.current_index
                ]
            )

            dx = (
                waypoint
                .pose
                .position
                .x
                - robot_x
            )

            dy = (
                waypoint
                .pose
                .position
                .y
                - robot_y
            )

            distance = math.sqrt(
                dx * dx
                +
                dy * dy
            )

            if (
                distance
                <
                self.waypoint_tolerance
            ):

                self.current_index += 1

            else:

                break

        # ------------------------------------------------------------
        # FIND LOOKAHEAD TARGET
        # ------------------------------------------------------------

        target_index = (
            self.current_index
        )

        for i in range(
            self.current_index,
            len(poses)
        ):

            dx = (
                poses[i]
                .pose
                .position
                .x
                - robot_x
            )

            dy = (
                poses[i]
                .pose
                .position
                .y
                - robot_y
            )

            distance = math.sqrt(
                dx * dx
                +
                dy * dy
            )

            if (
                distance
                >=
                self.lookahead_distance
            ):

                target_index = i

                break

            target_index = i

        target_pose = (
            poses[
                target_index
            ]
        )

        target_x = (
            target_pose
            .pose
            .position
            .x
        )

        target_y = (
            target_pose
            .pose
            .position
            .y
        )

        # ------------------------------------------------------------
        # TARGET ANGLE
        # ------------------------------------------------------------

        dx = (
            target_x
            - robot_x
        )

        dy = (
            target_y
            - robot_y
        )

        target_angle = (
            math.atan2(
                dy,
                dx
            )
        )

        angle_error = (
            self.normalize_angle(
                target_angle
                - robot_yaw
            )
        )

        # ------------------------------------------------------------
        # ANGULAR CONTROL
        # ------------------------------------------------------------

        angular_z = (
            1.5
            * angle_error
        )

        angular_z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular_z
            )
        )

        # ------------------------------------------------------------
        # LINEAR CONTROL
        # ------------------------------------------------------------

        # Slow down as heading error increases.

        angle_factor = max(
            0.0,
            1.0
            -
            abs(angle_error)
            / 1.2
        )

        linear_x = (
            self.max_linear_speed
            *
            angle_factor
        )

        # Rotate in place if badly misaligned.

        if abs(angle_error) > 0.7:

            linear_x = 0.0

        # ------------------------------------------------------------
        # PUBLISH COMMAND
        # ------------------------------------------------------------

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        cmd.header.frame_id = (
            'base_footprint'
        )

        cmd.twist.linear.x = (
            linear_x
        )

        cmd.twist.angular.z = (
            angular_z
        )

        self.cmd_pub.publish(
            cmd
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = PathFollower()

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
