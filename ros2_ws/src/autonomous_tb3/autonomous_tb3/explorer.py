import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleExplorer(Node):

    def __init__(self):
        super().__init__('simple_explorer')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):

        ranges = list(msg.ranges)

        n = len(ranges)

        # Front sector: roughly +/- 15 degrees
        front = ranges[0:int(n * 0.04)] + ranges[int(n * 0.96):]

        # Left sector
        left = ranges[int(n * 0.20):int(n * 0.30)]

        # Right sector
        right = ranges[int(n * 0.70):int(n * 0.80)]

        def minimum_valid(values):
            valid = [
                x for x in values
                if msg.range_min < x < msg.range_max
            ]

            return min(valid) if valid else float('inf')

        self.front_distance = minimum_valid(front)
        self.left_distance = minimum_valid(left)
        self.right_distance = minimum_valid(right)

    def control_loop(self):

        cmd = Twist()

        # Something close in front
        if self.front_distance < 0.6:

            cmd.linear.x = 0.0

            # Turn toward the side with more free space
            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.6
            else:
                cmd.angular.z = -0.6

        else:

            # Nothing immediately ahead
            cmd.linear.x = 0.08
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = SimpleExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
