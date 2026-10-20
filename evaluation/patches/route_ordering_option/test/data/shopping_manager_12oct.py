#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)


class ShoppingManager(Node):

    def __init__(self):

        super().__init__('shopping_manager')

        # ============================================================
        # QoS FOR CURRENT SHOPPING STATE
        # ============================================================

        shopping_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/shopping_goal',
            shopping_qos
        )

        self.item_publisher = self.create_publisher(
            String,
            '/shopping_item',
            shopping_qos
        )

        # ============================================================
        # NAVIGATION COMPLETION SUBSCRIBER
        # ============================================================

        self.completion_subscriber = self.create_subscription(
            Bool,
            '/navigation_goal_reached',
            self.navigation_complete_callback,
            shopping_qos
        )

        # ============================================================
        # LOAD GENERATED STORE ITEM LOCATIONS
        # ============================================================

        item_locations_path = os.path.expanduser(
            '~/slam_ws/src/arc_store_sim/config/item_locations.yaml'
        )

        try:
            with open(item_locations_path, 'r') as file:
                item_data = yaml.safe_load(file) or {}
        except Exception as error:
            self.get_logger().error(
                f'Failed to load item locations: {error}'
            )
            raise

        self.item_frame = item_data.get('frame_id', 'world')
        self.items = item_data.get('items', {})

        self.get_logger().info(
            f'Loaded {len(self.items)} item locations '
            f'from {item_locations_path}.'
        )
        self.get_logger().info(
            f'Item location frame: {self.item_frame}'
        )
        self.get_logger().info('Shopping Manager ready.')

        # ============================================================
        # GET SHOPPING LIST
        # ============================================================

        shopping_list = self.get_shopping_list()

        if not shopping_list:

            self.get_logger().warn(
                'Shopping list is empty.'
            )

            return

        # ============================================================
        # DISPLAY FINAL LIST
        # ============================================================

        self.display_shopping_list(
            shopping_list,
            title='FINAL SHOPPING LIST'
        )

        # ============================================================
        # CREATE NAVIGATION GOALS
        # ============================================================

        self.navigation_goals = (
            self.create_navigation_goals(
                shopping_list
            )
        )

        if not self.navigation_goals:

            self.get_logger().error(
                'No valid navigation goals were created.'
            )

            return

        # ============================================================
        # DISPLAY NAVIGATION SEQUENCE
        # ============================================================

        self.display_navigation_goals(
            self.navigation_goals
        )

        # ============================================================
        # SHOPPING STATE
        # ============================================================

        self.current_goal_index = 0

        self.waiting_for_confirmation = False

        self.navigation_complete = False

        # ============================================================
        # START FIRST GOAL
        # ============================================================

        self.publish_current_goal()

    # ================================================================
    # SHOPPING LIST INPUT
    # ================================================================

    def get_shopping_list(self):

        shopping_list = []

        self.get_logger().info(
            'You can add as many items as required.'
        )

        self.get_logger().info(
            'Items must exist in the store database.'
        )

        while True:

            user_input = input(
                '\nAdd items (comma separated): '
            ).strip()

            if not user_input:

                print(
                    'No items entered. Please try again.'
                )

                continue

            new_items = [
                item.strip().lower()
                for item in user_input.split(',')
                if item.strip()
            ]

            for item in new_items:

                if item not in self.items:

                    self.get_logger().warn(
                        f'"{item}" is not in the '
                        f'store database and will not '
                        f'be added.'
                    )

                    continue

                shopping_list.append(item)

            self.display_shopping_list(
                shopping_list
            )

            while True:

                add_more = input(
                    '\nAdd more items? (y/n): '
                ).strip().lower()

                if add_more in ['y', 'yes']:
                    break

                if add_more in ['n', 'no']:
                    break

                print(
                    'Please enter y or n.'
                )

            if add_more in ['y', 'yes']:

                continue

            while True:

                confirm = input(
                    '\nConfirm shopping list? (y/n): '
                ).strip().lower()

                if confirm in ['y', 'yes']:

                    return shopping_list

                if confirm in ['n', 'no']:

                    print(
                        '\nList not confirmed.'
                    )

                    print(
                        'You can add more items.'
                    )

                    break

                print(
                    'Please enter y or n.'
                )

    # ================================================================
    # DISPLAY SHOPPING LIST
    # ================================================================

    def display_shopping_list(
        self,
        shopping_list,
        title='CURRENT SHOPPING LIST'
    ):

        print(
            f'\n========== {title} =========='
        )

        if not shopping_list:

            print(
                '  (empty)'
            )

        else:

            for index, item in enumerate(
                shopping_list,
                start=1
            ):

                print(
                    f'  {index}. {item}'
                )

        print(
            '==================================='
        )

    # ================================================================
    # CREATE NAVIGATION GOALS
    # ================================================================

    def create_navigation_goals(self, shopping_list):

        navigation_goals = []

        for item in shopping_list:
            location = self.items.get(item)

            if not location or 'x' not in location or 'y' not in location:
                self.get_logger().error(
                    f'No valid pick coordinates found for "{item}".'
                )
                continue

            shelf_id = location.get('shelf', 'unknown')

            navigation_goals.append({
                'item': item,
                'shelf_id': shelf_id,
                'shelf_name': shelf_id,
                'x': float(location['x']),
                'y': float(location['y']),
                'yaw': float(location.get('yaw', 0.0)),
            })

        return navigation_goals

    # ================================================================
    # DISPLAY NAVIGATION GOALS
    # ================================================================

    def display_navigation_goals(
        self,
        navigation_goals
    ):

        print(
            '\n========== NAVIGATION SEQUENCE =========='
        )

        for index, goal in enumerate(
            navigation_goals,
            start=1
        ):

            print(
                f'\n  Goal {index}'
            )

            print(
                f'    Item:   {goal["item"]}'
            )

            print(
                f'    Shelf:  '
                f'{goal["shelf_id"]} '
                f'({goal["shelf_name"]})'
            )

            print(
                f'    Target: '
                f'x={goal["x"]:.2f}, '
                f'y={goal["y"]:.2f}'
            )

        print(
            '\n=========================================='
        )

    # ================================================================
    # PUBLISH CURRENT GOAL
    # ================================================================

    def publish_current_goal(self):

        if self.current_goal_index >= len(
            self.navigation_goals
        ):

            self.get_logger().info(
                'All shopping goals completed.'
            )

            self.get_logger().info(
                'Shopping trip finished.'
            )

            return

        goal = self.navigation_goals[
            self.current_goal_index
        ]

        # ------------------------------------------------------------
        # Reset navigation state
        # ------------------------------------------------------------

        self.waiting_for_confirmation = False
        self.navigation_complete = False

        # ------------------------------------------------------------
        # Publish current item
        # ------------------------------------------------------------

        item_message = String()

        item_message.data = goal['item']

        self.item_publisher.publish(
            item_message
        )

        # ------------------------------------------------------------
        # Publish navigation goal
        # ------------------------------------------------------------

        goal_message = PoseStamped()

        goal_message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Item coordinates are generated in item_locations.yaml's frame.
        goal_message.header.frame_id = self.item_frame

        goal_message.pose.position.x = goal['x']
        goal_message.pose.position.y = goal['y']
        goal_message.pose.position.z = 0.0

        import math
        yaw = goal.get('yaw', 0.0)
        goal_message.pose.orientation.x = 0.0
        goal_message.pose.orientation.y = 0.0
        goal_message.pose.orientation.z = math.sin(yaw / 2.0)
        goal_message.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_publisher.publish(
            goal_message
        )

        self.get_logger().info(
            f'Published shopping goal '
            f'{self.current_goal_index + 1}/'
            f'{len(self.navigation_goals)}: '
            f'{goal["item"]} → '
            f'({goal["x"]:.2f}, {goal["y"]:.2f})'
        )

        print(
            f'\n>>> Robot navigating to: '
            f'{goal["item"]} '
            f'({goal["shelf_name"]})'
        )

    # ================================================================
    # NAVIGATION COMPLETION CALLBACK
    # ================================================================

    def navigation_complete_callback(self, msg):

        if not msg.data:

            return

        # Ignore completion messages when we are not
        # currently navigating.

        if self.navigation_complete:

            return

        self.navigation_complete = True

        current_goal = self.navigation_goals[
            self.current_goal_index
        ]

        self.get_logger().info(
            f'Navigation completed for '
            f'"{current_goal["item"]}".'
        )

        # ============================================================
        # LAST ITEM
        # ============================================================

        if (
            self.current_goal_index
            >= len(self.navigation_goals) - 1
        ):

            print(
                f'\n✓ Reached final item: '
                f'{current_goal["item"]}'
            )

            print(
                '\n========== SHOPPING COMPLETE =========='
            )

            self.waiting_for_confirmation = True

            return

        # ============================================================
        # ASK USER BEFORE NEXT ITEM
        # ============================================================

        next_goal = self.navigation_goals[
            self.current_goal_index + 1
        ]

        print(
            '\n=========================================='
        )

        print(
            f'✓ Reached: {current_goal["item"]}'
        )

        print(
            f'Next item: {next_goal["item"]} '
            f'({next_goal["shelf_name"]})'
        )

        print(
            '=========================================='
        )

        while True:

            confirmation = input(
                '\nContinue to next item? (y/n): '
            ).strip().lower()

            if confirmation in ['y', 'yes']:

                self.current_goal_index += 1

                self.publish_current_goal()

                return

            if confirmation in ['n', 'no']:

                self.get_logger().info(
                    'User chose not to continue '
                    'to the next shopping item.'
                )

                print(
                    '\nRobot will remain stopped '
                    'at the current location.'
                )

                self.waiting_for_confirmation = True

                return

            print(
                'Please enter y or n.'
            )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ShoppingManager()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
