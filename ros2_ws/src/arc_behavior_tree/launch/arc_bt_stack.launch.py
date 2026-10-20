from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ---------------------------------------------------------
        # Behaviour Tree
        # ---------------------------------------------------------
        Node(
            package='arc_behavior_tree',
            executable='person_assistant_bt',
            name='person_assistant_bt',
            output='screen',
        ),

        # ---------------------------------------------------------
        # Motion Gate
        #
        # /cmd_vel_nav  ->  BT policy  ->  /cmd_vel
        #
        # This remains the ONLY node in the stack that should
        # publish the final /cmd_vel command to Gazebo.
        # ---------------------------------------------------------
        Node(
            package='arc_behavior_tree',
            executable='arc_motion_gate',
            name='arc_motion_gate',
            output='screen',
        ),

    ])
