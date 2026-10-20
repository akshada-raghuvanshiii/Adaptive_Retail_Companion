"""ARC motor bridge on its own: /cmd_vel -> Uno -> wheels.

    ros2 launch arc_hw_bridge arc_hw.launch.py
    ros2 launch arc_hw_bridge arc_hw.launch.py port:=/dev/ttyUSB0
    ros2 launch arc_hw_bridge arc_hw.launch.py params_file:=/path/to/my_arc_hw.yaml

Start it after the ARC stack (bringup.sh all), in a terminal that has sourced ~/arc_test_kit/env.sh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("arc_hw_bridge"), "config", "arc_hw.yaml")
    params_file = LaunchConfiguration("params_file")
    port = LaunchConfiguration("port")
    port_given = PythonExpression(["'", port, "' != ''"])
    common = dict(package="arc_hw_bridge", executable="motor_bridge", name="arc_hw_bridge",
                  output="screen", emulate_tty=True)
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params, description="YAML with the bridge settings"),
        DeclareLaunchArgument("port", default_value="", description="override the serial port from the YAML"),
        Node(**common, parameters=[params_file], condition=UnlessCondition(port_given)),
        Node(**common, parameters=[params_file, {"port": port}], condition=IfCondition(port_given)),
    ])
