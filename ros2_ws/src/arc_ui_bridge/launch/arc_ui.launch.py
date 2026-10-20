"""ros2 launch arc_ui_bridge arc_ui.launch.py [host:=0.0.0.0] [port:=8080] [strategy:=exact]

host:=0.0.0.0 lets a phone or tablet on the same hotspot open http://<laptop IP>:8080.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument('host', default_value='127.0.0.1'),
            DeclareLaunchArgument('port', default_value='8080'),
            DeclareLaunchArgument('strategy', default_value='exact'),
            DeclareLaunchArgument('use_sim_time', default_value='true')]
    node = Node(package='arc_ui_bridge', executable='ui_bridge', name='arc_ui_bridge', output='screen',
                parameters=[{'host': LaunchConfiguration('host'), 'port': LaunchConfiguration('port'),
                             'strategy': LaunchConfiguration('strategy'),
                             'use_sim_time': LaunchConfiguration('use_sim_time')}])
    return LaunchDescription(args + [node])
