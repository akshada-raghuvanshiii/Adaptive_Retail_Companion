"""ros2 launch arc_ui_bridge arc_ui.launch.py [host:=0.0.0.0] [port:=8080] [strategy:=exact]
                                              [route_choice:=ask|fixed] [end_at_checkout:=true|false]

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
            DeclareLaunchArgument('route_choice', default_value='ask'),
            DeclareLaunchArgument('end_at_checkout', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='true')]
    node = Node(package='arc_ui_bridge', executable='ui_bridge', name='arc_ui_bridge', output='screen',
                parameters=[{'host': LaunchConfiguration('host'), 'port': LaunchConfiguration('port'),
                             'strategy': LaunchConfiguration('strategy'),
                             'route_choice': LaunchConfiguration('route_choice'),
                             'end_at_checkout': LaunchConfiguration('end_at_checkout'),
                             'use_sim_time': LaunchConfiguration('use_sim_time')}])
    return LaunchDescription(args + [node])
