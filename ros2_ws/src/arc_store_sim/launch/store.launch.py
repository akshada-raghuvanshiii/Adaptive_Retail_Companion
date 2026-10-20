"""Launch the ARC store sandbox in Gazebo Harmonic with the ROS 2 bridge.

    ros2 launch arc_store_sim store.launch.py                         # busy store, GUI
    ros2 launch arc_store_sim store.launch.py world:=arc_store_searching.sdf
    ros2 launch arc_store_sim store.launch.py gz_extra_args:='-s --headless-rendering'   # no GUI
    ros2 launch arc_store_sim store.launch.py record:=true            # save overhead camera video

This package only starts the world and the bridge. It does not spawn the ARC robot
and does not touch any navigation package; spawn ARC from its own description
(see README).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("arc_store_sim")
    world = LaunchConfiguration("world")
    world_path = PathJoinSubstitution([pkg, "worlds", world])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={
            "gz_args": ["-r ", world_path, " ", LaunchConfiguration("gz_extra_args")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="arc_store_bridge",
        parameters=[{
            "config_file": PathJoinSubstitution([pkg, "config", "bridge.yaml"]),
            "use_sim_time": True,
        }],
        output="screen",
    )

    recorder = Node(
        package="arc_store_sim",
        executable="camera_recorder",
        name="overhead_cam_recorder",
        parameters=[{
            "image_topic": "/store/overhead_cam/image",
            "output_path": LaunchConfiguration("record_path"),
            "fps": 15.0,
            "use_sim_time": True,
        }],
        condition=IfCondition(LaunchConfiguration("record")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="arc_store_busy.sdf",
                              description="arc_store.sdf | arc_store_normal.sdf | "
                                          "arc_store_searching.sdf | arc_store_busy.sdf"),
        DeclareLaunchArgument("gz_extra_args", default_value="",
                              description="Extra gz sim flags, e.g. '-s --headless-rendering'"),
        DeclareLaunchArgument("record", default_value="false",
                              description="Record the overhead camera to an mp4"),
        DeclareLaunchArgument("record_path", default_value="store_overhead.mp4",
                              description="Output video path (a matching _frames.csv is written too)"),
        gz, bridge, recorder,
    ])
