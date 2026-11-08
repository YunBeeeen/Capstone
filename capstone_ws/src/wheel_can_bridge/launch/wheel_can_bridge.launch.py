from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = str(Path(get_package_share_directory("wheel_can_bridge")) / "config" / "wheel_can_bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can_drive"),
        Node(
            package="wheel_can_bridge",
            executable="wheel_can_bridge_node",
            name="wheel_can_bridge",
            output="screen",
            parameters=[params, {"can_interface": LaunchConfiguration("can_interface")}],
        ),
    ])
