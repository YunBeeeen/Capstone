# SHAKS S0 → joy_node(/joy) → gamepad_teleop(원형 데드존) → /cmd_vel
#   ros2 launch capstone_teleop teleop.launch.py
#   ros2 launch capstone_teleop teleop.launch.py cmd_vel_topic:=/test_cmd_vel   # 모터 안 붙이고 부호 확인용
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = str(Path(get_package_share_directory("capstone_teleop")) / "config" / "shaks_s0.yaml")
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="capstone_teleop",
            executable="gamepad_teleop.py",
            name="gamepad_teleop",
            output="screen",
            parameters=[params],
            remappings=[("cmd_vel", LaunchConfiguration("cmd_vel_topic"))],
        ),
    ])
