"""실기 Nav2 — nav.launch.py 에 실기 값을 기본으로.  ros2 launch capstone_navigation drive.launch.py

= nav.launch.py odom_topic:=/zed_base_tf/odom cmd_vel_topic:=/nav/cmd_vel
나머지 인자(inflation_radius, floor_clear, cloud_max_depth, params_file)는 nav.launch.py 것을 그대로 받음:
  ros2 launch capstone_navigation drive.launch.py inflation_radius:=0.30
속도 명령은 /nav/cmd_vel 로 나가고, 가상 패드(tools/pad.sh)에서 RT 를 누르는 동안만 로봇에 전달됨.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav = Path(get_package_share_directory("capstone_navigation")) / "launch" / "nav.launch.py"
    d = {"odom_topic": "/zed_base_tf/odom", "cmd_vel_topic": "/nav/cmd_vel"}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in d.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav)),
                                 launch_arguments={k: LaunchConfiguration(k) for k in d}.items()),
    ])
