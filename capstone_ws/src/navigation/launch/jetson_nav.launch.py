"""Jetson 내비게이션 한 번에 = ZED + 재위치 (loc.launch) + 바닥 보정 자동 + Nav2 (drive.launch) + Nav2 명령 중계 (nav_cmd_udp → 라파 패드).
  ros2 launch capstone_navigation jetson_nav.launch.py map:=X
  ros2 launch capstone_navigation jetson_nav.launch.py map:=X floor:=false relay:=false
순서: 바로 ZED·재위치 → 20 초 뒤 바닥 보정 → 25 초 뒤 Nav2 ("Managed nodes are active") · 중계 ("192.168.x.12 로 보냄").
켤 때 로봇 앞에 2 m 이상 빈 바닥이 보이게. 그다음 다른 터미널에서 위치 찾기 (rtabmap_loc_monitor.py) → 장소로 (nav_goal_ahead.py).
게임패드는 라파 (pad_pi.sh): RT 당기는 동안만 Nav2 명령이 로봇에 감.
"""
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jetson_common import floor_level, include  # noqa: E402


def generate_launch_description():
    share = Path(get_package_share_directory("capstone_navigation")) / "launch"
    inc = lambda f: include(share, f)  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument("floor", default_value="true", description="바닥 보정 자동 (zed_floor_level --apply)"),
        DeclareLaunchArgument("relay", default_value="true", description="Nav2 명령을 라파 패드로 (nav_cmd_udp.py)"),
        inc("zed.launch.py"),
        inc("loc.launch.py"),
        floor_level(),
        TimerAction(period=25.0, actions=[inc("drive.launch.py")]),
        TimerAction(period=25.0, condition=IfCondition(LaunchConfiguration("relay")), actions=[
            Node(package="capstone_teleop", executable="nav_cmd_udp.py", name="nav_cmd_udp", output="screen")]),
    ])
