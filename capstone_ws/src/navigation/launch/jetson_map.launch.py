"""Jetson 매핑 한 번에 = ZED (zed.launch) + 매핑 (map.launch) + 바닥 보정 자동 (20 초 뒤 zed_floor_level --apply).
  ros2 launch capstone_navigation jetson_map.launch.py map:=X new_map:=true    # 새 지도 ~/maps/X.db
  ros2 launch capstone_navigation jetson_map.launch.py map:=X                  # 기존 X 에 이어서
  ros2 launch capstone_navigation jetson_map.launch.py map:=X floor:=false     # 바닥 보정 빼기
켤 때 로봇 앞에 2 m 이상 빈 바닥이 보이게 둘 것 (그 바닥으로 카메라 기울기를 맞춤). "[바닥 보정] 끝" 이 나오면 돌아다니기.
map.launch.py 의 다른 인자 (cam_roll_offset 등) 도 그대로 붙이면 넘어감.
끄기: Ctrl+C 한 번 → 지도 저장 끝날 때까지 기다림 (프롬프트가 돌아올 때까지).
"""
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jetson_common import floor_level, include  # noqa: E402


def generate_launch_description():
    share = Path(get_package_share_directory("capstone_navigation")) / "launch"
    inc = lambda f: include(share, f)  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument("floor", default_value="true", description="바닥 보정 자동 (zed_floor_level --apply)"),
        inc("zed.launch.py"),
        inc("map.launch.py"),
        floor_level(),
    ])
