"""실기 매핑 — slam.launch.py 에 이 로봇 값을 기본으로 넣은 것. 바꾸고 싶은 것만 인자로.

  ros2 launch capstone_navigation map.launch.py map:=venue new_map:=true    # 새 지도 ~/maps/venue.db
  ros2 launch capstone_navigation map.launch.py map:=venue                  # 그 지도에 이어서
  ros2 launch capstone_navigation map.launch.py map:=venue cam_roll_offset:=-0.9   # 보정값만 바꿈
재위치는 loc.launch.py (같은 인자).

기본값 (2026-11 실측, 리프트 최저): odom_source zed, cam_x 0.167, cam_z 0.675, cam_pitch_offset 1.3, cam_roll_offset 0.7.
  카메라를 다시 달거나 보정값이 바뀌면 아래 ROBOT 을 고치고 colcon build --packages-select capstone_navigation.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

ROBOT = {"odom_source": "zed", "cam_x": "0.167", "cam_y": "0.0", "cam_z": "0.675",
         "cam_pitch_offset": "1.3", "cam_roll_offset": "0.7"}
PASS = {"new_map": "false", "localization": "false", "ground_height": "0.20", "viz": "false"}


def generate_launch_description():
    slam = Path(get_package_share_directory("capstone_navigation")) / "launch" / "slam.launch.py"
    db = PathJoinSubstitution([LaunchConfiguration("map_dir"), PythonExpression(["'", LaunchConfiguration("map"), "' + '.db'"])])
    args = {k: LaunchConfiguration(k) for k in {**ROBOT, **PASS}}
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="venue", description="지도 이름 → <map_dir>/<map>.db"),
        DeclareLaunchArgument("map_dir", default_value=str(Path.home() / "maps")),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in {**ROBOT, **PASS}.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam)),
                                 launch_arguments={**args, "database_path": db}.items()),
    ])
