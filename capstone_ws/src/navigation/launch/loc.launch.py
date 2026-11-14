"""실기 재위치 (지도 고정) — map.launch.py 에 localization:=true.  인자는 map.launch.py 와 같음.

  ros2 launch capstone_navigation loc.launch.py map:=venue
  ros2 launch capstone_navigation drive.launch.py        # 그 위에 Nav2
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    m = Path(get_package_share_directory("capstone_navigation")) / "launch" / "map.launch.py"
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(m)), launch_arguments={"localization": "true"}.items()),
    ])
