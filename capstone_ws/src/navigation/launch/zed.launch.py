"""실기 ZED 2i — 늘 쓰던 인자를 기본값으로.  ros2 launch capstone_navigation zed.launch.py

= zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed publish_tf:=false publish_map_tf:=false
  ros_params_override_path:=<config/zed_vio_only.yaml>
TF 는 zed_base_tf.py (map.launch / loc.launch) 가 내므로 publish_tf 는 false.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("capstone_navigation"))
    zed = Path(get_package_share_directory("zed_wrapper")) / "launch" / "zed_camera.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("camera_model", default_value="zed2i"),
        DeclareLaunchArgument("params", default_value=str(share / "config" / "zed_vio_only.yaml"), description="ZED 설정 덮어쓰기 파일"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(zed)), launch_arguments={
            "camera_model": LaunchConfiguration("camera_model"), "camera_name": "zed",
            "publish_tf": "false", "publish_map_tf": "false",
            "ros_params_override_path": LaunchConfiguration("params"),
        }.items()),
    ])
