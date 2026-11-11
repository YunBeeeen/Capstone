from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
def generate_launch_description():
    p=Path(get_package_share_directory('mobile_robot_description'))
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'launch/display.launch.py')),launch_arguments={'model':'right_arm'}.items())])
