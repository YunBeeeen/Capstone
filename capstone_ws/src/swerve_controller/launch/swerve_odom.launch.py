"""swerve_odom 실행 — wheel_state + steer_state → /odom (+ TF odom → world).

  ros2 launch swerve_controller swerve_odom.launch.py                  # 실기 (브리지 두 개가 떠 있어야 함)
  ros2 launch swerve_controller swerve_odom.launch.py publish_tf:=false # EKF 가 TF 를 낼 때
  ros2 service call /swerve_odom/reset std_srvs/srv/Trigger            # 포즈 0 으로
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = Path(get_package_share_directory('swerve_controller')) / 'config' / 'swerve_controller.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='true', description='odom → base_frame TF 발행'),
        Node(package='swerve_controller', executable='swerve_odom', name='swerve_odom', output='screen',
             parameters=[str(cfg),
                         {'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool)}]),
    ])
