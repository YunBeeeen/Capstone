"""swerve_controller 실행. 바퀴까지 돌리려면 wheel_can_bridge / steer_can_bridge 를 따로 띄울 것."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = Path(get_package_share_directory('swerve_controller')) / 'config' / 'swerve_controller.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('require_steer_feedback', default_value='true',
                              description='false = 조향 브리지 없이 시험 (정렬 게이트 끔, 바퀴 띄우고만)'),
        Node(package='swerve_controller', executable='swerve_node', name='swerve_controller', output='screen',
             parameters=[str(cfg), {'require_steer_feedback': LaunchConfiguration('require_steer_feedback')}]),
    ])
