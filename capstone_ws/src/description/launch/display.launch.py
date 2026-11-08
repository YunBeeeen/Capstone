from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('mobile_robot_right_arm_description'))
    description = (share/'urdf/mobile_robot_right_arm.urdf').read_text()
    state_params = {
        'robot_description': description,
        'use_mimic_tags': True,
        'zeros.lift_stage_1_joint': 0.0,
        'zeros.right_arm_joint2': -2.0,
        'zeros.right_arm_joint3': -1.2,
        'zeros.right_arm_joint4': -1.0,
        'zeros.right_arm_finger_left': 0.035,
    }
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[state_params], condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[state_params], condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share/'rviz/mobile_robot.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
