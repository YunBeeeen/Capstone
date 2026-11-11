from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    share = Path(get_package_share_directory('mobile_robot_description'))
    model = LaunchConfiguration('model').perform(context)
    stem = f'mobile_robot_{model}_collision'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/display.launch.py')),
            launch_arguments={'model': f'{model}_collision',
                              'gui': LaunchConfiguration('gui'),
                              'rviz': LaunchConfiguration('rviz')}.items()),
        Node(package='mobile_robot_description', executable='collision_scene_node',
             output='screen', parameters=[{
                 'robot_description': (share / 'urdf' / f'{stem}.urdf').read_text(),
                 'robot_description_semantic': (share / 'config' / f'{stem}.srdf').read_text(),
             }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='right_arm', choices=['right_arm', 'dual_arm']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=make_nodes),
    ])
