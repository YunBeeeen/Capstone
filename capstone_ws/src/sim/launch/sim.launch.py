"""MuJoCo 시뮬 + swerve_controller (실기와 같은 설정 파일).

  ros2 launch capstone_sim sim.launch.py                    # 뷰어 + 컨트롤러
  ros2 launch capstone_sim sim.launch.py pad:=true          # + 화면 가상 패드 (마우스)
  ros2 launch capstone_sim sim.launch.py teleop:=true       # + SHAKS S0 게임패드 (RB 누른 채 스틱)
  ros2 launch capstone_sim sim.launch.py headless:=true     # MuJoCo 창 없이
  ros2 launch capstone_sim sim.launch.py rviz:=true         # + RViz
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    swerve_cfg = Path(get_package_share_directory('swerve_controller')) / 'config' / 'swerve_controller.yaml'
    teleop_launch = Path(get_package_share_directory('capstone_teleop')) / 'launch' / 'teleop.launch.py'
    desc = Path(get_package_share_directory('mobile_robot_right_arm_description'))
    urdf = (desc / 'urdf' / 'mobile_robot_right_arm.urdf').read_text()  # MJCF 와 같은 원본
    rviz_cfg = Path(get_package_share_directory('capstone_sim')) / 'rviz' / 'sim.rviz'
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false', description='SHAKS S0 게임패드 teleop 같이 띄움'),
        DeclareLaunchArgument('pad', default_value='false', description='화면 가상 패드 같이 띄움'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz (시뮬 joint_states + TF odom→world)'),
        # RViz: 시뮬이 joint_states 와 TF odom→world 를 내고, 나머지 링크 TF 는 URDF 로 계산
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz_cfg)], output='log',
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='capstone_sim', executable='mujoco_sim.py', name='mujoco_sim', output='screen',
             parameters=[{'headless': LaunchConfiguration('headless')}]),
        Node(package='swerve_controller', executable='swerve_node', name='swerve_controller', output='screen',
             parameters=[str(swerve_cfg)]),
        Node(package='capstone_teleop', executable='virtual_pad.py', name='virtual_pad', output='screen',
             condition=IfCondition(LaunchConfiguration('pad'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop_launch)),
                                 condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
