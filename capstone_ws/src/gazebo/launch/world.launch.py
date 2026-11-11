"""Gazebo Classic — school_floor 월드 (+ 시험용 로봇).

  ros2 launch capstone_gazebo world.launch.py                      # 월드 + 시험용 로봇 (박스 + ZED 흉내 카메라)
  ros2 launch capstone_gazebo world.launch.py robot:=none          # 월드만
  ros2 launch capstone_gazebo world.launch.py gui:=false           # 창 없이 (gzserver 만)
  ros2 launch capstone_gazebo world.launch.py x:=10 y:=11 yaw:=3.14  # 스폰 위치 (기본 = 남서쪽 복도)
  ros2 launch capstone_gazebo world.launch.py robot:=/경로/모델.urdf.xacro   # 다른 로봇 (xacro/urdf)
  ros2 launch capstone_gazebo world.launch.py odom_tf:=false       # odom TF 끔 — RTAB-Map 시각 odometry 가 TF 를 냄
  ros2 launch capstone_gazebo world.launch.py odom_noise:=false    # /odom 오차 끔 (기본: 실기 같은 오차 — noisy_odom.py)

  조종: ros2 run capstone_teleop virtual_pad.py   (또는 teleop:=true 로 SHAKS 게임패드)
  카메라: ros2 run rqt_image_view rqt_image_view /zed/zed_node/rgb/color/rect/image
실기 DDS 망과 섞이지 않게: export ROS_DOMAIN_ID=<다른 번호>
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def robot_nodes(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot in ('', 'none'):
        return []
    share = Path(get_package_share_directory('capstone_gazebo'))
    path = share / 'urdf' / 'test_rig.urdf.xacro' if robot == 'test_rig' else Path(robot)
    desc = ParameterValue(Command(['xacro ', str(path),
                                   ' cam_z:=', LaunchConfiguration('cam_z'),
                                   ' cam_pitch:=', LaunchConfiguration('cam_pitch')]), value_type=str)
    get = lambda k: LaunchConfiguration(k).perform(context)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': desc, 'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-topic', 'robot_description', '-entity', 'robot',
                        '-x', get('x'), '-y', get('y'), '-z', '0.0', '-Y', get('yaw')]),
        # 가짜 바퀴 odom: planar_move(완벽) + 실기 같은 오차 → /odom + TF (실기 swerve_odom 과 같은 토픽)
        Node(package='capstone_gazebo', executable='noisy_odom.py', name='noisy_odom', output='screen',
             parameters=[{'use_sim_time': True,
                          'noise': ParameterValue(LaunchConfiguration('odom_noise'), value_type=bool),
                          'publish_tf': ParameterValue(LaunchConfiguration('odom_tf'), value_type=bool)}]),
        # 가짜 ZED VIO: 참값 + VIO 수준 드리프트 → /zed/zed_node/odom (실기 ZED SDK GEN_3 자리, TF 없음)
        Node(package='capstone_gazebo', executable='fake_vio.py', name='fake_vio', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('vio'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('capstone_gazebo'))
    gz = Path(get_package_share_directory('gazebo_ros')) / 'launch' / 'gazebo.launch.py'
    teleop = Path(get_package_share_directory('capstone_teleop')) / 'launch' / 'teleop.launch.py'
    models = str(share / 'models')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'school_floor.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('robot', default_value='test_rig', description="test_rig | none | <xacro/urdf 경로>"),
        DeclareLaunchArgument('cam_z', default_value='0.69', description='시험용 로봇 카메라 높이 [m] (실기 리프트 최저 0.69)'),
        DeclareLaunchArgument('cam_pitch', default_value='0.377', description='시험용 로봇 카메라 아래로 [rad] (실기 IMU 실측 21.6°)'),
        DeclareLaunchArgument('odom_tf', default_value='true',
                              description='false = /odom 의 TF odom→base 를 안 냄 (RTAB-Map 시각 odometry 가 TF 를 낼 때)'),
        DeclareLaunchArgument('odom_noise', default_value='true',
                              description='/odom 에 실기 같은 오차 (false = 완벽한 odom)'),
        DeclareLaunchArgument('vio', default_value='true',
                              description='가짜 ZED VIO (/zed/zed_node/odom) — 실기 GEN_3 자리. 열화 구간에선 오차 큼'),
        DeclareLaunchArgument('x', default_value='1.25'),
        DeclareLaunchArgument('y', default_value='1.25'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('teleop', default_value='false', description='SHAKS S0 게임패드 (RB 누른 채 스틱)'),
        # 텍스처(model://capstone_materials) 찾는 경로. package.xml export 로도 들어가지만 확실하게
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(
            p for p in (models, os.environ.get('GAZEBO_MODEL_PATH', ''), '/usr/share/gazebo-11/models') if p)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),  # 온라인 모델 DB 조회 끔 (없으면 시작이 수십 초 멈춤)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz)), launch_arguments={
            'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'), 'verbose': 'false'}.items()),
        OpaqueFunction(function=robot_nodes),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop)),
                                 condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
