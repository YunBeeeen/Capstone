"""Nav2 주행 — slam.launch.py (EKF + RTAB-Map) 위에 띄움. 시뮬·실기 공용.

  ros2 launch capstone_navigation nav.launch.py use_sim_time:=true    # 시뮬
  ros2 launch capstone_navigation nav.launch.py                       # 실기 (EKF)
  ros2 launch capstone_navigation nav.launch.py odom_topic:=/zed_base_tf/odom cmd_vel_topic:=/nav/cmd_vel   # 실기 ZED 단독
     → 가상 패드: virtual_pad.py --udp <라파> --nav-topic /nav/cmd_vel  (N 누르는 동안만 Nav2 명령을 로봇에 보냄)
  목표: RViz "Nav2 Goal" 또는
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"

구성:
  ZED depth ─▶ rtabmap_util/point_cloud_xyz (4 배 추출, 5 cm 격자, 4 m 까지) ─▶ /nav/obstacle_cloud ─▶ costmap 장애물
  /rtabmap/map ─▶ 전역 costmap 정적 층       TF map→odom (RTAB-Map), odom→base_footprint (EKF)
  controller(MPPI) ─▶ cmd_vel_nav ─▶ velocity_smoother ─▶ /cmd_vel ─▶ swerve_controller (실기) / planar_move (시뮬)
설정: config/nav2.yaml (근거는 파일 머리 주석)
TODO 실기: 게임패드가 Nav2 보다 우선하도록 twist_mux (ros-humble-twist-mux), Collision Monitor
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

ZED = "/zed/zed_node/"


def generate_launch_description():
    share = Path(get_package_share_directory("capstone_navigation"))
    bringup = Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py"
    sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "nav2.yaml")),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered",
                              description="Nav2 가 속도를 읽는 odom (EKF /odometry/filtered, ZED 단독이면 /zed_base_tf/odom)"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel",
                              description="Nav2 최종 속도 명령 토픽 (시뮬 /cmd_vel, 실기 /nav/cmd_vel → 가상 패드 데드맨)"),
        Node(package="rtabmap_util", executable="point_cloud_xyz", name="obstacle_cloud", output="log",
             parameters=[{"use_sim_time": sim, "decimation": 4, "voxel_size": 0.05, "max_depth": 4.0,
                          "approx_sync": False}],
             remappings=[("depth/image", ZED + "depth/depth_registered"),
                         ("depth/camera_info", ZED + "depth/camera_info"),
                         ("cloud", "/nav/obstacle_cloud")]),
        # Nav2 출력 /cmd_vel → cmd_vel_topic (실기 기본 /nav/cmd_vel: 로봇에 바로 안 감. 가상 패드가 데드맨 누를 때만 UDP 로 전달)
        GroupAction([
            # velocity_smoother 출력 이름 cmd_vel_smoothed 를 바꿈. 전역 remap 이 노드 remap 보다 먼저 맞아서
            # src="/cmd_vel" 로 하면 controller 출력까지 바뀌어 smoother 를 건너뜀 (2026-11-12 확인)
            SetRemap(src="cmd_vel_smoothed", dst=LaunchConfiguration("cmd_vel_topic")),
            # behavior_server(회복 동작: 회전·후진)는 Humble bringup 에서 /cmd_vel 로 바로 냄 → 모두 smoother 입력으로 모음
            SetRemap(src="cmd_vel", dst="cmd_vel_nav"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup)), launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": RewrittenYaml(source_file=LaunchConfiguration("params_file"),
                                             param_rewrites={"odom_topic": LaunchConfiguration("odom_topic")},
                                             convert_types=True),
                "autostart": "true",
                "use_composition": "False",
            }.items()),
        ]),
    ])
