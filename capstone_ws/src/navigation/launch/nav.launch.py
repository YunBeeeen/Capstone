"""Nav2 주행 — slam.launch.py (EKF + RTAB-Map) 위에 띄움. 시뮬·실기 공용.

  ros2 launch capstone_navigation nav.launch.py use_sim_time:=true    # 시뮬
  ros2 launch capstone_navigation nav.launch.py                       # 실기 (EKF)
  ros2 launch capstone_navigation nav.launch.py odom_topic:=/zed_base_tf/odom cmd_vel_topic:=/nav/cmd_vel   # 실기 ZED 단독
     → 가상 패드: virtual_pad.py --udp <라파> --nav-topic /nav/cmd_vel  (N 누르는 동안만 Nav2 명령을 로봇에 보냄)
  목표: RViz "Nav2 Goal" 또는
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"

구성:
  ZED depth ─▶ rtabmap_util/point_cloud_xyz (4 배 추출, 5 cm 격자, 8 m 까지) ─▶ /nav/obstacle_cloud ─▶ costmap 장애물
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
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        DeclareLaunchArgument("inflation_radius", default_value="0.45",
                              description="장애물 주변 여유 구역 폭 [m] (costmap 두 곳 + MPPI 충돌 판정이 같이 바뀜). 좁은 통로에서 머뭇거리면 0.30"),
        DeclareLaunchArgument("floor_clear", default_value="true", choices=["true", "false"],
                              description="true = 바닥 점으로도 장애물 지우기 (가짜 장애물이 남는 문제). false = 예전 방식"),
        DeclareLaunchArgument("cloud_max_depth", default_value="8.0",
                              description="장애물 점구름 최대 거리 [m]. 표시는 nav2.yaml obstacle_max_range 2.5 m 까지만, 그 너머 점은 지우기에만 씀"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel",
                              description="Nav2 최종 속도 명령 토픽 (시뮬 /cmd_vel, 실기 /nav/cmd_vel → 가상 패드 데드맨)"),
        Node(package="rtabmap_util", executable="point_cloud_xyz", name="obstacle_cloud", output="log",
             parameters=[{"use_sim_time": sim, "decimation": 4, "voxel_size": 0.05, "max_depth": ParameterValue(LaunchConfiguration("cloud_max_depth"), value_type=float),
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
                                             param_rewrites={"odom_topic": LaunchConfiguration("odom_topic"),
                                                             "inflation_radius": LaunchConfiguration("inflation_radius"),
                                                             "observation_sources": PythonExpression([
                                                                 "'pointcloud clear' if '", LaunchConfiguration("floor_clear"),
                                                                 "' == 'true' else 'pointcloud'"])},
                                             convert_types=True),
                "autostart": "true",
                "use_composition": "False",
            }.items()),
        ]),
    ])
