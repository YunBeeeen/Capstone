"""위치 추정 + 매핑: EKF(바퀴 odom + IMU 자이로) → RTAB-Map (RGB-D). 시뮬·실기 공용.

  # 시뮬 (Gazebo, odom TF 는 EKF 가 내야 하므로 odom_tf:=false)
  ros2 launch capstone_gazebo world.launch.py odom_tf:=false
  ros2 launch capstone_navigation slam.launch.py use_sim_time:=true

  # 실기: swerve_odom 을 publish_tf:=false 로 + ZED wrapper
  ros2 launch capstone_navigation slam.launch.py

  ros2 launch capstone_navigation slam.launch.py new_map:=true        # 새 지도로 시작 (기존 파일은 백업 후 비움)
  ros2 launch capstone_navigation slam.launch.py                      # 기존 지도에 이어서 매핑 (기본 — 안 지움)
  ros2 launch capstone_navigation slam.launch.py localization:=true   # 저장한 지도에서 위치만 (지도 안 바뀜) ← 대회 2일차

지도 파일: ~/.ros/capstone_rtabmap.db (database_path:=). 매핑 모드로 켤 때마다 기존 파일을
  ~/.ros/rtabmap_backup/<이름>_<날짜_시각>.db 로 복사해 둠 (localization 모드는 파일을 안 바꾸므로 백업 안 함).

입력 (ZED wrapper 이름): /zed/zed_node/rgb/image_rect_color, depth/depth_registered, rgb/camera_info, imu/data, /odom
출력: TF map → odom (RTAB-Map), odom → base_footprint (EKF), /rtabmap/map (2D 점유격자), /rtabmap/cloud_map, /odometry/filtered

타임스탬프: RTAB-Map 은 odom 을 토픽이 아니라 **TF** 로 받음 (odom_frame_id) → 영상 찍힌 시각의 위치를 TF 에서 보간.
  odom 과 영상 주기·시계가 달라도 맞음 (토픽 exact sync 는 스탬프가 똑같아야만 짝지어져서 실기에선 안 됨).
  실기는 RPi(odom) 와 상위 컴퓨터(ZED) 시계를 chrony 로 맞춰야 함.
"""
import shutil
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ZED = "/zed/zed_node/"

# RTAB-Map 라이브러리 파라미터 (문자열). 근거는 README
RTABMAP = {
    "Reg/Force3DoF": "true",              # 평면 로봇: x, y, yaw 만
    "Optimizer/GravitySigma": "0",
    # 루프 클로저 검증 — 반복되는 복도에서 잘못된 루프 클로저 막기 (시뮬 2026-11-11: 3 번 중 2 번 북↔남 복도 착각)
    "Vis/MinInliers": "25",               # 기본 20
    "RGBD/OptimizeMaxError": "3.0",       # 루프 클로저로 그래프를 최적화했을 때 링크 오차가 odom 분산의 이 배수를 넘으면 거부
    "RGBD/ProximityPathMaxNeighbors": "10",
    # 2D 점유격자 (depth 에서)
    "Grid/Sensor": "1",
    "Grid/RangeMax": "5.0",
    "Grid/MaxGroundHeight": "0.05",
    "Grid/MaxObstacleHeight": "2.0",
    "Grid/RayTracing": "true",
}


def nodes(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    sim = get("use_sim_time").lower() == "true"
    loc = get("localization").lower() == "true"
    ekf_cfg = Path(get_package_share_directory("capstone_navigation")) / "config" / "ekf.yaml"
    db = Path(get("database_path")).expanduser()
    new_map = get("new_map").lower() == "true" and not loc
    params = dict(RTABMAP)
    if loc:
        params.update({"Mem/IncrementalMemory": "false", "Mem/InitWMWithAllNodes": "true"})
    rtab = {
        "use_sim_time": sim,
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",          # odom 은 TF 로 (보간) — 토픽 sync 안 씀
        "map_frame_id": "map",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "approx_sync": False,             # 영상·depth 는 같은 카메라라 스탬프 같음
        "wait_for_transform": 0.2,
        # odom TF 에는 공분산이 없어서 여기서 줌: 노드 사이(~1 s, ~0.3 m) odom 오차 크기. OptimizeMaxError 판정 기준이 됨
        "odom_tf_linear_variance": float(get("odom_linear_variance")),
        "odom_tf_angular_variance": float(get("odom_angular_variance")),
        "database_path": str(db),
        **params,
    }
    remap = [("rgb/image", ZED + "rgb/image_rect_color"), ("depth/image", ZED + "depth/depth_registered"),
             ("rgb/camera_info", ZED + "rgb/camera_info")]
    out = []
    if loc and not db.exists():
        out.append(LogInfo(msg=f"[slam] 경고: localization 모드인데 지도 파일이 없음: {db}"))
    if not loc and db.exists() and db.stat().st_size > 0:  # 매핑 모드: 켤 때마다 백업 (새 지도든 이어 쓰기든)
        bk = db.parent / "rtabmap_backup" / f"{db.stem}_{time.strftime('%Y%m%d_%H%M%S')}{db.suffix}"
        bk.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(db, bk)
        out.append(LogInfo(msg=f"[slam] 기존 지도 백업: {bk} ({db.stat().st_size / 1e6:.1f} MB)"))
    out.append(LogInfo(msg=f"[slam] {'localization (지도 고정)' if loc else ('새 지도' if new_map else '기존 지도에 이어서 매핑')}: {db}"))
    out += [
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
             parameters=[str(ekf_cfg), {"use_sim_time": sim}]),
        Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="rtabmap", output="screen",
             parameters=[rtab], remappings=remap,
             arguments=["-d"] if new_map else []),
    ]
    if get("viz").lower() == "true":
        out.append(Node(package="rtabmap_viz", executable="rtabmap_viz", namespace="rtabmap", output="log",
                        parameters=[{"use_sim_time": sim, "frame_id": "base_footprint", "odom_frame_id": "odom",
                                     "subscribe_depth": True, "subscribe_rgb": True, "approx_sync": False}],
                        remappings=remap))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("localization", default_value="false", description="true = 저장 지도에서 위치만"),
        DeclareLaunchArgument("new_map", default_value="false",
                              description="true = 새 지도로 시작 (기존 파일은 백업 후 비움). 기본 false = 이어서"),
        DeclareLaunchArgument("database_path", default_value=str(Path.home() / ".ros" / "capstone_rtabmap.db")),
        DeclareLaunchArgument("odom_linear_variance", default_value="0.0001", description="[m²] 노드 사이 odom 위치 분산"),
        DeclareLaunchArgument("odom_angular_variance", default_value="0.0001", description="[rad²] 노드 사이 odom yaw 분산"),
        DeclareLaunchArgument("viz", default_value="false", description="rtabmap_viz 창"),
        OpaqueFunction(function=nodes),
    ])
