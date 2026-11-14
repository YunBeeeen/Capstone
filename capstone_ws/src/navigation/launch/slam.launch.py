"""위치 추정 + 매핑: EKF(바퀴 odom + IMU 자이로) → RTAB-Map (RGB-D). 시뮬·실기 공용.

  # 시뮬 (Gazebo, odom TF 는 EKF 가 내야 하므로 odom_tf:=false)
  ros2 launch capstone_gazebo world.launch.py odom_tf:=false
  ros2 launch capstone_navigation slam.launch.py use_sim_time:=true

  # 실기: swerve_odom 을 publish_tf:=false 로 + ZED wrapper
  ros2 launch capstone_navigation slam.launch.py

  ros2 launch capstone_navigation slam.launch.py new_map:=true        # 새 지도로 시작 (기존 파일은 백업 후 비움)
  ros2 launch capstone_navigation slam.launch.py                      # 기존 지도에 이어서 매핑 (기본 — 안 지움)
  ros2 launch capstone_navigation slam.launch.py localization:=true   # 저장한 지도에서 위치만 (지도 안 바뀜) ← 대회 2일차

  # ZED 단독 (바퀴 odom 이 노트북에 안 올 때): ZED wrapper 를 publish_tf:=false + config/zed_vio_only.yaml 로 띄우고
  ros2 launch capstone_navigation slam.launch.py odom_source:=zed new_map:=true cam_z:=1.0   # 카메라 pitch 는 자동

지도 파일: ~/.ros/capstone_rtabmap.db (database_path:=). 매핑 모드로 켤 때마다 기존 파일을
  ~/.ros/rtabmap_backup/<이름>_<날짜_시각>.db 로 복사해 둠 (localization 모드는 파일을 안 바꾸므로 백업 안 함).

입력 (ZED wrapper 이름): /zed/zed_node/rgb/color/rect/image, depth/depth_registered, rgb/color/rect/camera_info, imu/data, /odom
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
    "Grid/RangeMax": "3.5",               # 5 → 3.5: ZED depth 오차는 거리² 로 커짐 (실측: 5 m 까지 쓰니 벽이 30~50 cm 로 번짐)
    "Grid/3D": "false",                   # Nav2 는 2D 격자만 씀 (기본 true 는 3D 점을 노드마다 저장 → DB 의 19 %)
    "Grid/NoiseFilteringRadius": "0.1",   # 주변 10 cm 안에 점이 3 개 미만인 외톨이 점(depth 잡음) 제거
    "Grid/NoiseFilteringMinNeighbors": "3",
    "Mem/ImagePostDecimation": "2",       # DB 에 저장할 때만 영상·depth 를 1/2 (1280x720 → 640x360). 특징점은 원본에서 뽑음 (PreDecimation 1)
                                          # 실측 2026-11-11: 원본 저장이면 4 분에 688 MB (depth 71 %)
    "Mem/SaveDepth16Format": "true",      # depth 를 32 bit float → 16 bit mm 로 저장 (65 m 넘는 값만 잘림)
    # Grid/MaxGroundHeight 는 launch 인자 ground_height 로 (아래)
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
    # 바닥으로 볼 최대 높이. 법선이 수직(평평)이고 이 높이 아래면 바닥, 위면 장애물 (Grid/NormalsSegmentation 기본 true).
    # 0.05 → 0.20: 실측 2026-11-11 ZED 2i NEURAL_LIGHT, 광택 바닥(테라조) — 바닥 점 높이 표준편차 ~10 cm,
    #   1.5~3.5 m 에서 27~36 % 가 5 cm 를 넘어 바닥이 큰 장애물 덩어리로 찍힘. 세워진 면(벽·상자 옆면·다리)은 높이와 상관없이 장애물
    params["Grid/MaxGroundHeight"] = get("ground_height")
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
    remap = [("rgb/image", ZED + "rgb/color/rect/image"), ("depth/image", ZED + "depth/depth_registered"),
             ("rgb/camera_info", ZED + "rgb/color/rect/camera_info")]
    out = []
    if loc and not db.exists():
        out.append(LogInfo(msg=f"[slam] 경고: localization 모드인데 지도 파일이 없음: {db}"))
    if not loc and db.exists() and db.stat().st_size > 0:  # 매핑 모드: 켤 때마다 백업 (새 지도든 이어 쓰기든)
        bk = db.parent / "rtabmap_backup" / f"{db.stem}_{time.strftime('%Y%m%d_%H%M%S')}{db.suffix}"
        bk.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(db, bk)
        out.append(LogInfo(msg=f"[slam] 기존 지도 백업: {bk} ({db.stat().st_size / 1e6:.1f} MB)"))
    out.append(LogInfo(msg=f"[slam] {'localization (지도 고정)' if loc else ('새 지도' if new_map else '기존 지도에 이어서 매핑')}: {db}"))
    if get("odom_source") == "zed":
        # ZED 단독: scripts/zed_base_tf.py 가 ZED odom(카메라 자세)을 받아
        #   odom → base_footprint (x, y, yaw — 항상 수평) + base_footprint → zed_camera_link (cam_* + 카메라 roll·pitch) 를 냄.
        # ZED wrapper 는 publish_tf:=false (zed_camera_link 의 부모가 둘이 되면 안 됨). 카메라 pitch 는 ZED 가 중력으로 앎 → 잴 필요 없음
        cx, cy, cz = (float(get(k)) for k in ("cam_x", "cam_y", "cam_z"))
        out.append(LogInfo(msg=f"[slam] odom = ZED 단독 (EKF 없음). 카메라 위치 (차체 기준) x {cx} y {cy} z {cz} m"))
        out.append(Node(package="capstone_navigation", executable="zed_base_tf.py", name="zed_base_tf", output="log",
                        parameters=[{"use_sim_time": sim, "cam_x": cx, "cam_y": cy, "cam_z": cz,
                                     "pitch_offset_deg": float(get("cam_pitch_offset")),
                                     "roll_offset_deg": float(get("cam_roll_offset"))}]))
    elif get("odom_source") == "external":
        # TF odom→base_footprint 를 다른 곳에서 냄 (시뮬: world.launch odom_tf:=true odom_noise:=false = 참값 odom).
        # 긴 루프 시뮬에서 EKF 가 가짜 자이로 바이어스로 틀어질 때, 위치 추정 말고 다른 것(선반 찾기 등)을 시험하려고 씀
        out.append(LogInfo(msg="[slam] odom = 외부 TF (EKF·zed_base_tf 안 띄움)"))
    else:
        out.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
                        parameters=[str(ekf_cfg), {"use_sim_time": sim}]))
    out += [
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
        DeclareLaunchArgument("odom_source", default_value="ekf", choices=["ekf", "zed", "external"],
                              description="ekf = 바퀴+VIO+자이로 EKF / zed = ZED 단독 (바퀴 odom 없을 때, ZED publish_tf:=false) / external = odom TF 를 다른 곳에서 냄"),
        DeclareLaunchArgument("ground_height", default_value="0.20", description="[m] 평평한 면을 바닥으로 볼 최대 높이 (depth 잡음 흡수)"),
        DeclareLaunchArgument("cam_x", default_value="0.0", description="[odom_source:=zed] 차체 중심→카메라 앞쪽 [m]"),
        DeclareLaunchArgument("cam_y", default_value="0.0", description="[odom_source:=zed] 왼쪽 [m]"),
        DeclareLaunchArgument("cam_pitch_offset", default_value="0.0",
                              description="[odom_source:=zed] ZED 가 추정한 숙임에 더할 보정 [deg] (tools/zed_floor_level.py 권장값)"),
        DeclareLaunchArgument("cam_roll_offset", default_value="0.0", description="[odom_source:=zed] 옆 기울기 보정 [deg]"),
        DeclareLaunchArgument("cam_z", default_value="1.0", description="[odom_source:=zed] 바닥→zed_camera_link 높이 [m] (줄자로)"),
        OpaqueFunction(function=nodes),
    ])
