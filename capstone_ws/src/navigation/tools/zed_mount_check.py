#!/usr/bin/env python3
"""ZED 장착 점검 (로봇 정지, 카메라 앞 2 m 바닥이 비어 있을 때). ZED wrapper 만 떠 있으면 됨.

  python3 src/navigation/tools/zed_mount_check.py --cam-z 0.675 --pitch 23.9

확인:
  1) 숙임 각도: IMU(중력) 기준 카메라 pitch·roll ↔ 기대값 (URDF front_camera_joint)
  2) 높이: 점구름을 중력 기준으로 수평으로 돌려 바닥 평면 높이 → zed_camera_link(삼각대 나사) 높이 ↔ 줄자 값
  3) 바닥 평탄도: 거리별 바닥 점 높이 퍼짐 (클수록 바닥이 장애물로 찍히기 쉬움 → slam.launch ground_height)
  4) 앞쪽 바닥이 처음 보이는 거리 (그보다 가까운 낮은 장애물은 못 봄)
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def rot_from_q(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default="/zed/zed_node")
    ap.add_argument("--cam-z", type=float, required=True, help="줄자: 바닥 → 카메라 삼각대 나사 [m]")
    ap.add_argument("--pitch", type=float, default=23.9, help="기대 숙임 [deg]")
    ap.add_argument("--frames", type=int, default=10)
    a = ap.parse_args()
    rclpy.init()
    n = Node("zed_mount_check")
    st = {"imu": [], "clouds": []}
    n.create_subscription(Imu, f"{a.ns}/imu/data", lambda m: st["imu"].append(m), qos_profile_sensor_data)
    n.create_subscription(PointCloud2, f"{a.ns}/point_cloud/cloud_registered",
                          lambda m: st["clouds"].append(m) if len(st["clouds"]) < a.frames else None, qos_profile_sensor_data)
    tf = Buffer()
    TransformListener(tf, n)
    t0 = time.time()
    while (len(st["clouds"]) < a.frames or len(st["imu"]) < 100) and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not st["clouds"] or not st["imu"]:
        print(f"데이터 없음: cloud {len(st['clouds'])}, imu {len(st['imu'])}")
        return

    # 1) 자세 (IMU orientation 은 중력 기준 roll/pitch)
    q = np.mean([[m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w] for m in st["imu"][-100:]], 0)
    q /= np.linalg.norm(q)
    R_imu = rot_from_q(*q)
    pitch = math.degrees(math.asin(-R_imu[2, 0]))
    roll = math.degrees(math.atan2(R_imu[2, 1], R_imu[2, 2]))
    print(f"== 1) 자세 (IMU): 아래로 숙임 {pitch:+.1f}° (기대 {a.pitch:.1f}°, 차 {pitch - a.pitch:+.1f}°), 옆 기울기 roll {roll:+.1f}°")

    # 점구름 프레임 → zed_imu_link 회전, 점구름 원점 → zed_camera_link(나사) 위치
    cf = st["clouds"][0].header.frame_id
    try:
        T_lc = tf.lookup_transform("zed_camera_link", cf, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)).transform
    except Exception as e:
        print(f"TF 없음 ({e}) — wrapper 의 publish_urdf 가 꺼져 있나?")
        return
    R_lc = rot_from_q(T_lc.rotation.x, T_lc.rotation.y, T_lc.rotation.z, T_lc.rotation.w)
    t_lc = np.array([T_lc.translation.x, T_lc.translation.y, T_lc.translation.z])
    # 나사 기준 점 = R_lc p + t_lc. IMU 축 = zed_camera_link 축으로 봄 (wrapper 시작 로그 Camera-IMU 회전 Q = [0,0,0,1];
    # zed_imu_link 는 TF 에 없을 수 있음) → 중력 수평 = R_imu
    R_level = R_imu
    P = []
    for m in st["clouds"]:
        p = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        p = p[np.isfinite(p).all(1)][::3]
        P.append((p @ R_lc.T + t_lc) @ R_level.T)
    P = np.concatenate(P)
    d = np.hypot(P[:, 0], P[:, 1])

    # 2) 바닥: 줄자 높이 근처(±25 cm)에서 가장 점이 많은 높이
    z = P[:, 2]
    cand = z[np.abs(z + a.cam_z) < 0.25]
    if cand.size < 500:
        print(f"== 2) 바닥을 못 찾음 (줄자 높이 ±25 cm 안 점 {cand.size} 개) — 앞 2 m 바닥이 보이는지 확인")
        return
    h, e = np.histogram(cand, bins=np.arange(-a.cam_z - 0.25, -a.cam_z + 0.25, 0.01))
    i = np.argmax(h)
    fz = np.median(cand[(cand >= e[i] - 0.03) & (cand <= e[i + 1] + 0.03)])
    print(f"== 2) 높이: depth 로 잰 바닥 → 나사 {-fz:.3f} m (줄자 {a.cam_z:.3f} m, 차 {(-fz - a.cam_z) * 100:+.1f} cm)")

    # 3) 바닥 평탄도
    fl = np.abs(z - fz) < 0.25
    print("== 3) 바닥 점 높이 퍼짐 (바닥 ±25 cm 안)")
    for lo, hi in ((0.3, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0)):
        m = fl & (d >= lo) & (d < hi)
        if m.sum() < 50:
            print(f"   {lo:.0f}~{hi:.0f} m: 점 부족 ({m.sum()})")
            continue
        dz = z[m] - fz
        print(f"   {lo:.1f}~{hi:.1f} m: 표준편차 {dz.std() * 100:4.1f} cm, 5 cm 넘음 {np.mean(dz > 0.05) * 100:4.1f}%, "
              f"20 cm 넘음 {np.mean(dz > 0.20) * 100:4.1f}%  (점 {m.sum()})")

    # 4) 바닥이 처음 보이는 거리 (나사 기준 앞쪽)
    fwd = P[fl & (np.abs(P[:, 1]) < 0.3) & (P[:, 0] > 0), 0]
    if fwd.size:
        print(f"== 4) 정면 바닥이 보이기 시작하는 거리: 카메라(나사)에서 앞 {np.percentile(fwd, 1):.2f} m "
              f"(계산값 {a.cam_z / math.tan(math.radians(a.pitch + 35)):.2f} m, 세로 화각 70° 가정)")
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
