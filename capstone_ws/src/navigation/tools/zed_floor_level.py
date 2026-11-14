#!/usr/bin/env python3
"""바닥이 평평하게 보이는지 점검 — slam.launch (odom_source:=zed) 가 떠 있고 로봇이 평평한 바닥에 서 있을 때.

  python3 src/navigation/tools/zed_floor_level.py

costmap 이 쓰는 것과 같은 TF (base_footprint → 카메라) 로 depth 를 로봇 좌표로 옮겨, 로봇 앞 거리별 바닥 높이를 봄.
정상이면 모든 거리에서 0 cm 근처. 멀수록 높아지면(또는 낮아지면) 숙임 각이 틀린 것 → 권장 보정값을 알려 줌:
  켠 채로:   ros2 param set /zed_base_tf pitch_offset_deg <값>
  다음부터:  slam.launch.py ... cam_pitch_offset:=<값>
왜 필요한가: 장애물 기준이 "바닥에서 8 cm" 라서, 2.5 m 앞 바닥이 5 cm 떠 보이면 잡음 여유가 3 cm 밖에 안 남아 가짜 장애물이 찍힘.
앞 바닥이 2 m 이상 비어 있는 곳에서 재야 정확 (가구·벽이 많으면 점이 부족).
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

Z = "/zed/zed_node/"


def main():
    rclpy.init()
    n = Node("zed_floor_level")
    st = {"d": []}
    tf = Buffer()
    TransformListener(tf, n)
    n.create_subscription(Image, Z + "depth/depth_registered",
                          lambda m: st.__setitem__("d", (st["d"] + [m])[-5:]), qos_profile_sensor_data)
    n.create_subscription(CameraInfo, Z + "depth/camera_info", lambda m: st.__setitem__("k", m), qos_profile_sensor_data)
    t0, T = time.time(), None
    while time.time() - t0 < 15 and T is None:
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(st["d"]) >= 5 and "k" in st:
            try:
                T = tf.lookup_transform("base_footprint", st["d"][-1].header.frame_id, rclpy.time.Time()).transform
            except Exception:
                pass
    if T is None:
        print(f"데이터 없음 (depth {len(st['d'])} 장, camera_info {'k' in st}, TF base_footprint→카메라) — ZED·slam.launch 가 떠 있나?")
        return 1
    x, y, z, w = T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    k = st["k"].k
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    pts = []
    for m in st["d"]:
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        v, u = np.mgrid[0:m.height:3, 0:m.width:3]
        zz = d[::3, ::3]
        ok = np.isfinite(zz) & (zz > 0.3) & (zz < 5)
        P = np.c_[(u[ok] - cx) * zz[ok] / fx, (v[ok] - cy) * zz[ok] / fy, zz[ok]]   # 광학 좌표 (x 오른쪽, y 아래, z 앞)
        pts.append(P @ R.T + [T.translation.x, T.translation.y, T.translation.z])
    B = np.vstack(pts)
    F = B[(np.abs(B[:, 1]) < 1.0) & (np.abs(B[:, 2]) < 0.20) & (B[:, 0] > 0.6) & (B[:, 0] < 3.5)]   # 바닥 후보
    print("로봇 앞 거리   바닥 높이(중앙값)   8 cm 넘는 비율")
    for a, b in [(0.6, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 3.5)]:
        s = F[(F[:, 0] >= a) & (F[:, 0] < b)]
        if len(s) < 200:
            print(f"  {a:.1f}~{b:.1f} m     점 부족 ({len(s)})")
            continue
        print(f"  {a:.1f}~{b:.1f} m       {np.median(s[:, 2]) * 100:+5.1f} cm          {np.mean(s[:, 2] > 0.08) * 100:5.1f}%")
    near = F[F[:, 0] < 2.5]
    if len(near) < 2000 or near[:, 0].max() - near[:, 0].min() < 1.0:
        print("바닥 점이 부족해서 기울기를 못 구함 — 앞 바닥이 2 m 이상 비어 있는 곳에서 다시")
        return 1
    A = np.c_[near[:, 0], near[:, 1], np.ones(len(near))]
    for _ in range(3):                                  # 튀는 점 빼고 다시 맞춤
        co = np.linalg.lstsq(A, near[:, 2], rcond=None)[0]
        keep = np.abs(near[:, 2] - A @ co) < 0.04
        A, near = A[keep], near[keep]
    dp, dr = math.degrees(math.atan(co[0])), -math.degrees(math.atan(co[1]))
    print(f"바닥 기울기 (0.6~2.5 m): 앞으로 1 m 당 {co[0] * 100:+.2f} cm, 왼쪽으로 1 m 당 {co[1] * 100:+.2f} cm, 로봇 밑에서 {co[2] * 100:+.1f} cm")
    if abs(dp) < 0.3 and abs(dr) < 0.5:
        print("→ 평평함 (보정 필요 없음)")
    else:
        print(f"→ 지금 값에 더할 보정: pitch_offset_deg {dp:+.1f}, roll_offset_deg {dr:+.1f}")
        print("   (이미 보정을 넣은 상태면 지금 값 + 위 값. 확인: ros2 param get /zed_base_tf pitch_offset_deg)")
    if abs(co[2]) > 0.02:
        print(f"   로봇 밑 높이가 {co[2] * 100:+.1f} cm → cam_z 를 {-co[2] * 100:+.1f} cm 고치면 0 이 됨 (줄자 값을 먼저 다시 확인)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
