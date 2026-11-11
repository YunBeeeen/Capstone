#!/usr/bin/env python3
"""정면 장애물 확인 — 로봇 앞 통로(폭 ±0.3 m) 안에서 바닥 위 5 cm ~ 1.5 m 높이 점이 있는 가장 가까운 거리.
ZED wrapper 만 떠 있으면 됨. 카메라 자세는 IMU(중력), 바닥 높이는 --cam-z.

  python3 src/navigation/tools/zed_front_obstacle.py --cam-z 0.675 --cam-x 0.167   # 1 초마다 출력, Ctrl+C 끝
  python3 src/navigation/tools/zed_front_obstacle.py --cam-z 0.675 --once           # 한 번만
거리는 로봇 중심 기준 (cam-x 를 더함). 바닥이 보이기 시작하는 거리보다 가까운 낮은 물체는 안 보이는 게 정상.
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from zed_mount_check import rot_from_q


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default="/zed/zed_node")
    ap.add_argument("--cam-z", type=float, required=True)
    ap.add_argument("--cam-x", type=float, default=0.0)
    ap.add_argument("--half-width", type=float, default=0.3)
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Node("zed_front_obstacle")
    st = {"q": None, "cloud": None}
    n.create_subscription(Imu, f"{a.ns}/imu/data", lambda m: st.__setitem__("q", m.orientation), qos_profile_sensor_data)
    n.create_subscription(PointCloud2, f"{a.ns}/point_cloud/cloud_registered", lambda m: st.__setitem__("cloud", m),
                          qos_profile_sensor_data)
    tf = Buffer()
    TransformListener(tf, n)
    T = None
    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            if st["cloud"] is None or st["q"] is None or time.time() - last < 1.0:
                continue
            m = st["cloud"]
            if T is None:
                try:
                    tr = tf.lookup_transform("zed_camera_link", m.header.frame_id, rclpy.time.Time()).transform
                    T = (rot_from_q(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w),
                         np.array([tr.translation.x, tr.translation.y, tr.translation.z]))
                except Exception:
                    continue
            last = time.time()
            q = st["q"]
            R = rot_from_q(q.x, q.y, q.z, q.w)
            p = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
            p = p[np.isfinite(p).all(1)]
            p = (p @ T[0].T + T[1]) @ R.T
            p[:, 2] += a.cam_z  # 바닥 = 0
            sel = (np.abs(p[:, 1]) < a.half_width) & (p[:, 0] > 0) & (p[:, 2] > 0.05) & (p[:, 2] < 1.5)
            floor = (np.abs(p[:, 1]) < a.half_width) & (p[:, 0] > 0) & (np.abs(p[:, 2]) < 0.03)
            fx = np.percentile(p[floor, 0], 1) + a.cam_x if floor.sum() > 50 else float("nan")
            if sel.sum() < 30:
                print(f"정면 장애물 없음 (통로 ±{a.half_width} m)   바닥 보이기 시작 {fx:.2f} m", flush=True)
            else:
                ob = p[sel]
                near = np.percentile(ob[:, 0], 2)
                close = ob[ob[:, 0] < near + 0.1]
                print(f"가장 가까운 장애물: 로봇 중심에서 {near + a.cam_x:.2f} m (카메라에서 {near:.2f} m), "
                      f"높이 {close[:, 2].min():.2f}~{close[:, 2].max():.2f} m, 옆 {np.median(close[:, 1]):+.2f} m, 점 {sel.sum()}"
                      f"   | 바닥 보이기 시작 {fx:.2f} m", flush=True)
            if a.once:
                break
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
