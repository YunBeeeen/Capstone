#!/usr/bin/env python3
"""가짜 ZED VIO — Gazebo 참값에 VIO 수준 드리프트를 섞어 실기 ZED wrapper 와 같은 토픽으로 냄.

  sub  sim/ground_truth       nav_msgs/Odometry (참값, world 기준)
  pub  /zed/zed_node/odom     nav_msgs/Odometry (frame odom → base_footprint)  ← 실기 ZED SDK GEN_3 자리
  TF 는 안 냄 (실기도 publish_tf:=false — TF 는 EKF 가)

ZED SDK 위치 추정은 Gazebo 에서 못 돌리므로 오차 모양만 흉내:
  - 평소: 이동량에 비례하는 작은 잡음 + 스케일 오차 (Stereolabs 공개치 GEN_3 실내 ~400 m 평균 APE 0.56 m 정도)
  - 열화 구간(degraded_zones): 특징점 적은 곳 — GEN_3 문서: "특징점이 아주 적으면 덜 강건".
    잡음·바이어스 degraded_gain 배 + 공분산도 크게 → EKF 가 VIO 를 덜 믿는지 시험.
    기본 구간 = school_floor 에서 RTAB-Map 시각 odometry 가 끊긴 곳 (남쪽 복도 사물함 앞, 동쪽 흰 벽 복도).
  vio:=false (world.launch.py) 면 안 띄움.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FakeVio(Node):
    def __init__(self):
        super().__init__("fake_vio")
        p = lambda k, v: self.declare_parameter(k, v).value
        self.scale = p("scale", 0.003)            # 이동 스케일 오차
        self.sig_rel = p("sigma_rel", 0.004)      # 이동 1 m 당 위치 잡음 [m]
        self.sig_yaw = p("sigma_yaw_rel", 0.003)  # 회전 1 rad 당 yaw 잡음 [rad]
        self.yaw_bias = p("yaw_bias", 0.0002)     # [rad/s] 움직일 때만
        self.gain = p("degraded_gain", 10.0)
        # [x0, x1, y0, y1, ...] world 좌표 (school_floor). 비우면 열화 없음
        z = p("degraded_zones", [13.5, 20.0, 0.0, 2.5, 17.5, 20.0, 2.5, 12.0])
        self.zones = [z[i:i + 4] for i in range(0, len(z) - 3, 4)]
        self.cov_ok, self.cov_bad = p("cov_ok", 1e-4), p("cov_bad", 0.05)
        self.rng = np.random.default_rng(p("seed", 2))
        self.prev = None  # (t, x, y, yaw) 참값
        self.x = self.y = self.yaw = 0.0
        self.pub = self.create_publisher(Odometry, "/zed/zed_node/odom", 20)
        self.create_subscription(Odometry, "sim/ground_truth", self.on_gt, 50)
        self.in_zone = False
        self.get_logger().info(f"가짜 VIO: 스케일 {self.scale:+.1%}, 잡음 {self.sig_rel} m/m, 열화 구간 {len(self.zones)} 개 ×{self.gain}")

    def degraded(self, x, y):
        return any(a <= x <= b and c <= y <= d for a, b, c, d in self.zones)

    def on_gt(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        g = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation))
        if self.prev is None or t <= self.prev[0]:
            self.prev = (t, *g)
            return
        t0, x0, y0, w0 = self.prev
        self.prev = (t, *g)
        dt = t - t0
        c, s = math.cos(w0), math.sin(w0)  # 참값 이동을 차체 기준으로
        bx, by = c * (g[0] - x0) + s * (g[1] - y0), -s * (g[0] - x0) + c * (g[1] - y0)
        dw = wrap(g[2] - w0)
        d = math.hypot(bx, by)
        bad = self.degraded(g[0], g[1])
        if bad != self.in_zone:
            self.in_zone = bad
            self.get_logger().info(f"{'열화 구간 진입' if bad else '열화 구간 벗어남'} ({g[0]:.1f}, {g[1]:.1f})")
        k = self.gain if bad else 1.0
        moving = d > 1e-4 or abs(dw) > 1e-4
        if moving:
            bx = bx * (1 + self.scale) + self.rng.normal(0, k * self.sig_rel * d + 1e-5)
            by = by * (1 + self.scale) + self.rng.normal(0, k * self.sig_rel * d + 1e-5)
            dw = dw + self.rng.normal(0, k * self.sig_yaw * abs(dw) + 1e-5) + k * self.yaw_bias * dt
        h = self.yaw + 0.5 * dw
        self.x += math.cos(h) * bx - math.sin(h) * by
        self.y += math.sin(h) * bx + math.cos(h) * by
        self.yaw = wrap(self.yaw + dw)

        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id, o.child_frame_id = "odom", "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        cv = self.cov_bad if bad else self.cov_ok
        for i in (0, 7, 35):
            o.pose.covariance[i] = cv
        for i in (14, 21, 28):
            o.pose.covariance[i] = 1e6
        if dt > 0:
            o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = bx / dt, by / dt, dw / dt
        self.pub.publish(o)


def main():
    rclpy.init()
    n = FakeVio()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / launch 종료
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
