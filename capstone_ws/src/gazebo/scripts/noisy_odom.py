#!/usr/bin/env python3
"""가짜 바퀴 오도메트리 — planar_move 의 완벽한 odom 에 실기 같은 오차를 넣어 /odom 으로 냄.

  sub  sim/planar_odom   nav_msgs/Odometry (planar_move, 잡음 없음)
  pub  odom              nav_msgs/Odometry (frame odom → base_frame)  ← 실기 swerve_odom 과 같은 토픽
  tf   odom → base_frame (publish_tf=true)

오차 모델 (차체 속도에 곱하고 더한 뒤 적분 — 그래서 시간이 갈수록 쌓임):
  vx·(1+scale_x) + N(0, sigma_v),  vy·(1+scale_y) + N(0, sigma_v),  ω·(1+scale_yaw) + N(0, sigma_w) + yaw_bias
기본값은 MuJoCo 스워브 시험(2026-11-11: 직선 0.1~0.5 %, 회전 1.6 %, 조향 전환 때 2~3°)보다 조금 나쁘게
— 실기 바닥·타이어는 시뮬보다 나쁨. noise:=false 면 오차 0 (완벽한 odom 그대로).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class NoisyOdom(Node):
    def __init__(self):
        super().__init__("noisy_odom")
        p = lambda k, v: self.declare_parameter(k, v).value
        self.noise = p("noise", True)
        self.sx, self.sy, self.sw = p("scale_x", 0.015), p("scale_y", -0.01), p("scale_yaw", 0.02)
        self.sig_v, self.sig_w = p("sigma_v", 0.01), p("sigma_w", 0.01)
        self.bias_w = p("yaw_bias", 0.0005)  # [rad/s] 움직일 때만 (200 s 주행 ≈ 6°)
        self.odom_frame, self.base_frame = p("odom_frame", "odom"), p("base_frame", "base_footprint")
        self.publish_tf = p("publish_tf", True)
        self.rng = np.random.default_rng(p("seed", 1))
        self.x = self.y = self.yaw = 0.0
        self.t = None
        self.pub = self.create_publisher(Odometry, "odom", 20)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, "sim/planar_odom", self.on_odom, 50)
        self.get_logger().info(
            f"/odom ({self.odom_frame}→{self.base_frame}), 오차 {'켬' if self.noise else '끔'}: "
            f"scale x {self.sx:+.1%} y {self.sy:+.1%} yaw {self.sw:+.1%}, σv {self.sig_v} σω {self.sig_w}, "
            f"yaw bias {self.bias_w} rad/s, TF {'발행' if self.publish_tf else '안 함'}")

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t is None or t <= self.t:
            self.t = t
            return
        dt = min(t - self.t, 0.2)
        self.t = t
        v = m.twist.twist  # planar_move: 차체 기준
        vx, vy, w = v.linear.x, v.linear.y, v.angular.z
        if self.noise:
            moving = abs(vx) + abs(vy) + abs(w) > 1e-3  # 멈춰 있을 땐 잡음 안 넣음 (실기 엔코더도 안 굴면 0)
            vx = vx * (1 + self.sx) + (self.rng.normal(0, self.sig_v) if moving else 0)
            vy = vy * (1 + self.sy) + (self.rng.normal(0, self.sig_v) if moving else 0)
            w = w * (1 + self.sw) + ((self.rng.normal(0, self.sig_w) + self.bias_w) if moving else 0)
        h = self.yaw + 0.5 * w * dt
        self.x += (vx * math.cos(h) - vy * math.sin(h)) * dt
        self.y += (vx * math.sin(h) + vy * math.cos(h)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))

        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id, o.child_frame_id = self.odom_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, w
        for i, c in zip((0, 7, 35), (1e-3, 1e-3, 1e-2)):
            o.pose.covariance[i] = o.twist.covariance[i] = c
        for i in (14, 21, 28):
            o.pose.covariance[i] = o.twist.covariance[i] = 1e6
        self.pub.publish(o)
        if self.tf:
            tr = TransformStamped()
            tr.header = o.header
            tr.child_frame_id = self.base_frame
            tr.transform.translation.x, tr.transform.translation.y = self.x, self.y
            tr.transform.rotation = o.pose.pose.orientation
            self.tf.sendTransform(tr)


def main():
    rclpy.init()
    n = NoisyOdom()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / launch 종료
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
