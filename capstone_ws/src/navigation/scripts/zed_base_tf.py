#!/usr/bin/env python3
"""ZED odom(카메라 자세) → 수평 차체 프레임 TF.  slam.launch.py odom_source:=zed 에서 씀 (ZED wrapper 는 publish_tf:=false).

  odom ─(x, y, yaw 만)─▶ base_footprint ─(cam_x, cam_y, cam_z + 카메라 roll·pitch)─▶ zed_camera_link

왜: 카메라 아래에 고정 TF 로 base_footprint 를 달면 카메라가 숙이거나 흔들릴 때 차체 프레임도 같이 기울어서
    먼 바닥이 장애물로 찍힘 (실측 2026-11-11 손에 들고 매핑: 장애물 비율 40 %, 큰 덩어리).
    여기서는 카메라 자세를 yaw 와 roll·pitch 로 나눠서 base_footprint 는 항상 수평·바닥 높이, 기울기는 그 아래 링크에 넣음.
    → 카메라 장착 pitch 를 따로 잴 필요 없음 (ZED 가 중력으로 앎. zed_override 의 set_gravity_as_origin: true 필요)
    카메라의 높이 변화(z)는 버림 — 차체 기준 높이는 cam_z 고정 (손에 들 때는 대략, 로봇은 줄자로).
입력: /zed/zed_node/pose (PoseStamped, 중력 기준 map 프레임). /zed/zed_node/odom 은 쓰지 않음 —
  실측 2026-11-11: ZED odom 프레임은 켤 때의 카메라 방향 기준(21.6° 숙임 그대로)이라 평지 2.55 m 직진이 높이 +1.0 m 로 찍힘.
  pose 는 set_gravity_as_origin 으로 중력 정렬됨 (pitch 21.9° 로 IMU 와 일치). area_memory false 라 루프 클로저 점프 없음.
  공분산은 odom 메시지의 것을 가져다 씀.
출력: TF odom→base_footprint, base_footprint→zed_camera_link (둘 다 ZED pose 스탬프),
      odom_topic (기본 ~/odom = /zed_base_tf/odom): base_footprint 기준 Odometry + 속도(pose 차분) — Nav2 odom_topic
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def q_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def rpy_from_q(x, y, z, w):  # ZYX (R = Rz(yaw) Ry(pitch) Rx(roll))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class ZedBaseTf(Node):
    def __init__(self):
        super().__init__("zed_base_tf")
        self.cx = self.declare_parameter("cam_x", 0.0).value   # 차체 중심 → 카메라 (앞 +) [m]
        self.cy = self.declare_parameter("cam_y", 0.0).value   # (왼 +)
        self.cz = self.declare_parameter("cam_z", 1.0).value   # 바닥 → zed_camera_link 높이
        # ZED 가 추정한 숙임·옆 기울기에 더할 보정 [deg]. 실측 2026-11-12: ZED pose 의 숙임 20.9° 로는 바닥이 1 m 당 1.9 cm 씩
        #  떠 보임 (2.5 m 에서 +5 cm → 8 cm 기준에 가까워져 가짜 장애물). tools/zed_floor_level.py 가 권장값을 알려 줌.
        #  매 콜백마다 읽으므로 ros2 param set /zed_base_tf pitch_offset_deg 1.1 로 켠 채 바꿀 수 있음
        self.declare_parameter("pitch_offset_deg", 0.0)   # + = 더 숙임
        self.declare_parameter("roll_offset_deg", 0.0)
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.cam_frame = self.declare_parameter("camera_frame", "zed_camera_link").value
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, self.declare_parameter("odom_topic", "~/odom").value, 10)
        # 속도: 연속 pose 의 차분 → 차체 기준 (vx, vy, ω), 지수 평균으로 다듬음 (Nav2 controller·velocity_smoother 가 씀)
        self.alpha = self.declare_parameter("twist_alpha", 0.3).value  # 새 값 비중 (30 Hz 에서 ~0.1 s 시상수)
        self.prev = None
        self.vel = [0.0, 0.0, 0.0]
        self.cov = [0.0] * 36
        self.create_subscription(Odometry, self.declare_parameter("zed_odom", "/zed/zed_node/odom").value,
                                 lambda m: setattr(self, "cov", m.pose.covariance), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.declare_parameter("zed_pose", "/zed/zed_node/pose").value,
                                 self.cb, qos_profile_sensor_data)

    def cb(self, m):
        p, q = m.pose.position, m.pose.orientation
        roll, pitch, yaw = rpy_from_q(q.x, q.y, q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = p.x - (c * self.cx - s * self.cy), p.y - (s * self.cx + c * self.cy)
        roll += math.radians(self.get_parameter("roll_offset_deg").value)
        pitch += math.radians(self.get_parameter("pitch_offset_deg").value)
        qb, qc = q_from_rpy(0.0, 0.0, yaw), q_from_rpy(roll, pitch, 0.0)

        t1 = TransformStamped()
        t1.header.stamp, t1.header.frame_id, t1.child_frame_id = m.header.stamp, self.odom_frame, self.base_frame
        t1.transform.translation.x, t1.transform.translation.y = bx, by
        (t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w) = qb
        t2 = TransformStamped()
        t2.header.stamp, t2.header.frame_id, t2.child_frame_id = m.header.stamp, self.base_frame, self.cam_frame
        t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z = self.cx, self.cy, self.cz
        (t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w) = qc
        self.tf.sendTransform([t1, t2])

        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = m.header.stamp, self.odom_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = bx, by
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w) = qb
        o.pose.covariance = self.cov  # ZED odom 의 공분산 (카메라 기준, 근사)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.prev is not None:
            dt = t - self.prev[0]
            if 0.005 < dt < 0.5:
                dx, dy = bx - self.prev[1], by - self.prev[2]
                dyaw = math.atan2(math.sin(yaw - self.prev[3]), math.cos(yaw - self.prev[3]))
                ym = self.prev[3] + dyaw / 2  # 구간 중간 방향으로 차체 기준 변환
                raw = (( math.cos(ym) * dx + math.sin(ym) * dy) / dt,
                       (-math.sin(ym) * dx + math.cos(ym) * dy) / dt, dyaw / dt)
                self.vel = [v + self.alpha * (r - v) for v, r in zip(self.vel, raw)]
            elif dt >= 0.5:
                self.vel = [0.0, 0.0, 0.0]  # 끊겼다 다시 옴
        self.prev = (t, bx, by, yaw)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = self.vel
        o.twist.covariance[0] = o.twist.covariance[7] = 0.01 ** 2
        o.twist.covariance[35] = 0.02 ** 2
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(ZedBaseTf())


if __name__ == "__main__":
    main()
