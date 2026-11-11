#!/usr/bin/env python3
"""ZED odom 움직임 점검 — 줄자로 잰 거리·제자리 회전과 ZED 가 낸 값을 비교.

  python3 src/navigation/tools/zed_motion_check.py [--topic /zed/zed_node/pose]
  Enter = 기준점 다시 잡기 (매 시험 시작 전), Ctrl+C = 끝

시험 (카메라를 실제 높이·각도로 로봇/카트에 달고):
  1) 직진: 줄자로 3 m 표시 → Enter → 3 m 이동 후 멈춤 → '직선 거리' 가 3.00 m 에 얼마나 가까운지
  2) 왕복: 3 m 갔다 출발점 표시로 돌아옴 → '직선 거리' 가 0 에 가까운지 (돌아온 오차 = 드리프트)
  3) 제자리 회전: Enter → 천천히 한 바퀴 (바닥 표시에 맞춰) → 'yaw' 가 0 에 가까운지, '위치' 가 거의 안 변했는지
     (카메라가 회전 중심에서 떨어져 있으면 카메라 자체는 원을 그림 — 한 바퀴 뒤엔 제자리여야 함)
  4) 흰 벽·유리 앞을 지나가기: 위치가 튀는지, 경로 길이가 실제보다 크게 늘어나는지
"""
import argparse
import math
import select
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class M(Node):
    def __init__(self, topic):
        super().__init__("zed_motion_check")
        self.p = None
        self.ref = None
        self.path = 0.0
        self.turn = 0.0
        self.last = None
        self.n = 0
        self.create_subscription(PoseStamped, topic, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        q = m.pose
        self.p = (q.position.x, q.position.y, q.position.z, yaw_q(q.orientation))
        self.n += 1
        if self.ref is None:
            self.reset()
        if self.last is not None:
            self.path += math.dist(self.p[:3], self.last[:3])
            self.turn += wrap(self.p[3] - self.last[3])
        self.last = self.p

    def reset(self):
        self.ref, self.path, self.turn, self.last = self.p, 0.0, 0.0, self.p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/zed/zed_node/pose",
                    help="중력 기준 pose (odom 은 켤 때 카메라 방향 기준이라 숙인 카메라면 수평 이동이 높이로 섞임)")
    a = ap.parse_args()
    rclpy.init()
    n = M(a.topic)
    print(f"{a.topic} 기다리는 중... (Enter = 기준점 다시 잡기)")
    t_print = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                if n.p:
                    n.reset()
                    print("\n-- 기준점 다시 잡음")
            if n.p and time.monotonic() - t_print > 0.5:
                t_print = time.monotonic()
                r, p = n.ref, n.p
                c, s = math.cos(r[3]), math.sin(r[3])
                dx, dy = p[0] - r[0], p[1] - r[1]
                fwd, left = c * dx + s * dy, -s * dx + c * dy
                print(f"\r직선 거리 {math.hypot(dx, dy):6.3f} m (앞 {fwd:+.3f}, 왼 {left:+.3f}, 높이 {p[2] - r[2]:+.3f})  "
                      f"경로 {n.path:6.2f} m  yaw {math.degrees(wrap(p[3] - r[3])):+7.2f}° (누적 {math.degrees(n.turn):+7.1f}°)  "
                      f"[{n.n} msg]   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    print()
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
