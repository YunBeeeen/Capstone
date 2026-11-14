#!/usr/bin/env python3
"""바퀴 오도메트리가 얼마나 맞는지 줄자로 확인 — ZED VIO 와 나란히 비교.

  python3 odom_check.py line          # 직진: 5 m 쯤 똑바로 가서 줄자로 잰 거리와 비교
  python3 odom_check.py spin          # 제자리 회전: 한 바퀴 돌아 처음 방향에 다시 맞추고 360° 와 비교
  python3 odom_check.py line --true 5.02    # 잰 값을 미리 줌 (안 주면 끝에 물어봄)
  python3 odom_check.py spin --true -720    # 시계 방향 두 바퀴

준비
  라파:   ros2 launch swerve_controller swerve_odom.launch.py publish_tf:=false
          (publish_tf:=false 꼭 — TF 는 ZED 쪽이 내고 있음. 구동·조향 브리지가 떠 있어야 함)
  노트북: ZED + slam.launch.py odom_source:=zed (매핑이든 재위치든). 없어도 바퀴만 잼.
방법
  line: 바닥에 테이프로 시작 선 → 바퀴(또는 차체 앞 모서리)를 선에 맞춤 → Enter → 패드(RB)로 똑바로 → 멈춤 → Enter
        → 같은 기준점까지 줄자로 잰 거리 입력. 왕복하지 말고 한 방향으로만.
  spin: 차체 모서리를 바닥 테이프에 맞춤 → Enter → 제자리에서 한 바퀴 → 처음 방향에 다시 맞춰 멈춤 → Enter.
        왼쪽(반시계)으로 돌면 +360, 오른쪽이면 -360.
판정 기준 (EKF 에 넣어 볼 만한가): 직진 거리 오차 2 % 이내, 한 바퀴에 5° 이내.
  거리 오차가 일정한 비율이면 바퀴 반지름 보정값을 알려 줌 (swerve_controller.yaml 의 wheel_radius 에 곱함).
"""
import argparse
import math
import sys
import threading
import time

import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    """odom 하나를 따라가며 시작점 기준 변위·누적 회전·지나간 길이를 셈."""

    STEP = 0.005     # [m] 이만큼 움직여야 길이에 더함 (서 있을 때 잡음이 쌓이지 않게)

    def __init__(self, name):
        self.name, self.n, self.t_last = name, 0, 0.0
        self.p = self.p0 = self.anchor = None
        self.turn = self.path = 0.0

    def cb(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        if self.p is not None and self.p0 is not None:
            self.turn += wrap(p[2] - self.p[2])
            d = math.hypot(p[0] - self.anchor[0], p[1] - self.anchor[1])
            if d > self.STEP:
                self.path += d
                self.anchor = p
        self.p, self.n, self.t_last = p, self.n + 1, time.time()

    def start(self):
        self.p0 = self.anchor = self.p
        self.turn = self.path = 0.0

    def alive(self):
        return self.p is not None and time.time() - self.t_last < 1.0

    def result(self):
        """(직선 거리, 앞쪽, 왼쪽, 누적 회전°, 지나간 길이) — 앞/왼쪽은 시작 때 차체 방향 기준."""
        dx, dy = self.p[0] - self.p0[0], self.p[1] - self.p0[1]
        c, s = math.cos(self.p0[2]), math.sin(self.p0[2])
        return math.hypot(dx, dy), c * dx + s * dy, -s * dx + c * dy, math.degrees(self.turn), self.path


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["line", "spin"])
    ap.add_argument("--true", type=float, dest="true", help="줄자로 잰 거리 [m] (line) / 실제로 돈 각도 [deg] (spin, 기본 360)")
    ap.add_argument("--wheel", default="/odom", help="바퀴 오도메트리 토픽")
    ap.add_argument("--vio", default="/zed_base_tf/odom", help="ZED VIO 토픽 (차체 기준)")
    a = ap.parse_args()

    rclpy.init()
    n = Node("odom_check")
    tr = {"바퀴": Track("바퀴"), "VIO": Track("VIO")}
    n.create_subscription(Odometry, a.wheel, tr["바퀴"].cb, qos_profile_sensor_data)
    n.create_subscription(Odometry, a.vio, tr["VIO"].cb, qos_profile_sensor_data)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    th_spin = threading.Thread(target=ex.spin, daemon=True)
    th_spin.start()

    def done(code):
        ex.shutdown()
        th_spin.join(timeout=2)
        rclpy.try_shutdown()
        return code

    t0 = time.time()
    while time.time() - t0 < 5 and not all(t.alive() for t in tr.values()):
        time.sleep(0.1)
    for k, t in tr.items():
        print(f"  {k:3s} {a.wheel if k == '바퀴' else a.vio}: {'들어옴' if t.alive() else '안 들어옴'}")
    if not tr["바퀴"].alive():
        print("바퀴 오도메트리가 없음 — 라파에서 swerve_odom.launch.py publish_tf:=false 를 켰나? (구동·조향 브리지도)")
        return done(1)
    use = [t for t in tr.values() if t.alive()]

    what = "똑바로 5 m 쯤 가서 멈춤" if a.mode == "line" else "제자리에서 한 바퀴 돌아 처음 방향에 다시 맞춰 멈춤"
    ask(f"\n시작 자리에 세우고 Enter ▶ ")
    for t in use:
        t.start()
    print(f"  재는 중 — {what}. (움직이는 동안 값이 아래에 찍힘)")
    stop = threading.Event()

    def show():
        while not stop.wait(1.0):
            s = "  ".join(f"{t.name} {t.result()[0]:5.2f} m {t.result()[3]:+7.1f}°" for t in use)
            print(f"\r    {s}   멈춘 뒤 Enter ▶ ", end="", flush=True)
    th = threading.Thread(target=show, daemon=True)
    th.start()
    ask("")
    stop.set()
    time.sleep(0.3)        # 마지막 메시지까지
    dead = [t.name for t in use if not t.alive()]
    if dead:
        print(f"  경고: 재는 동안 {', '.join(dead)} 가 끊김 — 값이 틀릴 수 있음")
    res = {t.name: t.result() for t in use}

    true = a.true
    if true is None:
        s = ask("줄자로 잰 거리 [m] (모르면 Enter) ▶ " if a.mode == "line" else "실제로 돈 각도 [deg] (Enter = 360, 오른쪽으로 돌았으면 -360) ▶ ")
        try:
            true = float(s) if s else (None if a.mode == "line" else 360.0)
        except ValueError:
            print(f"  숫자가 아님: {s!r} — 참값 없이 표시")
            true = None

    print()
    ok = None
    if a.mode == "line":
        print(f"  {'':4s} {'직선 거리':>9s} {'옆으로':>8s} {'틀어짐':>8s}" + (f" {'거리 오차':>12s}" if true else ""))
        for k, (dist, fwd, left, turn, path) in res.items():
            e = f" {dist - true:+7.3f} m {100 * (dist - true) / true:+5.1f} %" if true else ""
            print(f"  {k:4s} {dist:8.3f} m {left:+7.3f} m {turn:+7.1f}°{e}")
        if true:
            dist, _, _, turn, path = res["바퀴"]
            pct = 100 * (dist - true) / true
            ok = abs(pct) <= 2.0
            if true < 2.0:
                print("  (2 m 보다 짧으면 줄자 오차가 커서 % 가 부정확 — 5 m 쯤으로)")
            if path > 1.15 * dist:
                print(f"  (지나간 길이 {path:.2f} m 가 직선 거리보다 많이 김 — 왔다 갔다 했으면 다시)")
            if not ok and abs(pct) < 15:
                print(f"  바퀴 반지름 보정: wheel_radius 에 {true / dist:.4f} 를 곱함 (두세 번 재서 같은 비율이 나올 때만)")
        if "VIO" in res:
            d = res["바퀴"][0] - res["VIO"][0]
            print(f"  바퀴 − VIO: 거리 {d:+.3f} m, 틀어짐 {res['바퀴'][3] - res['VIO'][3]:+.1f}°")
    else:
        print(f"  {'':4s} {'돈 각도':>9s} {'자리 밀림':>9s}" + (f" {'각도 오차':>10s}" if true else ""))
        for k, (dist, fwd, left, turn, path) in res.items():
            e = f" {turn - true:+8.1f}°" if true else ""
            print(f"  {k:4s} {turn:+8.1f}° {dist:8.3f} m{e}")
        if true:
            turn = res["바퀴"][3]
            per = (turn - true) / (abs(true) / 360.0)
            ok = abs(per) <= 5.0
            print(f"  바퀴: 한 바퀴에 {per:+.1f}°")
            if turn * true < 0:
                print("  (방향 부호가 반대 — 왼쪽으로 돌았으면 +, 오른쪽이면 −)")
                ok = None
        if "VIO" in res:
            print(f"  바퀴 − VIO: {res['바퀴'][3] - res['VIO'][3]:+.1f}°")
    if ok is not None:
        lim = "거리 2 %" if a.mode == "line" else "한 바퀴 5°"
        print(f"\n  판정: {'기준 안' if ok else '기준 밖'} ({lim}) — " +
              ("EKF 에 넣어 볼 만함" if ok else "보정하거나 원인(미끄러짐·조향 영점·반지름)을 먼저 봄"))
    return done(0)


if __name__ == "__main__":
    sys.exit(main())
