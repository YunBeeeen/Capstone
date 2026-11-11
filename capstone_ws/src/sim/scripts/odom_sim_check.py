#!/usr/bin/env python3
"""바퀴 오도메트리 자동 검증 — 정해진 cmd_vel 시나리오를 시뮬에 넣고 /odom 을 참값과 비교, 합격/불합격.

  ros2 launch capstone_sim sim.launch.py headless:=true     # 시뮬 + swerve_controller + swerve_odom
  ros2 run capstone_sim odom_sim_check.py                   # 다른 터미널. 약 90 s, 끝나면 표 + 종료코드 (0 = 전부 합격)
  ros2 run capstone_sim odom_sim_check.py --plot /tmp/odom_check.png
  ros2 run capstone_sim odom_sim_check.py --only 직진 회전   # 이름에 이 글자가 든 시나리오만

시나리오마다 두 단계로 나눠 봄:
  ① 조향 전환 — 명령 직후 ~ 조향이 멈추고(각속도 ≈ 0) 바퀴가 구를 때까지.
     정렬 게이트는 목표 ±10° 안이면 구동을 풀어서, 조향이 마지막 몇 도를 도는 동안 가속이 겹침
     → 조향 모터 4 개의 반작용 + 바퀴 미끄러짐으로 차체가 조금 돎. 바퀴가 그만큼 구르지 않으니
     **바퀴 odom 이 원리상 못 보는 움직임**. 판정에 넣지 않고 "① yaw 오차" 로 따로 보고
     (이 값이 EKF 에 IMU 자이로를 넣는 이유).
  ② 구름 — 조향이 멈춘 채 바퀴가 구르기 시작한 뒤 ~ 정지(settle) 끝. 이 구간의 상대 운동끼리 비교해서 합격/불합격.
마지막에 처음부터 누적 오차(①② 포함)도 참고로 보고.
합격 기준 (바꾸려면 옵션): 위치 오차 ≤ pos_tol_m + pos_tol_pct·이동거리, yaw 오차 ≤ yaw_tol_deg + yaw_tol_pct·회전량
시뮬은 바퀴-바닥 미끄러짐이 있어서 0 이 아님 — 실기 전 "계산·부호·기하가 맞는지" 확인용.
"""
import argparse
import math
import sys
import time

import rclpy
from capstone_msgs.msg import SteerStateArray, WheelStateArray
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor

from odom_compare import OdomComparer, stamp_s, wrap

# (이름, vx [m/s], vy [m/s], ω [rad/s], 시간 [s])
SCENARIOS = [
    ("직진", 0.30, 0.0, 0.0, 4.0),
    ("후진", -0.30, 0.0, 0.0, 3.0),
    ("옆(좌)", 0.0, 0.20, 0.0, 4.0),
    ("옆(우)", 0.0, -0.20, 0.0, 4.0),
    ("대각", 0.20, 0.20, 0.0, 4.0),
    ("제자리 회전 +360°", 0.0, 0.0, 0.6, 2 * math.pi / 0.6),
    ("제자리 회전 -180°", 0.0, 0.0, -0.6, math.pi / 0.6),
    ("원호 (전진+좌회전)", 0.25, 0.0, 0.40, 8.0),
    ("옆으로+회전", 0.0, 0.20, -0.30, 5.0),
    ("사각형 1/4 앞", 0.25, 0.0, 0.0, 4.0),
    ("사각형 2/4 좌", 0.0, 0.25, 0.0, 4.0),
    ("사각형 3/4 뒤", -0.25, 0.0, 0.0, 4.0),
    ("사각형 4/4 우", 0.0, -0.25, 0.0, 4.0),
]


class Checker:
    def __init__(self, args):
        self.args = args
        self.cmp = OdomComparer(args.odom, args.gt, name="odom_sim_check")
        self.log = self.cmp.get_logger()
        self.pub = self.cmp.create_publisher(Twist, "cmd_vel", 10)
        self.cmd = (0.0, 0.0, 0.0)
        self.cmp.create_timer(0.05, self.send)  # 20 Hz — swerve cmd_timeout 0.3 s 보다 촘촘히
        self.rolling = False
        self.steer_still = False
        self.prev_steer = None
        self.roll_idx = None  # 이번 시나리오에서 "조향 멈춤 + 바퀴 구름" 이 된 pairs 인덱스
        self.cmp.create_subscription(WheelStateArray, "wheel_state", self.on_wheel, 20)
        self.cmp.create_subscription(SteerStateArray, "steer_state", self.on_steer, 20)

    def on_wheel(self, m):
        self.rolling = any(abs(w.velocity) > self.args.roll_thresh for w in m.wheels)

    def on_steer(self, m):
        t, a = stamp_s(m.header), list(m.angle)
        if self.prev_steer and len(a) == len(self.prev_steer[1]) and t > self.prev_steer[0]:
            rate = max(abs(x - y) for x, y in zip(a, self.prev_steer[1])) / (t - self.prev_steer[0])
            self.steer_still = rate < math.radians(self.args.steer_still_dps)
        self.prev_steer = (t, a)

    def send(self):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = self.cmd
        self.pub.publish(t)

    def spin_for(self, ex, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.01)
            if self.roll_idx is None and self.rolling and self.steer_still and self.cmd != (0.0, 0.0, 0.0):
                self.roll_idx = max(len(self.cmp.pairs) - 2, 0)  # wheel_state 가 odom 보다 한 샘플 먼저 올 수 있음

    def wait_ready(self, ex, timeout=15.0):
        self.log.info("/odom 과 /sim/ground_truth 기다리는 중... (시뮬 + swerve_controller + swerve_odom 떠 있어야 함)")
        end = time.monotonic() + timeout
        n0 = None
        while rclpy.ok() and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.05)
            if len(self.cmp.pairs) >= 5:
                n0 = len(self.cmp.pairs)
                break
        if n0 is None:
            self.log.error("짝지은 샘플이 안 들어옴 — sim.launch.py (odom:=true) 가 떠 있는지, ros2 topic hz /odom 확인")
            return False
        return True

    def path(self, i0, i1):
        """pairs[i0..i1] 참값 이동 거리, 누적 |회전|."""
        d = r = 0.0
        for k in range(i0 + 1, i1 + 1):
            a, b = self.cmp.pairs[k - 1][1], self.cmp.pairs[k][1]
            d += math.hypot(b[0] - a[0], b[1] - a[1])
            r += abs(wrap(b[2] - a[2]))
        return d, r

    def run(self, ex):
        a = self.args
        # 처음: 잠깐 0 명령으로 ACTIVE 진입 + 조향 정렬
        self.cmd = (0.0, 0.0, 0.0)
        self.spin_for(ex, 1.0)
        rows = []
        scen = [s for s in SCENARIOS if not a.only or any(k in s[0] for k in a.only)]
        i_first = len(self.cmp.pairs) - 1
        for name, vx, vy, w, dur in scen:
            i0 = len(self.cmp.pairs) - 1
            self.log.info(f"▶ {name}: vx={vx:+.2f} vy={vy:+.2f} ω={w:+.2f} × {dur:.1f} s")
            self.roll_idx = None
            self.cmd = (vx, vy, w)
            self.spin_for(ex, dur)
            self.cmd = (0.0, 0.0, 0.0)
            self.spin_for(ex, a.settle)
            i1 = len(self.cmp.pairs) - 1
            ir = self.roll_idx
            if ir is None or i1 - ir < 5:
                rows.append((name, i0, i1, None))
                self.log.warn("   바퀴가 안 구름 (또는 조향이 안 멈춤) / 샘플 부족")
                continue
            p = self.cmp.pairs
            _, _, _, steer_yaw = OdomComparer.segment_error(p[i0], p[ir])  # ① 조향 전환 중 yaw 오차 (odom − 참값)
            _, _, ep, ey = OdomComparer.segment_error(p[ir], p[i1])  # ② 구름 구간
            dist, rot = self.path(ir, i1)
            tol_p = a.pos_tol_m + a.pos_tol_pct / 100 * dist
            tol_y = math.radians(a.yaw_tol_deg) + a.yaw_tol_pct / 100 * rot
            ok = ep <= tol_p and abs(ey) <= tol_y
            rows.append((name, i0, i1, (steer_yaw, dist, rot, ep, ey, tol_p, tol_y, ok)))
            self.log.info(f"   ① 전환중 yaw 오차 {math.degrees(steer_yaw):+.2f}° | 구름: 이동 {dist:.3f} m, "
                          f"회전 {math.degrees(rot):.0f}° → 오차 {ep * 100:.1f} cm / {math.degrees(ey):+.2f}°  "
                          f"{'OK' if ok else 'NG'}")
        self.cmd = (0.0, 0.0, 0.0)
        self.send()
        return rows, i_first

    def report(self, rows, i_first):
        a = self.args
        W = 104
        print("\n" + "=" * W)
        print(f"{'':<20}{'① 조향 전환':>12} | {'② 조향 멈춘 뒤 바퀴 구름 — 판정 대상':<60}")
        print(f"{'시나리오':<20}{'yaw오차[°]':>12} |{'이동[m]':>8}{'회전[°]':>8}{'위치오차[cm]':>13}{'허용[cm]':>9}"
              f"{'yaw오차[°]':>11}{'허용[°]':>8}  판정")
        print("-" * W)
        all_ok = True
        for name, _, _, r in rows:
            if r is None:
                print(f"{name:<20}  샘플 부족 — NG")
                all_ok = False
                continue
            sy, dist, rot, ep, ey, tp, ty, ok = r
            all_ok &= ok
            print(f"{name:<20}{math.degrees(sy):+12.2f} |{dist:8.3f}{math.degrees(rot):8.0f}{ep * 100:13.2f}"
                  f"{tp * 100:9.2f}{math.degrees(ey):+11.2f}{math.degrees(ty):8.2f}  {'OK' if ok else 'NG'}")
        print("-" * W)
        p = self.cmp.pairs
        if len(p) > i_first + 1:
            _, _, ep, ey = OdomComparer.segment_error(p[i_first], p[-1])
            dist, rot = self.path(i_first, len(p) - 1)
            steer_total = sum(r[3][0] for r in rows if r[3])
            print(f"① 조향 전환 중 yaw 오차 합 (바퀴 odom 사각지대 → IMU 로 보완): {math.degrees(steer_total):+.2f}°, "
                  f"절댓값 합 {math.degrees(sum(abs(r[3][0]) for r in rows if r[3])):.2f}°")
            print(f"전체 누적 (①② 포함, 참고): 이동 {dist:.2f} m, 회전 {math.degrees(rot):.0f}° → 위치 오차 {ep * 100:.1f} cm "
                  f"({100 * ep / max(dist, 1e-9):.2f} %), yaw 오차 {math.degrees(ey):+.2f}°")
            sq = [r for r in rows if r[0].startswith("사각형")]
            if len(sq) == 4:
                s0, s1 = p[sq[0][1]], p[sq[-1][2]]
                dg = math.hypot(s1[1][0] - s0[1][0], s1[1][1] - s0[1][1])
                do = math.hypot(s1[2][0] - s0[2][0], s1[2][1] - s0[2][1])
                _, _, ep, ey = OdomComparer.segment_error(s0, s1)
                print(f"사각형 폐합: 참값 끝점-시작점 {dg * 100:.1f} cm, odom {do * 100:.1f} cm, "
                      f"odom-참값 차이 {ep * 100:.1f} cm / {math.degrees(ey):+.2f}°")
        print(f"판정 기준: 위치 ≤ {a.pos_tol_m * 100:.1f} cm + {a.pos_tol_pct:.1f} %·이동,  "
              f"yaw ≤ {a.yaw_tol_deg:.1f}° + {a.yaw_tol_pct:.1f} %·회전")
        print("결과:", "전부 합격" if all_ok else "불합격 있음")
        return all_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--odom", default="odom")
    ap.add_argument("--gt", default="sim/ground_truth")
    ap.add_argument("--settle", type=float, default=2.0, help="구간마다 정지 후 기다림 [s]")
    ap.add_argument("--roll-thresh", type=float, default=0.05, help="바퀴가 '구른다' 기준 [turn/s] (≈ 1.6 cm/s)")
    ap.add_argument("--steer-still-dps", type=float, default=3.0, help="조향이 '멈췄다' 기준 [°/s] (4 축 중 최대)")
    ap.add_argument("--pos-tol-m", type=float, default=0.02)
    ap.add_argument("--pos-tol-pct", type=float, default=3.0)
    ap.add_argument("--yaw-tol-deg", type=float, default=1.0)
    ap.add_argument("--yaw-tol-pct", type=float, default=3.0)
    ap.add_argument("--only", nargs="*", help="이름에 이 글자가 든 시나리오만")
    ap.add_argument("--plot", help="궤적/오차 그림 저장 경로 (.png)")
    ap.add_argument("--csv", help="짝 샘플 CSV 저장 경로")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    chk = Checker(args)
    ex = SingleThreadedExecutor()
    ex.add_node(chk.cmp)
    ok = False
    try:
        if chk.wait_ready(ex):
            rows, i_first = chk.run(ex)
            ok = chk.report(rows, i_first)
            if args.csv:
                chk.cmp.save_csv(args.csv)
                print(f"CSV: {args.csv}")
            if args.plot and chk.cmp.save_plot(args.plot, "odom_sim_check"):
                print(f"그림: {args.plot}")
    except KeyboardInterrupt:
        chk.cmd = (0.0, 0.0, 0.0)
        chk.send()
    finally:
        ex.shutdown()
        chk.cmp.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
