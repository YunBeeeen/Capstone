#!/usr/bin/env python3
"""바퀴 오도메트리(/odom) ↔ 시뮬 참값(/sim/ground_truth) 실시간 비교.

패드/게임패드로 시뮬을 몰면서 띄워 두면 1 초마다 누적 오차를 찍고, Ctrl+C 때 요약 (+ 그림/CSV).

  ros2 launch capstone_sim sim.launch.py pad:=true          # swerve_odom 포함 (odom:=true 기본)
  ros2 run capstone_sim odom_compare.py                     # 다른 터미널
  ros2 run capstone_sim odom_compare.py --plot /tmp/odom.png --csv /tmp/odom.csv

비교 방식: 처음 맞춘 시점의 두 포즈를 각각 원점으로 잡고 (상대 포즈끼리 비교) — odom 시작점이 참값과 달라도 됨.
시간 매칭: 같은 stamp 의 참값 (시뮬은 wheel_state 와 ground_truth 를 같은 stamp 로 냄), 없으면 선형 보간.
"""
import argparse
import bisect
import math
import sys
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rel(p0, p):
    """p0 기준으로 본 p (2D 포즈 (x, y, yaw))."""
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - p0[2]))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class OdomComparer(Node):
    """/odom 이 올 때마다 같은 시각 참값과 짝지어 저장. odom_sim_check.py 도 이 클래스를 씀."""

    def __init__(self, odom_topic="odom", gt_topic="sim/ground_truth", name="odom_compare"):
        super().__init__(name)
        self.gt_t, self.gt_p = deque(maxlen=500), deque(maxlen=500)  # 50 Hz × 10 s
        self.pairs = []        # (t, gt_pose, odom_pose) — 절대 포즈
        self.gt0 = self.od0 = None
        self.gt_dist = 0.0     # 참값 이동 거리 [m]
        self.gt_rot = 0.0      # 참값 누적 회전 |Δyaw| [rad]
        self.unmatched = 0
        self.pending = deque(maxlen=200)  # 참값이 아직 안 온 odom (t, pose) — 같은 stamp 참값이 뒤에 도착할 수 있음
        self.create_subscription(Odometry, gt_topic, self.on_gt, 50)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 50)

    def on_gt(self, m):
        p = m.pose.pose
        t = stamp_s(m.header)
        if self.gt_t and t <= self.gt_t[-1]:
            return
        self.gt_t.append(t)
        self.gt_p.append((p.position.x, p.position.y, yaw_of(p.orientation)))
        self.match()

    def gt_at(self, t):
        """t 의 참값 (같은 stamp 면 그대로, 사이면 선형 보간). t 가 버퍼 범위 밖이면 None."""
        ts = list(self.gt_t)
        if not ts or t < ts[0] - 1e-6 or t > ts[-1] + 1e-6:
            return None
        i = bisect.bisect_left(ts, t)
        if i < len(ts) and abs(ts[i] - t) < 1e-6:
            return self.gt_p[i]
        if i == 0 or i == len(ts):
            return self.gt_p[min(i, len(ts) - 1)]
        a, b = self.gt_p[i - 1], self.gt_p[i]
        r = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), a[2] + r * wrap(b[2] - a[2]))

    def on_odom(self, m):
        p = m.pose.pose
        self.pending.append((stamp_s(m.header), (p.position.x, p.position.y, yaw_of(p.orientation))))
        self.match()

    def match(self):
        """참값이 그 시각까지 들어온 odom 만 짝지음 (도착 순서가 바뀌어도 한 샘플 밀리지 않게)."""
        while self.pending and self.gt_t and self.pending[0][0] <= self.gt_t[-1] + 1e-6:
            t, o = self.pending.popleft()
            g = self.gt_at(t)
            if g is None:  # 참값 버퍼보다 오래된 odom
                self.unmatched += 1
                continue
            self.add_pair(t, g, o)

    def add_pair(self, t, g, o):
        if self.pairs:
            pg = self.pairs[-1][1]
            self.gt_dist += math.hypot(g[0] - pg[0], g[1] - pg[1])
            self.gt_rot += abs(wrap(g[2] - pg[2]))
        else:
            self.gt0, self.od0 = g, o
        self.pairs.append((t, g, o))

    # ---------------------------------------------------------------- 오차
    def latest(self):
        return self.pairs[-1] if self.pairs else None

    @staticmethod
    def segment_error(start, end):
        """두 짝 (t, gt, od) 사이 상대 운동끼리 비교 → (참값 이동 [m], 참값 회전 [rad], 위치 오차 [m], yaw 오차 [rad])."""
        dg = rel(start[1], end[1])
        do = rel(start[2], end[2])
        return math.hypot(dg[0], dg[1]), dg[2], math.hypot(do[0] - dg[0], do[1] - dg[1]), wrap(do[2] - dg[2])

    def total_error(self):
        """처음 짝 기준 누적 오차 → (위치 오차 [m], yaw 오차 [rad])."""
        if not self.pairs:
            return None
        _, _, ep, ey = self.segment_error(self.pairs[0], self.pairs[-1])
        return ep, ey

    # ---------------------------------------------------------------- 저장
    def save_csv(self, path):
        with open(path, "w") as f:
            f.write("t,gt_x,gt_y,gt_yaw,od_x,od_y,od_yaw\n")
            for t, g, o in self.pairs:
                f.write(f"{t:.4f},{g[0]:.5f},{g[1]:.5f},{g[2]:.5f},{o[0]:.5f},{o[1]:.5f},{o[2]:.5f}\n")

    def save_plot(self, path, title="swerve odom vs ground truth"):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn("matplotlib 없음 — 그림 생략")
            return False
        if len(self.pairs) < 2:
            return False
        t0 = self.pairs[0][0]
        g = [rel(self.pairs[0][1], p[1]) for p in self.pairs]
        o = [rel(self.pairs[0][2], p[2]) for p in self.pairs]
        ts = [p[0] - t0 for p in self.pairs]
        ep = [math.hypot(a[0] - b[0], a[1] - b[1]) * 100 for a, b in zip(o, g)]
        ey = [math.degrees(wrap(a[2] - b[2])) for a, b in zip(o, g)]
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        ax[0].plot([p[0] for p in g], [p[1] for p in g], label="ground truth", lw=2)
        ax[0].plot([p[0] for p in o], [p[1] for p in o], "--", label="wheel odom", lw=1.5)
        ax[0].set_aspect("equal")
        ax[0].set_xlabel("x [m]")
        ax[0].set_ylabel("y [m]")
        ax[0].legend()
        ax[0].grid(alpha=0.3)
        ax[0].set_title(title)
        ax[1].plot(ts, ep, label="position error [cm]")
        ax1b = ax[1].twinx()
        ax1b.plot(ts, ey, color="tab:red", alpha=0.7, label="yaw error [deg]")
        ax[1].set_xlabel("t [s]")
        ax[1].set_ylabel("position error [cm]")
        ax1b.set_ylabel("yaw error [deg]", color="tab:red")
        ax[1].grid(alpha=0.3)
        ax[1].set_title("cumulative error (from start)")
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return True


def summary(c):
    if not c.pairs:
        return "짝지은 샘플 없음 — /odom 과 /sim/ground_truth 가 둘 다 나오는지 확인"
    ep, ey = c.total_error()
    pct = 100 * ep / c.gt_dist if c.gt_dist > 0.05 else float("nan")
    return (f"{c.pairs[-1][0] - c.pairs[0][0]:6.1f} s | 이동 {c.gt_dist:6.2f} m, 회전 {math.degrees(c.gt_rot):6.0f}° | "
            f"위치 오차 {ep * 100:5.1f} cm ({pct:4.1f} %) | yaw 오차 {math.degrees(ey):+5.2f}° | 샘플 {len(c.pairs)}"
            + (f" (짝 못 찾음 {c.unmatched})" if c.unmatched else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--odom", default="odom")
    ap.add_argument("--gt", default="sim/ground_truth")
    ap.add_argument("--period", type=float, default=1.0, help="출력 주기 [s]")
    ap.add_argument("--plot", help="끝낼 때 궤적/오차 그림 저장 경로 (.png)")
    ap.add_argument("--csv", help="끝낼 때 짝 샘플 CSV 저장 경로")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    c = OdomComparer(args.odom, args.gt)
    c.create_timer(args.period, lambda: c.get_logger().info(summary(c)))
    c.get_logger().info(f"/{args.odom} ↔ /{args.gt} 비교 시작 — 시뮬을 움직여 보세요 (Ctrl+C 로 요약)")
    try:
        rclpy.spin(c)
    except KeyboardInterrupt:
        pass
    print("\n=== 요약 ===\n" + summary(c))
    if args.csv:
        c.save_csv(args.csv)
        print(f"CSV: {args.csv}")
    if args.plot and c.save_plot(args.plot):
        print(f"그림: {args.plot}")
    c.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
