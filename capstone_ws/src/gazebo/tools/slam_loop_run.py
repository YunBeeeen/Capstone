"""school_floor 복도 루프(61 m)를 참값으로 자동 주행하면서 SLAM 결과 기록 → 그림 + 수치.
run_slam_loop.sh 가 Gazebo + slam.launch 를 띄운 뒤 부름. 최종 지도 정확도는 slam_eval_db.py (DB 의 최적화 포즈).

  python3 slam_loop_run.py <출력디렉터리> [비교할 odometry 토픽 (기본 odometry/filtered)]
기록: 참값(sim/ground_truth), odometry, SLAM 실시간 포즈(TF map→base_footprint — 루프 클로저 전엔 odom 과 같음),
      odom_info.lost (RTAB-Map 시각 odometry 쓸 때만), info.loop_closure_id, /rtabmap/map (점유격자)
※ 시뮬 수치는 "파이프라인이 연결되고 동작한다" 확인용. 성능 판단은 실제 데이터로 (1 회 결과로 결론 내지 말 것)
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info, OdomInfo
from tf2_ros import Buffer, TransformListener

OUT = sys.argv[1]
ODOM = sys.argv[2] if len(sys.argv) > 2 else "odometry/filtered"  # 비교할 odometry 토픽
# 복도 루프: 남(동쪽으로) → 동(북쪽으로, 흰 벽) → 북(서쪽으로) → 서(남쪽으로) → 남쪽 다시 (루프 클로저)
WAYPOINTS = [(18.75, 1.25), (18.75, 10.75), (1.25, 10.75), (1.25, 1.25), (9.0, 1.25)]
V, W_MAX = 0.3, 0.4


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Run(Node):
    def __init__(self):
        super().__init__("vslam_run", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.gt = self.vo = None
        self.lost = False
        self.lost_count = self.odom_info_count = 0
        self.loops = []
        self.map = None
        self.log = []  # (t, gt, vo, slam, lost)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "sim/ground_truth", self.on_gt, 20)
        self.create_subscription(Odometry, ODOM, self.on_vo, 20)
        self.create_subscription(OdomInfo, "rtabmap/odom_info", self.on_oi, 20)
        self.create_subscription(Info, "rtabmap/info", self.on_info, 20)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "rtabmap/map", self.on_map, latched)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_q(p.orientation))

    def on_vo(self, m):
        p = m.pose.pose
        self.vo = (p.position.x, p.position.y, yaw_q(p.orientation))

    def on_oi(self, m):
        self.odom_info_count += 1
        self.lost = m.lost
        self.lost_count += int(m.lost)

    def on_info(self, m):
        if m.loop_closure_id > 0:
            self.loops.append((self.get_clock().now().nanoseconds * 1e-9, m.ref_id, m.loop_closure_id, self.gt))

    def on_map(self, m):
        self.map = m

    def slam_pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        r = t.transform
        return (r.translation.x, r.translation.y, yaw_q(r.rotation))

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def record(self):
        self.log.append((self.get_clock().now().nanoseconds * 1e-9, self.gt, self.vo, self.slam_pose(), self.lost))

    def drive(self):
        last_rec = 0.0
        for wx, wy in WAYPOINTS:
            self.get_logger().info(f"→ ({wx}, {wy})")
            t_start = time.monotonic()
            while time.monotonic() - t_start < 200:
                rclpy.spin_once(self, timeout_sec=0.05)
                x, y, yaw = self.gt
                dx, dy = wx - x, wy - y
                dist = math.hypot(dx, dy)
                if dist < 0.15:
                    break
                e = wrap(math.atan2(dy, dx) - yaw)
                cmd = Twist()
                cmd.angular.z = max(-W_MAX, min(W_MAX, 1.5 * e))
                if abs(e) < 0.25:
                    cmd.linear.x = min(V, 0.8 * dist)
                    # 옆 오차 보정 (복도 가운데 유지) — 진행 방향에 수직인 성분
                    cmd.linear.y = max(-0.1, min(0.1, dist * math.sin(e)))
                self.pub.publish(cmd)
                if time.monotonic() - last_rec > 0.2:
                    last_rec = time.monotonic()
                    self.record()
            self.pub.publish(Twist())
        self.spin_for(3.0)
        self.record()


def rel(p0, p):
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - p0[2]))


def main():
    rclpy.init()
    n = Run()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 60 and (n.gt is None or n.vo is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.gt is None or n.vo is None:
        print(f"시작 실패: gt={n.gt is not None} vo={n.vo is not None}")
        return
    n.spin_for(2.0)
    n.record()
    print("주행 시작")
    t_drive = time.monotonic()
    n.drive()
    print(f"주행 끝 ({time.monotonic() - t_drive:.0f} s 실시간)")

    # ---- 분석 (시작 포즈 기준 상대)
    L = [r for r in n.log if r[1] and r[2]]
    g0, v0 = L[0][1], L[0][2]
    s0 = next((r[3] for r in L if r[3]), None)
    gt = np.array([rel(g0, r[1]) for r in L])
    vo = np.array([rel(v0, r[2]) for r in L])
    sl = np.array([rel(s0, r[3]) if r[3] and s0 else (np.nan,) * 3 for r in L])
    lost = np.array([r[4] for r in L])
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(gt[:, 0]), np.diff(gt[:, 1])))])
    e_vo = np.hypot(vo[:, 0] - gt[:, 0], vo[:, 1] - gt[:, 1])
    e_sl = np.hypot(sl[:, 0] - gt[:, 0], sl[:, 1] - gt[:, 1])
    print(f"주행 거리 {dist[-1]:.1f} m, 샘플 {len(L)}, odom_info {n.odom_info_count} (lost {n.lost_count}), "
          f"lost 샘플 비율 {lost.mean() * 100:.1f}%")
    print(f"odometry({ODOM}) 끝점 오차 {e_vo[-1] * 100:.1f} cm ({e_vo[-1] / dist[-1] * 100:.2f} %), "
          f"yaw {math.degrees(wrap(vo[-1, 2] - gt[-1, 2])):+.2f}°, 최대 {np.nanmax(e_vo) * 100:.1f} cm")
    print(f"SLAM(map) 끝점 오차 {e_sl[-1] * 100:.1f} cm, yaw {math.degrees(wrap(sl[-1, 2] - gt[-1, 2])):+.2f}°, "
          f"최대 {np.nanmax(e_sl) * 100:.1f} cm, 평균 {np.nanmean(e_sl) * 100:.1f} cm")
    print(f"루프 클로저 {len(n.loops)} 번" + (": " + ", ".join(
        f"노드 {a}↔{b} @({g[0]:.1f},{g[1]:.1f})" for _, a, b, g in n.loops[:8]) if n.loops else ""))
    # lost 구간 위치
    if lost.any():
        idx = np.where(lost)[0]
        pts = [L[i][1] for i in idx]
        print(f"lost 위치 (참값): x {min(p[0] for p in pts):.1f}~{max(p[0] for p in pts):.1f}, "
              f"y {min(p[1] for p in pts):.1f}~{max(p[1] for p in pts):.1f}")
    np.save(f"{OUT}/gt_abs.npy", np.array([(r[0],) + tuple(r[1]) for r in n.log if r[1]]))  # t(sim), x, y, yaw (월드)
    if n.map is not None:
        m = n.map
        np.save(f"{OUT}/grid.npy", np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width))
        np.save(f"{OUT}/grid_info.npy", np.array([m.info.resolution, m.info.origin.position.x, m.info.origin.position.y]))
    np.save(f"{OUT}/traj.npy", np.concatenate([gt, vo, sl, lost[:, None], dist[:, None]], 1))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    ax[0].plot(gt[:, 0], gt[:, 1], lw=2, label="ground truth")
    ax[0].plot(vo[:, 0], vo[:, 1], "--", label=f"odometry ({ODOM})")
    ax[0].plot(sl[:, 0], sl[:, 1], ":", lw=2, label="SLAM (map, real-time)")
    if lost.any():
        ax[0].plot(gt[lost, 0], gt[lost, 1], "rx", ms=4, label="VO lost")
    ax[0].set_aspect("equal")
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[0].set_title("trajectory (start-relative)")
    ax[1].plot(dist, e_vo * 100, label=f"odometry ({ODOM})")
    ax[1].plot(dist, e_sl * 100, label="SLAM (map)")
    ax[1].set_xlabel("distance [m]")
    ax[1].set_ylabel("position error [cm]")
    ax[1].legend()
    ax[1].grid(alpha=0.3)
    if n.map is not None:
        m = n.map
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        img = np.full(g.shape, 0.5)
        img[g == 0] = 1.0
        img[g > 50] = 0.0
        o = m.info.origin.position
        ext = [o.x, o.x + m.info.width * m.info.resolution, o.y, o.y + m.info.height * m.info.resolution]
        ax[2].imshow(img, cmap="gray", origin="lower", extent=ext)
        ax[2].set_title(f"rtabmap/map {m.info.width}x{m.info.height} @ {m.info.resolution:.2f} m")
        print(f"점유격자 {m.info.width}x{m.info.height}, 해상도 {m.info.resolution} m, "
              f"빈칸 {(g == 0).sum()}, 장애물 {(g > 50).sum()}, 모름 {(g < 0).sum()}")
    else:
        print("점유격자 안 받음")
    fig.tight_layout()
    fig.savefig(f"{OUT}/vslam_result.png", dpi=90)
    rclpy.shutdown()


main()
