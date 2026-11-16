#!/usr/bin/env python3
"""RViz 에서 IK 후보와 작업 공간을 본다 (4층 사다리 선반 + 책상, ik_scene.py 와 같은 장면).

  source /opt/ros/humble/setup.bash && source capstone_ws/install/setup.bash
  python3 tools/ik_rviz.py --target shelf3 --approach into
  python3 tools/ik_rviz.py --xyz 0.3 -0.5 0.95          # 로봇 바닥 기준 목표 직접
  python3 tools/ik_rviz.py --target desk --no-rviz      # RViz 는 따로 띄울 때

RViz 고정 프레임 = 로봇 바닥 기준 'world' (x 앞 빨강, y 왼쪽 초록, z 위 파랑).
터미널 입력: n 다음 후보 · p 이전 · 숫자 그 후보 · w 작업 공간 전체 ↔ 목표 높이 단면 · q 종료.
발행: /robot_description, TF(world → 각 링크, Pinocchio FK), /ik/markers, /ik/workspace (intensity = σmin).
로봇 상태 퍼블리셔·joint_states 없이 이 노드가 TF 를 직접 낸다.
"""
import argparse
import math
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
import rclpy.executors
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics import B601Kinematics  # noqa: E402
from arm_kinematics.collision import CollisionChecker  # noqa: E402
from arm_kinematics.ik import B601IK, describe  # noqa: E402
from arm_kinematics.scene import (  # noqa: E402
    ALONG, APPROACH_HELP, FRONT, LATERAL, PRESETS, env_for, orientation_args, Placement, preset_target, robot_to_room, room_to_robot, to_robot)

LIFTS_MM = "0,210,420,630"
FRAME = "world"


def latched():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def workspace_cloud(kin, checker, lift, n, seed=0):
    rng = np.random.default_rng(seed)
    lo, hi = np.degrees(kin.arm_lower), np.degrees(kin.arm_upper)
    pts, sig = [], []
    for _ in range(n):
        arm = list(rng.uniform(lo, hi))
        if checker.in_collision(arm, lift):
            continue
        p = kin.fk(arm, lift).translation
        if p[2] < 0.02:
            continue
        pts.append(p)
        sig.append(kin.manipulability(arm, lift)["sigma_min"])
    return np.array(pts, dtype=np.float32).reshape(-1, 3), np.array(sig, dtype=np.float32)


class IKViz(Node):
    def __init__(self, args):
        super().__init__("ik_rviz")
        self.kin = kin = B601Kinematics()
        if args.xyz:
            self.robot = Placement(args.robot[0], args.robot[1], math.radians(args.robot[2])) if args.robot else Placement(0, 0, 0)
            self.target = np.array(args.xyz)
            into = np.array([1.0, 0.0, 0.0])
        else:
            tgt_room, into_room, self.robot = preset_target(args.target, args.height, front=args.front, lateral=args.lateral, along=args.along)
            if args.robot:
                self.robot = Placement(args.robot[0], args.robot[1], math.radians(args.robot[2]))
            self.target = room_to_robot(tgt_room, self.robot)
            into = room_to_robot(robot_to_room([0, 0, 0], self.robot) + into_room, self.robot)
        self.ik_kw, self.plan_kw, self.approach = orientation_args(
            args.approach, into, [float(v) for v in args.tilt.split(",")])
        self.env = to_robot(env_for(args.target, args.front, args.lateral), self.robot)
        self.checker = CollisionChecker(kin, self.env)
        lifts = [float(v) / 2000 for v in args.lifts.split(",")]
        cands = B601IK(kin, collision=self.checker).solve(self.target, lifts, n_random=args.seeds, **self.ik_kw)
        self.cands = cands[:args.top]
        print(f"목표(로봇 기준) {np.round(self.target, 3)}  접근 {args.approach}  후보 {len(cands)} (OK {sum(c.ok for c in cands)})")
        for i, c in enumerate(self.cands):
            print(f"{i:2d} {describe(c)}")
        if not self.cands:
            raise SystemExit("후보 없음 — --front / --lateral / --approach free / --lifts 를 바꿔 보세요.")
        self.ws_n, self.ws_cache, self.slice = args.ws_samples, {}, False
        self.i = 0

        self.pub_desc = self.create_publisher(String, "/robot_description", latched())
        self.pub_mk = self.create_publisher(MarkerArray, "/ik/markers", latched())
        self.pub_ws = self.create_publisher(PointCloud2, "/ik/workspace", latched())
        self.tf = TransformBroadcaster(self)
        self.tf_data = kin.model.createData()          # TF 타이머 스레드 전용 (작업 공간 계산과 분리)
        self.pub_desc.publish(String(data=Path(kin.urdf_path).read_text()))
        self.lock = threading.Lock()
        self.show(0)
        self.create_timer(0.1, self.publish_tf)

    # ---- state ----
    def config(self):
        c = self.cands[self.i]
        return self.kin.configuration(c.motors_deg, c.lift_stage_m), c

    def publish_tf(self):
        with self.lock:
            q, _ = self.config()
        m, d = self.kin.model, self.tf_data
        pin.framesForwardKinematics(m, d, q)
        root = d.oMf[m.getFrameId(FRAME)]
        now = self.get_clock().now().to_msg()
        msgs = []
        for fid, f in enumerate(m.frames):
            if f.type != pin.FrameType.BODY or f.name == FRAME:
                continue
            T = root.actInv(d.oMf[fid])
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, FRAME, f.name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T.translation)
            qx, qy, qz, qw = pin.Quaternion(T.rotation).coeffs()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, (qx, qy, qz, qw))
            msgs.append(t)
        self.tf.sendTransform(msgs)

    def show(self, i):
        with self.lock:
            self.i = i % len(self.cands)
        c = self.cands[self.i]
        print(f"\n[{self.i + 1}/{len(self.cands)}] {describe(c)}")
        self.publish_markers(c)
        self.publish_workspace(c)

    def publish_markers(self, c):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = FRAME
        arr.markers.append(clear)

        def mk(ns, mid, mtype, rgba):
            m = Marker()
            m.header.frame_id, m.header.stamp, m.ns, m.id, m.type, m.action = FRAME, now, ns, mid, mtype, Marker.ADD
            m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
            m.pose.orientation.w = 1.0
            m.lifetime = Duration()
            return m
        for k, b in enumerate(self.env):
            col = (0.9, 0.88, 0.94, 0.9) if "board" in b.name else (0.82, 0.65, 0.43, 0.9) if b.name == "desk_top" else (0.15, 0.15, 0.15, 0.9)
            m = mk("env", k, Marker.CUBE, col)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, b.center)
            m.pose.orientation.z, m.pose.orientation.w = math.sin(b.yaw / 2), math.cos(b.yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = map(float, b.size)
            arr.markers.append(m)
        t = mk("target", 0, Marker.SPHERE, (0.95, 0.1, 0.1, 1.0))
        t.pose.position.x, t.pose.position.y, t.pose.position.z = map(float, self.target)
        t.scale.x = t.scale.y = t.scale.z = 0.03
        arr.markers.append(t)
        if self.approach is not None:
            a = mk("target", 1, Marker.ARROW, (0.95, 0.1, 0.1, 1.0))
            a.points = [Point(x=float(v), y=float(w), z=float(u)) for v, w, u in (self.target - 0.15 * self.approach, self.target)]
            a.scale.x, a.scale.y, a.scale.z = 0.008, 0.02, 0.03
            arr.markers.append(a)
        txt = mk("text", 0, Marker.TEXT_VIEW_FACING, (1, 1, 1, 1))
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = float(self.target[0]), float(self.target[1]), float(self.target[2] + 0.25)
        terms = " + ".join(f"{k} {v:.2f}" for k, v in c.cost_terms.items())
        # RViz 글꼴에 한글이 없을 수 있어 ASCII 로
        txt.text = (f"cand {self.i + 1}/{len(self.cands)}  {'OK' if c.ok else 'COLLISION/NOT CONVERGED'}\n"
                    f"cost {c.cost:.2f} = {terms}\nlift {2000 * c.lift_stage_m:.0f} mm  sigma_min {c.sigma_min:.3f}  load {c.load:.2f}")
        txt.scale.z = 0.035
        arr.markers.append(txt)
        self.pub_mk.publish(arr)

    def publish_workspace(self, c):
        key = c.lift_stage_m
        if key not in self.ws_cache:
            print(f"작업 공간 계산 중 (리프트 {2000 * key:.0f} mm, {self.ws_n} 샘플, 환경 충돌 포함)…", flush=True)
            self.ws_cache[key] = workspace_cloud(self.kin, self.checker, key, self.ws_n)
        pts, sig = self.ws_cache[key]
        if self.slice:
            sel = np.abs(pts[:, 2] - self.target[2]) <= 0.03
            pts, sig = pts[sel], sig[sel]
        data = np.hstack((pts, sig[:, None])).astype(np.float32)
        msg = PointCloud2(header=Header(frame_id=FRAME, stamp=self.get_clock().now().to_msg()), height=1, width=len(data),
                          fields=[PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                                  for k, n in enumerate(("x", "y", "z", "intensity"))],
                          is_bigendian=False, point_step=16, row_step=16 * len(data), is_dense=True, data=data.tobytes())
        self.pub_ws.publish(msg)
        print(f"작업 공간 점 {len(data)} 개 ({'목표 높이 ±3 cm' if self.slice else '전체'})", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=PRESETS, default="desk")
    ap.add_argument("--xyz", type=float, nargs=3)
    ap.add_argument("--robot", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"))
    ap.add_argument("--approach", default="heading", help=APPROACH_HELP)
    ap.add_argument("--tilt", default="-10,45", help="heading 모드 기울기 범위 [도] (아래 +)")
    ap.add_argument("--front", type=float, default=FRONT, help="로봇 중심 → 가구 앞면 [m] (설 자리 거리)")
    ap.add_argument("--lateral", type=float, default=LATERAL, help="가구 가운데 y [m] (− = 로봇 오른쪽)")
    ap.add_argument("--along", type=float, default=ALONG, help="목표를 가구 가운데에서 y 로 옮긴 양 [m]")
    ap.add_argument("--height", type=float, default=0.05)
    ap.add_argument("--lifts", default=LIFTS_MM, help="총 리프트 mm 목록")
    ap.add_argument("--seeds", type=int, default=24)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--ws-samples", type=int, default=12000)
    ap.add_argument("--no-rviz", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = IKViz(args)
    rviz = None if args.no_rviz else subprocess.Popen(["rviz2", "-d", str(ROOT / "tools/ik_rviz.rviz")])
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    print("\n입력: n 다음 · p 이전 · 숫자 · w 전체/단면 · q 종료")
    try:
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd in ("n", ""):
                node.show(node.i + 1)
            elif cmd == "p":
                node.show(node.i - 1)
            elif cmd == "w":
                node.slice = not node.slice
                node.publish_workspace(node.cands[node.i])
            elif cmd.isdigit():
                node.show(int(cmd))
    except KeyboardInterrupt:
        pass
    finally:
        if rviz:
            rviz.terminate()
        executor.shutdown()
        spin.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
