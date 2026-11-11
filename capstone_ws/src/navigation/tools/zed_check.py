#!/usr/bin/env python3
"""ZED 2i 연결 직후 점검 — zed wrapper 를 띄운 상태에서 실행. 카메라를 책상 위에 가만히 둘 것.

  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed publish_tf:=false publish_map_tf:=false \\
    ros_params_override_path:=$(ros2 pkg prefix capstone_navigation)/share/capstone_navigation/config/zed_override.yaml
  python3 src/navigation/tools/zed_check.py                 # 기본: 정지 드리프트 300 s
  python3 src/navigation/tools/zed_check.py --drift 60      # 짧게
  python3 src/navigation/tools/zed_check.py --ns /zed/zed_node --drift 0   # 드리프트 시험 생략
  python3 src/navigation/tools/zed_check.py --vio-only      # ZED 단독 구성 (zed_vio_only.yaml, publish_tf:=false) 기준으로 확인

확인 항목 (결과에 따라 할 일은 출력에 같이 나옴):
  1) 토픽: 시뮬(capstone_gazebo)·RTAB-Map·Nav2 가 기대하는 이름이 있는지, 없으면 비슷한 이름 제안
  2) 주기: RGB, depth, IMU, odom
  3) odom 프레임: header.frame_id / child_frame_id — 카메라 프레임이면 차체 기준 변환 노드 필요
  4) 파라미터: GEN_3, two_d_mode, area_memory(꺼져야), imu_fusion, publish_tf(꺼져야), min_depth
  5) 정지 드리프트: 자이로 적분 각도·바이어스, ZED odom 의 위치·방향 변화 (가만히 있는데 움직이면 그만큼 드리프트)
"""
import argparse
import math
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

EXPECT = {  # 상대 이름 → 왜 필요한가
    "rgb/color/rect/image": "RTAB-Map rgb, 선반 인식",
    "rgb/color/rect/camera_info": "RTAB-Map",
    "depth/depth_registered": "RTAB-Map depth, Nav2 장애물",
    "depth/camera_info": "Nav2 장애물 점구름",
    "imu/data": "EKF 자이로",
    "odom": "EKF VIO (GEN_3)",
    "pose": "참고 (루프 클로저 반영 위치)",
}
PARAMS = {  # 이름 → (기대값, 설명)
    "pos_tracking.pos_tracking_mode": ("GEN_3", "GEN_3 여야 함"),
    "pos_tracking.imu_fusion": (True, "카메라+IMU 융합"),
    "pos_tracking.area_memory": (False, "켜져 있으면 루프 클로저 때 odom 순간이동 → EKF 튐"),
    "pos_tracking.two_d_mode": (True, "바닥 로봇"),
    "pos_tracking.publish_tf": (False, "TF 는 EKF 가 냄 (launch 인자 publish_tf:=false)"),
    "pos_tracking.publish_map_tf": (False, "map→odom 은 RTAB-Map 이 냄"),
    "depth.min_depth": (0.3, "ZED 2i 최소 거리"),
    "general.grab_frame_rate": (30, None),
}


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self, ns):
        super().__init__("zed_check")
        self.ns = ns.rstrip("/")

    def topics(self):
        return dict(self.get_topic_names_and_types())

    def measure(self, names, seconds):
        """토픽별 수신 개수 + 마지막 메시지."""
        tt = self.topics()
        cnt, last, subs = {}, {}, []
        for n in names:
            full = f"{self.ns}/{n}"
            if full not in tt:
                continue
            cls = get_message(tt[full][0])
            cnt[n] = 0

            def cb(m, n=n):
                cnt[n] += 1
                last[n] = m
            subs.append(self.create_subscription(cls, full, cb, qos_profile_sensor_data))
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        for s in subs:
            self.destroy_subscription(s)
        return {n: c / seconds for n, c in cnt.items()}, last

    def params(self):
        cli = self.create_client(GetParameters, f"{self.ns}/get_parameters")
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        out = {}
        for name in PARAMS:  # 하나씩 (한 번에 물으면 없는 이름이 하나라도 있을 때 빈 응답이 옴)
            req = GetParameters.Request()
            req.names = [name]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            vals = fut.result().values if fut.result() else []
            v = vals[0] if vals else None
            out[name] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(v.type, "(없음)") if v else "(없음)"
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default="/zed/zed_node")
    ap.add_argument("--drift", type=float, default=300.0, help="정지 드리프트 시험 시간 [s] (0 = 생략)")
    ap.add_argument("--vio-only", action="store_true", help="ZED 단독 구성: two_d_mode false, publish_tf false 를 기대")
    a = ap.parse_args()
    if a.vio_only:
        PARAMS["pos_tracking.two_d_mode"] = (False, "ZED 단독: 기울여 단 카메라의 pitch 가 0 으로 고정되면 안 됨")
        PARAMS["pos_tracking.publish_tf"] = (False, "ZED 단독: TF 는 zed_base_tf.py 가 냄 (launch 인자 publish_tf:=false)")
    rclpy.init()
    n = Check(a.ns)
    time.sleep(1.0)
    warn = []

    # 1) 토픽
    print(f"== 1) 토픽 ({n.ns}/...)")
    tt = n.topics()
    have = sorted(t for t in tt if t.startswith(n.ns + "/"))
    if not have:
        print(f"  {n.ns} 아래 토픽 없음 — wrapper 가 떠 있는지, camera_name:=zed 인지 확인")
        return
    for rel, why in EXPECT.items():
        full = f"{n.ns}/{rel}"
        if full in tt:
            print(f"  OK   {rel:28s} [{tt[full][0]}]")
        else:
            near = [t.replace(n.ns + "/", "") for t in have if rel.split("/")[0] in t][:4]
            print(f"  없음 {rel:28s} ({why}) — 비슷한 것: {near}")
            warn.append(f"토픽 이름 다름: {rel} → {near} 로 시뮬 센서·RTAB-Map·Nav2 remap 수정")

    # 2) 주기 + 3) 프레임
    print("== 2) 주기 (5 s)")
    hz, last = n.measure(list(EXPECT), 5.0)
    for k, v in hz.items():
        print(f"  {k:28s} {v:6.1f} Hz")
    if hz.get("imu/data", 0) < 90:  # 래퍼 기본 100 Hz, 5 s 측정이라 98 정도로 나옴
        warn.append("IMU 주기가 낮음 (< 90 Hz)")
    print("== 3) 프레임")
    for k in ("odom", "pose", "imu/data", "rgb/color/rect/image", "depth/depth_registered"):
        m = last.get(k)
        if m is None:
            continue
        child = getattr(m, "child_frame_id", "")
        print(f"  {k:28s} frame_id={m.header.frame_id!r}" + (f" child_frame_id={child!r}" if child else ""))
    od = last.get("odom")
    if od is not None:
        if "base" not in od.child_frame_id and not a.vio_only:  # ZED 단독은 slam.launch 가 고정 TF 로 base_footprint 를 붙임
            warn.append(f"odom 이 카메라 프레임({od.child_frame_id}) 기준 → EKF 전에 차체(base_footprint) 기준으로 바꾸는 노드 필요")
        cov = list(od.pose.covariance)
        print(f"  odom 공분산 대각 (x, y, yaw) = {cov[0]:.2e}, {cov[7]:.2e}, {cov[35]:.2e}" + ("  ← 0 이면 EKF 에 못 씀, 노드에서 채워야 함" if cov[0] == 0 else ""))
        if cov[0] == 0:
            warn.append("odom 공분산이 0 — 변환 노드에서 값 채우기")
    st = [t for t in have if "status" in t]
    print(f"  상태 토픽: {st}")

    # 4) 파라미터
    print("== 4) 파라미터")
    p = n.params()
    if p is None:
        print("  get_parameters 서비스 응답 없음")
    else:
        for k, (exp, why) in PARAMS.items():
            v = p[k]
            ok = (abs(v - exp) < 1e-6) if isinstance(exp, float) and isinstance(v, (int, float)) else v == exp
            print(f"  {'OK  ' if ok else '다름'} {k:34s} = {v!r}" + ("" if ok else f"  (기대 {exp!r}{' — ' + why if why else ''})"))
            if k == "pos_tracking.publish_map_tf" and v == "(없음)" and p.get("pos_tracking.publish_tf") is False:
                ok = True  # publish_tf 가 꺼지면 wrapper 가 이 파라미터를 만들지 않음
            if not ok and why:
                warn.append(f"{k} = {v!r} (기대 {exp!r}: {why})")

    # 5) 정지 드리프트
    if a.drift > 0:
        print(f"== 5) 정지 드리프트 {a.drift:.0f} s — 카메라를 건드리지 말 것")
        imu_cls = get_message(tt[f"{n.ns}/imu/data"][0]) if f"{n.ns}/imu/data" in tt else None
        odom_cls = get_message(tt[f"{n.ns}/odom"][0]) if f"{n.ns}/odom" in tt else None
        acc = {"t": None, "ang": [0.0, 0.0, 0.0], "w": [], "o0": None, "o1": None}

        def on_imu(m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            w = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
            if acc["t"] is not None and 0 < t - acc["t"] < 0.1:
                for i in range(3):
                    acc["ang"][i] += w[i] * (t - acc["t"])
            acc["t"] = t
            acc["w"].append(w)

        def on_odom(m):
            p_ = m.pose.pose
            o = (p_.position.x, p_.position.y, yaw_q(p_.orientation))
            acc["o0"] = acc["o0"] or o
            acc["o1"] = o
        subs = []
        if imu_cls:
            subs.append(n.create_subscription(imu_cls, f"{n.ns}/imu/data", on_imu, qos_profile_sensor_data))
        if odom_cls:
            subs.append(n.create_subscription(odom_cls, f"{n.ns}/odom", on_odom, qos_profile_sensor_data))
        t0 = time.monotonic()
        nxt = 0
        while time.monotonic() - t0 < a.drift:
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.monotonic() - t0 > nxt:
                nxt += 30
                print(f"  {time.monotonic() - t0:5.0f} s: 자이로 적분 z {math.degrees(acc['ang'][2]):+.2f}°")
        if acc["w"]:
            k = len(acc["w"])
            bias = [sum(w[i] for w in acc["w"]) / k for i in range(3)]
            print(f"  자이로 적분 (x, y, z) = " + ", ".join(f"{math.degrees(v):+.2f}°" for v in acc["ang"])
                  + f"  /  바이어스 z {math.degrees(bias[2]):+.4f}°/s")
            dz = abs(math.degrees(acc["ang"][2])) * 300 / a.drift
            print(f"  → 5 분 환산 yaw 드리프트 {dz:.2f}°  ({'시뮬 기본값 수준' if dz < 2 else '시뮬보다 큼 — 시뮬 IMU 잡음을 이 값으로 올릴 것'})")
        if acc["o0"] and acc["o1"]:
            o0, o1 = acc["o0"], acc["o1"]
            print(f"  ZED odom 변화: 위치 {math.hypot(o1[0] - o0[0], o1[1] - o0[1]) * 100:.1f} cm, "
                  f"방향 {math.degrees(math.atan2(math.sin(o1[2] - o0[2]), math.cos(o1[2] - o0[2]))):+.2f}° (정지 중이므로 0 에 가까워야)")

    print("== 요약")
    print("  문제 없음" if not warn else "\n".join(f"  - {w}" for w in warn))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
