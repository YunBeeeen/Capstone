#!/usr/bin/env python3
"""리프트 높이 → 카메라 높이 (zed_base_tf 의 cam_z) 를 계속 맞춤 — **Jetson 에서 실행** (jetson_nav.launch 가 같이 켬).

  python3 lift_cam_z.py                          # 라파 = $PAD_HOST (없으면 raspberrypi.local), 포트 5007
  python3 lift_cam_z.py --to 192.168.0.12

0.2 s 마다 라파 lift_udp.py 에 높이를 묻고  cam_z = base + height_mm / 1000  (base 0.675 = 리프트 최저일 때 바닥→ZED 나사)
을 /zed_base_tf 파라미터로 넣음 (3 mm 넘게 바뀔 때만). 리프트 325 mm → 1.000 m.
**값을 못 받거나 이상하면 cam_z = 1.0 m** (--fallback): 답이 1.5 s 넘게 없음, /lift_state 가 1 s 넘게 안 옴,
Pico 끊김, 원점 미등록 (NaN), 높이가 −5..650 mm 밖.
바뀔 때만 한 줄 출력 ("리프트 325 mm → cam_z 1.000" / "리프트 값 없음 (…) → cam_z 1.000 (기본)").
"""
import argparse
import math
import os
import socket
import sys
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_packet as up  # noqa: E402
from nav_cmd_udp import FALLBACK, pick_dest  # noqa: E402


def judge(rep, now, t_rep, base, fallback, lo=-5.0, hi=650.0, timeout=1.5):
    """마지막 답 (decode_lift 결과, 받은 시각) → (cam_z, 이유 또는 None). 이유가 있으면 fallback 을 쓴 것."""
    if rep is None or now - t_rep > timeout:
        return fallback, "라파 lift_udp 답 없음"
    _, h, flags, age = rep
    if not flags & up.LIFT_HAVE:
        return fallback, "라파에 /lift_state 없음"
    if age > 1.0:
        return fallback, "/lift_state 1 s 넘게 안 옴"
    if not flags & up.LIFT_CONNECTED:
        return fallback, "Pico 끊김"
    if not flags & up.LIFT_HOMED or not math.isfinite(h):
        return fallback, "원점 미등록"
    if not lo <= h <= hi:
        return fallback, f"이상한 높이 {h:.0f} mm"
    return base + max(h, 0.0) / 1000.0, None


class LiftCamZ(Node):
    def __init__(self, a, dest):
        super().__init__("lift_cam_z")
        self.a, self.dest = a, dest
        self.cli = self.create_client(SetParameters, f"{a.node}/set_parameters")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.seq, self.rep, self.t_rep = 0, None, -1e9
        self.sent_z, self.sent_why, self.t0, self.t_sent, self.shown = None, "", time.monotonic(), 0.0, None
        self.pending = None
        self.create_timer(1.0 / a.rate, self.tick)

    def tick(self):
        now = time.monotonic()
        while True:
            try:
                data, _ = self.sock.recvfrom(64)
            except (BlockingIOError, OSError):
                break
            r = up.decode_lift(data)
            if r is not None:
                self.rep, self.t_rep = r, now
        self.seq += 1
        try:
            self.sock.sendto(up.encode_lift_req(self.seq), self.dest)
        except OSError:
            pass
        if now - self.t0 < 1.0:          # 켠 직후 첫 답을 잠깐 기다림
            return
        z, why = judge(self.rep, now, self.t_rep, self.a.base, self.a.fallback)
        same = self.sent_z is not None and abs(z - self.sent_z) < self.a.min_step and (why or "") == self.sent_why
        if same and now - self.t_sent < 5.0:      # 5 s 마다는 다시 넣음 (zed_base_tf 를 다시 켰을 때)
            return
        if self.pending is not None and not self.pending.done():
            return
        if not self.cli.service_is_ready():
            if self.sent_why != "zed_base_tf 없음":
                print(f"[lift_cam_z] {self.a.node} 가 아직 없음 — 뜨면 넣음", flush=True)
                self.sent_why = "zed_base_tf 없음"
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name="cam_z", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(z)))]
        self.pending = self.cli.call_async(req)
        self.t_sent = now
        if self.shown is None or self.shown[1] != why or abs(self.shown[0] - z) >= 0.01:   # 1 cm 넘게 바뀌거나 이유가 바뀔 때만 출력
            h = self.rep[1] if self.rep else float("nan")
            print("[lift_cam_z] " + (f"리프트 값 없음 ({why}) → cam_z {z:.3f} (기본)" if why else f"리프트 {h:.0f} mm → cam_z {z:.3f}"), flush=True)
            self.shown = (z, why)
        self.sent_z, self.sent_why = z, why or ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--to", default=os.environ.get("PAD_HOST", "raspberrypi.local"), help="라파 HOST (기본 $PAD_HOST)")
    ap.add_argument("--port", type=int, default=up.LIFT_PORT)
    ap.add_argument("--node", default="/zed_base_tf", help="cam_z 를 넣을 노드")
    ap.add_argument("--base", type=float, default=0.675, help="리프트 0 mm 일 때 바닥→ZED 나사 [m]")
    ap.add_argument("--fallback", type=float, default=1.0, help="값을 못 받거나 이상할 때 cam_z [m]")
    ap.add_argument("--min-step", type=float, default=0.003, help="이만큼 [m] 넘게 바뀔 때만 넣음")
    ap.add_argument("--rate", type=float, default=5.0)
    a, ros_args = ap.parse_known_args()
    ip, name = pick_dest([a.to] + [f for f in FALLBACK if f != a.to], a.port)
    print(f"[lift_cam_z] 라파 {name} ({ip}:{a.port}) 에 높이 물음, cam_z = {a.base} + mm/1000, 못 받으면 {a.fallback}", flush=True)
    rclpy.init(args=ros_args)
    n = LiftCamZ(a, (ip, a.port))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
