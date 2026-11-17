#!/usr/bin/env python3
"""리프트 높이를 Jetson 에 알려 줌 — **라파에서 실행** (lift_bridge 의 /lift_state 를 받아, Jetson 이 물으면 답함).

  source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
  python3 ~/pad/scripts/lift_udp.py

Jetson lift_cam_z.py 가 UDP 5007 로 b"LREQ" 를 보내면 같은 주소로 높이 (b"LIFT") 를 돌려줌 → Jetson 은 카메라 높이 (cam_z) 를
리프트에 맞춰 바꿈. 이 프로그램이 꺼져 있거나 /lift_state 가 안 오면 Jetson 은 cam_z 1.0 m 로 둠.
Jetson 주소는 몰라도 됨 (물어 온 곳으로 답함). 2 초마다 지금 높이·물어 온 곳을 한 줄로 보여 줌.
"""
import argparse
import math
import socket
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_packet as up  # noqa: E402


class LiftUdp(Node):
    def __init__(self, port):
        super().__init__("lift_udp")
        from capstone_msgs.msg import LiftState
        self.st, self.t = None, None
        self.create_subscription(LiftState, "lift_state", self.on_state, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.peer, self.n = None, 0
        self.create_timer(0.02, self.poll)
        self.create_timer(2.0, self.show)
        print(f"[lift_udp] UDP {port} 에서 Jetson 질문 기다림, /lift_state 받는 중", flush=True)

    def on_state(self, m):
        self.st, self.t = m, time.monotonic()

    def reply(self):
        st = self.st
        if st is None:
            return 0, float("nan"), None
        flags = up.LIFT_HAVE | (up.LIFT_CONNECTED if st.connected else 0) | (up.LIFT_HOMED if st.homed else 0)
        return flags, st.height_mm, time.monotonic() - self.t

    def poll(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(64)
            except (BlockingIOError, OSError):
                return
            seq = up.decode_lift_req(data)
            if seq is None:
                continue
            flags, h, age = self.reply()
            try:
                self.sock.sendto(up.encode_lift(seq, h, flags, age), addr)
            except OSError:
                continue
            if addr[0] != (self.peer or ("",))[0]:
                print(f"[lift_udp] Jetson {addr[0]} 이 물어 옴", flush=True)
            self.peer, self.n = addr, self.n + 1

    def show(self):
        flags, h, age = self.reply()
        if not flags:
            what = "/lift_state 아직 없음 (R5 lift_bridge 켜졌나?) → Jetson 은 cam_z 1.0"
        elif age > 1.0:
            what = f"/lift_state {age:.0f} s 째 안 옴 → Jetson 은 cam_z 1.0"
        elif not flags & up.LIFT_HOMED or not math.isfinite(h):
            what = "원점 미등록 (높이 모름) → Jetson 은 cam_z 1.0"
        else:
            what = f"높이 {h:.0f} mm"
        print(f"[lift_udp] {what} | 답한 횟수 {self.n} (2 s)", flush=True)
        self.n = 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=up.LIFT_PORT)
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    n = LiftUdp(a.port)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
