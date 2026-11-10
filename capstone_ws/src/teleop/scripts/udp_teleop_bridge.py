#!/usr/bin/env python3
"""노트북 가상 패드(UDP) → 라파 /cmd_vel. **라파에서 실행.** (ksrc pi/common/udp_spi_bridge.py 와 같은 구조)

  라파:   ros2 run capstone_teleop udp_teleop_bridge.py
  노트북: python3 virtual_pad.py --udp raspberrypi.local --max-lin 0.15 --max-ang 0.5   (ROS 필요 없음, pygame 만)

왜 UDP: 노트북엔 ROS 가 없어도 되고, 노트북(Humble)·라파(Jazzy) 섞임이나 와이파이에서 DDS 탐색이 불안정한 문제를 피함.
라파는 화면이 없어 pygame 창을 못 띄움 (ssh X 포워딩은 SDL MIT-SHM 오류 — ksrc 에서 겪음).

패킷 (노트북 → 라파): udp_packet.py — v1 20 B (vx, vy, ω) / v2 22 B (+ 리프트 조그·버튼)
응답 (라파 → 노트북, 5 Hz): JSON 한 줄 {"cmd":[vx,vy,w],"stale":bool,"steer":"ACTIVE ready","wheel":"ACTIVE","fault":"..","vbus":25.1,
                                    "lift":{"st":"IDLE","h":123.4,"rel":0.0,"task":"","parked":false,"fault":"","msg":".."}}
리프트: 패킷의 리프트 입력 → /lift_cmd (capstone_msgs/LiftCommand) → 라파 lift_bridge_node.py 가 Pico 로

안전:
  - 패킷이 STALE_S(0.3 s) 넘게 안 오면 즉시 0 을 냄 (창 닫힘, 와이파이 끊김). 0 은 1 s 동안 내다가 발행 멈춤
    → swerve_controller 가 cmd_timeout 뒤 HOLD
  - 받은 값은 --max-lin / --max-ang 로 다시 클램프. 길이·매직·유한값 검사, 순서 뒤집힌 오래된 패킷 버림
"""
import argparse
import json
import math
import socket
import time

import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import udp_packet as up  # noqa: E402

STALE_S = 0.3
ZERO_HOLD_S = 1.0  # 끊긴 뒤 0 을 내는 시간

STEER_PHASE = {0: "FREE", 1: "ENTERING", 2: "ACTIVE", 3: "FAULT"}
WHEEL_PHASE = {0: "FREE", 1: "ENTERING", 2: "ACTIVE", 3: "FAULT"}


class UdpTeleopBridge(Node):
    def __init__(self, a):
        super().__init__("udp_teleop_bridge")
        self.a = a
        self.pub = self.create_publisher(Twist, a.topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", a.port))
        self.sock.setblocking(False)
        self.cmd = (0.0, 0.0, 0.0)
        self.lift_in = (0, 0)  # (jog, flags)
        self.lift = None       # 마지막 LiftState
        self.lift_pub = None
        self.last_rx = -1e9
        self.last_seq = None
        self.peer = None
        self.n_rx = self.n_bad = self.n_old = 0
        self.steer = self.wheel = None
        self._subscribe_states()
        self.create_timer(1.0 / a.hz, self.tick)
        self.create_timer(0.2, self.reply)
        self.create_timer(2.0, self.status)
        self.get_logger().info(f"UDP {a.port} 대기 → {a.topic} {a.hz:.0f} Hz (상한 {a.max_lin} m/s, {a.max_ang} rad/s)")

    def _subscribe_states(self):
        # 상태 메시지는 있으면 응답에 실음 (capstone_msgs 없으면 생략)
        try:
            from capstone_msgs.msg import SteerStateArray, WheelStateArray
            self.create_subscription(SteerStateArray, "steer_state", lambda m: setattr(self, "steer", m), 10)
            self.create_subscription(WheelStateArray, "wheel_state", lambda m: setattr(self, "wheel", m), 10)
        except ImportError:
            pass
        try:
            from capstone_msgs.msg import LiftCommand, LiftState
            self.LiftCommand = LiftCommand
            self.lift_pub = self.create_publisher(LiftCommand, "lift_cmd", 10)
            self.create_subscription(LiftState, "lift_state", lambda m: setattr(self, "lift", m), 10)
        except ImportError:
            self.get_logger().warn("capstone_msgs 에 LiftCommand 없음 — 리프트 입력 무시 (capstone_msgs 다시 빌드)")

    def _drain(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(64)
            except (BlockingIOError, OSError):
                return
            p = up.decode(data)
            if p is None:
                self.n_bad += 1
                continue
            seq, vx, vy, w, jog, flags = p
            # 오래된 패킷(순서 뒤집힘) 버림. 노트북 재시작(seq 가 작게 다시 시작)은 허용
            if self.last_seq is not None and addr == self.peer and 0 < (self.last_seq - seq) % 2**32 < 2**31:
                self.n_old += 1
                continue
            if addr != self.peer:
                self.get_logger().info(f"패드 연결: {addr[0]}:{addr[1]}")
            self.last_seq, self.peer = seq, addr
            lim = lambda v, m: max(-m, min(m, v))
            self.cmd = (lim(vx, self.a.max_lin), lim(vy, self.a.max_lin), lim(w, self.a.max_ang))
            self.lift_in = (jog, flags)
            self.last_rx = time.monotonic()
            self.n_rx += 1

    def tick(self):
        self._drain()
        age = time.monotonic() - self.last_rx
        if age <= STALE_S:
            vx, vy, w = self.cmd
            jog, flags = self.lift_in
        elif age <= STALE_S + ZERO_HOLD_S:
            vx = vy = w = 0.0  # 끊김 → 즉시 정지 명령
            jog, flags = 0, 0
        else:
            return  # 발행 멈춤 → swerve 가 HOLD, lift_bridge 는 cmd_timeout 뒤 조그 정지
        if self.lift_pub is not None:
            lc = self.LiftCommand()
            lc.header.stamp = self.get_clock().now().to_msg()
            lc.jog = int(jog)
            lc.stop, lc.park = bool(flags & up.LIFT_STOP), bool(flags & up.LIFT_PARK)
            lc.zero, lc.clear = bool(flags & up.LIFT_ZERO), bool(flags & up.LIFT_CLEAR)
            self.lift_pub.publish(lc)
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(w)
        self.pub.publish(t)

    def reply(self):
        if self.peer is None:
            return
        stale = time.monotonic() - self.last_rx > STALE_S
        r = {"cmd": [round(v, 3) for v in self.cmd], "stale": stale}
        s, wh = self.steer, self.wheel
        if s is not None:
            r["steer"] = STEER_PHASE.get(s.phase, str(s.phase)) + (" ready" if s.ready else " NOT-ready")
            r["steer_deg"] = [round(math.degrees(a)) for a in s.angle]
            if s.fault:
                r["fault"] = "조향: " + s.fault
        if wh is not None:
            r["wheel"] = WHEEL_PHASE.get(wh.phase, str(wh.phase))
            r["vbus"] = round(wh.vbus, 1)
            if wh.fault:
                r["fault"] = (r.get("fault", "") + " 구동: " + wh.fault).strip()
        lf = self.lift
        if lf is not None:
            r["lift"] = {"conn": lf.connected, "st": lf.pico_state, "armed": lf.armed, "homed": lf.homed,
                         "h": None if math.isnan(lf.height_mm) else round(lf.height_mm, 1),
                         "rel": round(lf.rel_mm, 1), "task": lf.task, "parked": lf.parked,
                         "fault": lf.fault, "msg": lf.message}
        try:
            self.sock.sendto(json.dumps(r, ensure_ascii=False).encode(), self.peer)
        except OSError:
            pass

    def status(self):
        age = time.monotonic() - self.last_rx
        src = "대기(패드 없음)" if self.peer is None else ("끊김" if age > STALE_S else self.peer[0])
        self.get_logger().info(f"{src} | 수신 {self.n_rx} 버림(형식 {self.n_bad}, 순서 {self.n_old}) | cmd {self.cmd}",
                               throttle_duration_sec=2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--max-lin", type=float, default=0.6, help="받은 값 상한 [m/s] (swerve max_wheel_mps)")
    ap.add_argument("--max-ang", type=float, default=2.0, help="받은 값 상한 [rad/s]")
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = UdpTeleopBridge(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
