"""가짜 RS02 ×N (vcan) — 하드웨어 없이 steer_can_bridge / step.py / angle.py 시험.

    sudo modprobe vcan
    sudo ip link add dev vcan1 type vcan && sudo ip link set vcan1 up
    cd ~/capstone/capstone_ws/Steer_motor/steer_can && python3 -m tests.fake_rs02 --ch vcan1 [--start 0 30 -175 0]
    ros2 launch steer_can_bridge steer_can_bridge.launch.py can_interface:=vcan1

흉내 내는 것: Type00(UID) / Type03 enable → RUN / Type04 stop → RESET(+clear) / Type01 MIT (1차 지연으로
p 를 따라감, 토크 = kp·Δp + kd·Δv 를 ±17 로 자름) / Type18 / Type17(0 응답). 응답은 전부 Type02.
canTimeout(0x7028) 을 쓰면 그 시간 동안 프레임이 없을 때 RESET 으로 떨어짐.
--start 는 켤 때 출력축 각도 [deg] — −180~180 로 접어서 보고 (zero_sta=1 흉내, 접힘 시험용).
"""

import argparse
import math
import time

from lib.common import float_to_u16, pack_ext_id, unpack_ext_id, u16_to_float
from lib.frames import (MIT_RS02, TYPE00_GET_DEVICE_ID, TYPE00_REPLY_MARK, TYPE01, TYPE02, TYPE03, TYPE04,
                        TYPE17, TYPE18)
from lib.parse import MODE_RESET, MODE_RUN

R = MIT_RS02


class FakeMotor:
    def __init__(self, can_id: int, start_deg: float = 0.0, tau: float = 0.05):
        self.id = can_id
        a = math.radians(start_deg)
        self.pos = math.atan2(math.sin(a), math.cos(a))  # 켤 때 −π~π 로 접힘
        self.vel = 0.0
        self.tor = 0.0
        self.mode = MODE_RESET
        self.fault = 0
        self.tau = tau
        self.p_cmd = self.pos
        self.can_timeout = 0.0
        self.last_rx = time.monotonic()

    def feedback(self, host: int) -> tuple[int, bytes]:
        flags = (self.mode << 6) | self.fault
        arb = pack_ext_id(TYPE02, data2=(flags << 8) | self.id, data1=host)
        data = (float_to_u16(self.pos, R.P_MIN, R.P_MAX).to_bytes(2, "big")
                + float_to_u16(self.vel, R.V_MIN, R.V_MAX).to_bytes(2, "big")
                + float_to_u16(self.tor, R.T_MIN, R.T_MAX).to_bytes(2, "big")
                + (300).to_bytes(2, "big"))
        return arb, data

    def handle(self, arb: int, data: bytes, now: float) -> list[tuple[int, bytes]]:
        typ, d2, d1 = unpack_ext_id(arb)
        if d1 != self.id:
            return []
        self.last_rx = now
        host = d2 & 0xFF
        if typ == TYPE00_GET_DEVICE_ID:
            return [(pack_ext_id(TYPE00_GET_DEVICE_ID, data2=self.id, data1=TYPE00_REPLY_MARK),
                     bytes([0xFA, 0x4E, 0, 0, 0, 0, 0, self.id]))]
        if typ == TYPE03:
            self.mode = MODE_RUN
            self.p_cmd = self.pos
        elif typ == TYPE04:
            self.mode = MODE_RESET
            self.tor = 0.0
            if data[0] == 1:
                self.fault = 0
        elif typ == TYPE01:
            p = u16_to_float(int.from_bytes(data[0:2], "big"), R.P_MIN, R.P_MAX)
            v = u16_to_float(int.from_bytes(data[2:4], "big"), R.V_MIN, R.V_MAX)
            kp = u16_to_float(int.from_bytes(data[4:6], "big"), R.KP_MIN, R.KP_MAX)
            kd = u16_to_float(int.from_bytes(data[6:8], "big"), R.KD_MIN, R.KD_MAX)
            if self.mode == MODE_RUN:
                self.p_cmd = p
                self.tor = max(R.T_MIN, min(R.T_MAX, kp * (p - self.pos) + kd * (v - self.vel)))
            host = 0xFD  # Type01 엔 호스트 ID 가 없음
        elif typ == TYPE18:
            index = int.from_bytes(data[0:2], "little")
            if index == 0x7028:
                self.can_timeout = int.from_bytes(data[4:8], "little") / 20000.0
        elif typ == TYPE17:
            return [(pack_ext_id(TYPE17, data2=self.id, data1=host), bytes(data[0:4]) + bytes(4))]
        else:
            return []
        return [self.feedback(host)]

    def tick(self, dt: float, now: float):
        if self.mode == MODE_RUN and self.can_timeout > 0 and now - self.last_rx > self.can_timeout:
            self.mode = MODE_RESET
        if self.mode == MODE_RUN:
            new = self.pos + (self.p_cmd - self.pos) * min(1.0, dt / self.tau)
            self.vel = (new - self.pos) / dt if dt > 0 else 0.0
            self.pos = new
        else:
            self.vel = 0.0


def main():
    import can

    ap = argparse.ArgumentParser(description="가짜 RS02 (vcan)")
    ap.add_argument("--ch", default="vcan1")
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--start", type=float, nargs="+", default=None, help="켤 때 각도 [deg], ids 순서")
    args = ap.parse_args()
    start = args.start or [0.0] * len(args.ids)
    motors = {i: FakeMotor(i, s) for i, s in zip(args.ids, start)}
    bus = can.interface.Bus(channel=args.ch, interface="socketcan")
    print(f"fake RS02 {args.ids} on {args.ch} — 시작각 {start}")
    last = time.monotonic()
    try:
        while True:
            msg = bus.recv(timeout=0.001)
            now = time.monotonic()
            for m in motors.values():
                m.tick(now - last, now)
            last = now
            if msg is None or not msg.is_extended_id:
                continue
            for m in motors.values():
                for arb, data in m.handle(msg.arbitration_id, bytes(msg.data), now):
                    bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=True))
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
