"""python-can 가상 버스 위의 가짜 XDrive (fw 0.5.1 CANSimple 흉내) — 하드웨어 없이 스크립트 검증용.

단독 실행하면 vcan 위에서 보드 2장(id 1~4)처럼 동작 → C++ 브리지 노드를 하드웨어 없이 시험:
    sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
    cd ~/capstone/capstone_ws/Wheel_motor/wheel_can && python -m tests.fake_odrive --ch vcan0
"""
import struct
import threading
import time

import can

from lib import cansimple as cs


class FakeAxis:
    def __init__(self):
        self.state = cs.AxisState.IDLE
        self.error = 0
        self.vel_cmd = 0.0
        self.vel_history = []
        self.pos = 0.0
        self.count = 0


class FakeOdrive(threading.Thread):
    def __init__(self, channel: str, can_ids=(1, 2), hb_period=0.05, interface="virtual"):
        super().__init__(daemon=True)
        self.bus = can.Bus(interface=interface, channel=channel)
        self.axes = {n: FakeAxis() for n in can_ids}
        self.hb_period = hb_period
        self._halt = threading.Event()

    def stop(self):
        self._halt.set()
        self.join(1)
        self.bus.shutdown()

    def _send(self, can_id, cmd, data):
        self.bus.send(can.Message(arbitration_id=cs.arb_id(can_id, cmd), data=data, is_extended_id=False))

    def run(self):
        next_hb = 0.0
        last = time.monotonic()
        while not self._halt.is_set():
            now = time.monotonic()
            dt, last = now - last, now
            for ax in self.axes.values():  # 폐루프면 명령 속도로 즉시 회전한다고 가정
                v = ax.vel_cmd if ax.state == cs.AxisState.CLOSED_LOOP_CONTROL else 0.0
                ax.pos += v * dt
                ax.count = int(round(ax.pos * 60))
            if now >= next_hb:
                for n, ax in self.axes.items():
                    self._send(n, cs.Cmd.HEARTBEAT, struct.pack("<II", ax.error, ax.state))
                next_hb = now + self.hb_period
            msg = self.bus.recv(timeout=0.005)
            if msg is None:
                continue
            can_id, cmd = cs.split_id(msg.arbitration_id)
            ax = self.axes.get(can_id)
            if ax is None:
                continue
            if msg.is_remote_frame:
                if cmd == cs.Cmd.GET_ENCODER_ESTIMATES:
                    v = ax.vel_cmd if ax.state == cs.AxisState.CLOSED_LOOP_CONTROL else 0.0
                    self._send(can_id, cmd, struct.pack("<ff", ax.pos, v))
                elif cmd == cs.Cmd.GET_ENCODER_COUNT:
                    self._send(can_id, cmd, struct.pack("<ii", ax.count, ax.count % 60))
                elif cmd == cs.Cmd.GET_IQ:
                    self._send(can_id, cmd, struct.pack("<ff", 0.2, 0.21))
                elif cmd == cs.Cmd.GET_VBUS:
                    self._send(can_id, cmd, struct.pack("<f", 24.75) + b"\0\0\0\0")
                elif cmd in (cs.Cmd.GET_MOTOR_ERROR, cs.Cmd.GET_ENCODER_ERROR):
                    self._send(can_id, cmd, b"\0" * 8)
                continue
            if cmd == cs.Cmd.SET_AXIS_STATE:
                req = struct.unpack("<I", bytes(msg.data[:4]))[0]
                if req != 0:
                    ax.state = cs.AxisState(req)
            elif cmd == cs.Cmd.SET_INPUT_VEL:
                ax.vel_cmd = struct.unpack("<f", bytes(msg.data[:4]))[0]
                ax.vel_history.append(ax.vel_cmd)
            elif cmd == cs.Cmd.CLEAR_ERRORS:
                ax.error = 0
            elif cmd == cs.Cmd.ESTOP:
                ax.error |= cs.AxisError.ESTOP_REQUESTED
                ax.state = cs.AxisState.IDLE


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="vcan 위 가짜 XDrive")
    ap.add_argument("--ch", default="vcan0")
    ap.add_argument("--can-id", type=int, nargs="+", default=[1, 2, 3, 4])
    args = ap.parse_args()
    fake = FakeOdrive(args.ch, can_ids=tuple(args.can_id), hb_period=0.1, interface="socketcan")
    fake.start()
    print(f"가짜 XDrive {args.can_id} on {args.ch} (Ctrl+C 종료)")
    try:
        while True:
            time.sleep(1.0)
            print("  " + "  ".join(f"id{n} {cs.state_name(a.state)} v={a.vel_cmd:+.2f} pos={a.pos:+.2f}"
                                   for n, a in fake.axes.items()), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        fake.stop()
