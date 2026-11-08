"""socketcan 버스 래퍼: CANSimple 송신, RTR 요청/응답, heartbeat 캐시."""
import time
from dataclasses import dataclass

import can

from . import cansimple as cs


@dataclass
class Heartbeat:
    t: float          # time.monotonic() 수신 시각 (PC 기준)
    error: int
    state: int


class OdriveBus:
    def __init__(self, channel: str = "can_drive", interface: str = "socketcan"):
        self.bus = can.Bus(channel=channel, interface=interface)
        self.heartbeats: dict[int, Heartbeat] = {}
        self._closed = False

    # ---------------- 송신
    def send(self, can_id: int, cmd: int, data: bytes = b"") -> int:
        aid = cs.arb_id(can_id, cmd)
        self.bus.send(can.Message(arbitration_id=aid, data=data, is_extended_id=False))
        return aid

    def set_state(self, can_id: int, state: int):
        return self.send(can_id, cs.Cmd.SET_AXIS_STATE, cs.enc_axis_state(state))

    def set_vel(self, can_id: int, vel: float, torque_ff: float = 0.0):
        return self.send(can_id, cs.Cmd.SET_INPUT_VEL, cs.enc_input_vel(vel, torque_ff))

    def set_controller_mode(self, can_id: int, control_mode: int, input_mode: int):
        return self.send(can_id, cs.Cmd.SET_CONTROLLER_MODE, cs.enc_controller_mode(control_mode, input_mode))

    def clear_errors(self, can_id: int):
        return self.send(can_id, cs.Cmd.CLEAR_ERRORS)

    # ---------------- 수신
    def _observe(self, msg) -> None:
        if msg.is_extended_id or msg.is_remote_frame or msg.is_error_frame:
            return
        can_id, cmd = cs.split_id(msg.arbitration_id)
        if cmd == cs.Cmd.HEARTBEAT and len(msg.data) >= 5:
            err, state = cs.dec_heartbeat(msg.data)
            self.heartbeats[can_id] = Heartbeat(time.monotonic(), err, state)

    def pump(self, duration: float = 0.0) -> None:
        """duration 동안(0이면 쌓인 것만) 수신해서 heartbeat 캐시 갱신."""
        deadline = time.monotonic() + duration
        while True:
            rem = deadline - time.monotonic()
            msg = self.bus.recv(timeout=max(rem, 0.0))
            if msg is None:
                if rem <= 0:
                    return
                continue
            self._observe(msg)

    def request(self, can_id: int, cmd: int, timeout: float = 0.05) -> bytes | None:
        """RTR 요청 → 같은 ID 응답 데이터. 시간 안에 없으면 None."""
        aid = cs.arb_id(can_id, cmd)
        self.bus.send(can.Message(arbitration_id=aid, is_remote_frame=True, dlc=8, is_extended_id=False))
        deadline = time.monotonic() + timeout
        while (rem := deadline - time.monotonic()) > 0:
            msg = self.bus.recv(timeout=rem)
            if msg is None:
                break
            self._observe(msg)
            if msg.arbitration_id == aid and not msg.is_remote_frame and not msg.is_extended_id:
                return bytes(msg.data)
        return None

    def heartbeat(self, can_id: int, max_age: float) -> Heartbeat | None:
        hb = self.heartbeats.get(can_id)
        if hb is None or time.monotonic() - hb.t > max_age:
            return None
        return hb

    def wait_heartbeat(self, can_id: int, timeout: float = 0.5, newer_than: float = 0.0) -> Heartbeat | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(0.02)
            hb = self.heartbeats.get(can_id)
            if hb is not None and hb.t > newer_than:
                return hb
        return None

    # ---------------- 편의 Get
    def get_encoder(self, can_id: int):
        d = self.request(can_id, cs.Cmd.GET_ENCODER_ESTIMATES)
        return None if d is None else cs.dec_encoder_estimates(d)

    def get_count(self, can_id: int):
        d = self.request(can_id, cs.Cmd.GET_ENCODER_COUNT)
        return None if d is None else cs.dec_encoder_count(d)

    def get_iq(self, can_id: int):
        d = self.request(can_id, cs.Cmd.GET_IQ)
        return None if d is None else cs.dec_iq(d)

    def get_vbus(self, can_id: int):
        d = self.request(can_id, cs.Cmd.GET_VBUS)
        return None if d is None else cs.dec_vbus(d)

    def get_motor_error(self, can_id: int):
        d = self.request(can_id, cs.Cmd.GET_MOTOR_ERROR)
        return None if d is None else cs.dec_u32(d)

    def get_encoder_error(self, can_id: int):
        d = self.request(can_id, cs.Cmd.GET_ENCODER_ERROR)
        return None if d is None else cs.dec_u32(d)

    # ---------------- 종료
    def close(self):
        if not self._closed:
            self._closed = True
            self.bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
