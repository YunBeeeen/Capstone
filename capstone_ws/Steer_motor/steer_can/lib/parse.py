"""수신 프레임 파서: Type02 피드백, Type21 고장."""

from dataclasses import dataclass

from .common import unpack_ext_id, u16_to_float
from .frames import MIT_RS02, TYPE02, TYPE21, TYPE24

MODE_RESET, MODE_CALI, MODE_RUN = 0, 1, 2   # Type02 ID bit23~22

# Type02 ID bit21~16 (매뉴얼 4.1.2)
FAULT_BITS = {
    0: "undervoltage",
    1: "overcurrent",
    2: "overtemp",
    3: "magnetic_encoder",
    4: "stall_overload",
    5: "uncalibrated",
}


@dataclass
class Feedback:
    motor_id: int
    host_id: int
    mode_status: int
    fault_bits: int
    pos: float      # [rad] 출력축. 켤 때 zero_sta=1 이면 −π~π, 이후 ±4π 안에서 회전 수 셈
    vel: float      # [rad/s]
    tor: float      # [Nm]
    temp_c: float


def parse_feedback_like_type2(arb_id: int, data: bytes) -> Feedback | None:
    comm_type, data2, data1 = unpack_ext_id(arb_id)
    if comm_type != TYPE02 or len(data) != 8:
        return None
    r = MIT_RS02
    flags = (data2 >> 8) & 0xFF
    return Feedback(
        motor_id=data2 & 0xFF,
        host_id=data1,
        mode_status=(flags >> 6) & 0x03,
        fault_bits=flags & 0x3F,
        pos=u16_to_float(int.from_bytes(data[0:2], "big"), r.P_MIN, r.P_MAX),
        vel=u16_to_float(int.from_bytes(data[2:4], "big"), r.V_MIN, r.V_MAX),
        tor=u16_to_float(int.from_bytes(data[4:6], "big"), r.T_MIN, r.T_MAX),
        temp_c=int.from_bytes(data[6:8], "big") / 10.0,
    )


def parse_feedback_any(arb_id: int, data: bytes) -> Feedback | None:
    """Type02 응답 + Type24(0x18) active report 둘 다 → Feedback.
    RS02 의 active report 는 통신 타입만 0x18 이고 ID 비트(mode·fault·motor_id·host)와 데이터는 Type02 와 같음
    (2026-11-08 실기 candump: 180001FD [8] 9B FE 80 14 7F FF 00 FA). idle_ws monitor.py 와 같은 처리."""
    comm_type, _, _ = unpack_ext_id(arb_id)
    if comm_type == TYPE24:
        arb_id = (arb_id & ~(0x1F << 24)) | (TYPE02 << 24)
    return parse_feedback_like_type2(arb_id, data)


def parse_type21_fault(arb_id: int, data: bytes) -> tuple[int, int, int] | None:
    """Type21 → (motor_id, fault_u32, warning_u32). 바이트 순서 LE (매뉴얼에 명시 없음 — 실기 확인 필요)."""
    comm_type, data2, _ = unpack_ext_id(arb_id)
    if comm_type != TYPE21 or len(data) != 8:
        return None
    return data2 & 0xFF, int.from_bytes(data[0:4], "little"), int.from_bytes(data[4:8], "little")


def fault_names(bits: int) -> str:
    names = [n for b, n in FAULT_BITS.items() if bits & (1 << b)]
    return ",".join(names) if names else "OK"
