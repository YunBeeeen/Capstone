"""RS02 private 프로토콜 프레임 생성 (매뉴얼 4.1). idle_ws/motor/lib/frames.py 의 RS02 부분.

확장 29-bit ID = [28:24] 통신 타입 | [23:8] data2 | [7:0] data1
Type 1·3·4·18 의 응답은 전부 Type 2 피드백 → 토크 끈 상태(Type 4)에서도 위치를 읽을 수 있음.
"""

import math
import struct
from dataclasses import dataclass

from .common import pack_ext_id, float_to_u16

TYPE00_GET_DEVICE_ID      = 0x00
TYPE01_OPERATION_CONTROL  = 0x01  # MIT
TYPE02_FEEDBACK           = 0x02
TYPE03_ENABLE             = 0x03
TYPE04_STOP               = 0x04
TYPE06_SET_ZERO           = 0x06
TYPE07_SET_CAN_ID         = 0x07
TYPE17_PARAM_READ         = 0x11
TYPE18_PARAM_WRITE        = 0x12
TYPE21_FAULT_FEEDBACK     = 0x15
TYPE22_DATA_SAVE          = 0x16
TYPE24_ACTIVE_REPORT      = 0x18

TYPE01 = TYPE01_OPERATION_CONTROL
TYPE02 = TYPE02_FEEDBACK
TYPE03 = TYPE03_ENABLE
TYPE04 = TYPE04_STOP
TYPE06 = TYPE06_SET_ZERO
TYPE17 = TYPE17_PARAM_READ
TYPE18 = TYPE18_PARAM_WRITE
TYPE21 = TYPE21_FAULT_FEEDBACK
TYPE22 = TYPE22_DATA_SAVE
TYPE24 = TYPE24_ACTIVE_REPORT


@dataclass(frozen=True)
class MitRanges:
    P_MIN: float
    P_MAX: float
    V_MIN: float
    V_MAX: float
    T_MIN: float
    T_MAX: float
    KP_MIN: float
    KP_MAX: float
    KD_MIN: float
    KD_MAX: float


# 매뉴얼 4.1.2 (Type 2 위치 ±4π = 회전 수 셈)
MIT_RS02 = MitRanges(
    P_MIN=-4.0 * math.pi, P_MAX=4.0 * math.pi,
    V_MIN=-44.0,          V_MAX=44.0,
    T_MIN=-17.0,          T_MAX=17.0,
    KP_MIN=0.0,           KP_MAX=500.0,
    KD_MIN=0.0,           KD_MAX=5.0,
)


def frame_type03_enable(host_id: int, motor_id: int) -> tuple[int, bytes]:
    return pack_ext_id(TYPE03, data2=host_id, data1=motor_id), bytes(8)


def frame_type04_stop(host_id: int, motor_id: int, clear_fault: bool = False) -> tuple[int, bytes]:
    data = bytearray(8)
    data[0] = 1 if clear_fault else 0
    return pack_ext_id(TYPE04, data2=host_id, data1=motor_id), bytes(data)


def frame_type06_zero(host_id: int, motor_id: int) -> tuple[int, bytes]:
    data = bytearray(8)
    data[0] = 1
    return pack_ext_id(TYPE06, data2=host_id, data1=motor_id), bytes(data)


def frame_type24_active_report(host_id: int, motor_id: int, enable: bool) -> tuple[int, bytes]:
    data = bytes([1, 2, 3, 4, 5, 6, 1 if enable else 0, 0])
    return pack_ext_id(TYPE24, data2=host_id, data1=motor_id), data


TYPE00_REPLY_MARK = 0xFE  # Type00 응답의 ID bit7~0 고정값


def frame_type00_get_id(host_id: int, motor_id: int) -> tuple[int, bytes]:
    """Type00 장치 ID 요청. 응답: ID[15:8]=모터 CAN_ID, ID[7:0]=0xFE, data=MCU UID(8B)."""
    return pack_ext_id(TYPE00_GET_DEVICE_ID, data2=host_id, data1=motor_id), bytes(8)


def frame_type07_set_can_id(host_id: int, motor_id: int, new_id: int) -> tuple[int, bytes]:
    """Type07: ID[23:16]=새 CAN_ID, ID[15:8]=host_id, ID[7:0]=현재 CAN_ID. 성공 시 새 ID 로 Type00 응답."""
    data2 = ((int(new_id) & 0xFF) << 8) | (int(host_id) & 0xFF)
    return pack_ext_id(TYPE07_SET_CAN_ID, data2=data2, data1=motor_id), bytes(8)


def frame_type22_save(host_id: int, motor_id: int) -> tuple[int, bytes]:
    return pack_ext_id(TYPE22, data2=host_id, data1=motor_id), bytes([1, 2, 3, 4, 5, 6, 7, 8])


def frame_type17_read(host_id: int, motor_id: int, index: int) -> tuple[int, bytes]:
    """Type17: data[0:2]=index (LE), 나머지 0."""
    data = bytearray(8)
    data[0:2] = int(index).to_bytes(2, "little", signed=False)
    return pack_ext_id(TYPE17, data2=host_id, data1=motor_id), bytes(data)


def frame_type18_write_u32(host_id: int, motor_id: int, index: int, value_u32: int) -> tuple[int, bytes]:
    """Type18: data[0:2]=index (LE), data[2:4]=0, data[4:8]=값 (LE). RAM 만 — 저장은 Type22."""
    data = bytearray(8)
    data[0:2] = int(index).to_bytes(2, "little", signed=False)
    data[4:8] = int(value_u32 & 0xFFFFFFFF).to_bytes(4, "little", signed=False)
    return pack_ext_id(TYPE18, data2=host_id, data1=motor_id), bytes(data)


def frame_type18_write_f32(host_id: int, motor_id: int, index: int, value_f32: float) -> tuple[int, bytes]:
    u32 = int.from_bytes(struct.pack("<f", float(value_f32)), "little", signed=False)
    return frame_type18_write_u32(host_id, motor_id, index, u32)


def frame_type01_mit(motor_id: int, p: float, v: float, kp: float, kd: float, t: float) -> tuple[int, bytes]:
    """Type01 (MIT): t_ref = kp·(p − p_actual) + kd·(v − v_actual) + t  — 모터 안에서 계산.

    ID.data2 = 토크 u16, data = pos | vel | kp | kd (각 u16 big-endian).
    """
    r = MIT_RS02
    arb_id = pack_ext_id(TYPE01, data2=float_to_u16(t, r.T_MIN, r.T_MAX), data1=motor_id)
    data = (
        float_to_u16(p, r.P_MIN, r.P_MAX).to_bytes(2, "big")
        + float_to_u16(v, r.V_MIN, r.V_MAX).to_bytes(2, "big")
        + float_to_u16(kp, r.KP_MIN, r.KP_MAX).to_bytes(2, "big")
        + float_to_u16(kd, r.KD_MIN, r.KD_MAX).to_bytes(2, "big")
    )
    return arb_id, data
