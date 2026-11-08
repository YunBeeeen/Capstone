"""ODrive CANSimple (fw 0.5.1) 프레임 정의 + 인코드/디코드.

프레임 ID(11-bit) = can_id << 5 | cmd  (can_id = 보드의 can_node_id).  데이터는 little-endian.
출처: ODrive fw-v0.5.1 docs/can-protocol.md. 0.5.1 은 0x018 Clear_Errors 까지만 있음.
Get_* 은 RTR 요청 → 같은 ID 로 응답 (0.5.1 에는 주기 송신 설정 없음, heartbeat 만 자동).
"""
import struct
from enum import IntEnum, IntFlag


class Cmd(IntEnum):
    HEARTBEAT = 0x001
    ESTOP = 0x002
    GET_MOTOR_ERROR = 0x003
    GET_ENCODER_ERROR = 0x004
    GET_SENSORLESS_ERROR = 0x005
    SET_AXIS_NODE_ID = 0x006
    SET_AXIS_STATE = 0x007
    GET_ENCODER_ESTIMATES = 0x009
    GET_ENCODER_COUNT = 0x00A
    SET_CONTROLLER_MODE = 0x00B
    SET_INPUT_POS = 0x00C
    SET_INPUT_VEL = 0x00D
    SET_INPUT_TORQUE = 0x00E
    SET_VEL_LIMIT = 0x00F          # 0.5.1: 속도 제한만 (전류 제한은 CAN 불가)
    GET_IQ = 0x014
    REBOOT = 0x016
    GET_VBUS = 0x017
    CLEAR_ERRORS = 0x018


class AxisState(IntEnum):
    UNDEFINED = 0                  # "요청 없음" — 끄기 아님!
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    SENSORLESS_CONTROL = 5
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11


STATE_ALIASES = {
    "idle": AxisState.IDLE,
    "closed_loop": AxisState.CLOSED_LOOP_CONTROL,
    "motor_cal": AxisState.MOTOR_CALIBRATION,
    "offset_cal": AxisState.ENCODER_OFFSET_CALIBRATION,
    "full_cal": AxisState.FULL_CALIBRATION_SEQUENCE,
}


class ControlMode(IntEnum):
    VOLTAGE = 0
    TORQUE = 1
    VELOCITY = 2
    POSITION = 3


class InputMode(IntEnum):
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7


# 에러 비트 — fw 0.5.x enum 기준. 디코드가 이상하면 hex 값을 USB dump_errors 로 교차확인.
class AxisError(IntFlag):
    INVALID_STATE = 0x1
    DC_BUS_UNDER_VOLTAGE = 0x2
    DC_BUS_OVER_VOLTAGE = 0x4
    CURRENT_MEASUREMENT_TIMEOUT = 0x8
    BRAKE_RESISTOR_DISARMED = 0x10
    MOTOR_DISARMED = 0x20
    MOTOR_FAILED = 0x40
    SENSORLESS_ESTIMATOR_FAILED = 0x80
    ENCODER_FAILED = 0x100
    CONTROLLER_FAILED = 0x200
    POS_CTRL_DURING_SENSORLESS = 0x400
    WATCHDOG_TIMER_EXPIRED = 0x800
    MIN_ENDSTOP_PRESSED = 0x1000
    MAX_ENDSTOP_PRESSED = 0x2000
    ESTOP_REQUESTED = 0x4000
    HOMING_WITHOUT_ENDSTOP = 0x20000
    OVER_TEMP = 0x40000


class EncoderError(IntFlag):
    UNSTABLE_GAIN = 0x1
    CPR_POLEPAIRS_MISMATCH = 0x2
    NO_RESPONSE = 0x4
    UNSUPPORTED_ENCODER_MODE = 0x8
    ILLEGAL_HALL_STATE = 0x10
    INDEX_NOT_FOUND_YET = 0x20
    ABS_SPI_TIMEOUT = 0x40
    ABS_SPI_COM_FAIL = 0x80
    ABS_SPI_NOT_READY = 0x100


def decode_flags(value: int, flag_type) -> str:
    if value == 0:
        return "OK"
    names = [f.name for f in flag_type if value & f.value]
    unknown = value & ~sum(f.value for f in flag_type)
    if unknown:
        names.append(f"0x{unknown:X}?")
    return f"0x{value:X} " + "|".join(names)


def state_name(state: int) -> str:
    try:
        return AxisState(state).name
    except ValueError:
        return f"STATE_{state}"


# ---------------------------------------------------------------- id
def arb_id(can_id: int, cmd: int) -> int:
    if not 0 <= can_id <= 0x3F:
        raise ValueError(f"can_id 0..63, got {can_id}")
    return (can_id << 5) | (cmd & 0x1F)


def split_id(arb: int) -> tuple[int, int]:
    return arb >> 5, arb & 0x1F


# ---------------------------------------------------------------- encode (Pi → 보드)
def enc_axis_state(state: int) -> bytes:
    return struct.pack("<I", int(state))


def enc_input_vel(vel: float, torque_ff: float = 0.0) -> bytes:
    return struct.pack("<ff", float(vel), float(torque_ff))


def enc_input_torque(torque: float) -> bytes:
    return struct.pack("<f", float(torque))


def enc_controller_mode(control_mode: int, input_mode: int) -> bytes:
    return struct.pack("<ii", int(control_mode), int(input_mode))


def enc_vel_limit(vel_limit: float) -> bytes:
    return struct.pack("<f", float(vel_limit))


# ---------------------------------------------------------------- decode (보드 → Pi)
def dec_heartbeat(data: bytes) -> tuple[int, int]:
    """(axis_error, axis_current_state)"""
    err, state = struct.unpack("<II", bytes(data[:8]).ljust(8, b"\0"))
    return err, state


def dec_encoder_estimates(data: bytes) -> tuple[float, float]:
    """(pos [turn], vel [turn/s])"""
    return struct.unpack("<ff", bytes(data[:8]))


def dec_encoder_count(data: bytes) -> tuple[int, int]:
    """(shadow_count, count_in_cpr) — Hall 모드에선 Hall 엣지 1개 = 1카운트"""
    return struct.unpack("<ii", bytes(data[:8]))


def dec_iq(data: bytes) -> tuple[float, float]:
    """(Iq_setpoint [A], Iq_measured [A])"""
    return struct.unpack("<ff", bytes(data[:8]))


def dec_vbus(data: bytes) -> float:
    return struct.unpack("<f", bytes(data[:4]))[0]


def dec_u32(data: bytes) -> int:
    return struct.unpack("<I", bytes(data[:4]).ljust(4, b"\0"))[0]
