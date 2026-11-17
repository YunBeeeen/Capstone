"""노트북 패드 → 라파 udp_teleop_bridge UDP 패킷 (양쪽이 같이 씀).

v1 (20 B): b"CAPS" + <I seq> + <f vx> <f vy> <f omega>
v2 (22 B): v1 + <b lift_pwm> (리프트 조그 PWM %, + 위 / − 아래 / 0 정지, -100..100) + <B lift_flags> (누르고 있는 동안 1)
라파는 둘 다 받음 (v1 = 리프트 입력 없음).

Nav2 중계 (Jetson nav_cmd_udp.py → 라파 virtual_pad.py --nav-udp, 기본 포트 5006):
NAV (20 B): b"NAV2" + <I seq> + <f vx> <f vy> <f omega>   — 패드가 RT 를 당기는 동안만 로봇에 넘김
"""
import math
import struct

MAGIC = b"CAPS"
PKT_V1 = struct.Struct("<4sIfff")
PKT_V2 = struct.Struct("<4sIfffbB")

NAV_MAGIC = b"NAV2"
NAV_PORT = 5006

# lift_flags 비트
LIFT_STOP = 1   # 리프트 정지 + 주차 취소
LIFT_PARK = 2   # 주차 (최하단까지 내림)
LIFT_ZERO = 4   # 1 s 이상 누르면 지금 위치 = 0 mm
LIFT_CLEAR = 8  # Pico 오류 해제


def encode(seq, vx, vy, omega, lift_pwm=0, lift_flags=0):
    return PKT_V2.pack(MAGIC, seq & 0xFFFFFFFF, vx, vy, omega, max(-100, min(100, int(round(lift_pwm)))),
                       lift_flags & 0xFF)


def decode(data):
    """→ (seq, vx, vy, omega, lift_pwm, lift_flags) 또는 None (형식 오류)."""
    if len(data) == PKT_V2.size:
        magic, seq, vx, vy, w, pwm, flags = PKT_V2.unpack(data)
    elif len(data) == PKT_V1.size:
        magic, seq, vx, vy, w = PKT_V1.unpack(data)
        pwm, flags = 0, 0
    else:
        return None
    if magic != MAGIC or not all(math.isfinite(v) for v in (vx, vy, w)) or not -100 <= pwm <= 100:
        return None
    return seq, vx, vy, w, pwm, flags


def encode_nav(seq, vx, vy, omega):
    return PKT_V1.pack(NAV_MAGIC, seq & 0xFFFFFFFF, vx, vy, omega)


def decode_nav(data):
    """→ (seq, vx, vy, omega) 또는 None."""
    if len(data) != PKT_V1.size:
        return None
    magic, seq, vx, vy, w = PKT_V1.unpack(data)
    if magic != NAV_MAGIC or not all(math.isfinite(v) for v in (vx, vy, w)):
        return None
    return seq, vx, vy, w
