"""Type00 장치 ID 조회(ping) 헬퍼. scan.py / set_can_id.py 공용."""

import time

from .common import unpack_ext_id
from .frames import TYPE00_GET_DEVICE_ID, TYPE00_REPLY_MARK, frame_type00_get_id


def drain(bus):
    while bus.recv(timeout=0.0) is not None:
        pass


def wait_type00(bus, motor_id: int | None, timeout: float) -> tuple[int, bytes] | None:
    """Type00 응답 대기 → (motor_id, uid8). motor_id=None이면 아무 모터나."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data2, data1 = unpack_ext_id(msg.arbitration_id)
        if comm_type != TYPE00_GET_DEVICE_ID or data1 != TYPE00_REPLY_MARK:
            continue
        mid = data2 & 0xFF
        if motor_id is not None and mid != motor_id:
            continue
        return mid, bytes(msg.data)
    return None


def ping(bus, host_id: int, motor_id: int, timeout: float) -> bytes | None:
    """motor_id에 Type00 요청 → 응답 UID(8B) 또는 None."""
    drain(bus)
    bus.send_ext(*frame_type00_get_id(host_id, motor_id))
    r = wait_type00(bus, motor_id, timeout)
    return None if r is None else r[1]
