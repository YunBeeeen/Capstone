"""스크립트 공통: 인터페이스 확인, 버스 실행, 인자, 상태 전환 확인."""
import argparse
import sys
import time
from pathlib import Path
from typing import Callable, TypeVar

import can

from . import cansimple as cs
from .bus import OdriveBus
from .config import BITRATE, DEFAULT_CH, HEARTBEAT_TIMEOUT, CAN_IDS

T = TypeVar("T")


def check_interface(channel: str) -> None:
    p = Path(f"/sys/class/net/{channel}")
    if not p.exists():
        sys.exit(f"[{channel}] 인터페이스 없음 — USB-CAN 어댑터 연결 확인 (lsusb, ip -br link)")
    flags = int((p / "flags").read_text(), 16)
    if not flags & 0x1:  # IFF_UP
        sys.exit(f"[{channel}] DOWN 상태 →  sudo ip link set {channel} up type can bitrate {BITRATE}")


def run_with_bus(channel: str, fn: Callable[[OdriveBus], T], *, interface: str = "socketcan") -> T:
    if interface == "socketcan":
        check_interface(channel)
    bus = OdriveBus(channel, interface=interface)
    try:
        return fn(bus)
    except can.CanOperationError as e:
        if "105" in str(e) or "buffer space" in str(e):
            sys.exit(
                f"\n[{channel}] 송신 큐가 꽉 참 (ENOBUFS) — 버스에서 아무도 ACK 를 안 함.\n"
                f"  보드 전원 / CANH·CANL 뒤바뀜 / GND / 커넥터 / 비트레이트를 확인하세요.\n"
                f"  상태 확인:  ip -details -statistics link show {channel}\n"
                f"  BUS-OFF 면: sudo ip link set {channel} down && "
                f"sudo ip link set {channel} up type can bitrate {BITRATE} restart-ms 100")
        raise
    finally:
        bus.close()


def base_parser(desc: str, *, ids_required: bool = False) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--can-id", type=int, nargs="+", default=None if ids_required else list(CAN_IDS),
                    required=ids_required, help=f"모터 CAN ID 목록 (기본 {list(CAN_IDS)})")
    ap.add_argument("--ch", default=DEFAULT_CH)
    return ap


def id_label(can_id: int) -> str:
    return f"id{can_id} {CAN_IDS.get(can_id, '(미등록)')}"


def request_state(bus: OdriveBus, can_id: int, state: int, *, settle: float = 0.3) -> cs.AxisState | int | None:
    """상태 요청 후 heartbeat 로 결과 확인. 반환: 확인된 현재 상태 (heartbeat 없으면 None)."""
    t0 = time.monotonic()
    bus.set_state(can_id, state)
    time.sleep(settle)
    hb = bus.wait_heartbeat(can_id, timeout=HEARTBEAT_TIMEOUT, newer_than=t0 + settle * 0.5)
    if hb is None:
        print(f"  {id_label(can_id)}: heartbeat 없음 (CAN ID/배선/전원 확인)")
        return None
    ok = hb.state == state
    print(f"  {id_label(can_id)}: 요청 {cs.state_name(state)} → 현재 {cs.state_name(hb.state)}"
          f"  axis_error={cs.decode_flags(hb.error, cs.AxisError)}  {'OK' if ok else '!!'}")
    return hb.state


def wait_until_idle(bus: OdriveBus, can_id: int, timeout: float) -> int | None:
    """캘리브레이션처럼 끝나면 IDLE 로 돌아오는 상태를 기다림."""
    deadline = time.monotonic() + timeout
    time.sleep(0.5)
    while time.monotonic() < deadline:
        hb = bus.wait_heartbeat(can_id, timeout=HEARTBEAT_TIMEOUT, newer_than=time.monotonic() - 0.01)
        if hb is not None and hb.state == cs.AxisState.IDLE:
            return hb.error
    return None
