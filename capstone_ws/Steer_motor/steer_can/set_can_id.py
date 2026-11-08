#!/usr/bin/env python3
"""Type07로 모터 CAN_ID를 변경하는 유틸 (USB-CAN/socketcan 경유, 벤더 툴 불필요).

동작 순서:
  1) Type00으로 현재 ID 응답 확인 (없으면 중단)
  2) Type00으로 새 ID가 비어 있는지 확인 (응답 있으면 충돌로 중단, --force로 무시)
  3) Type07 전송 → 새 ID의 Type00 응답 대기
  4) 새 ID 응답 / 이전 ID 무응답 재확인

주의: 같은 ID 모터가 버스에 여러 개면 전부 바뀐다. 바꿀 모터만 연결하고 실행할 것.
     모터는 disable(stop) 상태에서 진행 권장.
"""

import argparse
import time

from lib.config import HOST_ID, DEFAULT_CH
from lib.device_id import drain as _drain, ping as _ping, wait_type00
from lib.frames import frame_type04_stop, frame_type07_set_can_id
from lib.runtime import run_with_bus


def main():
    ap = argparse.ArgumentParser(description="모터 CAN_ID 변경 (Type07)")
    ap.add_argument("--can_id", type=lambda x: int(x, 0), required=True, help="현재 CAN_ID")
    ap.add_argument("--new_id", type=lambda x: int(x, 0), required=True, help="변경할 CAN_ID (1~127)")
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x: int(x, 0), default=HOST_ID)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--force", action="store_true", help="현재 ID 무응답/새 ID 충돌 검사 무시")
    ap.add_argument("--yes", action="store_true", help="확인 프롬프트 생략")
    args = ap.parse_args()

    cur, new, host = args.can_id, args.new_id, args.host_id
    if not (1 <= new <= 127):
        raise SystemExit(f"new_id={new}: 1~127 범위만 허용")
    if new == cur:
        raise SystemExit("new_id가 현재 ID와 같음")
    if new == (host & 0xFF):
        raise SystemExit(f"new_id가 host_id(0x{host:02X})와 같음")

    def _run(bus):
        uid = _ping(bus, host, cur, args.timeout)
        if uid is None:
            print(f"[WARN] 현재 ID {cur} Type00 무응답")
            if not args.force:
                raise SystemExit("중단 (연결/ID 확인, 또는 --force)")
        else:
            print(f"[OK] 현재 ID {cur} 응답 uid={uid.hex()}")

        taken = _ping(bus, host, new, args.timeout)
        if taken is not None:
            print(f"[WARN] 새 ID {new}에 이미 장치 있음 uid={taken.hex()}")
            if not args.force:
                raise SystemExit("중단 (ID 충돌, 또는 --force)")

        if not args.yes:
            ans = input(f"CAN_ID {cur} -> {new} 변경합니까? [y/N] ").strip().lower()
            if ans != "y":
                raise SystemExit("취소")

        # 안전: 변경 전 disable
        bus.send_ext(*frame_type04_stop(host, cur, clear_fault=False))
        time.sleep(0.05)

        _drain(bus)
        arb_id, data = frame_type07_set_can_id(host, cur, new)
        bus.send_ext(arb_id, data)
        print(f"sent SET_CAN_ID: {cur} -> {new} arb_id=0x{arb_id:08X} data={data.hex()}")

        r = wait_type00(bus, new, args.timeout * 2)
        reply = None if r is None else r[1]
        if reply is not None:
            print(f"[OK] 새 ID {new} 응답 uid={reply.hex()}")
        else:
            print("[INFO] Type07 직후 응답 없음 → 재확인")

        time.sleep(0.1)
        ok_new = _ping(bus, host, new, args.timeout)
        old_alive = _ping(bus, host, cur, args.timeout)
        if ok_new is not None and old_alive is None:
            print(f"[DONE] CAN_ID {cur} -> {new} 변경 확인. 전원 재투입 후에도 유지되는지 확인할 것 (python3 scan.py)")
        else:
            print(f"[FAIL?] new({new}) 응답={'O' if ok_new is not None else 'X'} "
                  f"old({cur}) 응답={'O' if old_alive is not None else 'X'}")
            raise SystemExit(1)

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# 실행 예시 (기본 채널: can_steer, 바꿀 모터만 버스에 연결)
# python3 set_can_id.py --can_id 1 --new_id 5
# python3 set_can_id.py --can_id 127 --new_id 3 --yes
