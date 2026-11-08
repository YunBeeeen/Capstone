#!/usr/bin/env python3
"""Type00으로 버스의 모터 CAN_ID를 스캔한다 (응답한 ID와 MCU UID 출력)."""

import argparse
import time

from lib.config import HOST_ID, DEFAULT_CH
from lib.device_id import drain, wait_type00
from lib.frames import frame_type00_get_id
from lib.runtime import run_with_bus


def main():
    ap = argparse.ArgumentParser(description="모터 CAN_ID 스캔 (Type00)")
    ap.add_argument("--min", type=lambda x: int(x, 0), default=1)
    ap.add_argument("--max", type=lambda x: int(x, 0), default=127)
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x: int(x, 0), default=HOST_ID)
    ap.add_argument("--timeout", type=float, default=0.05, help="ID당 응답 대기(s)")
    args = ap.parse_args()

    def _run(bus):
        found: dict[int, set[bytes]] = {}
        drain(bus)
        for mid in range(args.min, args.max + 1):
            if mid == (args.host_id & 0xFF):
                continue
            bus.send_ext(*frame_type00_get_id(args.host_id, mid))
            # 응답이 늦게 와서 다음 ID 창에 섞여도 응답 ID 기준으로 기록
            deadline = time.time() + args.timeout
            while (left := deadline - time.time()) > 0:
                r = wait_type00(bus, None, left)
                if r is None:
                    break
                found.setdefault(r[0], set()).add(r[1])
        # 남은 지연 응답 수거
        while (r := wait_type00(bus, None, 0.2)) is not None:
            found.setdefault(r[0], set()).add(r[1])

        print(f"scan {args.min}~{args.max} on {args.ch}: {len(found)}개 발견")
        for mid in sorted(found):
            uids = found[mid]
            dup = "  [!] UID 여러 개 = 같은 ID 중복" if len(uids) > 1 else ""
            print(f"  can_id={mid:3d} (0x{mid:02X})  uid={','.join(u.hex() for u in sorted(uids))}{dup}")

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# 실행 예시 (기본 채널: can_steer = 조향 버스)
# python3 scan.py
# python3 scan.py --min 1 --max 10
