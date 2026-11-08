#!/usr/bin/env python3
"""Type02 피드백 / Type24 active report / Type21 고장 프레임을 실시간 출력 (수신만 — 브리지 돌 때 같이 써도 됨).

명령 없이 상태만 보려면 다른 터미널에서 active_report.py --enable 1 (다 보면 --enable 0)."""

import argparse
import math

from lib.common import unpack_ext_id
from lib.config import DEFAULT_CH, STEER_IDS
from lib.frames import TYPE02, TYPE21, TYPE24
from lib.parse import fault_names, parse_feedback_any, parse_type21_fault
from lib.runtime import run_with_bus

MODES = {0: "reset", 1: "cali", 2: "RUN"}


def main():
    ap = argparse.ArgumentParser(description="RS02 피드백 모니터 (송신 없음)")
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", default=None)
    ap.add_argument("--every", type=int, default=1, help="N 개마다 1줄 출력")
    args = ap.parse_args()
    filt = set(args.can_id) if args.can_id else None

    def _run(bus):
        cnt = 0
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None or not msg.is_extended_id:
                continue
            comm_type, _, _ = unpack_ext_id(msg.arbitration_id)
            if comm_type in (TYPE02, TYPE24):
                fb = parse_feedback_any(msg.arbitration_id, bytes(msg.data))
                if fb is None or (filt and fb.motor_id not in filt):
                    continue
                cnt += 1
                if cnt % args.every:
                    continue
                src = "T24" if comm_type == TYPE24 else "T02"
                print(f"[{fb.motor_id} {STEER_IDS.get(fb.motor_id, '?'):2s} {src}] "
                      f"pos={fb.pos:+.4f} rad ({math.degrees(fb.pos):+7.2f}°) vel={fb.vel:+.3f} "
                      f"tor={fb.tor:+.3f} Nm temp={fb.temp_c:.1f}C mode={MODES.get(fb.mode_status, fb.mode_status)} "
                      f"fault={fault_names(fb.fault_bits)}")
            elif comm_type == TYPE21:
                r = parse_type21_fault(msg.arbitration_id, bytes(msg.data))
                if r and (not filt or r[0] in filt):
                    print(f"[{r[0]}] TYPE21 FAULT fault=0x{r[1]:08X} warning=0x{r[2]:08X}")

    try:
        run_with_bus(args.ch, _run)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

# python3 monitor.py
# python3 monitor.py --can_id 1 --every 10
