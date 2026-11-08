#!/usr/bin/env python3
"""조향각 읽기 + 켤 때 각도 확정 여부.

기본: Type04(stop, 토크 끔)를 보내 응답 Type02 로 읽음 → 모터를 disable 시킴. 브리지가 돌 때는 --listen.

왜 확정이 필요한가: RS02 는 켤 때 한 바퀴 안의 각도만 앎 (zero_sta=1 → −π~π).
조향 기구 한계가 ±190° 라서 실제 +185° 는 켤 때 −175° 로 읽힘. 접힌(aliased) 값은 항상 |θ| ≥ 360−190 = 170°
이므로 |읽은 값| < confirm_limit(165°, 여유 5°) 면 실제 각이 확실함. 아니면 손으로 정면 쪽으로 돌려서
|θ| < 165° 가 되게 (정말 접힌 상태면 정면으로 돌려도 값이 줄지 않고 커짐 → 반대로 돌릴 것).
"""

import argparse
import math
import time

from lib.common import unpack_ext_id
from lib.config import DEFAULT_CH, HOST_ID, STEER_IDS
from lib.frames import TYPE02, frame_type04_stop
from lib.parse import fault_names, parse_feedback_like_type2
from lib.runtime import run_with_bus


def main():
    ap = argparse.ArgumentParser(description="조향각 + 확정 여부 (기본은 Type04 폴링 — 토크 끔)")
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", default=sorted(STEER_IDS))
    ap.add_argument("--sign", type=float, default=1.0, help="조향각 = sign × 모터각 (+ = 위에서 보고 반시계)")
    ap.add_argument("--confirm_deg", type=float, default=165.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--listen", action="store_true", help="송신 안 함 (브리지가 돌 때)")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x: int(x, 0), default=HOST_ID)
    args = ap.parse_args()

    def _run(bus):
        last = {}
        while True:
            if not args.listen:
                for cid in args.can_id:
                    bus.send_ext(*frame_type04_stop(args.host_id, cid))
            deadline = time.time() + 1.0 / args.hz
            while (left := deadline - time.time()) > 0:
                msg = bus.recv(timeout=left)
                if msg is None or not msg.is_extended_id:
                    continue
                if unpack_ext_id(msg.arbitration_id)[0] != TYPE02:
                    continue
                fb = parse_feedback_like_type2(msg.arbitration_id, bytes(msg.data))
                if fb and fb.motor_id in args.can_id:
                    last[fb.motor_id] = fb
            parts = []
            for cid in args.can_id:
                fb = last.get(cid)
                name = STEER_IDS.get(cid, "?")
                if fb is None:
                    parts.append(f"{cid} {name}: --- 무응답")
                    continue
                deg = math.degrees(args.sign * fb.pos)
                ok = "확정" if abs(deg) < args.confirm_deg else "미확정!"
                flt = "" if fb.fault_bits == 0 else f" FAULT={fault_names(fb.fault_bits)}"
                parts.append(f"{cid} {name}: {deg:+7.2f}° {ok}{flt}")
            print(" | ".join(parts), flush=True)
            if args.once:
                return
            last.clear()

    try:
        run_with_bus(args.ch, _run)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

# python3 angle.py                 # 4축, 5 Hz, Ctrl+C 종료
# python3 angle.py --can_id 1 --once
# python3 angle.py --listen        # 브리지 실행 중
