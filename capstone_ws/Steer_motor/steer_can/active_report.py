"""Type24 active report 켜기/끄기 — 켜면 모터가 명령 없이도 Type02 피드백을 주기적으로 보냄 (~/idle_ws/motor 에서 가져옴).

monitor.py 만 켜 두고 상태(위치·속도·토크·온도·mode·fault)를 보고 싶을 때. 끄는 걸 잊지 말 것 —
켜 둔 채 브리지를 돌리면 버스에 피드백 프레임이 그만큼 더 실림 (4축 × 100 Hz ≈ 1 Mbps 의 6% 정도).
설정은 RAM (전원 다시 켜면 꺼짐).
"""

import argparse
import time

from lib.config import DEFAULT_CH, HOST_ID, STEER_IDS
from lib.frames import frame_type24_active_report
from lib.runtime import run_with_bus


def _dedupe(values: list[int]) -> list[int]:
    return list(dict.fromkeys(values))


def main():
    ap = argparse.ArgumentParser(description="RS02 active report (Type24) 켜기/끄기")
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", default=sorted(STEER_IDS))
    ap.add_argument("--enable", type=int, choices=[0, 1], required=True)
    ap.add_argument("--hz", type=float, default=0.0, help=">0 이면 Type24 를 주기 송신, <=0 이면 1회 송신")
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x: int(x, 0), default=HOST_ID)
    args = ap.parse_args()
    can_ids = _dedupe(args.can_id)
    if args.hz < 0:
        raise SystemExit("--hz must be >= 0")

    def send_all(bus, tag=""):
        for can_id in can_ids:
            arb_id, data = frame_type24_active_report(args.host_id, can_id, enable=bool(args.enable))
            bus.send_ext(arb_id, data)
            print(f"sent ACTIVE_REPORT{tag}: can_id={can_id} ({STEER_IDS.get(can_id, '?')}) enable={args.enable} "
                  f"arb_id=0x{arb_id:08X} data={data.hex()}")

    def _run(bus):
        if args.hz <= 0:
            send_all(bus)
            return
        period, next_t, n = 1.0 / args.hz, time.perf_counter(), 0
        print(f"ACTIVE_REPORT stream start: can_id={can_ids} hz={args.hz} enable={args.enable}")
        while True:
            n += 1
            send_all(bus, f"[{n}]")
            next_t += period
            dt = next_t - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.perf_counter()

    try:
        run_with_bus(args.ch, _run)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

# 실행 예시 (기본 채널: can_steer, 기본 ID 1~4)
# python3 active_report.py --enable 1          # 4축 켜기 → 다른 터미널에서 python3 monitor.py
# python3 active_report.py --enable 0          # 4축 끄기
# python3 active_report.py --can_id 1 --enable 1
