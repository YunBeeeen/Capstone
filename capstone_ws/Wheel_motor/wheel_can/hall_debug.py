"""Hall 센서 디버그 (손으로 바퀴 돌리면서).

CAN 모드(기본): Get_Encoder_Count 로 Hall 카운트를 읽음. Hall 엣지 1개 = 1카운트.
  - 섹터 = count_in_cpr % 6 (전기각 1주기 안의 Hall 구간 0~5)
  - 카운트가 한 번에 2 이상 튀면 경고 (천천히 돌리는데도 튀면 노이즈/배선 의심)
  - Ctrl+C 시: 누적 Δcount, Δ/6 (= 돌린 전기각 주기 수), 1바퀴 돌렸다면 = 극쌍수
  - raw hall_state(1~6) 는 CAN 으로 못 읽음 → --usb

USB 모드(--usb): odrivetool 닫고, USB 연결 상태에서 encoder.hall_state 를 직접 읽음 (0/7 = 배선 불량).
"""
import sys
import time

from lib.config import CPR, POLE_PAIRS
from lib.runtime import base_parser, id_label, run_with_bus


def run_can(args):
    can_id = args.can_id[0]

    def _run(bus):
        first = bus.get_count(can_id)
        if first is None:
            sys.exit(f"{id_label(can_id)}: Get_Encoder_Count 응답 없음 (CAN ID/전원 확인)")
        c0 = prev = first[0]
        jumps, edges = 0, 0
        print(f"{id_label(can_id)}  CPR={CPR} (pp {POLE_PAIRS})  — 바퀴 표시 후 손으로 돌리기, Ctrl+C 로 결과")
        try:
            while True:
                r = bus.get_count(can_id)
                if r is not None:
                    sc, cic = r
                    d = sc - prev
                    if abs(d) >= 2:
                        jumps += 1
                    edges += abs(d)
                    prev = sc
                    tot = sc - c0
                    print(f"\r shadow={sc:+8d}  in_cpr={cic:3d}/{CPR}  sector={cic % 6}  "
                          f"Δ={tot:+6d}  Δ/6={tot / 6:+7.2f}  turns={tot / CPR:+6.3f}  jumps≥2: {jumps}   ",
                          end="", flush=True)
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            tot = prev - c0
            print(f"\n\n 누적 Δcount = {tot}  (엣지 합 {edges})")
            print(f" 전기각 주기 = Δ/6 = {tot / 6:.2f}   ← 정확히 1바퀴 돌렸다면 이게 극쌍수 (설정값 {POLE_PAIRS})")
            print(f" 기계 회전 = Δ/CPR = {tot / CPR:.3f} turn")
            if jumps:
                print(f" !! 2카운트 이상 점프 {jumps}회 — 빨리 돌렸으면 정상, 천천히 돌렸는데 나오면 Hall 노이즈/배선 확인")

    run_with_bus(args.ch, _run)


def run_usb(args):
    import odrive  # odrive_env 의 패키지
    axis_idx = args.axis
    print("USB 연결 중 ... (odrivetool 이 켜져 있으면 실패)")
    odrv = odrive.find_any(timeout=10)
    ax = getattr(odrv, f"axis{axis_idx}")
    ax.encoder.error = 0
    c0 = ax.encoder.shadow_count
    seen, last = set(), None
    seq = []
    print(f"axis{axis_idx}: hall_state 는 1~6 만 정상. 손으로 천천히 돌리기, Ctrl+C 로 결과")
    try:
        while True:
            h = ax.encoder.hall_state
            seen.add(h)
            if h != last:
                seq.append(h)
                last = h
            d = ax.encoder.shadow_count - c0
            print(f"\r hall_state={h}  seen={sorted(seen)}  최근 순서={seq[-7:]}  Δcount={d:+6d}  Δ/6={d / 6:+7.2f}   ",
                  end="", flush=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print(f"\n\n 관측된 상태 {sorted(seen)}")
        bad = seen - set(range(1, 7))
        missing = set(range(1, 7)) - seen
        if bad:
            print(f" !! 비정상 상태 {sorted(bad)} — Hall 5V/GND/신호선 확인")
        if missing:
            print(f" ?? 아직 안 나온 상태 {sorted(missing)} — 더 돌려보고도 안 나오면 Hall 선 하나 불량")
        if ax.encoder.error:
            print(f" !! encoder.error = 0x{ax.encoder.error:X}")


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--usb", action="store_true", help="USB 로 raw hall_state 읽기")
    ap.add_argument("--axis", type=int, choices=(0, 1), default=0, help="--usb 일 때 보드 axis 번호")
    ap.add_argument("--rate", type=float, default=50.0, help="CAN 폴링 주기 [Hz]")
    args = ap.parse_args()
    run_usb(args) if args.usb else run_can(args)


if __name__ == "__main__":
    main()

# python hall_debug.py --can-id 1            # CAN: Hall 카운트/섹터/극쌍수 측정
# python hall_debug.py --usb --axis 1      # USB: raw hall_state (odrivetool 닫고)
