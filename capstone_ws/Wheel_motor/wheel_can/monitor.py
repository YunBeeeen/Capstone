"""모터 상태 디버그: heartbeat(상태·에러), 위치/속도, Iq, 버스전압을 주기적으로 표시. 명령은 안 보냄(읽기 전용)."""
import time

from lib import cansimple as cs
from lib.config import HEARTBEAT_TIMEOUT, VBUS_TRIP, VBUS_WARN
from lib.runtime import base_parser, id_label, run_with_bus

CLEAR = "\033[H\033[J"


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--rate", type=float, default=5.0, help="갱신 주기 [Hz]")
    ap.add_argument("--errors", action="store_true", help="motor/encoder 에러 레지스터도 요청")
    ap.add_argument("--log", action="store_true", help="화면 갱신 대신 한 줄씩 출력 (파일로 저장할 때)")
    args = ap.parse_args()

    def _run(bus):
        period = 1.0 / args.rate
        t0 = time.monotonic()
        try:
            while True:
                tick = time.monotonic()
                bus.pump(0.0)
                lines, logs = [], []
                vbus = None
                for n in args.can_id:
                    hb = bus.heartbeat(n, HEARTBEAT_TIMEOUT)
                    enc, iq = bus.get_encoder(n), bus.get_iq(n)
                    if vbus is None:
                        vbus = bus.get_vbus(n)
                    st = "LOST(heartbeat 없음)" if hb is None else cs.state_name(hb.state)
                    ae = "-" if hb is None else cs.decode_flags(hb.error, cs.AxisError)
                    pos = f"{enc[0]:9.3f}" if enc else "      --"
                    vel = f"{enc[1]:+7.3f}" if enc else "     --"
                    iqs = f"{iq[0]:+6.2f}/{iq[1]:+6.2f}" if iq else "   --/--   "
                    lines.append(f"{id_label(n):24s} {st:22s} pos {pos} turn  vel {vel} turn/s  "
                                 f"Iq set/meas {iqs} A")
                    lines.append(f"{'':24s} axis_error {ae}")
                    if args.errors:
                        me, ee = bus.get_motor_error(n), bus.get_encoder_error(n)
                        lines.append(f"{'':24s} motor_error {'--' if me is None else f'0x{me:X}'}"
                                     f"   encoder_error {'--' if ee is None else cs.decode_flags(ee, cs.EncoderError)}")
                    logs.append(f"n{n} {st} pos={pos.strip()} vel={vel.strip()} iq={iqs.strip()} err={ae}")
                    if args.errors:
                        logs[-1] += f" motor_err={'--' if me is None else f'0x{me:X}'} enc_err={'--' if ee is None else f'0x{ee:X}'}"
                vb = f"{vbus:.2f} V" if vbus is not None else "--"
                if vbus is not None and vbus < VBUS_WARN:
                    vb += f"  !! 배터리 낮음 (<{VBUS_WARN} V) — 충전 필요, {VBUS_TRIP} V 에서 보드 트립"
                if args.log:
                    print(f"{time.monotonic() - t0:8.2f}s vbus={vb} " + " | ".join(logs), flush=True)
                else:
                    print(CLEAR + f"XDrive monitor  vbus {vb}   ({args.rate:.0f} Hz, Ctrl+C 종료)\n")
                    print("\n".join(lines), flush=True)
                time.sleep(max(0.0, period - (time.monotonic() - tick)))
        except KeyboardInterrupt:
            print()

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# python monitor.py                  # 등록된 모든 노드
# python monitor.py --can-id 1 --errors
# python monitor.py --log > log.txt
