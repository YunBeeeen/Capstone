"""속도 명령: (필요시 CLOSED_LOOP 진입) → 일정 시간 속도 유지(주기 재송신) → 0 → IDLE.

Ctrl+C / 에러 / 축이 폐루프에서 빠짐 → 즉시 속도 0 + IDLE.
명령을 주기적으로 다시 보내는 이유: 나중에 워치독을 켜도 그대로 동작하게.
"""
import time

from lib import cansimple as cs
from lib.config import HEARTBEAT_TIMEOUT, VEL_MAX, VEL_RAMP_RATE
from lib.runtime import base_parser, id_label, request_state, run_with_bus


def main():
    ap = base_parser(__doc__, ids_required=True)
    ap.add_argument("--vel", type=float, nargs="+", required=True,
                    help="turn/s. 값 1개면 모든 노드 동일, 노드 수만큼 주면 각각")
    ap.add_argument("--duration", type=float, default=5.0, help="유지 시간 [s]")
    ap.add_argument("--rate", type=float, default=20.0, help="명령 재송신/피드백 주기 [Hz]")
    ap.add_argument("--no-enter", action="store_true", help="CLOSED_LOOP 진입 생략 (이미 폐루프일 때)")
    ap.add_argument("--keep", action="store_true", help="끝나도 IDLE 로 안 내림 (속도 0 유지)")
    args = ap.parse_args()

    vels = args.vel * len(args.can_id) if len(args.vel) == 1 else args.vel
    if len(vels) != len(args.can_id):
        raise SystemExit("--vel 개수는 1 또는 --can-id 개수와 같아야 함")
    for v in vels:
        if abs(v) > VEL_MAX:
            raise SystemExit(f"|vel|={abs(v)} > VEL_MAX={VEL_MAX} (lib/config.py). 보드 vel_limit 도 확인")
    cmd = dict(zip(args.can_id, vels))

    def stop_all(bus, idle: bool):
        for n in args.can_id:
            bus.set_vel(n, 0.0)
        if idle:
            for n in args.can_id:
                bus.set_state(n, cs.AxisState.IDLE)

    def _run(bus):
        bus.pump(0.15)
        if not args.no_enter:
            print("CLOSED_LOOP 진입")
            for n in args.can_id:
                bus.set_vel(n, 0.0)
                if request_state(bus, n, cs.AxisState.CLOSED_LOOP_CONTROL) != cs.AxisState.CLOSED_LOOP_CONTROL:
                    stop_all(bus, idle=True)
                    raise SystemExit("폐루프 진입 실패 → monitor.py --errors 로 원인 확인")

        print(f"속도 {cmd} turn/s, {args.duration}s  (Ctrl+C = 즉시 정지)")
        pos0 = {n: (e[0] if (e := bus.get_encoder(n)) else None) for n in args.can_id}   # 이번 실행 기준점
        iq_log = {n: [] for n in args.can_id}
        period = 1.0 / args.rate
        t0 = time.monotonic()
        next_print = t0
        aborted = None
        try:
            while (el := time.monotonic() - t0) < args.duration:
                tick = time.monotonic()
                for n, v in cmd.items():
                    bus.set_vel(n, v)
                parts = []
                for n in args.can_id:
                    hb = bus.heartbeat(n, HEARTBEAT_TIMEOUT)
                    if hb is None:
                        aborted = f"id{n} heartbeat 끊김"
                    elif hb.state != cs.AxisState.CLOSED_LOOP_CONTROL:
                        aborted = (f"id{n} 폐루프 이탈 → {cs.state_name(hb.state)} "
                                   f"{cs.decode_flags(hb.error, cs.AxisError)}")
                    enc, iq = bus.get_encoder(n), bus.get_iq(n)
                    if enc and pos0[n] is not None:
                        parts.append(f"id{n} v{enc[1]:+.2f} Δ{enc[0] - pos0[n]:+6.2f}")
                    else:
                        parts.append(f"id{n} enc=--")
                    if iq:
                        iq_log[n].append(iq[1])
                        parts[-1] += f" {iq[1]:+.2f}A"
                    else:
                        parts[-1] += " --A"
                if aborted:
                    break
                if tick >= next_print:
                    print(f"\r t={el:5.2f}s  " + " | ".join(parts) + "   ", end="", flush=True)
                    next_print = tick + 0.1
                time.sleep(max(0.0, period - (time.monotonic() - tick)))
        except KeyboardInterrupt:
            aborted = "Ctrl+C"
        finally:
            print()
            if aborted:
                print(f"!! 중단: {aborted} → 속도 0 + IDLE")
                stop_all(bus, idle=True)
            else:
                stop_all(bus, idle=False)
                ramp = max(abs(v) for v in vels) / VEL_RAMP_RATE + 0.5
                print(f"속도 0, 감속 대기 {ramp:.1f}s")
                t_end = time.monotonic() + ramp
                while time.monotonic() < t_end:
                    for n in args.can_id:
                        bus.set_vel(n, 0.0)
                    bus.pump(0.05)
                if not args.keep:
                    stop_all(bus, idle=True)
                    print("IDLE")
        bus.pump(0.3)
        print("요약 (Δpos = 이번 실행에서 실제로 돈 바퀴 수, 감속 포함 / Iq 는 평균)")
        for n in args.can_id:
            enc = bus.get_encoder(n)
            dpos = f"{enc[0] - pos0[n]:+.3f} turn" if enc and pos0[n] is not None else "--"
            want = "" if aborted else f"  (기대 {cmd[n] * args.duration:+.3f})"
            iqs = iq_log[n]
            iq_avg = f"{sum(iqs) / len(iqs):+.2f} A" if iqs else "--"
            print(f"  {id_label(n)}: Δpos {dpos}{want}  Iq 평균 {iq_avg}")
        for n in args.can_id:
            hb = bus.heartbeat(n, HEARTBEAT_TIMEOUT)
            if hb:
                print(f"  {id_label(n)}: {cs.state_name(hb.state)} axis_error={cs.decode_flags(hb.error, cs.AxisError)}")

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# python vel.py --can-id 1 --vel 0.5 --duration 10           # 10초 → 5바퀴 확인
# python vel.py --can-id 1 --vel -0.5
# python vel.py --can-id 1 2 3 4 --vel 0.3                   # 네 바퀴 같이
# python vel.py --can-id 1 2 3 4 --vel 0.3 -0.3 0.3 -0.3     # 각각 다른 속도
# 실시간 줄: id{n} v{속도 turn/s} Δ{이번 실행 회전수} {Iq}A
