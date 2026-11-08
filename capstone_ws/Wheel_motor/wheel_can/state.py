"""축 상태 전환 (IDLE / CLOSED_LOOP / 캘리브레이션) — heartbeat 로 결과 확인."""
from lib import cansimple as cs
from lib.runtime import base_parser, id_label, request_state, run_with_bus, wait_until_idle

CAL_TIMEOUT = {cs.AxisState.MOTOR_CALIBRATION: 15, cs.AxisState.ENCODER_OFFSET_CALIBRATION: 30,
               cs.AxisState.FULL_CALIBRATION_SEQUENCE: 45}


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--state", required=True,
                    help=f"{' | '.join(cs.STATE_ALIASES)} 또는 숫자 (1=IDLE, 8=CLOSED_LOOP)")
    ap.add_argument("--clear", action="store_true", help="전환 전에 에러 클리어")
    args = ap.parse_args()

    s = args.state.lower()
    state = cs.STATE_ALIASES[s] if s in cs.STATE_ALIASES else cs.AxisState(int(s, 0))
    if state == cs.AxisState.UNDEFINED:
        raise SystemExit("0(UNDEFINED) 은 '요청 없음' 이라 아무것도 안 바뀜. 정지는 --state idle")
    if state in CAL_TIMEOUT:
        input(f"{state.name}: 바퀴 띄웠나? 모터가 움직임. Enter 로 진행 (Ctrl+C 취소) ")

    def _run(bus):
        bus.pump(0.15)
        for can_id in args.can_id:
            if args.clear:
                bus.clear_errors(can_id)
            if state in CAL_TIMEOUT:
                bus.set_state(can_id, state)
                print(f"  {id_label(can_id)}: {state.name} 진행 중 ...")
                err = wait_until_idle(bus, can_id, CAL_TIMEOUT[state])
                if err is None:
                    print("  !! 시간 초과 — IDLE 요청")
                    bus.set_state(can_id, cs.AxisState.IDLE)
                else:
                    print(f"  완료, axis_error={cs.decode_flags(err, cs.AxisError)}")
            else:
                request_state(bus, can_id, state)

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# 실행 예시 (wheel_can 폴더에서, venv 켜고)
# python state.py --can-id 1 --state closed_loop
# python state.py --can-id 1 2 3 4 --state idle
# python state.py --can-id 1 --state closed_loop --clear
# python state.py --can-id 1 --state offset_cal      # 캘리브레이션 (CAN 으로 결과 저장은 불가 → USB)
