"""전 노드 정지: 속도 0 → IDLE. --estop 이면 ESTOP 프레임 (에러가 걸려서 clear.py 필요)."""
from lib import cansimple as cs
from lib.runtime import base_parser, id_label, run_with_bus


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--estop", action="store_true", help="ESTOP 송신 (축에 ESTOP_REQUESTED 에러 남음)")
    args = ap.parse_args()

    def _run(bus):
        for can_id in args.can_id:
            if args.estop:
                bus.send(can_id, cs.Cmd.ESTOP)
                print(f"  {id_label(can_id)}: ESTOP 송신")
            else:
                bus.set_vel(can_id, 0.0)
                bus.set_state(can_id, cs.AxisState.IDLE)
                print(f"  {id_label(can_id)}: vel 0 + IDLE 송신")
        bus.pump(0.3)
        for can_id in args.can_id:
            hb = bus.heartbeat(can_id, 0.5)
            print(f"  {id_label(can_id)}: " + ("heartbeat 없음" if hb is None else
                  f"{cs.state_name(hb.state)} axis_error={cs.decode_flags(hb.error, cs.AxisError)}"))

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# python stop.py                 # 등록된 모든 노드
# python stop.py --can-id 1
# python stop.py --estop
