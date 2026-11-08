"""속도 명령만 1회 송신 (Set_Input_Vel 0x00D). 폐루프 진입/정지는 안 함 → state.py, stop.py 와 조합.

보드는 마지막 input_vel 을 계속 유지한다 (워치독 꺼진 현재 설정 기준) — 한 번 보내면 바꿀 때까지 그 속도.
보내기 전 heartbeat 로 CLOSED_LOOP 인지 확인하고, 아니면 경고만 하고 보내지 않음 (--force 로 무시).
"""
from lib import cansimple as cs
from lib.config import HEARTBEAT_TIMEOUT, VEL_MAX
from lib.runtime import base_parser, id_label, run_with_bus


def main():
    ap = base_parser(__doc__, ids_required=True)
    ap.add_argument("--vel", type=float, nargs="+", required=True,
                    help="turn/s. 값 1개면 모든 노드 동일, 노드 수만큼 주면 각각")
    ap.add_argument("--force", action="store_true", help="CLOSED_LOOP 아니어도 송신")
    args = ap.parse_args()

    vels = args.vel * len(args.can_id) if len(args.vel) == 1 else args.vel
    if len(vels) != len(args.can_id):
        raise SystemExit("--vel 개수는 1 또는 --can-id 개수와 같아야 함")
    for v in vels:
        if abs(v) > VEL_MAX:
            raise SystemExit(f"|vel|={abs(v)} > VEL_MAX={VEL_MAX} (lib/config.py)")

    def _run(bus):
        bus.pump(0.15)
        for n, v in zip(args.can_id, vels):
            hb = bus.heartbeat(n, HEARTBEAT_TIMEOUT)
            if hb is None:
                print(f"  {id_label(n)}: heartbeat 없음 — 송신 안 함")
                continue
            if hb.state != cs.AxisState.CLOSED_LOOP_CONTROL and not args.force:
                print(f"  {id_label(n)}: 현재 {cs.state_name(hb.state)} — 송신 안 함 "
                      f"(먼저 python state.py --can-id {n} --state closed_loop)")
                continue
            bus.set_vel(n, v)
            print(f"  {id_label(n)}: input_vel = {v:+.3f} turn/s 송신")

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# 단계별로 나눠서 쓰기
# python state.py  --can-id 1 --state closed_loop     # 1) 폐루프 진입
# python setvel.py --can-id 1 --vel 0.5               # 2) 속도 명령 (보드가 유지)
# python setvel.py --can-id 1 --vel -0.3              #    바꾸기
# python setvel.py --can-id 1 --vel 0                 # 3) 0
# python state.py  --can-id 1 --state idle            # 4) 정지   (급하면 python stop.py)
