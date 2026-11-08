"""에러 클리어 (Clear_Errors 0x018) 후 heartbeat 로 확인."""
import time

from lib import cansimple as cs
from lib.runtime import base_parser, id_label, run_with_bus


def main():
    args = base_parser(__doc__).parse_args()

    def _run(bus):
        t0 = time.monotonic()
        for can_id in args.can_id:
            bus.clear_errors(can_id)
        for can_id in args.can_id:
            hb = bus.wait_heartbeat(can_id, timeout=0.5, newer_than=t0 + 0.02)
            print(f"  {id_label(can_id)}: " + ("heartbeat 없음" if hb is None else
                  f"{cs.state_name(hb.state)} axis_error={cs.decode_flags(hb.error, cs.AxisError)}"))

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# python clear.py
# python clear.py --can-id 1
