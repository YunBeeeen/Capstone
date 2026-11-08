"""제어 모드 변경 (Set_Controller_Mode 0x00B) — RAM 에만 적용, 재부팅하면 USB 저장값으로 복귀."""
from lib import cansimple as cs
from lib.runtime import base_parser, id_label, run_with_bus


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--control", default="velocity", choices=[m.name.lower() for m in cs.ControlMode])
    ap.add_argument("--input", default="vel_ramp", choices=[m.name.lower() for m in cs.InputMode])
    args = ap.parse_args()
    cm, im = cs.ControlMode[args.control.upper()], cs.InputMode[args.input.upper()]

    def _run(bus):
        for can_id in args.can_id:
            bus.set_controller_mode(can_id, cm, im)
            print(f"  {id_label(can_id)}: control_mode={cm.name}({int(cm)}) input_mode={im.name}({int(im)}) 송신")
        print("  (CAN 으로는 읽어서 확인 불가 — 확인은 USB: axis.controller.config.control_mode)")

    run_with_bus(args.ch, _run)


if __name__ == "__main__":
    main()

# python mode.py --can-id 1 2                               # velocity + vel_ramp (기본, 저장된 설정과 동일)
# python mode.py --can-id 1 --control torque --input passthrough
