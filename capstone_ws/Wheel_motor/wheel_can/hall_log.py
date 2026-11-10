"""USB 로 Hall 상태·에러를 계속 기록 (CSV) — 주행 중 간헐적 ENCODER_FAILED 원인 찾기용.

PC 에서 실행 (odrive_env, 보드를 USB 로 연결, odrivetool 은 닫기). CAN 으로 주행하는 동안 같이 켜 둬도 됨.
  python hall_log.py --axis 1                      # 보드 #2 의 M1 = 4번 FL
  python hall_log.py --axis 1 --out fl_run1.csv

기록: t[s], hall_state, axis_error, encoder_error, motor_error, vel[turn/s], Iq_meas[A], vbus[V], shadow_count
화면: 1초마다 요약 한 줄 + hall 0/7 또는 에러가 생기는 순간 "!!" 줄 (몇 초에 무슨 값이었는지)
주의: USB 폴링(수백 Hz)은 보드의 Hall 샘플(수 kHz)보다 느려서 아주 짧은 튐은 hall_state 로 안 보일 수 있음
      → 그래도 encoder_error 가 0x10 으로 바뀌는 시각은 정확히 잡힘. 그 직전 줄들의 속도·전류를 볼 것.
"""
import argparse
import csv
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", type=int, choices=(0, 1), default=1, help="보드 axis 번호 (4번 FL = 보드 #2 의 1)")
    ap.add_argument("--out", default=None, help="CSV 경로 (기본 hall_axis<N>_<시각>.csv)")
    ap.add_argument("--no-clear", action="store_true", help="시작할 때 에러 클리어 안 함")
    args = ap.parse_args()
    out = Path(args.out or time.strftime(f"hall_axis{args.axis}_%Y%m%d_%H%M%S.csv"))

    import odrive  # odrive_env
    print("USB 연결 중 ... (odrivetool 이 켜져 있으면 실패)")
    odrv = odrive.find_any(timeout=10)
    ax = getattr(odrv, f"axis{args.axis}")
    print(f"연결: serial {odrv.serial_number:X}  axis{args.axis}  → {out}")
    if not args.no_clear:
        ax.error = 0
        ax.encoder.error = 0
        ax.motor.error = 0

    t0 = time.monotonic()
    n = bad_n = 0
    seen = set()
    prev_err = None
    last_print = 0.0
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "hall_state", "axis_error", "encoder_error", "motor_error", "vel", "iq_meas", "vbus",
                    "shadow_count"])
        try:
            while True:
                t = time.monotonic() - t0
                h = ax.encoder.hall_state
                ae, ee, me = ax.error, ax.encoder.error, ax.motor.error
                vel = ax.encoder.vel_estimate
                iq = ax.motor.current_control.Iq_measured
                vbus = odrv.vbus_voltage
                sc = ax.encoder.shadow_count
                w.writerow([f"{t:.4f}", h, ae, ee, me, f"{vel:.3f}", f"{iq:.3f}", f"{vbus:.2f}", sc])
                n += 1
                seen.add(h)
                if h in (0, 7):
                    bad_n += 1
                    print(f"\n!! t={t:8.3f}s  hall_state={h}  vel={vel:+.2f}  Iq={iq:+.2f} A", flush=True)
                err = (ae, ee, me)
                if err != prev_err and prev_err is not None:
                    print(f"\n!! t={t:8.3f}s  에러 변화 axis=0x{ae:X} encoder=0x{ee:X} motor=0x{me:X}  "
                          f"(vel={vel:+.2f} Iq={iq:+.2f} A vbus={vbus:.1f})", flush=True)
                    f.flush()
                prev_err = err
                if t - last_print >= 1.0:
                    last_print = t
                    print(f"\r t={t:7.1f}s  {n / max(t, 1e-6):5.0f} Hz  hall={h}  seen={sorted(seen)}  0/7 횟수={bad_n}  "
                          f"enc_err=0x{ee:X}  vel={vel:+.2f}  Iq={iq:+.2f}   ", end="", flush=True)
        except KeyboardInterrupt:
            pass
    print(f"\n\n{n} 줄 저장 → {out}   hall 0/7 {bad_n}회, 관측 {sorted(seen)}, 마지막 encoder_error=0x{prev_err[1]:X}"
          if prev_err else f"\n저장 → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
