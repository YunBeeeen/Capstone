#!/usr/bin/env python3
"""MIT 위치 제어 조향 스텝 시험 — 게인/속도 튜닝용. 브리지와 같은 방식.

순서: Type04 로 현재각 읽기(확정 확인) → Type03 enable → RUN 확인 → 현재각에서 시작해
      목표각들을 차례로 rate [rad/s] 로 램프 (p=램프 위치, v=램프 속도 피드포워드, kp, kd, t=0)
      → 각 목표에서 --hold 초 유지 → 끝/Ctrl+C/에러 시 Type04 (토크 끔).

출력: 목표마다 도착 시간(|오차| < 2°), 마지막 0.5 s 평균 오차, 최대 |토크|.  --log 로 CSV.
바닥에 내려놓고 제자리 조향(예: 0 → 90 → 0) 하면 타이어 마찰 때문에 남는 오차를 볼 수 있음 → kp 결정.
"""

import argparse
import csv
import math
import time

from lib.common import unpack_ext_id
from lib.config import DEFAULT_CH, HOST_ID, STEER_IDS
from lib.frames import TYPE02, frame_type01_mit, frame_type03_enable, frame_type04_stop
from lib.parse import MODE_RUN, fault_names, parse_feedback_like_type2
from lib.runtime import run_with_bus

LIMIT_DEG = 180.0     # 조향 운용 한계 (swerve steer_limit_deg 와 같게, 스토퍼 ±190° 까지 여유 10°)
CONFIRM_DEG = 165.0   # 켤 때 각도 확정 기준 (angle.py 설명)
ARRIVE_DEG = 2.0


def read_feedback(bus, fbs: dict, timeout: float = 0.0):
    """쌓인 프레임을 비우며 최신 Type02 를 fbs[motor_id] 에 저장."""
    deadline = time.perf_counter() + timeout
    while True:
        left = deadline - time.perf_counter()
        msg = bus.recv(timeout=max(0.0, left))
        if msg is None:
            if left <= 0:
                return
            continue
        if msg.is_extended_id and unpack_ext_id(msg.arbitration_id)[0] == TYPE02:
            fb = parse_feedback_like_type2(msg.arbitration_id, bytes(msg.data))
            if fb:
                fbs[fb.motor_id] = (time.perf_counter(), fb)


def main():
    ap = argparse.ArgumentParser(description="RS02 MIT 조향 스텝 시험")
    ap.add_argument("--can_id", type=lambda x: int(x, 0), nargs="+", required=True)
    ap.add_argument("--target", type=float, nargs="+", required=True, help="조향 목표각들 [deg], 차례로")
    ap.add_argument("--rate", type=float, default=2.0, help="램프 속도 [rad/s]")
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=2.0)
    ap.add_argument("--hold", type=float, default=2.0, help="목표마다 유지 [s]")
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--sign", type=float, default=1.0, help="조향각 = sign × 모터각")
    ap.add_argument("--log", help="CSV 경로")
    ap.add_argument("--ch", default=DEFAULT_CH)
    ap.add_argument("--host_id", type=lambda x: int(x, 0), default=HOST_ID)
    args = ap.parse_args()

    ids = list(dict.fromkeys(args.can_id))
    for t in args.target:
        if abs(t) > LIMIT_DEG:
            raise SystemExit(f"목표 {t}° 가 ±{LIMIT_DEG}° 밖")
    if args.sign not in (1.0, -1.0):
        raise SystemExit("--sign 은 1 또는 -1")
    sgn = args.sign

    def _run(bus):
        fbs: dict = {}
        stop_all = lambda: [bus.send_ext(*frame_type04_stop(args.host_id, i)) for i in ids]

        # 1) 현재각 (토크 끔 상태)
        stop_all()
        read_feedback(bus, fbs, 0.2)
        for i in ids:
            if i not in fbs:
                raise SystemExit(f"can_id={i} 무응답 (전원/배선/ID 확인: python3 scan.py)")
            fb = fbs[i][1]
            deg = math.degrees(sgn * fb.pos)
            if fb.fault_bits:
                raise SystemExit(f"can_id={i} 고장 {fault_names(fb.fault_bits)} → enable.py --enable 0 --clear_fault 1")
            if abs(deg) >= CONFIRM_DEG:
                raise SystemExit(f"can_id={i} {deg:+.1f}° — 켤 때 각도 미확정 (angle.py 설명대로 손으로 돌릴 것)")
            print(f"can_id={i} {STEER_IDS.get(i, '?')} 시작 {deg:+.2f}°")

        log = None
        if args.log:
            f = open(args.log, "w", newline="")
            log = csv.writer(f)
            log.writerow(["t", "can_id", "target_deg", "cmd_deg", "pos_deg", "err_deg", "vel", "tor", "temp"])

        try:
            # 2) enable → RUN 확인
            for i in ids:
                bus.send_ext(*frame_type03_enable(args.host_id, i))
            t_en = time.perf_counter()
            while True:
                read_feedback(bus, fbs, 0.02)
                if all(fbs[i][0] >= t_en and fbs[i][1].mode_status == MODE_RUN for i in ids):
                    break
                if time.perf_counter() - t_en > 1.0:
                    raise SystemExit("enable 후 RUN 모드 확인 실패")

            # 3) 램프 — 조향각 기준으로 계산하고 보낼 때 sign 적용
            q = {i: sgn * fbs[i][1].pos for i in ids}     # 명령 조향각 [rad]
            dt = 1.0 / args.hz
            t0 = time.perf_counter()
            nxt = t0
            for tgt_deg in args.target:
                tgt = math.radians(tgt_deg)
                seg_start = time.perf_counter()
                arrived = {i: None for i in ids}
                reached_at = None
                tail = {i: [] for i in ids}
                peak_tor = {i: 0.0 for i in ids}
                while True:
                    now = time.perf_counter()
                    for i in ids:
                        step = max(-args.rate * dt, min(args.rate * dt, tgt - q[i]))
                        q[i] += step
                        v_ff = step / dt
                        bus.send_ext(*frame_type01_mit(i, sgn * q[i], sgn * v_ff, args.kp, args.kd, 0.0))
                    read_feedback(bus, fbs, 0.0)
                    for i in ids:
                        ts, fb = fbs[i]
                        if fb.fault_bits:
                            raise SystemExit(f"can_id={i} 고장 {fault_names(fb.fault_bits)}")
                        if now - ts > 0.1:
                            raise SystemExit(f"can_id={i} 피드백 끊김")
                        pos = sgn * fb.pos
                        err = math.degrees(tgt - pos)
                        if arrived[i] is None and abs(err) < ARRIVE_DEG:
                            arrived[i] = now - seg_start
                        peak_tor[i] = max(peak_tor[i], abs(fb.tor))
                        if log:
                            log.writerow([f"{now - t0:.4f}", i, tgt_deg, f"{math.degrees(q[i]):.3f}",
                                          f"{math.degrees(pos):.3f}", f"{err:.3f}", f"{fb.vel:.3f}",
                                          f"{fb.tor:.3f}", f"{fb.temp_c:.1f}"])
                    if reached_at is None and all(abs(q[i] - tgt) < 1e-9 for i in ids):
                        reached_at = now
                    if reached_at is not None:
                        if now - reached_at > args.hold - 0.5:
                            for i in ids:
                                tail[i].append(math.degrees(tgt - sgn * fbs[i][1].pos))
                        if now - reached_at > args.hold:
                            break
                    nxt += dt
                    sl = nxt - time.perf_counter()
                    if sl > 0:
                        time.sleep(sl)
                    else:
                        nxt = time.perf_counter()
                for i in ids:
                    tl = tail[i]
                    mean_err = sum(tl) / len(tl) if tl else float("nan")
                    arr = f"{arrived[i]:.2f} s" if arrived[i] is not None else "미도착"
                    print(f"  → {tgt_deg:+.1f}° can_id={i}: 도착 {arr}, 최종 오차 {mean_err:+.2f}°, "
                          f"최대 |토크| {peak_tor[i]:.2f} Nm")
        finally:
            stop_all()
            if log:
                f.close()
            print("토크 끔 (Type04)")

    try:
        run_with_bus(args.ch, _run)
    except KeyboardInterrupt:
        print("중단")


if __name__ == "__main__":
    main()

# 바퀴 띄우고 먼저: python3 step.py --can_id 1 --target 30 0 --rate 1.0
# 바닥 제자리 조향:  python3 step.py --can_id 1 --target 90 0 --kp 20 --kd 2 --log fr_kp20.csv
# 4축 동시:         python3 step.py --can_id 1 2 3 4 --target 45 0
