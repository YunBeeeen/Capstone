#!/usr/bin/env python3
"""팔로워 모터 1~6 (+그리퍼) 각도를 읽기 전용으로 보여 주고, 모델상 손끝 위치를 같이 계산한다. 라파에서 실행.

  ~/rebot_env/venv/bin/python tools/arm_watch.py --zed-height 1.2          # 5 Hz 로 계속 (Ctrl+C 종료)
  ~/rebot_env/venv/bin/python tools/arm_watch.py --zed-height 1.2 --once   # 한 번 읽고 끝

모터에 enable·영점·위치·토크 명령을 보내지 않는다 (라파 ~/rebot_env/state_watch.py 와 같은 연결·읽기).
토크가 꺼진 팔을 손으로 옮겨 두고 (마찰로 버티는 자세) 손끝을 줄자로 재서 '손끝 (모델)' 과 비교한다.
각도는 팀 LeRobot 공개 각도 [도] = play_plan --pose · plan.json 과 같은 값. '--motors=' 줄은 fk_check.py 에 그대로 붙여 넣으면 된다.
손끝 = 두 손가락 끝의 가운데, 좌표 = 로봇 중심 아래 바닥 기준 (x 앞, y 왼쪽 +, z 위).
CAN 이 올라와 있어야 한다 (sudo ~/capstone/capstone_ws/can/can_up.sh).
"""
import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]   # 모터 1~6
ZED_HEIGHT_AT_LIFT0 = 0.675


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zed-height", type=float, help="바닥 → ZED 삼각대 나사 [m]. 주면 모델상 손끝 위치도 보여 줌")
    ap.add_argument("--once", action="store_true", help="한 번 읽고 끝")
    ap.add_argument("--rate", type=float, default=5.0)
    args = ap.parse_args()

    kin = None
    if args.zed_height is not None:
        sys.path.insert(0, str(ROOT))
        from arm_kinematics.b601 import B601Kinematics      # pinocchio 필요 (라파 rebot_env 에 있음)
        kin = B601Kinematics()
        lift_total = args.zed_height - ZED_HEIGHT_AT_LIFT0
        if not 0.0 <= lift_total <= 2 * kin.lift_stage_max + 1e-6:
            ap.error(f"--zed-height → 리프트 {1000 * lift_total:.0f} mm, 범위 0~{2000 * kin.lift_stage_max:.0f}")

    from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import RebotB601FollowerRobotConfig
    from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower
    r = RebotB601Follower(RebotB601FollowerRobotConfig(port="can_arm_r", can_adapter="socketcan", id="arm_r",
                                                       disable_torque_on_disconnect=False, gripper_open_deg=340.0))
    r.connect(calibrate=False)
    try:
        while True:
            for m in r.motors.values():
                m.request_feedback()
            for _ in range(3):
                r.bus.poll_feedback_once()
                time.sleep(0.001)
            rows, pub = [], {}
            for n in r.motor_names:
                s = r.motors[n].get_state()
                if s is None:
                    pub[n] = None
                    status = "  응답없음"
                else:
                    pub[n] = r._motor_to_public_position(n, math.degrees(s.pos))
                    status = f"{pub[n]:8.2f}°  {max(s.t_mos, s.t_rotor):3.0f}℃  st={s.status_code}"
                label = f"모터 {NAMES.index(n) + 1}" if n in NAMES else "그리퍼"
                rows.append(f"{label:6s} {n:13s}{status}")
            out = ["모터   이름            각도      온도   (읽기 전용" + (", Ctrl+C 종료)" if not args.once else ")")] + rows
            arm = [pub.get(n) for n in NAMES]
            if all(v is not None for v in arm):
                out.append("")
                out.append("--motors=" + ",".join(f"{v:.2f}" for v in arm))
                if kin is not None:
                    T = kin.fk(arm, lift_total / 2)
                    p, a = T.translation, T.rotation[:, 0]
                    out.append(f"손끝 (모델, ZED {args.zed_height:.3f} m, 리프트 {1000 * lift_total:.0f} mm): "
                               f"x {100 * p[0]:6.1f}  y {100 * p[1]:6.1f}  z {100 * p[2]:6.1f} cm   "
                               f"손가락 방향 수평에서 아래로 {math.degrees(math.asin(-a[2])) + 0.0:.0f}°")
            text = "\n".join(out) + "\n"
            if args.once:
                sys.stdout.write(text)
                break
            sys.stdout.write("\x1b[H\x1b[J" + text)
            sys.stdout.flush()
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
