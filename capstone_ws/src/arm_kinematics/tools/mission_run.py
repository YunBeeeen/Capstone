#!/usr/bin/env python3
"""실기 임무 한 번에 (라파): desk_1 → 물체 찾기 → 집기 → 바구니에 넣기 → 바구니 벽 잡고 들기 → shelf_A_view → shelf_A →
지정한 자리에 바구니 놓기. **팔 연결 하나로 끝까지** (play_plan 을 단계마다 따로 부르면 끝날 때 토크가 풀려 든 것을 놓침).

  cd ~/arm && PY=~/rebot_env/venv/bin/python
  $PY tools/mission_run.py pumpkin --place 0.50 -0.05 --board 1.05
  $PY tools/mission_run.py pumpkin --place 0.50 -0.05 --board 1.05 --from basket_grab     # 그 단계부터
  $PY tools/mission_run.py pumpkin --place 0.50 -0.05 --board 1.05 --dry-run --vision v.json   # 팔·Jetson 없이 계획·순서만

단계 (--from / --to 이름): nav_desk → lift_up → find → pick → drop → basket_grab → carry → lift_down → nav_shelf →
  lift_place → place → finish
  - 단계마다 할 일을 보여 주고 ENTER (s = 건너뛰기, q = 지금 자세 유지하고 끝). --yes 면 안 물음 (리프트 안내는 늘 물음)
  - 이동 (Nav2) 은 Jetson 의 nav_goal_ahead 를 부름 → **가는 동안 게임패드 RT 를 당기고 있어야 함**
  - 리프트는 사람이 게임패드로 (RB+Y/X 올림, RB+A/B 내림). 주행은 반드시 리프트 최저 (내비가 카메라 높이를 최저로 가정)
  - 물체 찾기 = Jetson ros_find (ZED ROS 를 켠 채로, 카메라 높이는 보이는 바닥으로 잼). OWLv2 서버 필요 (docker start arm_vision_server)
  - 팔이 막히거나 (Stalled) Ctrl+C: 그 자리 고정 → ENTER = 그 동작의 시작 자세로 같은 길 복귀, q = 그대로 끝
  - 끝날 때 팀 팔로워가 "팔을 받친 뒤 ENTER" 를 물은 다음 토크를 끔 (바구니를 든 채면 먼저 받칠 것)

동작 (오른팔, 로봇 바닥 기준 m):
  pick        물체 중심 6 cm 위 → 곧게 내려와 닫기 → 곧게 6 cm 올림
  drop        바구니 가운데 위 (손끝 = 테두리 + 3 cm) 9 cm 위 → 곧게 내려와 열기 → 곧게 올림
  basket_grab 바구니 짧은 벽 (로봇 쪽 먼저) 가운데, 손끝 = 테두리 − 3 cm. 거의 수직 (70~89°) 으로 내려와 손가락이 벽을
              가로지르게 (IK 손가락 방향 조건) → 닫기 → 6 cm 올림
  carry       바구니를 든 채 로봇 오른쪽 앞 (카메라 시야 밖) 으로
  place       --place X Y 위, 판 높이 --board (잰 수평면 중 가장 가까운 것) 에 바구니 바닥이 1 cm 위가 되게 → 내려와 열기 →
              올림 → 같은 길로 carry 자세
바구니: 21×13×10 cm (--basket), 벽 두께 6 mm 가정. 바구니를 든 상태의 충돌은 계획이 모름 → 단계마다 눈으로 확인.
"""
import argparse
import json
import math
import shlex
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PY = sys.executable
STEPS = ["nav_desk", "lift_up", "find", "pick", "drop", "basket_grab", "carry", "lift_down", "nav_shelf",
         "lift_place", "place", "finish"]
CARRY = ((0.22, -0.30, 0.30), (0.25, -0.28, 0.32), (0.18, -0.32, 0.28), (0.28, -0.25, 0.35))   # (x, y, 책상 위 z)
WALL = 0.006
GRIP_BELOW_RIM = 0.03          # 바구니 벽을 테두리에서 이만큼 아래까지 손가락으로
DROP_ABOVE_RIM = 0.03          # 물체 놓을 때 손끝 = 테두리 + 이만큼
PLACE_CLEAR = 0.01             # 놓을 때 바구니 바닥 ↔ 판


def phrase(word):
    w = word.strip()
    return w if " " in w else ("an " if w[0].lower() in "aeiou" else "a ") + w


class Stop(Exception):
    """임무를 여기서 멈춤 (팔은 지금 자세 유지)."""


# ---------------------------------------------------------------------------------------------- 팔
class Arm:
    """팔로워 연결 하나 + play_plan 의 재생·그리퍼·복귀. dry 면 연결 없이 경로 끝 자세만 따라감."""

    def __init__(self, a):
        import play_plan as pp
        self.pp, self.a, self.dry = pp, a, a.dry_run
        self.pargs = pp.build_parser().parse_args([])
        self.pargs.speed = a.speed
        self.T = pp.load_timeline_module()
        self.robot = None
        if self.dry:
            self.q_fake = np.array([float(v) for v in a.start.split(",")]) if a.start else np.zeros(6)
            self.g_fake = -270.0
            return
        self.robot, cfg = pp.make_robot(self.pargs)
        print(f"팔로워 연결: {cfg.port}")
        self.robot.connect()
        pp.apply_gravity_scales(self.robot, self.pargs.gravity_scale)
        if pp.preflight(self.robot, drift_deg=self.pargs.hold_drift):
            raise Stop("시작 점검 실패 — 아무것도 안 움직임")

    def q(self):
        return self.q_fake.copy() if self.dry else self.pp.measured(self.robot)[0]

    def gripper(self):
        return self.g_fake if self.dry else self.pp.measured(self.robot)[1]

    def hold(self):
        if not self.dry:
            self.robot.hold()

    def close(self):
        if self.robot is not None:
            self.robot.hold()
            self.robot.disconnect()       # 팀 코드: 받친 뒤 ENTER → 토크 해제

    def grip(self, how):
        if self.dry:
            self.g_fake = -270.0 if how == "open" else -40.0
            print(f"  [dry] 그리퍼 {how}")
            return self.g_fake
        self.pp.recheck(self.robot, "그리퍼")
        self.pp.do_grip(self.robot, self.q(), how, self.pargs)
        self.robot.hold()
        return self.gripper()

    def timeline(self, plan):
        p = json.loads(Path(plan).read_text())
        if not p.get("validated") or not p.get("lift_fixed"):
            raise Stop(f"{plan}: 검증 안 된 계획이거나 리프트 고정이 아님")
        v = float(p.get("v_limit_deg", 30.0))
        acc = min(float(p.get("a_limit_deg", 120.0)), 120.0)
        return self.T.Timeline(p["waypoints"], v_limit_deg=v, speed=self.a.speed, a_limit_deg=acc), v, acc

    def _play(self, tl, t_end, label):
        pp, g = self.pp, self.gripper()
        if self.dry:
            self.q_fake = np.asarray(tl.sample(t_end)[0], float)
            print(f"  [dry] {label} {t_end:.1f} s")
            return
        pp.recheck(self.robot, label)
        pp.play(self.robot, tl, g, self.pargs.rate, self.pargs.lag_pause, t_end=t_end, label=f"[{label}] ",
                stall_s=self.pargs.stall, insert_lag_deg=self.pargs.insert_lag, settle_s=self.pargs.settle,
                settle_deg=self.pargs.settle_deg)
        self.robot.hold()

    def run(self, plan, label, upto=None, grip_before=None, grip_after=None):
        """계획 재생 (시작 맞춤 → [그리퍼] → 재생 → [그리퍼]). 막히면 복귀를 물음. 반환 (tl, 끝 시각)."""
        pp = self.pp
        tl, v, acc = self.timeline(plan)
        t_end = tl.phase_end(upto) if upto else tl.duration
        start = np.asarray(tl.sample(0.0)[0], float)
        if not self.dry:
            bad = pp.check_limits(self.robot, tl, t_end)
            if bad:
                raise Stop("팔로워 관절 한계 밖 점: " + "; ".join(bad[:3]))
        diff = float(np.max(np.abs(self.q() - start)))
        if diff > self.pargs.start_tol:
            raise Stop(f"지금 자세가 계획 시작과 {diff:.1f}° 다름 (다시 계획 필요)")
        if diff > 0.5 and not self.dry:
            align = self.T.Timeline([{"phase": "align", "motors_deg": {str(i): self.q()[i - 1] for i in range(1, 7)}},
                                     {"phase": "align", "motors_deg": {str(i): start[i - 1] for i in range(1, 7)}}],
                                    v_limit_deg=10.0, a_limit_deg=acc)
            self._play(align, align.duration, "시작 맞춤")
        if grip_before:
            self.grip(grip_before)
        try:
            self._play(tl, t_end, label)
        except (pp.Stalled, KeyboardInterrupt) as e:
            self.hold()
            why = "Ctrl+C" if isinstance(e, KeyboardInterrupt) else f"막힘: {e}"
            print(f"\n⚠ {label} {why} — 팔 고정")
            try:
                ask("ENTER = 이 동작 시작 자세로 같은 길 복귀, q = 그대로 끝: ", allow_skip=False, always=True)
            except Stop:
                raise Stop(f"{label} 중단 (복귀 안 함)")
            self.back(tl, t_end)
            raise Stop(f"{label} 중단 (시작 자세로 복귀함)")
        if grip_after:
            self.grip(grip_after)
        return tl, t_end

    def _sub(self, tl, phases):
        segs = [s for s in tl.segments if s[0] in phases]
        sub = self.T.Timeline.__new__(self.T.Timeline)
        sub.segments = segs
        sub.starts = np.concatenate([[0.0], np.cumsum([s[3] for s in segs])])
        sub.duration = float(sub.starts[-1])
        sub.v_limit, sub.a_limit, sub.acc = tl.v_limit, tl.a_limit, tl.acc
        return sub

    def back(self, tl, t_from, phases=None, label="복귀"):
        """재생한 길을 거꾸로 (phases 를 주면 그 단계만 — 예: insert = 곧게 내려온 만큼 곧게 올림)."""
        pp = self.pp
        if phases:
            tl = self._sub(tl, phases)
            t_from = tl.duration
        q_now = self.q()
        if not self.dry:
            t_from = pp.closest_time(tl, q_now, t_from)
        bt = pp.build_return_timeline(self.T, tl, t_from, q_now, tl.v_limit, tl.a_limit, self.a.speed)
        try:
            self._play(bt, bt.duration, label)
        except (pp.Stalled, KeyboardInterrupt) as e:
            self.hold()
            raise Stop(f"{label} 중 멈춤: {e}")


# ---------------------------------------------------------------------------------------------- 도우미
YES = False


def ask(msg, allow_skip=True, always=False):
    """ENTER = 진행, s = 건너뛰기, q = 끝. --yes 면 (always 가 아니면) 바로 진행."""
    if YES and not always:
        print(msg + "(--yes)")
        return ""
    try:
        r = input(msg).strip().lower()
    except EOFError:
        r = "q"
    if r == "q":
        raise Stop("사용자 중단")
    return "s" if (r == "s" and allow_skip) else ""


def jetson(a, cmd, stream=False):
    ssh = ["ssh", "-i", str(Path.home() / ".ssh/id_ed25519_jetson"), "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes",
           "-o", "ConnectTimeout=8", a.jetson, "bash -lc " + shlex.quote("source ~/nav_env.sh >/dev/null 2>&1; " + cmd)]
    if stream:
        return subprocess.run(ssh, stdin=subprocess.DEVNULL, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return subprocess.run(ssh, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=180)


def find(a, words):
    """Jetson ros_find → json dict (objects, planes, zed_height)."""
    if a.dry_run:
        f = a.vision if words else (a.vision_shelf or a.vision)
        if not f:
            raise Stop("--dry-run 에는 --vision <ros_find json> 이 필요")
        return json.loads(Path(f).read_text())
    q = " ".join(shlex.quote(w) for w in words)
    r = jetson(a, f"cd ~/arm_vision && python3 tools/ros_find.py {q} --min-score {a.min_score} --json /tmp/m_find.json "
                  f"--out /tmp/m_find.jpg > /tmp/m_find.log 2>&1; tail -8 /tmp/m_find.log; echo ===JSON===; cat /tmp/m_find.json")
    if "===JSON===" not in r.stdout:
        raise Stop(f"Jetson 찾기 실패: {r.stdout[-500:]} {r.stderr[-300:]}")
    log, js = r.stdout.split("===JSON===", 1)
    print("   " + "\n   ".join(x for x in log.strip().splitlines() if x.strip()))
    try:
        return json.loads(js)
    except json.JSONDecodeError:
        raise Stop("찾기 결과 json 이 없음 (위 로그)")


def nav(a, name):
    if a.dry_run:
        print(f"  [dry] Nav2 → {name}")
        return
    print(f"  Nav2 → {name}: **게임패드 RT 를 당기고 있기** (놓으면 멈춤)")
    r = jetson(a, f"python3 $NAV/nav_goal_ahead.py --name {shlex.quote(name)}", stream=True)
    print("   " + "\n   ".join(r.stdout.strip().splitlines()[-4:]))
    if "결과: 성공" not in r.stdout:
        raise Stop(f"{name} 로 이동 실패 (위 출력)")


def plan(a, arm, name, xyz, desk_top, front, lateral, zed, descend=None, extra=()):
    out = Path(f"/tmp/mission_{name}.json")
    out.unlink(missing_ok=True)
    start = ",".join(f"{v:.2f}" for v in arm.q())
    cmd = [PY, str(HERE / "ik_sim_run.py"), "--target", "desk", "--desk-top", f"{desk_top:.3f}", "--front", f"{front:.3f}",
           f"--lateral={lateral:.3f}", "--xyz", *(f"{v:.3f}" for v in xyz), "--zed-height", f"{zed:.3f}",
           f"--start={start}", "--no-sim", "--save-plan", str(out), *extra]
    if descend:
        cmd += ["--descend", f"{descend}"]
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL, timeout=600)
    if r.returncode != 0 or not out.exists():
        tail = [x for x in r.stdout.splitlines() if any(k in x for k in ("실패", "없음", "불가", "충돌", "범위"))]
        print(f"   계획 {name} 실패 ({time.time() - t0:.0f} s): " + (" / ".join(tail[-3:]) or r.stdout[-300:] + r.stderr[-300:]))
        return None
    sel = next((x.strip() for x in r.stdout.splitlines() if "리프트" in x and "부하" in x), "")
    dur = json.loads(out.read_text())["waypoints"][-1]["t"]
    print(f"   계획 {name}: 손끝 ({', '.join(f'{100 * v:.1f}' for v in xyz)}) cm, {dur:.1f} s, {sel[:90]} ({time.time() - t0:.0f} s)")
    return out


def tool_axes(plan_path):
    """계획 마지막 자세의 (접근축, 손가락축) — 바구니를 든 자세를 놓을 때 유지하려고."""
    sys.path.insert(0, str(HERE.parent))
    from arm_kinematics import B601Kinematics
    from arm_kinematics.ik import B601IK
    w = json.loads(Path(plan_path).read_text())["waypoints"][-1]
    kin = B601Kinematics()
    T, _ = B601IK(kin)._pose(kin.configuration([w["motors_deg"][str(i)] for i in range(1, 7)], w["lift_total_mm"] / 2000))
    return T.rotation[:, 0].copy(), T.rotation[:, 1].copy()


# ---------------------------------------------------------------------------------------------- 임무
def main():
    global YES
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("word", help="집을 물체 (한 단어면 a/an 붙임). 예: pumpkin, \"a toy corn\"")
    ap.add_argument("--place", type=float, nargs=2, required=True, metavar=("X", "Y"),
                    help="선반에 바구니를 놓을 자리 (shelf_A 설 자리에서 로봇 기준 손끝 x, y [m])")
    ap.add_argument("--board", type=float, required=True, help="놓을 판 높이 (바닥 → 판 윗면 m, 대략. 잰 수평면 중 ±6 cm 안 가장 가까운 것)")
    ap.add_argument("--desk", default="desk_1")
    ap.add_argument("--shelf", default="shelf_A")
    ap.add_argument("--shelf-view", default="shelf_A_view", help="선반 앞에서 먼저 들를 장소 (없으면 '')")
    ap.add_argument("--basket", default="basket", help="바구니를 찾는 말")
    ap.add_argument("--basket-size", type=float, nargs=3, default=(0.21, 0.13, 0.10), metavar=("L", "W", "H"))
    ap.add_argument("--descend", type=float, default=0.06)
    ap.add_argument("--dz", type=float, default=0.0, help="물체 목표 높이 보정 [m] (− = 더 깊이)")
    ap.add_argument("--min-score", type=float, default=0.15)
    ap.add_argument("--speed", type=float, default=1.0, help="재생 속도 배율 (1.0 = 30°/s)")
    ap.add_argument("--from", dest="from_", choices=STEPS, default=STEPS[0])
    ap.add_argument("--to", choices=STEPS, default=STEPS[-1])
    ap.add_argument("--yes", action="store_true", help="단계마다 묻지 않음 (리프트 안내는 물음)")
    ap.add_argument("--jetson", default="idle@192.168.1.11")
    ap.add_argument("--dry-run", action="store_true", help="팔·Jetson 없이: --vision json 으로 계획·순서만")
    ap.add_argument("--vision", help="--dry-run 용 ros_find json")
    ap.add_argument("--vision-shelf", help="--dry-run 용 선반 앞 ros_find json (없으면 --vision)")
    ap.add_argument("--start", help="--dry-run 시작 자세 모터 1~6")
    a = ap.parse_args()
    YES = a.yes
    steps = STEPS[STEPS.index(a.from_):STEPS.index(a.to) + 1]
    L, W, H = a.basket_size
    q_obj, q_bk = phrase(a.word), phrase(a.basket)
    st = {}                                                     # 단계 사이 값

    arm = Arm(a)
    print(f"시작 자세 (모터 1~6): {np.round(arm.q(), 1).tolist()}, 단계: {' → '.join(steps)}")
    try:
        for s in steps:
            print(f"\n=== {s} ===")
            arm.hold()
            if s == "nav_desk":
                if ask(f"리프트 최저·팔 접힘 확인 → {a.desk} 로 이동. ENTER (s = 건너뜀): ") != "s":
                    nav(a, a.desk)
            elif s == "lift_up":
                ask("게임패드로 리프트를 올려 책상 위가 보이게 (RB+Y/X) → ENTER: ", allow_skip=False, always=True)
            elif s == "find":
                v = find(a, [a.word, a.basket])
                ob, bk = v["objects"].get(q_obj), v["objects"].get(q_bk)
                if not ob or ob.get("support_z") is None or ob.get("desk_front_x") is None:
                    raise Stop(f"'{q_obj}' 를 못 찾았거나 책상 면·앞 모서리를 못 잼 (Jetson /tmp/m_find.jpg)")
                if not bk:
                    raise Stop(f"'{q_bk}' 를 못 찾음 (Jetson /tmp/m_find.jpg)")
                st.update(obj=ob, bk=bk, desk=ob["support_z"], front=ob["desk_front_x"], zed=ob["zed_height"])
                c, b = ob["center"], bk["center"]
                print(f"   {q_obj} ({100 * c[0]:.1f}, {100 * c[1]:.1f}, {100 * c[2]:.1f}) cm, {q_bk} ({100 * b[0]:.1f}, {100 * b[1]:.1f}) cm "
                      f"yaw {math.degrees(bk['yaw'] or 0):.0f}°, 책상 {100 * st['desk']:.1f} cm·앞 {100 * st['front']:.1f} cm, ZED {st['zed']:.3f} m")
            elif s == "pick":
                need(st, "obj")
                c = st["obj"]["center"]
                p = plan(a, arm, "pick", (c[0], c[1], c[2] + a.dz), st["desk"], st["front"], c[1], st["zed"], a.descend)
                if not p or ask("   집기: 열기 → 6 cm 위 → 내려와 닫기 → 곧게 올림. ENTER: ") == "s":
                    raise Stop("집기 계획 실패" if not p else "집기 건너뜀")
                tl, t = arm.run(p, "집기", grip_before="open", grip_after="close")
                g = arm.gripper()
                if g >= arm.pargs.grip_empty_deg:
                    arm.back(tl, t, phases=("insert",), label="올림")
                    raise Stop(f"빈손 ({g:.0f}°) — 물체를 다시 확인 (--from find)")
                arm.back(tl, t, phases=("insert",), label="올림")
            elif s == "drop":
                need(st, "bk")
                b = st["bk"]["center"]
                top = st["desk"] + H
                p = plan(a, arm, "drop", (b[0], b[1], top + DROP_ABOVE_RIM), st["desk"], st["front"], b[1], st["zed"],
                         a.descend + 0.03)
                if not p or ask("   바구니 위 → 내려와 열기 → 올림. ENTER: ") == "s":
                    raise Stop("넣기 계획 실패" if not p else "넣기 건너뜀")
                tl, t = arm.run(p, "넣기", grip_after="open")
                arm.back(tl, t, phases=("insert",), label="올림")
            elif s == "basket_grab":
                need(st, "bk")
                b, yaw = st["bk"]["center"], st["bk"].get("yaw") or 0.0
                u = np.array([math.cos(yaw), math.sin(yaw)])
                v_ = np.array([-u[1], u[0]])                          # 짧은 벽이 뻗은 방향
                top = st["desk"] + H
                walls = sorted([np.array(b[:2]) + k * (L / 2 - WALL / 2) * u for k in (-1, 1)], key=lambda w: w[0])
                p = None
                for wxy in walls:                                       # 로봇에 가까운 벽부터
                    for hd in ("1,0", f"{v_[0]:.3f},{v_[1]:.3f}", f"{-v_[0]:.3f},{-v_[1]:.3f}"):
                        p = plan(a, arm, "grab", (wxy[0], wxy[1], top - GRIP_BELOW_RIM), st["desk"], st["front"], wxy[1],
                                 st["zed"], a.descend, extra=["--tilt=70,89", f"--heading={hd}",
                                                              f"--finger-perp={v_[0]:.3f},{v_[1]:.3f}"])
                        if p:
                            break
                    if p:
                        break
                if not p or ask("   바구니 벽: 열기 → 위에서 내려와 닫기 → 6 cm 올림. ENTER: ") == "s":
                    raise Stop("바구니 잡기 계획 실패" if not p else "바구니 잡기 건너뜀")
                tl, t = arm.run(p, "바구니 잡기", grip_before="open", grip_after="close")
                st["grab_axes"] = tool_axes(p)
                print(f"   그리퍼 {arm.gripper():.0f}° (벽이 얇으면 거의 닫힘 — 눈으로 확인)")
                if ask("   바구니가 손가락 사이에 잘 끼었나? ENTER = 들어 올림, q = 멈춤: ", allow_skip=False, always=True) == "":
                    arm.back(tl, t, phases=("insert",), label="바구니 들기")
            elif s == "carry":
                need(st, "desk")
                p = None
                for x, y, dz in CARRY:
                    p = plan(a, arm, "carry", (x, y, st["desk"] + dz), st["desk"], st["front"], 0.0, st["zed"])
                    if p:
                        break
                if not p or ask("   바구니를 든 채 오른쪽 앞 (카메라 시야 밖) 으로. ENTER: ") == "s":
                    raise Stop("들고 다닐 자세 계획 실패" if not p else "건너뜀")
                arm.run(p, "들고 다닐 자세")
            elif s == "lift_down":
                ask("게임패드로 리프트를 최저로 (RB+A/B) — 바구니가 차체에 닿지 않는지 보면서 → ENTER: ",
                    allow_skip=False, always=True)
            elif s == "nav_shelf":
                if ask(f"{a.shelf_view or '-'} → {a.shelf} 로 이동. ENTER (s = 건너뜀): ") != "s":
                    if a.shelf_view:
                        nav(a, a.shelf_view)
                    nav(a, a.shelf)
            elif s == "lift_place":
                ask(f"게임패드로 리프트를 올려 놓을 판 ({100 * a.board:.0f} cm) 위가 보이게 → ENTER: ", allow_skip=False, always=True)
                v = find(a, [])
                pl = [p_ for p_ in v.get("planes", []) if abs(p_["z"] - a.board) < 0.06]
                if not pl:
                    raise Stop(f"{100 * a.board:.0f} cm ±6 cm 판을 못 찾음: 잰 수평면 "
                               + ", ".join(f"{100 * p_['z']:.1f}" for p_ in v.get("planes", [])))
                bd = min(pl, key=lambda p_: abs(p_["z"] - a.board))
                if bd.get("front_x") is None:
                    raise Stop(f"판 {100 * bd['z']:.1f} cm 의 앞 모서리를 못 잼")
                st.update(board=bd["z"], board_front=bd["front_x"], zed_shelf=v["zed_height"])
                print(f"   판 {100 * bd['z']:.1f} cm, 앞 모서리 {100 * bd['front_x']:.1f} cm, ZED {v['zed_height']:.3f} m")
            elif s == "place":
                need(st, "board")
                tip = (a.place[0], a.place[1], st["board"] + H - GRIP_BELOW_RIM + PLACE_CLEAR)
                extra = []
                if "grab_axes" in st:                                  # 잡을 때 기울기 그대로 (바구니가 수평으로)
                    ax = st["grab_axes"][0]
                    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -ax[2]))))
                    hz = ax[:2] / max(1e-6, float(np.linalg.norm(ax[:2])))
                    extra = [f"--tilt={max(-10.0, pitch - 4):.1f},{min(89.5, pitch + 4):.1f}",
                             f"--heading={hz[0]:.3f},{hz[1]:.3f}"]
                p = plan(a, arm, "place", tip, st["board"], st["board_front"], a.place[1], st["zed_shelf"], a.descend, extra)
                if not p and extra:
                    p = plan(a, arm, "place", tip, st["board"], st["board_front"], a.place[1], st["zed_shelf"], a.descend,
                             ["--tilt=60,89"])
                if not p or ask("   판 위 → 내려와 열기 → 올림 → 들고 다닐 자세로. ENTER: ") == "s":
                    raise Stop("놓기 계획 실패" if not p else "놓기 건너뜀")
                tl, t = arm.run(p, "놓기", grip_after="open")
                arm.back(tl, t, phases=("insert",), label="올림")
                arm.back(tl, tl.phase_end("approach"), label="빠지기")
            elif s == "finish":
                ask("게임패드로 리프트를 최저로 → ENTER (끝): ", allow_skip=False, always=True)
        print("\n임무 끝.")
    except Stop as e:
        print(f"\n■ 멈춤: {e}. 팔은 지금 자세 유지. 다시: --from <단계> (위 === 이름 ===)")
    finally:
        arm.close()
    return 0


def need(st, k):
    if k not in st:
        raise Stop(f"앞 단계 결과 ({k}) 가 없음 — find (선반이면 lift_place) 단계부터 같이 돌리세요")


if __name__ == "__main__":
    sys.exit(main())
