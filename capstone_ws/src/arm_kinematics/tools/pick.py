#!/usr/bin/env python3
"""물체 이름 → 찾기 (Jetson ZED + OWLv2·SAM) → 계획 (위에서 내려와 집기) → 실행 (팔로워). 라즈베리파이에서 실행.

  cd ~/arm_play/ws/arm_kinematics
  PY=~/rebot_env/venv/bin/python
  $PY tools/pick.py pumpkin --hover          # 첫 시험: 물체 6 cm 위까지만 가서 멈춤 (그리퍼 안 씀) → 정렬 눈으로 확인
  $PY tools/pick.py pumpkin                  # 그리퍼 열고 → 6 cm 위 → 곧게 내려옴 → 닫기 → (ENTER) 6 cm 들어 올리며 복귀
  $PY tools/pick.py "a toy corn" --dz -0.01  # 목표를 1 cm 낮게 (손가락이 더 깊이)

순서:
  1) 지금 팔 자세를 읽기 전용으로 읽음 (arm_watch, 토크 안 켬) → 계획의 시작 자세
  2) Jetson 에서 svo_find --live --zed-height auto (카메라 높이·책상 높이·앞 모서리를 화면에서 잼)
  3) ik_sim_run --descend (책상 = 잰 높이·앞 모서리, 받침면 여유 0.5 cm) → /tmp/pick_plan.json
  4) 요약을 보여 주고 y 를 입력해야 움직임
  5) play_plan (시작 점검·enable 확인·Ctrl+C 복귀·막힘 정지 그대로)
목표 = 물체 중심 (+ --dz). IK 가 맞추는 점은 손가락 끝 — 위에서 내려오면 손가락 끝이 물체 중심 높이까지 내려옴.
Jetson 접속: 라파의 ~/.ssh/id_ed25519_jetson 으로 idle@192.168.1.11 (수집기와 같은 키).
"""
import argparse
import json
import re
import shlex
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = sys.executable
JETSON = ["ssh", "-i", str(Path.home() / ".ssh/id_ed25519_jetson"), "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes",
          "-o", "ConnectTimeout=8"]


def phrase(word):
    w = word.strip()
    return w if " " in w else ("an " if w[0].lower() in "aeiou" else "a ") + w


def run(cmd, **kw):
    """중간 단계 실행 — 입력은 막음 (ssh 등이 사용자 입력을 먼저 읽어 가지 않게)."""
    return subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL, **kw)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("word", help="찾을 물체 (한 단어면 a/an 을 붙임). 예: pumpkin, \"a toy corn\"")
    ap.add_argument("--hover", action="store_true", help="물체 위 접근점까지만 (그리퍼 안 씀)")
    ap.add_argument("--descend", type=float, default=0.06, help="접근점 높이 = 목표 위 [m]")
    ap.add_argument("--dz", type=float, default=0.0, help="목표 높이 보정 [m] (− = 더 깊이)")
    ap.add_argument("--min-score", type=float, default=0.15)
    ap.add_argument("--jetson", default="idle@192.168.1.11")
    ap.add_argument("--speed", type=float, default=1.0, help="play_plan 속도 배율 (1.0 = 팔로워 한계 30°/s·120°/s²)")
    ap.add_argument("--settle-deg", type=float, default=2.0, help="play_plan 끝 도착 인정 오차 [도]")
    ap.add_argument("--yes", action="store_true", help="확인 질문 없이 바로 움직임")
    a = ap.parse_args()
    q = phrase(a.word)

    # 1) 지금 팔 자세 (읽기 전용)
    r = run([PY, str(HERE / "arm_watch.py"), "--once"], timeout=60)
    m = re.search(r"--motors=([-\d.,]+)", r.stdout)
    if not m:
        print("팔 자세를 못 읽음 (CAN 이 올라와 있나? sudo ~/capstone/capstone_ws/can/can_up.sh)\n", r.stdout[-800:], r.stderr[-800:])
        return 1
    start = m.group(1)
    print(f"1) 지금 팔 자세 (모터 1~6): {start}")

    # 2) Jetson 에서 찾기
    remote = ("cd ~/arm_vision && PYTHONPATH=~/zedpy python3 tools/svo_find.py --live --zed-height auto "
              f"{shlex.quote(q)} --min-score {a.min_score} --json /tmp/pick_vision.json --out /tmp/pick_vision.jpg "
              "> /tmp/pick_vision.log 2>&1; tail -6 /tmp/pick_vision.log; echo ===JSON===; cat /tmp/pick_vision.json 2>/dev/null")
    print(f"2) Jetson 에서 '{q}' 찾는 중 …")
    r = run(JETSON + [a.jetson, remote], timeout=180)
    if "===JSON===" not in r.stdout:
        print("Jetson 접속·실행 실패:", r.stdout[-800:], r.stderr[-800:])
        return 1
    log, js = r.stdout.split("===JSON===", 1)
    print("   " + "\n   ".join(line for line in log.strip().splitlines() if line.strip()))
    try:
        obj = json.loads(js)["objects"].get(q)
    except (json.JSONDecodeError, KeyError):
        obj = None
    run(["scp", "-q", "-i", str(Path.home() / ".ssh/id_ed25519_jetson"), "-o", "IdentitiesOnly=yes",
         f"{a.jetson}:/tmp/pick_vision.jpg", "/tmp/pick_vision.jpg"], timeout=30)
    if not obj or obj.get("support_z") is None or obj.get("desk_front_x") is None:
        print(f"'{q}' 를 못 찾았거나 책상 면·앞 모서리를 못 잼 — 그림: /tmp/pick_vision.jpg (라파), 다른 표현으로 다시")
        return 2
    c = obj["center"]
    tgt = [c[0], c[1], c[2] + a.dz]

    # 3) 계획
    plan = "/tmp/pick_plan.json"
    cmd = [PY, str(HERE / "ik_sim_run.py"), "--target", "desk", "--desk-top", f"{obj['support_z']:.3f}",
           "--front", f"{obj['desk_front_x']:.3f}", f"--lateral={c[1]:.3f}",
           "--xyz", f"{tgt[0]:.3f}", f"{tgt[1]:.3f}", f"{tgt[2]:.3f}", "--zed-height", f"{obj['zed_height']:.3f}",
           f"--start={start}", "--no-sim", "--descend", f"{a.descend}", "--save-plan", plan]
    print("3) 계획:", " ".join(shlex.quote(x) for x in cmd[1:]))
    Path(plan).unlink(missing_ok=True)
    r = run(cmd, timeout=300)
    if r.returncode != 0 or not Path(plan).exists():
        tail = [line for line in r.stdout.splitlines() if any(k in line for k in ("실패", "없음", "불가", "충돌", "범위"))]
        print("   계획 실패:", "\n   ".join(tail[-6:]) or r.stdout[-600:])
        print("   → 물체가 로봇 오른쪽 앞 (앞 0.45~0.6 m, 오른쪽 0~0.25 m) 에 오게 로봇을 옮겨 보세요 (오른팔)")
        return 3
    sel = next((line.strip() for line in r.stdout.splitlines() if "리프트" in line and "부하" in line), "")
    dur = json.loads(Path(plan).read_text())["waypoints"][-1]["t"]

    # 4) 확인
    print(f"\n4) {q} (점수 {obj['score']:.2f}): 중심 ({100 * c[0]:.1f}, {100 * c[1]:.1f}, {100 * c[2]:.1f}) cm, "
          f"책상 {100 * obj['support_z']:.1f} cm·앞 모서리 {100 * obj['desk_front_x']:.1f} cm, ZED {obj['zed_height']:.3f} m")
    print(f"   목표 (손끝) ({100 * tgt[0]:.1f}, {100 * tgt[1]:.1f}, {100 * tgt[2]:.1f}) cm, 접근점 {100 * a.descend:.0f} cm 위, "
          f"계획 {dur:.1f} s (속도 배율 전), {sel}")
    print("   " + ("물체 위 접근점까지만 (--hover)" if a.hover else "그리퍼 열기 → 접근점 → 내려옴 → 닫기 → ENTER 로 들어 올리며 복귀"))
    print("   그림 (라파): /tmp/pick_vision.jpg")
    if not a.yes:
        try:
            ok = input("   움직일까요? y 입력: ").strip().lower() == "y"
        except EOFError:
            ok = False
        if not ok:
            print("취소 (움직이지 않음)")
            return 0

    # 5) 실행
    pp = [PY, str(HERE / "play_plan.py"), plan, "--speed", str(a.speed), "--settle-deg", str(a.settle_deg)]
    pp += ["--stop-at", "approach"] if a.hover else ["--grip-before", "open", "--grip-after", "close"]
    print("5)", " ".join(pp[1:]))
    return subprocess.call(pp)


if __name__ == "__main__":
    sys.exit(main())
