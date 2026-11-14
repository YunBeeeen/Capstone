#!/usr/bin/env python3
"""선반 기억 2 단계 시뮬 시험 — run_shelf_test.sh 가 Gazebo·slam·(nav) 을 띄운 뒤 부름.

  python3 shelf_test.py map <출력>     # 1 일차: 참값으로 주행하며 매핑, 설 자리·보는 자리·home 저장
  python3 shelf_test.py run <출력>     # 2 일차: 선반을 옮겨 가며 place_go.py → 참값으로 "선반 기준 로봇 위치" 비교
  python3 shelf_test.py run4 <출력>    # 2 일차: 선반이 다른 곳(같은 복도 멀리 / 방 안 / 다른 복도)에 있음 → place_find.py 로 찾기
                                       #   (1 일차를 SHELF_MAP=wide 로 — 복도 한 바퀴 + 휴게실까지 매핑해야 함)
  python3 shelf_test.py run3 <출력>    # 2 일차: 정면이 막혔을 때 옆면으로 (한쪽 옆도 막힘 / 선반 옮겨짐 / 전부 막힘)
  python3 shelf_test.py run2 <출력>    # 2 일차: 물건 바뀜 / 빈 선반 / 사람이 가림(1/3, 절반, 전부) / 비킴 / 접근 중 끼어듦
선반(ladder_shelf)은 월드 (4.3, 2.24, 0°) — 남쪽 복도, 방 블록 남쪽 벽에 붙임. 앞면이 남쪽(−y).
평가값 = 선반 좌표계에서 본 로봇 위치가 1 일차에 저장할 때와 얼마나 다른가 (팔 작업에 필요한 건 이것).
※ 시뮬은 depth 가 깨끗하고 조명이 일정 — "논리가 맞다" 까지만. 정확도 수치는 실기에서 다시.
"""
import json
import os
import re
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.node import Node

TOOLS = Path(__file__).resolve().parents[2] / "navigation" / "tools"
sys.path.insert(0, str(TOOLS))
import places  # noqa: E402

SHELF0 = (4.3, 2.24, 0.0)                      # 선반 중심 (world)
STAND = (4.3, 2.24 - 0.165 - 0.12 - 0.26, math.pi / 2)   # 앞면에서 12 cm + 실기 외곽 앞 0.26 m
VIEW = (4.3, STAND[1] - 0.85, math.pi / 2)
HOME = (2.0, 1.25, 0.0)
# 2 일차 시험: (이름, 선반을 SHELF0 에서 (dx, dy, dyaw°) 만큼 옮김 또는 None = 치움, 기대 종료 코드)
CASES = [("그대로", (0, 0, 0), 0),
         ("옮김 A: 오른쪽 10 cm, 앞으로 5 cm, +6°", (0.10, -0.05, 6), 0),
         ("옮김 A 그대로 (기억 고친 뒤 다시)", (0.10, -0.05, 6), 0),
         ("옮김 B: 왼쪽 15 cm, 앞으로 8 cm, −8°", (-0.15, -0.08, -8), 0),
         ("치움", None, 2),
         ("옮김 B 자리에 다시 놓음", (-0.15, -0.08, -8), 0)]


PARK = {"cover_panel": (66.0, 60.0, 0.0), "person_box2": (62.0, 60.0, 0.0), "person_box3": (64.0, 60.0, 0.0), "ladder_shelf": (50.0, 50.0, 0.0), "ladder_shelf_alt": (52.0, 50.0, 0.0), "ladder_shelf_empty": (54.0, 50.0, 0.0),
        "person_box": (60.0, 60.0, 0.0)}
PERSON_Y = 1.78                                # 선반 앞면(2.075)에서 0.3 m 앞, 보는 자리 카메라에서 0.93 m
# run2: shelf = 어느 선반 모델, d = 옮김, person = 사람의 world x 오프셋 (로봇이 볼 때 −가 왼쪽), leave = 로봇이 보는 자리에 선 뒤 몇 초 후 비킴
CASES2 = [dict(name="1. 그대로"),
          dict(name="2. 물건이 바뀜", shelf="ladder_shelf_alt", args=["--allow-mid"]),
          dict(name="3. 물건 없음 (빈 선반)", shelf="ladder_shelf_empty", args=["--allow-mid"]),
          dict(name="4. 물건 바뀜 + 10 cm·6° 옮김", shelf="ladder_shelf_alt", d=(0.10, -0.05, 6), args=["--allow-mid"]),
          dict(name="5. 사람이 왼쪽 1/3 가림", person=-0.298, expect=6),
          dict(name="6. 사람이 절반 가림", person=-0.225, expect=6),
          dict(name="7. 사람이 전부 가림", person=0.0, expect=6),
          dict(name="8. 전부 가렸다가 8 초 뒤 비킴", person=0.0, leave=8.0, args=["--wait-blocked", "40"]),
          dict(name="9. 접근 중 사람이 끼어듦", intrude=True, expect=6),
          # 서고 → (로봇이 서 있는 동안 물건이 바뀜) → --leave 로 물러나며 기준 사진 갱신 → 다음 경우에서 바뀐 물건으로 다시
          dict(name="10. 선 뒤 물건이 바뀜 → 물러나며 사진 갱신", swap_leave="ladder_shelf_alt"),
          dict(name="11. 바뀐 물건인 채로 다시 (갱신한 사진)", shelf="ladder_shelf_alt"),
          # 못 알아보는 상황 (빈 선반, --allow-mid 없음) 에서 --go-anyway: 기억한 자리로 감
          dict(name="12. 빈 선반 + 못 알아봐도 감", shelf="ladder_shelf_empty", args=["--go-anyway"]),
          dict(name="13. 선반 치움 + 못 알아봐도 감", shelf="ladder_shelf_empty", d=(40, 40, 0), args=["--go-anyway"]),
          dict(name="14. 못 알아봄 + 앞에 사람 → 안 감", shelf="ladder_shelf_empty", person=0.0, args=["--go-anyway"], expect=6)]


# run3: front = 정면 사람, left/right = 그쪽 옆면 가는 길에 사람 (world 좌표). side = 기대하는 결과 ("front" / "left" / "right" / None = 못 감)
SIDE_BLOCK = {"left": (3.05, 2.05), "right": (5.55, 1.95)}
CASES3 = [dict(name="1. 정면이 비어 있음", side="front"),
          dict(name="2. 정면에 사람", front=True, side=("left", "right")),
          dict(name="3. 정면 + 왼쪽 옆에 사람", front=True, block=["left"], side="right"),
          dict(name="4. 정면 + 오른쪽 옆에 사람", front=True, block=["right"], side="left"),
          dict(name="5. 정면에 사람 + 선반 10 cm·6° 옮겨짐", front=True, d=(0.10, -0.05, 6), side=("left", "right")),
          dict(name="6. 정면·양옆 전부 사람", front=True, block=["left", "right"], side=None, expect=6),
          dict(name="7. 접근 중 정면에 끼어듦", intrude=True, side=("left", "right")),
          dict(name="8. 가림판이 선반을 완전히 가림", cover=True, side=("left", "right")),
          dict(name="9. 가림판만 있고 선반은 치워짐", cover=True, gone=True, side=None, expect=6),
          dict(name="10. 두 사람이 옆 통로까지 넓게 막음", front=True, wide=True, side=None, expect=6)]


# run4: 선반을 옮긴 곳 (world x, y, yaw°). 앞면 = 로컬 −y
CASES4 = [dict(name="1. 같은 복도 2.7 m 옆", shelf=(7.0, 2.24, 0)),
          dict(name="2. 휴게실 안 (서쪽 벽)", shelf=(13.26, 3.3, 90)),
          dict(name="3. 북쪽 복도 (반대편)", shelf=(12.0, 9.76, 180)),
          dict(name="4. 건물에서 치움", shelf=None, expect=2)]


class T(Node):
    def __init__(self):
        super().__init__("shelf_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.gt = None
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "sim/ground_truth", self.on_gt, 20)
        self.set_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.cap_pub = self.create_publisher(String, "/test/caption", 10)

    def caption(self, text):
        self.cap_pub.publish(String(data=text))

    def move(self, name, pose):
        st = EntityState(name=name)
        st.pose.position.x, st.pose.position.y = float(pose[0]), float(pose[1])
        st.pose.orientation.z, st.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.set_cli.wait_for_service(timeout_sec=10.0)
        f = self.set_cli.call_async(SetEntityState.Request(state=st))
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, places.yaw_q(p.orientation))

    def wait(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def goto(self, target, v=0.25, w=0.5):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 120:
            rclpy.spin_once(self, timeout_sec=0.03)
            e = places.rel(self.gt, target)
            d = math.hypot(e[0], e[1])
            if d < 0.01 and abs(e[2]) < 0.01:
                break
            c = Twist()
            s = min(v, 1.5 * d)
            if d > 1e-4:
                c.linear.x, c.linear.y = s * e[0] / d, s * e[1] / d
            c.angular.z = max(-w, min(w, 1.5 * e[2]))
            self.pub.publish(c)
        self.pub.publish(Twist())
        self.wait(1.0)

    def spin360(self):
        turned, prev = 0.0, self.gt[2]
        while turned < 2 * math.pi - 0.05:
            rclpy.spin_once(self, timeout_sec=0.03)
            turned += abs(places.wrap(self.gt[2] - prev))
            prev = self.gt[2]
            c = Twist()
            c.angular.z = 0.4
            self.pub.publish(c)
        self.pub.publish(Twist())
        self.wait(0.5)

    def move_shelf(self, pose):
        st = EntityState(name="ladder_shelf")
        st.pose.position.x, st.pose.position.y = float(pose[0]), float(pose[1])
        st.pose.orientation.z, st.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.set_cli.wait_for_service(timeout_sec=10.0)
        f = self.set_cli.call_async(SetEntityState.Request(state=st))
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        self.wait(1.0)


def tool(name, *args, timeout=600):
    r = subprocess.run([sys.executable, str(TOOLS / name), *map(str, args)], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def do_map(t, out):
    t.spin360()
    t.goto((4.3, 1.25, 0.0)); t.spin360()
    t.goto((4.3, 1.25, math.pi / 2))
    t.goto(STAND); t.wait(2.0)
    print(tool("place_mark.py", "shelf_T")[1])
    ref = places.rel(SHELF0, t.gt)
    t.goto(VIEW); t.wait(2.0)
    print(tool("place_mark.py", "shelf_T", "--view")[1])
    t.goto((4.3, 1.25, math.pi / 2)); t.goto((4.3, 1.25, 0.0))
    t.goto((7.5, 1.25, 0.0)); t.spin360()
    if os.environ.get("SHELF_MAP") == "wide":         # 복도 한 바퀴 + 휴게실 (선반 찾기 시험용)
        E, N, W_ = 0.0, math.pi / 2, math.pi
        for wp in ((15.0, 1.25, E), "spin", (15.0, 1.25, N), (15.0, 3.8, N), "spin", (15.0, 3.8, -N), (15.0, 1.25, -N),
                   (15.0, 1.25, E), (18.75, 1.25, E), (18.75, 1.25, N), (18.75, 10.75, N), "spin", (18.75, 10.75, W_),
                   (12.0, 10.75, W_), "spin", (6.0, 10.75, W_), "spin", (1.25, 10.75, W_), (1.25, 10.75, -N),
                   (1.25, 6.0, -N), "spin", (1.25, 1.25, -N), (1.25, 1.25, E), "spin", (7.5, 1.25, E)):
            t.spin360() if wp == "spin" else t.goto(wp, v=0.3)
    t.goto((7.5, 1.25, math.pi)); t.goto((2.0, 1.25, math.pi)); t.spin360()
    t.goto(HOME); t.wait(2.0)
    print(tool("place_mark.py", "home", "--kind", "start", "--no-image")[1])
    (out / "ref.json").write_text(json.dumps({"robot_in_shelf": ref}))
    print(f"기준: 선반 좌표계에서 로봇 ({ref[0]:.3f}, {ref[1]:.3f}, {math.degrees(ref[2]):.1f}°)")


def do_run(t, out):
    ref = json.loads((out / "ref.json").read_text())["robot_in_shelf"]
    rows = []
    for name, d, expect in CASES:
        shelf = (50.0, 50.0, 0.0) if d is None else (SHELF0[0] + d[0], SHELF0[1] + d[1], math.radians(d[2]))
        t.move_shelf(shelf)
        print(f"\n===== {name}", flush=True)
        t0 = time.monotonic()
        code, text = tool("place_go.py", "shelf_T", "--cmd-topic", "/cmd_vel")
        print(text)
        t.wait(1.0)
        line = f"종료 코드 {code} (기대 {expect}), {time.monotonic() - t0:.0f} s"
        if d is not None:
            e = places.rel(ref, places.rel(shelf, t.gt))       # 저장 때의 상대 위치 기준 오차
            line += f" | 선반 기준 로봇 오차(참값): 앞뒤 {e[0] * 100:+.1f} cm, 좌우 {e[1] * 100:+.1f} cm, {math.degrees(e[2]):+.1f}°"
            rows.append((name, code, expect, e))
        else:
            rows.append((name, code, expect, None))
        print("→ " + line, flush=True)
        code, text = tool("nav_goal_ahead.py", "--name", "home")
        t.wait(1.0)
    print("\n===== 요약")
    for name, code, expect, e in rows:
        es = f"{math.hypot(e[0], e[1]) * 100:4.1f} cm, {math.degrees(e[2]):+.1f}°" if e and code == 0 else "-"
        print(f"  {'OK ' if code == expect else 'NG '} {name:40s} 코드 {code}  선반 기준 오차 {es}")
    (out / "result.json").write_text(json.dumps([(n, c, x, e) for n, c, x, e in rows], ensure_ascii=False))


def do_run2(t, out):
    ref = json.loads((out / "ref.json").read_text())["robot_in_shelf"]
    rows = []
    only = os.environ.get("SHELF_CASES")                      # 예: SHELF_CASES=1,9 → 그 번호만
    for c in [c for c in CASES2 if not only or c["name"].split(".")[0] in only.split(",")]:
        model, d, expect = c.get("shelf", "ladder_shelf"), c.get("d", (0, 0, 0)), c.get("expect", 0)
        shelf = (SHELF0[0] + d[0], SHELF0[1] + d[1], math.radians(d[2]))
        for k, p in PARK.items():
            t.move(k, shelf if k == model else p)
        if "person" in c:
            t.move("person_box", (SHELF0[0] + c["person"], PERSON_Y, 0.0))
        t.wait(1.0)
        t.caption(c["name"])
        print(f"\n===== {c['name']}", flush=True)
        proc = subprocess.Popen([sys.executable, str(TOOLS / "place_go.py"), "shelf_T", "--cmd-topic", "/cmd_vel", "--no-update",
                                 "--side", "none", *c.get("args", [])], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        at_view, done = None, False
        while proc.poll() is None:
            rclpy.spin_once(t, timeout_sec=0.05)
            if at_view is None and math.hypot(t.gt[0] - VIEW[0], t.gt[1] - VIEW[1]) < 0.20:
                at_view = time.monotonic()
            if at_view and not done:
                if "leave" in c and time.monotonic() - at_view > c["leave"] + 3.0:
                    t.move("person_box", PARK["person_box"]); done = True
                    t.caption(c["name"] + " — 비킴")
                if c.get("intrude") and t.gt[1] > VIEW[1] + 0.25:
                    t.move("person_box", (SHELF0[0], PERSON_Y + 0.04, 0.0)); done = True
                    t.caption(c["name"] + " — 지금 끼어듦")
        text = proc.stdout.read().strip()
        code = proc.returncode
        print(text)
        t.wait(1.0)
        stand_w = places.compose(shelf, ref)                       # 참값: 지금 선반 기준 설 자리 (world)
        note = ""
        m = re.search(r"차체 기준 설 자리: 앞 ([-+.\d]+) m, 왼 ([-+.\d]+) m, ([-+.\d]+)°", text)
        if code == 0:
            e = places.rel(stand_w, t.gt)
            note = f"선 자리 오차(참값) {math.hypot(e[0], e[1]) * 100:.1f} cm, {math.degrees(e[2]):+.1f}°"
        elif m and "접근 중" not in text:                           # 안 움직였으면 잰 값 자체를 참값과 비교
            meas = (float(m[1]), float(m[2]), math.radians(float(m[3])))
            e = places.rel(places.rel(t.gt, stand_w), meas)
            note = f"잰 값 오차(참값) {math.hypot(e[0], e[1]) * 100:.1f} cm, {math.degrees(e[2]):+.1f}°"
        if c.get("intrude"):
            note += f" 멈춘 곳: 사람 앞면까지 {PERSON_Y + 0.04 - 0.12 - (t.gt[1] + 0.26):.2f} m (차체 앞 기준)"
        conf = re.findall(r"선반 확인: (\S+)", text)
        res = f"코드 {code} (기대 {expect}), 판정 {conf[-1] if conf else '?'}, {note}"
        print("→ " + res, flush=True)
        t.caption(f"{c['name']}  →  {'섰음' if code == 0 else {6: '막힘·가려짐 → 멈춤', 2: '없음 → 멈춤', 4: '확신 낮음 → 멈춤'}.get(code, f'코드 {code}')}  ({note})")
        t.wait(3.0)
        if c.get("swap_leave") and code == 0:                      # 선 채로 선반을 다른 물건 것으로 바꾸고 --leave
            t.move(model, PARK[model]); t.move(c["swap_leave"], shelf); t.wait(1.0)
            lc, ltext = tool("place_go.py", "shelf_T", "--cmd-topic", "/cmd_vel", "--leave")
            print(ltext)
            res += f" | --leave 코드 {lc}"
            code = code or lc
        rows.append((c["name"], code == expect, res))
        t.move("person_box", PARK["person_box"])
        t.caption("home 으로 돌아감")
        tool("nav_goal_ahead.py", "--name", "home")
        t.wait(1.0)
    print("\n===== 요약")
    for name, ok, res in rows:
        print(f"  {'OK ' if ok else 'NG '} {name:28s} {res}")
    (out / "result2.json").write_text(json.dumps(rows, ensure_ascii=False))


def do_run3(t, out):
    import shelf_check as sc
    ref = json.loads((out / "ref.json").read_text())["robot_in_shelf"]
    pl = places.load(out / "map.places.yaml")
    sides = sc.side_stands(pl["shelf_T"]["view"]["gate"])
    rows = []
    only = os.environ.get("SHELF_CASES")
    for c in [c for c in CASES3 if not only or c["name"].split(".")[0] in only.split(",")]:
        d, expect = c.get("d", (0, 0, 0)), c.get("expect", 0)
        shelf = (SHELF0[0] + d[0], SHELF0[1] + d[1], math.radians(d[2]))
        for k, p in PARK.items():
            t.move(k, shelf if k == "ladder_shelf" else p)
        if c.get("gone"):
            t.move("ladder_shelf", PARK["ladder_shelf"])
        if c.get("cover"):                                         # 선반 앞면 8 cm 앞, 선반 폭 — 옆 통로는 안 막음
            t.move("cover_panel", (SHELF0[0], SHELF0[1] - 0.165 - 0.08 - 0.05, 0.0))
        if c.get("front"):
            t.move("person_box", (SHELF0[0] - (0.22 if c.get("wide") else 0.0), PERSON_Y - (0.25 if c.get("near") else 0.0), 0.0))
        if c.get("wide"):
            t.move("person_box2", (SHELF0[0] + 0.22, PERSON_Y, 0.0))
        for k, name in zip(c.get("block", []), ("person_box2", "person_box3")):
            t.move(name, (*SIDE_BLOCK[k], 0.0))
        t.wait(1.0)
        t.caption(c["name"])
        print(f"\n===== {c['name']}", flush=True)
        proc = subprocess.Popen([sys.executable, str(TOOLS / "place_go.py"), "shelf_T", "--cmd-topic", "/cmd_vel", "--no-update",
                                 "--allow-mid"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        at_view, done = None, False
        while proc.poll() is None:
            rclpy.spin_once(t, timeout_sec=0.05)
            if at_view is None and math.hypot(t.gt[0] - VIEW[0], t.gt[1] - VIEW[1]) < 0.20:
                at_view = True
            if c.get("intrude") and at_view and not done and t.gt[1] > VIEW[1] + 0.25:
                t.move("person_box", (SHELF0[0], PERSON_Y + 0.04, 0.0)); done = True
                t.caption(c["name"] + " — 지금 끼어듦")
        text = proc.stdout.read().strip()
        code = proc.returncode
        print(text)
        t.wait(1.0)
        stand_w = places.compose(shelf, ref)                       # 참값: 지금 선반 기준 정면 설 자리 (world)
        cand = {"front": stand_w, **{k: places.compose(stand_w, v) for k, v in sides.items()}}
        got, note = None, ""
        if code == 0:
            got = min(cand, key=lambda k: math.hypot(t.gt[0] - cand[k][0], t.gt[1] - cand[k][1]))
            e = places.rel(cand[got], t.gt)
            note = f"{got} 에 섬, 오차(참값) {math.hypot(e[0], e[1]) * 100:.1f} cm, {math.degrees(e[2]):+.1f}°"
        want = c["side"] if isinstance(c["side"], tuple) else (c["side"],)
        ok = code == expect and (got in want if code == 0 else True)
        res = f"코드 {code} (기대 {expect}, {'/'.join(str(w) for w in want)}), {note}"
        print("→ " + res, flush=True)
        t.caption(f"{c['name']}  →  {note if code == 0 else '갈 곳 없음 → 멈춤' if code == 6 else f'코드 {code}'}")
        t.wait(3.0)
        rows.append((c["name"], ok, res))
        for k, p in PARK.items():
            if k.startswith("person") or k == "cover_panel":
                t.move(k, p)
        t.caption("home 으로 돌아감")
        tool("nav_goal_ahead.py", "--name", "home")
        t.wait(1.0)
    print("\n===== 요약")
    for name, ok, res in rows:
        print(f"  {'OK ' if ok else 'NG '} {name:34s} {res}")
    (out / "result3.json").write_text(json.dumps(rows, ensure_ascii=False))


def do_run4(t, out):
    ref = json.loads((out / "ref.json").read_text())["robot_in_shelf"]
    day1 = (out / "map.places.day1.yaml").read_text()
    rows = []
    only = os.environ.get("SHELF_CASES")
    for c in [c for c in CASES4 if not only or c["name"].split(".")[0] in only.split(",")]:
        (out / "map.places.yaml").write_text(day1)                 # 경우마다 1 일차 기억으로 되돌림
        expect = c.get("expect", 0)
        shelf = None if c["shelf"] is None else (c["shelf"][0], c["shelf"][1], math.radians(c["shelf"][2]))
        t.move("ladder_shelf", shelf or PARK["ladder_shelf"])
        t.wait(1.0)
        print(f"\n===== {c['name']}", flush=True)
        t.caption(c["name"] + " — 먼저 기억한 자리로")
        code0, text = tool("place_go.py", "shelf_T", "--cmd-topic", "/cmd_vel", "--side", "none")
        print(text.splitlines()[-1] if text else "", f"(place_go 코드 {code0})", flush=True)
        t.caption(c["name"] + " — 기억한 자리에 없음 → 찾는 중")
        t0 = time.monotonic()
        d0, last = 0.0, t.gt
        flog = out / f"find_{c['name'].split('.')[0]}.log"       # 진행을 바로 볼 수 있게 파일로
        proc = subprocess.Popen([sys.executable, "-u", str(TOOLS / "place_find.py"), "shelf_T", "--cmd-topic", "/cmd_vel",
                                 *os.environ.get("FIND_ARGS", "").split()], stdout=open(flog, "w"), stderr=subprocess.STDOUT)
        while proc.poll() is None:
            rclpy.spin_once(t, timeout_sec=0.05)
            d0 += math.hypot(t.gt[0] - last[0], t.gt[1] - last[1]); last = t.gt
            if time.monotonic() - t0 > 1500:
                proc.kill()
        text = flog.read_text().strip()
        code = proc.returncode
        print(text)
        note = f"찾기 {time.monotonic() - t0:.0f} s, 주행 {d0:.0f} m"
        if code == 0 and shelf:
            t.caption(c["name"] + " — 찾음 → 가서 서기")
            code2, text2 = tool("place_go.py", "shelf_T", "--cmd-topic", "/cmd_vel", "--no-update")
            print(text2)
            t.wait(1.0)
            e = places.rel(places.compose(shelf, ref), t.gt)
            note += f", 선 자리 오차(참값) {math.hypot(e[0], e[1]) * 100:.1f} cm, {math.degrees(e[2]):+.1f}° (place_go 코드 {code2})"
            ok = code2 == 0 and math.hypot(e[0], e[1]) < 0.05
        else:
            ok = code == expect
        res = f"place_find 코드 {code} (기대 {expect}), {note}"
        print("→ " + res, flush=True)
        t.caption(f"{c['name']}  →  {'찾아서 섬' if code == 0 else '못 찾음'}  ({note})")
        t.wait(3.0)
        rows.append((c["name"], ok, res))
        t.caption("home 으로 돌아감")
        tool("nav_goal_ahead.py", "--name", "home", timeout=600)
        t.wait(1.0)
    print("\n===== 요약")
    for name, ok, res in rows:
        print(f"  {'OK ' if ok else 'NG '} {name:24s} {res}")
    (out / "result4.json").write_text(json.dumps(rows, ensure_ascii=False))


def main():
    mode, out = sys.argv[1], Path(sys.argv[2])
    rclpy.init()
    t = T()
    while t.gt is None:
        rclpy.spin_once(t, timeout_sec=0.1)
    {"map": do_map, "run": do_run, "run2": do_run2, "run3": do_run3, "run4": do_run4}[mode](t, out)


if __name__ == "__main__":
    main()
