#!/usr/bin/env python3
"""기억한 선반 앞에 서기 (선반 기억 2 단계) — 보는 자리까지 Nav2 → 선반 확인 → 선반 기준으로 설 자리까지 천천히 접근.

  python3 src/navigation/tools/place_go.py shelf_A                 # 실기 (속도는 /nav/cmd_vel → 가상 패드 RT 를 누르고 있어야 움직임)
  python3 src/navigation/tools/place_go.py shelf_A --cmd-topic /cmd_vel    # 시뮬
  python3 src/navigation/tools/place_go.py shelf_A --check-only    # 보는 자리까지만 가서 재고 끝 (접근 안 함)
  python3 src/navigation/tools/place_go.py shelf_A --no-update     # 옮겨진 걸 찾아도 places.yaml 을 안 고침
  python3 src/navigation/tools/place_go.py shelf_A --go-anyway     # 못 알아봐도 기억한 자리로 감 (차체가 몇 cm 틀려도 되는 작업)
  python3 src/navigation/tools/place_go.py shelf_A --leave         # 팔 작업 뒤: 물러나면서 기준 사진을 새로 찍음 (물건이 바뀌는 선반)
먼저: place_mark.py shelf_A (설 자리) + place_mark.py shelf_A --view (0.8~1.2 m 뒤 보는 자리)

순서
  1. 보는 자리로 Nav2 (도착 오차 10 cm·6° 는 다음 단계가 흡수)
  2. shelf_check 로 재기 → 가려져서 확인 못 함: 멈춤 (종료 코드 6)
     / 선반은 보이는데 정면이 막힘: --wait-blocked 초 동안 비키기를 기다리고, 그래도 막혀 있으면 선반 옆면으로 (아래 5)
     / 없음: 멈춤 (2) / 너무 많이 옮겨짐(--max-shift): 멈춤 (3) / 낮은 확신: 멈춤 (4)
  3. 선반 기준 설 자리의 0.25 m 뒤로 가서 방향을 맞춘 다음 (옆·회전은 여기서), 똑바로 앞으로 접근. odom 좌표로 직접 제어
     (Nav2 를 안 쓰는 이유: 선반에 붙은 목표는 costmap 이 거절하기 쉽고 도착 허용 오차가 10 cm)
     가는 동안 depth 로 통로(차체 폭)를 계속 봄 — 뭔가 들어오면 바로 멈추고 종료 (6)
  4. 기억보다 4 cm·3° 넘게 옮겨졌고 확신이 '확실' 이면 places.yaml 의 설 자리·보는 자리를 새 위치로 고침
  5. 옆면: 선반을 벽에 붙은 상자로 보고 왼쪽·오른쪽 옆면 설 자리를 계산 (shelf_check.side_stands, 또는 place_mark --side 로 직접 저장한 자리).
     Nav2 costmap 에서 갈 수 있는 쪽을 골라 그 0.6 m 뒤까지 Nav2 → 통로 확인 → 접근. 둘 다 안 되면 멈춤 (6). --side none 이면 안 함
     옆면에 가서는 판의 옆 모서리로 한 번 더 맞춤 (기준 사진은 없고 선반 크기만 씀). 못 찾으면 정면에서 잰 위치 그대로
종료 코드 0 = 설 자리에 섬 (정면 또는 옆면), 5 = 접근 시간 초과.
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
import places  # noqa: E402
import shelf_check as sc  # noqa: E402


def nav_to(n, cli, pose):
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = pose[0], pose[1]
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
    h = places.send_goal(n, cli, g)
    if h is None or not h.accepted:
        return False
    res = h.get_result_async()
    try:
        while not res.done():
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        h.cancel_goal_async()
        rclpy.spin_once(n, timeout_sec=0.3)
        raise
    return res.result().status == 4


def drain(n, budget=0.02):
    """밀린 메시지를 budget 초 동안 전부 처리. spin_once 는 한 번에 콜백 하나만 처리해서, 루프당 한 번만 부르면
    TF·영상이 밀려 1 초 전 위치를 쓰게 됨 (시뮬 2026-11-13: 선반이 7 cm 가깝게 계산돼 '막힘' 오탐)."""
    end = time.time() + budget
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.002)


def odom_pose(n, mc):
    for _ in range(40):
        drain(n)
        try:
            t = mc.tf.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform
            return (t.translation.x, t.translation.y, places.yaw_q(t.rotation))
        except Exception:
            pass
    return None


def servo(n, mc, pub, target, vmax, wmax, tol_xy, tol_yaw, timeout, blocked=None):
    """odom 좌표 target (x, y, yaw) 까지 비례 제어. 반환 (True 도착 / False 시간 초과 / "blocked" 통로 막힘, 남은 오차 (앞, 왼, yaw)).
    blocked(cur_odom) → bool: 0.3 s 마다 불러서 두 번 연속 True 면 멈춤."""
    t0, ok_since, e, last_chk, hits = time.time(), None, None, 0.0, 0
    try:
        while time.time() - t0 < timeout:
            cur = odom_pose(n, mc)
            if cur is None:
                break
            if blocked and time.time() - last_chk > 0.3:
                last_chk = time.time()
                hits = hits + 1 if blocked(cur) else 0
                if hits >= 2:
                    return "blocked", places.rel(cur, target)
            e = places.rel(cur, target)
            d = math.hypot(e[0], e[1])
            if d < tol_xy and abs(e[2]) < tol_yaw:
                ok_since = ok_since or time.time()
                if time.time() - ok_since > 0.4:
                    return True, e
            else:
                ok_since = None
            v = min(vmax, 1.5 * d)
            cmd = Twist()
            if d > 1e-4:
                cmd.linear.x, cmd.linear.y = v * e[0] / d, v * e[1] / d
            cmd.angular.z = max(-wmax, min(wmax, 1.5 * e[2]))
            pub.publish(cmd)
    finally:
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.02)
    return False, e


def approach(n, mc, pub, target, x_end, speed):
    """지금 차체 기준 target (x, y, yaw) 에 서기: 0.25 m 뒤에서 옆·방향을 맞추고 똑바로 접근. 가는 동안 통로를 depth 로 감시.
    x_end = 통로 끝 (target 좌표에서 선반 면 7 cm 전). 반환 종료 코드 (0 도착 / 5 시간 초과 / 6 막힘)."""
    o = odom_pose(n, mc)
    if o is None:
        print("TF odom→base_footprint 없음")
        return 1
    goal = places.compose(o, target)                  # odom 기준 설 자리 (잰 순간의 차체 기준 → odom)
    pre = places.compose(goal, (-0.25, 0.0, 0.0))
    watch, seen = sc.DepthWatch(n, mc), {}

    def blocked_now(cur_odom):
        f = watch.frame()
        if not f:
            return False
        b, k, seen["where"] = sc.path_blocked(f, places.rel(cur_odom, goal), x_end)
        seen["left"] = places.rel(cur_odom, goal)[0]
        return b

    steps = ([(pre, 0.3, 0.015, 1.0, 60.0)] if target[0] > 0.30 else []) + [(goal, 0.2, 0.006, 0.5, 40.0)]
    e = None
    for tgt, wmax, tol, tol_deg, tmo in steps:
        ok, e = servo(n, mc, pub, tgt, speed, wmax, tol, math.radians(tol_deg), tmo, blocked_now)
        if ok == "blocked":
            print(f"  접근 중 앞에 뭔가 들어옴 → 멈춤 (설 자리까지 {seen.get('left', 0):.2f} m 남음. {seen.get('where', '')})")
            return 6
        if not ok:
            print("  시간 초과 (실기라면 RT 를 누르고 있었나?)" + (f" 남은 오차 앞 {e[0] * 100:+.1f} cm, 왼 {e[1] * 100:+.1f} cm" if e else ""))
            return 5
    print(f"  도착: 남은 오차 앞 {e[0] * 100:+.1f} cm, 왼 {e[1] * 100:+.1f} cm, {math.degrees(e[2]):+.1f}° (odom 기준)")
    return 0


class Costmap:
    """Nav2 전체 costmap (/global_costmap/costmap) 을 받아 두고, 어떤 자리에 차체 중심을 둘 수 있는지 봄."""

    def __init__(self, n):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.m = None
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "m", m), q)

    def cost(self, x, y):
        """map 좌표의 값: 0~98 갈 수 있음 (클수록 장애물에 가까움), 99 = 차체가 닿음, 100 = 장애물, -1 = 모르는 곳. 지도 밖·못 받음 = None."""
        if self.m is None:
            return None
        i = int((x - self.m.info.origin.position.x) / self.m.info.resolution)
        j = int((y - self.m.info.origin.position.y) / self.m.info.resolution)
        if not (0 <= i < self.m.info.width and 0 <= j < self.m.info.height):
            return None
        return self.m.data[j * self.m.info.width + i]


def try_sides(n, mc, cli, pub, cm, a, place, ref, stand_now, confirmed=True):
    """정면이 막혔을 때: 선반 옆면 설 자리로. stand_now = 방금 잰 정면 설 자리 (map). 반환 종료 코드 (0 = 옆면에 섬, 6 = 갈 수 있는 옆면 없음).
    옆면에 가서는 판의 옆 모서리로 한 번 더 맞춤 (shelf_check.side_refine).
    confirmed=False (정면에서 가려져 못 잼 → stand_now 는 기억한 자리): 옆 모서리를 찾아야만 접근 (선반이 거기 있다는 확인을 겸함)."""
    cand = sc.side_stands(ref["gate"])
    for k, v in (place.get("sides") or {}).items():           # 직접 저장한 옆면 자리가 있으면 그걸 씀
        cand[k] = (v[0], v[1], math.radians(v[2]))
    if a.side in cand:
        cand = {a.side: cand[a.side]}
    drain(n, 1.0)                                             # costmap 받을 시간
    plan = []
    for name, sp in cand.items():
        st = places.compose(stand_now, sp)                    # map 기준 옆면 설 자리
        vw = places.compose(st, (-a.side_back, 0.0, 0.0))     # 그 뒤 (Nav2 로 가는 곳)
        cs, cv = cm.cost(st[0], st[1]), cm.cost(vw[0], vw[1])
        ok = cs is not None and cv is not None and 0 <= cs < 99 and 0 <= cv < 99
        print(f"  옆면 {name}: 설 자리 map ({st[0]:.2f}, {st[1]:.2f}, {math.degrees(st[2]):.0f}°), costmap 값 설 자리 {cs} / 뒤 {cv}"
              + ("" if ok else "  → 갈 수 없음 (99 이상 = 장애물, -1 = 모르는 곳, None = costmap 없음)"))
        if ok:
            plan.append((max(cs, cv), name, sp, st, vw))
    x_end = sc.ROBOT_FRONT + (sc.shelf_box(ref["gate"])[0] - sc.ROBOT_FRONT) - 0.07      # 옆면까지 간격은 정면과 같음
    for _, name, sp, st, vw in sorted(plan):
        print(f"[3] 옆면 {name} 으로: 먼저 map ({vw[0]:.2f}, {vw[1]:.2f}, {math.degrees(vw[2]):.0f}°)")
        if not a.no_nav and not nav_to(n, cli, vw):
            print("  Nav2 실패 — 다음 옆면")
            continue
        time.sleep(1.5)
        cur = mc.robot_pose()
        frame = sc.grab(n, mc)
        if cur is None or frame is None:
            continue
        target = places.rel(cur, st)
        fixed, used = sc.side_refine(ref, frame, target, name) if name in ("left", "right") and name not in (place.get("sides") or {}) else (target, 0)
        d = places.rel(target, fixed)
        if os.environ.get("SHELF_DEBUG"):                 # 옆면에서 본 화면 저장 (원인 분석용)
            import numpy as np
            np.savez(os.path.join(os.environ["SHELF_DEBUG"], f"side_{name}_{int(time.time())}.npz"), depth=frame["depth"],
                     K=np.array(frame["K"]), T=frame["T"], target=np.array(target), bgr=frame["bgr"])
        if used >= 2 and math.hypot(d[0], d[1]) < (0.15 if confirmed else 0.25) and abs(d[2]) < math.radians(12):
            print(f"  옆 모서리로 다시 맞춤 (판 {used} 장): 앞 {d[0] * 100:+.1f} cm, 왼 {d[1] * 100:+.1f} cm, {math.degrees(d[2]):+.1f}°")
            target = fixed
        elif not confirmed:
            print(f"  옆에서도 선반 모서리를 못 찾음 (판 {used} 장) — 선반이 있는지 모르므로 접근 안 함")
            continue
        else:
            print(f"  옆 모서리를 못 찾음 (판 {used} 장) — 정면에서 잰 위치 그대로")
        blocked, npts, where = sc.path_blocked(frame, target, x_end)
        if blocked:
            print(f"  옆면 {name} 도 막힘 (점 {npts} 개: {where}) — 다음 옆면")
            continue
        print(f"  접근: 앞 {target[0]:.2f} m, 왼 {target[1]:+.2f} m, {math.degrees(target[2]):+.1f}°  ({a.cmd_topic}, {a.speed} m/s)")
        code = approach(n, mc, pub, target, x_end, a.speed)
        if code == 0:
            print(f"  선반 옆면({name})에 섬")
            return 0
        if code != 6:
            return code
    print("  갈 수 있는 옆면이 없음 → 멈춤")
    return 6


def leave(n, mc, a, path, pl, place, ref, graph):
    """설 자리에 서 있는 상태에서: 보는 자리만큼 물러나 기준 사진을 새로 저장. 선반과의 상대 위치는 물러난 거리(odom)로 정함 —
    선반을 다시 알아볼 필요가 없으므로 물건이 방금 바뀌었어도 됨. 설 자리 좌표·gate 는 그대로."""
    import cv2
    siv = ref["siv"]
    c, s = math.cos(siv[2]), math.sin(siv[2])
    inv = (-(c * siv[0] + s * siv[1]), -(-s * siv[0] + c * siv[1]), -siv[2])       # 설 자리 기준 보는 자리
    o_s = odom_pose(n, mc)
    if o_s is None:
        print("TF odom→base_footprint 없음")
        return 1
    pub = n.create_publisher(Twist, a.cmd_topic, 10)
    print(f"[1] 물러남: 뒤 {-inv[0]:.2f} m, 왼 {inv[1]:+.2f} m ({a.cmd_topic}, {a.speed} m/s) — 뒤는 카메라가 못 봄")
    ok, e = servo(n, mc, pub, places.compose(o_s, inv), a.speed, 0.3, 0.02, math.radians(1.5), 60.0)
    if not ok:
        print("  시간 초과 (실기라면 RT 를 누르고 있었나?) — 사진은 안 바꿈")
        return 5
    time.sleep(1.5)
    o_v, cur, frame = odom_pose(n, mc), mc.robot_pose(), sc.grab(n, mc)
    if o_v is None or cur is None or frame is None:
        print("  영상·TF 를 못 받음 — 사진은 안 바꿈")
        return 1
    new_siv = places.rel(o_v, o_s)                     # 지금 차체 기준 설 자리 (물러난 만큼)
    v = place["view"]
    d = path.with_suffix("")
    for kind in ("rgb.jpg", "depth.png"):              # 직전 기준은 _prev 로 하나만 보관
        src = d / f"{a.name}_view_{kind}"
        if src.exists():
            src.replace(d / f"{a.name}_view_prev_{kind}")
    cv2.imwrite(str(d / f"{a.name}_view_rgb.jpg"), frame["bgr"], [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(d / f"{a.name}_view_depth.png"), (frame["depth"] * 1000.0).clip(0, 65535).astype("uint16"))
    v["stand_in_view"] = [round(new_siv[0], 4), round(new_siv[1], 4), round(math.degrees(new_siv[2]), 2)]
    v["K"] = [round(float(x), 3) for x in frame["K"]]
    v["T_base_cam"] = [round(float(x), 5) for x in frame["T"].ravel()]
    v["pose"] = [round(cur[0], 3), round(cur[1], 3), round(math.degrees(cur[2]), 1)]
    v["anchors"] = places.make_anchors(graph, cur)
    v["refreshed"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    places.save(path, pl)
    new = sc.load_ref(place, path)
    print(f"[2] 기준 사진 새로 저장: 설 자리는 앞 {new_siv[0]:.2f} m, 왼 {new_siv[1]:+.2f} m. 선반 점 {len(new['pts'])} 개, 특징점 {len(new['fp'])} 개"
          + ("  (경고: 특징점이 적음)" if len(new["fp"]) < 30 else ""))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--places", help="places.yaml 경로 (기본: RTAB-Map 지도 옆)")
    ap.add_argument("--cmd-topic", default="/nav/cmd_vel", help="접근 속도 명령 토픽 (실기 /nav/cmd_vel = 가상 패드 데드맨, 시뮬 /cmd_vel)")
    ap.add_argument("--check-only", action="store_true", help="재기만 하고 접근하지 않음")
    ap.add_argument("--no-update", action="store_true", help="places.yaml 을 고치지 않음")
    ap.add_argument("--no-nav", action="store_true", help="Nav2 이동을 건너뜀 (이미 보는 자리 근처에 있을 때)")
    ap.add_argument("--max-shift", type=float, default=0.40, help="이보다 많이 옮겨졌으면 접근 안 함 [m]")
    ap.add_argument("--max-turn", type=float, default=20.0, help="이보다 많이 돌아갔으면 접근 안 함 [deg]")
    ap.add_argument("--allow-mid", action="store_true", help="사진은 안 맞고 모양·판만 맞는 '보통' 확신에서도 접근 (물건이 바뀐 선반)")
    ap.add_argument("--go-anyway", action="store_true",
                    help="선반을 확실히 못 알아봐도 (없음·낮음·보통) 멈추지 않고 기억한 설 자리로 감. 정확도는 지도 위치 오차 수준 (수 cm), "
                         "선반이 옮겨졌으면 못 따라감. 통로 감시는 그대로 — 앞이 막혀 있으면 안 감. 차체 위치가 몇 cm 틀려도 되는 작업용")
    ap.add_argument("--wait-blocked", type=float, default=0.0, help="앞이 막혔을 때 비키기를 기다릴 시간 [s] (기본 0 = 바로 멈춤)")
    ap.add_argument("--leave", action="store_true",
                    help="작업을 마친 뒤: 지금 선 자리에서 보는 자리만큼 똑바로 물러나 기준 사진·depth 를 새로 찍음 "
                         "(선반 위 물건이 바뀌었어도 다음에 알아보게). 뒤는 카메라가 못 보니 뒤가 비었는지 직접 볼 것")
    ap.add_argument("--side", default="auto", choices=["auto", "none", "left", "right"],
                    help="정면이 막혔을 때 옆면으로 갈지 (auto = 갈 수 있는 쪽, none = 안 감, left/right = 그쪽만)")
    ap.add_argument("--side-back", type=float, default=0.60, help="옆면 설 자리에서 이만큼 뒤까지 Nav2 로 감 [m]")
    ap.add_argument("--speed", type=float, default=0.08, help="접근 속도 [m/s]")
    ap.add_argument("-n", type=int, default=3, help="재는 횟수")
    a = ap.parse_args()
    rclpy.init()
    n = Node("place_go")
    mc = places.MapClient(n)
    path = places.find_places(mc, a.places)
    if path is None:
        print("RTAB-Map 에서 지도 경로를 못 얻음 — slam.launch 가 떠 있나? (또는 --places 로 지정)")
        return 1
    pl = places.load(path)
    if a.name not in pl:
        print(f"{path} 에 '{a.name}' 없음. 있는 것: {', '.join(pl) or '(없음)'}")
        return 1
    place = pl[a.name]
    ref = sc.load_ref(place, path)
    if ref is None:
        print(f"'{a.name}' 에 보는 자리 기록이 없음 → place_mark.py {a.name} --view  (없이 가려면 nav_goal_ahead.py --name)")
        return 1
    graph = mc.graph()
    if a.leave:
        return leave(n, mc, a, path, pl, place, ref, graph)
    stand, how = places.resolve(place, graph)
    if place.get("anchors") and "기준 노드 없음" in how:
        print("기준 노드가 지금 지도에 없음 → 보내지 않음 (다른 지도?)")
        return 1
    view, _ = places.resolve(place["view"] | {"stand_pose": place["view"]["pose"]}, graph)
    print(f"[1] 보는 자리로: map ({view[0]:.2f}, {view[1]:.2f}, {math.degrees(view[2]):.0f}°)")
    cm = Costmap(n)
    cli = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not a.no_nav:
        if not cli.wait_for_server(timeout_sec=10.0):
            print("navigate_to_pose 액션 없음 — nav.launch 가 떠 있나?")
            return 1
        if not nav_to(n, cli, view):
            print("  Nav2 실패 — 보는 자리에 못 감")
            return 1
    time.sleep(1.5)                                   # 멈춘 뒤 영상이 안정될 때까지
    t_end, was_blocked = time.time() + a.wait_blocked, False
    while True:
        cur = mc.robot_pose()
        if cur is None:
            print("TF map→base_footprint 없음")
            return 1
        init = places.rel(cur, stand)
        res, runs = sc.measure_n(n, mc, ref, init, a.n)
        frame = sc.grab(n, mc)
        blocked, npts, where = sc.path_blocked(frame, res["pose"] or init, ref["gate"][0] - 0.04) if frame else (False, 0, "")
        print(f"[2] 선반 확인: {sc.describe(res)}"
              + (f", 앞 통로에 뭔가 있음 (점 {npts} 개: {where})" if blocked else ""))
        if blocked and time.time() <= t_end:
            was_blocked = True
            print("  비키기를 기다림…", flush=True)
            time.sleep(2.0)
            continue
        if was_blocked and not blocked:               # 방금 비킴 → 가려진 채 잰 값일 수 있으니 한 번 더 잼
            was_blocked = False
            time.sleep(1.0)
            continue
        break
    if res["present"]:
        print(f"  차체 기준 설 자리: 앞 {res['pose'][0]:.3f} m, 왼 {res['pose'][1]:+.3f} m, {math.degrees(res['pose'][2]):+.2f}°")
    if blocked:
        if not res["present"]:
            if a.side == "none" or a.check_only:
                print("  앞이 가려져 선반을 확인 못 함 → 멈춤")
                return 6
            print("  앞이 가려져 선반을 확인 못 함 → 기억한 자리 기준으로 옆에 가서 확인")
            return try_sides(n, mc, cli, n.create_publisher(Twist, a.cmd_topic, 10), cm, a, place, ref, stand, confirmed=False)
        if a.side == "none" or a.check_only or res["conf"] == "low":
            print("  선반은 보이지만 앞이 막힘 → 접근 안 함")
            return 6
        shift = places.rel(init, res["pose"])
        if math.hypot(shift[0], shift[1]) > a.max_shift or abs(math.degrees(shift[2])) > a.max_turn:
            print("  선반은 보이지만 앞이 막혔고 너무 많이 옮겨짐 → 멈춤")
            return 3
        print("  선반은 보이지만 정면이 막힘 → 옆면을 알아봄")
        return try_sides(n, mc, cli, n.create_publisher(Twist, a.cmd_topic, 10), cm, a, place, ref, places.compose(cur, res["pose"]))
    def by_memory(why):
        """알아보지 못했지만 기억한 설 자리 (지도 좌표) 로 접근. 기억은 안 고침."""
        print(f"  {why} → --go-anyway: 기억한 설 자리로 감 (앞 {init[0]:.2f} m, 왼 {init[1]:+.2f} m, {math.degrees(init[2]):+.1f}°)")
        if a.check_only:
            return 0
        return approach(n, mc, n.create_publisher(Twist, a.cmd_topic, 10), init, ref["gate"][0] - 0.04, a.speed)

    if not res["present"]:
        if a.go_anyway:
            return by_memory("선반을 못 알아봄")
        print("  선반이 기억한 자리에 없음 → 멈춤")
        return 2
    shift = places.rel(init, res["pose"])
    dist, turn = math.hypot(shift[0], shift[1]), abs(math.degrees(shift[2]))
    print(f"  기억보다 앞 {shift[0] * 100:+.1f} cm, 왼 {shift[1] * 100:+.1f} cm, {math.degrees(shift[2]):+.1f}°")
    if dist > a.max_shift or turn > a.max_turn:
        print(f"  너무 많이 옮겨짐 ({dist * 100:.0f} cm, {turn:.0f}°) → 접근 안 함")
        return 3
    if res["conf"] == "low" or (res["conf"] == "mid" and not a.allow_mid):
        if a.go_anyway:
            return by_memory("확신이 낮음")
        print("  확신이 낮아 접근 안 함 (모양만 맞을 때 가려면 --allow-mid, 기억한 자리로 그냥 가려면 --go-anyway)")
        return 4
    new_stand = places.compose(cur, res["pose"])      # map 기준 새 설 자리
    if a.check_only:
        print(f"  새 설 자리 map ({new_stand[0]:.2f}, {new_stand[1]:.2f}, {math.degrees(new_stand[2]):.0f}°) — 접근은 안 함")
        return 0

    pub = n.create_publisher(Twist, a.cmd_topic, 10)
    print(f"[3] 접근: 앞 {res['pose'][0]:.2f} m, 왼 {res['pose'][1]:+.2f} m, {math.degrees(res['pose'][2]):+.1f}°  ({a.cmd_topic}, {a.speed} m/s)")
    code = approach(n, mc, pub, res["pose"], ref["gate"][0] - 0.04, a.speed)
    if code == 6 and a.side != "none":
        print("  정면이 막힘 → 옆면을 알아봄")
        return try_sides(n, mc, cli, pub, cm, a, place, ref, new_stand)
    if code:
        return code

    moved = dist > 0.04 or turn > 3.0    # 이보다 작으면 지도 위치 오차와 구분이 안 됨 (시뮬: 안 옮긴 선반이 2 cm·2.5° 로 재짐)
    if a.no_update:
        return 0
    place["last_seen"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    if moved and res["conf"] == "high":
        siv = ref["siv"]
        c, s = math.cos(siv[2]), math.sin(siv[2])
        inv = (-(c * siv[0] + s * siv[1]), -(-s * siv[0] + c * siv[1]), -siv[2])
        new_view = places.compose(new_stand, inv)
        place["stand_pose"] = [round(new_stand[0], 3), round(new_stand[1], 3), round(math.degrees(new_stand[2]), 1)]
        place["anchors"] = places.make_anchors(graph, new_stand)
        place["view"]["pose"] = [round(new_view[0], 3), round(new_view[1], 3), round(math.degrees(new_view[2]), 1)]
        place["view"]["anchors"] = places.make_anchors(graph, new_view)
        place["source"] = "shelf_check"
        print(f"[4] 기억 고침: 설 자리 map ({new_stand[0]:.2f}, {new_stand[1]:.2f}, {math.degrees(new_stand[2]):.0f}°) "
              f"← 이전 ({stand[0]:.2f}, {stand[1]:.2f}, {math.degrees(stand[2]):.0f}°)")
    else:
        print("[4] 기억 그대로 (옮겨진 양이 작음)" if not moved else "[4] 기억 그대로 (확신이 '확실' 이 아님)")
    places.save(path, pl)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n멈춤")
        sys.exit(130)
