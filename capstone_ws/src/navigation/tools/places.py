"""장소 기억 공용 코드 — place_mark.py (저장) 와 nav_goal_ahead.py --name (불러오기) 가 같이 씀.

파일: 지도 DB 옆 <지도>.places.yaml (예: ~/maps/lab_robot1.db → ~/maps/lab_robot1.places.yaml), 사진은 <지도>.places/
  shelf_A:
    kind: shelf                      # 종류 (자유 문자열)
    static: true                     # 고정된 것인가 (false = 옮겨질 수 있는 물체 → 도착 후 다시 찾아야 함)
    stand_pose: [3.21, -1.05, 92.0]  # 로봇이 설 자리: map x [m], y [m], yaw [deg] — 저장한 순간의 지도 좌표
    anchors:                         # 가까운 지도 노드 기준 상대 위치 (가까운 순, 최대 3 개)
    - {node: 412, offset: [0.12, -0.03, 1.5]}    # 노드 좌표계에서 앞 [m], 왼 [m], yaw [deg]
    last_seen: '2026-11-12T14:03:10'
    source: manual                   # manual = 사람이 표시 / 나중에 esam_siglip 등
    image: lab_robot1.places/shelf_A_rgb.jpg     # 표시한 순간 ZED 영상 (나중에 "이 선반" 찾기 기준)
    depth: lab_robot1.places/shelf_A_depth.png   # 16 bit [mm], 0 = 값 없음
    K: [fx, fy, cx, cy]

anchors 가 있는 이유: 표시한 뒤 루프 클로저가 나면 RTAB-Map 이 지도 전체를 다시 맞춰서 stand_pose 좌표가 어긋남.
노드 번호는 안 바뀌므로 불러올 때 "지금 지도에서 그 노드 위치 + offset" 으로 다시 계산 (resolve).
"""
import math
import time
from pathlib import Path

import rclpy
import yaml
from rcl_interfaces.srv import GetParameters
from rtabmap_msgs.srv import GetMap
from tf2_ros import Buffer, TransformListener

RTABMAP_NODE = "/rtabmap/rtabmap"
MAX_ANCHORS = 3


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rel(a, b):
    """a 좌표계에서 본 b (둘 다 map 기준 (x, y, yaw))."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


def compose(a, d):
    """map 기준 a 에 a 좌표계의 d 를 붙임 (rel 의 반대)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * d[0] - s * d[1], a[1] + s * d[0] + c * d[1], wrap(a[2] + d[2]))


def places_path(db):
    return Path(db).expanduser().with_suffix(".places.yaml")


def load(path):
    path = Path(path)
    return (yaml.safe_load(path.read_text()) or {}) if path.exists() else {}


def save(path, places):
    path = Path(path)
    if path.exists():
        path.with_suffix(".yaml.bak").write_text(path.read_text())   # 직전 상태 하나만 보관
    path.write_text(yaml.safe_dump(places, allow_unicode=True, sort_keys=False, default_flow_style=None, width=120))


def make_anchors(graph, pose, k=MAX_ANCHORS):
    """graph = {노드 id: (x, y, yaw)} 에서 pose 에 가까운 k 개 → anchors 목록."""
    near = sorted(graph.items(), key=lambda kv: math.hypot(kv[1][0] - pose[0], kv[1][1] - pose[1]))[:k]
    out = []
    for nid, npose in near:
        d = rel(npose, pose)
        out.append({"node": int(nid), "offset": [round(d[0], 3), round(d[1], 3), round(math.degrees(d[2]), 2)]})
    return out


def resolve(place, graph):
    """저장된 장소 → 지금 지도 좌표 (x, y, yaw[rad]) 와 근거 설명. 노드를 못 찾으면 stand_pose 그대로."""
    for a in place.get("anchors") or []:
        if a["node"] in graph:
            o = a["offset"]
            return compose(graph[a["node"]], (o[0], o[1], math.radians(o[2]))), f"노드 {a['node']} 기준"
    sp = place["stand_pose"]
    return (sp[0], sp[1], math.radians(sp[2])), "저장 좌표 그대로 (기준 노드 없음)"


def send_goal(node, cli, goal, feedback_callback=None, tries=3, timeout=5.0):
    """액션 목표 보내고 수락 응답(goal handle)을 받음. 응답이 timeout 안에 안 오면 다시 보냄. 끝내 못 받으면 None.
    (연결 직후에 보내면 서버가 응답을 못 돌려주는 경우가 있음 — 시뮬 2026-11-13: 무한 대기. 같은 목표를 다시 보내면 Nav2 는 새 목표로 바꿈)"""
    for k in range(tries):
        end = time.time() + 0.5
        while time.time() < end:                      # 서버·클라이언트가 서로를 찾을 시간
            rclpy.spin_once(node, timeout_sec=0.05)
        fut = cli.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if fut.done():
            return fut.result()
        print(f"  (목표 응답 없음 — 다시 보냄 {k + 1}/{tries})")
    return None


def find_places(mc, arg=None, tries=3):
    """places.yaml 경로: arg 가 있으면 그것, 없으면 RTAB-Map 지도 옆. RTAB-Map 이 바빠 답이 늦을 수 있어 몇 번 다시 물음. 못 찾으면 None."""
    if arg:
        return Path(arg).expanduser()
    for _ in range(tries):
        db = mc.db_path()
        if db:
            return places_path(db)
    return None


class MapClient:
    """RTAB-Map 에서 지도 파일 경로·노드 위치를, TF 에서 로봇 위치를 가져옴. node 는 호출하는 쪽 rclpy 노드."""

    def __init__(self, node):
        self.n = node
        self.tf = Buffer()
        TransformListener(self.tf, node)

    def _call(self, cli, req, timeout):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def db_path(self, timeout=5.0):
        """RTAB-Map 이 쓰는 지도 파일 (파라미터 database_path). 못 얻으면 None."""
        cli = self.n.create_client(GetParameters, RTABMAP_NODE + "/get_parameters")
        r = self._call(cli, GetParameters.Request(names=["database_path"]), timeout)
        return r.values[0].string_value if r and r.values and r.values[0].string_value else None

    def graph(self, timeout=10.0):
        """보정된 지도 노드 위치 {id: (x, y, yaw)} (map 기준). 못 얻으면 {}."""
        cli = self.n.create_client(GetMap, RTABMAP_NODE + "/get_map_data")
        r = self._call(cli, GetMap.Request(global_map=True, optimized=True, graph_only=True), timeout)
        if r is None:
            return {}
        g = r.data.graph
        return {i: (p.position.x, p.position.y, yaw_q(p.orientation)) for i, p in zip(g.poses_id, g.poses)}

    def robot_pose(self, timeout=5.0):
        """TF map→base_footprint → (x, y, yaw). 없으면 None."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.n, timeout_sec=0.1)
            try:
                t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
                return (t.translation.x, t.translation.y, yaw_q(t.rotation))
            except Exception:
                pass
        return None
