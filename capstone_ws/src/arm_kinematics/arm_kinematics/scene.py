"""시험 환경: 4층 사다리 선반 + 책상, 충돌 상자와 MuJoCo 장면을 같은 정의에서 만든다.

선반: 연구실 실물 실측 (2026-11-16 사용자): 높이 1.477 m, 판 윗면 바닥에서 0.21 m (1층),
      +0.34 → 0.55 (2층), +0.33 → 0.88 (3층), +0.43 → 1.31 (4층).
      폭 0.6 × 깊이 0.33 m, 판 두께 18 mm, 기둥 25 mm 는 gazebo/tools/make_shelf.py 와 같다
      (Gazebo 모델의 높이는 옛값 1.5 m / 0.129·0.529·0.909·1.279). 원점 = 바닥 중심, 앞면 = 로컬 −y.
책상: 1.0 × 0.5 m, 윗면 = 선반 3층 윗면 높이 (0.88 m). 원점 = 바닥 중심, 긴 변 = 로컬 x.
배치는 모두 '방 좌표'(MuJoCo world, 바닥 z=0). 로봇 자세 (x, y, yaw) 를 주면 로봇 바닥 기준으로 바꾼다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .collision import Box

SHELF_W, SHELF_D, SHELF_H = 0.60, 0.33, 1.477
SHELF_TOPS = (0.21, 0.55, 0.88, 1.31)              # 판 윗면 (실측: 0.21, +0.34, +0.33, +0.43)
SHELF_TH, POST = 0.018, 0.025
SHELF_BOARDS = tuple(t - SHELF_TH / 2 for t in SHELF_TOPS)   # 판 가운데
DESK_W, DESK_D = 1.0, 0.5
DESK_TOP = SHELF_TOPS[2]                           # 0.88 (선반 3층)
DESK_TH, DESK_LEG = 0.03, 0.04


def shelf_board_top(level: int) -> float:
    """아래에서 level 층 (1~4) 판 윗면 높이 [m]."""
    return SHELF_TOPS[level - 1]


@dataclass(frozen=True)
class Placement:
    x: float
    y: float
    yaw: float = 0.0

    def apply(self, local):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx, ly, lz = local
        return (self.x + c * lx - s * ly, self.y + s * lx + c * ly, lz)


def shelf_boxes(p: Placement, prefix: str = "shelf") -> list[Box]:
    """로컬 상자 → 방 좌표 상자."""
    out = []
    for i, sx in enumerate((-SHELF_W / 2 + POST / 2, SHELF_W / 2 - POST / 2)):
        for j, sy in enumerate((-SHELF_D / 2 + POST / 2, SHELF_D / 2 - POST / 2)):
            out.append(Box(f"{prefix}_post{i}{j}", (POST, POST, SHELF_H), p.apply((sx, sy, SHELF_H / 2)), p.yaw))
        out.append(Box(f"{prefix}_topbar{i}", (POST, SHELF_D, POST), p.apply((sx, 0, SHELF_H - POST / 2)), p.yaw))
    for k, z in enumerate(SHELF_BOARDS, start=1):
        out.append(Box(f"{prefix}_board{k}", (SHELF_W - 2 * POST, SHELF_D, SHELF_TH), p.apply((0, 0, z)), p.yaw))
    return out


def desk_boxes(p: Placement, prefix: str = "desk") -> list[Box]:
    out = [Box(f"{prefix}_top", (DESK_W, DESK_D, DESK_TH), p.apply((0, 0, DESK_TOP - DESK_TH / 2)), p.yaw)]
    leg_h = DESK_TOP - DESK_TH
    for i, sx in enumerate((-DESK_W / 2 + DESK_LEG, DESK_W / 2 - DESK_LEG)):
        for j, sy in enumerate((-DESK_D / 2 + DESK_LEG, DESK_D / 2 - DESK_LEG)):
            out.append(Box(f"{prefix}_leg{i}{j}", (DESK_LEG, DESK_LEG, leg_h), p.apply((sx, sy, leg_h / 2)), p.yaw))
    return out


# 배치: 내비게이션은 ZED(로봇 정면, +x)가 가구를 보게 '설 자리'에 선다 (place_go.py).
# 그래서 방 좌표 = 로봇 바닥 기준, 로봇은 원점에서 +x 를 보고, 가구는 정면에 놓는다.
#   front   : 로봇 중심 → 가구 앞면 [m] (차체 앞 끝 = ZED, x = 0.195)
#   lateral : 가구 폭 가운데의 y [m]. ZED 가 가구를 봐야 하므로 기본 0 (가구가 정면 가운데)
FRONT, LATERAL = 0.45, 0.0               # 선반 가운데가 ZED 정면. 앞면 0.45 m 에서 2~4층·책상 됨 (stand_sweep.py)
ALONG = -0.10                            # 기본 물체 위치: 선반 가운데에서 로봇 오른쪽 10 cm
BODY_FRONT_X = 0.195


def furniture(name: str, front: float = FRONT, lateral: float = LATERAL) -> tuple[str, Placement]:
    """목표 이름 → (가구 종류, 방 좌표 배치). 둘 다 앞면(로컬 −y)이 로봇을 본다."""
    if name == "desk":
        return "desk", Placement(front + DESK_D / 2, lateral, math.pi / 2)    # 긴 변이 로봇과 나란히
    if name.startswith("shelf"):
        return "shelf", Placement(front + SHELF_D / 2, lateral, -math.pi / 2)
    raise ValueError(f"unknown target preset {name!r}")


def env_for(name: str, front: float = FRONT, lateral: float = LATERAL) -> list[Box]:
    kind, p = furniture(name, front, lateral)
    return desk_boxes(p) if kind == "desk" else shelf_boxes(p)


def default_env() -> list[Box]:
    return env_for("shelf3")


def to_robot(boxes: list[Box], robot: Placement) -> list[Box]:
    """방 좌표 상자 → 로봇 바닥 기준(world 프레임) 상자."""
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    out = []
    for b in boxes:
        dx, dy = b.center[0] - robot.x, b.center[1] - robot.y
        out.append(Box(b.name, b.size, (c * dx + s * dy, -s * dx + c * dy, b.center[2]), b.yaw - robot.yaw, b.margin))
    return out


def room_to_robot(point, robot: Placement):
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    dx, dy = point[0] - robot.x, point[1] - robot.y
    return np.array([c * dx + s * dy, -s * dx + c * dy, point[2]])


def robot_to_room(point, robot: Placement):
    return np.array(robot.apply(point))


PRESETS = ("desk", "shelf1", "shelf2", "shelf3", "shelf4")
APPROACH_HELP = ("heading (기본: 그리퍼가 가구 안쪽을 보며 기울기만 TILT 범위) | into (안쪽 수평 고정) | "
                 "down (위에서) | free (방향 자유) | ax,ay,az (로봇 기준 접근축)")


def orientation_args(mode: str, into, tilt_deg=(-10.0, 45.0)):
    """CLI 접근 모드 → (B601IK.solve 인자, MotionPlanner.plan 인자, 그릴 화살표 방향)."""
    into = np.asarray(into, float)
    if mode == "free":
        return {}, {}, None
    if mode == "heading":
        return {"heading": into, "tilt_deg": tuple(tilt_deg)}, {"heading": into}, into
    a = {"down": np.array([0, 0, -1.0]), "into": into}.get(mode)
    if a is None:
        a = np.array([float(v) for v in mode.split(",")])
    a = a / np.linalg.norm(a)
    return {"approach": a}, {"approach": a}, a


def preset_target(name: str, height: float = 0.05, inset: float | None = None,
                  front: float = FRONT, lateral: float = LATERAL, along: float = ALONG):
    """방 좌표 (목표점, 들어가는 수평 방향, 로봇 자리). 로봇은 원점에서 +x.

    desk    : 책상 앞 모서리에서 inset(기본 0.15 m) 안쪽
    shelf1~4: 아래에서 n 층 판, 앞면에서 inset(기본 0.10 m) 안쪽
    along   : 가구 폭 가운데에서 로봇 y 방향으로 옮긴 양 [m]
    height  : 면 위로 띄울 높이 (잡을 물체 중심)
    """
    if name not in PRESETS:
        raise ValueError(f"unknown target preset {name!r} ({', '.join(PRESETS)})")
    kind, p = furniture(name, front, lateral)
    if kind == "desk":
        d, top = (0.15 if inset is None else inset), DESK_TOP
    else:
        d, top = (0.10 if inset is None else inset), shelf_board_top(int(name[5:]))
    target = np.array([front + d, lateral + along, top + height])
    return target, np.array([1.0, 0.0, 0.0]), Placement(0.0, 0.0, 0.0)


# ---- MuJoCo ------------------------------------------------------------------------------------
def build_mujoco(robot_xml, boxes: list[Box], arm_limits_deg: dict[int, tuple[float, float]] | None = None,
                 prefix: str = "right_arm_"):
    """로봇 MJCF + 환경 상자 → (MjModel, MjData). 상자는 방 좌표. 상자 이름 'shelf*'/'desk*' 색 구분.

    arm_limits_deg 를 주면 팔 관절 범위·액추에이터 ctrlrange 를 그 값으로 바꾼다
    (MJCF 는 URDF 원본 범위라 실기의 넓힌 한계 −210/−200° 를 못 씀)."""
    import mujoco

    spec = mujoco.MjSpec.from_file(str(robot_xml))
    if arm_limits_deg:
        for n, (lo, hi) in arm_limits_deg.items():
            name = f"{prefix}joint{n}"
            rng = [math.radians(lo), math.radians(hi)]
            spec.joint(name).range = rng
            spec.actuator(name).ctrlrange = rng
    colors = {"shelf_board": [0.90, 0.88, 0.94, 1], "shelf": [0.09, 0.09, 0.09, 1],
              "desk_top": [0.82, 0.65, 0.43, 1], "desk": [0.35, 0.30, 0.25, 1]}
    for b in boxes:
        body = spec.worldbody.add_body(name=f"env_{b.name}", pos=list(b.center),
                                       quat=[math.cos(b.yaw / 2), 0, 0, math.sin(b.yaw / 2)])
        key = next((k for k in colors if b.name.startswith(k)), "shelf")
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[s / 2 for s in b.size], rgba=colors[key],
                      contype=0, conaffinity=0)
    model = spec.compile()
    return model, mujoco.MjData(model)


BASE_LINK_Z = -0.020   # URDF world_to_base: base_link 는 바닥 기준점보다 20 mm 아래


def set_mujoco_pose(model, data, robot: Placement, arm, lift_stage_m: float = 0.0, prefix: str = "right_arm_"):
    """물리 없이 자세만 놓는다 (mj_forward). arm = 모터 번호/이름/리스트 [도]."""
    import mujoco

    from .b601 import public_to_q

    def adr(name):
        return model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]

    a = adr("base_free")
    data.qpos[a:a + 3] = [robot.x, robot.y, BASE_LINK_Z]
    data.qpos[a + 3:a + 7] = [math.cos(robot.yaw / 2), 0, 0, math.sin(robot.yaw / 2)]
    for name in ("lift_stage_1_joint", "lift_stage_2_joint"):
        data.qpos[adr(name)] = lift_stage_m
    for i, q in enumerate(public_to_q(arm), start=1):
        data.qpos[adr(f"{prefix}joint{i}")] = q
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
