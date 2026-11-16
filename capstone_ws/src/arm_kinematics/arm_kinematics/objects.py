"""책상 위 물체 (장난감 과일·채소, 바구니) — MuJoCo 형상, 놓는 자리, 계획용 상자를 한 곳에서.

mission.py (임무 시뮬) 과 tools/export_scene.py (장면 MJCF) 가 같이 쓴다.
크기는 흔한 장난감 모형 기준 추정값이다 — 실물을 재면 여기만 고치면 된다.

몸체 원점: 과일·채소는 형상 가운데 (누우면 책상 위 rest_z), 바구니는 바닥 가운데.
채소는 몸체 x 축을 따라 길게 누워 있다 (yaw 로 책상 위 방향).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

FRICTION = [1.0, 0.01, 0.001]


@dataclass
class Part:
    kind: str                       # sphere | ellipsoid | capsule | cylinder | box
    size: tuple                     # MuJoCo size (반지름·반길이). capsule/cylinder 는 fromto 와 함께 반지름만
    rgba: tuple
    pos: tuple = (0.0, 0.0, 0.0)
    fromto: tuple | None = None     # capsule/cylinder: (x1, y1, z1, x2, y2, z2)


@dataclass
class ObjectSpec:
    name: str
    parts: list[Part]
    mass: float                     # 전체 [kg] (부품에 부피 비례 대신 똑같이 나눔)
    rest_z: float                   # 면 위에 놓였을 때 몸체 원점 높이 [m]
    bbox: tuple                     # 계획용 상자 전체 크기 (몸체 기준) [m]
    bbox_center: tuple = (0.0, 0.0, 0.0)
    label: str = ""


def _caps(points, r, rgba):
    """점들을 잇는 캡슐들 (휜 바나나·고추 등)."""
    return [Part("capsule", (r,), rgba, fromto=(*a, *b)) for a, b in zip(points, points[1:])]


ORANGE_R = 0.03
# 실물 21 x 13 x 10 cm (테두리 높이 10). 손잡이 높이는 아직 안 잼 — 테두리 + 3 cm 로 가정
BASKET_SIZE, BASKET_WALL, BASKET_HANDLE_H = (0.21, 0.13, 0.10), 0.006, 0.13


def _basket_parts():
    bx, by, bz = BASKET_SIZE
    w, hh, rgba = BASKET_WALL, BASKET_HANDLE_H, (0.75, 0.6, 0.35, 1)
    boxes = [((bx / 2, by / 2, w / 2), (0, 0, w / 2)),                                          # 바닥
             ((w / 2, by / 2, bz / 2), (bx / 2 - w / 2, 0, bz / 2)), ((w / 2, by / 2, bz / 2), (-bx / 2 + w / 2, 0, bz / 2)),
             ((bx / 2, w / 2, bz / 2), (0, by / 2 - w / 2, bz / 2)), ((bx / 2, w / 2, bz / 2), (0, -by / 2 + w / 2, bz / 2)),
             ((0.006, by / 2, 0.006), (0, 0, hh)),                                              # 손잡이 (y 방향 막대)
             ((0.006, 0.006, (hh - bz) / 2), (0, by / 2 - 0.006, (hh + bz) / 2)),
             ((0.006, 0.006, (hh - bz) / 2), (0, -by / 2 + 0.006, (hh + bz) / 2))]
    return [Part("box", half, rgba, pos) for half, pos in boxes]


GREEN, DARK_GREEN = (0.30, 0.65, 0.20, 1), (0.15, 0.40, 0.10, 1)

OBJECTS: dict[str, ObjectSpec] = {o.name: o for o in [
    ObjectSpec("fruit", [Part("sphere", (ORANGE_R,), (0.95, 0.45, 0.10, 1))], 0.05, ORANGE_R,
               (2 * ORANGE_R,) * 3, label="orange (지름 6 cm)"),
    ObjectSpec("basket", _basket_parts(), 0.16, 0.0,
               (BASKET_SIZE[0], BASKET_SIZE[1], BASKET_HANDLE_H), (0, 0, BASKET_HANDLE_H / 2),
               label="바구니 21×13×10 cm (실측), 손잡이 높이 13 cm (가정)"),
    ObjectSpec("eggplant", [Part("ellipsoid", (0.07, 0.032, 0.028), (0.35, 0.12, 0.45, 1)),   # 살짝 납작 (안 구르게)
                            Part("capsule", (0.016,), DARK_GREEN, fromto=(0.06, 0, 0, 0.08, 0, 0)),
                            Part("capsule", (0.005,), DARK_GREEN, fromto=(0.08, 0, 0, 0.10, 0, 0))],
               0.06, 0.028, (0.20, 0.064, 0.056), (0.015, 0, 0), label="가지 길이 17 cm, 지름 6 cm"),
    ObjectSpec("banana", _caps([(-0.08, 0.025, 0), (-0.03, 0.0, 0), (0.03, 0.0, 0), (0.08, 0.025, 0)], 0.017,
                               (0.95, 0.85, 0.20, 1))
               + [Part("capsule", (0.006,), (0.35, 0.25, 0.10, 1), fromto=(0.08, 0.025, 0, 0.095, 0.035, 0))],
               0.05, 0.017, (0.21, 0.07, 0.034), (0.0, 0.015, 0), label="바나나 길이 19 cm"),
    ObjectSpec("carrot", [Part("capsule", (0.018,), (0.95, 0.50, 0.10, 1), fromto=(-0.06, 0, 0, 0.0, 0, 0)),
                          Part("capsule", (0.012,), (0.95, 0.50, 0.10, 1), fromto=(0.0, 0, 0, 0.05, 0, 0)),
                          Part("capsule", (0.006,), (0.95, 0.50, 0.10, 1), fromto=(0.05, 0, 0, 0.075, 0, 0)),
                          Part("capsule", (0.004,), GREEN, fromto=(-0.075, 0, 0, -0.12, 0.012, 0)),
                          Part("capsule", (0.004,), GREEN, fromto=(-0.075, 0, 0, -0.12, -0.012, 0))],
               0.04, 0.018, (0.20, 0.04, 0.036), (-0.02, 0, 0), label="당근 길이 20 cm (잎 포함)"),
    ObjectSpec("corn", [Part("capsule", (0.025,), (0.98, 0.80, 0.25, 1), fromto=(-0.065, 0, 0, 0.075, 0, 0)),
                        Part("ellipsoid", (0.04, 0.018, 0.014), GREEN, pos=(-0.08, 0.012, 0.0)),
                        Part("ellipsoid", (0.04, 0.018, 0.014), GREEN, pos=(-0.08, -0.012, 0.0))],
               0.08, 0.025, (0.24, 0.05, 0.05), (-0.01, 0, 0), label="옥수수 길이 22 cm, 지름 5 cm"),
    ObjectSpec("chilli", _caps([(-0.04, 0.0, 0), (0.0, 0.005, 0), (0.04, 0.016, 0)], 0.009, (0.85, 0.08, 0.05, 1))
               + [Part("capsule", (0.0035,), DARK_GREEN, fromto=(-0.04, 0, 0, -0.056, -0.004, 0))],
               0.015, 0.009, (0.11, 0.03, 0.018), (-0.005, 0.006, 0), label="고추 길이 10 cm"),
    ObjectSpec("pepper", [Part("ellipsoid", (0.037, 0.037, 0.04), (0.15, 0.60, 0.15, 1)),
                          Part("cylinder", (0.026,), (0.15, 0.60, 0.15, 1), fromto=(0, 0, -0.04, 0, 0, -0.03)),  # 평평한 바닥
                          Part("cylinder", (0.006,), DARK_GREEN, fromto=(0, 0, 0.035, 0, 0, 0.055))],
               0.05, 0.04, (0.075, 0.075, 0.095), (0, 0, 0.0075), label="피망 지름 7.5 cm, 높이 8 cm"),
]}


@dataclass
class DeskLayout:
    """책상 설 자리 기준 (x 앞, y 왼쪽 +) 물체 자리 (x, y, yaw[도]). 책상은 x 0.35~0.85, y −0.60~+0.40."""
    poses: dict = field(default_factory=lambda: {
        "fruit": (0.50, -0.22, 0.0),
        "basket": (0.53, 0.00, 0.0),
        "banana": (0.72, -0.24, 90.0),
        "eggplant": (0.74, -0.03, 80.0),
        "carrot": (0.72, 0.20, 100.0),
        "corn": (0.50, 0.27, 0.0),
        "chilli": (0.52, -0.43, 30.0),
        "pepper": (0.74, -0.44, 0.0),
    })


def add_to_spec(spec, mj, name: str, pos, yaw: float):
    """MjSpec 에 자유 물체 하나 (몸체 이름 = name, 자유 관절 = name_free). 접촉 켬."""
    o = OBJECTS[name]
    body = spec.worldbody.add_body(name=name, pos=list(pos), quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    body.add_freejoint(name=f"{name}_free")
    kinds = {"sphere": mj.mjtGeom.mjGEOM_SPHERE, "ellipsoid": mj.mjtGeom.mjGEOM_ELLIPSOID,
             "capsule": mj.mjtGeom.mjGEOM_CAPSULE, "cylinder": mj.mjtGeom.mjGEOM_CYLINDER, "box": mj.mjtGeom.mjGEOM_BOX}
    for k, p in enumerate(o.parts):
        size = list(p.size) + [0.0] * (3 - len(p.size))
        g = body.add_geom(name=f"{name}_{k}", type=kinds[p.kind], size=size, rgba=list(p.rgba), pos=list(p.pos),
                          mass=o.mass / len(o.parts), contype=1, conaffinity=1, friction=FRICTION)
        if p.fromto is not None:
            g.fromto = list(p.fromto)
    return body
