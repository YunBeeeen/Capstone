"""팔 충돌 검사 (Pinocchio + hpp-fcl): 팔 자기 충돌, 팔 ↔ 로봇 몸체, 팔 ↔ 환경 상자.

충돌 쌍 = 전체 쌍 − SRDF Adjacent − 손가락끼리 − 팔이 없는 쌍 (+ 팔 ↔ 환경 상자).
손가락은 닫힘(0)과 벌림(finger_openings) 을 모두 검사한다 (그리퍼를 열고 들어갈 수 있으므로).
환경 상자는 margin 만큼 사방으로 키운다 (선반 위치 측정 오차·실기 추종 지연 여유).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import hppfcl
import numpy as np
import pinocchio as pin

from .b601 import B601Kinematics

PACKAGE = "mobile_robot_description"


@dataclass(frozen=True)
class Box:
    """로봇 바닥 기준(world 프레임) 상자. size 는 전체 길이 [m], yaw 는 z축 회전 [rad]."""
    name: str
    size: tuple[float, float, float]
    center: tuple[float, float, float]
    yaw: float = 0.0


class CollisionChecker:
    def __init__(self, kin: B601Kinematics, env: list[Box] = (), arm_prefix: str = "right_arm",
                 margin: float = 0.02, finger_openings: tuple[float, ...] | None = None):
        self.kin = kin
        self.model = kin.model
        self.data = self.model.createData()
        urdf = Path(kin.urdf_path)
        mesh_root = urdf.resolve().parents[1]           # .../description (= mobile_robot_description)
        xml = urdf.read_text().replace(f"package://{PACKAGE}/", f"{mesh_root}/")
        geom = pin.buildGeomFromUrdfString(self.model, xml, pin.GeometryType.COLLISION)
        geom.addAllCollisionPairs()
        srdf = urdf.with_suffix(".srdf").as_posix().replace("/urdf/", "/config/")
        if Path(srdf).is_file():
            pin.removeCollisionPairs(self.model, geom, srdf)
        names = [o.name for o in geom.geometryObjects]
        self.geom = pin.GeometryModel()
        for o in geom.geometryObjects:
            self.geom.addGeometryObject(o)
        for cp in geom.collisionPairs:
            a, b = names[cp.first], names[cp.second]
            if a.startswith(f"{arm_prefix}_finger") and b.startswith(f"{arm_prefix}_finger"):
                continue
            if a.startswith(arm_prefix) or b.startswith(arm_prefix):
                self.geom.addCollisionPair(cp)
        arm_ids = [i for i, n in enumerate(names) if n.startswith(arm_prefix)]
        self.env = list(env)
        self.margin = float(margin)
        self.finger_openings = (0.0, kin.finger_max) if finger_openings is None else tuple(finger_openings)
        for box in self.env:
            placement = pin.SE3(pin.rpy.rpyToMatrix(0, 0, box.yaw), np.array(box.center, dtype=float))
            size = [s + 2 * self.margin for s in box.size]
            obj = pin.GeometryObject(f"env/{box.name}", 0, 0, placement, hppfcl.Box(*size))
            gid = self.geom.addGeometryObject(obj)
            for a in arm_ids:
                self.geom.addCollisionPair(pin.CollisionPair(a, gid))
        self.gdata = pin.GeometryData(self.geom)
        self.names = [o.name for o in self.geom.geometryObjects]

    def in_collision(self, arm, lift_stage_m: float = 0.0) -> bool:
        for f in self.finger_openings:
            q = self.kin.configuration(arm, lift_stage_m, f)
            if pin.computeCollisions(self.model, self.data, self.geom, self.gdata, q, True):
                return True
        return False

    def colliding_pairs(self, arm, lift_stage_m: float = 0.0) -> list[tuple[str, str]]:
        out = []
        for f in self.finger_openings:
            q = self.kin.configuration(arm, lift_stage_m, f)
            pin.computeCollisions(self.model, self.data, self.geom, self.gdata, q, False)
            for k, res in enumerate(self.gdata.collisionResults):
                if res.isCollision():
                    cp = self.geom.collisionPairs[k]
                    pair = (self.names[cp.first], self.names[cp.second])
                    if pair not in out:
                        out.append(pair)
        return out
