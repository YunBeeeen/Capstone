"""팔 충돌 검사 (Pinocchio + hpp-fcl): 팔 자기 충돌, 팔 ↔ 로봇 몸체, 팔 ↔ 환경 상자.

충돌 쌍 = 전체 쌍 − SRDF Adjacent − 손가락끼리 − 팔이 없는 쌍 (+ 팔 ↔ 환경 상자).
손가락은 닫힌 상태(관절 0)로 검사한다.
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
    def __init__(self, kin: B601Kinematics, env: list[Box] = (), arm_prefix: str = "right_arm"):
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
        for box in self.env:
            placement = pin.SE3(pin.rpy.rpyToMatrix(0, 0, box.yaw), np.array(box.center, dtype=float))
            obj = pin.GeometryObject(f"env/{box.name}", 0, 0, placement, hppfcl.Box(*box.size))
            gid = self.geom.addGeometryObject(obj)
            for a in arm_ids:
                self.geom.addCollisionPair(pin.CollisionPair(a, gid))
        self.gdata = pin.GeometryData(self.geom)
        self.names = [o.name for o in self.geom.geometryObjects]

    def in_collision(self, arm, lift_stage_m: float = 0.0) -> bool:
        q = self.kin.configuration(arm, lift_stage_m)
        return bool(pin.computeCollisions(self.model, self.data, self.geom, self.gdata, q, True))

    def colliding_pairs(self, arm, lift_stage_m: float = 0.0) -> list[tuple[str, str]]:
        q = self.kin.configuration(arm, lift_stage_m)
        pin.computeCollisions(self.model, self.data, self.geom, self.gdata, q, False)
        out = []
        for k, res in enumerate(self.gdata.collisionResults):
            if res.isCollision():
                cp = self.geom.collisionPairs[k]
                out.append((self.names[cp.first], self.names[cp.second]))
        return out
