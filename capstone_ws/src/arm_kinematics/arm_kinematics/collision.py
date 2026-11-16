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
    CLEARANCE_LEVELS = (0.01, 0.025, 0.05)   # 여유 단계 [m] (margin 위에 더)

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
        level_pairs = {lv: [] for lv in self.CLEARANCE_LEVELS}
        for box in self.env:
            placement = pin.SE3(pin.rpy.rpyToMatrix(0, 0, box.yaw), np.array(box.center, dtype=float))
            for tag, extra in [("env", 0.0)] + [(f"clr{lv}", lv) for lv in self.CLEARANCE_LEVELS]:
                size = [s + 2 * (self.margin + extra) for s in box.size]
                gid = self.geom.addGeometryObject(
                    pin.GeometryObject(f"{tag}/{box.name}", 0, 0, placement, hppfcl.Box(*size)))
                for a in arm_ids:
                    self.geom.addCollisionPair(pin.CollisionPair(a, gid))
                    if extra:
                        level_pairs[extra].append(len(self.geom.collisionPairs) - 1)
        self.names = [o.name for o in self.geom.geometryObjects]
        clr = {k for ks in level_pairs.values() for k in ks}
        self.gdata = pin.GeometryData(self.geom)
        for k in clr:                                   # 여유 사본은 충돌 판정에서 뺀다
            self.gdata.deactivateCollisionPair(k)
        self._level_gdata = {}
        for lv, ks in level_pairs.items():              # 단계별: 그 단계 사본 쌍만 켬
            gd = pin.GeometryData(self.geom)
            keep = set(ks)
            for k in range(len(self.geom.collisionPairs)):
                if k not in keep:
                    gd.deactivateCollisionPair(k)
            self._level_gdata[lv] = gd
        self._env_gdata, self._env_pairs = None, []

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

    def min_distance(self, arm, lift_stage_m: float = 0.0) -> tuple[float, tuple[str, str] | None]:
        """팔 ↔ 환경 상자 쌍의 최소 거리 [m] 와 그 쌍 (손가락 닫힘·벌림 중 작은 쪽). 환경 상자는 margin 을 더한 크기 기준.
        팔 링크끼리·팔↔몸체 쌍은 원래 거의 붙어 있어 여유 지표로 의미가 없으므로 뺀다. 환경이 없으면 (inf, None)."""
        if self._env_gdata is None:
            pairs = [k for k, cp in enumerate(self.geom.collisionPairs)
                     if self.names[cp.second].startswith("env/")]
            self._env_pairs = pairs
            self._env_gdata = pin.GeometryData(self.geom)
            for k in range(len(self.geom.collisionPairs)):
                if k not in pairs:
                    self._env_gdata.deactivateCollisionPair(k)
        if not self._env_pairs:
            return float("inf"), None
        best, pair = float("inf"), None
        for f in self.finger_openings:
            q = self.kin.configuration(arm, lift_stage_m, f)
            pin.computeDistances(self.model, self.data, self.geom, self._env_gdata, q)
            for k in self._env_pairs:
                res = self._env_gdata.distanceResults[k]
                if res.min_distance < best:
                    cp = self.geom.collisionPairs[k]
                    best, pair = float(res.min_distance), (self.names[cp.first], self.names[cp.second])
        return best, pair

    def clearance(self, arm, lift_stage_m: float = 0.0) -> float:
        """팔 ↔ 환경 여유 [m] 를 단계로 (margin 위에 0, 0.01, 0.025, 0.05 이상). 충돌 판정만 써서 빠르다.
        환경이 없으면 inf."""
        if not self.env:
            return float("inf")
        free = 0.0
        for lv in self.CLEARANCE_LEVELS:
            gd = self._level_gdata[lv]
            hit = any(pin.computeCollisions(self.model, self.data, self.geom, gd,
                                            self.kin.configuration(arm, lift_stage_m, f), True)
                      for f in self.finger_openings)
            if hit:
                return free
            free = lv
        return free
