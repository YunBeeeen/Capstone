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
    """로봇 바닥 기준(world 프레임) 상자. size 는 전체 길이 [m], yaw 는 z축 회전 [rad].
    margin: 이 상자만의 여유 [m] (None 이면 검사기 기본). 집거나 놓을 물체를 받치는 면(책상 상판, 놓을 층의 판)은
    그리퍼·물체가 가까이 가야 하므로 작게 준다."""
    name: str
    size: tuple[float, float, float]
    center: tuple[float, float, float]
    yaw: float = 0.0
    margin: float | None = None


_ROBOT_GEOM_CACHE: dict = {}


class CollisionChecker:
    CLEARANCE_LEVELS = (0.01, 0.025, 0.05)   # 여유 단계 [m] (margin 위에 더)

    def _robot_geometry(self, urdf: Path, arm_prefix: str):
        """로봇 충돌 형상 + 쌍 (팔이 낀 쌍만, SRDF Adjacent·손가락끼리 제외). 반환 (GeometryModel, 이름들, 팔 id)."""
        mesh_root = urdf.resolve().parents[1]           # .../description (= mobile_robot_description)
        xml = urdf.read_text().replace(f"package://{PACKAGE}/", f"{mesh_root}/")
        geom = pin.buildGeomFromUrdfString(self.model, xml, pin.GeometryType.COLLISION)
        geom.addAllCollisionPairs()
        srdf = urdf.with_suffix(".srdf").as_posix().replace("/urdf/", "/config/")
        if Path(srdf).is_file():
            pin.removeCollisionPairs(self.model, geom, srdf)
        names = [o.name for o in geom.geometryObjects]
        out = pin.GeometryModel()
        for o in geom.geometryObjects:
            out.addGeometryObject(o)
        for cp in geom.collisionPairs:
            a, b = names[cp.first], names[cp.second]
            if a.startswith(f"{arm_prefix}_finger") and b.startswith(f"{arm_prefix}_finger"):
                continue
            if a.startswith(arm_prefix) or b.startswith(arm_prefix):
                out.addCollisionPair(cp)
        return out, names, [i for i, n in enumerate(names) if n.startswith(arm_prefix)]

    def __init__(self, kin: B601Kinematics, env: list[Box] = (), arm_prefix: str = "right_arm",
                 margin: float = 0.02, finger_openings: tuple[float, ...] | None = None,
                 attached: list[tuple[str, tuple[float, float, float], "pin.SE3"]] = (),
                 attach_frame: str = "right_arm_end_link"):
        """attached: 그리퍼가 든 물체 [(이름, 상자 크기, 자세)] 또는 [(이름, 상자 크기, 자세, "hang")].
        기본은 attach_frame 에 단단히 붙음 (자세 = attach_frame 기준). "hang" 은 손잡이로 매달린 물체: 위치만
        attach_frame 원점을 따라가고 방향은 로봇 기준으로 고정 (똑바로 섬) — 자세 = 그 점에 놓인 로봇 방향 프레임 기준.
        가구(+여유)·로봇 몸체와 충돌 검사, 팔 링크와는 검사하지 않는다 (잡고 있으므로)."""
        self.kin = kin
        self.model = kin.model
        self.data = self.model.createData()
        key = (str(Path(kin.urdf_path).resolve()), arm_prefix, id(kin.model))
        if key not in _ROBOT_GEOM_CACHE:          # 메시 읽기 (~2 s) 는 한 번만, 이후 복사
            _ROBOT_GEOM_CACHE[key] = self._robot_geometry(Path(kin.urdf_path), arm_prefix)
        base, names, arm_ids = _ROBOT_GEOM_CACHE[key]
        self.geom = base.copy()
        self.env = list(env)
        self.margin = float(margin)
        self.finger_openings = (0.0, kin.finger_max) if finger_openings is None else tuple(finger_openings)
        level_pairs = {lv: [] for lv in self.CLEARANCE_LEVELS}
        for box in self.env:
            placement = pin.SE3(pin.rpy.rpyToMatrix(0, 0, box.yaw), np.array(box.center, dtype=float))
            for tag, extra in [("env", 0.0)] + [(f"clr{lv}", lv) for lv in self.CLEARANCE_LEVELS]:
                m = self.margin if box.margin is None else box.margin
                size = [s + 2 * (m + extra) for s in box.size]
                gid = self.geom.addGeometryObject(
                    pin.GeometryObject(f"{tag}/{box.name}", 0, 0, placement, hppfcl.Box(*size)))
                for a in arm_ids:
                    self.geom.addCollisionPair(pin.CollisionPair(a, gid))
                    if extra:
                        level_pairs[extra].append(len(self.geom.collisionPairs) - 1)
        # 들고 있는 물체: attach_frame 의 부모 관절에 붙인다
        fid = self.model.getFrameId(attach_frame)
        frame = self.model.frames[fid]
        body_ids = [i for i, o in enumerate(self.geom.geometryObjects)
                    if not o.name.startswith(arm_prefix) and "/" not in o.name]      # 차체·리프트·바퀴·카메라
        env_ids = {i: o.name for i, o in enumerate(self.geom.geometryObjects) if "/" in o.name}
        self.attached = list(attached)
        self._attach_fid, self._hanging = fid, []
        for name, size, placement, *mode in self.attached:
            if mode and mode[0] == "hang":     # 세계(로봇 기준)에 두고 매 검사 전에 위치를 옮긴다
                gid = self.geom.addGeometryObject(pin.GeometryObject(
                    f"held/{name}", 0, 0, placement, hppfcl.Box(*size)))
                self._hanging.append((gid, placement))
            else:
                gid = self.geom.addGeometryObject(pin.GeometryObject(
                    f"held/{name}", frame.parentJoint, fid, frame.placement * placement, hppfcl.Box(*size)))
            for b in body_ids:
                self.geom.addCollisionPair(pin.CollisionPair(b, gid))
            for e, ename in env_ids.items():
                self.geom.addCollisionPair(pin.CollisionPair(e, gid))
                for lv in self.CLEARANCE_LEVELS:
                    if ename.startswith(f"clr{lv}/"):
                        level_pairs[lv].append(len(self.geom.collisionPairs) - 1)
        self.names = [o.name for o in self.geom.geometryObjects]
        clr = {k for ks in level_pairs.values() for k in ks}
        self.gdata = pin.GeometryData(self.geom)
        for k in clr:                                   # 여유 사본은 충돌 판정에서 뺀다
            self.gdata.deactivateCollisionPair(k)
        # 팔 링크끼리 쌍을 뺀 검사용 (접힌 출발 자세 근처: 메시가 원래 거의 붙어 있음)
        self._noself_gdata = pin.GeometryData(self.geom)
        for k, cp in enumerate(self.geom.collisionPairs):
            a, b = self.names[cp.first], self.names[cp.second]
            if k in clr or (a.startswith(arm_prefix) and b.startswith(arm_prefix)):
                self._noself_gdata.deactivateCollisionPair(k)
        self._level_gdata = {}
        for lv, ks in level_pairs.items():              # 단계별: 그 단계 사본 쌍만 켬
            gd = pin.GeometryData(self.geom)
            keep = set(ks)
            for k in range(len(self.geom.collisionPairs)):
                if k not in keep:
                    gd.deactivateCollisionPair(k)
            self._level_gdata[lv] = gd
        self._env_gdata, self._env_pairs = None, []

    def _q(self, arm, lift_stage_m, finger):
        """관절 벡터. 매달린 물체가 있으면 그 자리도 이 자세에 맞게 옮겨 둔다."""
        q = self.kin.configuration(arm, lift_stage_m, finger)
        if self._hanging:
            pin.framesForwardKinematics(self.model, self.data, q)
            p = self.data.oMf[self._attach_fid].translation
            for gid, placement in self._hanging:
                self.geom.geometryObjects[gid].placement = pin.SE3(np.eye(3), p.copy()) * placement
        return q

    def in_collision(self, arm, lift_stage_m: float = 0.0, ignore_self: bool = False) -> bool:
        """ignore_self: 팔 링크끼리 충돌은 보지 않음 (팔↔차체·가구·든 물체는 봄)."""
        gd = self._noself_gdata if ignore_self else self.gdata
        for f in self.finger_openings:
            q = self._q(arm, lift_stage_m, f)
            if pin.computeCollisions(self.model, self.data, self.geom, gd, q, True):
                return True
        return False

    def colliding_pairs(self, arm, lift_stage_m: float = 0.0, ignore_self: bool = False) -> list[tuple[str, str]]:
        gd = self._noself_gdata if ignore_self else self.gdata
        out = []
        for f in self.finger_openings:
            q = self._q(arm, lift_stage_m, f)
            pin.computeCollisions(self.model, self.data, self.geom, gd, q, False)
            for k, res in enumerate(gd.collisionResults):
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
            q = self._q(arm, lift_stage_m, f)
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
                                            self._q(arm, lift_stage_m, f), True)
                      for f in self.finger_openings)
            if hit:
                return free
            free = lv
        return free
