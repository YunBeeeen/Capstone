"""B601-DM 오른팔 FK / 자코비안 / 조작성 (모바일 베이스 + 리프트 + 45° 장착).

모델은 우리 URDF(`description/urdf/mobile_robot_right_arm.urdf`) 하나를 Pinocchio 로 읽는다.
체인: base_link → lift_stage_1(+s) → lift_stage_2(+s, mimic) → top_plate_link
      → right_arm_mount_joint (x축 roll +45°) → right_arm_base_link → joint1..6 → end_link(=tool0).

관절값 규약 (팀 data_collection 패치의 실기 팔로워와 같음):
  LeRobot 공개 각도[도] = 모터 각도[도] / joint_directions, DM 프로필은 전 관절 +1.
  팀 중력 보상기는 모터 각도[rad] 를 그대로 URDF q 로 넣는다 → q_urdf = radians(공개 각도).
ROS 와 무관한 순수 Python 모듈이다.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path

import numpy as np
import pinocchio as pin

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")
# 모터 번호 n ↔ LeRobot 이름 ↔ URDF right_arm_joint{n} (collector arm_limits._URDF_JOINT 와 같음).
MOTORS = dict(enumerate(ARM_JOINTS, start=1))
# LeRobot rebot_b601_follower/motor_family.py DM_PROFILE.joint_directions (전부 +1).
JOINT_DIRECTIONS = dict.fromkeys(ARM_JOINTS, 1.0)
# 실기 collector(arm_limits.USER_LIMITS_DEG, 2026-11-15)가 팔로워 한계와 URDF 사본을 넓혀 쓰는 값 [도].
# 1·5번은 URDF 한계를 쓴다. 기록 데이터(2번 −199° 등)가 URDF 한계를 넘는 이유.
USER_LIMITS_DEG = {
    "shoulder_lift": (-210.0, 1.0),   # 2번
    "elbow_flex": (-200.0, 1.0),      # 3번
    "wrist_flex": (-85.0, 120.0),     # 4번
    "wrist_roll": (-180.0, 170.0),    # 6번
}


def joint_name(key: int | str) -> str:
    """모터 번호(1~6) 또는 LeRobot 이름 → LeRobot 이름."""
    if isinstance(key, bool):
        raise KeyError(f"unknown arm joint: {key!r}")
    if isinstance(key, int):
        if key not in MOTORS:
            raise KeyError(f"motor number must be 1..6, got {key}")
        return MOTORS[key]
    if key not in ARM_JOINTS:
        raise KeyError(f"unknown arm joint: {key!r}")
    return key


def by_motor(values: Mapping[str, float]) -> dict[int, float]:
    """LeRobot 이름 키 dict → 모터 번호(1~6) 키 dict. 팔 관절 외 키(gripper 등)는 버린다."""
    return {n: values[name] for n, name in MOTORS.items() if name in values}
LIFT_JOINTS = ("lift_stage_1_joint", "lift_stage_2_joint")  # 2단, stage_2 는 stage_1 mimic(×1)
DEFAULT_URDF = Path(__file__).resolve().parents[2] / "description/urdf/mobile_robot_right_arm.urdf"
TOOL_FRAME = "right_arm_tool0"
# URDF 'world' 링크 = 로봇 바닥 기준점 (base_footprint 역할, TF odom → world). x 앞, y 왼쪽, z 바닥에서 위.
# base_link 는 이보다 20 mm 아래 (world_to_base). 높이를 바닥 기준으로 쓰려고 기본값을 world 로 둔다.
BASE_FRAME = "world"


def public_to_q(joints: Mapping[int | str, float] | Sequence[float]) -> np.ndarray:
    """LeRobot 공개 관절각[도] → URDF q[rad] 6개 (1~6번 모터 순서).

    입력: 모터 번호 키 {1: .., 6: ..}, LeRobot 이름 키 (gripper 등 다른 키는 무시),
    또는 1~6번 순서의 값 6개.
    """
    if isinstance(joints, Mapping):
        named: dict[str, float] = {}
        for key, value in joints.items():
            if isinstance(key, str) and key not in ARM_JOINTS:
                continue  # gripper 등 팔 6축이 아닌 키
            name = joint_name(key)
            if name in named:
                raise KeyError(f"joint given twice: motor {ARM_JOINTS.index(name) + 1} / {name}")
            named[name] = value
        missing = [f"{n}번({MOTORS[n]})" for n in MOTORS if MOTORS[n] not in named]
        if missing:
            raise KeyError(f"missing arm joints: {missing}")
        values = [named[n] for n in ARM_JOINTS]
    else:
        values = list(joints)
        if len(values) != len(ARM_JOINTS):
            raise ValueError(f"expected {len(ARM_JOINTS)} arm joints, got {len(values)}")
    deg = np.array([float(v) * JOINT_DIRECTIONS[n] for v, n in zip(values, ARM_JOINTS)])
    if not np.all(np.isfinite(deg)):
        raise ValueError("non-finite joint value")
    return np.radians(deg)


class B601Kinematics:
    """모바일 베이스 기준 B601 오른팔 FK/자코비안. 각 호출은 상태를 남기지 않는다."""

    def __init__(self, urdf_path: str | Path = DEFAULT_URDF, prefix: str = "right_arm_"):
        self.urdf_path = Path(urdf_path)
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.data = self.model.createData()
        self.prefix = prefix
        arm = [self._joint(f"{prefix}joint{i}") for i in range(1, 7)]
        self._arm_q = [j.idx_q for j in arm]
        self._arm_v = [j.idx_v for j in arm]
        lift = [self._joint(name) for name in LIFT_JOINTS]
        self._lift_q = [j.idx_q for j in lift]
        self.arm_lower = self.model.lowerPositionLimit[self._arm_q].copy()
        self.arm_upper = self.model.upperPositionLimit[self._arm_q].copy()
        for i, name in enumerate(ARM_JOINTS):
            if name in USER_LIMITS_DEG:
                self.arm_lower[i], self.arm_upper[i] = np.radians(USER_LIMITS_DEG[name])
        self.lift_stage_max = float(self.model.upperPositionLimit[self._lift_q[0]])
        fl, fr = f"{prefix}finger_left", f"{prefix}finger_right"
        self._finger_q = ([self._joint(fl).idx_q, self._joint(fr).idx_q]
                          if self.model.existJointName(fl) and self.model.existJointName(fr) else None)
        self.finger_max = float(self.model.upperPositionLimit[self._finger_q[0]]) if self._finger_q else 0.0

    def set_motor_limits_deg(self, limits: dict[int, tuple[float, float]]) -> None:
        """모터 번호별 사용 한계 [도] 를 바꾼다 (예: conservative_limits())."""
        for n, (lo, hi) in limits.items():
            if lo >= hi:
                raise ValueError(f"motor {n}: empty limit {lo}..{hi}")
            self.arm_lower[n - 1], self.arm_upper[n - 1] = np.radians([lo, hi])

    def _joint(self, name):
        if not self.model.existJointName(name):
            raise ValueError(f"{self.urdf_path} has no joint {name!r}")
        return self.model.joints[self.model.getJointId(name)]

    def _frame(self, name: str) -> int:
        if not self.model.existFrame(name):
            raise ValueError(f"{self.urdf_path} has no frame {name!r}")
        return self.model.getFrameId(name)

    def configuration(self, arm, lift_stage_m: float = 0.0, finger_m: float = 0.0) -> np.ndarray:
        """전체 q. `arm` 은 공개 각도[도] (dict/6개). `lift_stage_m` 은 단(stage) 하나의 이동량[m];
        두 단이 같이 움직이므로 상판 높이 변화는 2배다. `finger_m` = 손가락 하나의 벌림 [m] (0 = 닫힘)."""
        if not 0.0 <= lift_stage_m <= self.lift_stage_max + 1e-9:
            raise ValueError(f"lift_stage_m {lift_stage_m} outside [0, {self.lift_stage_max}]")
        q = pin.neutral(self.model)
        q[self._arm_q] = public_to_q(arm)
        q[self._lift_q] = lift_stage_m
        if self._finger_q and finger_m:
            q[self._finger_q[0]], q[self._finger_q[1]] = finger_m, -finger_m   # 오른손가락 = 왼손가락 × −1 (URDF mimic)
        return q

    def motor_limits_deg(self) -> dict[int, tuple[float, float]]:
        """모터 번호별 사용 한계 [도] (URDF + collector USER_LIMITS_DEG)."""
        lo, hi = np.degrees(self.arm_lower), np.degrees(self.arm_upper)
        return {n: (round(float(lo[n - 1]), 6), round(float(hi[n - 1]), 6)) for n in MOTORS}

    def within_arm_limits(self, arm, margin_deg: float = 0.0) -> bool:
        q = public_to_q(arm)
        m = np.radians(margin_deg)
        return bool(np.all(q >= self.arm_lower + m) and np.all(q <= self.arm_upper - m))

    def _placements(self, q, frame, reference):
        fid, rid = self._frame(frame), self._frame(reference)
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[rid], self.data.oMf[fid], fid

    def fk(self, arm, lift_stage_m: float = 0.0, frame: str = TOOL_FRAME,
           reference: str = BASE_FRAME) -> pin.SE3:
        """`reference` 좌표계에서 본 `frame` 의 자세 (pin.SE3: .translation [m], .rotation).
        `arm` 은 public_to_q 가 받는 형식 (모터 번호 키, 이름 키, 1~6번 순서 값)."""
        ref, target, _ = self._placements(self.configuration(arm, lift_stage_m), frame, reference)
        return ref.actInv(target)

    def jacobian(self, arm, lift_stage_m: float = 0.0, frame: str = TOOL_FRAME,
                 reference: str = BASE_FRAME) -> np.ndarray:
        """팔 6관절에 대한 6×6 자코비안 [선속도; 각속도] (열 k = 모터 k+1번), `reference` 축 방향으로 표현,
        기준점은 `frame` 원점. 단위: m/rad, rad/rad."""
        q = self.configuration(arm, lift_stage_m)
        ref, _, fid = self._placements(q, frame, reference)
        pin.computeJointJacobians(self.model, self.data, q)
        J = pin.getFrameJacobian(self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)[:, self._arm_v]
        R = ref.rotation.T
        return np.vstack((R @ J[:3], R @ J[3:]))

    def manipulability(self, arm, lift_stage_m: float = 0.0, frame: str = TOOL_FRAME,
                       length_scale_m: float = 0.1) -> dict[str, float]:
        """팔 자체의 조작성. 좌표축 회전에 불변이라 45° 장착·리프트와 무관하고 관절값에만 의존한다.

        w_pos     위치 조작성 √det(Jv Jvᵀ) [m³]
        w         [Jv; L·Jw] 의 √det (각속도 행을 길이 L 로 맞춘 6D 조작성)
        sigma_min 같은 6D 행렬의 최소 특이값 (0 이면 특이점)
        cond      조건수 σmax/σmin
        """
        J = self.jacobian(arm, lift_stage_m, frame)
        Jn = np.vstack((J[:3], length_scale_m * J[3:]))
        s = np.linalg.svd(Jn, compute_uv=False)
        sv = np.linalg.svd(J[:3], compute_uv=False)
        return {
            "w_pos": float(np.prod(sv)),
            "w": float(np.prod(s)),
            "sigma_min": float(s[-1]),
            "cond": float(s[0] / s[-1]) if s[-1] > 0 else float("inf"),
        }
