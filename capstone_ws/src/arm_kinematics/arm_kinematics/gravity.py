"""실기 팔로워와 같은 중력 토크 (팀 GravityCompensator + config.env + collector 관절별 배율).

팀 설정 그대로:
  data_collection/config.env  GRAVITY_URDF=assets/B601_DM_wrist_d435i_no_plate.urdf,
  GRAVITY_MOUNT_ROLL_DEG=45, PITCH=0, TAU_SCALE=1.0, FINGER_MASS_KG=0.03
  collector/gravity_scaling.apply_joint_scales: compute() 결과(클립 후)에 관절별 배율을 곱한다.
45° 는 팔 기준 좌표의 중력 방향으로만 들어가므로 베이스가 수평일 때 유효하다.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path

from .b601 import ARM_JOINTS, by_motor, joint_name, public_to_q
from .team_gravity import DEFAULT_TAU_LIMIT, GravityCompensator

TEAM_URDF = Path(__file__).resolve().parent / "assets/B601_DM_wrist_d435i_no_plate.urdf"
TEAM_CONFIG = {
    "mount_roll_deg": 45.0,
    "mount_pitch_deg": 0.0,
    "tau_scale": 1.0,
    "missing_finger_mass_kg": 0.03,
}
# 실기 collector 가 apply_joint_scales 로 거는 관절별 배율.
# 모터 번호 키. 4번(wrist_flex) 0.85 는 사용자 기억 값 — 수집 PC 설정과 대조할 것.
JOINT_SCALES = {4: 0.85}


def _validate_scales(scales: Mapping[int | str, float]) -> dict[str, float]:
    """collector.gravity_scaling.apply_joint_scales 와 같은 검사. 키는 모터 번호 또는 이름."""
    out = {}
    for key, value in scales.items():
        try:
            name = joint_name(key)
        except KeyError:
            raise ValueError(f"unknown joint: {key!r}") from None
        if name in out:
            raise ValueError(f"scale given twice for {name!r}")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"invalid scale for {name!r}: {value!r}")
        value = float(value)
        if not math.isfinite(value) or not 0.0 <= value <= 2.0:
            raise ValueError(f"invalid scale for {name!r}: {value!r}")
        out[name] = value
    return out


class ArmGravity:
    """공개 관절각[도] → 실기 팔로워가 내는 중력 feed-forward 토크[N·m]."""

    def __init__(self, urdf_path: str | Path = TEAM_URDF,
                 joint_scales: Mapping[int | str, float] = JOINT_SCALES, **config):
        unknown = set(config) - set(TEAM_CONFIG)
        if unknown:
            raise TypeError(f"unknown gravity options: {sorted(unknown)}")
        self.config = {**TEAM_CONFIG, **config}
        self.joint_scales = _validate_scales(joint_scales)
        self.compensator = GravityCompensator(urdf_path, ARM_JOINTS, **self.config)

    def torques(self, arm) -> dict[str, float]:
        q = dict(zip(ARM_JOINTS, public_to_q(arm)))
        tau = self.compensator.compute(q)
        for name, factor in self.joint_scales.items():
            tau[name] *= factor
        return tau

    def torques_by_motor(self, arm) -> dict[int, float]:
        """torques() 를 모터 번호(1~6) 키로."""
        return by_motor(self.torques(arm))

    def load(self, arm) -> float:
        """관절별 |τ| / 안전 상한 의 최댓값 (0~1). 비용 함수용."""
        tau = self.torques(arm)
        return max(abs(tau[n]) / DEFAULT_TAU_LIMIT[n] for n in ARM_JOINTS)


def team_urdf_limits_deg() -> dict[int, tuple[float, float]]:
    """팀 중력 URDF (실기 팔로워가 한계 교집합에 쓰는 원본) 의 관절 1~6 한계 [도]."""
    import numpy as np
    import pinocchio as pin
    m = pin.buildModelFromUrdf(str(TEAM_URDF))
    out = {}
    for n in range(1, 7):
        j = m.joints[m.getJointId(f"joint{n}")]
        out[n] = tuple(float(v) for v in np.degrees([m.lowerPositionLimit[j.idx_q], m.upperPositionLimit[j.idx_q]]))
    return out


def conservative_limits(kin) -> dict[int, tuple[float, float]]:
    """kin 의 한계 (URDF + collector) ∩ 팀 중력 URDF 한계. data_collection 설정 그대로인 팔로워가
    실제로 허용하는 쪽에 가깝다 (collector 처럼 넓힌 URDF 사본을 쓰지 않는 경우)."""
    team, cur = team_urdf_limits_deg(), kin.motor_limits_deg()
    return {n: (round(max(cur[n][0], team[n][0]), 3), round(min(cur[n][1], team[n][1]), 3)) for n in cur}
