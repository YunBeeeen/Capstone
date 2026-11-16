"""arm_kinematics: B601 팔 FK·IK·충돌·경로 계획, 임무 시뮬, 물체 3D (vision).

팔 모델(b601, pinocchio 필요) 은 실제로 쓸 때 불러온다 — vision·checks·timeline 같은 numpy 만 쓰는 모듈은
pinocchio 가 없는 곳 (Jetson 기본 파이썬, 라파) 에서도 `from arm_kinematics.vision import …` 로 쓸 수 있게.
"""
__all__ = ["ARM_JOINTS", "MOTORS", "B601Kinematics", "by_motor", "joint_name", "public_to_q"]


def __getattr__(name):
    if name in __all__:
        from . import b601
        return getattr(b601, name)
    raise AttributeError(f"module 'arm_kinematics' has no attribute {name!r}")
