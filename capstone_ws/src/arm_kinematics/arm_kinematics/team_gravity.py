#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Vendored verbatim from iDLE-capstone/data_collection@1758334
# patches/rebot_b601_gravity_ff.patch -> src/lerobot/robots/rebot_b601_follower/gravity.py
# (the gravity feed-forward the real follower runs). Do not edit; re-vendor instead.

"""Gravity-torque feed-forward for the reBot B601 follower.

The torque is computed from the arm URDF with Pinocchio. The gravity vector is
expressed in the arm base frame, so a base mounted tilted (for example rolled 45
degrees on a mobile base) is handled by giving the mount roll/pitch.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from pathlib import Path

import numpy as np

_G = 9.81

# Per-joint safety cap on the feed-forward torque in N.m. Gravity torque of this arm
# stays well below these values, so hitting a cap means something is wrong.
DEFAULT_TAU_LIMIT: dict[str, float] = {
    "shoulder_pan": 10.0,
    "shoulder_lift": 10.0,
    "elbow_flex": 10.0,
    "wrist_flex": 3.0,
    "wrist_yaw": 3.0,
    "wrist_roll": 3.0,
}


def gravity_in_base_frame(roll_deg: float, pitch_deg: float = 0.0) -> np.ndarray:
    """Gravity vector in the arm base frame for a base mounted with the given roll/pitch.

    ``roll_deg``/``pitch_deg`` follow the URDF ``rpy`` convention of the joint that
    places the arm base in a gravity-aligned parent frame (R = Ry(pitch) @ Rx(roll)).
    The vector is ``R.T @ (0, 0, -g)``.
    """
    if not all(math.isfinite(v) for v in (roll_deg, pitch_deg)):
        raise ValueError("Mount angles must be finite")
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return (ry @ rx).T @ np.array([0.0, 0.0, -_G])


def load_gravity_model(urdf_path: str | Path, missing_finger_mass_kg: float = 0.0):
    """Shared arm-only model loader for real control and the MuJoCo comparison."""
    import pinocchio as pin

    path = Path(urdf_path).expanduser()
    root = ET.parse(path).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    children = {joint.find("child").attrib["link"] for joint in root.findall("joint")}
    if links - children != {"base_link"}:
        raise ValueError("Gravity model must be arm-only, rooted at base_link (no mounting joint)")
    if not math.isfinite(missing_finger_mass_kg) or missing_finger_mass_kg < 0:
        raise ValueError("Missing finger mass must be finite and nonnegative")
    if missing_finger_mass_kg:
        for name in ("finger_left_link", "finger_right_link"):
            link = root.find(f"./link[@name='{name}']")
            if link is not None and link.find("inertial") is None:
                inertial = ET.SubElement(link, "inertial")
                ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
                ET.SubElement(inertial, "mass", value=str(missing_finger_mass_kg))
                ET.SubElement(
                    inertial,
                    "inertia",
                    ixx="0.00001",
                    iyy="0.00001",
                    izz="0.00001",
                    ixy="0",
                    ixz="0",
                    iyz="0",
                )
    return pin.buildModelFromXML(ET.tostring(root, encoding="unicode"))


class GravityCompensator:
    """Computes gravity torque for the six arm joints from joint positions."""

    def __init__(
        self,
        urdf_path: str | Path,
        joint_names: Sequence[str],
        urdf_joint_names: Sequence[str] | None = None,
        mount_roll_deg: float = 0.0,
        mount_pitch_deg: float = 0.0,
        tau_scale: float = 1.0,
        tau_limit: Mapping[str, float] | None = None,
        missing_finger_mass_kg: float = 0.0,
    ):
        try:
            import pinocchio as pin
        except ImportError as e:  # pragma: no cover - environment dependent
            raise ImportError("Gravity compensation needs Pinocchio: pip install pin") from e

        path = Path(urdf_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"gravity URDF not found: {path}")
        if not math.isfinite(tau_scale) or tau_scale < 0:
            raise ValueError(f"gravity tau_scale must be finite and >= 0, got {tau_scale}")

        self._pin = pin
        self.joint_names = list(joint_names)
        urdf_joints = (
            list(urdf_joint_names)
            if urdf_joint_names
            else [f"joint{i + 1}" for i in range(len(self.joint_names))]
        )
        if len(urdf_joints) != len(self.joint_names):
            raise ValueError("urdf_joint_names must match joint_names in length")

        self.model = load_gravity_model(path, missing_finger_mass_kg)
        self.data = self.model.createData()
        self._idx_q: list[int] = []
        self._idx_v: list[int] = []
        for name in urdf_joints:
            if not self.model.existJointName(name):
                raise ValueError(f"URDF {path} has no joint {name!r}")
            joint = self.model.joints[self.model.getJointId(name)]
            self._idx_q.append(joint.idx_q)
            self._idx_v.append(joint.idx_v)

        self.mount_roll_deg = float(mount_roll_deg)
        self.mount_pitch_deg = float(mount_pitch_deg)
        self.gravity = gravity_in_base_frame(self.mount_roll_deg, self.mount_pitch_deg)
        self.model.gravity = pin.Motion(np.concatenate([self.gravity, np.zeros(3)]))

        self.tau_scale = float(tau_scale)
        limits = {**DEFAULT_TAU_LIMIT, **(dict(tau_limit) if tau_limit else {})}
        self._limit = np.array([limits[n] for n in self.joint_names], dtype=float)
        if not np.all(np.isfinite(self._limit)) or np.any(self._limit <= 0):
            raise ValueError("Gravity torque limits must be finite and positive")
        self._q0 = pin.neutral(self.model)

    def compute(self, q_rad: Mapping[str, float]) -> dict[str, float]:
        """Gravity torque in N.m per joint for motor-frame joint positions in radians.

        Invalid inputs/results raise before any motor command is produced.
        """
        q = self._q0.copy()
        for name, i in zip(self.joint_names, self._idx_q, strict=True):
            q[i] = q_rad[name]
        if not np.all(np.isfinite(q)):
            raise ValueError("Non-finite gravity input")
        tau = np.asarray(self._pin.computeGeneralizedGravity(self.model, self.data, q))[self._idx_v]
        if not np.all(np.isfinite(tau)):
            raise ValueError("Non-finite gravity torque")
        tau = np.clip(tau * self.tau_scale, -self._limit, self._limit)
        if not np.all(np.isfinite(tau)):
            raise ValueError("Non-finite scaled gravity torque")
        return {name: float(t) for name, t in zip(self.joint_names, tau, strict=True)}
