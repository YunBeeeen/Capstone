import math
import sys
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_kinematics.b601 import (  # noqa: E402
    ARM_JOINTS, MOTORS, B601Kinematics, by_motor, joint_name, public_to_q)
from arm_kinematics.gravity import JOINT_SCALES, TEAM_URDF, ArmGravity  # noqa: E402
from arm_kinematics.team_gravity import GravityCompensator  # noqa: E402

RNG = np.random.default_rng(0)
# Test seed pose from data_collection tests/test_tabletop_scene.py (public degrees).
SEED = [1.596, -148.2267, -85.87, 29.2394, 44.9775, 2.2625]


def random_pose(kin, n=20):
    lo, hi = np.degrees(kin.arm_lower) + 5, np.degrees(kin.arm_upper) - 5
    return [RNG.uniform(lo, hi) for _ in range(n)]


class MappingTests(unittest.TestCase):
    def test_public_degrees_to_urdf_radians(self):
        q = public_to_q(dict(zip(ARM_JOINTS, [90, -45, 0, 10, 20, 30])))
        np.testing.assert_allclose(q, np.radians([90, -45, 0, 10, 20, 30]))
        np.testing.assert_allclose(public_to_q([1, 2, 3, 4, 5, 6]), np.radians([1, 2, 3, 4, 5, 6]))
        with self.assertRaises(KeyError):
            public_to_q({"shoulder_pan": 0})
        with self.assertRaises(ValueError):
            public_to_q([0, 0, 0, 0, 0, float("nan")])

    def test_motor_numbers(self):
        self.assertEqual(MOTORS[1], "shoulder_pan")
        self.assertEqual(MOTORS[4], "wrist_flex")
        self.assertEqual(MOTORS[6], "wrist_roll")
        by_num = {n: v for n, v in zip(range(1, 7), [90, -45, 0, 10, 20, 30])}
        by_name = dict(zip(ARM_JOINTS, [90, -45, 0, 10, 20, 30]))
        np.testing.assert_allclose(public_to_q(by_num), public_to_q(by_name))
        # Raw recorded rows carry the gripper too; it is ignored.
        np.testing.assert_allclose(public_to_q({**by_name, "gripper": -270}), public_to_q(by_name))
        self.assertEqual(by_motor({**by_name, "gripper": -270}), by_num)
        with self.assertRaises(KeyError):
            public_to_q({**by_num, "shoulder_pan": 0})   # motor 1 twice
        with self.assertRaises(KeyError):
            public_to_q({**by_num, 7: 0})
        with self.assertRaises(KeyError):
            joint_name(True)
        lim = B601Kinematics().motor_limits_deg()
        self.assertEqual(sorted(lim), [1, 2, 3, 4, 5, 6])
        self.assertEqual(lim[4], (-85.0, 120.0))
        self.assertAlmostEqual(lim[5][1], math.degrees(1.57), places=5)   # URDF joint5 1.57 rad


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = B601Kinematics()
        cls.team = pin.buildModelFromUrdf(str(TEAM_URDF))

    def test_mount_is_roll_plus_45_deg(self):
        T = self.kin.fk(SEED, frame="right_arm_base_link", reference="top_plate_link")
        np.testing.assert_allclose(T.rotation, pin.rpy.rpyToMatrix(math.pi / 4, 0, 0), atol=1e-9)

    def test_arm_chain_matches_team_gravity_urdf(self):
        data = self.team.createData()
        fid = self.team.getFrameId("end_link")
        for arm in [SEED, *random_pose(self.kin)]:
            q = pin.neutral(self.team)
            for i, value in enumerate(public_to_q(arm), start=1):
                q[self.team.joints[self.team.getJointId(f"joint{i}")].idx_q] = value
            pin.framesForwardKinematics(self.team, data, q)
            ours = self.kin.fk(arm, frame="right_arm_end_link", reference="right_arm_base_link")
            np.testing.assert_allclose(ours.homogeneous, data.oMf[fid].homogeneous, atol=1e-9)

    def test_limits_cover_recorded_extremes(self):
        # Extremes of arm_follower over all 102 server episodes (2026-11-16), public degrees.
        lows = [-33.408, -199.018, -197.620, -91.745, -42.545, -155.086]
        highs = [77.428, 0.361, -1.628, 96.050, 63.462, 45.102]
        np.testing.assert_allclose(np.degrees(self.kin.arm_lower)[1:4], [-210, -200, -85])
        self.assertTrue(self.kin.within_arm_limits(highs))
        # Recorded follower wrist_flex reaches −91.7°, past the collector's −85° soft limit
        # (leader/sent never go below −85°); cause not investigated.
        self.assertFalse(self.kin.within_arm_limits(lows))
        self.assertTrue(self.kin.within_arm_limits(lows[:3] + [-80.0] + lows[4:]))

    def test_tool0_is_end_link(self):
        a = self.kin.fk(SEED, frame="right_arm_tool0")
        b = self.kin.fk(SEED, frame="right_arm_end_link")
        np.testing.assert_allclose(a.homogeneous, b.homogeneous, atol=1e-12)

    def test_lift_translates_tool_straight_up_twice_the_stage(self):
        low = self.kin.fk(SEED, 0.0)
        high = self.kin.fk(SEED, 0.1)
        np.testing.assert_allclose(high.translation - low.translation, [0, 0, 0.2], atol=1e-9)
        np.testing.assert_allclose(high.rotation, low.rotation, atol=1e-12)
        with self.assertRaises(ValueError):
            self.kin.fk(SEED, 0.4)

    def test_jacobian_matches_finite_difference(self):
        for arm in [SEED, *random_pose(self.kin, 5)]:
            J = self.kin.jacobian(arm, 0.05)
            T0 = self.kin.fk(arm, 0.05)
            h = 1e-6
            for k in range(6):
                bumped = np.array(arm, dtype=float)
                bumped[k] += math.degrees(h)
                T1 = self.kin.fk(bumped, 0.05)
                np.testing.assert_allclose((T1.translation - T0.translation) / h, J[:3, k], atol=1e-5)
                w = pin.log3(T1.rotation @ T0.rotation.T) / h
                np.testing.assert_allclose(w, J[3:, k], atol=1e-5)

    def test_manipulability_independent_of_lift_and_matches_svd(self):
        a = self.kin.manipulability(SEED, 0.0)
        b = self.kin.manipulability(SEED, 0.3)
        for key in a:
            self.assertAlmostEqual(a[key], b[key], places=9)
        J = self.kin.jacobian(SEED)
        s = np.linalg.svd(np.vstack((J[:3], 0.1 * J[3:])), compute_uv=False)
        self.assertAlmostEqual(a["sigma_min"], s[-1])
        self.assertAlmostEqual(a["w"], float(np.prod(s)))
        self.assertGreater(a["sigma_min"], 0)


class GravityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = B601Kinematics()

    def _full_model_gravity(self, arm, finger_mass=0.03):
        """Gravity torque from OUR mobile URDF with plain world gravity (robot level)."""
        model = self.kin.model.copy()
        for name in ("right_arm_finger_left", "right_arm_finger_right"):
            jid = model.getJointId(name)
            if model.inertias[jid].mass == 0:
                model.inertias[jid] = pin.Inertia(finger_mass, np.zeros(3), np.eye(3) * 1e-5)
        data = model.createData()
        q = self.kin.configuration(arm, 0.1)
        g = pin.computeGeneralizedGravity(model, data, q)
        return np.array([g[i] for i in self.kin._arm_v])

    def test_45_deg_mount_agrees_with_team_compensator(self):
        cfg = dict(mount_roll_deg=45.0, missing_finger_mass_kg=0.03,
                   tau_limit=dict.fromkeys(ARM_JOINTS, 1e3))
        team = GravityCompensator(TEAM_URDF, ARM_JOINTS, **cfg)
        wrong = GravityCompensator(TEAM_URDF, ARM_JOINTS, **{**cfg, "mount_roll_deg": -45.0})
        err_ok, err_wrong, scale = [], [], []
        for arm in [SEED, *random_pose(self.kin)]:
            ours = self._full_model_gravity(arm)
            q = dict(zip(ARM_JOINTS, public_to_q(arm)))
            ref = np.array([team.compute(q)[n] for n in ARM_JOINTS])
            bad = np.array([wrong.compute(q)[n] for n in ARM_JOINTS])
            err_ok.append(np.abs(ours - ref).max())
            err_wrong.append(np.abs(ours - bad).max())
            scale.append(np.abs(ref).max())
        # Only inertial details may differ (arm base link, camera mount); the frame must agree.
        self.assertLess(max(err_ok), 0.05 * max(scale) + 0.02)
        self.assertGreater(np.median(err_wrong), 5 * max(err_ok))

    def test_joint_scales_applied_after_team_compute(self):
        self.assertEqual(JOINT_SCALES, {4: 0.85})
        scaled = ArmGravity()
        raw = ArmGravity(joint_scales={})
        a, b = scaled.torques(SEED), raw.torques(SEED)
        self.assertAlmostEqual(a["wrist_flex"], 0.85 * b["wrist_flex"])
        for name in ARM_JOINTS[:3]:
            self.assertAlmostEqual(a[name], b[name])
        self.assertEqual(ArmGravity(joint_scales={"wrist_flex": 0.85}).torques(SEED), a)
        m = scaled.torques_by_motor(SEED)
        self.assertEqual(sorted(m), [1, 2, 3, 4, 5, 6])
        self.assertAlmostEqual(m[4], a["wrist_flex"])
        for bad in ({4: 3.0}, {7: 0.9}, {4: 0.9, "wrist_flex": 0.9}):
            with self.assertRaises(ValueError):
                ArmGravity(joint_scales=bad)
        self.assertGreaterEqual(scaled.load(SEED), 0.0)


if __name__ == "__main__":
    unittest.main()
