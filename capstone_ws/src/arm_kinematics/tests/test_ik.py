import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_kinematics import B601Kinematics  # noqa: E402
from arm_kinematics.collision import Box, CollisionChecker  # noqa: E402
from arm_kinematics.ik import B601IK  # noqa: E402
from arm_kinematics.scene import (  # noqa: E402
    ALONG, BODY_FRONT_X, DESK_TOP, FRONT, LATERAL, PRESETS, Placement, default_env, env_for, preset_target,
    robot_to_room, room_to_robot, shelf_board_top, to_robot)

ROBOT_XML = Path(__file__).resolve().parents[2] / "description/mjcf/mobile_robot_right_arm.xml"


class IKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = B601Kinematics()
        cls.ik = B601IK(cls.kin)
        rng = np.random.default_rng(5)
        lo, hi = np.degrees(cls.kin.arm_lower) + 10, np.degrees(cls.kin.arm_upper) - 10
        cls.poses = [list(rng.uniform(lo, hi)) for _ in range(8)]

    def test_position_only_reaches_fk_targets(self):
        for arm in self.poses:
            target = self.kin.fk(arm, 0.1).translation
            best = self.ik.solve(target, 0.1, n_random=12)[0]
            self.assertTrue(best.converged)
            got = self.kin.fk(best.motors_deg, 0.1).translation
            self.assertLess(np.linalg.norm(got - target) * 1000, 1.0)
            self.assertTrue(self.kin.within_arm_limits(best.motors_deg))

    def test_approach_axis_and_full_rotation(self):
        for arm in self.poses[:4]:
            T = self.kin.fk(arm, 0.0)
            a = T.rotation[:, 0]
            best = self.ik.solve(T.translation, 0.0, approach=a, n_random=12)[0]
            self.assertTrue(best.converged)
            got = self.kin.fk(best.motors_deg, 0.0)
            self.assertLess(math.degrees(math.acos(min(1.0, got.rotation[:, 0] @ a))), 1.0)
            best = self.ik.solve(T.translation, 0.0, rotation=T.rotation, n_random=12)[0]
            self.assertTrue(best.converged)
            got = self.kin.fk(best.motors_deg, 0.0)
            np.testing.assert_allclose(got.rotation, T.rotation, atol=0.02)

    def test_heading_keeps_gripper_facing_direction_with_bounded_tilt(self):
        h = np.array([0.0, -1.0, 0.0])
        for target in ([0.0, -0.55, 0.96], [0.1, -0.5, 1.2], [-0.05, -0.6, 0.7]):
            cands = [c for c in self.ik.solve(target, [0.0, 0.21], heading=h, tilt_deg=(-10, 45), n_random=16) if c.converged]
            self.assertTrue(cands, target)
            for c in cands:
                a = self.kin.fk(c.motors_deg, c.lift_stage_m).rotation[:, 0]
                lateral = math.degrees(math.asin(abs(a[0])))            # x = 옆 방향 (h = −y)
                pitch = math.degrees(math.asin(-a[2]))
                self.assertLess(lateral, 1.0)
                self.assertGreater(a @ h, 0)
                self.assertTrue(-10.5 <= pitch <= 45.5, pitch)

    def test_cost_is_per_candidate_sum_of_terms(self):
        for c in self.ik.solve(self.kin.fk(self.poses[0], 0.0).translation, 0.0, n_random=8):
            self.assertAlmostEqual(c.cost, sum(c.cost_terms.values()), places=3)
            # sigma_min is stored rounded to 5 decimals; compare relatively
            ref = 0.02 / max(c.sigma_min, 1e-4)
            self.assertLess(abs(c.cost_terms["singular"] - ref), 0.01 * ref + 1e-3)

    def test_unreachable_target_gives_no_ok_candidate(self):
        cands = self.ik.solve([3.0, 0.0, 1.0], 0.0, n_random=6)
        self.assertFalse(any(c.ok for c in cands))

    def test_env_box_blocks_solution(self):
        arm = self.poses[1]
        target = self.kin.fk(arm, 0.1).translation
        free = B601IK(self.kin, collision=CollisionChecker(self.kin)).solve(target, 0.1, n_random=8)
        self.assertTrue(any(c.collision is False for c in free))
        wall = CollisionChecker(self.kin, [Box("wall", (0.3, 0.3, 0.3), tuple(target))])   # box around the target
        blocked = B601IK(self.kin, collision=wall).solve(target, 0.1, n_random=8)
        self.assertTrue(all(c.collision for c in blocked))
        self.assertTrue(any("env/wall" in p for c in blocked for pair in c.collision_pairs for p in pair))


class SceneTests(unittest.TestCase):
    def test_dimensions(self):
        # 실측: 1층 0.21 m, +0.34, +0.33, +0.43, 전체 1.477 m
        for level, top in zip(range(1, 5), (0.21, 0.55, 0.88, 1.31)):
            self.assertAlmostEqual(shelf_board_top(level), top)
        self.assertAlmostEqual(DESK_TOP, shelf_board_top(3))
        boxes = env_for("shelf3")
        self.assertAlmostEqual(max(b.center[2] + b.size[2] / 2 for b in boxes), 1.477)
        for level in range(1, 5):
            board = next(b for b in boxes if b.name == f"shelf_board{level}")
            self.assertAlmostEqual(board.center[2] + board.size[2] / 2, shelf_board_top(level))

    def test_room_robot_round_trip(self):
        robot = Placement(1.2, -0.4, 0.7)
        p = np.array([0.3, -0.5, 0.9])
        np.testing.assert_allclose(room_to_robot(robot_to_room(p, robot), robot), p, atol=1e-12)
        boxes = to_robot(default_env(), robot)
        self.assertEqual(len(boxes), len(default_env()))

    def test_furniture_in_front_facing_robot(self):
        self.assertGreater(FRONT, BODY_FRONT_X + 0.05)
        for name in PRESETS:
            target, into, robot = preset_target(name)
            np.testing.assert_allclose(into, [1, 0, 0])
            self.assertEqual((robot.x, robot.y, robot.yaw), (0.0, 0.0, 0.0))
            self.assertAlmostEqual(target[1], LATERAL + ALONG)
            self.assertEqual(LATERAL, 0.0)   # ZED 가 가구 가운데를 본다
            boxes = env_for(name)
            front_face = min(b.center[0] - (b.size[1] if abs(abs(b.yaw) - math.pi / 2) < 1e-6 else b.size[0]) / 2 for b in boxes)
            self.assertAlmostEqual(front_face, FRONT, places=6)

    def test_presets_reachable_from_default_stand(self):
        kin = B601Kinematics()
        for name in PRESETS:
            front = 0.50 if name == "shelf1" else FRONT       # 1층은 0.50 m 에서만 됨 (stand_sweep.py)
            target, into, _ = preset_target(name, front=front)
            ik = B601IK(kin, collision=CollisionChecker(kin, env_for(name, front)))
            cands = ik.solve(target, [0.0, 0.105, 0.21, 0.315], heading=into, n_random=16)
            self.assertTrue(any(c.ok for c in cands), name)


class MotionTests(unittest.TestCase):
    def test_shelf_plan_is_collision_free_and_ends_on_target(self):
        from arm_kinematics.motion import MotionPlanner, sample
        kin = B601Kinematics()
        target, into, _ = preset_target("shelf3")
        checker = CollisionChecker(kin, env_for("shelf3"))
        ik = B601IK(kin, collision=checker)
        cands = ik.solve(target, [0.0, 0.105, 0.21], heading=into, n_random=16)
        plan = MotionPlanner(ik).plan({i: 0.0 for i in range(1, 7)}, target, cands, heading=into)
        self.assertIsNotNone(plan.candidate)
        end = plan.waypoints[-1]
        self.assertLess(np.linalg.norm(kin.fk(end.motors_deg, end.lift_stage_m).translation - target) * 1000, 2.0)
        for t in np.linspace(0, plan.duration, 200):
            q, s, _ = sample(plan, t)
            self.assertFalse(checker.in_collision(list(np.degrees(q)), s), f"t={t:.2f}")
        times = [w.t for w in plan.waypoints]
        self.assertEqual(times, sorted(times))


class MujocoTests(unittest.TestCase):
    def test_mujoco_matches_pinocchio(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("mujoco not installed")
        from arm_kinematics.scene import build_mujoco, set_mujoco_pose
        kin = B601Kinematics()
        model, data = build_mujoco(ROBOT_XML, default_env())
        tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_arm_tool0")
        rng = np.random.default_rng(2)
        lo, hi = np.degrees(kin.arm_lower), np.degrees(kin.arm_upper)
        for robot in (Placement(0, 0, 0), Placement(0.4, -0.2, 0.7)):
            c, s = math.cos(robot.yaw), math.sin(robot.yaw)
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            for _ in range(10):
                arm, lift = list(rng.uniform(lo, hi)), rng.uniform(0, 0.315)
                set_mujoco_pose(model, data, robot, arm, lift)
                T = kin.fk(arm, lift)
                np.testing.assert_allclose(room_to_robot(data.xpos[tool], robot), T.translation, atol=1e-5)
                np.testing.assert_allclose(Rz.T @ data.xmat[tool].reshape(3, 3), T.rotation, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
