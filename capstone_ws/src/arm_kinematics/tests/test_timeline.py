import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics.timeline import PEAK, Timeline  # noqa: E402

spec = importlib.util.spec_from_file_location("play_plan", ROOT / "tools/play_plan.py")
play_plan = importlib.util.module_from_spec(spec)
spec.loader.exec_module(play_plan)


def wp(t, phase, q):
    return {"t": t, "phase": phase, "motors_deg": {str(i): float(v) for i, v in enumerate(q, start=1)}}


WAYPOINTS = [
    wp(0.0, "lift", [0] * 6), wp(2.0, "lift", [0] * 6),
    wp(5.0, "approach", [-40, -60, -30, 20, 10, 0]),
    wp(5.3, "insert", [-41, -61, -30, 20, 10, 0]), wp(5.6, "insert", [-42, -62, -30, 21, 10, 0]),
    wp(5.9, "insert", [-43, -63, -31, 21, 10, 0]),
]


class TimelineTests(unittest.TestCase):
    def test_endpoints_phases_and_speed_limit(self):
        tl = Timeline(WAYPOINTS, v_limit_deg=30.0)
        np.testing.assert_allclose(tl.sample(0.0)[0], np.zeros(6))
        np.testing.assert_allclose(tl.sample(tl.duration)[0], [-43, -63, -31, 21, 10, 0])
        self.assertEqual(tl.sample(tl.phase_end("approach") - 1e-6)[1], "approach")
        np.testing.assert_allclose(tl.sample(tl.phase_end("approach"))[0], [-40, -60, -30, 20, 10, 0], atol=1e-6)
        ts = np.linspace(0, tl.duration, 6000)
        q = np.array([tl.sample(t)[0] for t in ts])
        v = np.max(np.abs(np.diff(q, axis=0)), axis=1) / np.diff(ts)
        self.assertLessEqual(v.max(), 30.0 * 1.01)

    def test_planned_times_are_respected_at_peak(self):
        tl = Timeline(WAYPOINTS, v_limit_deg=30.0)
        insert = [s for s in tl.segments if s[0] == "insert"][0]
        self.assertGreaterEqual(insert[3], PEAK * 0.9 - 1e-9)        # planned 0.9 s
        lift = [s for s in tl.segments if s[0] == "lift"][0]
        self.assertGreaterEqual(lift[3], PEAK * 2.0 - 1e-9)

    def test_speed_scale_and_reverse(self):
        a, b = Timeline(WAYPOINTS), Timeline(WAYPOINTS, speed=0.5)
        self.assertAlmostEqual(b.duration, 2 * a.duration)
        r = a.reversed()
        np.testing.assert_allclose(r.sample(0)[0], a.sample(a.duration)[0])
        np.testing.assert_allclose(r.sample(r.duration)[0], np.zeros(6), atol=1e-9)
        with self.assertRaises(ValueError):
            Timeline(WAYPOINTS, speed=0.0)


class FakeFollower:
    """Measured position chases the target with a first-order lag; can stall one joint."""

    def __init__(self, q0, tau=0.15, stall=None):
        self.q = np.array(q0, float)
        self.target = self.q.copy()
        self.tau, self.stall, self.t_last, self.sent, self.held = tau, stall, None, 0, False

    def send_action(self, action):
        import time
        now = time.perf_counter()
        dt = 0.0 if self.t_last is None else now - self.t_last
        self.t_last = now
        self.target = np.array([action[f"{n}.pos"] for n in play_plan.NAMES])
        self.q += (self.target - self.q) * min(1.0, dt / self.tau)
        if self.stall is not None and self.sent < self.stall[1]:
            self.q[self.stall[0]] = 0.0       # joint held back for the first N commands
        self.sent += 1
        return action

    def hold(self):
        self.held = True
        self.target = self.q.copy()

    def get_observation(self):
        obs = {f"{n}.pos": float(v) for n, v in zip(play_plan.NAMES, self.q)}
        obs["gripper.pos"] = -10.0
        return obs


class PlayerTests(unittest.TestCase):
    def test_play_reaches_end_and_pauses_on_lag(self):
        wps = [wp(0.0, "approach", [0] * 6), wp(0.3, "approach", [-12, 0, 0, 0, 0, 0])]
        tl = Timeline(wps, v_limit_deg=60.0)
        robot = FakeFollower([0] * 6, stall=(0, 60))
        t = play_plan.play(robot, tl, -10.0, rate_hz=200.0, lag_pause_deg=5.0)
        self.assertAlmostEqual(t, tl.duration)
        np.testing.assert_allclose(robot.q, [-12, 0, 0, 0, 0, 0], atol=2.0)
        self.assertGreater(robot.sent, 60)

    def test_blocked_joint_stops_with_hold_and_retreats_from_actual_pose(self):
        wps = [wp(0.0, "insert", [0] * 6), wp(0.4, "insert", [-30, 0, 0, 0, 0, 0])]
        tl = Timeline(wps, v_limit_deg=60.0)
        robot = FakeFollower([0] * 6, stall=(0, 10 ** 9))          # 1번 관절이 영영 막힘 (장애물)
        with self.assertRaises(play_plan.Stalled) as ctx:
            play_plan.play(robot, tl, -10.0, rate_hz=200.0, lag_pause_deg=5.0, stall_s=0.3)
        self.assertTrue(robot.held)
        e = ctx.exception
        self.assertLess(e.t_plan, tl.duration)                       # 끝까지 밀지 않았다
        t_back = play_plan.closest_time(tl, e.q_meas, e.t_plan)
        self.assertLess(t_back, e.t_plan)                             # 실제 팔 위치 = 경로 앞쪽
        self.assertLess(abs(tl.sample(t_back)[0][0] - e.q_meas[0]), 1.0)

    def test_reverse_points_retrace_the_path(self):
        tl = Timeline(WAYPOINTS, v_limit_deg=30.0)
        pts = play_plan._reverse_points(tl, tl.duration)
        np.testing.assert_allclose(pts[0][2], tl.sample(tl.duration)[0])
        np.testing.assert_allclose(pts[-1][2], np.zeros(6), atol=1e-9)
        times = [p[0] for p in pts]
        self.assertEqual(times, sorted(times))


if __name__ == "__main__":
    unittest.main()
