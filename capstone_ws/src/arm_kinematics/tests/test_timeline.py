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

    def test_short_moves_respect_acceleration_limit(self):
        # 10° 이동: 속도만 보면 0.625 s → 최고 가속도 ≈ 148°/s² (> 120). 가속도 하한으로 늘어나야 한다.
        for dist in (3.0, 10.0, 20.0):
            wps = [wp(None, "approach", [0] * 6), wp(None, "approach", [-dist, dist / 2, 0, 0, 0, 0])]
            for w in wps:
                del w["t"]
            tl = Timeline(wps, v_limit_deg=30.0, a_limit_deg=120.0, min_phase_s=0.0)
            dt = 0.0005
            ts = np.arange(0, tl.duration + dt, dt)
            q = np.array([tl.sample(t)[0] for t in ts])
            a = np.abs(np.diff(q, 2, axis=0)) / dt ** 2
            self.assertLessEqual(a.max(), 120.0 * 1.02, dist)
            v = np.abs(np.diff(q, axis=0)) / dt
            self.assertLessEqual(v.max(), 30.0 * 1.01, dist)

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
    """Imitates the team follower closely enough for the player logic:
    first-order lag, optional stalled joint, optional steady offset, 0.25 s command timeout that latches
    an error unless hold() was called (team behaviour), effective joint limits, connect/disconnect."""

    def __init__(self, q0, tau=0.15, stall=None, offset=None, limits=None, timeout=0.25):
        import time
        self.q = np.array(q0, float)
        self.target = self.q.copy()
        self.tau, self.stall, self.sent, self.held = tau, stall, 0, False
        self.offset = np.zeros(6) if offset is None else np.array(offset, float)
        self._joint_limits = limits or {n: (-200.0, 200.0) for n in play_plan.NAMES}
        self.timeout, self.error, self.disconnected = timeout, None, False
        self.t_last = time.perf_counter()

    def connect(self):
        pass

    def _check_timeout(self):
        import time
        if not self.held and self.sent and time.perf_counter() - self.t_last > self.timeout:
            self.error = "target update timeout"

    def send_action(self, action):
        import time
        self._check_timeout()
        if self.error:
            raise RuntimeError(self.error)
        now = time.perf_counter()
        dt = now - self.t_last
        self.t_last, self.held = now, False
        self.target = np.array([action[f"{n}.pos"] for n in play_plan.NAMES])
        self.q += (self.target + self.offset - self.q) * min(1.0, dt / self.tau)
        if self.stall is not None and self.sent < self.stall[1]:
            self.q[self.stall[0]] = 0.0       # joint held back for the first N commands
        self.sent += 1
        return action

    def hold(self):
        self.held = True
        self.target = self.q.copy()

    def disconnect(self):
        self.disconnected = True

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

    def test_loop_hiccup_resumes_without_jump_or_burst(self):
        import time

        class Hiccup(FakeFollower):
            def send_action(self, action):
                if self.sent == 60:
                    time.sleep(0.2)                         # 루프가 이 틱에서 0.2 s 멈춤 (팔로워 0.25 s 한도 안)
                self.cmds.append((time.perf_counter(), np.array([action[f"{n}.pos"] for n in play_plan.NAMES])))
                return super().send_action(action)

        tl = Timeline([wp(0.0, "approach", [0] * 6), wp(1.0, "approach", [-30, -15, 0, 0, 0, 0])], v_limit_deg=30.0)
        robot = Hiccup([0] * 6)
        robot.cmds = []
        stats = {}
        play_plan.play(robot, tl, -10.0, rate_hz=50.0, lag_pause_deg=10.0, stats=stats)
        ts = np.array([c[0] for c in robot.cmds])
        q = np.array([c[1] for c in robot.cmds])
        steps = np.max(np.abs(np.diff(q, axis=0)), axis=1)
        normal = np.max(steps[30:58])
        self.assertLessEqual(steps[58:75].max(), normal * 1.2)      # 멈춘 뒤 목표가 건너뛰지 않는다
        self.assertLessEqual(int(np.sum(np.diff(ts)[61:75] < 0.005)), 1)   # 밀린 명령을 몰아 보내지 않는다
        self.assertGreaterEqual(stats["late_ticks"], 1)
        self.assertGreater(stats["max_gap_s"], 0.18)

    def test_end_offset_inside_pause_band_stops_after_settle(self):
        # 끝에서 3° 늦은 채 머무름 (2° < 3° < 10°): 예전에는 무한히 밀었다
        wps = [wp(0.0, "insert", [0] * 6), wp(0.2, "insert", [-6, 0, 0, 0, 0, 0])]
        tl = Timeline(wps, v_limit_deg=60.0)
        robot = FakeFollower([0] * 6, offset=[3, 0, 0, 0, 0, 0])
        t0 = __import__("time").perf_counter()
        with self.assertRaises(play_plan.Stalled):
            play_plan.play(robot, tl, -10.0, rate_hz=200.0, lag_pause_deg=10.0, settle_s=0.3)
        self.assertTrue(robot.held)
        self.assertLess(__import__("time").perf_counter() - t0, tl.duration + 2.0)

    def test_insert_phase_uses_smaller_lag_threshold(self):
        wps = [wp(0.0, "insert", [0] * 6), wp(0.4, "insert", [-30, 0, 0, 0, 0, 0])]
        tl = Timeline(wps, v_limit_deg=60.0)
        robot = FakeFollower([0] * 6, stall=(0, 10 ** 9))
        with self.assertRaises(play_plan.Stalled) as ctx:
            play_plan.play(robot, tl, -10.0, rate_hz=200.0, lag_pause_deg=10.0, insert_lag_deg=3.0, stall_s=0.2)
        self.assertLess(ctx.exception.lag, 10.0)                       # 10° 가 아니라 3° 에서 멈춤

    def test_return_waypoints_start_at_measured_pose(self):
        tl = Timeline(WAYPOINTS, v_limit_deg=30.0)
        q_now = np.array([-30, -50, -20, 15, 8, 2.0])
        wps = play_plan.return_waypoints(tl, tl.duration * 0.8, q_now)
        np.testing.assert_allclose([wps[0]["motors_deg"][str(i)] for i in range(1, 7)], q_now)
        back = Timeline(wps, v_limit_deg=30.0)
        np.testing.assert_allclose(back.sample(0.0)[0], q_now)
        np.testing.assert_allclose(back.sample(back.duration)[0], np.zeros(6), atol=1e-9)

    def test_reverse_points_retrace_the_path(self):
        tl = Timeline(WAYPOINTS, v_limit_deg=30.0)
        pts = play_plan._reverse_points(tl, tl.duration)
        np.testing.assert_allclose(pts[0][2], tl.sample(tl.duration)[0])
        np.testing.assert_allclose(pts[-1][2], np.zeros(6), atol=1e-9)
        times = [p[0] for p in pts]
        self.assertEqual(times, sorted(times))


class PlayerMainTests(unittest.TestCase):
    """main() with a fake follower: plan file → checks → align → hold → play → return → disconnect."""

    def _plan(self, tmp, **over):
        import json
        wps = [wp(0.0, "lift", [0] * 6), wp(0.5, "lift", [0] * 6), wp(1.0, "approach", [-10, -10, -5, 5, 2, 0]),
               wp(1.4, "insert", [-12, -11, -6, 5, 2, 0])]
        for w in wps:
            w["lift_total_mm"] = 0.0
        plan = {"validated": True, "lift_fixed": True, "v_limit_deg": 30.0, "waypoints": wps, **over}
        path = Path(tmp) / "plan.json"
        path.write_text(json.dumps(plan))
        return path

    def _run(self, plan_path, robot, inputs, extra=()):
        from types import SimpleNamespace
        from unittest import mock

        def fake_input(prompt=""):
            __import__("time").sleep(0.4)                       # 사람이 ENTER 를 누르기까지
            return inputs.pop(0) if inputs else ""
        argv = ["play_plan.py", str(plan_path), "--speed", "1.0", *extra]
        with mock.patch.object(play_plan, "make_robot", lambda a: (robot, SimpleNamespace(port="fake", max_relative_target=15.0))), \
                mock.patch("builtins.input", fake_input), mock.patch.object(sys, "argv", argv), \
                mock.patch.object(play_plan, "apply_gravity_scales", lambda r, s: {}):
            return play_plan.main()

    def test_full_flow_align_hold_play_return(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            robot = FakeFollower([2.0, -1.0, -3.0, 0.5, -0.9, -0.6])   # 쉬는 자세: 경로 시작과 3° 안
            rc = self._run(self._plan(tmp), robot, ["", ""])
        self.assertEqual(rc, 0)
        self.assertIsNone(robot.error)                                # align 뒤 hold 덕분에 명령 끊김 오류 없음
        self.assertTrue(robot.disconnected)
        np.testing.assert_allclose(robot.q, np.zeros(6), atol=1.5)   # 같은 길로 시작까지 복귀

    def test_refuses_unvalidated_or_moving_lift_plans_before_connecting(self):
        import tempfile
        for over in ({"validated": False}, {"lift_fixed": False}):
            with tempfile.TemporaryDirectory() as tmp:
                robot = FakeFollower([0] * 6)
                rc = self._run(self._plan(tmp, **over), robot, [])
            self.assertEqual(rc, 2, over)
            self.assertEqual(robot.sent, 0)
            self.assertFalse(robot.disconnected)                        # 연결조차 안 함

    def test_refuses_plan_outside_follower_limits(self):
        import tempfile
        limits = {n: (-200.0, 200.0) for n in play_plan.NAMES}
        limits["shoulder_pan"] = (-5.0, 5.0)                           # 경로는 −12° 까지 감
        with tempfile.TemporaryDirectory() as tmp:
            robot = FakeFollower([0] * 6, limits=limits)
            rc = self._run(self._plan(tmp), robot, [])
        self.assertEqual(rc, 2)
        self.assertEqual(robot.sent, 0)
        self.assertTrue(robot.disconnected)


if __name__ == "__main__":
    unittest.main()
