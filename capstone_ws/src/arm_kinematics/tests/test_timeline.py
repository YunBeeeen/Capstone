import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics.timeline import Timeline  # noqa: E402

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
        # 계획 시각이 있으면 최고 속도가 계획 평균 속도를 넘지 않는다 (삽입 3 cm/s·리프트 50 mm/s 를 최고 속도로 지킴)
        tl = Timeline(WAYPOINTS, v_limit_deg=30.0)
        insert = [s for s in tl.segments if s[0] == "insert"][0]
        self.assertGreaterEqual(insert[3], 0.9 - 1e-9)                     # planned 0.9 s 보다 짧지 않음
        k = [s[0] for s in tl.segments].index("insert")
        ts = np.linspace(tl.starts[k], tl.starts[k + 1], 4001)
        q = np.array([tl.sample(t)[0] for t in ts])
        v = np.max(np.abs(np.diff(q, axis=0)), axis=1) / np.diff(ts)
        self.assertLessEqual(v.max(), insert[2][-1] / 0.9 * 1.01)           # 최고 속도 ≤ 계획 평균
        lift = [s for s in tl.segments if s[0] == "lift"][0]
        self.assertGreaterEqual(lift[3], 2.0 - 1e-9)
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

    def __init__(self, q0, tau=0.15, stall=None, offset=None, limits=None, timeout=0.25, grip_object=None, status=None):
        """grip_object: 손가락 사이 물체가 막는 각도 [공개 도] (닫기가 거기서 멈춤). None 이면 빈손.
        status: 모터 이름 → 상태 코드 (기본 모두 1 = 켜짐, 0 = 꺼짐)."""
        import time
        from types import SimpleNamespace
        self._codes = {n: 1 for n in [*play_plan.NAMES, "gripper"]}
        self._codes.update(status or {})
        self.motors = {n: SimpleNamespace(get_state=lambda n=n: SimpleNamespace(
            status_code=self._codes[n], t_mos=35.0, t_rotor=35.0)) for n in self._codes}
        self.config = SimpleNamespace(max_temperature_c=75.0)
        self._control_error = None
        self.g, self.g_target, self.grip_object = -10.0, -10.0, grip_object
        self.q = np.array(q0, float)
        self.target = self.q.copy()
        self.tau, self.stall, self.sent, self.held = tau, stall, 0, False
        self.offset = np.zeros(6) if offset is None else np.array(offset, float)
        self._joint_limits = limits or {n: (-200.0, 200.0) for n in play_plan.NAMES}
        self.timeout, self.error, self.disconnected = timeout, None, False
        self.t_last = time.perf_counter()

    def connect(self):
        pass

    def _read_feedback(self):
        from types import SimpleNamespace
        return {n: SimpleNamespace(status_code=c) for n, c in self._codes.items()}

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
        self.g_target = float(action["gripper.pos"])
        step = float(np.clip(self.g_target - self.g, -300.0 * dt, 300.0 * dt))   # 그리퍼 300°/s
        self.g += step
        if self.grip_object is not None and step > 0 and self.g > self.grip_object:
            self.g = self.grip_object                                            # 물체에 막힘 (힘 제한)
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
        obs["gripper.pos"] = self.g
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

    def test_grip_open_then_close_on_object_and_return_holding(self):
        import contextlib
        import io
        import tempfile
        out = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(out):
            robot = FakeFollower([0.0] * 6, grip_object=-120.0)
            rc = self._run(self._plan(tmp), robot, ["", ""], ["--grip-before", "open", "--grip-after", "close"])
        self.assertEqual(rc, 0)
        self.assertIsNone(robot.error)
        self.assertIn("잡힘", out.getvalue())
        self.assertAlmostEqual(robot.g, -120.0, delta=1.0)               # 물체에서 멈춤
        self.assertEqual(robot.g_target, 0.0)                            # 돌아오는 동안에도 쥔 채
        np.testing.assert_allclose(robot.q, np.zeros(6), atol=1.5)

    def test_close_without_object_warns_empty(self):
        import contextlib
        import io
        import tempfile
        out = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(out):
            robot = FakeFollower([0.0] * 6)
            rc = self._run(self._plan(tmp), robot, ["", "q"], ["--grip-before", "open", "--grip-after", "close"])
        self.assertEqual(rc, 0)
        self.assertIn("빈손", out.getvalue())
        self.assertGreater(robot.g, -3.0)

    def test_hold_drift_small_settle_passes_real_sag_fails(self):
        import contextlib
        import io
        import time as _t

        class Sagging(FakeFollower):
            def __init__(self, rate_deg_s, grip_jump=0.0):
                super().__init__([0.0] * 6)
                self.rate, self.t_hold, self.grip_jump = rate_deg_s, None, grip_jump

            def hold(self):
                super().hold()
                self.t_hold = self.t_hold or _t.perf_counter()

            def get_observation(self):
                obs = super().get_observation()
                if self.t_hold is not None:
                    obs["shoulder_lift.pos"] -= self.rate * (_t.perf_counter() - self.t_hold)
                    obs["gripper.pos"] += self.grip_jump
                return obs

        with contextlib.redirect_stdout(io.StringIO()):
            ok = play_plan.preflight(Sagging(0.8, grip_jump=5.0), hold_s=1.0)     # 1 s 에 0.8° + 그리퍼 5° → 정상
            bad = play_plan.preflight(Sagging(5.0), hold_s=1.0)                  # 1 s 에 5° 처짐 → 멈춤
        self.assertEqual(ok, [])
        self.assertTrue(any("모터 2" in b for b in bad), bad)

    def test_disabled_gripper_blocks_any_motion(self):
        import contextlib
        import io
        import tempfile
        out = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(out):
            robot = FakeFollower([0.0] * 6, status={"gripper": 0})
            rc = self._run(self._plan(tmp), robot, ["", ""], ["--grip-before", "open"])
        self.assertEqual(rc, 2)
        self.assertEqual(robot.sent, 0)                                  # 명령을 하나도 안 보냄
        self.assertIn("그리퍼 꺼짐", out.getvalue())
        self.assertTrue(robot.disconnected)

    def test_gripper_only_command_holds_arm(self):
        import contextlib
        import io
        from types import SimpleNamespace
        from unittest import mock
        robot = FakeFollower([5.0, -30.0, -20.0, 10.0, 0.0, 0.0])
        q_before = robot.q.copy()
        argv = ["play_plan.py", "--gripper", "open"]
        with mock.patch.object(play_plan, "make_robot", lambda a: (robot, SimpleNamespace(port="fake", max_relative_target=15.0))), \
                mock.patch.object(sys, "argv", argv), mock.patch.object(play_plan, "apply_gravity_scales", lambda r, s: {}), \
                contextlib.redirect_stdout(io.StringIO()):
            rc = play_plan.main()
        self.assertEqual(rc, 0)
        self.assertAlmostEqual(robot.g, -270.0, delta=2.0)
        np.testing.assert_allclose(robot.q, q_before, atol=0.1)          # 팔은 그대로
        self.assertTrue(robot.disconnected)

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


class CtrlCReturnTests(unittest.TestCase):
    _plan = PlayerMainTests._plan
    def test_ctrl_c_during_motion_returns_only_travelled_path(self):
        self._interrupt_case("moving")

    def test_ctrl_c_at_target_returns(self):
        self._interrupt_case("target")

    def test_second_ctrl_c_stops_return(self):
        self._interrupt_case("return")

    def test_ctrl_c_before_play_does_not_start_return(self):
        self._interrupt_case("before")

    def _interrupt_case(self, where):
        import tempfile
        from types import SimpleNamespace
        from unittest import mock
        robot = FakeFollower([0.0] * 6)
        calls = []
        prompts = []
        travelled = None

        def fake_play(r, tl, gripper, *a, **kw):
            nonlocal travelled
            is_back = kw.get("label") == "[복귀] "
            calls.append((tl, is_back))
            if is_back:
                self.assertTrue(all(ph.startswith("back_") for ph, *_ in tl.segments))
                if where == "moving":
                    # Never visit a later part of the outgoing trajectory.
                    self.assertLessEqual(max(abs(pts[:, 0]).max() for _, pts, *_ in tl.segments), abs(travelled[0]) + 1e-8)
                if where == "return":
                    raise KeyboardInterrupt
                r.q = tl.sample(tl.duration)[0]
            else:
                t = tl.phase_end("approach") * 0.8 if where == "moving" else kw["t_end"]
                r.q = tl.sample(t)[0]
                travelled = r.q.copy()
                kw["stats"]["t_plan"] = t
                if where == "moving":
                    raise KeyboardInterrupt

        def fake_input(prompt=""):
            prompts.append(prompt)
            if len(prompts) == 1 and where == "before":
                raise KeyboardInterrupt
            if len(prompts) == 2:
                # The worker holds while waiting at the goal.
                self.assertTrue(robot.held)
                raise KeyboardInterrupt
            return ""

        with tempfile.TemporaryDirectory() as tmp:
            path = self._plan(tmp)
            argv = ["play_plan.py", str(path)]
            with mock.patch.object(play_plan, "make_robot", lambda a: (robot, SimpleNamespace(port="fake", max_relative_target=15.0))), mock.patch.object(play_plan, "apply_gravity_scales", lambda r, s: {}), mock.patch.object(play_plan, "play", fake_play), mock.patch("builtins.input", fake_input), mock.patch.object(sys, "argv", argv):
                self.assertEqual(play_plan.main(), 0)
        self.assertTrue(robot.disconnected)
        if where == "before":
            self.assertEqual(calls, [])
        else:
            self.assertEqual(len(calls), 2)
            self.assertTrue(calls[-1][1])
            if where != "return":
                np.testing.assert_allclose(robot.q, np.zeros(6), atol=1e-8)


class ReturnTimingAndEnableTests(unittest.TestCase):
    def test_return_does_not_apply_speed_and_quintic_factors_twice(self):
        T = play_plan.load_timeline_module()
        for speed in (0.5, 1.0):
            tl = Timeline(WAYPOINTS, v_limit_deg=30.0, speed=speed)
            q = tl.sample(tl.duration)[0]
            back = play_plan.build_return_timeline(T, tl, tl.duration, q, 30.0, 120.0, speed)
            self.assertLess(back.duration, tl.duration + 3.0)
            self.assertGreater(back.duration, tl.duration - 0.2)
            ts = np.linspace(0, back.duration, 20001)
            qs = np.array([back.sample(t)[0] for t in ts])
            self.assertLessEqual(np.max(np.abs(np.diff(qs, axis=0))) / (ts[1] - ts[0]), 30.0 * speed * 1.01)
            np.testing.assert_allclose(back.sample(0)[0], q)
            np.testing.assert_allclose(back.sample(back.duration)[0], np.zeros(6))

    def test_partial_return_preserves_measured_start_and_home(self):
        T = play_plan.load_timeline_module()
        tl = Timeline(WAYPOINTS, speed=0.5)
        t_from = tl.duration * 0.4
        q = tl.sample(t_from)[0] + 0.3
        back = play_plan.build_return_timeline(T, tl, t_from, q, 30.0, 120.0, 0.5)
        np.testing.assert_allclose(back.sample(0)[0], q)
        np.testing.assert_allclose(back.sample(back.duration)[0], np.zeros(6))

    def test_disabled_motor_refuses_before_any_motion(self):
        from types import SimpleNamespace
        robot = FakeFollower([0] * 6)
        robot._read_feedback = lambda: {n: SimpleNamespace(status_code=0 if n == "elbow_flex" else 1) for n in [*play_plan.NAMES, "gripper"]}
        with self.assertRaisesRegex(RuntimeError, "enable 확인 실패"):
            play_plan.wait_enabled(robot, stable_s=0.0, timeout_s=0.0)
        self.assertEqual(robot.sent, 0)
        self.assertTrue(robot.held)

    def test_all_motors_must_remain_enabled_for_confirmation(self):
        from types import SimpleNamespace
        from unittest import mock
        robot = FakeFollower([0] * 6)
        snapshots = [True, False, True, True, True]
        def feedback():
            enabled = snapshots.pop(0)
            return {n: SimpleNamespace(status_code=1 if enabled else 0) for n in [*play_plan.NAMES, "gripper"]}
        robot._read_feedback = feedback
        with mock.patch.object(play_plan.time, "perf_counter", side_effect=[0.0, 0.0, 0.1, 0.2, 0.3, 0.41]), mock.patch.object(play_plan.time, "sleep"):
            play_plan.wait_enabled(robot, stable_s=0.2, timeout_s=2.0)
        self.assertEqual(snapshots, [])
        self.assertEqual(robot.sent, 0)

    def test_control_error_is_not_ignored_by_enable_check(self):
        from unittest import mock
        robot = FakeFollower([0] * 6)
        with mock.patch.object(robot, "_read_feedback", side_effect=RuntimeError("control stopped")):
            with self.assertRaisesRegex(RuntimeError, "control stopped"):
                play_plan.wait_enabled(robot)
        self.assertEqual(robot.sent, 0)


if __name__ == "__main__":
    unittest.main()
