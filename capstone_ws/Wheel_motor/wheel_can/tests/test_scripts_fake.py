"""가짜 XDrive(가상 CAN 버스)로 스크립트 흐름 검증: RTR 요청, 상태 확인, vel.py 시작→유지→정지."""
import io
import sys
import time
import unittest
import uuid
from contextlib import redirect_stdout

from lib import cansimple as cs
from lib.bus import OdriveBus
from lib.runtime import request_state
from tests.fake_odrive import FakeOdrive


def virtual_runner(channel):
    def _run_with_bus(_ch, fn):
        bus = OdriveBus(channel, interface="virtual")
        try:
            return fn(bus)
        finally:
            bus.close()
    return _run_with_bus


class FakeBusTest(unittest.TestCase):
    def setUp(self):
        self.ch = f"t-{uuid.uuid4().hex[:6]}"
        self.fake = FakeOdrive(self.ch)
        self.fake.start()
        time.sleep(0.05)

    def tearDown(self):
        self.fake.stop()

    def test_request_and_state(self):
        with OdriveBus(self.ch, interface="virtual") as bus:
            self.assertAlmostEqual(bus.get_vbus(2), 24.75, places=2)
            self.assertIsNotNone(bus.get_encoder(1))
            with redirect_stdout(io.StringIO()):
                st = request_state(bus, 1, cs.AxisState.CLOSED_LOOP_CONTROL, settle=0.05)
            self.assertEqual(st, cs.AxisState.CLOSED_LOOP_CONTROL)
            self.assertEqual(self.fake.axes[1].state, cs.AxisState.CLOSED_LOOP_CONTROL)
            self.assertEqual(self.fake.axes[2].state, cs.AxisState.IDLE)   # 다른 노드는 영향 없음

    def test_vel_script_full_cycle(self):
        import vel
        vel.run_with_bus = virtual_runner(self.ch)
        argv = sys.argv
        sys.argv = ["vel.py", "--can-id", "1", "2", "--vel", "0.5", "-0.5", "--duration", "0.4"]
        try:
            with redirect_stdout(io.StringIO()) as out:
                vel.main()
        finally:
            sys.argv = argv
        a0, a1 = self.fake.axes[1], self.fake.axes[2]
        self.assertIn(0.5, a0.vel_history)
        self.assertIn(-0.5, a1.vel_history)
        self.assertEqual(a0.vel_cmd, 0.0)
        self.assertEqual(a0.state, cs.AxisState.IDLE)          # 끝나면 IDLE
        self.assertEqual(a1.state, cs.AxisState.IDLE)
        self.assertGreater(a0.pos, 0.05)                       # 실제로 '돌았음'
        self.assertLess(a1.pos, -0.05)
        self.assertIn("IDLE", out.getvalue())

    def test_vel_rejects_over_limit(self):
        import vel
        vel.run_with_bus = virtual_runner(self.ch)
        argv = sys.argv
        sys.argv = ["vel.py", "--can-id", "1", "--vel", "5"]
        try:
            with self.assertRaises(SystemExit):
                vel.main()
        finally:
            sys.argv = argv
        self.assertEqual(self.fake.axes[1].state, cs.AxisState.IDLE)
        self.assertEqual(self.fake.axes[1].vel_history, [])

    def test_vel_aborts_if_axis_drops_out(self):
        import vel
        vel.run_with_bus = virtual_runner(self.ch)

        def drop():                                            # 도는 중 보드가 에러로 IDLE 전환
            time.sleep(0.5)
            self.fake.axes[1].error = cs.AxisError.MOTOR_FAILED
            self.fake.axes[1].state = cs.AxisState.IDLE
        import threading
        threading.Thread(target=drop, daemon=True).start()
        argv = sys.argv
        sys.argv = ["vel.py", "--can-id", "1", "--vel", "0.5", "--duration", "3"]
        t0 = time.monotonic()
        try:
            with redirect_stdout(io.StringIO()) as out:
                vel.main()
        finally:
            sys.argv = argv
        self.assertLess(time.monotonic() - t0, 2.5)            # 3초 다 안 기다리고 중단
        self.assertIn("폐루프 이탈", out.getvalue())
        self.assertEqual(self.fake.axes[1].vel_cmd, 0.0)

    def test_hall_debug_counts(self):
        import hall_debug
        hall_debug.run_with_bus = virtual_runner(self.ch)
        self.fake.axes[1].state = cs.AxisState.CLOSED_LOOP_CONTROL
        self.fake.axes[1].vel_cmd = 1.0                        # 1 turn/s = 60 count/s

        class A:  # argparse 대용
            can_id, ch, rate = [1], self.ch, 50.0

        import threading, _thread
        threading.Timer(0.6, _thread.interrupt_main).start()
        with redirect_stdout(io.StringIO()) as out:
            hall_debug.run_can(A)
        self.assertIn("누적 Δcount", out.getvalue())


    def test_split_state_then_setvel(self):
        import setvel, state
        setvel.run_with_bus = virtual_runner(self.ch)
        state.run_with_bus = virtual_runner(self.ch)
        argv = sys.argv
        try:
            with redirect_stdout(io.StringIO()) as out:
                sys.argv = ["setvel.py", "--can-id", "1", "--vel", "0.5"]      # IDLE 이면 거부
                setvel.main()
                self.assertEqual(self.fake.axes[1].vel_history, [])
                sys.argv = ["state.py", "--can-id", "1", "--state", "closed_loop"]
                state.main()
                sys.argv = ["setvel.py", "--can-id", "1", "--vel", "0.5"]
                setvel.main()
        finally:
            sys.argv = argv
        time.sleep(0.05)                                                   # 가짜 보드 수신 처리 대기
        self.assertEqual(self.fake.axes[1].state, cs.AxisState.CLOSED_LOOP_CONTROL)
        self.assertEqual(self.fake.axes[1].vel_cmd, 0.5)                   # 스크립트 끝나도 유지
        self.assertIn("송신 안 함", out.getvalue())


if __name__ == "__main__":
    unittest.main()
