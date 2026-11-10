#!/usr/bin/env python3
"""lift_logic 을 동료 Pico 펌웨어의 PositionController (capstone_ws/Lift/firmware/main.py) 와 붙여서 검증.

펌웨어 코드를 그대로 import (machine / utime / micropython 은 가짜 모듈) + 간단한 리프트 모델:
  - 실제 높이 x [mm], 0 = 물리적 최하단 (아래로 더 못 감). 엔코더 카운트 = x / mm_per_count (부팅 때 0 부터)
  - 속도 = 38 mm/s × (duty − 0.15)/0.85, 1차 지연 50 ms (duty 0.15 이하는 안 움직임)
루프: 10 ms 마다 모델 + 펌웨어 update, 100 ms 마다 상태 JSON → 로직, 50 ms 마다 로직 → 명령 JSON → 펌웨어.
"""
import importlib.util
import json
import math
import random
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from lift_logic import LiftCmd, LiftConfig, LiftLogic  # noqa: E402

FW = Path(__file__).resolve().parents[3] / "Lift" / "firmware" / "main.py"
K = 0.21574  # mm/count (Pico 보정값과 같게)

CLOCK = {"ms": 0}


def _load_firmware():
    utime = types.ModuleType("utime")
    utime.ticks_ms = lambda: CLOCK["ms"]
    utime.ticks_diff = lambda a, b: a - b
    utime.sleep_ms = lambda ms: None
    sys.modules["utime"] = utime
    sys.modules.setdefault("machine", types.ModuleType("machine"))
    sys.modules.setdefault("micropython", types.ModuleType("micropython"))
    spec = importlib.util.spec_from_file_location("lift_fw", FW)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakeStore:
    def load(self):
        return {"mm_per_count": K, "encoder_sign": 1, "control": {"kp": 0.04, "kd": 0.001, "max_duty": 1.0}}


class Sim:
    """Pico (펌웨어) + 리프트 모델 + 상위 로직."""

    def __init__(self, fw, x0=0.0, store=None):
        self.fw = fw
        self.x = x0          # 실제 높이 [mm]
        self.v = 0.0
        self.boot_x = x0     # Pico 부팅 시 높이 (카운트 0)
        self.cmd = (0, 0.0)
        self.pulses = 0
        self.last_count = 0
        self.t_ms = CLOCK["ms"]
        self.faults = []
        self.sent = []
        self.ctrl = fw.PositionController(self.drive, lambda: CLOCK["ms"], self.encoder,
                                          calibration_store=FakeStore(), boot_reason="power_on")
        self.logic = LiftLogic(LiftConfig(mm_per_count=K), store=store)

    def drive(self, direction, duty):
        self.cmd = (direction, duty)

    def encoder(self):
        c = math.floor((self.x - self.boot_x) / K + 0.5)
        if c != self.last_count:
            self.pulses += abs(c - self.last_count)
            self.last_count = c
        return c & (self.fw.COUNTER_MODULUS - 1), self.pulses & (self.fw.COUNTER_MODULUS - 1)

    def reboot_pico(self):
        """Pico 전원 껐다 켬: 카운트 0 부터, 원점 없음. 로직도 새로 (파일 store 만 남음)."""
        self.boot_x = self.x
        self.last_count = 0
        self.ctrl = self.fw.PositionController(self.drive, lambda: CLOCK["ms"], self.encoder,
                                               calibration_store=FakeStore(), boot_reason="power_on")
        self.cmd = (0, 0.0)
        self.logic = LiftLogic(LiftConfig(mm_per_count=K), store=dict(self.logic.store))

    def now(self):
        return CLOCK["ms"] / 1000.0

    def run(self, seconds, cmd=None, until=None):
        """cmd: LiftCmd 또는 t→LiftCmd 함수 (None 이면 명령 안 보냄). until: logic→bool 이면 조기 종료."""
        end = CLOCK["ms"] + int(seconds * 1000)
        while CLOCK["ms"] < end:
            CLOCK["ms"] += 10
            t = self.now()
            # 모델
            d, duty = self.cmd
            target_v = d * 38.0 * max(0.0, duty - 0.15) / 0.85
            self.v += (target_v - self.v) * 0.01 / 0.05
            self.x += self.v * 0.01
            if self.x < 0.0:
                self.x, self.v = 0.0, 0.0
            self.ctrl.update()
            if CLOCK["ms"] % 100 == 0:
                st = json.loads(json.dumps(self.ctrl.status()))
                if st["fault"] and (not self.faults or self.faults[-1] != st["fault"]):
                    self.faults.append(st["fault"])
                self.logic.on_status(st, t)
            if CLOCK["ms"] % 50 == 0:
                c = cmd(t) if callable(cmd) else cmd
                if c is not None:
                    self.logic.on_command(c, t)
                for m in self.logic.step(t):
                    self.sent.append(m["cmd"])
                    self.ctrl.command(json.loads(json.dumps(m)))
            if until is not None and until(self.logic):
                return True
        return False

    def park(self, timeout=60.0):
        self.run(0.2, LiftCmd(park=True))
        return self.run(timeout, LiftCmd(), until=lambda lg: not lg.busy())


@unittest.skipUnless(FW.exists(), f"펌웨어 없음: {FW}")
class TestLiftLogic(unittest.TestCase):
    def setUp(self):
        self.fw = _load_firmware()
        CLOCK["ms"] = 0

    def boot_zeroed(self):
        s = Sim(self.fw, x0=0.0)
        s.run(0.5)
        s.run(1.3, LiftCmd(zero=True))
        self.assertTrue(s.logic.homed(), s.logic.message)
        return s

    def test_unhomed_jog_moves_and_counts(self):
        s = Sim(self.fw, x0=50.0)
        s.run(0.5)
        self.assertFalse(s.logic.homed())
        s.run(2.0, LiftCmd(jog=1))
        s.run(0.5, LiftCmd())
        self.assertGreater(s.x, 60.0)
        self.assertAlmostEqual(s.logic.rel_mm(), s.x - 50.0, delta=0.5)
        self.assertEqual(s.ctrl.mode, "idle")
        self.assertEqual(s.faults, [])

    def test_jog_reversal_without_fault(self):
        s = Sim(self.fw, x0=100.0)
        s.run(0.5)
        s.run(1.0, LiftCmd(jog=1))
        top = s.x
        s.run(1.5, LiftCmd(jog=-1))  # 떼지 않고 바로 반대
        s.run(0.5, LiftCmd())
        self.assertEqual(s.faults, [], "반전 시 jog_reverse_requires_stop 나면 안 됨")
        self.assertLess(s.x, top - 3.0)

    def test_stale_command_stops(self):
        s = Sim(self.fw, x0=100.0)
        s.run(0.5)
        s.run(1.0, LiftCmd(jog=1))
        s.run(0.5, None)  # 패드 끊김: 명령 안 옴
        x = s.x
        s.run(1.0, None)
        self.assertAlmostEqual(s.x, x, delta=0.3)
        self.assertEqual(s.ctrl.mode, "idle")

    def test_zero_needs_hold(self):
        s = Sim(self.fw, x0=0.0)
        s.run(0.5)
        s.run(0.6, LiftCmd(zero=True))
        s.run(0.3, LiftCmd())
        self.assertFalse(s.logic.homed())
        s.run(1.3, LiftCmd(zero=True))
        self.assertTrue(s.logic.homed())
        self.assertAlmostEqual(s.logic.height(), 0.0, delta=0.01)

    def test_homed_jog_respects_soft_limit(self):
        s = self.boot_zeroed()
        s.run(2.0, LiftCmd(jog=-1))
        self.assertLess(s.x, 0.01)  # 이미 바닥 → 안 내려감 (soft_min)
        self.assertEqual(s.faults, [])

    def test_park_from_height(self):
        s = self.boot_zeroed()
        s.run(6.0, LiftCmd(jog=1))
        s.run(0.5, LiftCmd())
        self.assertGreater(s.logic.height(), 30.0)
        self.assertTrue(s.park(), f"주차가 안 끝남: task={s.logic.task} msg={s.logic.message}")
        self.assertEqual(s.logic.park_result, "ok", s.logic.message)
        self.assertLess(s.x, 0.6)
        self.assertAlmostEqual(s.logic.height(), s.x, delta=0.3)
        self.assertTrue(s.logic.store["parked"])
        self.assertTrue(s.logic.is_parked())
        self.assertEqual(s.faults, [])
        # 주차 뒤 다시 조그하면 주차 무효
        s.run(1.5, LiftCmd(jog=1))
        s.run(0.5, LiftCmd())
        self.assertFalse(s.logic.store["parked"])

    def test_park_requires_homed(self):
        s = Sim(self.fw, x0=80.0)
        s.run(0.5)
        s.park(timeout=2.0)
        self.assertEqual(s.logic.park_result, "failed")
        self.assertAlmostEqual(s.x, 80.0, delta=0.01)

    def test_stop_aborts_park(self):
        s = self.boot_zeroed()
        s.run(6.0, LiftCmd(jog=1))
        s.run(0.5, LiftCmd())
        s.run(0.2, LiftCmd(park=True))
        s.run(0.5, LiftCmd())
        self.assertTrue(s.logic.busy())
        s.run(0.3, LiftCmd(stop=True))
        x = s.x
        s.run(1.0, LiftCmd())
        self.assertFalse(s.logic.busy())
        self.assertEqual(s.logic.park_result, "aborted")
        self.assertAlmostEqual(s.x, x, delta=1.0)

    def test_reboot_cycles_no_drift(self):
        """주차 → Pico 재부팅 → 자동 원점 → 조그 → 주차 ... 를 반복해도 오차가 안 쌓임."""
        rnd = random.Random(1)
        s = self.boot_zeroed()
        for cycle in range(6):
            s.run(rnd.uniform(1.0, 6.0), LiftCmd(jog=1))
            s.run(0.5, LiftCmd())
            self.assertTrue(s.park(), f"{cycle}: {s.logic.message}")
            self.assertEqual(s.logic.park_result, "ok", f"{cycle}: {s.logic.message}")
            s.reboot_pico()
            s.run(2.0)  # 자동 원점 등록
            self.assertTrue(s.logic.homed(), f"{cycle}: 자동 원점 안 됨 — {s.logic.message}")
            self.assertLess(s.x, 0.6, f"{cycle}: 바닥에서 {s.x:.2f} mm")
            self.assertAlmostEqual(s.logic.height(), s.x, delta=0.3, msg=f"{cycle}")
        self.assertEqual(s.faults, [])

    def test_boot_without_parked_file_stays_unhomed(self):
        s = Sim(self.fw, x0=0.0, store={"parked": False})
        s.run(1.5)
        self.assertFalse(s.logic.homed())
        self.assertIn("원점 미등록", s.logic.message)

    def test_weak_park_duty_fails_instead_of_false_zero(self):
        s = self.boot_zeroed()
        s.run(4.0, LiftCmd(jog=1))
        s.run(0.5, LiftCmd())
        s.logic.cfg.park_duty = 0.12  # 모델에서 0.15 이하는 안 움직임
        s.park()
        self.assertEqual(s.logic.park_result, "failed")
        self.assertIn("park_duty", s.logic.message)
        self.assertFalse(s.logic.store["parked"])
        self.assertGreater(s.x, 8.0)

    def test_scale_mismatch_warns(self):
        s = Sim(self.fw, x0=0.0)
        s.logic = LiftLogic(LiftConfig(mm_per_count=K * 1.1))
        s.run(0.5)
        s.run(1.3, LiftCmd(zero=True))
        s.run(8.0, LiftCmd(jog=1))
        self.assertIn("mm/count", s.logic.message)

    def test_shutdown_request_park_when_already_parked(self):
        s = self.boot_zeroed()
        s.run(3.0, LiftCmd(jog=1))
        s.run(0.5, LiftCmd())
        self.assertTrue(s.park())
        n = len(s.sent)
        s.logic.request_park(s.now())
        self.assertFalse(s.logic.busy())
        self.assertEqual(len(s.sent), n)


if __name__ == "__main__":
    unittest.main()
