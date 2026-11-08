"""2026-11-03 실기에서 candump 로 잡은 프레임으로 인코드/디코드 검증 + C++ 와 공용 벡터(cansimple_vectors.csv)."""
import unittest
from pathlib import Path

from lib import cansimple as cs


class TestIds(unittest.TestCase):
    def test_arb_id(self):
        self.assertEqual(cs.arb_id(0, cs.Cmd.HEARTBEAT), 0x001)
        self.assertEqual(cs.arb_id(1, cs.Cmd.HEARTBEAT), 0x021)
        self.assertEqual(cs.arb_id(16, cs.Cmd.HEARTBEAT), 0x201)      # XDrive 공장 axis0
        self.assertEqual(cs.arb_id(1, cs.Cmd.GET_ENCODER_ESTIMATES), 0x029)
        self.assertEqual(cs.arb_id(1, cs.Cmd.GET_VBUS), 0x037)
        self.assertEqual(cs.split_id(0x209), (16, 0x09))


class TestRealFrames(unittest.TestCase):
    def test_heartbeat_idle(self):
        self.assertEqual(cs.dec_heartbeat(bytes.fromhex("0000000001000000")), (0, cs.AxisState.IDLE))

    def test_encoder_estimates(self):
        pos, vel = cs.dec_encoder_estimates(bytes.fromhex("76DA903E00000000"))
        self.assertAlmostEqual(pos, 0.28292, places=4)
        self.assertEqual(vel, 0.0)

    def test_vbus(self):
        self.assertAlmostEqual(cs.dec_vbus(bytes.fromhex("F304C64100000000")), 24.7524, places=3)

    def test_encode_matches_cansend(self):
        self.assertEqual(cs.enc_axis_state(8).hex().upper(), "08000000")
        self.assertEqual(cs.enc_input_vel(0.5).hex().upper(), "0000003F00000000")
        self.assertEqual(cs.enc_input_vel(-0.5).hex().upper(), "000000BF00000000")

    def test_decode_flags(self):
        self.assertEqual(cs.decode_flags(0, cs.AxisError), "OK")
        self.assertIn("ENCODER_FAILED", cs.decode_flags(0x100, cs.AxisError))


VECTORS = Path(__file__).with_name("cansimple_vectors.csv")
ENC = {
    "axis_state": (cs.Cmd.SET_AXIS_STATE, lambda a, b: cs.enc_axis_state(int(a))),
    "input_vel": (cs.Cmd.SET_INPUT_VEL, lambda a, b: cs.enc_input_vel(float(a), float(b))),
    "controller_mode": (cs.Cmd.SET_CONTROLLER_MODE, lambda a, b: cs.enc_controller_mode(int(a), int(b))),
    "clear_errors": (cs.Cmd.CLEAR_ERRORS, lambda a, b: b""),
    "rtr_encoder": (cs.Cmd.GET_ENCODER_ESTIMATES, lambda a, b: b""),
    "rtr_iq": (cs.Cmd.GET_IQ, lambda a, b: b""),
}
DEC = {
    "heartbeat": (cs.Cmd.HEARTBEAT, cs.dec_heartbeat),
    "encoder_estimates": (cs.Cmd.GET_ENCODER_ESTIMATES, cs.dec_encoder_estimates),
    "iq": (cs.Cmd.GET_IQ, cs.dec_iq),
    "vbus": (cs.Cmd.GET_VBUS, lambda d: (cs.dec_vbus(d),)),
}


class TestSharedVectors(unittest.TestCase):
    def test_vectors(self):
        rows = [l.split(",") for l in VECTORS.read_text().splitlines() if l and not l.startswith("#")]
        self.assertGreater(len(rows), 10)
        for kind, cid, a, b, arb, data in rows:
            with self.subTest(kind=kind, can_id=cid):
                raw = bytes.fromhex(data)
                if kind in ENC:
                    cmd, fn = ENC[kind]
                    self.assertEqual(cs.arb_id(int(cid), cmd), int(arb, 16))
                    self.assertEqual(fn(a, b), raw)
                else:
                    cmd, fn = DEC[kind]
                    self.assertEqual(cs.arb_id(int(cid), cmd), int(arb, 16))
                    want = [float(x) for x in (a, b) if x != ""]
                    for got, w in zip(fn(raw), want):
                        self.assertAlmostEqual(float(got), w, places=4)


if __name__ == "__main__":
    unittest.main()
