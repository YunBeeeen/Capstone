import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_kinematics.checks import in_basket  # noqa: E402

B = (0.50, -0.10, 0.88)


class InBasketTests(unittest.TestCase):
    def test_near_center_is_inside_any_orientation(self):
        v = in_basket((0.55, -0.08, 0.92), B)
        self.assertTrue(v.inside)
        self.assertEqual(v.zone, "inside")

    def test_beside_basket_on_desk_is_outside(self):
        # 바구니 옆 책상 위 (중심 거리 15 cm) — 대각선 전체 기준이면 '안' 으로 잘못 나옴
        v = in_basket((0.65, -0.10, 0.91), B)
        self.assertFalse(v.inside)
        self.assertEqual(v.zone, "outside")

    def test_unsure_zone_uses_yaw_rectangle(self):
        p = (0.50 + 0.09, -0.10, 0.92)                         # 중심에서 9 cm
        self.assertTrue(in_basket(p, B, basket_yaw=0.0).inside)            # 긴 변 방향 → 안
        self.assertFalse(in_basket(p, B, basket_yaw=math.pi / 2).inside)   # 짧은 변 방향 → 밖
        self.assertEqual(in_basket(p, B, basket_yaw=0.0).zone, "unsure")

    def test_unsure_zone_uses_image_boxes(self):
        p = (0.50 + 0.08, -0.10, 0.92)
        basket_box, obj_in, obj_out = (100, 100, 300, 220), (180, 140, 220, 180), (310, 140, 350, 180)
        self.assertTrue(in_basket(p, B, obj_box=obj_in, basket_box=basket_box).inside)
        self.assertFalse(in_basket(p, B, obj_box=obj_out, basket_box=basket_box).inside)
        self.assertFalse(in_basket(p, B, obj_box=obj_in, basket_box=basket_box, seen_elsewhere=True).inside)

    def test_unsure_without_info_is_not_trusted(self):
        v = in_basket((0.58, -0.10, 0.92), B)
        self.assertFalse(v.inside)
        self.assertEqual(v.zone, "unsure")


if __name__ == "__main__":
    unittest.main()
