"""임무 단계 확인 판정 (인식 결과 → 다음 단계로 넘어가도 되나). numpy 만 씀.

in_basket: 물체가 바구니 안인가 — 바구니 중심 ↔ 물체 중심의 수평 거리로 세 구간.
  ≤ 짧은 변/2           : 안 (바구니가 어느 방향이든 안쪽)
  > 긴 변/2             : 밖
  그 사이 (애매)        : 바구니 방향(yaw)을 알면 바구니 기준 사각형으로,
                           모르면 영상에서 물체 상자 중심이 바구니 상자 안이고 다른 데서 물체가 안 잡히면 안.
높이는 보지 않는다 — 바구니 바닥과 책상 높이 차이가 1 cm 안팎이라 안/옆을 못 가른다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

BASKET_INNER = (0.21, 0.13)      # 실측 21 x 13 cm (바깥). 판정은 이 크기로


@dataclass
class Verdict:
    inside: bool
    zone: str                    # "inside" | "outside" | "unsure"
    reason: str
    dist_m: float


def in_basket(obj_xyz, basket_xyz, basket_size=BASKET_INNER, basket_yaw: float | None = None,
              obj_box=None, basket_box=None, seen_elsewhere: bool = False) -> Verdict:
    """obj_xyz, basket_xyz: 같은 좌표계 (로봇 바닥 기준 등) 의 중심 [m]. basket_size: (긴 변, 짧은 변) [m].
    basket_yaw: 바구니 긴 변 방향 [rad] (모르면 None). obj_box, basket_box: 영상 상자 (x0, y0, x1, y1) 픽셀.
    seen_elsewhere: 바구니 밖 다른 곳에서도 같은 물체가 잡혔나 (두 번 잡히면 밖일 가능성)."""
    long_half, short_half = max(basket_size) / 2, min(basket_size) / 2
    d = np.asarray(obj_xyz, float)[:2] - np.asarray(basket_xyz, float)[:2]
    dist = float(np.hypot(*d))
    if dist <= short_half:
        return Verdict(True, "inside", f"수평 {100 * dist:.1f} cm ≤ 짧은 변 절반 {100 * short_half:.1f} cm", dist)
    if dist > long_half:
        return Verdict(False, "outside", f"수평 {100 * dist:.1f} cm > 긴 변 절반 {100 * long_half:.1f} cm", dist)
    if basket_yaw is not None:
        c, s = math.cos(basket_yaw), math.sin(basket_yaw)
        along, across = c * d[0] + s * d[1], -s * d[0] + c * d[1]
        ok = abs(along) <= long_half and abs(across) <= short_half
        return Verdict(ok, "unsure", f"애매 구간, 바구니 기준 (긴 {100 * along:+.1f}, 짧은 {100 * across:+.1f}) cm → "
                                     + ("사각형 안" if ok else "사각형 밖"), dist)
    if obj_box is not None and basket_box is not None:
        cx, cy = (obj_box[0] + obj_box[2]) / 2, (obj_box[1] + obj_box[3]) / 2
        in_img = basket_box[0] <= cx <= basket_box[2] and basket_box[1] <= cy <= basket_box[3]
        ok = in_img and not seen_elsewhere
        why = "영상에서 바구니 상자 안" if in_img else "영상에서 바구니 상자 밖"
        if in_img and seen_elsewhere:
            why += " 이지만 다른 곳에서도 잡힘"
        return Verdict(ok, "unsure", f"애매 구간 ({100 * dist:.1f} cm), {why}", dist)
    return Verdict(False, "unsure", f"애매 구간 ({100 * dist:.1f} cm), 방향·영상 정보 없음 → 밖으로 봄 (다시 보기)", dist)
