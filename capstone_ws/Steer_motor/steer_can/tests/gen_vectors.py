"""rs02_vectors.csv 생성 — lib(Python, idle_ws 실기 검증 구현)이 기준. C++ steer_can_bridge 테스트가 같은 파일을 읽음.

    cd ~/capstone/capstone_ws/Steer_motor/steer_can && python3 -m tests.gen_vectors
"""

import math
from pathlib import Path

from lib.common import pack_ext_id, float_to_u16
from lib.frames import (MIT_RS02, frame_type01_mit, frame_type03_enable, frame_type04_stop,
                        frame_type18_write_f32)

OUT = Path(__file__).with_name("rs02_vectors.csv")
HOST = 0xFD
HEADER = """\
# RS02 private 프로토콜 공용 테스트 벡터 — Python(tests/test_rs02.py)과 C++(steer_can_bridge test)이 같이 읽음.
# 생성: python3 -m tests.gen_vectors (lib 가 기준). 손으로 고치지 말 것.
# kind,can_id,a,b,c,d,e,arb_id_hex,data_hex
#   인코드 (Pi→모터, host_id=0xFD): enable / stop(a=clear_fault) / mit(a=p,b=v,c=kp,d=kd,e=t) / write_f32(a=index,b=value)
#   디코드 (모터→Pi): feedback(a=pos,b=vel,c=tor,d=temp,e=flags = mode<<6 | fault)
#   mit 의 p=0 / v=0 / t=0 은 u16 변환이 정확히 x.5 → 반올림 규칙(짝수 쪽)까지 맞는지 확인
"""


def row(kind, cid, vals, arb, data):
    vals = list(vals) + [""] * (5 - len(vals))
    return ",".join([kind, str(cid)] + [str(v) for v in vals] + [f"{arb:08X}", data.hex()])


def main():
    rows = []
    for cid in (1, 4, 127):
        rows.append(row("enable", cid, [], *frame_type03_enable(HOST, cid)))
    for cid, clr in ((1, 0), (2, 1)):
        rows.append(row("stop", cid, [clr], *frame_type04_stop(HOST, cid, bool(clr))))
    mits = [
        (1, 0.0, 0.0, 20.0, 2.0, 0.0),
        (2, math.pi / 2, 1.5, 20.0, 2.0, 0.0),
        (3, -math.radians(170), -2.0, 30.0, 2.2, 0.0),
        (4, 2.5, 0.0, 250.0, 2.5, 0.0),
        (1, -0.1234, 0.25, 5.0, 0.5, 1.5),
        (2, 20.0, 50.0, 600.0, 6.0, 20.0),     # 범위 밖 → 끝값으로 클램프
        (3, -20.0, -50.0, -1.0, -1.0, -20.0),
    ]
    for cid, p, v, kp, kd, t in mits:
        rows.append(row("mit", cid, [repr(p), repr(v), repr(kp), repr(kd), repr(t)],
                        *frame_type01_mit(cid, p, v, kp, kd, t)))
    for cid, idx, val in ((1, 0x7028, 10000.0), (2, 0x7017, 3.0)):
        rows.append(row("write_f32", cid, [f"0x{idx:04X}", repr(val)], *frame_type18_write_f32(HOST, cid, idx, val)))

    r = MIT_RS02
    fbs = [  # (motor_id, mode, fault, pos, vel, tor, temp)
        (1, 2, 0, 0.0, 0.0, 0.0, 31.5),
        (2, 0, 0, math.radians(-175), 0.0, 0.0, 28.0),
        (3, 2, 0, 1.2, -0.8, 2.75, 40.2),
        (4, 2, 0b010001, -3.0, 3.3, -6.0, 55.0),   # undervoltage + stall
    ]
    for mid, mode, fault, pos, vel, tor, temp in fbs:
        flags = (mode << 6) | fault
        arb = pack_ext_id(0x02, data2=(flags << 8) | mid, data1=HOST)
        data = (float_to_u16(pos, r.P_MIN, r.P_MAX).to_bytes(2, "big")
                + float_to_u16(vel, r.V_MIN, r.V_MAX).to_bytes(2, "big")
                + float_to_u16(tor, r.T_MIN, r.T_MAX).to_bytes(2, "big")
                + int(round(temp * 10)).to_bytes(2, "big"))
        # 기대값은 디코드 결과 (u16 양자화 후) — 파서와 독립적으로 계산
        u = [int.from_bytes(data[k:k + 2], "big") for k in (0, 2, 4, 6)]
        dec = lambda q, lo, hi: lo + (q / 65535.0) * (hi - lo)
        vals = [repr(dec(u[0], r.P_MIN, r.P_MAX)), repr(dec(u[1], r.V_MIN, r.V_MAX)),
                repr(dec(u[2], r.T_MIN, r.T_MAX)), repr(u[3] / 10.0), flags]
        rows.append(row("feedback", mid, vals, arb, data))

    OUT.write_text(HEADER + "\n".join(rows) + "\n")
    print(f"{OUT}: {len(rows)} rows")


if __name__ == "__main__":
    main()
