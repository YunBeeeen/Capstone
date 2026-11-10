# 리프트 (Pico, 동료 작성 코드)

라파 `~/lift/position/` 에서 가져옴 (2026-11-11). Pico 에 올라간 펌웨어 = `firmware/main.py` (11/10 12:10 approach 버전으로 추정 —
`DEPLOYMENT.json` 의 sha 는 그 전 버전 기록).

| 파일 | 내용 |
|---|---|
| `firmware/main.py` | Pico MicroPython 펌웨어 — 위치 제어, 소프트 리미트, JSON 시리얼 프로토콜 `yor-lift-position-v1` |
| `lift_position.py`, `lift_joystick.py` | 동료 Tk 조작 창 (라파 화면 / ssh -X) — 보정(mm/count)·PD 튜닝은 이걸로 |
| `LIFT_POSITION_CONTROL.md` | 동료 문서 (보정·원점·프로토콜) |

패드 연동은 ROS 패키지 `src/lift_bridge` (`lift_bridge_node.py`) — 이 펌웨어를 그대로 씀 (Pico 다시 올릴 필요 없음).
`src/lift_bridge/test/test_lift_logic.py` 가 이 `firmware/main.py` 의 PositionController 를 import 해서 시뮬로 검증하므로
펌웨어를 바꾸면 여기도 같이 갱신할 것.
