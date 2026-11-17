#!/bin/sh
# 게임패드를 라파에 꽂을 때 — 라파에서 실행 (화면 없이). 자기 자신의 udp_teleop_bridge (4번) 로 보내고,
# Nav2 명령은 Jetson nav_cmd_udp.py 가 UDP 5006 으로 보내 줌 (어디서 왔는지 첫 수신 때 표시).
# 실기 배포: Nav2 는 누르지 않아도 움직임 (--nav-auto). RT 한 번 = 정지 (걸어 둠), Start = 다시, RB = 수동 (우선),
# 패드가 빠지면 정지.
#   NAV_DEADMAN=1 pad_pi.sh                    # 예전처럼 RT 를 당기는 동안만 Nav2
#   ~/pad/scripts/pad_pi.sh                    # 상한 0.25 m/s, 0.5 rad/s, 마찰 보상 끔
#   ~/pad/scripts/pad_pi.sh --nav-min-lin 0.08 --nav-min-ang 0.25   # Nav2 작은 속도를 최소값까지 올림 (마찰)
#   ~/pad/scripts/pad_pi.sh --max-lin 0.3      # 뒤에 붙인 인자가 우선
#   NAV_HOST=192.168.1.11 pad_pi.sh            # 이 주소에서 온 Nav2 명령만 받기 (기본: 아무 데서나 — 대역이 바뀌어도 되게)
# 끄기: Ctrl+C (0 을 몇 번 보내고 끝)
export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-dummy}" SDL_AUDIODRIVER=dummy SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1
AUTO=--nav-auto; [ -n "$NAV_DEADMAN" ] && AUTO=
exec python3 "$(dirname "$(readlink -f "$0")")/virtual_pad.py" --udp 127.0.0.1 \
    --nav-udp 5006 ${NAV_HOST:+--nav-udp-from "$NAV_HOST"} $AUTO --max-lin 0.25 --max-ang 0.5 "$@"
