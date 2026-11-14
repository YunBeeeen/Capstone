#!/bin/sh
# 실기 가상 패드 — 늘 쓰던 인자를 기본으로 (RB = 수동, RT = Nav2).
#   ~/capstone/capstone_ws/src/navigation/tools/pad.sh                 # 0.15 m/s, 0.4 rad/s
#   ~/capstone/capstone_ws/src/navigation/tools/pad.sh --max-lin 0.3   # 뒤에 붙인 인자가 우선
#   PAD_HOST=192.168.0.7 pad.sh                                        # 라파 주소가 다를 때
exec python3 "$(dirname "$(readlink -f "$0")")/../../teleop/scripts/virtual_pad.py" \
    --udp "${PAD_HOST:-raspberrypi.local}" --nav-topic /nav/cmd_vel --max-lin 0.15 --max-ang 0.4 "$@"
