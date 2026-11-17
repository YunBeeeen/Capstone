#!/bin/bash
# Nav2 로 저장한 장소까지 이동 — 도착할 때까지 기다렸다가 종료 코드로 결과를 알려 줌 (다른 sh 에서 부르기용).
# 라파·노트북에서 부르면 ssh 로 Jetson 의 nav_goal_ahead.py 를 돌리고, Jetson 에서 부르면 바로 돌림.
#
#   nav_to.sh desk_1                 # desk_1 로
#   nav_to.sh shelf_A --back 0.6     # shelf_A 에서 0.6 m 뒤 (선반 보는 자리)
#   nav_to.sh shelf_A --dry          # 목표 좌표만 (안 움직임)
#   JETSON=idle@192.168.0.11 nav_to.sh desk_1    # jetson.local 이 안 풀릴 때
#
# 종료 코드: 0 = 도착, 1 = 실패·취소 (Nav2), 2 = 준비 안 됨 (재위치·Nav2·장소 없음), 3 = Jetson 접속 안 됨,
#            130 = Ctrl+C (이때 Jetson 의 Nav2 목표도 취소함)
# 로봇이 실제로 움직이려면 라파 패드 (pad_pi.sh, --nav-auto) 와 노트북 estop.py 가 ▶ 진행이어야 함.
#
# 예 — 팔 정책 뒤에 이어 붙이기:
#   nav_to.sh desk_1             || exit 1
#   <팔 정책: 집어서 바구니에>   || exit 1      # 정책이 끝나면 프로그램이 끝나야 함 (성공 0)
#   nav_to.sh shelf_A            || exit 1
JETSON=${JETSON:-idle@jetson.local}
[ $# -ge 1 ] || { sed -n 2,13p "$0"; exit 2; }
ARGS="--name $(printf %q "$1")"; shift
for x in "$@"; do ARGS="$ARGS $(printf %q "$x")"; done

if [ -n "$NAV" ] && [ -f "$NAV/nav_goal_ahead.py" ]; then   # Jetson 에서 직접 (nav_env.sh 를 한 터미널)
    eval exec python3 "\$NAV/nav_goal_ahead.py" $ARGS
fi

SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8)
[ -f ~/.ssh/id_ed25519_jetson ] && SSH+=(-i ~/.ssh/id_ed25519_jetson -o IdentitiesOnly=yes)
on_jetson() { "${SSH[@]}" "$JETSON" "bash -lc $(printf %q "source ~/nav_env.sh >/dev/null 2>&1; $1")"; }

cancel() {
    echo "[nav_to] Nav2 목표 취소"
    on_jetson "pkill -INT -f '[n]av_goal_ahead.py'; ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}' >/dev/null"
    exit 130
}
trap cancel INT TERM

on_jetson "python3 \$NAV/nav_goal_ahead.py $ARGS"
rc=$?
[ $rc = 255 ] && { echo "[nav_to] Jetson ($JETSON) 접속 안 됨"; exit 3; }
exit $rc
