#!/bin/bash
# 노트북 → 라파·Jetson 에 실기 코드 넣기 (해시 확인). 기본은 확인만, --apply 면 복사.
#   ./deploy_robot.sh            # 무엇이 바뀌었나만
#   ./deploy_robot.sh --apply    # 넣기
# 규칙: 거기 파일이 지금 것과 같으면 건너뜀, 이 저장소의 예전 커밋 중 하나와 같으면 (= 거기서 안 고침) 덮어씀,
#       둘 다 아니면 (= 거기서 누가 고침) 넣지 않고 알려 줌 → 가져와서 합칠 것.
cd "$(dirname "$(readlink -f "$0")")/capstone_ws/src" || exit 1
APPLY=0; [ "$1" = "--apply" ] && APPLY=1
PI=${PI:-raspberrypi}; JETSON=${JETSON:-jetson}

put() {   # put <호스트> <저장소 파일> <거기 경로 (~ 기준)>
  local h=$1 src=$2 dst=$3 cur new c
  cur=$(ssh -o ConnectTimeout=5 "$h" "sha256sum $dst 2>/dev/null" | cut -c1-64) || { echo "✗ $h 접속 안 됨"; return; }
  new=$(sha256sum "$src" | cut -c1-64)
  if [ "$cur" = "$new" ]; then echo "  = $h:$dst"; return; fi
  if [ -n "$cur" ]; then
    for c in $(git log --format=%h -- "$src"); do
      [ "$cur" = "$(git show "$c:./$src" 2>/dev/null | sha256sum | cut -c1-64)" ] && break
      c=""
    done
    [ -z "$c" ] && { echo "  ⚠ $h:$dst 거기서 고친 것 → 안 넣음 (가져와서 합치기)"; return; }
  fi
  if [ $APPLY = 1 ]; then
    scp -q "$src" "$h:$dst" && echo "  ✓ $h:$dst 넣음 (${c:-새 파일})"
  else
    echo "  → $h:$dst 바뀜 (${c:+예전 $c}${c:-거기 없음}) — --apply 로 넣기"
  fi
}

echo "라파 ($PI)"
put $PI arm_kinematics/tools/mission_run.py      arm/tools/mission_run.py
put $PI arm_kinematics/tools/ik_sim_run.py       arm/tools/ik_sim_run.py
put $PI arm_kinematics/tools/play_plan.py        arm/tools/play_plan.py
put $PI arm_kinematics/arm_kinematics/ik.py      arm/arm_kinematics/ik.py
put $PI arm_kinematics/arm_kinematics/collision.py arm/arm_kinematics/collision.py
put $PI teleop/scripts/virtual_pad.py            pad/scripts/virtual_pad.py
put $PI teleop/scripts/udp_packet.py             pad/scripts/udp_packet.py
put $PI teleop/scripts/pad_pi.sh                 pad/scripts/pad_pi.sh
echo "Jetson ($JETSON)"
put $JETSON navigation/tools/nav_goal_ahead.py   capstone_ws/src/navigation/tools/nav_goal_ahead.py
put $JETSON arm_kinematics/tools/ros_find.py     arm_vision/tools/ros_find.py
put $JETSON arm_kinematics/tools/svo_find.py     arm_vision/tools/svo_find.py
