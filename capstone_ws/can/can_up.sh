#!/usr/bin/env bash
# 캡스톤 CAN 버스 올리기 (꽂혀 있는 것만). 어댑터 꽂을 때 / 재부팅마다.
#   ./can_up.sh          # 전부
#   ./can_up.sh down     # 전부 내리기
# 이름은 80-capstone-can.rules 로 고정 (설치 안 했으면 먼저 설치 — README.md).
set -u
declare -A BITRATE=(
  [can_drive]=500000   # XDrive ×2 (BL41105 ×4), ODrive CANSimple — 보드에 저장된 500k
  [can_steer]=1000000  # RobStride 02 ×4
  [can_arm_r]=1000000  # 오른팔 B601-DM (DM 모터)
  [can_arm_l]=1000000  # 왼팔 B601-DM
)
ORDER=(can_drive can_steer can_arm_r can_arm_l)
for n in "${ORDER[@]}"; do
  if ! ip link show "$n" >/dev/null 2>&1; then
    printf "%-10s 없음 (어댑터 안 꽂힘?)\n" "$n"
    continue
  fi
  sudo ip link set "$n" down
  if [[ "${1:-up}" == "down" ]]; then printf "%-10s down\n" "$n"; continue; fi
  # restart-ms: bus-off 자동 복구 / txqueuelen: 기본 10 은 버스트에 부족 (ENOBUFS)
  sudo ip link set "$n" type can bitrate "${BITRATE[$n]}" restart-ms 100
  sudo ip link set "$n" txqueuelen 100
  sudo ip link set "$n" up
  printf "%-10s up  %7d bps\n" "$n" "${BITRATE[$n]}"
done
leftover=$(ip -br link | awk '$1 ~ /^can[0-9]+$/ {print $1}')
[[ -n "$leftover" ]] && echo "!! 이름 고정 안 된 어댑터: $leftover — 규칙 설치/시리얼 확인 (README.md)"
exit 0
