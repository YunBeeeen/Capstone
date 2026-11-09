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
  sudo ip link set "$n" type can bitrate "${BITRATE[$n]}" || { echo "!! $n bitrate 설정 실패"; continue; }
  # restart-ms: bus-off 자동 복구. 드라이버가 지원할 때만 (라파 6.8-raspi gs_usb 는 미지원 → bus-off 면 이 스크립트 다시)
  sudo ip link set "$n" type can restart-ms 100 2>/dev/null || true
  # txqueuelen: 기본 10 은 버스트에 부족 (ENOBUFS)
  sudo ip link set "$n" txqueuelen 100
  sudo ip link set "$n" up
  # 실제 적용된 값 읽어서 표시
  info=$(ip -details link show "$n")
  br=$(grep -o -E 'bitrate [0-9]+' <<<"$info" | awk '{print $2}')
  rs=$(grep -o -E 'restart-ms [0-9]+' <<<"$info" | awk '{print $2}')
  st=$(grep -o -E 'can state [A-Z-]+' <<<"$info" | awk '{print $3}')
  printf "%-10s up  %7s bps  restart-ms %-3s %s\n" "$n" "$br" "${rs:-?}" "${st:-}"
  [[ "${rs:-0}" == "0" ]] && echo "           (bus-off 자동 복구 미지원 — 통신 끊기면 ./can_up.sh 다시)"
done
leftover=$(ip -br link | awk '$1 ~ /^can[0-9]+$/ {print $1}')
[[ -n "$leftover" ]] && echo "!! 이름 고정 안 된 어댑터: $leftover — 규칙 설치/시리얼 확인 (README.md)"
exit 0
