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

# can_drive: 구동 보드 heartbeat 확인.
# 구동용 CANable 은 up 직후 수신을 못 하다가 자기가 한 번 송신해야 들리는 경우가 있음 (2026-11-15 재현:
# 보드는 계속 heartbeat 보내는데 어댑터만 0 → 엔코더 RTR 한 번 보내면 bus-off 한 번 거치고 정상).
# 그래서 안 들리면 RTR(읽기 요청, 모터 안 움직임)로 깨우고 다시 듣는다.
DRIVE_IDS="1 2 3 4"   # wheel_can_bridge.yaml can_ids 와 같게
if [[ "${1:-up}" != "down" ]] && ip link show can_drive >/dev/null 2>&1; then
  if python3 -c "import can" 2>/dev/null; then
    python3 - $DRIVE_IDS <<'PY'
import sys, time, can
ids = [int(x) for x in sys.argv[1:]]
bus = can.Bus(interface="socketcan", channel="can_drive")

def listen(sec):
    n = {i: 0 for i in ids}
    end = time.time() + sec
    while time.time() < end:
        m = bus.recv(0.05)
        if m and not m.is_error_frame and (m.arbitration_id & 0x1F) == 0x01 and (m.arbitration_id >> 5) in n:
            n[m.arbitration_id >> 5] += 1
    return n

n = listen(0.5)
wakes = 0
while not all(n.values()) and wakes < 3:
    wakes += 1
    for i in ids:  # 0x09 = Get_Encoder_Estimates RTR
        try:
            bus.send(can.Message(arbitration_id=(i << 5) | 0x09, is_remote_frame=True,
                                 is_extended_id=False, dlc=8), timeout=0.05)
        except can.CanError:
            pass
    time.sleep(0.3)
    n = listen(1.0)
bus.shutdown()
line = "  ".join(f"id{i} {'OK' if n[i] else '--'}" for i in ids)
print(f"can_drive  heartbeat  {line}" + (f"  (깨움 {wakes}회)" if wakes else ""))
if not all(n.values()):
    if not any(n.values()):
        print("           전부 안 들림 → 24 V · 보드 전원 / CAN 선 확인, 그래도면 ./can_up.sh 한 번 더")
    else:
        print("           -- 인 바퀴 보드 전원 / CAN 선 확인 (보드 #1 = id1·2, 보드 #2 = id3·4)")
PY
  else
    echo "can_drive  heartbeat 확인 건너뜀 (python3-can 없음)"
  fi
fi

leftover=$(ip -br link | awk '$1 ~ /^can[0-9]+$/ {print $1}')
[[ -n "$leftover" ]] && echo "!! 이름 고정 안 된 어댑터: $leftover — 규칙 설치/시리얼 확인 (README.md)"
exit 0
