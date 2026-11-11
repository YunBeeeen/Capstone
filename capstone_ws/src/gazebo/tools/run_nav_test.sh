#!/bin/bash
# 대회 운영 흉내 (Gazebo):  1 일차 매핑 → 끄기 → 2 일차 localization + Nav2 → 선반 앞 목표 → 참값 비교
# 사용: src/gazebo/tools/run_nav_test.sh <출력디렉터리> [goal_x goal_y goal_yaw] [spawn_x spawn_y spawn_yaw]
#   기본 목표 = 휴게실(303) 책장 앞 (14.1, 7.4, π: 서쪽 책장을 바라봄), 2 일차 기본 출발 = 1 일차와 같은 (1.25, 1.25, 0)
#   <출력>/map.db 가 이미 있으면 1 일차를 건너뜀 (같은 지도로 목표·출발만 바꿔 여러 번)
OUT=$1; mkdir -p "$OUT"
GX=${2:-14.1}; GY=${3:-7.4}; GYAW=${4:-3.1416}
SX=${5:-1.25}; SY=${6:-1.25}; SYAW=${7:-0.0}
D=$(dirname "$0")
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
source /home/eomyunbeen/capstone/capstone_ws/install/setup.bash

if [ ! -s "$OUT/map.db" ]; then
  echo "== 1 일차: 매핑 (복도 루프 + 휴게실)"
  setsid ros2 launch capstone_gazebo world.launch.py gui:=false odom_tf:=false > "$OUT/day1_gz.log" 2>&1 &
  G=$!; sleep 15
  setsid ros2 launch capstone_navigation slam.launch.py use_sim_time:=true new_map:=true database_path:="$OUT/map.db" > "$OUT/day1_slam.log" 2>&1 &
  R=$!; sleep 8
  SLAM_ROUTE=loop_lounge timeout 1200 python3 "$D/slam_loop_run.py" "$OUT/day1" odometry/filtered | grep -E "^(odometry|SLAM|루프|점유)" | cut -c1-140
  kill -INT -- -$R -$G 2>/dev/null; sleep 6; kill -KILL -- -$R -$G 2>/dev/null; sleep 2
fi

echo "== 2 일차: localization + Nav2 (출발 world $SX, $SY, $SYAW)"
setsid ros2 launch capstone_gazebo world.launch.py gui:=false odom_tf:=false x:=$SX y:=$SY yaw:=$SYAW > "$OUT/day2_gz.log" 2>&1 &
G=$!; sleep 15
setsid ros2 launch capstone_navigation slam.launch.py use_sim_time:=true localization:=true database_path:="$OUT/map.db" > "$OUT/day2_slam.log" 2>&1 &
R=$!; sleep 10
setsid ros2 launch capstone_navigation nav.launch.py use_sim_time:=true > "$OUT/day2_nav.log" 2>&1 &
N=$!; sleep 15
timeout 400 python3 "$D/nav_goal_test.py" "$OUT" $GX $GY $GYAW 2>&1 | grep -E "^(출발|결과)|Error|Traceback"
kill -INT -- -$N -$R -$G 2>/dev/null; sleep 6; kill -KILL -- -$N -$R -$G 2>/dev/null
echo "nav 경고/오류 상위:"; grep -E "\[(WARN|ERROR)\]" "$OUT/day2_nav.log" | sed -E 's/\[[0-9.]+\]//' | sort | uniq -c | sort -rn | head -5 | cut -c1-200
