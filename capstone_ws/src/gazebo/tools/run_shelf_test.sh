#!/bin/bash
# 선반 기억 2 단계 시뮬 시험:  1 일차 매핑 + 장소 저장 → 끄기 → 2 일차 재위치 + Nav2 → 선반을 옮겨 가며 place_go
# 사용: src/gazebo/tools/run_shelf_test.sh <출력디렉터리> [run|run2] [record]
#   <출력>/map.db 가 있으면 1 일차를 건너뜀.  run = 선반 옮기기·치우기 (기본),  run2 = 물건 바뀜·사람 가림·끼어듦,  run3 = 정면이 막혔을 때 옆면으로,  run4 = 다른 곳으로 옮겨진 선반 찾기 (SHELF_MAP=wide 로 매핑)
#   record = 구경 카메라 화면을 <출력>/<모드>.mp4 로 녹화
OUT=$(realpath -m "$1"); mkdir -p "$OUT"
D=$(dirname "$(realpath "$0")")
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TEST:-77}
source "$D/../../../install/setup.bash"
MODE=${2:-run}; REC=$3
# SIM_ODOM=perfect = Gazebo 참값 odom 을 그대로 씀 (EKF 안 거침). 긴 루프 매핑에서 가짜 odom·자이로 오차로 지도가 틀어질 때 —
#   위치 추정이 아니라 찾기 논리를 보는 run4 는 이걸로 시험함
# 로봇: 기본 = 팔 달린 실제 모델 (urdf/mobile_robot_sim.urdf.xacro). SIM_ROBOT=test_rig 이면 예전 시험용 상자
if [ "$SIM_ODOM" = perfect ]; then WORLD_ODOM="odom_tf:=true odom_noise:=false vio:=false"; SLAM_ODOM="odom_source:=external"; NAV_ODOM="odom_topic:=/odom"
else WORLD_ODOM="odom_tf:=false"; SLAM_ODOM=""; NAV_ODOM=""; fi
ROBOT=${SIM_ROBOT:-$(ros2 pkg prefix capstone_gazebo)/share/capstone_gazebo/urdf/mobile_robot_sim.urdf.xacro}
M="$D/../models"
SHELF="$M/ladder_shelf/model.sdf"
spawn_extra() {
  ros2 run gazebo_ros spawn_entity.py -entity ladder_shelf_alt -file "$M/ladder_shelf_alt/model.sdf" -x 52 -y 50 > /dev/null 2>&1
  ros2 run gazebo_ros spawn_entity.py -entity ladder_shelf_empty -file "$M/ladder_shelf_empty/model.sdf" -x 54 -y 50 > /dev/null 2>&1
  ros2 run gazebo_ros spawn_entity.py -entity person_box -file "$M/person_box/model.sdf" -x 60 -y 60 > /dev/null 2>&1
  ros2 run gazebo_ros spawn_entity.py -entity person_box2 -file "$M/person_box/model.sdf" -x 62 -y 60 > /dev/null 2>&1
  ros2 run gazebo_ros spawn_entity.py -entity cover_panel -file "$M/cover_panel/model.sdf" -x 66 -y 60 > /dev/null 2>&1
  ros2 run gazebo_ros spawn_entity.py -entity person_box3 -file "$M/person_box/model.sdf" -x 64 -y 60 > /dev/null 2>&1
}
spawn_shelf() { ros2 run gazebo_ros spawn_entity.py -entity ladder_shelf -file "$SHELF" -x 4.3 -y 2.24 -Y 0 > "$OUT/spawn_shelf.log" 2>&1; }
stop() { kill -INT -- $(printf -- '-%s ' "$@") 2>/dev/null; sleep 8; kill -KILL -- $(printf -- '-%s ' "$@") 2>/dev/null; sleep 2; }

if [ ! -s "$OUT/map.db" ]; then
  echo "== 1 일차: 매핑 + 장소 저장"
  setsid ros2 launch capstone_gazebo world.launch.py gui:=false $WORLD_ODOM robot:=$ROBOT > "$OUT/day1_gz.log" 2>&1 &
  G=$!; sleep 15; spawn_shelf
  setsid ros2 launch capstone_navigation slam.launch.py use_sim_time:=true $SLAM_ODOM new_map:=true database_path:="$OUT/map.db" > "$OUT/day1_slam.log" 2>&1 &
  R=$!; sleep 10
  timeout 2400 python3 "$D/shelf_test.py" map "$OUT" 2>&1 | tee "$OUT/day1.txt"
  stop $R $G
fi

echo "== 2 일차: 재위치 + Nav2 + place_go"
[ -f "$OUT/map.places.day1.yaml" ] || cp "$OUT/map.places.yaml" "$OUT/map.places.day1.yaml"   # 1 일차 기억 원본 (한 번만)
[ -d "$OUT/map.places.day1" ] || cp -r "$OUT/map.places" "$OUT/map.places.day1"                       # 기준 사진·depth 원본 (--leave 가 덮어씀)
rm -rf "$OUT/map.places"; cp -r "$OUT/map.places.day1" "$OUT/map.places"
cp "$OUT/map.places.day1.yaml" "$OUT/map.places.yaml"                                                # 매번 원본에서 시작
setsid ros2 launch capstone_gazebo world.launch.py gui:=false $WORLD_ODOM robot:=$ROBOT > "$OUT/day2_gz.log" 2>&1 &
G=$!; sleep 15; spawn_shelf
[ "$MODE" != run ] && spawn_extra
if [ "$REC" = record ]; then
  ros2 run gazebo_ros spawn_entity.py -entity spectator_cam -file "$M/spectator_cam/model.sdf" -x 5.9 -y 0.4 -z 2.1 -P 0.55 -Y 2.58 > /dev/null 2>&1
  python3 "$D/record_view.py" "$OUT/$MODE.raw.mp4" > "$OUT/record.log" 2>&1 &
  V=$!
fi
setsid ros2 launch capstone_navigation slam.launch.py use_sim_time:=true $SLAM_ODOM localization:=true database_path:="$OUT/map.db" > "$OUT/day2_slam.log" 2>&1 &
R=$!; sleep 10
setsid ros2 launch capstone_navigation nav.launch.py use_sim_time:=true $NAV_ODOM > "$OUT/day2_nav.log" 2>&1 &
N=$!; sleep 20
timeout 6000 python3 "$D/shelf_test.py" $MODE "$OUT" 2>&1 | tee "$OUT/day2_$MODE.txt"
if [ -n "$V" ]; then
  kill -INT $V; sleep 3
  ffmpeg -y -loglevel error -i "$OUT/$MODE.raw.mp4" -vf "setpts=PTS/2" -r 20 -c:v libx264 -pix_fmt yuv420p -crf 26 "$OUT/$MODE.mp4" && rm "$OUT/$MODE.raw.mp4"
  echo "녹화: $OUT/$MODE.mp4 (2 배속)"
fi
stop $N $R $G
