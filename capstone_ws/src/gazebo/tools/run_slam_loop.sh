#!/bin/bash
# Gazebo(school_floor, 가짜 바퀴 odom + 가짜 VIO) + slam.launch(EKF + RTAB-Map) → 복도 루프 자동 주행 → 평가
# 사용: src/gazebo/tools/run_slam_loop.sh <출력디렉터리> [slam.launch 인자...]   (capstone_ws 빌드 후)
# ROS_DOMAIN_ID=77 로 실기 망과 분리. 끝나면 <출력>/vslam_result.png, final_map.png
OUT=$1; shift; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
source /home/eomyunbeen/capstone/capstone_ws/install/setup.bash
setsid ros2 launch capstone_gazebo world.launch.py gui:=false odom_tf:=false > "$OUT/gz.log" 2>&1 &
G=$!
sleep 15
setsid ros2 launch capstone_navigation slam.launch.py use_sim_time:=true new_map:=true database_path:="$OUT/rtabmap.db" "$@" > "$OUT/rtabmap.log" 2>&1 &
R=$!
sleep 8
timeout ${SLAM_TIMEOUT:-900} python3 "$(dirname "$0")/slam_loop_run.py" "$OUT" odometry/filtered
kill -INT -- -$R -$G 2>/dev/null; sleep 5; kill -KILL -- -$R -$G 2>/dev/null
sleep 1
rtabmap-export --poses --output_dir "$OUT" "$OUT/rtabmap.db" > "$OUT/export.log" 2>&1
python3 "$(dirname "$0")/slam_eval_db.py" "$OUT"
echo "rejected LC: $(grep -c 'Rejected loop closure' $OUT/rtabmap.log), 5s no-data: $(grep -c 'Did not receive data' $OUT/rtabmap.log)"
