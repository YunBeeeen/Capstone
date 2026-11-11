# capstone_navigation — 위치 추정·매핑 (시뮬·실기 공용)

```
바퀴 odom (/odom) ─────────────┐
ZED VIO (/zed/zed_node/odom) ──┼─▶ EKF (robot_localization) ─▶ TF odom→base_footprint ─▶ RTAB-Map ─▶ TF map→odom, /rtabmap/map
ZED 자이로 (imu/data, z 축만) ─┘                                                       (RGB-D, odom 은 TF 로 받음)
```
| 파일 | 내용 |
|---|---|
| `config/ekf.yaml` | EKF 입력: 바퀴 odom 속도(vx, vy, vyaw) + ZED VIO 변화량(differential x, y, yaw) + 자이로 z. 근거는 파일 주석 |
| `launch/slam.launch.py` | EKF + RTAB-Map. 모드: `new_map:=true` / 기본(이어서 매핑, 안 지움) / `localization:=true` (대회 2 일차) |

```bash
ros2 launch capstone_navigation slam.launch.py use_sim_time:=true new_map:=true   # 시뮬 (Gazebo odom_tf:=false 와)
ros2 launch capstone_navigation slam.launch.py localization:=true                 # 저장된 지도로 위치만
```
- 지도 파일 `~/.ros/capstone_rtabmap.db`. 매핑 모드로 켤 때마다 `~/.ros/rtabmap_backup/` 에 자동 백업
- RTAB-Map 은 odom 을 **TF 로** 받음 (영상 시각으로 보간) — 토픽 exact sync 는 주기·시계가 다르면 짝이 안 맞음
- 실기 전 할 일: RPi ↔ 상위 컴퓨터 **chrony 시간 동기화**, 바퀴 브리지 스탬프를 측정 시각으로, `swerve_odom publish_tf:=false`,
  ZED wrapper `publish_tf:=false publish_map_tf:=false`, `pos_tracking.area_memory:=false` (루프 클로저 때 odom 순간이동 방지),
  `pos_tracking.two_d_mode:=true`
- 시뮬 검증은 `capstone_gazebo` README "SLAM 시험" 참고. **시뮬 수치는 성능 근거가 아님**
