# XDrive(ODrive v3.6) + BL41105 명령어 정리

보드: Makerbase XDrive V3.6 (= ODrive v3.6-56V), **firmware v0.5.1** — 업데이트/재플래시 금지.
PC: `source ~/odrive_env/bin/activate` → `odrivetool` (odrive 0.6.11).
상세 배경·배선: `~/Downloads/capstone_xdrive_odrive_handoff.md`. 스크립트 버전: `bringup.py`.

---

## 0. 확정된 값 (보드 #1, 2026-11-03)

| 항목 | 값 | 근거 |
|---|---|---|
| pole_pairs | **10** | Hall 60카운트/바퀴 실측 + 0.5 turn/s × 10s = 정확히 5바퀴. (판매처 표의 "극수 10"은 실제로 극쌍수) |
| encoder | Hall, `mode=1`, `cpr=60` (=6×pp) | |
| R / L (상당) | axis0 **0.281 Ω / 157 µH**, axis1 **0.276 Ω / 153 µH** | state 4 측정. 다른 모터도 ±10% 안이어야 정상 |
| Kv / Kt | 41.7 rpm/V / 0.198 Nm/A | 사양 24V 1000rpm 역산 — **근사값**, 토크 쓸 때 실측 필요 |
| 사양 | 정격 3.5 Nm, 최대 6 Nm, 타이어 Ø105 mm, Hall 5V | 판매 페이지 상세 이미지 |
| 무부하 Iq | ≈ 0.2–0.3 A @ 0.5 turn/s (두 축 동시, vbus 24.74 V) | |

---

## 1. ⚠️ 공장 설정 함정 (모든 축·모든 보드에서 먼저 처리)

출고 상태:
```
axis.config.startup_encoder_offset_calibration = True
axis.config.startup_closed_loop_control        = True
axis.motor.config.pre_calibrated               = True   # 시험용 모터 R/L (0.085Ω, 14µH) 이 들어있음
```
→ 전원만 넣어도 바퀴가 돌며 오프셋 캘리브 후 **자동 폐루프**, 그것도 **틀린 R/L 로 계산된 전류 게인**으로.
처리:
```python
a = odrv0.axis0          # axis1 도 동일
a.requested_state = 1
a.config.startup_encoder_offset_calibration = False
a.config.startup_closed_loop_control = False
a.motor.config.pre_calibrated = False
```

---

## 2. 객체 구조 — `axis` 의 의미

ODrive 한 장 = 모터 채널 2개. 각 채널이 `axis0`(M0), `axis1`(M1).
아래 표에서 `axis.` 는 `odrv0.axis0.` 또는 `odrv0.axis1.` 의 줄임.

```
odrv0                         보드 전체 (vbus_voltage, save_configuration(), reboot())
├─ axis0                      M0 채널 = 모터 1개의 전체 제어 단위  (CAN 에서는 CAN ID 하나 = can_node_id)
│  ├─ requested_state / current_state / error / config.startup_*
│  ├─ motor        상 전류·R·L·극쌍수·전류제한     (motor.config.*, motor.current_control.*)
│  ├─ encoder      Hall 모드·cpr·위치/속도 추정    (encoder.config.*, encoder.vel_estimate)
│  └─ controller   제어모드·명령값·게인            (controller.config.*, controller.input_vel)
└─ axis1                      M1 채널 (구조 동일)
```

---

## 3. 상태 (`requested_state`, fw 0.5.x)

| 값 | 이름 | 동작 |
|---|---|---|
| 0 | UNDEFINED | **"요청 없음" — 꺼지지 않음.** 현재 상태 유지 |
| 1 | IDLE | 전류 차단, 손으로 자유회전. **정지는 항상 1** |
| 4 | MOTOR_CALIBRATION | R/L 측정. 삐— 소리, 거의 안 돎 |
| 7 | ENCODER_OFFSET_CALIBRATION | Hall-전기각 오프셋. 바퀴가 ~2.4바퀴 갔다가 돌아옴 |
| 8 | CLOSED_LOOP_CONTROL | 폐루프 제어 |

---

## 4. 브링업 절차 (axis0 에서 실제로 한 순서)

> odrivetool 은 여러 줄을 붙이면 **마지막 줄 결과만 출력** → 값 확인은 `print(...)`.
> 재부팅/“disappeared” 후엔 **`a = odrv0.axis0` 다시 실행** (예전 객체는 죽은 연결).
> "기다려야 하는" 명령(input_vel 등)은 **한 줄씩 Enter**.

**준비**: 축 평면부가 브라켓 슬롯에 끼워짐, 너트 양쪽 적당히, 타이어 자유회전, 바퀴 띄움.

```python
# (1) Hall 모드
odrv0.axis0.encoder.config.mode = 1
odrv0.save_configuration(); odrv0.reboot()

# (2) Hall 확인 + 극쌍수 측정 (재연결 후)
a = odrv0.axis0
a.encoder.error = 0
a.encoder.hall_state                         # 손으로 돌리며 → 1~6 만 나와야 함 (0/7 = 배선 불량)
c0 = a.encoder.shadow_count                  # 정확히 1바퀴 돌린 뒤 ↓
abs(a.encoder.shadow_count - c0) / 6         # = pole_pairs

# (3) 설정 (한 번에 붙여도 됨 — 모터 안 움직임)
pp, kv = 10, 41.7
a.motor.config.pole_pairs = pp
a.motor.config.torque_constant = 8.27 / kv
a.motor.config.resistance_calib_max_voltage = 4
a.motor.config.requested_current_range = 25        # 재부팅 후 적용
a.motor.config.current_control_bandwidth = 100
a.motor.config.calibration_current = 5
a.motor.config.current_lim = 10
a.encoder.config.cpr = 6 * pp
a.encoder.config.bandwidth = 100
a.encoder.config.calib_scan_distance = 150
a.controller.config.control_mode = 2               # VELOCITY
a.controller.config.input_mode = 2                 # VEL_RAMP
a.controller.config.vel_ramp_rate = 2
a.controller.config.vel_limit = 2
a.controller.config.pos_gain = 1
a.controller.config.vel_gain = 0.02 * a.motor.config.torque_constant * a.encoder.config.cpr
a.controller.config.vel_integrator_gain = 0.1 * a.motor.config.torque_constant * a.encoder.config.cpr
odrv0.save_configuration(); odrv0.reboot()

# (4) 공장 함정 처리 → §1

# (5) 캘리브레이션
a = odrv0.axis0
a.error = 0; a.motor.error = 0; a.encoder.error = 0; a.controller.error = 0
a.requested_state = 4
print(a.current_state, a.motor.config.phase_resistance, a.motor.config.phase_inductance, a.motor.is_calibrated)
a.requested_state = 7
print(a.current_state, a.encoder.is_ready); dump_errors(odrv0)

# (6) 결과 저장 → 다음 부팅부터 캘리브 생략, IDLE 로 대기
a.motor.config.pre_calibrated = True
a.encoder.config.pre_calibrated = True
odrv0.save_configuration(); odrv0.reboot()
a = odrv0.axis0
print(a.current_state, a.motor.is_calibrated, a.encoder.is_ready)   # 1 True True

# (7) 회전 시험 — 한 줄씩
a.requested_state = 8
a.controller.input_vel = 0.5        # 10초 → 5바퀴
print(a.encoder.vel_estimate, a.motor.current_control.Iq_measured)
a.controller.input_vel = 0
a.controller.input_vel = -0.5
a.controller.input_vel = 0
a.requested_state = 1
```

**비상정지**: `odrv0.axis0.requested_state = 1` (미리 쳐두고 ↑ 로 재호출) 또는 배터리 차단.

설정 백업 (odrivetool 종료 후): `odrivetool backup-config ~/capstone/capstone_ws/Wheel_motor/xdrive1_config.json`

---

## 5. 게인

ODrive 은 3중 루프. **D 항은 없음** (위치 P → 속도 PI → 전류 PI).

```
input_pos ─[pos_gain]→ vel ─[vel_gain, vel_integrator_gain]→ torque ─÷Kt→ Iq ─[전류 PI]→ V → SVPWM
```

| 루프 | 파라미터 | 현재 값 | 출처 |
|---|---|---|---|
| 위치 P | `controller.config.pos_gain` | 1 (turn/s)/turn | hoverboard 가이드 (속도모드라 지금은 안 씀) |
| 속도 P | `controller.config.vel_gain` | **0.238** Nm/(turn/s) ≈ 1.2 A per 1 turn/s 오차 | `0.02·Kt·cpr` |
| 속도 I | `controller.config.vel_integrator_gain` | **1.19** Nm/turn ≈ 6 A per 1 turn 누적오차 | `0.1·Kt·cpr` |
| 전류 PI | 자동: Kp = L·ω_bw, Ki = R·ω_bw | Kp ≈ 0.0157 V/A, Ki ≈ 28.1 V/(A·s) | `current_control_bandwidth=100`, 캘리브의 R/L 로 자동 계산 (극-영점 상쇄) |

참고: ODrive 공장 기본값은 pos_gain 20, vel_gain 0.1667, vel_integrator_gain 0.333 (소형 모터용).
게인 공식의 Kt 는 뒤의 ÷Kt 와 상쇄 → **Kv 가 틀려도 실제 전류 응답은 같음**.

현재 값은 **출발점**. 바퀴 띄운 상태(관성 거의 0)에선 예민하게 느껴지는 게 정상이고,
손으로 잡고 있으면 I 항이 쌓였다가 놓는 순간 튕김(와인드업). **튜닝은 지면에 내려 로봇 하중을 실은 뒤.**
Hall 6° 해상도 때문에 저속(≲0.5 turn/s) 속도추정 노이즈 ±20% 정도 — 평균은 정확.

---

## 6. odrivetool ↔ CAN (CANSimple, fw 0.5.1) 대응

프레임 ID = `(can_id << 5) | cmd_id` (can_id = 보드의 `can_node_id`, 스크립트 `--can-id`). 기본 250 kbps. 데이터 little-endian.
XDrive 공장값은 axis0 = 16, axis1 = 1 → **CAN ID 는 1~4 로 통일: 보드 #1 = 1/2, 보드 #2 = 3/4** (USB 로 바꿔 저장 후 같은 버스).
버스 양끝 120 Ω (전원 끄고 CANH–CANL ≈ 60 Ω).

| odrivetool | CAN 메시지 (cmd) | 방향 |
|---|---|---|
| `axis.requested_state = N` | Set_Axis_State (0x007) | Pi → 보드 |
| `axis.current_state`, `axis.error` | Heartbeat (0x001), 주기 자동 — byte0-3 axis_error u32, byte4-7 state u32 | 보드 → Pi |
| `axis.controller.input_vel` | Set_Input_Vel (0x00D) — vel, torque_ff (float×2) | Pi → 보드 |
| `axis.controller.input_torque` | Set_Input_Torque (0x00E) | Pi → 보드 |
| `axis.controller.config.control_mode / input_mode` | Set_Controller_Mode (0x00B) | Pi → 보드 |
| `axis.controller.config.vel_limit` | **0.5.1: Set_Velocity_Limit (0x00F)** — current_lim 은 CAN 불가 (0.5.6 부터 Set_Limits) | Pi → 보드 |
| `axis.encoder.pos_estimate / vel_estimate` | Get_Encoder_Estimates (0x009) — **0.5.1 은 RTR 요청→응답만** (주기 송신 `encoder_rate_ms` 는 0.5.4+) | 보드 → Pi |
| `axis.motor.current_control.Iq_measured` | Get_Iq (0x014), RTR | 보드 → Pi |
| `odrv0.vbus_voltage` | Get_Vbus_Voltage (0x017) | 보드 → Pi |
| 에러 클리어 | Clear_Errors (0x018) | Pi → 보드 |
| `odrv0.reboot()` | Reboot (0x016) | Pi → 보드 |
| 비상정지 | Estop (0x002) | Pi → 보드 |

0.5.1 메시지는 **0x018 Clear_Errors 까지**. Set_Pos_Gain/Set_Vel_Gains(0x01A/0x01B) 등은 0.5.6+ → **게인은 CAN 으로 못 바꿈**(USB 저장값 사용).
워치독이 어떤 CAN 메시지로 feed 되는지 문서에 없음 → 실험으로 확인.
출처: fw-v0.5.1 `docs/can-protocol.md`, docs.odriverobotics.com/v/0.5.6/can-protocol.html

**실측 확인 (2026-11-03, 보드 #1, candleLight gs_usb 어댑터)**
- 500 kbps 로 설정(`odrv0.can.set_baud_rate(500000)`), heartbeat 100 ms 주기, `00 00 00 00 | 01 00 00 00` = error 0, IDLE
- **XDrive 공장 CAN ID (can_node_id): axis0 = 16 (ID 0x2xx), axis1 = 1** → 바꾸기 전 보드 2장 같은 버스 금지 (axis1 = 1 충돌)
- (당시 ID 0/1 로 측정) RTR `009#R` / `029#R` (DLC 0 이나 8 모두) → Get_Encoder_Estimates 응답 OK, 손으로 돌리면 pos 변함. `037#R` → vbus 24.75 V
- 보드 DFU/RUN 스위치는 **RUN 고정**. CAN 종단 스위치 ON
- 수신 보기: `candump can_drive | grep -v -E " (021|041|061|081) "` (heartbeat 제외)

예 (CAN ID 1 → 프레임 ID = 1<<5 | cmd = 0x020 | cmd):
```bash
sudo ip link set can_drive up type can bitrate 250000
candump can_drive                              # 021 = ID1 heartbeat, 041 = ID2, 061 = ID3, 081 = ID4
cansend can_drive 027#08000000                 # CLOSED_LOOP
cansend can_drive 02D#0000003F00000000         # input_vel = 0.5 turn/s
cansend can_drive 027#01000000                 # IDLE
```

CAN 전환 시 USB 로 미리 (0.5.1 이름): `odrv0.axis0.config.can_node_id = 1`, `odrv0.can.set_baud_rate(500000)` (함수, 재부팅 불필요), **워치독**
(`axis.config.enable_watchdog = True`, `watchdog_timeout ≈ 0.2 s` — Pi/CAN 끊기면 자동 정지).

설계 방침: Pi 드라이버는 `requested_state`, `input_vel`, `vel_estimate` … **odrivetool 과 같은 이름**의 인터페이스로,
그 아래 전송만 USB/CAN 교체. 상태 전환(8)은 Pi 가 준비됐을 때만 — 부팅 자동 폐루프는 끔(§1).
로깅은 토크 대신 **Iq 원본** (Kt 는 나중에 실측해서 곱함).

---

## 7. 진행 체크리스트

- [x] 보드 #1 axis0: Hall, pp=10, R/L, 오프셋, pre_calibrated, 5바퀴 시험
- [ ] 설정 백업 (`backup-config`)
- [x] 보드 #1 axis1: 공장 함정 동일하게 있었음(확인·처리), 설정 복사, R/L ±3%, 오프셋, pre_calibrated
- [x] 두 모터 동시 구동
- [ ] 좌우 회전 부호 기록 (+input_vel 일 때 바깥에서 본 방향)
- [x] 배터리 확인: 볼턴 24V 30Ah 리튬이온(LG 셀) = **7S** (충전기 29.4 V, 2026-11-05). 완충 29.4 / 공칭 25.9 / 하한 ~21 V
- [ ] 보드별 (USB): 브레이크 저항(보드당 1개, AUX) 연결 후 `enable_brake_resistor=True`, `brake_resistance`=실측Ω, `max_regen_current=0`,
      `dc_bus_overvoltage_trip_level=32.0`, `dc_bus_undervoltage_trip_level=21.0` → 저장. 먼저 현재값 읽어서 0.5.1 속성 이름 확인
- [x] 소프트웨어 경고: `monitor.py` / 브리지 `vbus_warn` 22.5 V (3.2 V/셀) 이하 "충전 필요"
- [ ] (선택) 역기전력으로 Kv 실측
- [x] CAN 링크: 500k, heartbeat, RTR 피드백
- [x] CAN 회전 시험 (보드 #1, `wheel_can/vel.py`, 2026-11-04)
- [ ] 워치독 동작 확인
- [ ] CAN ID 1~4 재지정 (보드 #1 = 1/2, 보드 #2 = 3/4)
- [x] 보드 #2: startup 플래그·pre_calibrated 해제, 500k 저장 (serial 3160367B3335)
- [ ] 보드 #2 모터 연결 후 공통값 + 캘리브레이션 + CAN 회전
- [ ] 지면 하중 상태에서 속도 게인 튜닝
- [ ] 토크 쓸 단계 전에 Kt 레버+저울 실측
