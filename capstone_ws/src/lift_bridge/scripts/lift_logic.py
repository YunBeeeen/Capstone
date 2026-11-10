"""리프트 Pico (yor-lift-position-v1) 상위 로직 — ROS·시리얼 무관 (가짜 시간·상태로 테스트).

Pico 펌웨어 (capstone_ws/Lift/firmware/main.py, 동료 작성) 요약:
  - 라파 → Pico: JSON 한 줄 명령 (arm / disarm / jog / move_to / heartbeat / stop / clear / zero / forget_zero)
  - Pico → 라파: 100 ms 마다 JSON 상태 한 줄
  - 움직이는 동안 명령(jog 반복 또는 heartbeat)이 300 ms 끊기면 Pico 가 스스로 멈춤 (command_timeout)
  - 엔코더는 증분식 → 원점(0 mm)은 Pico 가 켜질 때마다 새로 등록해야 함. 보정값(mm/count)은 Pico 에 저장돼 있음
  - 원점 등록 뒤엔 10..620 mm 밖으로 jog/move_to 안 감 (소프트 리미트)

이 로직이 하는 일:
  1) 조그: 누르는 동안 jog 반복, 떼면 stop. 방향을 바꿀 땐 Pico 가 멈춘 걸 확인한 뒤 반대로 (펌웨어가 바로 반전하면 fault)
  2) 주차 (종료 시 / 패드 P): move_to 10 mm → 원점 기준 잠깐 해제 → 남은 거리를 엔코더 카운트로 세며 천천히 내림
     → 멈춘 뒤 잔차 기록 → 그 자리 = 0 으로 다시 등록. 주차 정보는 파일로 남김
  3) 켤 때: 주차 파일이 있으면 (= 깨끗하게 최하단에 세워 두고 껐음) 자동으로 원점 등록 (잔차는 offset 으로 보정)
     없으면 미등록 — 조그로 최하단까지 내린 뒤 zero 를 1 s 누름
  높이 = Pico 위치 + offset (offset = 주차 잔차, |offset| ≲ 0.5 mm. 주차를 반복해도 오차가 쌓이지 않게)
"""
import math
from dataclasses import dataclass

PROTOCOL = "yor-lift-position-v1"
MOVING_STATES = ("JOG", "MOVING", "SETTLING")
PARK_TASKS = ("PARK_MOVE", "PARK_DISARM", "PARK_FORGET", "PARK_DESCEND", "PARK_SETTLE")


@dataclass
class LiftConfig:
    mm_per_count: float = 0.21574    # Pico 보정값과 같게 (동료 실측: 122.32653 mm / 567 count)
    encoder_sign: int = 1
    jog_duty: float = 0.50           # 조그 PWM 기본 (명령 duty = 0 일 때). 패드: 느림 0.5 / 빠름 1.0
    max_jog_duty: float = 1.0
    unhomed_max_duty: float = 0.5    # 원점 미등록 (끝단 보호 없음) 일 땐 이 이상 안 냄
    jog_start_duty: float = 0.3      # 조그 시작 PWM — 펌웨어 조그엔 램프가 없어서 브리지가 올림
    jog_ramp_per_s: float = 2.0      # [PWM/s] 0.3 → 1.0 에 0.35 s. 낮출 땐 즉시
    cmd_timeout: float = 0.3         # [s] 명령이 이보다 오래되면 조그 정지
    status_timeout: float = 0.6      # [s] Pico 상태가 이보다 오래되면 연결 끊김으로 봄
    zero_hold_s: float = 1.0         # [s] zero 를 이만큼 계속 눌러야 원점 등록
    retry_s: float = 0.3             # [s] arm / zero / forget_zero / move_to 재요청 주기
    stop_confirm_s: float = 0.15     # [s] stop 보낸 뒤 이만큼 지난 상태로 "멈춤" 확인 (Pico 보고 100 ms)
    park_target_mm: float = 10.0     # 펌웨어 move_to 하한 (WORK_MARGIN_MM)
    park_move_duty: float = 0.6      # 10 mm 까지 move_to 최대 PWM
    park_duty: float = 0.25          # 마지막 구간 (원점 해제 상태) 조그 PWM
    park_stop_mm: float = 0.3        # 바닥 이만큼 전에 stop (관성)
    park_descend_max_mm: float = 15.0  # 마지막 구간이 이보다 길면 이상 → 중단
    park_move_timeout: float = 60.0  # [s] 630 mm 를 30 mm/s 로 21 s
    park_descend_timeout: float = 10.0
    stationary_s: float = 0.4        # [s] 카운트가 이만큼 안 변하면 정지로 봄 (펌웨어 STATIONARY_MS 300)
    zero_timeout: float = 3.0
    parked_tol_mm: float = 1.0       # 주차 후 이보다 움직이면 주차 무효


@dataclass
class LiftCmd:
    jog: int = 0
    duty: float = 0.0
    stop: bool = False
    park: bool = False
    zero: bool = False
    clear: bool = False


def msg(cmd, **kw):
    d = {"cmd": cmd}
    d.update(kw)
    return d


class LiftLogic:
    def __init__(self, cfg: LiftConfig, store=None):
        self.cfg = cfg
        self.kmm = cfg.mm_per_count * cfg.encoder_sign  # [mm/count] (부호 포함)
        # 파일로 남는 값: parked(주차 후 안 움직임), residual_mm(주차 잔차), offset_mm(이번 원점의 보정)
        self.store = {"parked": False, "residual_mm": 0.0, "offset_mm": 0.0}
        self.store.update(store or {})
        self.store_dirty = False
        self.st = None
        self.st_time = -math.inf
        self.cmd = LiftCmd()
        self.cmd_time = -math.inf
        self.prev = LiftCmd()
        self.zero_since = None
        self.zero_used = False
        self.count_start = None
        self.offset = 0.0
        self.task = ""
        self.task_t0 = 0.0
        self.last_req = -math.inf
        self.last_arm = -math.inf
        self.jogging = False
        self.jog_duty_now = 0.0
        self.jog_duty_time = 0.0
        self.last_dir = 0
        self.stop_time = -math.inf
        self.still_count = None
        self.still_since = 0.0
        self.c0 = None             # 주차: 최하단(실제 0 mm) 의 카운트
        self.zero_offset = 0.0     # ZERO 작업이 끝나면 쓸 offset
        self.zero_parked = False   # ZERO 작업이 주차의 마지막 단계(또는 주차 뒤 부팅)인가
        self.zero_boot = False     # 켤 때 자동 등록인가
        self.zero_sent = -math.inf
        self.park_result = None    # None / "ok" / "failed" / "aborted"
        self.message = ""
        self.new_message = False
        self.booted = False
        self.was_homed = False

    # ── 입력 ──
    def on_status(self, st: dict, now: float) -> bool:
        if not isinstance(st, dict) or st.get("protocol") != PROTOCOL or "raw_count" not in st:
            return False
        self.st, self.st_time = st, now
        c = st["raw_count"]
        if self.count_start is None:
            self.count_start = c
        if c != self.still_count:
            self.still_count, self.still_since = c, now
        if not self.booted:
            self.booted = True
            self._on_boot(now)
        homed = bool(st.get("homed"))
        self._check_scale(st)
        if self.was_homed and not homed and self.task not in PARK_TASKS and self.task != "ZERO":
            self._say(f"Pico 원점 잃음 ({st.get('fault') or st.get('event')}) — 최하단에서 원점 다시 등록 필요")
            self._set_store(parked=False)
        self.was_homed = homed
        if self.store["parked"] and homed and self.task == "":
            h = self.height()
            if h is not None and abs(h) > self.cfg.parked_tol_mm:
                self._set_store(parked=False)
        return True

    def on_command(self, cmd: LiftCmd, now: float):
        self.cmd, self.cmd_time = cmd, now

    def request_park(self, now: float):
        """종료 시: 이미 주차돼 있으면 아무것도 안 함."""
        if self.is_parked():
            self._say("이미 주차됨 (최하단)")
            self.park_result = "ok"
            return
        self._start_park(now)

    # ── 조회 ──
    def connected(self, now):
        return self.st is not None and now - self.st_time <= self.cfg.status_timeout

    def homed(self):
        return bool(self.st and self.st.get("homed"))

    def height(self):
        if not self.homed() or self.st.get("position_mm") is None:
            return None
        return self.st["position_mm"] + self.offset

    def rel_mm(self):
        if self.st is None or self.count_start is None:
            return 0.0
        return (self.st["raw_count"] - self.count_start) * self.kmm

    def is_parked(self):
        h = self.height()
        return self.store["parked"] and h is not None and abs(h) <= self.cfg.parked_tol_mm

    def busy(self):
        return self.task != ""

    # ── 주기 실행 ──
    def step(self, now: float) -> list:
        out = []
        if not self.connected(now):
            return out
        c = self.cmd if now - self.cmd_time <= self.cfg.cmd_timeout else LiftCmd()
        park_edge = c.park and not self.prev.park
        clear_edge = c.clear and not self.prev.clear
        self.prev = c
        if c.zero:
            self.zero_since = now if self.zero_since is None else self.zero_since
        else:
            self.zero_since, self.zero_used = None, False

        if self.task and (c.stop or c.jog != 0):
            self._abort(now, out, "정지 버튼" if c.stop else "조그 입력")
            return out
        if c.stop:
            self._stop(now, out)
            return out
        if clear_edge:
            out.append(msg("clear"))
            self._say("Pico 오류 해제 요청")
        if park_edge and not self.task:
            self.request_park(now)
        if (self.zero_since is not None and not self.zero_used and not self.task
                and now - self.zero_since >= self.cfg.zero_hold_s):
            self.zero_used = True
            self._start_zero(now, offset=0.0, parked=False, why="수동 (최하단 확인)")

        if self.task:
            out += self._task_step(now)
            return out
        out += self._jog_step(now, c)
        return out

    # ── 조그 ──
    def _jog_step(self, now, c):
        out = []
        st = self.st
        want = max(-1, min(1, int(c.jog)))
        pstate = st.get("state")
        if want == 0:
            if self.jogging:
                self._stop(now, out)
            elif pstate == "JOG" and now - self.stop_time >= self.cfg.retry_s:
                self._stop(now, out)  # 놓친 stop 재송신
            return out
        if st.get("fault"):
            return out  # clear 필요
        if not st.get("armed"):
            if now - self.last_arm >= self.cfg.retry_s:
                out.append(msg("arm"))
                self.last_arm = now
            return out
        if want != self.last_dir and self.last_dir != 0:
            # 반전: 앞 방향이 멈춘 게 확인될 때까지 기다림 (stop 이후에 만들어진 상태로 확인)
            if self.jogging:
                self._stop(now, out)
                return out
            if pstate == "JOG" or self.st_time < self.stop_time + self.cfg.stop_confirm_s:
                return out
        cfg = self.cfg
        target = min(c.duty if c.duty > 0 else cfg.jog_duty, cfg.max_jog_duty)
        if not self.homed():
            target = min(target, cfg.unhomed_max_duty)
        if not self.jogging or want != self.last_dir:
            duty = min(target, cfg.jog_start_duty)
        else:
            duty = min(target, self.jog_duty_now + cfg.jog_ramp_per_s * (now - self.jog_duty_time))
        self.jog_duty_now, self.jog_duty_time = duty, now
        out.append(msg("jog", direction=want, duty=round(duty, 3)))
        self.jogging, self.last_dir = True, want
        if self.store["parked"]:
            self._set_store(parked=False)
        return out

    def _stop(self, now, out):
        out.append(msg("stop"))
        self.stop_time = now
        self.jogging = False

    # ── 작업 (주차 / 원점 등록) ──
    def _start_park(self, now):
        if not self.homed():
            self._say("원점 미등록이라 주차 불가 — 조그로 최하단까지 내린 뒤 원점 등록 (zero 1 s)")
            self.park_result = "failed"
            return
        self._set_task("PARK_MOVE", now)
        self.park_result = None
        self._say(f"주차 시작: {self.height():.1f} mm → 0 mm")

    def _start_zero(self, now, offset, parked, why, boot=False):
        self.zero_offset, self.zero_parked, self.zero_boot = offset, parked, boot
        self.zero_sent = -math.inf
        self._set_task("ZERO", now)
        self._say(f"원점 등록 ({why})")

    def _set_task(self, task, now):
        self.task, self.task_t0, self.last_req = task, now, -math.inf

    def _abort(self, now, out, why):
        was = self.task
        self._stop(now, out)
        out.append(msg("disarm"))
        self.task = ""
        if was in PARK_TASKS:
            self.park_result = "aborted"
            self._say(f"주차 취소 ({why})" + ("" if self.homed() else " — 원점 해제된 상태, 다시 등록 필요"))
        else:
            self._say(f"원점 등록 취소 ({why})")

    def _fail(self, now, out, why):
        was = self.task
        self._stop(now, out)
        out.append(msg("disarm"))
        self.task = ""
        parking = was in PARK_TASKS or (was == "ZERO" and self.zero_parked)
        if parking:
            self.park_result = "failed"
        self._say(f"{'주차' if parking else '원점 등록'} 실패: {why}")
        self._set_store(parked=False)

    def _stationary(self, now):
        return now - self.still_since >= self.cfg.stationary_s

    def _task_step(self, now):
        out = []
        st, cfg = self.st, self.cfg
        pstate, fault = st.get("state"), st.get("fault") or ""
        count = st["raw_count"]
        elapsed = now - self.task_t0

        if self.task == "PARK_MOVE":
            if fault:
                self._fail(now, out, f"이동 중 Pico fault {fault}")
                return out
            pos = st.get("position_mm")
            target = math.ceil(cfg.park_target_mm / cfg.mm_per_count - 1e-9) * cfg.mm_per_count
            if pstate in ("MOVING", "SETTLING"):
                out.append(msg("heartbeat"))
            elif pstate == "JOG":
                self._stop(now, out)
            elif pos is not None and pos <= target + 1.5:
                self._set_task("PARK_DISARM", now)  # 이미 10 mm 근처 (또는 도착)
            elif not st.get("armed"):
                if now - self.last_arm >= cfg.retry_s:
                    out.append(msg("arm"))
                    self.last_arm = now
            elif now - self.last_req >= cfg.retry_s:
                out.append(msg("move_to", target_mm=round(target, 5), max_duty=cfg.park_move_duty))
                self.last_req = now
            if self.task == "PARK_MOVE" and elapsed > cfg.park_move_timeout:
                self._fail(now, out, f"{cfg.park_move_timeout:.0f} s 안에 {cfg.park_target_mm:.0f} mm 도착 못 함")
            return out

        if self.task == "PARK_DISARM":
            if st.get("armed"):
                if now - self.last_req >= cfg.retry_s:
                    out.append(msg("disarm"))
                    self.last_req = now
            elif self._stationary(now):
                pos = st.get("position_mm")
                if pos is None:
                    self._fail(now, out, "원점 정보 없음")
                    return out
                dist = pos + self.offset  # 실제 바닥까지 남은 거리
                if not -cfg.parked_tol_mm <= dist <= cfg.park_descend_max_mm:
                    self._fail(now, out, f"마지막 구간 {dist:.1f} mm — 예상 밖")
                    return out
                self.c0 = count - dist / self.kmm
                self._set_task("PARK_FORGET", now)
            elif elapsed > 5.0:
                self._fail(now, out, "정지 확인 안 됨")
            return out

        if self.task == "PARK_FORGET":
            if not st.get("homed"):
                self._set_task("PARK_DESCEND", now)
            elif now - self.last_req >= cfg.retry_s:
                out.append(msg("forget_zero"))
                self.last_req = now
            if self.task == "PARK_FORGET" and elapsed > 3.0:
                self._fail(now, out, "원점 해제(forget_zero) 거부됨")
            return out

        if self.task == "PARK_DESCEND":
            remaining = (count - self.c0) * self.kmm
            if remaining <= cfg.park_stop_mm:
                self._stop(now, out)
                self._set_task("PARK_SETTLE", now)
            elif fault:
                if fault == "encoder_no_pulse" and remaining <= 3.0:
                    # 예상보다 먼저 바닥에 닿음 (이전 원점이 약간 위였음) → 여기가 바닥.
                    # 3 mm 넘게 남았는데 멈췄으면 park_duty 가 약해 안 움직인 것 → 아래 실패로
                    self._say(f"바닥에 {remaining:.1f} mm 먼저 닿음 — 여기를 0 으로")
                    self.c0 = count
                    self._set_task("PARK_SETTLE", now)
                else:
                    hint = " — park_duty 를 올려 볼 것" if fault in ("encoder_no_pulse", "motion_no_progress") else ""
                    self._fail(now, out, f"내리는 중 Pico fault {fault} ({remaining:.1f} mm 남음){hint}")
            elif not st.get("armed"):
                if now - self.last_arm >= cfg.retry_s:
                    out.append(msg("arm"))
                    self.last_arm = now
            else:
                out.append(msg("jog", direction=-1, duty=cfg.park_duty))
                self.jogging, self.last_dir = True, -1
            if self.task == "PARK_DESCEND" and elapsed > cfg.park_descend_timeout:
                self._fail(now, out, f"마지막 {remaining:.1f} mm 를 {cfg.park_descend_timeout:.0f} s 안에 못 내려감")
            return out

        if self.task == "PARK_SETTLE":
            if pstate == "JOG" and now - self.stop_time >= cfg.retry_s:
                self._stop(now, out)
            elif st.get("armed"):
                if now - self.last_req >= cfg.retry_s:
                    out.append(msg("disarm"))
                    self.last_req = now
            elif self._stationary(now) and pstate != "JOG":
                residual = (count - self.c0) * self.kmm
                self._start_zero(now, offset=residual, parked=True, why=f"주차 끝, 잔차 {residual:+.2f} mm")
            elif elapsed > 5.0:
                self._fail(now, out, "정지 확인 안 됨")
            return out

        if self.task == "ZERO":
            # zero 를 보낸 뒤에 만들어진 상태로 확인 (예전 event 가 남아 있을 수 있음)
            if (st.get("event") == "manual_zero_set" and st.get("homed")
                    and self.st_time > self.zero_sent + cfg.stop_confirm_s):
                self.offset = self.zero_offset
                self.task = ""
                if self.zero_parked:
                    self._set_store(parked=True, residual_mm=self.zero_offset, offset_mm=self.zero_offset)
                    if self.zero_boot:
                        self._say(f"자동 원점 등록 완료 — 최하단 = 0 mm (보정 {self.zero_offset:+.2f} mm). 끝단 보호 켜짐")
                    else:
                        self.park_result = "ok"
                        self._say(f"주차 완료 — 최하단 (잔차 {self.zero_offset:+.2f} mm)")
                else:
                    self._set_store(parked=False, offset_mm=self.zero_offset)
                    self._say(f"원점 등록 완료 (offset {self.zero_offset:+.2f} mm)")
                return out
            if fault:
                if now - self.last_req >= cfg.retry_s:
                    out.append(msg("clear"))
                    self.last_req = now
            elif st.get("armed"):
                if now - self.last_req >= cfg.retry_s:
                    out.append(msg("disarm"))
                    self.last_req = now
            elif not st.get("calibrated"):
                self._fail(now, out, "Pico 보정값(mm/count) 없음 — 동료 GUI 로 보정 먼저")
                return out
            elif now - self.last_req >= cfg.retry_s:
                out.append(msg("zero", at_lower_reference=True))
                self.last_req = self.zero_sent = now
            if self.task == "ZERO" and elapsed > cfg.zero_timeout:
                self._fail(now, out, f"Pico 가 원점 등록 거부 ({st.get('event')})")
            return out
        return out

    # ── 켤 때 ──
    def _on_boot(self, now):
        st = self.st
        if st.get("homed"):
            self.offset = float(self.store.get("offset_mm", 0.0))  # Pico 는 안 꺼지고 브리지만 다시 켜짐
            self._say(f"Pico 원점 유지 중 — 높이 {self.height():.1f} mm")
        elif self.store.get("parked"):
            self._start_zero(now, offset=float(self.store.get("residual_mm", 0.0)), parked=True,
                             why="자동 — 지난번 최하단에 주차하고 종료함", boot=True)
        else:
            self._say("원점 미등록 — 조그로 최하단까지 내린 뒤 zero 를 1 s (그 전엔 끝단 보호 없음, 보면서 조작)")

    def _check_scale(self, st):
        """원점 있을 때 Pico 위치 변화 / 카운트 변화 = 이 브리지의 mm_per_count 인지 (주차 마지막 구간이 이 값에 의존)."""
        pos = st.get("position_mm")
        if not st.get("homed") or pos is None:
            self.scale_ref = None
            return
        ref = getattr(self, "scale_ref", None)
        if ref is None:
            self.scale_ref = (st["raw_count"], pos)
            return
        dc = st["raw_count"] - ref[0]
        if abs(dc) >= 100:
            k = (pos - ref[1]) / dc
            if abs(k - self.kmm) > 0.03 * abs(self.kmm) and not getattr(self, "scale_warned", False):
                self.scale_warned = True
                self._say(f"경고: Pico mm/count {k:.5f} ≠ 브리지 {self.kmm:.5f} — lift_bridge 파라미터 mm_per_count 맞출 것")
            self.scale_ref = (st["raw_count"], pos)

    # ── 기타 ──
    def _set_store(self, **kw):
        for k, v in kw.items():
            if self.store.get(k) != v:
                self.store[k] = v
                self.store_dirty = True

    def _say(self, text):
        self.message = text
        self.new_message = True

    def state(self, now) -> dict:
        st = self.st or {}
        h = self.height()
        return {
            "connected": self.connected(now),
            "pico_state": st.get("state", ""),
            "armed": bool(st.get("armed")),
            "homed": self.homed(),
            "height_mm": h if h is not None else math.nan,
            "rel_mm": self.rel_mm(),
            "velocity_mm_s": float(st.get("velocity_mm_s") or 0.0),
            "task": self.task,
            "parked": self.is_parked(),
            "fault": st.get("fault", "") or "",
            "event": st.get("event", "") or "",
            "message": self.message,
        }
