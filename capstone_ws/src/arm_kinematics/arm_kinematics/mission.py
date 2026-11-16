"""MuJoCo 임무 시뮬: 책상에서 과일 집기 → 바구니에 넣기 → 바구니 들기 → 3 m 뒤 선반으로 주행 → 3층에 두기 → 팔 빼기.

ROS·Nav2 없이 임무 순서·IK·충돌·정지 오차 강건성을 본다.
- 배치 (방 좌표): 책상 설 자리 = 원점 +x, 선반 설 자리 = (−3, 0) −x. 가구는 choose_stand.py 결과 자리
  (책상 앞면 0.35 m, 선반 앞면 0.40 m, 둘 다 가운데가 로봇 오른쪽 0.10 m). 리프트 210 mm 고정.
- 주행: 베이스를 경로대로 옮김 (제자리 회전 → 직진 → 회전) + 정지 오차 (정규분포, 최대 nav_max 로 자름).
- 인식: 시뮬의 실제 물체 위치 → 로봇 기준 (perception_noise 로 오차 추가 가능). 가구도 실제 베이스 자세 기준으로 다시 잼.
- 잡기: 그리퍼를 닫으면 물체가 그 순간의 상대 자세로 그리퍼를 따라가고, 열면 물리로 떨어진다. 들고 있는 물체는
  충돌 모델에 붙여 가구·차체와 검사한다.
- 팔 동작: 계획(IK·경로 비용·기구학 검사) → 5차 곡선을 MuJoCo 위치 제어로 재생 (실기 실행기와 같은 timeline).
  놓은/잡은 뒤에는 같은 길로 되돌아온다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin

from .b601 import MOTORS, B601Kinematics
from .collision import Box, CollisionChecker
from .gravity import conservative_limits
from .ik import B601IK
from .motion import MotionPlanner
from .scene import DESK_TOP, Placement, env_for, shelf_board_top
from .timeline import Timeline

DESK_STAND = Placement(0.0, 0.0, 0.0)
SHELF_STAND = Placement(-3.0, 0.0, math.pi)
DESK_FRONT, SHELF_FRONT, LATERAL = 0.35, 0.40, -0.10
LIFT_STAGE = 0.105                     # 리프트 총 210 mm 고정
BASE_LINK_Z = -0.020
FRUIT_R = 0.03
BASKET = dict(size=(0.20, 0.14, 0.07), wall=0.006, handle_h=0.10)   # 바닥 기준 손잡이 높이
OBJECT_MARGIN = 0.005    # 위치를 아는 물체(과일·바구니) 상자 여유
FINGER_OPEN, FINGER_CLOSED = 0.035, 0.0
# 바구니를 들고 다닐 자세 (로봇 기준 그리퍼 위치 후보 [m]): 0점 자세는 그리퍼가 책상 앞 모서리 위(0.26, −0.22, 0.97)라
# 매달린 바구니(바닥이 그리퍼 10 cm 아래)가 책상 상판에 걸린다 → 더 높고 몸 쪽으로
CARRY_TARGETS = ((0.22, -0.28, 1.12), (0.18, -0.30, 1.15), (0.25, -0.25, 1.10), (0.20, -0.32, 1.20))


SUPPORT_MARGIN = 0.005    # 받침면(집을 물체 아래 책상 상판, 놓을 층의 판) 여유 — 나머지 가구는 2 cm


def stand_to_room(boxes: list[Box], stand: Placement) -> list[Box]:
    return [Box(b.name, b.size, tuple(stand.apply(b.center)), b.yaw + stand.yaw, b.margin) for b in boxes]


def with_support(boxes: list[Box], support: str) -> list[Box]:
    return [Box(b.name, b.size, b.center, b.yaw, SUPPORT_MARGIN if b.name == support else b.margin) for b in boxes]


def room_to_frame(p, stand: Placement):
    c, s = math.cos(stand.yaw), math.sin(stand.yaw)
    dx, dy = p[0] - stand.x, p[1] - stand.y
    return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])


def boxes_to_robot(boxes: list[Box], robot: Placement) -> list[Box]:
    c, s = math.cos(robot.yaw), math.sin(robot.yaw)
    out = []
    for b in boxes:
        dx, dy = b.center[0] - robot.x, b.center[1] - robot.y
        out.append(Box(b.name, b.size, (c * dx + s * dy, -s * dx + c * dy, b.center[2]), b.yaw - robot.yaw, b.margin))
    return out


@dataclass
class StepLog:
    name: str
    ok: bool
    detail: str = ""


@dataclass
class MissionResult:
    success: bool = False
    steps: list[StepLog] = field(default_factory=list)
    nav_errors: list[tuple[float, float, float]] = field(default_factory=list)

    def add(self, name, ok, detail=""):
        self.steps.append(StepLog(name, ok, detail))
        return ok


class MissionSim:
    def __init__(self, robot_xml, seed=0, nav_sigma=(0.02, 0.02, 2.0), nav_max=(0.03, 0.03, 3.0),
                 perception_noise=0.0, speed_deg=30.0, log=print):
        import mujoco
        self.mj = mujoco
        self.rng = np.random.default_rng(seed)
        self.nav_sigma, self.nav_max, self.pnoise, self.log = nav_sigma, nav_max, perception_noise, log
        self.kin = B601Kinematics()
        self.kin.set_motor_limits_deg(conservative_limits(self.kin))
        self.speed = speed_deg
        # 가구 (방 좌표)
        self.desk_boxes = with_support(stand_to_room(env_for("desk", DESK_FRONT, LATERAL), DESK_STAND), "desk_top")
        self.shelf_boxes = with_support(stand_to_room(env_for("shelf3", SHELF_FRONT, LATERAL), SHELF_STAND), "shelf_board3")
        # 물체 처음 자리 (책상 설 자리 기준 → 방)
        self.fruit0 = DESK_STAND.apply((DESK_FRONT + 0.15, LATERAL - 0.12, DESK_TOP + FRUIT_R))
        self.basket0 = DESK_STAND.apply((DESK_FRONT + 0.18, LATERAL + 0.10, DESK_TOP))
        self.held = None            # (이름, 물체 qpos 주소, end_link 기준 자세 pin.SE3)
        self._offset = {}           # 물체별 인식 오차 (_look)
        self._build(robot_xml)

    # ---- MuJoCo 장면 -------------------------------------------------------------------------
    def _build(self, robot_xml):
        mj = self.mj
        spec = mj.MjSpec.from_file(str(robot_xml))
        for n, (lo, hi) in self.kin.motor_limits_deg().items():
            rng = [math.radians(lo), math.radians(hi)]
            spec.joint(f"right_arm_joint{n}").range = rng
            spec.actuator(f"right_arm_joint{n}").ctrlrange = rng
        colors = {"shelf_board": [0.90, 0.88, 0.94, 1], "shelf": [0.09, 0.09, 0.09, 1],
                  "desk_top": [0.82, 0.65, 0.43, 1], "desk": [0.35, 0.30, 0.25, 1]}
        for b in self.desk_boxes + self.shelf_boxes:
            body = spec.worldbody.add_body(name=f"env_{b.name}", pos=list(b.center),
                                           quat=[math.cos(b.yaw / 2), 0, 0, math.sin(b.yaw / 2)])
            key = next((k for k in colors if b.name.startswith(k)), "shelf")
            body.add_geom(type=mj.mjtGeom.mjGEOM_BOX, size=[s / 2 for s in b.size], rgba=colors[key],
                          contype=1, conaffinity=1, friction=[1.0, 0.01, 0.001])
        fruit = spec.worldbody.add_body(name="fruit", pos=list(self.fruit0))
        fruit.add_freejoint(name="fruit_free")
        fruit.add_geom(type=mj.mjtGeom.mjGEOM_SPHERE, size=[FRUIT_R, 0, 0], rgba=[0.95, 0.45, 0.1, 1],
                       mass=0.05, contype=1, conaffinity=1, friction=[1.0, 0.01, 0.001])
        bx, by, bz = BASKET["size"]
        w, hh = BASKET["wall"], BASKET["handle_h"]
        basket = spec.worldbody.add_body(name="basket", pos=list(self.basket0))
        basket.add_freejoint(name="basket_free")
        rgba = [0.75, 0.6, 0.35, 1]
        parts = [((bx / 2, by / 2, w / 2), (0, 0, w / 2)),                                   # 바닥
                 ((w / 2, by / 2, bz / 2), (bx / 2 - w / 2, 0, bz / 2)), ((w / 2, by / 2, bz / 2), (-bx / 2 + w / 2, 0, bz / 2)),
                 ((bx / 2, w / 2, bz / 2), (0, by / 2 - w / 2, bz / 2)), ((bx / 2, w / 2, bz / 2), (0, -by / 2 + w / 2, bz / 2)),
                 ((0.006, by / 2, 0.006), (0, 0, hh)),                                       # 손잡이 (y 방향 막대)
                 ((0.006, 0.006, (hh - bz) / 2), (0, by / 2 - 0.006, (hh + bz) / 2)),
                 ((0.006, 0.006, (hh - bz) / 2), (0, -by / 2 + 0.006, (hh + bz) / 2))]
        for half, pos in parts:
            basket.add_geom(type=mj.mjtGeom.mjGEOM_BOX, size=list(half), pos=list(pos), rgba=rgba,
                            mass=0.02, contype=1, conaffinity=1, friction=[1.0, 0.01, 0.001])
        self.model = spec.compile()
        self.data = mj.MjData(self.model)
        m = self.model
        jadr = lambda n: m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)]   # noqa: E731
        self.base_adr = jadr("base_free")
        self.arm_adr = [jadr(f"right_arm_joint{i}") for i in MOTORS]
        self.lift_adr = [jadr("lift_stage_1_joint"), jadr("lift_stage_2_joint")]
        self.finger_adr = jadr("right_arm_finger_left")
        self.obj_adr = {"fruit": jadr("fruit_free"), "basket": jadr("basket_free")}
        self.act = {mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, a): a for a in range(m.nu)}
        self.end_body = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "right_arm_end_link")
        self.viewer = None
        self.reset()

    def reset(self, seed=None):
        """처음 상태로 (모델은 그대로 — MjSpec 컴파일마다 ~350 MB 가 남으므로 여러 번 돌릴 때 다시 만들지 않는다)."""
        mj, m, d = self.mj, self.model, self.data
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mj.mj_resetDataKeyframe(m, d, 0)
        self.held = None
        self._offset = {}
        d.qpos[self.base_adr:self.base_adr + 3] = [DESK_STAND.x, DESK_STAND.y, BASE_LINK_Z]
        d.qpos[self.base_adr + 3:self.base_adr + 7] = [1, 0, 0, 0]
        for a in self.arm_adr:
            d.qpos[a] = 0.0
        for a in self.lift_adr:
            d.qpos[a] = LIFT_STAGE
        d.qpos[self.finger_adr] = FINGER_OPEN
        # keyframe 'home' 은 로봇 MJCF 기준이라 새로 넣은 물체 자유 관절을 0 으로 덮는다 → 처음 자리로 다시
        for name, pos in (("fruit", self.fruit0), ("basket", self.basket0)):
            a = self.obj_adr[name]
            d.qpos[a:a + 7] = [*pos, 1, 0, 0, 0]
        self.q_cmd = np.zeros(6)
        self.finger_cmd = FINGER_OPEN
        mj.mj_forward(m, d)

    # ---- 상태 -----------------------------------------------------------------------------------
    def base_pose(self) -> Placement:
        x, y = self.data.qpos[self.base_adr:self.base_adr + 2]
        w, _, _, z = self.data.qpos[self.base_adr + 3:self.base_adr + 7]
        return Placement(float(x), float(y), 2 * math.atan2(z, w))

    def arm_deg(self):
        return np.degrees([self.data.qpos[a] for a in self.arm_adr])

    def obj_pos(self, name):
        a = self.obj_adr[name]
        return np.array(self.data.qpos[a:a + 3])

    def _end_pose(self) -> pin.SE3:
        return pin.SE3(self.data.xmat[self.end_body].reshape(3, 3).copy(), self.data.xpos[self.end_body].copy())

    def _obj_pose(self, name) -> pin.SE3:
        a = self.obj_adr[name]
        w, x, y, z = self.data.qpos[a + 3:a + 7]
        return pin.SE3(pin.Quaternion(w, x, y, z).toRotationMatrix(), np.array(self.data.qpos[a:a + 3]))

    # ---- 물리 스텝 -------------------------------------------------------------------------------
    def _step(self, n=1):
        mj, m, d = self.mj, self.model, self.data
        for _ in range(n):
            for i in MOTORS:
                d.ctrl[self.act[f"right_arm_joint{i}"]] = math.radians(self.q_cmd[i - 1])
            d.ctrl[self.act["lift_stage_1_joint"]] = LIFT_STAGE
            d.ctrl[self.act["right_arm_finger_left"]] = self.finger_cmd
            mj.mj_step(m, d)
            if self.held:
                name, adr, rel = self.held
                T = self._held_pose()
                d.qpos[adr:adr + 3] = T.translation
                qx, qy, qz, qw = pin.Quaternion(T.rotation).coeffs()
                d.qpos[adr + 3:adr + 7] = [qw, qx, qy, qz]
                dof = m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, f"{name}_free")]
                d.qvel[dof:dof + 6] = 0.0
        if self.viewer is not None:
            self.viewer.sync()

    def settle(self, seconds=0.5):
        self._step(int(seconds / self.model.opt.timestep))

    # ---- 주행 (Nav2 흉내) ------------------------------------------------------------------------
    def drive_to(self, stand: Placement, res: MissionResult, v=0.5, w=math.radians(60)):
        mj, m, d = self.mj, self.model, self.data
        start = self.base_pose()
        err = [float(np.clip(self.rng.normal(0, sg), -mx, mx)) for sg, mx in zip(self.nav_sigma, self.nav_max)]
        goal = Placement(stand.x + err[0] * math.cos(stand.yaw) - err[1] * math.sin(stand.yaw),
                         stand.y + err[0] * math.sin(stand.yaw) + err[1] * math.cos(stand.yaw),
                         stand.yaw + math.radians(err[2]))
        dist = math.hypot(goal.x - start.x, goal.y - start.y)
        heading = math.atan2(goal.y - start.y, goal.x - start.x) if dist > 0.05 else start.yaw
        segs = []      # (x, y, yaw) 키프레임
        segs.append((start.x, start.y, start.yaw))
        segs.append((start.x, start.y, heading))
        segs.append((goal.x, goal.y, heading))
        segs.append((goal.x, goal.y, goal.yaw))
        dt = m.opt.timestep
        for (x0, y0, a0), (x1, y1, a1) in zip(segs, segs[1:]):
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            T = max(math.hypot(x1 - x0, y1 - y0) / v, abs(da) / w, 0.05)
            for k in range(int(T / dt)):
                u = (k + 1) * dt / T
                yaw = a0 + da * u
                d.qpos[self.base_adr:self.base_adr + 3] = [x0 + (x1 - x0) * u, y0 + (y1 - y0) * u, BASE_LINK_Z]
                d.qpos[self.base_adr + 3:self.base_adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
                d.qvel[m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "base_free")]:][:6] = 0
                self._step()
        self.settle(0.5)
        res.nav_errors.append((err[0], err[1], err[2]))
        return res.add(f"주행 → ({stand.x:.1f}, {stand.y:.1f}, {math.degrees(stand.yaw):.0f}°)", True,
                       f"정지 오차 앞뒤 {100 * err[0]:+.1f} cm, 좌우 {100 * err[1]:+.1f} cm, 방향 {err[2]:+.1f}°")

    # ---- 인식 + 계획 + 실행 -----------------------------------------------------------------------
    def _perceive(self, p_room):
        """방 좌표 → 지금 (실제) 베이스 기준. 인식 오차는 물체를 볼 때 (_look) 넣는다."""
        return room_to_frame(np.asarray(p_room), self.base_pose())

    def _look(self):
        """물체를 다시 봄: 물체마다 수평 인식 오차 하나씩 (높이는 놓인 면에서 정하므로 오차 없음).
        잡을 곳·놓을 곳·장애물 상자 모두 이 '본' 위치를 쓴다. 성공 판정은 실제 위치."""
        self._offset = {n: np.array([*self.rng.normal(0, self.pnoise, 2), 0.0]) if self.pnoise > 0 else np.zeros(3)
                        for n in self.obj_adr}

    def seen_pos(self, name):
        return self.obj_pos(name) + self._offset.get(name, 0.0)

    def _checker(self, furniture: list[Box], obstacles: list[Box]):
        robot = self.base_pose()
        attached = []
        if self.held:
            name, adr, rel = self.held
            if name == "fruit":                     # 손가락 사이에 단단히
                attached = [(name, (FRUIT_R * 2,) * 3, rel)]
            else:                                   # 손잡이로 매달림: 시뮬과 같은 모델 (위치만 그리퍼, 똑바로, 로봇과 같이 돎)
                grip_local, yaw0, robot_yaw0 = self._hang
                R = pin.rpy.rpyToMatrix(0, 0, yaw0 - robot_yaw0)          # 로봇 기준 바구니 방향
                # 바구니 원점 = 바닥 가운데 → 상자 가운데는 위로 손잡이 높이의 절반
                box = pin.SE3(R, -R @ grip_local) * pin.SE3(np.eye(3), np.array([0, 0, BASKET["handle_h"] / 2]))
                attached = [(name, (BASKET["size"][0], BASKET["size"][1], BASKET["handle_h"]), box, "hang")]
        return CollisionChecker(self.kin, boxes_to_robot(list(furniture) + list(obstacles), robot), margin=0.02, attached=attached)

    def _object_boxes(self, name) -> list[Box]:
        """계획용 장애물 상자들 (본 위치, 실제 크기, 여유 OBJECT_MARGIN).
        바구니 = 테두리 높이까지의 몸통 + 손잡이 막대 (손잡이를 피해 테두리 위로 과일을 놓을 수 있게)."""
        p = self.seen_pos(name)
        if name == "fruit":
            return [Box("obj_fruit", (2 * FRUIT_R,) * 3, tuple(p), 0.0, OBJECT_MARGIN)]
        bx, by, bz = BASKET["size"]
        hh, yaw = BASKET["handle_h"], self._obj_yaw(name)
        return [Box("obj_basket", (bx, by, bz), (p[0], p[1], p[2] + bz / 2), yaw, OBJECT_MARGIN),
                Box("obj_basket_handle", (0.012, by, 0.012), (p[0], p[1], p[2] + hh), yaw, OBJECT_MARGIN)]

    def _obj_yaw(self, name):
        a = self.obj_adr[name]
        w, x, y, z = self.data.qpos[a + 3:a + 7]
        return 2 * math.atan2(z, w)

    def plan_move(self, target_room, furniture, obstacles, label, res, tilt=(-10.0, 60.0)):
        robot = self.base_pose()
        target = self._perceive(target_room)
        into = np.array([1.0, 0.0, 0.0])                       # 가구는 로봇 정면 (ZED 방향)
        checker = self._checker(furniture, obstacles)
        ik = B601IK(self.kin, collision=checker)
        start = dict(zip(MOTORS, self.arm_deg()))
        cands = ik.solve(target, [LIFT_STAGE], heading=into, tilt_deg=tilt, n_random=24, seed=start)
        plan = MotionPlanner(ik, max_joint_speed_deg=self.speed).plan(start, target, cands, heading=into,
                                                                      start_lift_stage_m=LIFT_STAGE)
        if plan is None or plan.candidate is None:
            why = plan.tried[-1] if plan is not None and plan.tried else "IK 해 없음"
            res.add(label, False, f"계획 실패: {why}")
            return None
        wps = [{"t": w.t, "phase": w.phase, "motors_deg": {str(k): v for k, v in w.motors_deg.items()}}
               for w in plan.waypoints]
        tl = Timeline(wps, v_limit_deg=self.speed)
        self._last_plan = plan
        self._play(tl)
        end = self._end_pose()
        tool_room = end.translation
        target_room_now = np.asarray(target_room)
        err = 1000 * float(np.linalg.norm(tool_room - target_room_now))
        robot_now = self.base_pose()
        res.add(label, True, f"cost {plan.total_cost:.2f}, 끝 오차 {err:.1f} mm, 접근 {100 * plan.standoff:.0f} cm, "
                             f"베이스 ({robot.x:+.3f},{robot.y:+.3f},{math.degrees(robot.yaw):+.1f}°)"
                             f"→({robot_now.x:+.3f},{robot_now.y:+.3f})")
        return tl

    def _play(self, tl: Timeline):
        dt = self.model.opt.timestep
        for k in range(int(tl.duration / dt) + 1):
            self.q_cmd = tl.sample(k * dt)[0]
            self._step()
        self.settle(0.4)

    def retract(self, tl: Timeline, label, res, furniture=None, obstacles=(), lift_off=0.03, carry=False):
        """팔 접기. 들고 있지 않으면 온 길 그대로 되돌아감.
        들고 있으면: 그 자리에서 lift_off 만큼 곧게 들어 올림 → 삽입 구간을 그만큼 높여 다시 푼 길로 빠짐
        → 접근 구간(경유점)을 거꾸로 접기. 이 길 전체를 든 물체 포함 충돌 검사하고, 부딪히면 실패."""
        if not self.held or furniture is None:
            self._play(tl.reversed())
            return res.add(label, True)
        out, why = self._retreat_path(self._last_plan, lift_off, furniture, obstacles, carry)
        if out is None:
            return res.add(label, False, why)
        self._play(out)
        return res.add(label, True, f"{100 * lift_off:.0f} cm 들고 빠짐" + (f", 들고 다닐 자세 {why}" if carry else "")
                       + ", 든 물체 포함 충돌 없음")

    def _path_clear(self, checker, qa, qb, step_deg=1.0, q_end=None):
        """qa→qb 관절 보간이 충돌 없나 (도 단위 dict). q_end 근처(접힌 자세)는 팔 링크끼리 무시."""
        a = np.array([qa[k] for k in MOTORS]); b = np.array([qb[k] for k in MOTORS])
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / step_deg)))
        for k in range(n + 1):
            q = a + (b - a) * k / n
            near = q_end is not None and float(np.max(np.abs(q - q_end))) < MotionPlanner.START_SELF_FREE_DEG
            if checker.in_collision(list(q), LIFT_STAGE, ignore_self=near):
                return False
        return True

    def _carry_pose(self, checker, q_from):
        """들고 다닐 자세: CARRY_TARGETS 에서 IK (든 물체 포함 충돌 검사) → q_from 에서 곧장 가도 충돌 없는 첫 해."""
        ik = B601IK(self.kin, collision=checker)
        for tgt in CARRY_TARGETS:
            sols = [c for c in ik.solve(np.array(tgt), LIFT_STAGE, heading=np.array([1.0, 0, 0]), tilt_deg=(-10, 60),
                                        seed=q_from, n_random=12) if c.ok]
            sols.sort(key=lambda c: max(abs(c.motors_deg[k] - q_from[k]) for k in MOTORS))
            for c in sols[:4]:
                if self._path_clear(checker, q_from, c.motors_deg):
                    return dict(c.motors_deg), tgt
        return None, None

    def _retreat_path(self, plan, lift_off, furniture, obstacles, carry=False):
        kin, ik = self.kin, B601IK(self.kin)
        checker = self._checker(furniture, obstacles)
        w = plan.waypoints
        first_ins = next(i for i, x in enumerate(w) if x.phase == "insert")
        out_pts = []                    # 목표 → 접근점 순으로, 높인 자세
        prev = w[-1].motors_deg
        for x in reversed(w[first_ins - 1:]):
            T = kin.fk(x.motors_deg, LIFT_STAGE)
            sol = ik.solve(T.translation + [0, 0, lift_off], LIFT_STAGE, seed=prev, n_random=0, rotation=T.rotation)
            if not sol or not sol[0].converged:
                return None, "빠지는 길 IK 실패"
            q = sol[0].motors_deg
            if max(abs(q[k] - prev[k]) for k in MOTORS) > 10:
                return None, "빠지는 길 관절 튐"
            out_pts.append(dict(q))
            prev = q
        folds = [x.motors_deg for x in reversed(w[1:first_ins - 1]) if x.phase == "approach"] + [w[0].motors_deg]
        carry_note = ""
        if carry:
            q_carry, tgt = self._carry_pose(checker, out_pts[-1])
            if q_carry is None:
                return None, "들고 다닐 자세를 못 찾음 (든 물체 포함 충돌)"
            folds, carry_note = [q_carry], f"({', '.join(f'{v:.2f}' for v in tgt)}) m"
        seq = [("insert", w[-1].motors_deg)] + [("insert", q) for q in out_pts] + [("approach", q) for q in folds]
        wps, t, prev_q, prev_p = [], 0.0, None, None
        for phase, q in seq:
            p = kin.fk(q, LIFT_STAGE).translation
            if prev_q is not None:
                dq = max(abs(q[k] - prev_q[k]) for k in MOTORS)
                t += max(dq / self.speed, float(np.linalg.norm(p - prev_p)) / 0.03 if phase == "insert" else 0.5)
            wps.append({"t": t, "phase": phase, "motors_deg": {str(k): float(v) for k, v in q.items()}})
            prev_q, prev_p = q, p
        tl = Timeline(wps, v_limit_deg=self.speed)
        q_end = tl.sample(tl.duration)[0] if not carry else None
        t_lifted = tl.starts[1]           # 첫 구간 = 받침면에서 곧게 들어 올림 (받침면에 닿아 있는 게 당연) → 그 뒤부터 검사
        for t in np.append(np.arange(t_lifted, tl.duration, 0.05), tl.duration):
            q = tl.sample(t)[0]
            near = q_end is not None and float(np.max(np.abs(q - q_end))) < MotionPlanner.START_SELF_FREE_DEG
            if checker.in_collision(list(q), LIFT_STAGE, ignore_self=near):
                pairs = checker.colliding_pairs(list(q), LIFT_STAGE, ignore_self=near)
                return None, f"빠지는 길 t={t:.1f}s 충돌 {pairs[:2]}"
        return tl, carry_note

    def grip(self, name, res):
        self.finger_cmd = FINGER_CLOSED
        self.settle(0.3)
        dist = float(np.linalg.norm(self._end_pose().translation - self._grasp_point(name, seen=False)))
        if dist > 0.04:
            return res.add(f"잡기 {name}", False, f"그리퍼가 잡을 곳에서 {100 * dist:.1f} cm 떨어짐")
        self.held = (name, self.obj_adr[name], self._end_pose().actInv(self._obj_pose(name)))
        if name == "basket":
            # 손잡이를 잡은 바구니는 매달려 똑바로 선다: 손잡이 점만 그리퍼를 따라가고, 방향은 로봇과 같이 돈다
            obj = self._obj_pose(name)
            self._hang = (obj.actInv(self._end_pose().translation), self._obj_yaw(name), self.base_pose().yaw)
        return res.add(f"잡기 {name}", True, f"그리퍼 ↔ 잡을 곳 {1000 * dist:.0f} mm")

    def _held_pose(self) -> pin.SE3:
        name, adr, rel = self.held
        if name != "basket":
            return self._end_pose() * rel
        grip_local, yaw0, robot_yaw0 = self._hang
        yaw = yaw0 + (self.base_pose().yaw - robot_yaw0)
        R = pin.rpy.rpyToMatrix(0, 0, yaw)
        return pin.SE3(R, self._end_pose().translation - R @ grip_local)

    def release(self, res):
        name = self.held[0] if self.held else "?"
        self.held = None
        self.finger_cmd = FINGER_OPEN
        self.settle(1.0)
        return res.add(f"놓기 {name}", True)

    def _grasp_point(self, name, seen=True):
        p = self.seen_pos(name) if seen else self.obj_pos(name)
        return p if name == "fruit" else p + np.array([0, 0, BASKET["handle_h"]])

    # ---- 임무 -----------------------------------------------------------------------------------
    def run(self) -> MissionResult:
        res = MissionResult()
        self.drive_to(DESK_STAND, res)
        # 1) 과일 집기 (장애물: 책상 + 바구니)
        self._look()
        tl = self.plan_move(self._grasp_point("fruit"), self.desk_boxes, self._object_boxes("basket"), "과일로 팔 뻗기", res)
        if tl is None or not self.grip("fruit", res):
            return res
        if not self.retract(tl, "과일 들고 팔 접기", res, self.desk_boxes, self._object_boxes("basket")):
            return res
        # 2) 바구니 위에 놓기: 손잡이를 피해 바구니 앞쪽(로봇 쪽) 5 cm, 테두리 위 4 cm
        self._look()
        bp = self.seen_pos("basket")
        near = self._toward_robot(bp, 0.05)
        drop = near + np.array([0, 0, BASKET["size"][2] + 0.04 + FRUIT_R])
        tl = self.plan_move(drop, self.desk_boxes, self._object_boxes("basket"), "바구니 위로 팔 뻗기", res)
        if tl is None:
            return res
        self.release(res)
        self.retract(tl, "팔 접기", res)
        inside, rel = self._fruit_in_basket()
        if not res.add("과일이 바구니 안", inside, f"바구니 기준 ({100 * rel[0]:+.1f}, {100 * rel[1]:+.1f}, {100 * rel[2]:+.1f}) cm"):
            return res
        # 3) 바구니 손잡이 잡기 (장애물: 책상)
        self._look()
        tl = self.plan_move(self._grasp_point("basket"), self.desk_boxes, [], "바구니 손잡이로 팔 뻗기", res)
        if tl is None or not self.grip("basket", res):
            return res
        if not self.retract(tl, "바구니 들고 다닐 자세로", res, self.desk_boxes, carry=True):
            return res
        # 4) 선반으로 주행 → 3층에 두기 (바구니 바닥이 판 위 1.5 cm 가 되게 손잡이 목표)
        self.drive_to(SHELF_STAND, res)
        place = SHELF_STAND.apply((SHELF_FRONT + 0.12, LATERAL, shelf_board_top(3) + 0.015 + BASKET["handle_h"]))
        tl = self.plan_move(place, self.shelf_boxes, [], "선반 3층으로 바구니 넣기", res)
        if tl is None:
            return res
        self.release(res)
        self.retract(tl, "팔 빼기", res)
        bp = self.obj_pos("basket")
        local = room_to_frame(bp, SHELF_STAND)
        on_board = (abs(bp[2] - shelf_board_top(3)) < 0.02
                    and SHELF_FRONT < local[0] < SHELF_FRONT + 0.33 and abs(local[1] - LATERAL) < 0.27)
        res.add("바구니가 선반 3층 위", on_board, f"바구니 바닥 z {bp[2]:.3f} m (판 {shelf_board_top(3):.3f}), "
                                               f"선반 기준 ({local[0]:.2f}, {local[1]:+.2f}) m")
        still, rel = self._fruit_in_basket()
        res.add("과일이 여전히 바구니 안", still, f"바구니 기준 ({100 * rel[0]:+.1f}, {100 * rel[1]:+.1f}, {100 * rel[2]:+.1f}) cm")
        res.success = all(s.ok for s in res.steps)
        return res

    def _fruit_in_basket(self):
        """과일 중심이 바구니 안쪽 (바닥 위, 테두리 아래, 벽 안) 인지와 바구니 기준 위치."""
        fp, bp = self.obj_pos("fruit"), self.obj_pos("basket")
        rel = room_to_frame(fp, Placement(bp[0], bp[1], self._obj_yaw("basket")))
        rel[2] = fp[2] - bp[2]
        bx, by, bz = BASKET["size"]
        inside = abs(rel[0]) < bx / 2 and abs(rel[1]) < by / 2 and 0.0 < rel[2] < bz
        return inside, rel

    def _toward_robot(self, p, d):
        r = self.base_pose()
        v = np.array([r.x - p[0], r.y - p[1], 0.0])
        v /= max(np.linalg.norm(v), 1e-9)
        return np.asarray(p) + d * v
