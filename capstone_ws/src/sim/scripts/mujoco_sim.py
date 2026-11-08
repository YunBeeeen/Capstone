#!/usr/bin/env python3
"""MuJoCo 시뮬 — 실기의 조향 브리지 + wheel_can_bridge(+ XDrive/RobStride) 자리를 대신한다.

  sub  steer_cmd    capstone_msgs/SteerCommand     → 조향 4축 (RobStride 02 흉내: PD 위치제어, ±17 Nm)
  sub  wheel_cmd    capstone_msgs/WheelCommand     → 구동 4축 (ODrive 흉내: 속도 램프 2 turn/s² → 속도 PI, ±6 Nm, DRIVE 끊기면 HOLD)
  pub  steer_state  capstone_msgs/SteerStateArray  (ready = true, 실제 조향각)
  pub  wheel_state  capstone_msgs/WheelStateArray  (phase, 바퀴별 위치/속도/Iq)
  pub  joint_states sensor_msgs/JointState         (전 관절 → robot_state_publisher / RViz)
  pub  sim/ground_truth  nav_msgs/Odometry (base_link) + TF odom → world (참값)

그래서 swerve_controller 노드는 실기와 **같은 바이너리·같은 설정**으로 이 시뮬에 붙는다.

모델: mobile_robot_right_arm_description/mjcf/mobile_robot_right_arm.xml (미리 생성, 켤 때 변환 없음)
  → URDF 를 고쳤으면 `python3 src/sim/scripts/build_mjcf.py` 로 다시 생성. 모델 쪽 가정은 그 파일 docstring 참고.
  다른 MJCF 를 쓰려면 파라미터 mjcf:=<경로>.
URDF 의 'world' 링크 = 로봇 바닥 기준점(base_footprint 역할) → TF 는 odom → world 로 발행.
모터 게인은 **시뮬 전용 임의값** (식별 안 함).
"""
import math
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from capstone_msgs.msg import SteerCommand, SteerStateArray, WheelCommand, WheelState, WheelStateArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# 순서 [1 FR, 2 RR, 3 RL, 4 FL] → URDF 이름
MODULES = ["fr", "rr", "rl", "fl"]
KT = 0.2  # [Nm/A] BL41105 — Iq 표시용
WORLD_Z = 0.020  # URDF world_to_base: base_link 가 world(바닥 기준점) 보다 20 mm 아래


def default_mjcf():
    share = Path(get_package_share_directory("mobile_robot_right_arm_description"))
    return str(share / "mjcf" / "mobile_robot_right_arm.xml")


class SimNode(Node):
    def __init__(self):
        super().__init__("mujoco_sim")
        self.headless = self.declare_parameter("headless", False).value
        self.pub_rate = self.declare_parameter("publish_rate_hz", 50.0).value
        self.steer_kp = self.declare_parameter("steer_kp", 40.0).value         # [Nm/rad]
        self.steer_kd = self.declare_parameter("steer_kd", 1.5).value          # [Nm·s/rad]
        self.drive_kp = self.declare_parameter("drive_kp", 0.6).value          # [Nm/(rad/s)]
        self.drive_ki = self.declare_parameter("drive_ki", 6.0).value          # [Nm/rad] — HOLD 때 위치를 붙잡는 항
        self.vel_max = self.declare_parameter("vel_max", 2.0).value            # [turn/s] 보드 vel_limit
        self.cmd_timeout = self.declare_parameter("wheel_cmd_timeout", 0.2).value  # 브리지와 같음
        # 보드 input_mode = VEL_RAMP, vel_ramp_rate = 2 turn/s² (bringup.py 로 저장된 값) — 속도 목표가 이 기울기로만 바뀜.
        # 0 이면 램프 없음 (목표로 즉시 → 최대 토크로 급정지 → 키 큰 차체가 넘어짐)
        self.vel_ramp = self.declare_parameter("vel_ramp_rate", 2.0).value      # [turn/s²]

        path = self.declare_parameter("mjcf", "").value or default_mjcf()
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        try:
            mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key("home").id)  # 팔 기본 자세
        except KeyError:
            pass
        mujoco.mj_forward(self.m, self.d)
        J = lambda name: self.m.joint(name)
        A = lambda name: self.m.actuator(name).id
        self.steer_q = [J(f"steering_{n}_joint").qposadr[0] for n in MODULES]
        self.steer_v = [J(f"steering_{n}_joint").dofadr[0] for n in MODULES]
        self.drive_q = [J(f"drive_{n}_joint").qposadr[0] for n in MODULES]
        self.drive_v = [J(f"drive_{n}_joint").dofadr[0] for n in MODULES]
        self.steer_a = [A(f"steering_{n}_joint") for n in MODULES]
        self.drive_a = [A(f"drive_{n}_joint") for n in MODULES]
        self.base_id = self.m.body("base_link").id
        self.js_names, self.js_q, self.js_v = [], [], []
        for j in range(self.m.njnt):
            if self.m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                self.js_names.append(self.m.joint(j).name)
                self.js_q.append(self.m.jnt_qposadr[j])
                self.js_v.append(self.m.jnt_dofadr[j])

        self.lock = threading.Lock()
        self.steer_mode = SteerCommand.MODE_FREE
        self.steer_tgt = np.zeros(4)
        self.wheel_mode = WheelCommand.MODE_FREE
        self.wheel_tgt = np.zeros(4)  # [turn/s]
        self.wheel_cmd_time = -1e9
        self.integ = np.zeros(4)
        self.tau_drive = np.zeros(4)
        self.vel_sp = np.zeros(4)  # 램프 지난 속도 목표 [turn/s] (ODrive vel_setpoint)

        self.create_subscription(SteerCommand, "steer_cmd", self.on_steer, 10)
        self.create_subscription(WheelCommand, "wheel_cmd", self.on_wheel, 10)
        self.steer_pub = self.create_publisher(SteerStateArray, "steer_state", 10)
        self.wheel_pub = self.create_publisher(WheelStateArray, "wheel_state", 10)
        self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "sim/ground_truth", 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info(f"모델 {path}\n준비: bodies={self.m.nbody} joints={self.m.njnt} actuators={self.m.nu}. "
                               "조향/구동은 FREE 로 시작 — swerve_controller 에 cmd_vel 을 주면 움직임")

    # ------------------------------------------------------------ 명령 수신
    def on_steer(self, msg):
        with self.lock:
            if msg.mode == SteerCommand.MODE_POSITION and len(msg.angle) == 4:
                self.steer_mode = msg.mode
                self.steer_tgt[:] = msg.angle
            else:
                self.steer_mode = SteerCommand.MODE_FREE

    def on_wheel(self, msg):
        with self.lock:
            if msg.mode == WheelCommand.MODE_DRIVE:
                if len(msg.velocity) != 4 or not all(math.isfinite(v) for v in msg.velocity):
                    self.get_logger().warn("wheel_cmd DRIVE 길이/값 이상 — 무시", throttle_duration_sec=2.0)
                    return
                self.wheel_tgt[:] = np.clip(msg.velocity, -self.vel_max, self.vel_max)
            self.wheel_mode = msg.mode
            self.wheel_cmd_time = time.monotonic()

    # ------------------------------------------------------------ 모터 흉내 (매 물리 스텝)
    def control(self):
        m, d, dt = self.m, self.d, self.m.opt.timestep
        with self.lock:
            steer_mode, steer_tgt = self.steer_mode, self.steer_tgt.copy()
            mode, tgt = self.wheel_mode, self.wheel_tgt.copy()
            stale = time.monotonic() - self.wheel_cmd_time > self.cmd_timeout
        if mode == WheelCommand.MODE_DRIVE and stale:
            mode = WheelCommand.MODE_HOLD  # 브리지와 같음: DRIVE 끊기면 속도 0
        self.eff_mode = mode
        for i in range(4):
            # 조향 (RobStride 위치 모드 흉내)
            if steer_mode == SteerCommand.MODE_POSITION:
                tau = self.steer_kp * (steer_tgt[i] - d.qpos[self.steer_q[i]]) - self.steer_kd * d.qvel[self.steer_v[i]]
            else:
                tau = 0.0
            d.ctrl[self.steer_a[i]] = tau
            # 구동 (ODrive 속도 PI 흉내). + = 바퀴 기준 (조향 0° 에서 전진)
            if mode == WheelCommand.MODE_FREE:
                self.integ[i] = 0.0
                self.vel_sp[i] = 0.0
                tau = 0.0
            else:
                goal = tgt[i] if mode == WheelCommand.MODE_DRIVE else 0.0  # HOLD 도 램프로 감속
                if self.vel_ramp > 0:
                    step = self.vel_ramp * dt
                    self.vel_sp[i] += float(np.clip(goal - self.vel_sp[i], -step, step))
                else:
                    self.vel_sp[i] = goal
                err = 2 * math.pi * self.vel_sp[i] - d.qvel[self.drive_v[i]]
                self.integ[i] = float(np.clip(self.integ[i] + err * dt, -6.0 / self.drive_ki, 6.0 / self.drive_ki))
                tau = self.drive_kp * err + self.drive_ki * self.integ[i]
            d.ctrl[self.drive_a[i]] = tau
            self.tau_drive[i] = float(np.clip(tau, -6.0, 6.0))

    # ------------------------------------------------------------ 상태 발행
    def publish(self):
        d = self.d
        stamp = self.get_clock().now().to_msg()
        ss = SteerStateArray()
        ss.header.stamp = stamp
        ss.ready = True
        ss.angle = [float(d.qpos[q]) for q in self.steer_q]
        self.steer_pub.publish(ss)

        ws = WheelStateArray()
        ws.header.stamp = stamp
        active = self.eff_mode != WheelCommand.MODE_FREE
        ws.phase = WheelStateArray.PHASE_ACTIVE if active else WheelStateArray.PHASE_FREE
        ws.ready, ws.vbus = True, 25.9
        for i in range(4):
            w = WheelState()
            w.can_id, w.alive = i + 1, True
            w.axis_state = 8 if active else 1
            w.position = float(d.qpos[self.drive_q[i]] / (2 * math.pi))
            w.velocity = float(d.qvel[self.drive_v[i]] / (2 * math.pi))
            w.iq_setpoint = w.iq_measured = float(self.tau_drive[i] / KT)
            ws.wheels.append(w)
        self.wheel_pub.publish(ws)

        js = JointState()
        js.header.stamp = stamp
        js.name = self.js_names
        js.position = [float(d.qpos[q]) for q in self.js_q]
        js.velocity = [float(d.qvel[v]) for v in self.js_v]
        self.js_pub.publish(js)

        pos = d.xpos[self.base_id]
        qw, qx, qy, qz = d.xquat[self.base_id]
        R = d.xmat[self.base_id].reshape(3, 3)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, pos)
        o = od.pose.pose.orientation
        o.w, o.x, o.y, o.z = float(qw), float(qx), float(qy), float(qz)
        v_body = R.T @ d.qvel[0:3]  # freejoint: 선속도는 world, 각속도는 body 기준
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, v_body)
        od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = map(float, d.qvel[3:6])
        self.odom_pub.publish(od)

        t = TransformStamped()
        t.header = od.header
        t.child_frame_id = "world"  # URDF 루트 (= 바닥 기준점). robot_state_publisher 가 world→base_link 를 냄
        wpos = pos + R @ np.array([0.0, 0.0, WORLD_Z])
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, wpos)
        t.transform.rotation = o
        self.tf.sendTransform(t)

    # ------------------------------------------------------------ 실시간 루프
    def run(self):
        m, d = self.m, self.d
        self.eff_mode = WheelCommand.MODE_FREE
        pub_dt = 1.0 / self.pub_rate
        viewer = None
        if not self.headless:
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(m, d)
            viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 2.2, -25, 135
            viewer.cam.trackbodyid = self.base_id
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        t0, next_pub, next_sync = time.monotonic(), 0.0, 0.0
        try:
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                wall = time.monotonic() - t0
                if d.time > wall + 0.1:  # 앞서 있으면 쉼
                    time.sleep(0.001)
                    continue
                if wall - d.time > 0.5:  # 렌더 등으로 많이 밀리면 따라잡기 포기 (실시간 기준 유지)
                    t0 += wall - d.time
                with (viewer.lock() if viewer else _nullctx()):
                    self.control()
                    mujoco.mj_step(m, d)
                if d.time >= next_pub:
                    next_pub = d.time + pub_dt
                    self.publish()
                if viewer and wall >= next_sync:
                    next_sync = wall + 1 / 60
                    viewer.sync()
        finally:
            if viewer:
                viewer.close()


class _nullctx:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def main():
    rclpy.init()
    node = SimNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    def spin():
        try:
            ex.spin()
        except ExternalShutdownException:  # Ctrl+C / launch 종료
            pass

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()      # spin 스레드를 깨움
        th.join(timeout=2.0)      # 스레드가 끝난 뒤에 노드 정리 (안 그러면 종료 때 abort)
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
