#!/usr/bin/env python3
"""게임패드 teleop: /joy (sensor_msgs/Joy) → /cmd_vel (geometry_msgs/Twist). teleop_twist_joy 대신.

teleop_twist_joy 와 같은 동작 + 원형 데드존 (gamepad_map.py):
  - deadman(enable_button) 누르는 동안만 /joy 받을 때마다 발행 (joy_node autorepeat 20 Hz → swerve cmd_timeout 안 걸림)
  - deadman 놓으면 0 을 한 번 보내고 멈춤 → swerve 가 cmd_timeout 뒤 HOLD
  - /joy 가 끊기면 (USB 분리 등) 발행도 멈춤 → 같은 경로로 HOLD

  ros2 launch capstone_teleop teleop.launch.py
"""
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gamepad_map import joy_to_cmd  # noqa: E402

PARAMS = {
    "require_enable_button": True,
    "enable_button": 5,
    "axis_linear_x": 1,
    "axis_linear_y": 0,
    "axis_angular": 3,
    "scale_linear_x": 0.25,
    "scale_linear_y": 0.25,
    "scale_angular": 0.5,
    "deadzone": 0.15,
    "deadzone_angular": 0.15,
}


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_teleop")
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.was_enabled = False
        c = self.cfg
        self.get_logger().info(
            f"deadman button {c['enable_button']}  vx=axes[{c['axis_linear_x']}]×{c['scale_linear_x']} "
            f"vy=axes[{c['axis_linear_y']}]×{c['scale_linear_y']} ω=axes[{c['axis_angular']}]×{c['scale_angular']}  "
            f"원형 데드존 {c['deadzone']}")

    def on_joy(self, msg: Joy):
        enabled, vx, vy, w = joy_to_cmd(msg.axes, msg.buttons, self.cfg)
        if enabled or self.was_enabled:  # 놓는 순간 0 한 번
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, w
            self.pub.publish(t)
        self.was_enabled = enabled


def main():
    rclpy.init()
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # 종료 시 0
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
