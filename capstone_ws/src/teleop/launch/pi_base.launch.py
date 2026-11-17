"""라파 주행 한 번에 = 조향 + 구동 + 스워브 + 패드 UDP 수신 + 리프트 (예전 터미널 1~5). 게임패드 (pad_pi.sh) 는 따로 켬.
  sudo ~/capstone/capstone_ws/can/can_up.sh                        # CAN 먼저 (부팅마다 한 번)
  source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
  ros2 launch ~/pad/launch/pi_base.launch.py
  ros2 launch ~/pad/launch/pi_base.launch.py lift:=false            # 리프트 빼고 (udp:=false 도 있음)
켤 때 순서: 조향 → 1 초 뒤 구동 → 2 초 뒤 스워브 → 3 초 뒤 UDP 수신·리프트. CAN (can_steer, can_drive) 이 안 올라와 있으면 안 켜고 알려 줌.
끄기: **Ctrl+C 한 번** → 리프트가 최하단까지 내려간 뒤 꺼짐 (리프트만 최대 60 초 기다려 줌). 끝날 때까지 기다릴 것.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAN = ("can_steer", "can_drive")


def can_up(name):
    try:
        return int(Path(f"/sys/class/net/{name}/flags").read_text(), 16) & 0x1 == 1     # IFF_UP
    except (OSError, ValueError):
        return False


def start(context):
    down = [c for c in CAN if not can_up(c)]
    if down:
        return [LogInfo(msg=f"CAN {', '.join(down)} 이 안 올라옴 → 먼저: sudo ~/capstone/capstone_ws/can/can_up.sh"),
                Shutdown(reason="CAN down")]
    def inc(pkg, **args):
        # 따로 감쌈: 조향·구동 launch 가 같은 이름 인자 (can_interface) 를 다른 기본값으로 씀 → 섞이지 않게 + 직접 넘김
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory(pkg)) / "launch" / f"{pkg}.launch.py")),
            launch_arguments=list(args.items()))])
    return [
        inc("steer_can_bridge", can_interface="can_steer"),
        TimerAction(period=1.0, actions=[inc("wheel_can_bridge", can_interface="can_drive")]),
        TimerAction(period=2.0, actions=[inc("swerve_controller")]),
        TimerAction(period=3.0, condition=IfCondition(LaunchConfiguration("udp")), actions=[
            Node(package="capstone_teleop", executable="udp_teleop_bridge.py", name="udp_teleop_bridge", output="screen")]),
        TimerAction(period=3.0, condition=IfCondition(LaunchConfiguration("lift")), actions=[
            Node(package="lift_bridge", executable="lift_bridge_node.py", name="lift_bridge", output="screen",
                 sigterm_timeout="60")]),                     # Ctrl+C 때 최하단까지 내려가는 동안 강제 종료 안 되게
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("udp", default_value="true", description="패드 UDP 수신 (udp_teleop_bridge)"),
        DeclareLaunchArgument("lift", default_value="true", description="리프트 (lift_bridge_node)"),
        OpaqueFunction(function=start),
    ])
