"""jetson_map / jetson_nav launch 가 같이 쓰는 것 (launch 파일 아님)."""
import os
from pathlib import Path

from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def nav_tools():
    """navigation/tools 폴더 (설치 안 되는 도구들): $NAV → Jetson ~/capstone_ws → 노트북 ~/capstone/capstone_ws."""
    for c in (os.environ.get("NAV", ""), "~/capstone_ws/src/navigation/tools", "~/capstone/capstone_ws/src/navigation/tools"):
        p = Path(c).expanduser()
        if c and (p / "zed_floor_level.py").exists():
            return p
    return Path("~/capstone_ws/src/navigation/tools").expanduser()


def floor_level(delay=20.0, tries=6):
    """ZED·zed_base_tf 가 뜬 뒤 바닥 보정 (zed_floor_level.py --apply) 을 자동으로. 못 재면 10 초 뒤 다시, tries 번.
    floor:=false 면 안 함. 성공·실패를 [바닥 보정] 줄로 알려 줌 (실패해도 다른 것은 계속 돔)."""
    t = nav_tools() / "zed_floor_level.py"
    sh = (f'for i in $(seq 1 {tries}); do '
          f'python3 {t} --apply && {{ echo "[바닥 보정] 끝"; exit 0; }}; '
          f'echo "[바닥 보정] 다시 ($i/{tries}, 10 초 뒤) — 로봇 앞 2 m 이상 빈 바닥이 보이게"; sleep 10; done; '
          f'echo "[바닥 보정] 실패 — 앞이 빈 곳에서 직접: python3 {t} --apply"')
    return TimerAction(period=delay, condition=IfCondition(LaunchConfiguration("floor")),
                       actions=[ExecuteProcess(cmd=["bash", "-c", sh], output="screen", name="floor_level")])


def include(share, name):
    """launch 하나를 따로 감싸 포함 (인자 이름이 겹쳐도 서로 안 섞임, 명령줄 인자는 그대로 넘어감)."""
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / name)))])
