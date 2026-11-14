#!/bin/sh
# 절전에서 깨어난 뒤 CUDA 가 고장 나는 문제 대처 (증상: ZED "NO GPU DETECTED", "CUDA error =999").
# nvidia_uvm 모듈을 내렸다 다시 올림. 화면은 정상인데 CUDA 만 안 될 때.
#
# 손으로 쓸 때 (ZED 등 CUDA 를 쓰는 프로그램을 먼저 끄고):
#   sudo ~/capstone/capstone_ws/src/navigation/tools/nvidia_uvm_reload.sh
#
# 깨어날 때마다 자동으로 하려면 (한 번만 설치):
#   sudo install -m 755 ~/capstone/capstone_ws/src/navigation/tools/nvidia_uvm_reload.sh /etc/systemd/system-sleep/50-nvidia-uvm-reload
#   지우기: sudo rm /etc/systemd/system-sleep/50-nvidia-uvm-reload
# systemd 는 이 스크립트를 "pre|post suspend" 인자로 부름 — post (깨어난 뒤) 일 때만 동작. 인자 없이 부르면 바로 동작.
if [ -z "$1" ] || [ "$1" = "post" ]; then
    [ "$1" = "post" ] && sleep 2
    if rmmod nvidia_uvm 2>/dev/null; then
        modprobe nvidia_uvm && logger -s -t nvidia-uvm-reload "nvidia_uvm 다시 올림"
    else
        logger -s -t nvidia-uvm-reload "nvidia_uvm 사용 중이라 못 내림 — CUDA 프로그램(ZED 등)을 끄고 다시 실행"
    fi
fi
exit 0
