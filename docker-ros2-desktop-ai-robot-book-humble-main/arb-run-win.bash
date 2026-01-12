#!/bin/bash

# 此脚本和arb-run.bash的区别在于，此脚本是在windows中用WSL2使用ubuntu时使用此脚本运行docker容器
# sudo apt update
# sudo apt install pulseaudio

MY_IMAGE=ros2-desktop-ai-robot-book-humble:zh

# 获取当前用户 UID（用于路径）
USER_ID=$(id -u)

# === 关键修改：检测是否在 WSL2 中运行 ===
if grep -q Microsoft /proc/version || [ -d /mnt/wslg ]; then
    # WSL2 + WSLg 环境
    echo "Detected WSL2 with WSLg. Using WSLg PulseAudio."

    PULSE_SERVER_PATH="/mnt/wslg/PulseServer"
    if [ ! -S "$PULSE_SERVER_PATH" ]; then
        echo "Warning: $PULSE_SERVER_PATH not found. Audio may not work."
    fi

    docker run \
        -e RESOLUTION=1920x1080 \
        -e PULSE_SERVER=unix:/run/user/$USER_ID/pulse/native \
        -e PULSE_COOKIE=/dev/null \
        -v "$PULSE_SERVER_PATH":/run/user/$USER_ID/pulse/native \
        --name ai_robot_book \
        -p 15900:5900 \
        -p 13389:3389 \
        -p 6080:80 \
        --shm-size=512m \
        --privileged \
        "$@" \
        "$MY_IMAGE"

else
    # 非 WSL2 环境（原生 Linux）
    echo "Running on native Linux. Using standard PulseAudio."

    docker run \
        -e RESOLUTION=1920x1080 \
        -e PULSE_COOKIE=/tmp/pulse/cookie \
        -e PULSE_SERVER=unix:/tmp/pulse/native \
        -v /run/user/$USER_ID/pulse/native:/tmp/pulse/native \
        -v ~/.config/pulse/cookie:/tmp/pulse/cookie:ro \
        --name ai_robot_book \
        -p 15900:5900 \
        -p 13389:3389 \
        -p 6080:80 \
        --shm-size=512m \
        --privileged \
        "$@" \
        "$MY_IMAGE"
fi