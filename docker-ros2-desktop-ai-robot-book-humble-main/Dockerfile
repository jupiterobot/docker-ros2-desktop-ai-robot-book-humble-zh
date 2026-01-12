# Copyright 2024, 2025 MASUTANI Yasuhiro
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ┌──────────────────────────────────────────────────────┐
# │ 中文定制版 ROS 2 Humble 桌面镜像 (Updated zh-Hans)      │
# ├──────────────────────────────────────────────────────┤
# │ 此镜像是对原版 docker-ros2-desktop-vnc 的中文增强版本，   │
# │ 主要改进包括：                                         │
# │ • 系统界面与终端全面支持简体中文 (zh_CN.UTF-8)            │
# │ • 预装中文字体（文泉驿）与拼音输入法（fcitx）              │
# │ • 使用 VS Code 官方版替代 VSCodium（国内源加速）         │
# │ • 自动安装 VS Code 中文语言包与 Python 插件              │
# │ • 优化 VNC 桌面启动脚本以确保中文环境生效                  │
# │                                                      │
# │ 构建命令：                                             │
# │   docker build -t ros2-desktop-ai-robot-book-humble:zh . │
# │                                                      │
# └──────────────────────────────────────────────────────┘
#
# This Dockerfile is based on https://github.com/Tiryoh/docker-ros2-desktop-vnc
# which is released under the Apache-2.0 license.

FROM ubuntu:jammy-20240808

ARG TARGETPLATFORM
LABEL maintainer="MASUTANI Yasuhiro <ai-robot-book@googlegroups.com>"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# [Layer 1] 系统升级
RUN apt-get update -q && \
    apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# [Layer 2] 安装 MATE 桌面
RUN apt-get update -q && \
    apt-get install -y ubuntu-mate-desktop && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# [Layer 3] 安装基础工具、中文字体、输入法、VNC、开发依赖
RUN apt-get update && \
    apt-get install -y \
        tigervnc-standalone-server tigervnc-common \
        supervisor wget curl gosu git sudo python3-pip tini \
        build-essential vim lsb-release locales \
        bash-completion tzdata terminator dos2unix apt-utils \
        nvtop \
        language-pack-zh-hans \
        fonts-wqy-zenhei fonts-wqy-microhei \
        fcitx fcitx-pinyin dbus-x11 && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# [Layer 4] 安装 noVNC（改用 v1.3.0，修复 fetchJSON 问题）
RUN mkdir -p /usr/lib/novnc && \
    cd /usr/lib/novnc && \
    for i in {1..3}; do \
        echo "Attempt $i to clone noVNC v1.3.0..."; \
        if git clone --depth=1 --branch v1.3.0 https://github.com/novnc/noVNC.git .; then \
            break; \
        else \
            echo "Clone failed, retrying in 5s..."; \
            sleep 5; \
        fi; \
    done && \
    pip install 'numpy<1.25.0' && \
    pip install 'websockify==0.10.0' && \
    ln -sf /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

# [Layer 5] 启用远程窗口自适应（v1.2.0 中有效）
RUN sed -i "s/UI.initSetting('resize', 'off');/UI.initSetting('resize', 'remote');/g" /usr/lib/novnc/app/ui.js

# [Layer 6] 禁用自动更新和崩溃报告
RUN sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades && \
    sed -i 's/enabled=1/enabled=0/g' /etc/default/apport

# [Layer 7] 启用 apt 补全（删除限制缓存的配置）
RUN rm -f /etc/apt/apt.conf.d/docker-clean

# [Layer 8] 安装 Firefox（通过 PPA）
RUN add-apt-repository ppa:mozillateam/ppa -y && \
    echo 'Package: *' > /etc/apt/preferences.d/mozilla-firefox && \
    echo 'Pin: release o=LP-PPA-mozillateam' >> /etc/apt/preferences.d/mozilla-firefox && \
    echo 'Pin-Priority: 1001' >> /etc/apt/preferences.d/mozilla-firefox && \
    apt-get update -q && \
    apt-get install -y firefox && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# [Layer 9] 安装 VSCodium（开源 VS Code 替代品，使用官方仓库）
RUN wget -qO- https://gitlab.com/paulcarroty/vscodium-deb-rpm-repo/raw/master/pub.gpg \
    | gpg --dearmor -o /usr/share/keyrings/vscodium-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/vscodium-archive-keyring.gpg] https://download.vscodium.com/debs vscodium main' \
    | tee /etc/apt/sources.list.d/vscodium.list && \
    apt-get update -q && \
    apt-get install -y codium && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# [Layer 10] 创建 ubuntu 用户
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo "ubuntu:ubuntu" | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo 'Defaults env_keep="http_proxy https_proxy no_proxy"' >> /etc/sudoers

# [Layer 11] 全局 Git 配置：强制使用 HTTP/1.1（解决 GitHub HTTP/2 流中断问题）
USER ubuntu
RUN git config --global http.version HTTP/1.1 && \
    git config --global advice.detachedHead false

# [Layer 12] 安装 ROS 2 Humble（含 rosdep，使用清华镜像）
ENV ROS_DISTRO=humble
ARG INSTALL_PACKAGE=desktop
USER root
RUN apt-get update -q && \
    apt-get install -y curl gnupg lsb-release && \
    curl -s https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update -q && \
    apt-get install -y \
        ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-${ROS_DISTRO}-rqt-* \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-ros-ign && \
    rm -rf /var/lib/apt/lists/*

# [Layer 13] 配置 rosdep 使用清华源并初始化（含 index 镜像）
ENV ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/20-default.list && \
    echo "yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/python.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list && \
    echo "yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/ruby.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init --include-eol-distros 2>/dev/null || true && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

# [Layer 14] 设置工作空间目录结构
ENV AIBOT_WS=/home/ubuntu/airobot_ws
RUN mkdir -p ${AIBOT_WS}/src && \
    chown -R ubuntu:ubuntu ${AIBOT_WS}
WORKDIR ${AIBOT_WS}

# [Layer 15] 克隆中文整合仓库（唯一代码源）
USER ubuntu
RUN cd ${AIBOT_WS}/src && \
    for i in {1..3}; do \
        echo "Attempt $i to clone AI-Robot-Book-Humble-zh (zh-CN integrated repo)..."; \
        if git clone https://github.com/jupiterobot/AI-Robot-Book-Humble-zh.git .; then \
            break; \
        else \
            echo "Failed, retrying in 5s..."; \
            sleep 5; \
        fi; \
    done

# [Layer 16] 安装 Whisper、音频工具及覆盖系统 coverage
USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        portaudio19-dev \
        pulseaudio \
        mpg123 \
        ffmpeg \
    && apt-get remove -y python3-coverage \
    && rm -rf /var/lib/apt/lists/*

USER ubuntu
RUN pip3 install --no-cache-dir 'numpy<1.25.0' && \
    pip3 install --no-cache-dir openai-whisper pyaudio SpeechRecognition soundfile gTTS mpg123 && \
    pip3 install --user --upgrade coverage numba llvmlite

# [Layer 17] 安装导航/SLAM/硬件相关 ROS 包
USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-teleop-tools \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        ros-humble-dynamixel-sdk \
        ros-humble-xacro \
        ros-humble-ament-cmake-clang-format \
        ros-humble-turtlebot3-msgs \
    && rm -rf /var/lib/apt/lists/*

# [Layer 18] 安装 Python 可视化库
USER ubuntu
RUN pip3 install --no-cache-dir matplotlib seaborn

# [Layer 19] 配置 .bashrc（元数据层）
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/ubuntu/.bashrc && \
    echo "export TURTLEBOT3_MODEL=happy_mini" >> /home/ubuntu/.bashrc

# [Layer 20] 复制地图文件（假设整合仓库中 chapter4/map 存在）
USER root
RUN cp -r ${AIBOT_WS}/src/chapter4/map /home/ubuntu/ || echo "Map directory not found, skipping."
USER ubuntu

# [Layer 21] 安装 OpenCV + Ultralytics（CPU 版）
USER root
RUN pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --no-cache-dir opencv-contrib-python==4.5.5.64 ultralytics && \
    pip3 uninstall -y opencv-python

# [Layer 22] 安装 ROS 视觉/相机驱动
USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-humble-vision-opencv \
        ros-humble-usb-cam \
        ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# [Layer 23] 安装 MoveIt 相关 ROS 包
USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-humble-joint-state-publisher-gui \
        ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# [Layer 24] 安装 transforms3d
USER ubuntu
RUN pip3 install --no-cache-dir --upgrade transforms3d

# [Layer 25] 写入 udev 规则
USER root
RUN echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' > /etc/udev/rules.d/99-ftdi_sio.rules

# [Layer 26] 安装 SMACH 状态机包
USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-humble-smach \
        ros-humble-executive-smach \
    && rm -rf /var/lib/apt/lists/*

# [Layer 27] 克隆 FlexBE 行为引擎（第三方库） + 安装 WebUI 依赖
USER ubuntu
RUN cd ${AIBOT_WS}/src && \
    for i in {1..3}; do \
        echo "Attempt $i to clone flexbe_behavior_engine..."; \
        if git clone -b 4.0.0 https://github.com/FlexBE/flexbe_behavior_engine; then \
            break; \
        else \
            echo "Failed, retrying in 5s..."; \
            sleep 5; \
        fi; \
    done && \
    for i in {1..3}; do \
        echo "Attempt $i to clone flexbe_webui..."; \
        if git clone https://github.com/AI-Robot-Book-Humble/flexbe_webui; then \
            break; \
        else \
            echo "Failed, retrying in 5s..."; \
            sleep 5; \
        fi; \
    done && \
    cd flexbe_webui && \
    pip3 install --no-cache-dir -r requires.txt

# [Layer 28] 克隆 pymoveit2（第三方库）
USER ubuntu
RUN cd ${AIBOT_WS}/src && \
    for i in {1..3}; do \
        echo "Attempt $i to clone pymoveit2..."; \
        if git clone https://github.com/AndrejOrsula/pymoveit2; then \
            break; \
        else \
            echo "Failed, retrying in 5s..."; \
            sleep 5; \
        fi; \
    done

# [Layer 29] 全局配置：colcon 默认 + CycloneDDS
USER ubuntu
RUN mkdir -p /home/ubuntu/.colcon && \
    echo '{ "build": { "symlink-install": true } }' > /home/ubuntu/.colcon/defaults.yaml && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ubuntu/.bashrc

# [Layer 30] 安装 CycloneDDS ROS 包
USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# [Layer 31] 安装 pydantic via pip and skip rosdep install
USER ubuntu
RUN pip3 install --no-cache-dir "pydantic>=1.9,<2.0"

USER root
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    cat > /etc/ros/rosdep/sources.list.d/10-pydantic-override.yaml <<'EOF'
pydantic:
  ubuntu:
    jammy: { skip: true }
EOF

# [Layer 32] 编译 pymoveit2
USER ubuntu
RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-select pymoveit2 --cmake-args "-DCMAKE_BUILD_TYPE=Release"

# [Layer 33] 编译其余包（排除 pymoveit2 已编译）
USER ubuntu
RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-ignore pymoveit2

# [Layer 34] 激活环境变量（写入 .bashrc）
USER ubuntu
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ubuntu/.bashrc && \
    echo "source ${AIBOT_WS}/install/setup.bash" >> /home/ubuntu/.bashrc

# [Layer 35] 清理 APT 缓存
USER root
RUN rm -rf /var/lib/apt/lists/*

# [Layer 36] 清理 pip 缓存
USER ubuntu
RUN pip3 cache purge

# [Layer 37] 设置默认用户与工作目录
USER ubuntu
WORKDIR /home/ubuntu

# [Layer 38] 设置中文 locale 环境变量
ENV LANG=zh_CN.UTF-8 \
    LANGUAGE=zh_CN:zh:Hans \
    LC_ALL=zh_CN.UTF-8

# [Layer 39] 生成中文 locale
USER root
RUN locale-gen zh_CN.UTF-8

# [Layer 40] 覆盖 VNC 启动脚本：注入中文环境
USER ubuntu
RUN mkdir -p .vnc && \
    echo '#!/bin/sh' > .vnc/vnc_run.sh && \
    echo 'export LANG=zh_CN.UTF-8' >> .vnc/vnc_run.sh && \
    echo 'export LANGUAGE=zh_CN:zh:Hans' >> .vnc/vnc_run.sh && \
    echo 'export LC_ALL=zh_CN.UTF-8' >> .vnc/vnc_run.sh && \
    echo 'exec vncserver :0 -fg -geometry 1920x1080 -depth 32 -localhost no' >> .vnc/vnc_run.sh && \
    chmod +x .vnc/vnc_run.sh

# [Layer 41] 配置 .xsession 使用 MATE 并启用中文
RUN echo '#!/bin/sh' > .xsession && \
    echo 'export LANG=zh_CN.UTF-8' >> .xsession && \
    echo 'export LANGUAGE=zh_CN:zh:Hans' >> .xsession && \
    echo 'exec mate-session' >> .xsession && \
    chmod +x .xsession

# [Layer 42] 移除 sudo 提示
USER root
RUN sed -i '/^# sudo hint$/,/^fi$/d' /etc/bash.bashrc

# [Layer 43] 复制并设置入口点脚本
COPY ./entrypoint.sh /
RUN dos2unix /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "-c", "/entrypoint.sh" ]

# [Layer 44] 设置运行时环境变量
ENV USER=ubuntu
ENV HOME=/home/ubuntu