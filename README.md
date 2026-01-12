# 《ROS 2与Python实战：AI机器人入门（修订第2版）》配套教材的Docker镜像构建说明

## 概要

- 包含《ROS 2与Python实战：AI机器人入门（修订第2版）》一书所需的所有库、ROS 2包和示例程序（中文翻译版）。
- 基于 Daisuke Sato 提供的 [tiryoh/ros2-desktop-vnc](https://github.com/Tiryoh/docker-ros2-desktop-vnc) 项目中的 ROS 2 Humble 版本进行定制。
- 集成 VSCodium 开发环境。
- 支持中文语言环境。

## 获取镜像方式一：下载镜像压缩包（国内用户推荐）

https://gitee.com/jupiterobot/docker-ros2-desktop-ai-robot-book-humble-zh

## 获取镜像方式二：镜像构建

终端进入此目录运行

./build.bash

## 获取镜像方式三：docker官网拉取镜像

docker pull humble911/ros2-desktop-ai-robot-book:humble-zh

## 查看镜像

docker images

# 运行

## 在 Linux 上运行

### 启动容器

./arb-run.bash

如要使用GPU

./arb-run.bash --gpus all

可以通过浏览器访问就表示成功，之后可以通过以下指令启动docker容器

docker start ai_robot_book

### 使用桌面环境

- **通过 Web 浏览器访问（HTML5 VNC）**  
  访问地址：http://127.0.0.1:6080  
  密码：ubuntu

- **通过 Remmina 等 VNC 客户端访问**  
  地址：127.0.0.1:15900  
  密码：ubuntu

### 将当前容器保存为新镜像

./arb-commit.bash

## 在 Windows 上运行

### 启动容器

windows终端之前运行下面的指令无法访问音频设备（不能语音识别与合成）

arb-run.bat

若需要在windows下运行且需要用到音频设备，需要安装WSL虚拟ubuntu再到ubuntu终端运行以下指令

arb-run-win.bash

如要使用GPU

arb-run.bat --gpus all

可以通过浏览器访问就表示成功，之后可以通过以下指令启动docker容器

docker start ai_robot_book

### 使用桌面环境

- **通过 Web 浏览器访问**  
  访问地址：http://127.0.0.1:6080  
  密码：ubuntu

- **通过 VNC 客户端访问**  
  地址：127.0.0.1:15900  
  密码：ubuntu

### 将当前容器保存为新镜像

arb-commit.bat

## 已知问题与后续改进

（暂无内容）

## 作者（日语原版）

升谷 保博

## 历史

- 2024-12-31: 初始创建

- 2026-1-9：中文版创建

## 许可证

Copyright (c) 2024, 2025 MASUTANI Yasuhiro  
All rights reserved.  
本项目采用 Apache License 2.0 许可证，完整条款见项目根目录下的 LICENSE 文件。

## 参考文献

- Daisuke Sato: Dockerfiles to provide HTML5 VNC interface to access Ubuntu Desktop + ROS 2, [GitHub tiryoh/ros2-desktop-vnc](https://github.com/Tiryoh/docker-ros2-desktop-vnc)