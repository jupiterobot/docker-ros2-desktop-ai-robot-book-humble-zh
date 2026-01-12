#!/usr/bin/env bash

# 清理空的代理环境变量（避免干扰）
for v in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY
do
  eval "test -v $v -a -z $"$v" && unset $v"
done

# 源 ROS 环境（注意：此处 ROS_DISTRO 需已定义，例如 humble）
source /opt/ros/${ROS_DISTRO}/setup.bash

set -eu  # 出错或使用未定义变量时立即退出
set -v   # 打印执行的命令（用于调试）

# 为避免 `set -u` 导致 `source install/setup.bash` 报 "unbound variable" 错误，
# 预先设置可能被引用但未定义的变量为空
export COLCON_TRACE=
export AMENT_TRACE_SETUP_FILES=
export AMENT_PYTHON_EXECUTABLE=
export COLCON_PREFIX_PATH=
export COLCON_PYTHON_EXECUTABLE=
export CMAKE_PREFIX_PATH=

# 用于 apt-get 的非交互模式
AG_ENV="DEBIAN_FRONTEND=noninteractive"

# 解决 pip 安装时可能出现的 ReadTimeoutError
# 参考: https://www.infocircus.jp/2020/06/30/pip-readtimeouterror-certbot-auto/
export PIP_DEFAULT_TIMEOUT=1000

# 更新系统包列表
sudo $AG_ENV apt-get update -q

# 升级已安装的系统包
sudo $AG_ENV apt-get upgrade -yq

# 创建 ROS 工作空间
mkdir -p ~/airobot_ws/src

# 安装兼容的 numpy 版本：
# Whisper 要求 numpy < 1.25.0，YOLO 要求 numpy < 2.0
pip3 install 'numpy<1.25.0'

# 安装 Cyclone DDS 作为 RMW 实现，并设为默认
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# 配置 colcon 默认使用 --symlink-install（加快开发迭代）
mkdir -p ~/.colcon
cat << __EOF__ > ~/.colcon/defaults.yaml
{
    "build": {
        "symlink-install": true
    }
}
__EOF__

# ========== 第2章 ==========
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/chapter2
sudo $AG_ENV apt-get -y install xterm
# 将自定义的 turtlesim 启动文件复制到系统目录
sudo cp ~/airobot_ws/src/chapter2/turtlesim_launch/mysim.launch.py /opt/ros/${ROS_DISTRO}/share/turtlesim/launch

# ========== 第3章 ==========
sudo $AG_ENV apt-get -y install portaudio19-dev
sudo $AG_ENV apt-get -y install pulseaudio
pip3 install pyaudio
pip3 install SpeechRecognition
pip3 install SpeechRecognition[whisper-local] soundfile
pip3 install gTTS
sudo $AG_ENV apt-get -y install mpg123
pip3 install mpg123
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/chapter3

# ========== 第4章 ==========
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-navigation2
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-nav2-bringup
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-slam-toolbox
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-teleop-tools
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-cartographer
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-cartographer-ros
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-dynamixel-sdk
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-xacro
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-ament-cmake-clang-format
pip3 install matplotlib seaborn
cd ~/airobot_ws/src/
git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/AI-Robot-Book-Humble/chapter4.git
git clone https://github.com/AI-Robot-Book-Humble/turtlebot3_happy_mini.git
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=happy_mini" >> ~/.bashrc
cp -r ~/airobot_ws/src/chapter4/map ~

# ========== 第5章 ==========
# 安装指定版本的 OpenCV（与 YOLO 兼容）
pip3 install opencv-contrib-python==4.5.5.64
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-vision-opencv
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-usb-cam
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-realsense2-camera
pip3 install ultralytics
# 卸载 ultralytics 自动安装的 opencv-python（避免冲突）
pip3 uninstall -y opencv_python
cd ~/airobot_ws/src
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
git clone https://github.com/AI-Robot-Book-Humble/chapter5

# ========== 第6章 ==========
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-joint-state-publisher-gui
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/crane_plus
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-tf-transformations
# 升级 transforms3d 以兼容新版本 numpy（解决旧版 transforms3d 与 numpy>=2 的兼容性问题）
pip3 install transforms3d --upgrade
# 配置 USB 设备权限（用于 CRANE+ 控制器）
echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' > ~/99-ftdi_sio.rules
sudo mv ~/99-ftdi_sio.rules /etc/udev/rules.d/
git clone https://github.com/AndrejOrsula/pymoveit2
git clone https://github.com/AI-Robot-Book-Humble/chapter6

# ========== 第7章 ==========
sudo $AG_ENV apt-get install -y ros-${ROS_DISTRO}-smach
sudo $AG_ENV apt-get install -y ros-${ROS_DISTRO}-executive-smach
cd ~/airobot_ws/src
git clone -b 4.0.0 https://github.com/FlexBE/flexbe_behavior_engine
git clone https://github.com/AI-Robot-Book-Humble/flexbe_webui
git clone https://github.com/AI-Robot-Book-Humble/chapter7
cd ~/airobot_ws/src/flexbe_webui
pip3 install -r requires.txt
echo "export WORKSPACE_ROOT=~/airobot_ws" >> ~/.bashrc

# ========== 附录B ==========
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/appendixB

# ========== 附录E ==========
sudo $AG_ENV apt-get -y install ros-${ROS_DISTRO}-tf-transformations
pip3 install transforms3d --upgrade
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/appendixE

# ========== 构建所有包 ==========
cd ~/airobot_ws
# 安装依赖
rosdep install --default-yes --from-paths src --ignore-src
# 单独构建 pymoveit2（带 Release 优化）
colcon build --packages-select pymoveit2 --cmake-args "-DCMAKE_BUILD_TYPE=Release"
# 构建其余所有包（排除 pymoveit2）
colcon build --packages-ignore pymoveit2

# 激活工作空间
set +v  # 关闭命令回显
source install/setup.bash
set -v  # 重新开启命令回显

# ========== 下载 Ignition Gazebo 模型 ==========
ign fuel download -v 4 -u https://fuel.gazebosim.org/1.0/openrobotics/models/sun
ign fuel download -v 4 -u https://fuel.gazebosim.org/1.0/openrobotics/models/ground%20plane
ign fuel download -v 4 -u https://fuel.gazebosim.org/1.0/openrobotics/models/wood%20cube%205cm
ign fuel download -v 4 -u https://fuel.gazebosim.org/1.0/openrobotics/models/table

# ========== 下载 Classic Gazebo 模型 ==========
function download_model(){
    if [[ -d $HOME'/.gazebo/models/'$1 ]]; then
        echo "模型 $1 已存在。"
    else
        wget -l1 -np -nc -r "http://models.gazebosim.org/"$1 --accept=gz
    fi
}

mkdir -p ~/.gazebo/models && cd ~/.gazebo/models
cd /tmp 
TMPDIR=$(mktemp -d tmp.XXXXXXXXXX) 
cd $TMPDIR 
download_model sun
download_model ground_plane
download_model mailbox
download_model cafe_table
download_model first_2015_trash_can
download_model table_marble
if [[ -d "models.gazebosim.org" ]]; then
    cd "models.gazebosim.org"
    for i in *; do tar -zvxf "$i/model.tar.gz"; done
    cp -vfR * ~/.gazebo/models/
fi
rm -rf $TMPDIR

# ========== 后续配置 ==========
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "source ~/airobot_ws/install/setup.bash" >> ~/.bashrc

# 关闭严格变量检查（因 setup.bash 可能引用未定义变量）
set +u

# 清理 apt 缓存以减小镜像体积
sudo rm -rf /var/lib/apt/lists/*

# 清理 pip 缓存
pip3 cache purge

# ========== 安装 VS Code 插件（确保当前为 ubuntu 用户）==========
echo "正在安装 VS Code 扩展..."
code --install-extension ms-ceintl.vscode-language-pack-zh-hans
code --install-extension ms-python.python
echo "VS Code 配置完成。"