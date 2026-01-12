@echo off
setlocal
set ORIGINAL_IMAGE=humble911/ros2-desktop-ai-robot-book:humble-zh
set MY_IMAGE=ros2-desktop-ai-robot-book-humble-zh:mine
for /f "usebackq" %%A in (`docker image ls -q %ORIGINAL_IMAGE%`) do set ID=%%A
if [%ID%] == [] (
    docker pull %ORIGINAL_IMAGE%
) else (
    echo %ORIGINAL_IMAGE%已经在本地存在
)
if errorlevel 1 (
    echo "获取镜像失败"
    exit 1
)
for /f "usebackq" %%A in (`docker image ls -q %ORIGINAL_IMAGE%`) do set ID=%%A
docker tag %ID% %MY_IMAGE%
docker rmi %ORIGINAL_IMAGE%
