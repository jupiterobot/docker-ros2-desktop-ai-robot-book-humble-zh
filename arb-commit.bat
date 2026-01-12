@echo off
setlocal
set MY_IMAGE=ros2-desktop-ai-robot-book-humble:zh
docker commit ai_robot_book %MY_IMAGE%
