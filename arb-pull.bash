#!/bin/bash
ORIGINAL_IMAGE=humble911/ros2-desktop-ai-robot-book:humble-zh
MY_IMAGE=ros2-desktop-ai-robot-book-humble-zh:mine
ID=$(docker image ls -q $ORIGINAL_IMAGE)
if [ -z "$ID" ]; then
    docker pull $ORIGINAL_IMAGE
else
    echo "$ORIGINAL_IMAGE已经在本地存在"
fi
if [ $? -gt 0 ]; then
    echo "获取镜像失败"
    exit 1
fi
ID=$(docker image ls -q $ORIGINAL_IMAGE)
docker tag $ID  $MY_IMAGE
docker rmi $ORIGINAL_IMAGE
