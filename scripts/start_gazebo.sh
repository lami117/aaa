#!/bin/bash

# 检查参数是否为空
if [ -z "$1" ]; then
  echo "Error: User not type the world name!!!"
  exit 1
fi

# 检查 setup.bash 文件是否存在
if [ ! -f "../devel/setup.bash" ]; then
  echo "Error: ../devel/setup.bash not found"
  exit 1
fi

source ../devel/setup.bash
echo "Start gazebo with world: $1"
roslaunch mrsl_quadrotor_launch gazebo.launch world:=$1