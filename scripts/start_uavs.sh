#!/bin/bash

trap 'kill $(jobs -p)' SIGINT

# 检查参数是否为空
if [ -z "$1" ]; then
  echo "Error: User not type the num of UAV swarm!!!"
  exit 1
fi

# 检查 setup.bash 文件是否存在
if [ ! -f "../devel/setup.bash" ]; then
  echo "Error: ../devel/setup.bash not found"
  exit 1
fi

source ../devel/setup.bash

echo "Loading $1 UAVs."

for (( num=1; num<=$1; num++))
do
group_name="uav""$num"
if (($num == 1))
then
x=-4.0
y=4.0
Y=0
roslaunch mrsl_quadrotor_launch user_defined_multi_uav.launch group_name:=$group_name x:=$x y:=$y Y:=$Y 
fi

if (($num == 2))
then
x=0.0
y=-4.0
Y=0
roslaunch mrsl_quadrotor_launch user_defined_multi_uav.launch group_name:=$group_name x:=$x y:=$y Y:=$Y 
fi

if (($num == 3))
then
x=0.0
y=4.0
Y=0
roslaunch mrsl_quadrotor_launch user_defined_multi_uav.launch group_name:=$group_name x:=$x y:=$y Y:=$Y 
fi

if (($num == 4))
then
x=25.0
y=-25.0
Y=0.0
roslaunch mrsl_quadrotor_launch user_defined_multi_uav.launch group_name:=$group_name x:=$x y:=$y Y:=$Y 
fi
done

# 启动tf转换
for (( num=1; num<=$1; num++))
do
tf_name="uav""$num""_msg_to_tf"
trans_name="uav""$num""_ctrller_wrapper"
uav_name="uav""$num"
uav_id="$num"

roslaunch mrsl_quadrotor_utils user_defined_tf_pub.launch tf_name:=$tf_name trans_name:=$trans_name uav_name:=$uav_name uav_id:=$uav_id &
done

wait