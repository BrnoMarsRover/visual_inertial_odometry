#!/bin/bash

SESSION="vio"
WS="/home/martin/ros2_ws"
SOURCE="source $WS/install/setup.bash"

if [ "$1" == "kill" ]; then
    tmux kill-session -t $SESSION 2>/dev/null && echo "Session '$SESSION' ukončena." || echo "Session '$SESSION' neexistuje."
    exit 0
fi

ROBOT_NAME="${1:-f450}"
ROBOT_NUMBER="${2:-1}"

tmux new-session -d -s $SESSION -x 220 -y 50

# Pane 0: OAK-D camera driver
tmux send-keys -t $SESSION:0 "$SOURCE && ros2 launch oak_d_pro_driver oakd_camera_driver.launch.py \
  robot_name:=$ROBOT_NAME \
  robot_number:=$ROBOT_NUMBER \
  config:=sqrtVINS_config.yaml" Enter

# Pane 1: sqrtVINS
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "$SOURCE && ros2 launch visual_inertial_odometry sqrtVINS.launch.py \
  robot_name:=$ROBOT_NAME \
  robot_number:=$ROBOT_NUMBER \
  image_topic:=/oak/left/image_raw \
  imu_topic:=/oak/imu/data \
  rviz_enable:=true" Enter

tmux attach -t $SESSION
