#!/bin/bash

REPO="/home/martin/ros2_ws/src/visual_inertial_odometry"
WS="/home/martin/ros2_ws"
SESSION="dev"
SOURCE="source $WS/install/setup.bash"
ROBOT_NAME="${1:-f450}"
ROBOT_NUMBER="${2:-1}"

# Open VS Code in the repo
code "$REPO" &

# Kill existing session if any
tmux kill-session -t $SESSION 2>/dev/null

# Create session (pane 0 = left)
tmux new-session -d -s $SESSION -x 220 -y 50

# Split right half (pane 1 = right)
tmux split-window -h -t "$SESSION:0.0"

# Split right pane vertically (pane 1 = top-right, pane 2 = bottom-right)
tmux split-window -v -t "$SESSION:0.1"

# Left: claude in repo
tmux send-keys -t "$SESSION:0.0" "cd $REPO && claude" Enter

# Right top: ros2bags directory
tmux send-keys -t "$SESSION:0.1" "cd /mnt/ros2bags" Enter

# Right bottom: sqrtVINS launch
tmux send-keys -t "$SESSION:0.2" "$SOURCE && ros2 launch visual_inertial_odometry sqrtVINS.launch.py \
  robot_name:=$ROBOT_NAME \
  robot_number:=$ROBOT_NUMBER \
  image_topic:=/oak/left/image_raw \
  imu_topic:=/oak/imu/data \
  rviz_enable:=true" Enter

# Focus left pane (claude)
tmux select-pane -t "$SESSION:0.0"

tmux attach -t $SESSION
