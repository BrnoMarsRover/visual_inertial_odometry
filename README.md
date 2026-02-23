# Visual inertial odometry

## Introduction
This repository contains all code and documentation related to visual-inertial odometry (VIO).
The main goal is to find the most effective way to localize our drone and rover, which will be used in the European Rover Challenge.

## [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git)
The algorithm was developed in collaboration with NVIDIA. It is GPU-accelerated and has very specific software and hardware requirements.

### Launch file command
```bash
ros2 launch visual_inertial_odometry isaac_ros_visual_slam.launch.py \
  robot_name:=f450 \
  robot_number:=1 \
  config:=vio_isaac_f450.yaml \
  imu_fusion:=True \
  slam_localization:=True
```
### Documentation
- [Setup](/docs/isaac_ros_vslam/installation_steps.md)
- [IMU fusion](/docs/isaac_ros_vslam/imu_fusion.md)
- [Used commands](/docs/isaac_ros_vslam/commands.md)

## Authors
- Martin Kriz
