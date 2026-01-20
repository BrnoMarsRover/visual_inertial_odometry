# Visual inertial odometry

## Introduction
This repository contains all code and documentation related to visual-inertial odometry (VIO).
The main goal is to find the most effective way to localize our drone and rover, which will be used in the European Rover Challenge.

## [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git)
The algorithm was developed in collaboration with NVIDIA. It is GPU-accelerated and has very specific software and hardware requirements.

### Setup list
- Isaac ROS visual SLAM : release-3.2
- Jetson Orin with installed JetPack 6.2
- ROS2 Humble
- Intel RealSense D435 (without IMU)
- IMU from Pixhawk PX4 (Cube orange)

ROS2 driver for RealSense and ROS2 bridge for Pixhawk was already done.

### Installation steps
I followed these steps from the documentation:
1. Setup [Isaac Apt Repository](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/isaac_apt_repository.html)
2. Setup [Developer Environment](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html)
3. [Quickstart](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#quickstart) - I installed prebuild binary package

### Launch file
A custom launch file was created to integrate Isaac VIO into the namespaces that we are using. This example is command for our drone f450.
```bash
ros2 launch visual_inertial_odometry isaac_ros_visual_slam.launch.py \
  robot_name:=f450 \
  robot_number:=1 \
  config:=vio_isaac_f450.yaml \
  imu_fusion:=True \
  slam_localization:=True

```

### IMU integration
The first time I tried the algorithm with IMU fusion enabled, it was very buggy, and the algorithm works more effectively with IMU fusion disabled.

I took a measurement of IMU noise characteristics. I used [Alan ROS2](https://github.com/CruxDevStuff/allan_ros2.git) package. And the results is in f450 config file.

Next step is to get exact tranformation between IMU and realsense.

## Authors
- Martin Kriz
