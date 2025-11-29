# Visual inertial odometry

## Introduction
This repository contains all code and documentation related to visual-inertial odometry (VIO).
The main goal is to find the most effective way to localize our drone and rover, which will be used in the European Rover Challenge.

## [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git)
The algorithm was developed in collaboration with NVIDIA. It is GPU-accelerated and has very specific software and hardware requirements.

### Setup
- Isaac ROS visual SLAM : release-3.2
- Jetson Orin with installed JetPack 6.2
- ROS2 Humble
- Intel RealSense D435 (without IMU)
- IMU from Pixhawk PX4
ROS2 driver for RealSense and ROS2 bridge was already done.

### Install
I followed these steps from the documentation:
1. Setup [Isaac Apt Repository](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/isaac_apt_repository.html)
2. Setup [Developer Environment](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html)
3. [Quickstart](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#quickstart) - I installed prebuild binary package 