# Commands

## Enter Docker Container

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

## Inside Docker Container

### Install Dependencies

```bash
sudo apt-get install -y ros-humble-isaac-ros-visual-slam
```

### Source ROS Environment

```bash
source /opt/ros/humble/setup.bash
```

### Navigate to Workspace

```bash
cd /workspaces/isaac_ros-dev
source install/setup.bash
```

### Build Package (if not built yet)

```bash
colcon build --packages-select visual_inertial_odometry
source install/setup.bash
```

### Run IMU Filter

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -r imu/data_raw:=/f450_1/aircraft/imu/lowpass \
  -r imu/data:=/f450_1/aircraft/imu/filtered \
  -p use_mag:=false \
  -p publish_tf:=false \
  -p world_frame:=enu \
  -p remove_gravity:=false \
  -p gain:=0.01
```
