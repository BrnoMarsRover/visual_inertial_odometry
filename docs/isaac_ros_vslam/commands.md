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

### Fix vslam Not Running

Pokud vslam nejede, reinstaluj balíček:

```bash
sudo apt-get update
sudo apt-get install -y --fix-missing ros-humble-isaac-ros-visual-slam
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

### Run IMU Inverter

```bash
ros2 run visual_inertial_odometry imu_inverter.py
```

Subscribes to `/f450_1/aircraft/imu` and publishes inverted linear acceleration and angular velocity to `/f450_1/aircraft/imu/invert`.

### Run IMU Accel Inverter

```bash
ros2 run visual_inertial_odometry imu_accel_inverter.py
```

Subscribes to `/f450_1/aircraft/imu` and publishes inverted linear acceleration (gyro unchanged) to `/f450_1/aircraft/imu/accel_invert`.

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

### Run Visual SLAM (with inverted IMU)

```bash
ros2 launch visual_inertial_odometry isaac_ros_visual_slam.launch.py \
  robot_name:=f450 \
  robot_number:=1 \
  config:=vio_isaac_f450.yaml \
  imu_fusion:=True \
  slam_localization:=False \
  imu_topic:=/aircraft/imu/invert
```
