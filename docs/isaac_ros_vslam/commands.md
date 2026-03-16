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

## ROS2 Bag Play

### Přehrání bag bez vio_isaac topics

Topics `/f450_1/vio_isaac/*` jsou vyloučeny (chybí balíčky `uas_status`, `pylon_ros2_camera_interfaces`, `vision_msgs`, `uas_detections`).
Poznámka: `--exclude` není v Humble podporováno, je nutné explicitně uvést `--topics`.

```bash
TOPICS="/clicked_point /diagnostics /events/read_split /events/write_split /f450_1/aircraft/altimeter /f450_1/aircraft/battery /f450_1/aircraft/cmd_vel /f450_1/aircraft/gps_odometry /f450_1/aircraft/imu /f450_1/aircraft/landed_state /f450_1/aircraft/odometry /f450_1/aircraft/raw_gps /f450_1/aircraft/raw_gps/reference_time /f450_1/aircraft/robot_description /f450_1/aruco_detector/aruco_detector/debug /f450_1/aruco_detector/aruco_detector/marker /f450_1/aruco_detector/aruco_detector/pixel /f450_1/aruco_detector/aruco_detector/pose /f450_1/aruco_detector/aruco_detector/position /f450_1/aruco_detector/aruco_detector/result /f450_1/aruco_detector/aruco_detector/transform /f450_1/aruco_detector/pose /f450_1/control_manager/pid/error /f450_1/control_manager/pid/set_point /f450_1/control_manager/remain_trajectory /f450_1/detection/detections/image /f450_1/detection_visualizer/visualization /f450_1/ground_object_fusion/occupancy /f450_1/object_localization/objects /f450_1/object_localization/objects/visualization /f450_1/planner_manager/path /f450_1/planner_manager/trajectory /f450_1/sensors/basler_camera/blaze_camera_info /f450_1/sensors/basler_camera/blaze_cloud /f450_1/sensors/basler_camera/blaze_confidence /f450_1/sensors/basler_camera/blaze_depth_map /f450_1/sensors/basler_camera/blaze_depth_map_color /f450_1/sensors/basler_camera/blaze_intensity /f450_1/sensors/basler_camera/camera_info /f450_1/sensors/basler_camera/image_raw /f450_1/sensors/basler_camera/image_rect /f450_1/sensors/realsense_d435/realsense_camera_node/depth/camera_info /f450_1/sensors/realsense_d435/realsense_camera_node/depth/image_rect_raw /f450_1/sensors/realsense_d435/realsense_camera_node/depth/metadata /f450_1/sensors/realsense_d435/realsense_camera_node/extrinsics/depth_to_infra1 /f450_1/sensors/realsense_d435/realsense_camera_node/extrinsics/depth_to_infra2 /f450_1/sensors/realsense_d435/realsense_camera_node/infra1/camera_info /f450_1/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw /f450_1/sensors/realsense_d435/realsense_camera_node/infra1/metadata /f450_1/sensors/realsense_d435/realsense_camera_node/infra2/camera_info /f450_1/sensors/realsense_d435/realsense_camera_node/infra2/image_rect_raw /f450_1/sensors/realsense_d435/realsense_camera_node/infra2/metadata /f450_1/sensors/realsense_d435/scan /goal_pose /husky_1/goal_pose /initialpose /parameter_events /rosout /tf /tf_static"

ros2 bag play <bag_folder> --read-ahead-queue-size 10000 --topics $TOPICS
```
