#!/usr/bin/env python3
"""
Extrahuje VSLAM data z rosbagů a uloží jako .mat:
  slam_status.mat      - VisualSlamStatus (tracking time, vo_state)
  slam_observations.mat - observations_cloud (feature count per frame)

Použití: python3 extract_slam_status.py
"""

import os, sys
try:
    import scipy.io as sio
except ImportError:
    print("Chybí scipy: pip install scipy"); sys.exit(1)
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Spusť ve ROS2 prostředí: source /opt/ros/humble/setup.bash"); sys.exit(1)

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Bag se status topicm (velký bag, má VisualSlamStatus + camera_info) ---
STATUS_BAG   = '/mnt/ros2bags/rosbag2_2026_03_06-10_53_23'
STATUS_TOPIC = '/f450_1/vio_isaac/visual_slam/status'
STATUS_MAT   = os.path.join(SCRIPT_DIR, 'slam_status.mat')

# --- Bag s observations_cloud (malý bag, přehrávaný z velkého) ---
OBS_BAG   = os.path.join(SCRIPT_DIR, '..', 'rosbags', '6.3.10_53', 'gps_vio_observations')
OBS_TOPIC = '/f450_1/vio_isaac/visual_slam/vis/observations_cloud'
OBS_MAT    = os.path.join(SCRIPT_DIR, 'slam_observations.mat')
TIMING_MAT = os.path.join(SCRIPT_DIR, 'cam_timing.mat')
CAM_TOPIC  = '/f450_1/sensors/realsense_d435/realsense_camera_node/infra1/camera_info'

def open_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    return reader

# ---- 1. VisualSlamStatus ----
print(f"Čtu velký bag: {STATUS_BAG}")
reader      = open_bag(STATUS_BAG)
StatusType  = get_message('isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus')
CamInfoType = get_message('sensor_msgs/msg/CameraInfo')

t_list, vo_list, track_list, cb_list = [], [], [], []
t_cam_list = []

while reader.has_next():
    tp, data, _ = reader.read_next()
    if tp == STATUS_TOPIC:
        msg = deserialize_message(data, StatusType)
        t   = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        t_list.append(t)
        vo_list.append(float(msg.vo_state))
        track_list.append(msg.track_execution_time * 1000.0)
        cb_list.append(msg.node_callback_execution_time * 1000.0)
    elif tp == CAM_TOPIC:
        msg = deserialize_message(data, CamInfoType)
        t   = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        t_cam_list.append(t)

sio.savemat(STATUS_MAT, {
    't_status_abs':  np.array(t_list),
    'vo_state':      np.array(vo_list),
    'track_time_ms': np.array(track_list),
    'cb_time_ms':    np.array(cb_list),
})
print(f"  -> {STATUS_MAT}  ({len(t_list)} zpráv, track mean={np.mean(track_list):.2f} ms, max={np.max(track_list):.2f} ms)")

# Camera timing — t_vio_origin se dopočítá v MATLABu, ukládáme absolutní časy
t_cam_arr = np.array(t_cam_list)
dt_cam    = np.diff(t_cam_arr) * 1000.0   # ms
fps_cam   = 1000.0 / dt_cam
# t_cam_norm se počítá v MATLABu jako t_cam_abs - t_vio(1), ukládáme abs časy
sio.savemat(TIMING_MAT, {
    't_cam_abs': t_cam_arr,
    'dt_cam':    dt_cam,
    'fps_cam':   fps_cam,
})
print(f"  -> {TIMING_MAT}  ({len(t_cam_arr)} framů, mean dt={dt_cam.mean():.2f} ms)")

# ---- 2. observations_cloud (feature count) ----
print(f"Čtu observations bag: {OBS_BAG}")
reader2  = open_bag(os.path.normpath(OBS_BAG))
ObsType  = get_message('sensor_msgs/msg/PointCloud2')

t2_list, count_list = [], []
while reader2.has_next():
    tp, data, _ = reader2.read_next()
    if tp != OBS_TOPIC: continue
    msg = deserialize_message(data, ObsType)
    t   = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    t2_list.append(t)
    count_list.append(float(msg.width))

counts = np.array(count_list)
sio.savemat(OBS_MAT, {
    't_obs_abs':     np.array(t2_list),
    'feature_count': counts,
})
print(f"  -> {OBS_MAT}  ({len(counts)} zpráv, mean={counts.mean():.0f}, min={counts.min():.0f}, max={counts.max():.0f})")
print(f"     < 80 features: {(counts<80).sum()} framů ({100*(counts<80).mean():.1f}%)")
