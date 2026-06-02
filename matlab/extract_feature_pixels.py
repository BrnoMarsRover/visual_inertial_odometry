#!/usr/bin/env python3
"""
Projektuje observations_cloud (3D body v map frame) do pixel koordinát kamerového snímku.

Transform chain:
  map ≈ odom  (žádný TF offset)
  odom → base_link          : z odometrie (VIO pose)
  base_link → rs_base       : t=(0.17, 0, -0.044), R=I  (statický TF)
  rs_base → infra1_optical  : t=(0,0,0), q=(-0.5, 0.5, -0.5, 0.5) (statický TF)

Camera intrinsics (infra1, 848x480):
  fx=425.59  fy=425.59  cx=426.06  cy=241.34  (D=0, rektifikováno)

Výstup: feat_pixels.mat
  t_feat_abs  : (M,)      absolutní timestampy observations_cloud
  feat_px     : (M, Nmax) pixel X souřadnice (NaN = prázdné místo)
  feat_py     : (M, Nmax) pixel Y souřadnice (NaN = prázdné místo)
  feat_count  : (M,)      počet platných features v každém framu
"""

import os, sys, struct
import numpy as np
from scipy.spatial.transform import Rotation
import scipy.io as sio

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("source /opt/ros/humble/setup.bash"); sys.exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OBS_BAG    = os.path.normpath(os.path.join(SCRIPT_DIR, '../rosbags/6.3.10_53/gps_vio_observations'))
OUT_MAT    = os.path.join(SCRIPT_DIR, 'feat_pixels.mat')

OBS_TOPIC  = '/f450_1/vio_isaac/visual_slam/vis/observations_cloud'
ODOM_TOPIC = '/f450_1/vio_isaac/visual_slam/tracking/odometry'

# ---- Camera intrinsics (infra1 rectified, 848x480) ----
FX, FY = 425.59, 425.59
CX, CY = 426.06, 241.34
IMG_W,  IMG_H = 848, 480

# ---- Statický transform: base_link → infra1_optical_frame ----
# base_link → rs_base: pure translation
T_BASE_RS = np.eye(4)
T_BASE_RS[:3, 3] = [0.170, 0.0, -0.044]

# rs_base → infra1_optical: pure rotation q=(-0.5, 0.5, -0.5, 0.5) [x y z w]
R_optical = Rotation.from_quat([-0.5, 0.5, -0.5, 0.5]).as_matrix()
T_RS_OPT = np.eye(4)
T_RS_OPT[:3, :3] = R_optical

# Celkový statický TF: base_link → infra1_optical
T_BASE_OPT = T_BASE_RS @ T_RS_OPT

def quat_pos_to_T(pos, quat_xyzw):
    """VIO odometry pose → 4x4 transform matrix (odom→base_link)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    T[:3, 3]  = pos
    return T

def project_points(pts_world, T_world_base):
    """Vrátí (u, v) pixel souřadnice pro 3D body v world frame. Filtruje mimo obraz."""
    T_world_opt = T_world_base @ T_BASE_OPT
    T_opt_world = np.linalg.inv(T_world_opt)

    ones = np.ones((len(pts_world), 1))
    pts_h = np.hstack([pts_world, ones])           # (N, 4)
    pts_cam = (T_opt_world @ pts_h.T).T            # (N, 4)

    mask = pts_cam[:, 2] > 0.1                     # před kamerou (z > 0.1 m)
    u = FX * pts_cam[:, 0] / pts_cam[:, 2] + CX
    v = FY * pts_cam[:, 1] / pts_cam[:, 2] + CY

    in_img = mask & (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    u[~in_img] = np.nan
    v[~in_img] = np.nan
    return u, v

# ---- Načtení ----
print(f"Čtu bag: {OBS_BAG}")

def open_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    return r

# Průchod 1: načti všechny odometry zprávy do slovníku {t → pose}
print("  Načítám odometrii...")
OdomType = get_message('nav_msgs/msg/Odometry')
odom_times, odom_poses = [], []
reader = open_bag(OBS_BAG)
while reader.has_next():
    tp, data, _ = reader.read_next()
    if tp != ODOM_TOPIC: continue
    msg = deserialize_message(data, OdomType)
    t   = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    p   = msg.pose.pose.position
    q   = msg.pose.pose.orientation
    odom_times.append(t)
    odom_poses.append((
        np.array([p.x, p.y, p.z]),
        np.array([q.x, q.y, q.z, q.w])
    ))
odom_times = np.array(odom_times)
print(f"  Odometry: {len(odom_times)} zpráv")

# Průchod 2: projekce observations_cloud
print("  Projektuji observations_cloud...")
ObsType = get_message('sensor_msgs/msg/PointCloud2')
all_t, all_px, all_py, all_count = [], [], [], []

reader2 = open_bag(OBS_BAG)
while reader2.has_next():
    tp, data, _ = reader2.read_next()
    if tp != OBS_TOPIC: continue
    msg = deserialize_message(data, ObsType)
    t_obs = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    # Nejbližší odometry zpráva
    idx = np.argmin(np.abs(odom_times - t_obs))
    dt  = abs(odom_times[idx] - t_obs)
    if dt > 0.1:   # > 100 ms = přeskočit
        continue

    pos, quat = odom_poses[idx]
    T_world_base = quat_pos_to_T(pos, quat)

    # Extrakce 3D bodů z PointCloud2
    pts = []
    for i in range(msg.width):
        off = i * msg.point_step
        x, y, z = struct.unpack_from('fff', bytes(msg.data[off:off+12]))
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            pts.append([x, y, z])

    if not pts:
        continue

    pts = np.array(pts)
    u, v = project_points(pts, T_world_base)

    all_t.append(t_obs)
    all_px.append(u)
    all_py.append(v)
    all_count.append(int(np.sum(np.isfinite(u))))

# Zarovnání na stejnou délku (NaN padding)
max_n = max(len(x) for x in all_px) if all_px else 0
M = len(all_t)
px_mat = np.full((M, max_n), np.nan)
py_mat = np.full((M, max_n), np.nan)
for i, (ux, vy) in enumerate(zip(all_px, all_py)):
    px_mat[i, :len(ux)] = ux
    py_mat[i, :len(vy)] = vy

sio.savemat(OUT_MAT, {
    't_feat_abs':  np.array(all_t),
    'feat_px':     px_mat,
    'feat_py':     py_mat,
    'feat_count':  np.array(all_count, dtype=np.float64),
})

print(f"Uloženo: {OUT_MAT}")
print(f"  {M} framů, max features/frame = {max_n}")
print(f"  Průměr platných features v obraze: {np.mean(all_count):.0f}")
