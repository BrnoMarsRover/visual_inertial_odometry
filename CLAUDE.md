# CLAUDE.md — Visual Inertial Odometry

## Projekt

ROS 2 balíček pro VIO (Visual-Inertial Odometry) na dronu F450. Primárně sqrtVINS (OpenVINS) se stereo OAK-D kamerou.

**Dron:** F450, ROS 2 topics prefixovány `/f450_1/`  
**Kamera:** OAK-D (serial: `oak-d-2DPW-K2535-0004`), stereo 640×400, IMU 400 Hz  
**Aktivní konfig:** `config/f450_sqrtVINS_stereo/`

---

## Struktura

```
config/
  f450_sqrtVINS_stereo/     # aktivní stereo konfig (OpenVINS)
  f450_sqrtVINS_mono/       # mono konfig
  f450/                     # Isaac ROS VSLAM konfig
calibrations/
  oak-d-2DPW-K2535-0004/
    static/                 # pinhole-radtan (pouze kamera, bez IMU)
    dynamic/                # pinhole-radtan + IMU-cam extrinsics
  oak-d-2DPW-K2535-0004-stereo_2/
                            # pinhole-equidistant + IMU-cam
launch/
matlab/                     # analytické skripty + Python extraktory
rosbags/
```

---

## Kalibrace (OAK-D)

Aktivní stereo konfig používá `kalibr_imucam_chain.yaml` s modelem **pinhole-equidistant**.

Novější kalibrace (`calibrations/oak-d-2DPW-K2535-0004/`) používá **pinhole-radtan** — pokud bude přepnuta, je třeba aktualizovat `relative_config_imucam` v `estimator_config.yaml`.

---

## Známé problémy a ladění

### Z-osa má velký drift

Příčiny a doporučené změny v `config/f450_sqrtVINS_stereo/`:

**1. IMU noise hodnoty jsou 10× nafouklé** (`kalibr_imu_chain.yaml`)

Komentář "Inflated values (multiplied by 10)" — přefouknutý šum znamená, že filtr nevěří IMU. Z osa závisí na IMU víc než X/Y (gravitace), takže trpí nejvíc. Zkusit snížit na 3×:

```yaml
accelerometer_noise_density: 0.010   # bylo 0.036 (10x Allan)
accelerometer_random_walk: 0.00073   # bylo 0.00245
gyroscope_noise_density: 0.0019      # bylo 0.0065
gyroscope_random_walk: 1.15e-05      # bylo 3.86e-05
```

**2. `Ta` matice má cross-axis coupling v Z**

`Ta[2][0] = 0.0147` → 1.5 % X akcelerace (dopředný let) přeléká do Z měření jako bias. Hodnoty jsou nakalibrované, ale online refinement je vypnutý:

```yaml
# estimator_config.yaml
calib_imu_intrinsics: true      # bylo false — online refinement Ta, Tw
calib_imu_g_sensitivity: true   # bylo false — Tg je nenulové, ale ignorované
```

**3. Málo klonů a SLAM features**

```yaml
max_clones: 15                  # bylo 11
max_slam: 75                    # bylo 50
max_slam_in_update: 35          # bylo 25
```

**4. ZUPT (Zero Velocity Update)**

Pokud drone visí nebo přistává, ZUPT výrazně stabilizuje Z:

```yaml
try_zupt: true
zupt_chi2_multipler: 1
zupt_max_velocity: 0.15
zupt_max_disparity: 1.0
zupt_only_at_beginning: false
```

**Priorita změn:**

| Změna | Dopad |
|---|---|
| Snížit IMU noise (10× → 3×) | Velký |
| `calib_imu_intrinsics: true` | Střední |
| `max_clones: 15` | Střední |
| ZUPT | Velký při hoveru/přistání |

---

## Analytika (matlab/)

Hlavní skript: `matlab/gps_vio_compare_6310.m` — porovnání GPS (ENU) vs VIO, drift analýza.

Python extraktory (čtou 25.6 GB bag → .mat cache):
- `extract_slam_status.py` → `slam_status.mat`, `slam_observations.mat`, `cam_timing.mat`
- `extract_feature_pixels.py` → `feat_pixels.mat`

Klíčové rosbag soubory:

| Bag | Obsah |
|-----|-------|
| `rosbags/6.3.10_53/gps_vio_2026_03_06` | GPS + VIO, 221 s |
| `rosbags/6.3.10_53/gps_vio_observations` | GPS + VIO + observations_cloud |
| `/mnt/ros2bags/rosbag2_2026_03_06-10_53_23` | Plný let 6.3., 25.6 GB |
