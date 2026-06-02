The calibration has three steps:

1. Static - Camera Intrinsic Calibration, (Kalibr)
2. Static - IMU Noise Calibration, (allan_variance_ros)

Build in docker because it is in ros1, there is some ros2 adaptation but I rather using original.

3. Dynamic - IMU-Camera calibration, (Kalibr)



## Links

- [Kalibr](https://github.com/ethz-asl/kalibr)
- [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)
- [openVINS docs](https://docs.openvins.com/gs-calibration.html)