# IMU Data Fusion

When I first tried the algorithm with IMU fusion enabled, it was very error-prone and the algorithm works more efficiently without it.

## Data Preparation

- I performed IMU noise characterization measurements. I used the [Allan ROS2](https://github.com/CruxDevStuff/allan_ros2.git) package. The results are in the f450 configuration file.

- An accurate camera-to-IMU transformation was obtained from the fusion model, where the IMU frame is considered to be the `base_frame`.

- The axis orientation was adjusted to match the ENU coordinate system convention.

## Flight 11.02.2026 (indoor)
Flight with motors off (the drone was held by hand) so that the IMU data would be easy to evaluate. During the flight, I gradually moved the drone along individual axes, first to test the accelerometer and then to test the gyroscope.

The recorded data are evaluated in MATLAB using the `imu_fusion_compare.m` script.

<figure align="center">
  <img src="images/IMU_fusion_ENABLE_timeline.png" alt="IMU Fusion ENABLE timeline">
  <figcaption><i>IMU Fusion ENABLE - position, acceleration and gyroscope timeline</i></figcaption>
</figure>

The figure shows all three axes of VIO odometry over time. During the accelerometer test, the problem with jerky odometry did not manifest. An interesting moment is highlighted by the red line — during the gyroscope test on the y-axis, an unknown error occurred which caused the VIO to fail. From that point on, the computed odometry no longer makes sense.

### Conclusion

- The IMU axis orientation is correct (verified against the camera image).
- The error cannot be conclusively determined from the data, because at the marked moment the RealSense camera was pointing towards the ceiling lights. It is therefore possible that the algorithm simply prioritized the IMU data at that time, which could have also happened at other moments.

## Gravity Vector

A discussion [forum post](https://forums.developer.nvidia.com/t/imu-fusion/316150) states that:

- The gravity vector arrow first appears after successful initialization (camera-to-IMU alignment), which takes 10-20 seconds of camera movement. From this point on, IMU fusion runs continuously.
- The gravity vector arrow reappears when visual tracking is lost and the algorithm subsequently re-aligns the camera and IMU after tracking is restored.                           

After inspecting the gravity error, I discovered that the gravity vector was inverted (pointing up [0 0 9,81]).

## [REP145](https://www.ros.org/reps/rep-0145.html)
ROS2 convetion for IMU.

- When the device is at rest, the vector will represent the specific force solely due to gravity. I.e. if the body z axis points upwards, its z axis should indicate +g. This data must be in m/s^2.

- The rotational velocity is right handed with respect to the body axes, and independent of the orientation of the device. This data must be in rad/s.

I fixed the IMU data based on this convetion and it looks like it works.

