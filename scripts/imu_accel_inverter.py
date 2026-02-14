#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuAccelInverter(Node):
    def __init__(self):
        super().__init__('imu_accel_inverter')
        self.sub = self.create_subscription(
            Imu, '/f450_1/aircraft/imu', self.callback, 10)
        self.pub = self.create_publisher(
            Imu, '/f450_1/aircraft/imu/accel_invert', 10)

    def callback(self, msg: Imu):
        msg.linear_acceleration.x = -msg.linear_acceleration.x
        msg.linear_acceleration.y = -msg.linear_acceleration.y
        msg.linear_acceleration.z = -msg.linear_acceleration.z
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuAccelInverter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
