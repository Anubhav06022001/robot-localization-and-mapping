#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class NoiseInjection(Node):

    def __init__(self):
        super().__init__('noise_injection_node')

        # Noise parameters
        self.declare_parameter('sigma_v', 0.02)
        self.declare_parameter('sigma_g', 0.005)
        self.declare_parameter('sigma_b', 0.0005)

        self.sigma_v = self.get_parameter('sigma_v').value
        self.sigma_g = self.get_parameter('sigma_g').value
        self.sigma_b = self.get_parameter('sigma_b').value

        # IMU bias
        self.gyro_bias = 0.0
        self.last_imu_time = None

        # Noisy odometry state
        self.last_odom_time = None
        self.noisy_x = 0.0
        self.noisy_y = 0.0
        self.noisy_yaw = 0.0
        self.odom_initialized = False

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            '/slipping_odom',
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/degraded_imu',
            10
        )

        self.get_logger().info(
            'Noise injection node started.'
        )

    def odom_callback(self, msg):

        current_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        # Initialize noisy trajectory from ground truth
        if not self.odom_initialized:

            self.noisy_x = msg.pose.pose.position.x
            self.noisy_y = msg.pose.pose.position.y

            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            self.noisy_yaw = 2.0 * np.arctan2(qz, qw)

            self.last_odom_time = current_time
            self.odom_initialized = True

            return

        # Time difference
        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        dt = max(1e-4, min(dt, 0.1))

        # Assignment corridor coordinate:
        # world X = -10 -> corridor X = 0
        position_x = msg.pose.pose.position.x + 10.0

        # Velocity scale
        if 10.0 <= position_x <= 15.0:
            scale = 0.70
        else:
            scale = 1.00

        # True velocity
        v_true = msg.twist.twist.linear.x

        # Gaussian velocity noise
        gaussian_noise = np.random.normal(
            0.0,
            self.sigma_v
        )

        # Recorded velocity
        v_recorded = (
            scale * v_true
            + gaussian_noise
        )

        # Angular velocity
        omega = msg.twist.twist.angular.z

        # Integrate corrupted velocity
        self.noisy_x += (
            v_recorded
            * np.cos(self.noisy_yaw)
            * dt
        )

        self.noisy_y += (
            v_recorded
            * np.sin(self.noisy_yaw)
            * dt
        )

        self.noisy_yaw += omega * dt

        # Create noisy odometry message
        noisy_msg = Odometry()

        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id

        noisy_msg.pose.pose.position.x = self.noisy_x
        noisy_msg.pose.pose.position.y = self.noisy_y

        noisy_msg.pose.pose.orientation.z = (
            np.sin(self.noisy_yaw / 2.0)
        )

        noisy_msg.pose.pose.orientation.w = (
            np.cos(self.noisy_yaw / 2.0)
        )

        noisy_msg.twist.twist.linear.x = v_recorded
        noisy_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(noisy_msg)

    def imu_callback(self, msg):

        current_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        dt = max(1e-4, min(dt, 0.1))

        # Random-walk gyro bias
        bias_noise = np.random.normal(
            0.0,
            self.sigma_b * np.sqrt(dt)
        )

        self.gyro_bias += bias_noise

        # Measurement noise
        gyro_noise = np.random.normal(
            0.0,
            self.sigma_g
        )

        measured_gyro_z = (
            msg.angular_velocity.z
            + self.gyro_bias
            + gyro_noise
        )

        # Create degraded IMU
        degraded_msg = msg

        degraded_msg.angular_velocity.z = measured_gyro_z

        self.imu_pub.publish(degraded_msg)


def main(args=None):

    rclpy.init(args=args)

    node = NoiseInjection()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
