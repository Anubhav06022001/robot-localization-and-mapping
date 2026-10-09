#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class EKFLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization_node')

        # ============================================================
        # State:
        # [x, y, yaw, velocity, gyro_bias]
        # ============================================================
        self.x = np.zeros(5)

        self.P = np.diag([
            0.05,      # x uncertainty
            0.05,      # y uncertainty
            0.02,      # yaw uncertainty
            0.10,      # velocity uncertainty
            0.001      # gyro bias uncertainty
        ])

        # Process noise
        self.Q = np.diag([
            0.001,     # x
            0.001,     # y
            0.0005,    # yaw
            0.01,      # velocity
            1e-6       # gyro bias
        ])

        # Velocity measurement noise
        self.R_v = np.array([
            [0.02 ** 2]
        ])

        self.initialized = False
        self.last_time = None

        # Latest degraded gyro measurement
        self.gyro_z = 0.0

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            '/slipping_odom',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/degraded_imu',
            self.imu_callback,
            10
        )

        # ------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------

        self.fused_pub = self.create_publisher(
            Odometry,
            '/fused_odom',
            10
        )

        # ------------------------------------------------------------
        # Covariance logging
        # ------------------------------------------------------------

        self.csv_file = open('/home/anubhav/Documents/pace/data/ekf_covariance.csv', 'w', buffering=1)

        self.csv_file.write('time,Pxx,Pyy,Pyaw,Pvv,Pbb\n')

        self.get_logger().info( 'EKF localization node started.' )

    # ================================================================
    # IMU CALLBACK
    # ================================================================

    def imu_callback(self, msg):

        self.gyro_z = msg.angular_velocity.z

    # ================================================================
    # ODOM CALLBACK
    # ================================================================

    def odom_callback(self, msg):

        current_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        # ------------------------------------------------------------
        # Initial state
        # ------------------------------------------------------------

        if not self.initialized:

            self.x[0] = msg.pose.pose.position.x
            self.x[1] = msg.pose.pose.position.y

            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            self.x[2] = 2.0 * np.arctan2(qz, qw)

            self.x[3] = msg.twist.twist.linear.x

            self.x[4] = 0.0

            self.last_time = current_time

            self.initialized = True

            return

        # ------------------------------------------------------------
        # Time step
        # ------------------------------------------------------------

        dt = current_time - self.last_time

        self.last_time = current_time

        dt = np.clip(dt, 1e-4, 0.1)

        # ------------------------------------------------------------
        # EKF prediction
        # ------------------------------------------------------------

        self.predict(dt)

        # ------------------------------------------------------------
        # Velocity measurement update
        # ------------------------------------------------------------

        measured_velocity = msg.twist.twist.linear.x

        self.update_velocity(measured_velocity)

        # ------------------------------------------------------------
        # Publish
        # ------------------------------------------------------------

        self.publish_state(msg.header.stamp)

    # ================================================================
    # PREDICTION
    # ================================================================

    def predict(self, dt):

        px, py, yaw, v, bias = self.x

        # Correct gyro using estimated bias

        omega = self.gyro_z - bias

        # ------------------------------------------------------------
        # Motion model
        # ------------------------------------------------------------

        self.x[0] = px + v * np.cos(yaw) * dt

        self.x[1] = py + v * np.sin(yaw) * dt

        self.x[2] = yaw + omega * dt

        self.x[3] = v

        self.x[4] = bias

        # ------------------------------------------------------------
        # Jacobian F
        # ------------------------------------------------------------

        F = np.eye(5)

        F[0, 2] = -v * np.sin(yaw) * dt
        F[0, 3] =  np.cos(yaw) * dt

        F[1, 2] =  v * np.cos(yaw) * dt
        F[1, 3] =  np.sin(yaw) * dt

        F[2, 4] = -dt

        # ------------------------------------------------------------
        # Covariance prediction
        # ------------------------------------------------------------

        self.P = (
            F @ self.P @ F.T
            + self.Q * dt
        )

        # Numerical symmetry
        self.P = 0.5 * (
            self.P + self.P.T
        )

    # ================================================================
    # VELOCITY UPDATE
    # ================================================================

    def update_velocity(self, measured_velocity):

        # Measurement:
        #
        # z = velocity
        #
        # h(x) = v

        H = np.array([
            [0.0, 0.0, 0.0, 1.0, 0.0]
        ])

        z = np.array([
            [measured_velocity]
        ])

        h = np.array([
            [self.x[3]]
        ])

        # ------------------------------------------------------------
        # Innovation
        # ------------------------------------------------------------

        innovation = z - h

        # ------------------------------------------------------------
        # Innovation covariance
        # ------------------------------------------------------------

        S = (
            H @ self.P @ H.T
            + self.R_v
        )

        # ------------------------------------------------------------
        # Kalman gain
        # ------------------------------------------------------------

        K = (
            self.P @ H.T
            @ np.linalg.inv(S)
        )

        # ------------------------------------------------------------
        # State update
        # ------------------------------------------------------------

        self.x = (
            self.x
            + (K @ innovation).flatten()
        )

        # ------------------------------------------------------------
        # Joseph covariance update
        # ------------------------------------------------------------

        I = np.eye(5)

        self.P = (
            (I - K @ H)
            @ self.P
            @ (I - K @ H).T
            + K @ self.R_v @ K.T
        )

        self.P = 0.5 * (
            self.P + self.P.T
        )

    # ================================================================
    # PUBLISH
    # ================================================================

    def publish_state(self, stamp):

        msg = Odometry()

        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Position

        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]

        # Yaw -> quaternion

        half_yaw = self.x[2] * 0.5

        msg.pose.pose.orientation.z = np.sin(
            half_yaw
        )

        msg.pose.pose.orientation.w = np.cos(
            half_yaw
        )

        # Velocity

        msg.twist.twist.linear.x = self.x[3]

        msg.twist.twist.angular.z = (
            self.gyro_z - self.x[4]
        )

        # ------------------------------------------------------------
        # Pose covariance
        # ------------------------------------------------------------

        covariance = np.zeros(36)

        covariance[0] = self.P[0, 0]      # x
        covariance[7] = self.P[1, 1]      # y
        covariance[35] = self.P[2, 2]     # yaw

        msg.pose.covariance = covariance.tolist()

        self.fused_pub.publish(msg)

        # ------------------------------------------------------------
        # CSV
        # ------------------------------------------------------------

        t = (
            stamp.sec
            + stamp.nanosec * 1e-9
        )

        self.csv_file.write(
            f'{t},'
            f'{self.P[0,0]},'
            f'{self.P[1,1]},'
            f'{self.P[2,2]},'
            f'{self.P[3,3]},'
            f'{self.P[4,4]}\n'
        )

    # ================================================================
    # CLEANUP
    # ================================================================

    def destroy_node(self):

        self.csv_file.close()

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = EKFLocalization()

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