#!/usr/bin/env python3

import csv
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ScanDegeneracyNode(Node):

    def __init__(self):
        super().__init__('scan_degeneracy_node')

        self.sub = self.create_subscription(
            PointCloud2,
            '/points',
            self.pointcloud_callback,
            10
        )

        self.warning_active = False

        self.csv_file = open('/home/anubhav/Documents/pace/data/scan_degeneracy.csv', 'w', buffering=1)

        self.csv_writer = csv.writer(self.csv_file)

        if self.csv_file.tell() == 0:
            self.csv_writer.writerow([
                'timestamp',
                'points',
                'Ixx',
                'Iyy',
                'Iyaw',
                'lambda_min',
                'lambda_max',
                'condition_number',
                'degenerate',
                'warning'
            ])

        self.get_logger().info(
            'Scan degeneracy detector started.'
        )

    def pointcloud_callback(self, msg):

        points = []

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x, y, z = p

            # Select points belonging to the corridor walls.
            if abs(abs(y) - 2.0) < 0.30:
                points.append((x, y))

        if len(points) < 20:
            return

        H = np.zeros((3, 3))

        for x, y in points:

            # Approximate wall normal.
            if y > 0:
                nx = 0.0
                ny = 1.0
            else:
                nx = 0.0
                ny = -1.0

            # Jacobian with respect to [x, y, yaw].
            J = np.array([
                nx,
                ny,
                nx * (-y) + ny * x
            ])

            H += np.outer(J, J)

        H_reg = H + 1e-9 * np.eye(3)

        eigenvalues = np.linalg.eigvalsh(H_reg)

        lambda_min = float(np.min(eigenvalues))
        lambda_max = float(np.max(eigenvalues))

        condition_number = (
            lambda_max / max(lambda_min, 1e-9)
        )

        Ixx = float(H[0, 0])
        Iyy = float(H[1, 1])
        Iyaw = float(H[2, 2])

        # Axial degeneracy criterion.
        degenerate = Ixx < 1e-3

        timestamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        warning = ''

        # Log warning only when entering degeneracy.
        if degenerate and not self.warning_active:

            warning = 'LOCALIZATION_DEGENERACY_WARNING'

            self.get_logger().warn(
                'LOCALIZATION_DEGENERACY_WARNING: '
                'Low information along corridor X axis.'
            )

        self.warning_active = degenerate

        self.csv_writer.writerow([
            timestamp,
            len(points),
            Ixx,
            Iyy,
            Iyaw,
            lambda_min,
            lambda_max,
            condition_number,
            int(degenerate),
            warning
        ])

    def destroy_node(self):

        self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = ScanDegeneracyNode()

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
