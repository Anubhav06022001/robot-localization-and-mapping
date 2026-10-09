#!/usr/bin/env python3

import math
import csv
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class DynamicMapNode(Node):

    def __init__(self):
        super().__init__('dynamic_map_node')

        # A map cell is removed if it has not been observed
        # for this amount of time.
        self.declare_parameter('stale_timeout', 3.0)

        self.stale_timeout = float(
            self.get_parameter('stale_timeout').value
        )

        # Quantization resolution of the local map.
        self.declare_parameter('resolution', 0.10)

        self.resolution = float(
            self.get_parameter('resolution').value
        )

        # key = (x_cell, y_cell)
        # value = last observation timestamp
        self.map_cells = {}

        self.sub = self.create_subscription(
            PointCloud2,
            '/points',
            self.pointcloud_callback,
            10
        )

        self.csv_file = open( '/home/anubhav/Documents/pace/data/map_update.csv', 'a',buffering=1)

        self.csv_writer = csv.writer(self.csv_file)

        if self.csv_file.tell() == 0:
            self.csv_writer.writerow([
                'timestamp',
                'observed_cells',
                'map_cells',
                'removed_cells'
            ])

        self.get_logger().info(
            'Dynamic map update node started.'
        )

        self.get_logger().info(
            f'resolution={self.resolution} m, '
            f'stale_timeout={self.stale_timeout} s'
        )

    def pointcloud_callback(self, msg):

        timestamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        observed_cells = set()

        # Read LiDAR points.
        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):

            x, y, z = p

            # Ignore points outside a useful local region.
            if math.sqrt(x * x + y * y) > 10.0:
                continue

            # Ground filtering.
            if z < -0.2 or z > 2.5:
                continue

            # Quantize point into map cell.
            cell_x = int(
                math.floor(x / self.resolution)
            )

            cell_y = int(
                math.floor(y / self.resolution)
            )

            observed_cells.add(
                (cell_x, cell_y)
            )

        # Refresh timestamp for currently observed cells.
        for cell in observed_cells:
            self.map_cells[cell] = timestamp

        # Remove cells that have become stale.
        stale_cells = []

        for cell, last_seen in self.map_cells.items():

            if timestamp - last_seen > self.stale_timeout:
                stale_cells.append(cell)

        for cell in stale_cells:
            del self.map_cells[cell]

        # Log map statistics.
        self.csv_writer.writerow([
            timestamp,
            len(observed_cells),
            len(self.map_cells),
            len(stale_cells)
        ])

        # Only report when stale points are actually removed.
        if stale_cells:

            self.get_logger().info(
                f'Dynamic map update: removed '
                f'{len(stale_cells)} stale cells; '
                f'active map cells={len(self.map_cells)}'
            )

    def destroy_node(self):

        self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DynamicMapNode()

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
