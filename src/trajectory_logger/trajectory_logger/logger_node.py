#!/usr/bin/env python3

import csv

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TrajectoryLogger(Node):

    def __init__(self):
        super().__init__('trajectory_logger')

        self.gt = None
        self.slip = None
        self.fused = None

        self.csv_file = open('/home/anubhav/Documents/pace/data/trajectory_comparison.csv', 'w', buffering=1)

        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([ 'timestamp',  'gt_x', 'gt_y',  'slip_x',   'slip_y',    'fused_x', 'fused_y' ])

        self.create_subscription( Odometry, '/odom', self.gt_callback, 10  )

        self.create_subscription( Odometry,'/slipping_odom', self.slip_callback, 10 )

        self.create_subscription( Odometry,  '/fused_odom', self.fused_callback,   10 )

        self.timer = self.create_timer( 0.05, self.write_data )

        self.get_logger().info('Trajectory logger started.'  )

    def gt_callback(self, msg):
        self.gt = msg

    def slip_callback(self, msg):
        self.slip = msg

    def fused_callback(self, msg):
        self.fused = msg

    def write_data(self):

        if self.gt is None:
            return

        if self.slip is None:
            return

        if self.fused is None:
            return

        stamp = self.gt.header.stamp

        timestamp = (
            stamp.sec +
            stamp.nanosec * 1e-9
        )

        self.writer.writerow([
            timestamp,

            self.gt.pose.pose.position.x,
            self.gt.pose.pose.position.y,

            self.slip.pose.pose.position.x,
            self.slip.pose.pose.position.y,

            self.fused.pose.pose.position.x,
            self.fused.pose.pose.position.y
        ])

    def destroy_node(self):

        self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryLogger()

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
