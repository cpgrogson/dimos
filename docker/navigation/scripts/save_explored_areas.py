#!/usr/bin/env python3
"""Save /explored_areas PointCloud2 topic to PLY file.

Buffers the latest point cloud and only saves when messages stop (bagfile ends).
This ensures we capture the final accumulated map, not intermediate states.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import sys
import signal
import time


class PointCloudSaver(Node):
    def __init__(self, output_file, idle_timeout=5.0):
        super().__init__('pointcloud_saver')
        self.output_file = output_file
        self.idle_timeout = idle_timeout
        self.last_msg_time = None
        self.last_msg = None  # Buffer the last message, don't save until idle
        self.msg_count = 0
        self.max_points_seen = 0

        self.subscription = self.create_subscription(
            PointCloud2,
            '/explored_areas',
            self.callback,
            10)

        # Timer to check if topic stopped publishing
        self.check_timer = self.create_timer(1.0, self.check_idle)
        self.get_logger().info(f'Listening to /explored_areas (will save to {output_file})')
        self.get_logger().info(f'Will save final map {idle_timeout}s after last message...')

    def check_idle(self):
        if self.last_msg_time is not None:
            elapsed = time.time() - self.last_msg_time
            if elapsed > self.idle_timeout:
                if self.last_msg is not None:
                    self.save_ply(self.last_msg)
                    self.get_logger().info(f'Saved final map: {self.max_points_seen} points')
                    self.get_logger().info(f'Total messages received: {self.msg_count}')
                else:
                    self.get_logger().warn('No messages received, nothing to save.')
                rclpy.shutdown()

    def callback(self, msg):
        self.last_msg_time = time.time()
        self.last_msg = msg  # Buffer latest message
        self.msg_count += 1

        # Track point count for logging
        point_count = msg.width
        if point_count > self.max_points_seen:
            self.max_points_seen = point_count

        if self.msg_count == 1:
            self.get_logger().info(f'First message: {point_count} points')
        elif self.msg_count % 20 == 0:
            self.get_logger().info(f'Message #{self.msg_count}: {point_count} points (max seen: {self.max_points_seen})')

    def save_ply(self, msg):
        """Save PointCloud2 message to PLY file."""
        points = []
        point_step = msg.point_step
        data = msg.data

        for i in range(msg.width):
            offset = i * point_step
            x = struct.unpack_from('f', data, offset)[0]
            y = struct.unpack_from('f', data, offset + 4)[0]
            z = struct.unpack_from('f', data, offset + 8)[0]
            points.append((x, y, z))

        with open(self.output_file, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('end_header\n')
            for p in points:
                f.write(f'{p[0]} {p[1]} {p[2]}\n')


def main():
    if len(sys.argv) < 2:
        print('Usage: save_explored_areas.py <output_file.ply> [idle_timeout_seconds]')
        print('  Buffers /explored_areas messages and saves final map to PLY.')
        print('  Saves after idle_timeout seconds of no messages (default: 5s)')
        sys.exit(1)

    output_file = sys.argv[1]
    idle_timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    rclpy.init()
    node = PointCloudSaver(output_file, idle_timeout)

    # Handle Ctrl+C gracefully - save final map before exit
    def shutdown_handler(sig, frame):
        if node.last_msg is not None:
            node.get_logger().info(f'Interrupted. Saving final map...')
            node.save_ply(node.last_msg)
            node.get_logger().info(f'Saved {node.max_points_seen} points to {output_file}')
        else:
            node.get_logger().warn('Interrupted. No messages received.')
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
