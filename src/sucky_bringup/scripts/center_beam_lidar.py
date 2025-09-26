#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
import math

class CenterBeamDual(Node):
    def __init__(self):
        super().__init__('center_beam_dual_timer')

        # Create independent callback groups for each subscriber
        self.lidar_group = ReentrantCallbackGroup()
        self.depth_group = ReentrantCallbackGroup()

        # Subscribers
        self.sub_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10,
            callback_group=self.lidar_group
        )

        self.sub_depth = self.create_subscription(
            LaserScan,
            '/scan_depth',
            self.depth_callback,
            10,
            callback_group=self.depth_group
        )

        # Store latest distances
        self.lidar_distance = None
        self.depth_distance = None

        # Timer at 2 Hz
        self.timer = self.create_timer(0.5, self.timer_callback)  # 0.5 sec = 2 Hz

        self.get_logger().info("Center beam monitor started. Subscribed to /scan and /scan_depth")

    def lidar_callback(self, msg: LaserScan):
        """Store center beam distance from lidar"""
        if len(msg.ranges) == 0:
            return
        center_index = len(msg.ranges) // 2
        distance = msg.ranges[center_index]
        if math.isfinite(distance):
            self.lidar_distance = distance
        else:
            self.lidar_distance = None

    def depth_callback(self, msg: LaserScan):
        """Store center beam distance from depth scan"""
        if len(msg.ranges) == 0:
            return
        center_index = len(msg.ranges) // 2
        distance = msg.ranges[center_index]
        if math.isfinite(distance):
            self.depth_distance = distance
        else:
            self.depth_distance = None

    def timer_callback(self):
        """Print both distances at a steady 2 Hz"""
        lidar_str = f"{self.lidar_distance:.3f} m" if self.lidar_distance is not None else "No data"
        depth_str = f"{self.depth_distance:.3f} m" if self.depth_distance is not None else "No data"

        self.get_logger().info(f"[LIDAR] {lidar_str} | [DEPTH] {depth_str}")

def main():
    rclpy.init()
    node = CenterBeamDual()

    # Use a MultiThreadedExecutor to handle multiple callback groups properly
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
