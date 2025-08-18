#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/interpolated_path_markers', 10)

        self.get_logger().info("Visualizing interpolated path from: /plan")

    def path_callback(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.ns = "interpolated_path"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.1
        marker.color.a = 1.0
        marker.lifetime.sec = 0

        marker.points = [pose.pose.position for pose in msg.poses]
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
