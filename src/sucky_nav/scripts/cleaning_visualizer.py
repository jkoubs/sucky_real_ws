#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class CleanedAreaVisualizer(Node):
    def __init__(self):
        super().__init__('cleaned_area_visualizer')

        self.declare_parameter('cleaning_radius', 0.33)
        self.cleaning_radius = self.get_parameter('cleaning_radius').get_parameter_value().double_value

        self.map_metadata = None
        self.cleaned_map = None

        # QoS for latched map topic
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribe to /map with latched QoS
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile=map_qos
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.cleaned_map_pub = self.create_publisher(
            OccupancyGrid,
            '/cleaned_area_map',
            10
        )

        self.get_logger().info('CleanedAreaVisualizer node initialized.')

    def map_callback(self, msg):
        if self.cleaned_map is not None:
            return  # Already initialized

        self.map_metadata = msg.info
        self.cleaned_map = OccupancyGrid()
        self.cleaned_map.header = msg.header
        self.cleaned_map.info = msg.info
        self.cleaned_map.data = [-1] * (msg.info.width * msg.info.height)

        self.get_logger().info(
            f'Map received: size={msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution:.3f}, origin=({msg.info.origin.position.x}, {msg.info.origin.position.y})'
        )

    def pose_callback(self, msg):
        if self.cleaned_map is None:
            self.get_logger().warn('Received pose but map is not initialized yet.')
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        #self.get_logger().info(f'Pose received: x={x:.2f}, y={y:.2f}')

        mx = int((x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
        my = int((y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)

        #self.get_logger().info(f'Converted to map coordinates: mx={mx}, my={my}')

        radius_in_cells = int(self.cleaning_radius / self.map_metadata.resolution)
        updated_cells = 0

        for dx in range(-radius_in_cells, radius_in_cells + 1):
            for dy in range(-radius_in_cells, radius_in_cells + 1):
                if dx**2 + dy**2 <= radius_in_cells**2:
                    cx = mx + dx
                    cy = my + dy
                    if 0 <= cx < self.map_metadata.width and 0 <= cy < self.map_metadata.height:
                        index = cy * self.map_metadata.width + cx
                        if self.cleaned_map.data[index] != 100:
                            self.cleaned_map.data[index] = 100
                            updated_cells += 1

        self.cleaned_map.header.stamp = self.get_clock().now().to_msg()
        self.cleaned_map_pub.publish(self.cleaned_map)

        #self.get_logger().info(f'Updated {updated_cells} cleaned cells and published map.')

def main():
    rclpy.init()
    node = CleanedAreaVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
