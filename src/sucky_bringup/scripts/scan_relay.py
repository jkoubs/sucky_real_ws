#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

def sensor_qos(depth: int) -> QoSProfile:
    q = QoSProfile(depth=depth)
    q.reliability = QoSReliabilityPolicy.BEST_EFFORT
    q.durability  = QoSDurabilityPolicy.VOLATILE
    q.history     = HistoryPolicy.KEEP_LAST
    return q

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')

        # params (so you can override from launch)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_be')
        self.declare_parameter('sub_depth', 10)
        self.declare_parameter('pub_depth', 5)

        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        sub_depth = self.get_parameter('sub_depth').get_parameter_value().integer_value
        pub_depth = self.get_parameter('pub_depth').get_parameter_value().integer_value

        sub_qos = sensor_qos(max(1, sub_depth))
        pub_qos = sensor_qos(max(1, pub_depth))

        self.pub = self.create_publisher(LaserScan, out_topic, pub_qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self.pub.publish, sub_qos)

        self.get_logger().info(f"Relaying {in_topic} → {out_topic} "
                               f"(sub depth={sub_qos.depth}, pub depth={pub_qos.depth}, "
                               "QoS=BEST_EFFORT/VOLATILE/KEEP_LAST)")

def main():
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
