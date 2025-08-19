#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time as TimeMsg

def sensor_qos(depth=5):
    q = QoSProfile(depth=depth)
    q.reliability = QoSReliabilityPolicy.BEST_EFFORT
    q.history = QoSHistoryPolicy.KEEP_LAST
    return q

class ScanVizRelay(Node):
    def __init__(self):
        super().__init__('scan_viz_relay')

        # Parameters
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_viz')
        self.declare_parameter('target_hz', 8.0)           # throttle to ~8 Hz
        self.declare_parameter('decimate', 1)              # take every Nth beam
        self.declare_parameter('restamp', True)            # stamp with "now"
        self.declare_parameter('frame_id_override', '')    # keep empty to use input frame
        self.declare_parameter('qos_depth', 5)

        self.input_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_hz    = float(self.get_parameter('target_hz').value)
        self.decimate     = max(1, int(self.get_parameter('decimate').value))
        self.restamp      = bool(self.get_parameter('restamp').value)
        self.frame_id_override = self.get_parameter('frame_id_override').get_parameter_value().string_value
        qos_depth         = int(self.get_parameter('qos_depth').value)

        self.min_period = 1.0 / max(0.1, self.target_hz)
        self.last_pub_time = None

        qos = sensor_qos(depth=qos_depth)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self.cb, qos)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)

        self.get_logger().info(
            f"Relaying {self.input_topic} -> {self.output_topic} at ≤{self.target_hz:.1f} Hz, "
            f"decimate={self.decimate}, restamp={self.restamp}, qos_depth={qos_depth}"
        )

    def _now_msg(self) -> TimeMsg:
        now = self.get_clock().now().to_msg()
        return now

    def cb(self, msg: LaserScan):
        # Throttle by time
        now = self.get_clock().now()
        if self.last_pub_time is not None:
            if (now - self.last_pub_time).nanoseconds < self.min_period * 1e9:
                return

        out = LaserScan()
        out.header = msg.header
        if self.restamp:
            out.header.stamp = self._now_msg()
        if self.frame_id_override:
            out.header.frame_id = self.frame_id_override

        # Copy scalar fields
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        # Decimate beams if requested
        if self.decimate > 1:
            out.angle_increment = msg.angle_increment * self.decimate
            out.ranges = msg.ranges[::self.decimate]
            if msg.intensities:
                out.intensities = msg.intensities[::self.decimate]
            # Recompute angle_max to be consistent with new count
            if out.ranges:
                out.angle_max = out.angle_min + out.angle_increment * (len(out.ranges) - 1)
        else:
            out.angle_increment = msg.angle_increment
            out.ranges = msg.ranges
            out.intensities = msg.intensities

        self.pub.publish(out)
        self.last_pub_time = now

def main():
    rclpy.init()
    node = ScanVizRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
