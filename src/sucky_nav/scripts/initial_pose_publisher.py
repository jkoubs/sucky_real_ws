#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(0.5, self.check_and_publish)
        self.has_published = False

    def check_and_publish(self):
        if self.pub.get_subscription_count() > 0 and not self.has_published:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = 0.0
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.068

            self.pub.publish(msg)
            self.get_logger().info('Initial pose published (after waiting for AMCL).')
            self.has_published = True
            self.timer.cancel()
            rclpy.shutdown()
        else:
            self.get_logger().info('Waiting for AMCL to subscribe to /initialpose...')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
