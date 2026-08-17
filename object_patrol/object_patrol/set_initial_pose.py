"""スポーン座標をAMCL初期位置として一定回数配信して終了するノード。"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('x', -3.0)
        self.declare_parameter('y', 2.0)
        self.declare_parameter('yaw', 0.0)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.count = 0
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.get_parameter('x').value
        msg.pose.pose.position.y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.pub.publish(msg)
        self.count += 1
        if self.count >= 5:
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(SetInitialPose())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
