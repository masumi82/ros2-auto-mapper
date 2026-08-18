"""スポーン座標をAMCL初期位置として、map→odom TFが出るまで配信し続けるノード。

AMCLの起動タイミングに依存しないよう、TF成立の確認をもって終了する
(統合実行2回目で「initialposeがAMCL起動前に流れて全ゴール拒否」が発生した対策)。
"""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time


class SetInitialPose(Node):
    TIMEOUT_TICKS = 90  # 1秒周期

    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('x', -3.0)
        self.declare_parameter('y', 2.0)
        self.declare_parameter('yaw', 0.0)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.count = 0
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        self.count += 1
        if self.count > self.TIMEOUT_TICKS:
            self.get_logger().error('map->odom TF did not appear, giving up')
            raise SystemExit
        # 数回送った後は、TFが成立していれば完了
        if self.count > 3 and self.tf_buffer.can_transform('map', 'odom', Time()):
            self.get_logger().info(f'map->odom TF confirmed after {self.count}s')
            raise SystemExit
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


def main():
    rclpy.init()
    try:
        rclpy.spin(SetInitialPose())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
