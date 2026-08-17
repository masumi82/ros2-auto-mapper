"""waypointを順に巡回し、各点で360度見回す。完了後にobject_mapperへ集計を指示。"""
import math
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

STATUS_SUCCEEDED = 4


class PatrolNode(Node):
    SPIN_SPEED = 0.5  # rad/s(検出のため低速)

    def __init__(self):
        super().__init__('patrol_node')
        self.declare_parameter('points_file', 'patrol_points.yaml')
        with open(self.get_parameter('points_file').value) as f:
            self.points = yaml.safe_load(f)['patrol_points']
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.finalize_cli = self.create_client(Trigger, '/object_mapper/finalize')

    def run(self):
        self.nav.wait_for_server()
        for p in self.points:
            ok = self.goto(p) or self.goto(p)  # リトライ1回
            if not ok:
                self.get_logger().warn(f"skip waypoint {p['name']} (aborted twice)")
                continue
            self.get_logger().info(f"arrived {p['name']}, spinning")
            self.spin_around()
        self.get_logger().info('patrol finished, requesting finalize')
        if self.finalize_cli.wait_for_service(timeout_sec=10.0):
            fut = self.finalize_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            if fut.done() and fut.result() is not None:
                self.get_logger().info(f'finalize: {fut.result().message}')
        else:
            self.get_logger().warn('finalize service not available')

    def goto(self, p):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(p['x'])
        goal.pose.pose.position.y = float(p['y'])
        goal.pose.pose.orientation.z = math.sin(float(p['yaw']) / 2)
        goal.pose.pose.orientation.w = math.cos(float(p['yaw']) / 2)
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=300.0)
        return res.done() and res.result().status == STATUS_SUCCEEDED

    def spin_around(self):
        twist = Twist()
        twist.angular.z = self.SPIN_SPEED
        end = time.monotonic() + 2 * math.pi / self.SPIN_SPEED
        while time.monotonic() < end:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())  # 停止


def main():
    rclpy.init()
    node = PatrolNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
