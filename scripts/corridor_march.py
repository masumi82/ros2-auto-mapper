#!/usr/bin/env python3
"""Corridor march: drive the robot through a narrow corridor with direct Nav2 goals.

The frontier explorer cannot pick up frontiers inside very narrow corridors
(the clearance search around the frontier cells fails), so exploration can
stall with the east half of the house unmapped. Running this script pushes
the robot through the corridor; once the far side opens up, restart
auto_explorer and it takes over with normal frontier exploration.

Usage:
    pkill -9 -f "auto_explore[r]"          # stop the explorer first
    python3 corridor_march.py               # default TB3-house waypoints
    ros2 run auto_mapper auto_explorer ...  # then restart the explorer
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

# Waypoints through the TB3-house east corridor (map frame)
WAYPOINTS = [(0.6, 0.4), (1.6, 0.4), (3.2, 0.2), (4.2, 0.2)]


def main():
    rclpy.init()
    n = Node('corridor_march')
    pose = [None]
    n.create_subscription(
        Odometry, '/odom',
        lambda m: pose.__setitem__(0, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    ac.wait_for_server(timeout_sec=10)

    def go(x, y, tmax=110):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.w = 1.0
        f = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, f, timeout_sec=10)
        h = f.result()
        if h is None or not h.accepted:
            print(f'({x},{y}): rejected')
            return False
        rf = h.get_result_async()
        t = time.time() + tmax
        while time.time() < t and not rf.done():
            rclpy.spin_once(n, timeout_sec=0.3)
        if not rf.done():
            h.cancel_goal_async()
            print(f'({x},{y}): timeout pos={pose[0]}')
            return False
        st = rf.result().status
        print(f'({x},{y}): status={st} pos={pose[0]}')
        return st == 4  # SUCCEEDED

    for wp in WAYPOINTS:
        go(*wp)
    print('march done')


if __name__ == '__main__':
    main()
