import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String as StringMsg


class AutoExplorer(Node):
    """Fully autonomous mapping: repeatedly picks a reachable free cell near
    the frontier of the unknown, navigates there, and stops when the map has
    stopped growing — then saves the finished map. No human input needed."""

    def __init__(self):
        super().__init__('auto_explorer')
        self.declare_parameter('max_goals', 20)
        self.declare_parameter('clearance_cells', 5)   # 5 * 0.05m = 0.25m
        self.declare_parameter('min_goal_dist', 1.0)   # meters
        self.declare_parameter('min_growth', 150)      # (kept for compatibility)
        self.declare_parameter('dry_limit', 2)         # (kept for compatibility)
        self.declare_parameter('min_cluster', 8)       # frontier cells to count as a real opening
        self.declare_parameter('mode', 'frontier')     # 'frontier' | 'coverage'
        self.declare_parameter('grid_step', 1.0)       # coverage grid spacing (m)
        self.declare_parameter('sweep_speed', 0.25)    # rad/s; slow keeps SLAM accurate
        self.declare_parameter('map_save_name',
                               '/home/m-horiuchi/ros2_ws/maps/auto_explored')

        self.map_msg = None
        self.pose = (0.0, 0.0)
        self.yaw = 0.0
        self.scan = None
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.saver = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def look_around(self, turns=1.0):
        """Rotate in place so the LiDAR sweeps the surroundings into the map
        (what a robot vacuum does right after starting). Slow rotation keeps
        SLAM's scan matching accurate — fast spins create ghost walls."""
        speed = float(self.get_parameter('sweep_speed').value or 0.25)
        sec = turns * 2 * math.pi / speed
        self.get_logger().info(f'looking around ({turns:.0f} turn, {sec:.0f}s)...')
        t = Twist()
        t.angular.z = speed
        end = time.time() + sec
        while time.time() < end:
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cmd_pub.publish(Twist())  # stop

    def on_map(self, m):
        self.map_msg = m

    def on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))

    def on_scan(self, m):
        self.scan = m

    def ray_goal(self, blacklist=()):
        """User's idea: head where the 360-degree lidar gets no echo.
        A bundle of max-range rays = an opening (doorway / unexplored space).
        Sensor-space frontier finding — catches openings the map-space
        clustering misses."""
        s = self.scan
        if s is None:
            return None
        n = len(s.ranges)
        # clip no-echo rays to max range, then smooth over a 5-ray window and
        # head toward the farthest-looking direction (always exists — this
        # doubles as the bootstrap when the map is still tiny)
        r = [min(x, s.range_max) if not math.isinf(x) and x > 0.05 else s.range_max
             for x in s.ranges]
        best_i, best_v = None, 0.0
        for i in range(n):
            v = min(r[(i + k) % n] for k in (-2, -1, 0, 1, 2))
            if v > best_v:
                best_v, best_i = v, i
        if best_i is None or best_v < 1.2:     # boxed in — nothing useful
            return None
        angle = s.angle_min + best_i * s.angle_increment + self.yaw
        step = min(best_v - 0.5, 2.0)
        gx = self.pose[0] + step * math.cos(angle)
        gy = self.pose[1] + step * math.sin(angle)
        if any(math.dist((gx, gy), b) < 0.6 for b in blacklist):
            return None
        return (gx, gy)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.3)

    def known_cells(self):
        return sum(1 for c in self.map_msg.data if c >= 0) if self.map_msg else 0

    def pick_goal(self, blacklist=()):
        """Textbook frontier exploration: enumerate all frontier cells
        (free cells touching unknown), cluster them, and head for the most
        attractive cluster (big and near). Returns None when no frontier
        clusters remain — which IS the map-complete condition."""
        m = self.map_msg
        if m is None:
            return None
        cl = int(self.get_parameter('clearance_cells').value or 5)
        min_cluster = int(self.get_parameter('min_cluster').value or 8)
        info, data = m.info, m.data
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        rx, ry = self.pose

        def val(cx, cy):
            return data[cy * w + cx] if 0 <= cx < w and 0 <= cy < h else 100

        def free(cx, cy):
            return 0 <= val(cx, cy) <= 10

        # 1) all frontier cells: free, with an unknown 4-neighbour
        frontier = set()
        for cy in range(1, h - 1):
            for cx in range(1, w - 1):
                if free(cx, cy) and any(
                        val(cx + dx, cy + dy) == -1
                        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                    frontier.add((cx, cy))
        if not frontier:
            return None

        # 2) cluster them (BFS over 8-neighbourhood)
        clusters = []
        seen = set()
        for cell in frontier:
            if cell in seen:
                continue
            stack, cluster = [cell], []
            seen.add(cell)
            while stack:
                c = stack.pop()
                cluster.append(c)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nb = (c[0] + dx, c[1] + dy)
                        if nb in frontier and nb not in seen:
                            seen.add(nb)
                            stack.append(nb)
            if len(cluster) >= min_cluster:
                clusters.append(cluster)
        if not clusters:
            return None

        # 3) goal per cluster: a clear free cell near its centroid;
        #    score prefers big clusters that are close by
        def clear(cx, cy):
            return all(free(cx + dx, cy + dy)
                       for dx in (-cl, 0, cl) for dy in (-cl, 0, cl))

        best, best_score = None, -1.0
        for cluster in clusters:
            mx = sum(c[0] for c in cluster) / len(cluster)
            my = sum(c[1] for c in cluster) / len(cluster)
            # goal = the frontier cell nearest the cluster centre: standing ON
            # the boundary lets the 360-degree lidar expose the space behind it
            goal_cell = min(cluster,
                            key=lambda c: (c[0] - mx) ** 2 + (c[1] - my) ** 2)
            if not clear(goal_cell[0], goal_cell[1]):
                # too tight — search near the FRONTIER CELL (known side), not
                # the centroid: a big arc-shaped cluster's centroid often sits
                # in unknown space where no clear cell can ever be found
                bx, by = goal_cell
                goal_cell = None
                for r in range(1, 12):
                    for dx in range(-r, r + 1):
                        for dy in (-r, r) if abs(dx) != r else range(-r, r + 1):
                            cx, cy = bx + dx, by + dy
                            if free(cx, cy) and clear(cx, cy):
                                goal_cell = (cx, cy)
                                break
                        if goal_cell:
                            break
                    if goal_cell:
                        break
            if goal_cell is None:
                continue
            wx, wy = ox + goal_cell[0] * res, oy + goal_cell[1] * res
            # THE doorway fix: push the goal 0.7 m PAST the frontier into the
            # unknown, along the robot->frontier direction. Nav2 plans through
            # unknown space, so the robot physically crosses the doorway and
            # the room behind it gets revealed. A goal ON the boundary lets
            # the robot stop at the door frame and never enter.
            d0 = math.dist((wx, wy), (rx, ry))
            if d0 > 0.05:
                push = 0.7
                px = wx + (wx - rx) / d0 * push
                py = wy + (wy - ry) / d0 * push
                # only if the pushed point is not a known obstacle
                cxp = int((px - ox) / res)
                cyp = int((py - oy) / res)
                if 0 <= cxp < w and 0 <= cyp < h and data[cyp * w + cxp] <= 10:
                    wx, wy = px, py
            if any(math.dist((wx, wy), b) < 0.6 for b in blacklist):
                continue
            d = math.dist((wx, wy), (rx, ry))
            score = len(cluster) / (1.0 + d)   # big and near wins
            if score > best_score:
                best_score, best = score, (wx, wy)
        return best

    def navigate(self, x, y, timeout=120):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        handle = fut.result()
        if not handle or not handle.accepted:
            return 'rejected'
        rf = handle.get_result_async()
        t0 = time.time()
        while not rf.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return rf.result().status if rf.done() else 'timeout'

    def coverage_explore(self, max_goals):
        """Brute-force coverage: lay a grid of viewpoints over the WHOLE map
        area (unknown space included) and visit each, nearest first. Whether a
        point is reachable is decided entirely by Nav2's planner — if a room
        is connected at all, some grid point inside it will be visited. Ends
        when every point is either visited or rejected."""
        self.spin_for(5)
        m = self.map_msg
        if m is None:
            self.get_logger().error('no map')
            return
        step = float(self.get_parameter('grid_step').value or 1.0)
        info = m.info
        x0, y0 = info.origin.position.x, info.origin.position.y
        x1 = x0 + info.width * info.resolution
        y1 = y0 + info.height * info.resolution

        def cell_val(x, y):
            mm = self.map_msg
            ii = mm.info
            cx = int((x - ii.origin.position.x) / ii.resolution)
            cy = int((y - ii.origin.position.y) / ii.resolution)
            if 0 <= cx < ii.width and 0 <= cy < ii.height:
                return mm.data[cy * ii.width + cx]
            return 100

        pts = []
        y = y0 + 0.4
        while y < y1 - 0.4:
            x = x0 + 0.4
            while x < x1 - 0.4:
                pts.append((x, y))
                x += step
            y += step
        self.get_logger().info(f'coverage grid: {len(pts)} viewpoints')
        visited = rejected = 0
        goals = 0
        while pts and goals < max_goals:
            rx, ry = self.pose
            pts.sort(key=lambda p: math.dist(p, (rx, ry)))
            p = pts.pop(0)
            if cell_val(*p) > 50:            # known obstacle — skip silently
                rejected += 1
                continue
            goals += 1
            self.get_logger().info(
                f'[{goals}/{max_goals}] viewpoint ({p[0]:.2f},{p[1]:.2f}) '
                f'known={self.known_cells()} 残り{len(pts)}点')
            status = self.navigate(*p, timeout=90)
            if status == 4:
                visited += 1
            else:
                rejected += 1
            self.get_logger().info(f'  -> status={status} (4=到達)')
        self.get_logger().info(
            f'coverage done: visited={visited} rejected={rejected} '
            f'known={self.known_cells()}')

    def save_map(self):
        name = str(self.get_parameter('map_save_name').value)
        if not self.saver.wait_for_service(timeout_sec=10):
            self.get_logger().error('map save service unavailable')
            return False
        req = SaveMap.Request()
        req.name = StringMsg(data=name)
        fut = self.saver.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20)
        ok = fut.result() is not None
        self.get_logger().info(f'map saved to {name}: {"OK" if ok else "FAILED"}')
        return ok

    def explore(self):
        max_goals = int(self.get_parameter('max_goals').value or 20)
        min_growth = int(self.get_parameter('min_growth').value or 150)
        if not self.nav.wait_for_server(timeout_sec=30):
            self.get_logger().error('Nav2 action server unavailable')
            return
        if str(self.get_parameter('mode').value) == 'coverage':
            self.coverage_explore(max_goals)
            self.save_map()
            return
        # bootstrap: never judge "complete" before data actually arrives
        # (DDS discovery can take >10 s in a busy graph)
        t0 = time.time()
        while (self.scan is None or self.map_msg is None) and time.time() - t0 < 60:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(
            f'bootstrap: scan={"OK" if self.scan else "NG"} '
            f'map={"OK" if self.map_msg else "NG"} ({time.time()-t0:.0f}s)')
        blacklist = []
        empty_checks = 0
        for i in range(1, max_goals + 1):
            self.spin_for(3)
            goal_xy = self.pick_goal(blacklist=blacklist)
            via = 'frontier'
            if goal_xy is None:
                # map-space says done — double-check in sensor space:
                # head where the lidar gets no echo (user's suggestion)
                goal_xy = self.ray_goal(blacklist=blacklist)
                via = 'lidar-ray'
            if goal_xy is None:
                # WANDER burst: big open interiors return no lidar echo, so
                # SLAM cannot carve them from a distance — the map only grows
                # by physically roaming (what a human does with teleop).
                kb = self.known_cells()
                for k in range(3):
                    g = self.ray_goal(blacklist=[])
                    if g is None:
                        break
                    self.get_logger().info(
                        f'[{i}] wander {k+1}/3 -> ({g[0]:.2f},{g[1]:.2f})')
                    self.navigate(*g, timeout=60)
                    self.spin_for(2)
                grew_wander = self.known_cells() - kb
                self.get_logger().info(f'[{i}] wander grew +{grew_wander}')
                if grew_wander < 60:
                    empty_checks += 1
                    if empty_checks >= 2:
                        self.get_logger().info(
                            'no frontiers, no rays, wander dry -> map complete')
                        break
                else:
                    empty_checks = 0
                continue
            empty_checks = 0
            known = self.known_cells()
            self.get_logger().info(
                f'[{i}/{max_goals}] {via} goal ({goal_xy[0]:.2f},{goal_xy[1]:.2f}) '
                f'known={known}')
            status = self.navigate(*goal_xy)
            self.spin_for(2)
            grew = self.known_cells() - known
            self.get_logger().info(
                f'  -> status={status} (4=SUCCEEDED), grew +{grew}')
            # Blacklist only unproductive goals: failures and visits that
            # revealed nothing. A goal whose trip grew the map stays reusable
            # (a doorway may need several pushes) — permanent blacklisting of
            # every visited goal is what starved the east/south-west rooms.
            if status != 4 or grew < 30:
                blacklist.append(goal_xy)
        self.get_logger().info(f'exploration finished, known={self.known_cells()}')
        self.save_map()


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        node.explore()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
