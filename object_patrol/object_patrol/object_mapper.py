"""検出結果をLiDARフュージョンで地図座標に変換し、記録・出力するノード。

カメラ方位角 + その方向のLiDAR距離 → base_link座標 → TFでmap座標へ。
巡回完了時に/object_mapper/finalizeサービスでレポートと物体マップ画像を出力する。
"""
import os

import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PointStamped変換の登録に必要)
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from object_patrol.fusion import bearing_range_to_xy, pixel_to_bearing, scan_range_at
from object_patrol.registry import ObjectRegistry
from object_patrol_msgs.msg import DetectedObjects

HFOV = 1.085595  # waffle_piカメラ水平画角[rad]
IMG_WIDTH = 640
MAX_RANGE = 3.5  # これより遠い検出は捨てる(遠距離のまぐれ検出が誤登録の温床)
WINDOW_DEG = 2.0  # LiDAR距離抽出の方位角窓(細いポールで壁ビーム混入を減らす)


class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')
        self.declare_parameter('report_dir', os.path.expanduser('~/ros2_ws/object_report'))
        self.declare_parameter('map_yaml', os.path.expanduser('~/ros2_ws/maps/house_sealed_final.yaml'))
        # 0.8/4: 統合実行1回目の実測調整(0.5/3では重複クラスタとまぐれ誤登録が残った)
        self.registry = ObjectRegistry(merge_dist=0.8, confirm_frames=4)
        self.last_scan = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 5)
        self.create_subscription(DetectedObjects, '/detected_objects', self.det_cb, 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 1)
        self.create_service(Trigger, '/object_mapper/finalize', self.finalize_cb)
        self.get_logger().info('object_mapper ready')

    def scan_cb(self, msg):
        self.last_scan = msg

    def det_cb(self, msg):
        if self.last_scan is None:
            return
        scan = self.last_scan
        updated = False
        for d in msg.objects:
            bearing = pixel_to_bearing((d.x1 + d.x2) / 2.0, IMG_WIDTH, HFOV)
            rng = scan_range_at(list(scan.ranges), scan.angle_min,
                                scan.angle_increment, bearing, window_deg=WINDOW_DEG)
            if rng is None or rng > MAX_RANGE:
                continue
            x, y = bearing_range_to_xy(bearing, rng)
            pt = PointStamped()
            pt.header.frame_id = 'base_link'
            pt.header.stamp = msg.header.stamp  # 画像時刻のTFで変換(回転中のずれ防止)
            pt.point.x, pt.point.y = x, y
            try:
                mp = self.tf_buffer.transform(pt, 'map', timeout=Duration(seconds=0.3))
            except Exception as e:  # Lookup/Extrapolation/Connectivity
                self.get_logger().debug(f'tf failed: {e}')
                continue
            o = self.registry.observe(d.class_name, mp.point.x, mp.point.y)
            if o['confirmed']:
                updated = True
                self.get_logger().info(
                    f"confirmed {d.class_name} at ({mp.point.x:.2f}, {mp.point.y:.2f}) "
                    f"count={o['count']}")
        if updated:
            self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for i, o in enumerate(self.registry.confirmed()):
            m = Marker()
            m.header.frame_id = 'map'
            m.id = i
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y = o['x'], o['y']
            m.pose.position.z = 0.3
            m.scale.x = m.scale.y = 0.3
            m.scale.z = 0.6
            m.color.r, m.color.g, m.color.a = 1.0, 0.3, 0.9
            arr.markers.append(m)
            t = Marker()
            t.header.frame_id = 'map'
            t.id = 100 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.text = o['class_name']
            t.pose.position.x, t.pose.position.y = o['x'], o['y']
            t.pose.position.z = 0.9
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            arr.markers.append(t)
        self.marker_pub.publish(arr)

    def finalize_cb(self, req, res):
        outdir = self.get_parameter('report_dir').value
        os.makedirs(outdir, exist_ok=True)
        objs = self.registry.confirmed()
        lines = ['# 物体探索レポート', '', f'発見物体: {len(objs)}個', '',
                 '| クラス | x [m] | y [m] | 観測回数 |', '|---|---|---|---|']
        for o in objs:
            lines.append(f"| {o['class_name']} | {o['x']:.2f} | {o['y']:.2f} | {o['count']} |")
        with open(os.path.join(outdir, 'report.md'), 'w') as f:
            f.write('\n'.join(lines) + '\n')
        try:
            self.draw_object_map(outdir, objs)
        except Exception as e:
            self.get_logger().error(f'object map drawing failed: {e}')
        res.success = True
        res.message = f'{len(objs)} objects reported to {outdir}'
        self.get_logger().info(res.message)
        return res

    def draw_object_map(self, outdir, objs):
        """地図PGMに発見物体を描き込んだPNGを出力する。"""
        import cv2
        import yaml
        map_yaml = self.get_parameter('map_yaml').value
        with open(map_yaml) as f:
            info = yaml.safe_load(f)
        img = cv2.imread(os.path.join(os.path.dirname(map_yaml), info['image']))
        resolution = info['resolution']
        ox, oy = info['origin'][0], info['origin'][1]
        h = img.shape[0]
        for o in objs:
            px = int((o['x'] - ox) / resolution)
            py = h - 1 - int((o['y'] - oy) / resolution)
            cv2.circle(img, (px, py), 6, (0, 0, 255), -1)
            cv2.putText(img, o['class_name'], (px + 8, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        cv2.imwrite(os.path.join(outdir, 'object_map.png'), img)


def main():
    rclpy.init()
    rclpy.spin(ObjectMapper())


if __name__ == '__main__':
    main()
