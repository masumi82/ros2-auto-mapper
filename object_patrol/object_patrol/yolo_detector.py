"""カメラ画像をYOLOv8nで解析し、検出結果を/detected_objectsに配信するノード。

ultralyticsが必要なため、yolo_venvのpythonで実行する:
  ~/ros2_ws/yolo_venv/bin/python -m object_patrol.yolo_detector
"""
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from object_patrol_msgs.msg import DetectedObject, DetectedObjects


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        # 0.5: Gazebo CGモデルは実写より信頼度が出にくい実測に基づく値
        # (person 0.44-0.93で距離・見え方により変動。誤検出は3フレーム確認で抑制)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('infer_interval_sec', 0.4)
        self.conf = self.get_parameter('conf_threshold').value
        self.interval = self.get_parameter('infer_interval_sec').value
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.last_infer = 0.0
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
        self.pub = self.create_publisher(DetectedObjects, '/detected_objects', 10)
        self.pub_img = self.create_publisher(Image, '/detection_image', 1)
        self.get_logger().info('yolo_detector ready')

    def cb(self, msg):
        now = time.monotonic()
        if now - self.last_infer < self.interval:
            return  # 間引き(WSL2のCPU負荷対策)
        self.last_infer = now
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.model(img, verbose=False)[0]
        out = DetectedObjects()
        out.header = msg.header  # TF時刻参照のため入力画像の時刻を引き継ぐ
        for b in res.boxes:
            if float(b.conf) < self.conf:
                continue
            d = DetectedObject()
            d.class_name = self.model.names[int(b.cls)]
            d.confidence = float(b.conf)
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
            d.x1, d.y1, d.x2, d.y2 = x1, y1, x2, y2
            out.objects.append(d)
        self.pub.publish(out)
        # res.plot()の配列はcv_bridgeが型判定に失敗することがあるため手組みでImage化
        plot = res.plot()
        ann = Image()
        ann.header = msg.header
        ann.height, ann.width = plot.shape[0], plot.shape[1]
        ann.encoding = 'bgr8'
        ann.step = plot.shape[1] * 3
        ann.data = plot.tobytes()
        self.pub_img.publish(ann)


def main():
    rclpy.init()
    rclpy.spin(YoloDetector())


if __name__ == '__main__':
    main()
