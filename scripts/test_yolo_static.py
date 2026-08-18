#!/usr/bin/env python3
"""YOLO静止認識テスト: /camera/image_raw を1フレーム推論して結果を表示・保存する。

使い方: Gazeboでwaffle_pi+対象モデルを起動した状態で
  ~/ros2_ws/yolo_venv/bin/python scripts/test_yolo_static.py [出力PNG] [フレーム数]
"""
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class StaticTest(Node):
    def __init__(self, out_path, n_frames):
        super().__init__('yolo_static_test')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.out_path = out_path
        self.remaining = n_frames
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 1)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.model(img, conf=0.25, verbose=False)[0]
        dets = [(self.model.names[int(b.cls)], round(float(b.conf), 3)) for b in res.boxes]
        print(f'frame({msg.width}x{msg.height}):', dets if dets else '(no detection)', flush=True)
        cv2.imwrite(self.out_path, res.plot())
        self.remaining -= 1
        if self.remaining <= 0:
            raise SystemExit


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/yolo_test.png'
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    rclpy.init()
    try:
        rclpy.spin(StaticTest(out, n))
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
