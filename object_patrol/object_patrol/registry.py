"""発見物体の登録簿。近接同一クラスは統合し、複数フレーム観測で確定する。"""
import math


class ObjectRegistry:
    def __init__(self, merge_dist=0.5, confirm_frames=3):
        self.merge_dist = merge_dist
        self.confirm_frames = confirm_frames
        self._objects = []

    def observe(self, class_name, x, y):
        for o in self._objects:
            if o['class_name'] == class_name and math.hypot(o['x'] - x, o['y'] - y) <= self.merge_dist:
                n = o['count']
                o['x'] = (o['x'] * n + x) / (n + 1)
                o['y'] = (o['y'] * n + y) / (n + 1)
                o['count'] = n + 1
                o['confirmed'] = o['count'] >= self.confirm_frames
                return o
        o = {'class_name': class_name, 'x': x, 'y': y, 'count': 1,
             'confirmed': self.confirm_frames <= 1}
        self._objects.append(o)
        return o

    def confirmed(self):
        return [o for o in self._objects if o['confirmed']]
