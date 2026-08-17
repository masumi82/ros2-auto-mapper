"""カメラ+LiDARフュージョンの純粋関数群。ROS非依存でpytest可能。"""
import math


def pixel_to_bearing(px, image_width, hfov):
    """バウンディングボックス中心の横ピクセル → 方位角[rad](左が正)。"""
    fx = image_width / (2.0 * math.tan(hfov / 2.0))
    cx = image_width / 2.0
    return math.atan2(cx - px, fx)


def scan_range_at(ranges, angle_min, angle_increment, bearing, window_deg=3.0):
    """bearing±window内の有効距離の中央値。有効値なしならNone。"""
    window = math.radians(window_deg)
    vals = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment
        diff = math.atan2(math.sin(a - bearing), math.cos(a - bearing))
        if abs(diff) <= window and r is not None and math.isfinite(r) and r > 0.05:
            vals.append(r)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def bearing_range_to_xy(bearing, rng):
    """(方位角, 距離) → ロボット座標系の(x, y)。前方x+、左y+。"""
    return rng * math.cos(bearing), rng * math.sin(bearing)
