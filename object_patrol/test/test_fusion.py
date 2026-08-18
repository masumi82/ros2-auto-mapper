import math

import pytest

from object_patrol.fusion import bearing_range_to_xy, pixel_to_bearing, scan_range_at

HFOV = 1.085595  # waffle_pi camera horizontal FOV [rad]


def test_center_pixel_is_zero_bearing():
    assert pixel_to_bearing(320, 640, HFOV) == pytest.approx(0.0)


def test_left_edge_is_positive_half_fov():
    assert pixel_to_bearing(0, 640, HFOV) == pytest.approx(HFOV / 2, abs=1e-6)


def test_right_edge_is_negative_half_fov():
    assert pixel_to_bearing(640, 640, HFOV) == pytest.approx(-HFOV / 2, abs=1e-6)


def test_scan_range_returns_nearest_valid():
    # 360本・1度刻み。bearing=0の±2度に有効値[2.0, 2.2]とinf
    # → 最近傍値2.0(細いポールは背後の壁より手前にあるため最近傍を採用)
    ranges = [float('inf')] * 360
    ranges[359], ranges[0], ranges[1] = 2.0, float('inf'), 2.2
    r = scan_range_at(ranges, 0.0, math.radians(1), 0.0, window_deg=2.0)
    assert r == pytest.approx(2.0)


def test_scan_range_returns_none_when_all_invalid():
    assert scan_range_at([float('inf')] * 360, 0.0, math.radians(1), 0.0) is None


def test_scan_range_wraps_around_zero():
    # bearing=-2度(=358度側)でもラップして拾える
    ranges = [float('inf')] * 360
    ranges[358] = 1.5
    r = scan_range_at(ranges, 0.0, math.radians(1), math.radians(-2), window_deg=1.5)
    assert r == pytest.approx(1.5)


def test_bearing_range_to_xy_forward():
    x, y = bearing_range_to_xy(0.0, 2.0)
    assert (x, y) == (pytest.approx(2.0), pytest.approx(0.0))


def test_bearing_range_to_xy_left():
    x, y = bearing_range_to_xy(math.pi / 2, 3.0)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(3.0)
