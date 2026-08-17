import pytest

from object_patrol.registry import ObjectRegistry


def test_new_object_not_confirmed_until_3_frames():
    reg = ObjectRegistry(merge_dist=0.5, confirm_frames=3)
    reg.observe('person', 1.0, 2.0)
    reg.observe('person', 1.1, 2.0)
    assert reg.confirmed() == []
    reg.observe('person', 1.0, 2.1)
    assert len(reg.confirmed()) == 1


def test_nearby_same_class_merges_with_mean_position():
    reg = ObjectRegistry(confirm_frames=1)
    reg.observe('chair', 0.0, 0.0)
    o = reg.observe('chair', 0.4, 0.0)
    assert len(reg.confirmed()) == 1
    assert o['x'] == pytest.approx(0.2)


def test_far_same_class_registers_separately():
    reg = ObjectRegistry(merge_dist=0.5, confirm_frames=1)
    reg.observe('chair', 0.0, 0.0)
    reg.observe('chair', 2.0, 0.0)
    assert len(reg.confirmed()) == 2


def test_different_class_never_merges():
    reg = ObjectRegistry(confirm_frames=1)
    reg.observe('chair', 0.0, 0.0)
    reg.observe('bottle', 0.1, 0.0)
    assert len(reg.confirmed()) == 2
