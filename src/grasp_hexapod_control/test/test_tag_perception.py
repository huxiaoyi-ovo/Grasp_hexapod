#!/usr/bin/env python3
import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "tag_perception.py"
SPEC = importlib.util.spec_from_file_location("tag_perception", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_angle_filter_suppresses_single_pose_spike():
    angle_filter = MODULE.AngleFilter(window_size=7, alpha=0.25,
                                      max_rate_deg_s=120.0)
    values = [angle_filter.update(value, index / 30.0)
              for index, value in enumerate([90, 91, 89, 90, 130, 90, 91])]
    assert max(values) - min(values) < 1.0


def test_angle_filter_tracks_sustained_motion():
    angle_filter = MODULE.AngleFilter(window_size=7, alpha=0.25,
                                      max_rate_deg_s=120.0)
    values = [angle_filter.update(90.0, index / 30.0) for index in range(10)]
    values += [angle_filter.update(110.0, index / 30.0)
               for index in range(10, 40)]
    assert values[-1] > 108.0


def test_angle_filter_resets_after_detection_gap():
    angle_filter = MODULE.AngleFilter(reset_timeout_s=0.7)
    angle_filter.update(90.0, 0.0)
    assert angle_filter.update(120.0, 1.0) == 120.0
