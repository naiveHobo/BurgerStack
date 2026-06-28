"""Pinhole deprojection: pixel + depth + intrinsics -> 3D point in the optical frame."""
import math

import numpy as np

from semantic_perception.deprojection import (
    deproject_detection,
    deproject_pixel,
    sample_bbox_depth,
)

# A simple intrinsics matrix (row-major K, as in sensor_msgs/CameraInfo.k).
FX, FY, CX, CY = 500.0, 500.0, 320.0, 240.0
K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]


def test_deproject_pixel_center_is_straight_ahead():
    p = deproject_pixel(CX, CY, 2.0, FX, FY, CX, CY)
    np.testing.assert_allclose(p, [0.0, 0.0, 2.0])


def test_deproject_pixel_offcenter():
    # 100 px right of centre, at 2 m: X = 100 * 2 / 500 = 0.4 m (optical +X = right)
    p = deproject_pixel(CX + 100, CY, 2.0, FX, FY, CX, CY)
    np.testing.assert_allclose(p, [0.4, 0.0, 2.0])


def test_sample_bbox_depth_median_ignores_invalid():
    depth = np.full((10, 10), np.nan, dtype=np.float32)
    depth[2:6, 2:6] = 2.0          # the object
    depth[2, 2] = 0.0              # invalid: no return
    depth[2, 3] = np.nan           # invalid: NaN
    depth[3, 2] = 50.0             # invalid: beyond depth_max
    z = sample_bbox_depth(depth, 2, 2, 4, 4, depth_min=0.1, depth_max=8.0)
    assert z == 2.0


def test_sample_bbox_depth_all_invalid_returns_nan():
    depth = np.zeros((10, 10), dtype=np.float32)   # all 0 -> no valid return
    z = sample_bbox_depth(depth, 2, 2, 4, 4, depth_min=0.1, depth_max=8.0)
    assert math.isnan(z)


def test_deproject_detection_combines_center_and_median_depth():
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    depth[220:260, 300:340] = 2.0          # bbox centred on the principal point
    p = deproject_detection((300, 220, 40, 40), depth, K, depth_min=0.1, depth_max=8.0)
    assert p is not None
    np.testing.assert_allclose(p, [0.0, 0.0, 2.0], atol=1e-6)


def test_deproject_detection_none_when_no_valid_depth():
    depth = np.zeros((480, 640), dtype=np.float32)
    p = deproject_detection((300, 220, 40, 40), depth, K, depth_min=0.1, depth_max=8.0)
    assert p is None
