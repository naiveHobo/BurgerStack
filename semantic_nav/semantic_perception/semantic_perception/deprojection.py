"""Pinhole deprojection of a 2D detection into a 3D point in the camera optical frame.

Pure NumPy, no ROS. The output point follows the REP-103 optical convention
(+X right, +Y down, +Z forward), matching camera_rgb_optical_frame; the ROS node
then transforms it into the map frame.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np


def deproject_pixel(u: float, v: float, z: float,
                    fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """Back-project pixel ``(u, v)`` at depth ``z`` (m) to an optical-frame point."""
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=float)


def sample_bbox_depth(depth: np.ndarray, x: int, y: int, w: int, h: int,
                      *, depth_min: float, depth_max: float) -> float:
    """Median depth over a bbox, ignoring non-finite / zero / out-of-range pixels.

    Returns ``nan`` if no valid depth is present (e.g. the object is out of range
    or the bbox sees only background).
    """
    patch = depth[y:y + h, x:x + w]
    valid = patch[np.isfinite(patch) & (patch > depth_min) & (patch <= depth_max)]
    if valid.size == 0:
        return float("nan")
    return float(np.median(valid))


def deproject_detection(bbox: Tuple[int, int, int, int], depth: np.ndarray,
                        k: Sequence[float], *, depth_min: float,
                        depth_max: float) -> Optional[np.ndarray]:
    """Deproject a detection: bbox-centre pixel + robust median depth -> optical point.

    ``k`` is the row-major 3x3 intrinsics (sensor_msgs/CameraInfo.k). Returns ``None``
    when the bbox has no valid depth.
    """
    x, y, w, h = bbox
    z = sample_bbox_depth(depth, x, y, w, h, depth_min=depth_min, depth_max=depth_max)
    if not np.isfinite(z):
        return None
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    u = x + w / 2.0
    v = y + h / 2.0
    return deproject_pixel(u, v, z, fx, fy, cx, cy)
