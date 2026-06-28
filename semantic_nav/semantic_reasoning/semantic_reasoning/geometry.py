"""Pure approach-pose geometry (no ROS).

"Go to <object>" must not drive the robot into the object: the object's centroid is a
lethal cell in the global costmap, and the RRT planner rejects an in-collision goal
outright. So we navigate to a *standoff* pose a short distance from the object, facing
it. The default standoff sits on the line toward the robot (it came from there, so that
side is usually clear); a costmap-aware caller can reject it and fall back to the nearest
free pose on a ring around the object.

Everything here is pure and injectable (``is_free`` is a plain predicate), so the whole
approach-selection logic is unit-tested with no costmap or ROS.
"""
from __future__ import annotations

import math
from typing import Callable, List, Optional, Tuple

Pose = Tuple[float, float, float]  # (x, y, yaw) in the map frame

_EPS = 1e-6


def approach_pose(obj_x: float, obj_y: float, robot_x: float, robot_y: float,
                  standoff: float) -> Pose:
    """A pose ``standoff`` m from the object toward the robot, facing the object.

    If the robot is already within ``standoff`` of the object, stop where it is and just
    face the object (don't back away). If the robot is essentially on the object the
    direction is undefined, so offset along +x — never return the object cell itself.
    """
    dx, dy = robot_x - obj_x, robot_y - obj_y
    dist = math.hypot(dx, dy)
    if dist < _EPS:
        px, py = obj_x + standoff, obj_y          # degenerate: arbitrary but valid offset
    elif dist <= standoff:
        px, py = robot_x, robot_y                 # already close enough: just face it
    else:
        ux, uy = dx / dist, dy / dist
        px, py = obj_x + ux * standoff, obj_y + uy * standoff
    yaw = math.atan2(obj_y - py, obj_x - px)
    return px, py, yaw


def ring_candidates(obj_x: float, obj_y: float, standoff: float, n: int) -> List[Pose]:
    """``n`` poses evenly spaced on a circle of radius ``standoff``, each facing the object."""
    out: List[Pose] = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        px = obj_x + standoff * math.cos(theta)
        py = obj_y + standoff * math.sin(theta)
        out.append((px, py, math.atan2(obj_y - py, obj_x - px)))
    return out


def choose_approach_pose(obj_x: float, obj_y: float, robot_x: float, robot_y: float,
                         standoff: float,
                         is_free: Optional[Callable[[float, float], bool]] = None,
                         n: int = 12) -> Pose:
    """Hybrid selector: toward-robot standoff, with a ring fallback when it's blocked.

    With no ``is_free`` validator, return the toward-robot pose. Otherwise, return it if
    free; else the ring candidate that is free and nearest the robot; else the toward-robot
    pose as a best effort (let the planner have the final say).
    """
    primary = approach_pose(obj_x, obj_y, robot_x, robot_y, standoff)
    if is_free is None or is_free(primary[0], primary[1]):
        return primary
    free = [c for c in ring_candidates(obj_x, obj_y, standoff, n) if is_free(c[0], c[1])]
    if free:
        return min(free, key=lambda c: math.hypot(c[0] - robot_x, c[1] - robot_y))
    return primary


def is_free_in_grid(origin_x: float, origin_y: float, resolution: float,
                    width: int, height: int, data, wx: float, wy: float,
                    threshold: int = 99) -> bool:
    """Is world point (wx, wy) a free cell in an OccupancyGrid?

    Nav2 publishes the costmap as an OccupancyGrid scaled 0-100 (99 = inscribed,
    100 = lethal, -1 = unknown). A cell is free when ``0 <= value < threshold``;
    out-of-bounds and unknown count as not free.
    """
    if resolution <= 0.0:
        return False
    mx = int((wx - origin_x) / resolution)
    my = int((wy - origin_y) / resolution)
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return False
    value = data[my * width + mx]
    if value < 0:
        return False
    return value < threshold
