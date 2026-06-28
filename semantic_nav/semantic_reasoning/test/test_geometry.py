"""Pure approach-pose geometry + costmap freeness. No ROS."""
import math

from semantic_reasoning.geometry import (
    approach_pose,
    choose_approach_pose,
    is_free_in_grid,
    ring_candidates,
)


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _faces(px, py, yaw, ox, oy, tol=1e-6):
    """True if `yaw` points from (px,py) toward (ox,oy)."""
    expected = math.atan2(oy - py, ox - px)
    return abs(math.atan2(math.sin(yaw - expected), math.cos(yaw - expected))) < tol


# --- approach_pose -------------------------------------------------------

def test_approach_pose_stands_off_toward_robot_and_faces_object():
    x, y, yaw = approach_pose(0.0, 0.0, 2.0, 0.0, standoff=0.5)
    assert (round(x, 6), round(y, 6)) == (0.5, 0.0)        # 0.5 m from obj toward robot
    assert _faces(x, y, yaw, 0.0, 0.0)                     # facing the object (yaw == pi)


def test_approach_pose_uses_actual_robot_direction():
    x, y, yaw = approach_pose(0.0, 0.0, 0.0, 2.0, standoff=0.5)
    assert (round(x, 6), round(y, 6)) == (0.0, 0.5)
    assert _faces(x, y, yaw, 0.0, 0.0)


def test_approach_pose_when_robot_already_within_standoff_just_faces():
    # robot 0.3 m away, standoff 0.5 -> don't drive away; stop at robot, face object.
    x, y, yaw = approach_pose(0.0, 0.0, 0.3, 0.0, standoff=0.5)
    assert (round(x, 6), round(y, 6)) == (0.3, 0.0)
    assert _faces(x, y, yaw, 0.0, 0.0)


def test_approach_pose_degenerate_robot_on_object_still_offsets():
    # robot ~ on the object: can't infer a direction, but must NOT return the object cell.
    x, y, yaw = approach_pose(1.0, 1.0, 1.0, 1.0, standoff=0.5)
    assert round(_dist(x, y, 1.0, 1.0), 6) == 0.5
    assert _faces(x, y, yaw, 1.0, 1.0)


# --- ring_candidates -----------------------------------------------------

def test_ring_candidates_count_radius_and_facing():
    cands = ring_candidates(0.0, 0.0, standoff=1.0, n=8)
    assert len(cands) == 8
    for px, py, yaw in cands:
        assert abs(_dist(px, py, 0.0, 0.0) - 1.0) < 1e-9
        assert _faces(px, py, yaw, 0.0, 0.0)


# --- choose_approach_pose (hybrid) ---------------------------------------

def test_choose_returns_toward_robot_when_no_validator():
    chosen = choose_approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0, is_free=None)
    assert chosen == approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0)


def test_choose_returns_toward_robot_when_it_is_free():
    chosen = choose_approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0,
                                  is_free=lambda x, y: True)
    assert chosen == approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0)


def test_choose_falls_back_to_nearest_free_ring_when_primary_blocked():
    primary = approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0)  # (1, 0)

    def is_free(x, y):  # block only the primary cell
        return not (abs(x - primary[0]) < 1e-6 and abs(y - primary[1]) < 1e-6)

    chosen = choose_approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0, is_free=is_free, n=4)
    assert chosen != primary
    assert is_free(chosen[0], chosen[1])
    assert abs(_dist(chosen[0], chosen[1], 0.0, 0.0) - 1.0) < 1e-9  # still at standoff


def test_choose_best_effort_when_everything_blocked():
    chosen = choose_approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0,
                                  is_free=lambda x, y: False, n=4)
    assert chosen == approach_pose(0.0, 0.0, 2.0, 0.0, standoff=1.0)


# --- is_free_in_grid -----------------------------------------------------

def _grid():
    # 3x3 grid, 1 m cells, origin (0,0). Center cell (1,1) lethal.
    data = [0] * 9
    data[1 * 3 + 1] = 100
    return dict(origin_x=0.0, origin_y=0.0, resolution=1.0, width=3, height=3, data=data)


def test_is_free_in_grid_free_cell():
    assert is_free_in_grid(**_grid(), wx=0.5, wy=0.5, threshold=99) is True


def test_is_free_in_grid_lethal_cell():
    assert is_free_in_grid(**_grid(), wx=1.5, wy=1.5, threshold=99) is False


def test_is_free_in_grid_out_of_bounds():
    assert is_free_in_grid(**_grid(), wx=5.0, wy=5.0, threshold=99) is False


def test_is_free_in_grid_unknown_is_not_free():
    g = _grid()
    g["data"][0] = -1
    assert is_free_in_grid(**g, wx=0.5, wy=0.5, threshold=99) is False
