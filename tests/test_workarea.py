"""Auto-derived largest safe work area."""

from bung_cover_robot.robot.fivebar_kinematics import FiveBarConfig, FiveBarKinematics
from bung_cover_robot.robot.workspace import (
    WorkArea,
    WorkspaceValidator,
    largest_safe_rectangle,
)
from bung_cover_robot.robot.workspace import _max_true_rectangle


def test_max_true_rectangle_basic():
    mask = [
        [1, 1, 0, 1],
        [1, 1, 1, 1],
        [1, 1, 1, 0],
    ]
    # largest all-1 rectangle is the 3-tall x 2-wide block in columns 0-1
    top, left, bottom, right = _max_true_rectangle(mask)
    assert (top, left, bottom, right) == (0, 0, 2, 1)
    assert _max_true_rectangle([[0, 0], [0, 0]]) is None


def test_derived_area_is_safe_everywhere():
    val = WorkspaceValidator(FiveBarKinematics())
    area = largest_safe_rectangle(val, step=4.0)
    assert area is not None
    assert area.width > 50 and area.height > 20        # a real, usable area
    # every corner (and the centre) of the derived rectangle passes the guard
    for x in (area.x_min, area.x_max, area.center[0]):
        for y in (area.y_min, area.y_max, area.center[1]):
            assert val.is_safe(x, y), f"({x:.0f},{y:.0f}) not safe"
    # roughly symmetric about x=0 (bases are symmetric)
    assert abs(area.center[0]) < 10.0


def test_smaller_robot_yields_smaller_area():
    big = largest_safe_rectangle(WorkspaceValidator(FiveBarKinematics()))
    small_cfg = FiveBarConfig(l1_mm=100.0, l2_mm=115.0, base_spacing_mm=40.0)
    small = largest_safe_rectangle(WorkspaceValidator(FiveBarKinematics(small_cfg)))
    assert big is not None and small is not None
    assert small.width < big.width and small.height < big.height
    # and the small robot's area is still genuinely safe
    assert WorkspaceValidator(FiveBarKinematics(small_cfg)).is_safe(*small.center)


def test_inset_shrinks_the_area():
    val = WorkspaceValidator(FiveBarKinematics())
    a = largest_safe_rectangle(val, step=4.0)
    b = largest_safe_rectangle(val, step=4.0, inset=10.0)
    assert b.width < a.width and b.height < a.height
