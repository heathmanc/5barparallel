"""PLC tag names — the single source of truth (Claude.md §11, §15).

Nothing else in the codebase should hard-code a tag string; import from here.
Two command surfaces share one ``VisionRobot`` UDT:

  * Cmd/Target/Status — the automatic pick/place job handshake (Claude.md §11).
  * Manual — the manual jog/home surface used by the Robot Test tab. Jog is
    *absolute-incremental*: Python writes a validated absolute angle target and
    the PLC does one coordinated move to it (see PlcRobotDriver).
"""

from __future__ import annotations

from typing import List

ROOT = "VisionRobot"


class Cmd:
    """Automatic pick/place command bits."""

    REQUEST_PICK_PLACE = f"{ROOT}.Cmd.RequestPickPlace"
    ABORT = f"{ROOT}.Cmd.Abort"
    RESET = f"{ROOT}.Cmd.Reset"
    COMMAND_ID = f"{ROOT}.Cmd.CommandID"


class Target:
    """Automatic pick/place target angles/ids."""

    PICK_LEFT_DEG = f"{ROOT}.Target.Pick_LeftDeg"
    PICK_RIGHT_DEG = f"{ROOT}.Target.Pick_RightDeg"
    DROP_LEFT_DEG = f"{ROOT}.Target.Drop_LeftDeg"
    DROP_RIGHT_DEG = f"{ROOT}.Target.Drop_RightDeg"
    HOLE_INDEX = f"{ROOT}.Target.HoleIndex"
    COVER_ID = f"{ROOT}.Target.CoverID"


class Manual:
    """Manual jog/home command surface (Robot Test tab)."""

    ENABLE = f"{ROOT}.Manual.Enable"            # BOOL: request drives enabled
    HOME_REQUEST = f"{ROOT}.Manual.HomeRequest"  # BOOL: run homing routine
    MOVE_TO_TARGET = f"{ROOT}.Manual.MoveToTarget"  # BOOL: go to Target*Deg
    ABORT = f"{ROOT}.Manual.Abort"              # BOOL: stop motion
    TARGET_LEFT_DEG = f"{ROOT}.Manual.TargetLeftDeg"    # REAL
    TARGET_RIGHT_DEG = f"{ROOT}.Manual.TargetRightDeg"  # REAL
    COMMAND_ID = f"{ROOT}.Manual.CommandID"     # DINT: rejects stale commands


class Status:
    """Status bits/values reported by the PLC."""

    READY = f"{ROOT}.Status.Ready"
    BUSY = f"{ROOT}.Status.Busy"
    DONE = f"{ROOT}.Status.Done"
    FAULTED = f"{ROOT}.Status.Faulted"
    FAULT_CODE = f"{ROOT}.Status.FaultCode"
    # manual/motion status
    ENABLED = f"{ROOT}.Status.Enabled"
    HOMED = f"{ROOT}.Status.Homed"
    IN_POSITION = f"{ROOT}.Status.InPosition"
    MOVING = f"{ROOT}.Status.Moving"
    ACTUAL_LEFT_DEG = f"{ROOT}.Status.ActualLeftDeg"
    ACTUAL_RIGHT_DEG = f"{ROOT}.Status.ActualRightDeg"
    # command-id acknowledgement (shared handshake)
    ACTIVE_COMMAND_ID = f"{ROOT}.Status.ActiveCommandID"
    COMPLETE_COMMAND_ID = f"{ROOT}.Status.CompleteCommandID"
    FAILED_COMMAND_ID = f"{ROOT}.Status.FailedCommandID"
    # process io
    VACUUM_OK = f"{ROOT}.Status.VacuumOK"
    CAMERA_CLEAR = f"{ROOT}.Status.CameraClear"
    READY_FOR_VISION = f"{ROOT}.Status.ReadyForVision"


def all_tags() -> List[str]:
    """Every tag string (for diagnostics / simulator seeding)."""
    tags: List[str] = []
    for group in (Cmd, Target, Manual, Status):
        for name, value in vars(group).items():
            if not name.startswith("_") and isinstance(value, str):
                tags.append(value)
    return tags
