"""PLC tag names — the single source of truth (Claude.md §11, §15).

Nothing else in the codebase should hard-code a tag string; import from here.
Two command surfaces share one ``VisionRobot`` UDT:

  * Cmd/Target/Status — the automatic pick/place job handshake (Claude.md §11).
  * Manual — the manual jog/home surface used by the Robot Test tab. Jog is
    *absolute-incremental*: Python writes a validated absolute angle target and
    the PLC does one coordinated move to it (see PlcRobotDriver).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

ROOT = "VisionRobot"

# Tag direction, from the PLC's point of view.
PC_TO_PLC = "PC → PLC"   # Python writes it (command / target)
PLC_TO_PC = "PLC → PC"   # Python reads it (status / feedback)


class Cmd:
    """Automatic pick/place command bits."""

    REQUEST_PICK_PLACE = f"{ROOT}.Cmd.RequestPickPlace"
    ABORT = f"{ROOT}.Cmd.Abort"
    RESET = f"{ROOT}.Cmd.Reset"
    COMMAND_ID = f"{ROOT}.Cmd.CommandID"
    HEARTBEAT = f"{ROOT}.Cmd.Heartbeat"  # DINT: PC increments continuously (watchdog)


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
    # heartbeat watchdog
    PC_ALIVE = f"{ROOT}.Status.PcAlive"    # BOOL: PLC saw the PC heartbeat in time
    HEARTBEAT = f"{ROOT}.Status.Heartbeat"  # DINT: PLC increments each scan


@dataclass(frozen=True)
class TagSpec:
    """One PLC tag the Studio 5000 program must implement."""

    name: str
    dtype: str        # BOOL | DINT | REAL
    direction: str    # PC_TO_PLC | PLC_TO_PC
    group: str        # Cmd | Target | Manual | Status
    description: str


# The full tag contract. Names reference the constants above (no duplication),
# so this list and the class constants can never drift.
TAG_SPECS: List[TagSpec] = [
    # --- automatic pick/place command (Claude.md §11) ---
    TagSpec(Cmd.REQUEST_PICK_PLACE, "BOOL", PC_TO_PLC, "Cmd",
            "Rising edge: execute the loaded pick/place job."),
    TagSpec(Cmd.ABORT, "BOOL", PC_TO_PLC, "Cmd",
            "Abort the current automatic job."),
    TagSpec(Cmd.RESET, "BOOL", PC_TO_PLC, "Cmd",
            "Clear a latched fault and return to Ready."),
    TagSpec(Cmd.COMMAND_ID, "DINT", PC_TO_PLC, "Cmd",
            "Monotonic id for the current job; rejects stale/duplicate commands."),
    TagSpec(Cmd.HEARTBEAT, "DINT", PC_TO_PLC, "Cmd",
            "PC increments continuously while connected; PLC watchdogs it (code 10)."),
    # --- automatic pick/place targets ---
    TagSpec(Target.PICK_LEFT_DEG, "REAL", PC_TO_PLC, "Target",
            "Left shoulder angle at the pick pose (deg)."),
    TagSpec(Target.PICK_RIGHT_DEG, "REAL", PC_TO_PLC, "Target",
            "Right shoulder angle at the pick pose (deg)."),
    TagSpec(Target.DROP_LEFT_DEG, "REAL", PC_TO_PLC, "Target",
            "Left shoulder angle at the drop/hole pose (deg)."),
    TagSpec(Target.DROP_RIGHT_DEG, "REAL", PC_TO_PLC, "Target",
            "Right shoulder angle at the drop/hole pose (deg)."),
    TagSpec(Target.HOLE_INDEX, "DINT", PC_TO_PLC, "Target",
            "Target hole index (0..5)."),
    TagSpec(Target.COVER_ID, "DINT", PC_TO_PLC, "Target",
            "Identifier of the selected cover."),
    # --- manual jog/home (Robot Test tab) ---
    TagSpec(Manual.ENABLE, "BOOL", PC_TO_PLC, "Manual",
            "Request the drives energized/enabled."),
    TagSpec(Manual.HOME_REQUEST, "BOOL", PC_TO_PLC, "Manual",
            "Rising edge: run the homing routine (find the switches)."),
    TagSpec(Manual.MOVE_TO_TARGET, "BOOL", PC_TO_PLC, "Manual",
            "Rising edge: move to Manual.TargetLeft/RightDeg (one coordinated move)."),
    TagSpec(Manual.ABORT, "BOOL", PC_TO_PLC, "Manual",
            "Stop manual motion."),
    TagSpec(Manual.TARGET_LEFT_DEG, "REAL", PC_TO_PLC, "Manual",
            "Commanded left shoulder angle for a manual move (deg)."),
    TagSpec(Manual.TARGET_RIGHT_DEG, "REAL", PC_TO_PLC, "Manual",
            "Commanded right shoulder angle for a manual move (deg)."),
    TagSpec(Manual.COMMAND_ID, "DINT", PC_TO_PLC, "Manual",
            "Monotonic id for the manual move; echoed in Status.CompleteCommandID."),
    # --- status / feedback ---
    TagSpec(Status.READY, "BOOL", PLC_TO_PC, "Status",
            "PLC idle and ready for a job."),
    TagSpec(Status.BUSY, "BOOL", PLC_TO_PC, "Status",
            "A job or move is in progress."),
    TagSpec(Status.DONE, "BOOL", PLC_TO_PC, "Status",
            "The last automatic job completed."),
    TagSpec(Status.FAULTED, "BOOL", PLC_TO_PC, "Status",
            "A fault is active; motion inhibited until Reset."),
    TagSpec(Status.FAULT_CODE, "DINT", PLC_TO_PC, "Status",
            "Active fault code (0 = none)."),
    TagSpec(Status.ENABLED, "BOOL", PLC_TO_PC, "Status",
            "Drives are enabled."),
    TagSpec(Status.HOMED, "BOOL", PLC_TO_PC, "Status",
            "Home reference has been established."),
    TagSpec(Status.IN_POSITION, "BOOL", PLC_TO_PC, "Status",
            "Axes have reached the commanded target."),
    TagSpec(Status.MOVING, "BOOL", PLC_TO_PC, "Status",
            "Axes are in motion."),
    TagSpec(Status.ACTUAL_LEFT_DEG, "REAL", PLC_TO_PC, "Status",
            "Current left shoulder angle (deg)."),
    TagSpec(Status.ACTUAL_RIGHT_DEG, "REAL", PLC_TO_PC, "Status",
            "Current right shoulder angle (deg)."),
    TagSpec(Status.ACTIVE_COMMAND_ID, "DINT", PLC_TO_PC, "Status",
            "CommandID currently being executed."),
    TagSpec(Status.COMPLETE_COMMAND_ID, "DINT", PLC_TO_PC, "Status",
            "CommandID of the last successfully completed command."),
    TagSpec(Status.FAILED_COMMAND_ID, "DINT", PLC_TO_PC, "Status",
            "CommandID of the last failed command."),
    TagSpec(Status.VACUUM_OK, "BOOL", PLC_TO_PC, "Status",
            "Vacuum sensor confirms a cover is held."),
    TagSpec(Status.CAMERA_CLEAR, "BOOL", PLC_TO_PC, "Status",
            "Robot is clear of the camera field of view."),
    TagSpec(Status.READY_FOR_VISION, "BOOL", PLC_TO_PC, "Status",
            "PLC is ready to receive a vision command."),
    TagSpec(Status.PC_ALIVE, "BOOL", PLC_TO_PC, "Status",
            "PLC saw the PC heartbeat within HB_TIMEOUT_MS; gates the drive enable."),
    TagSpec(Status.HEARTBEAT, "DINT", PLC_TO_PC, "Status",
            "PLC increments each scan; PC verifies the ladder is actually scanning."),
]


def all_tags() -> List[str]:
    """Every tag string (for diagnostics / simulator seeding)."""
    return [spec.name for spec in TAG_SPECS]


def tag_table_csv() -> str:
    """The tag contract as CSV (name,type,direction,group,description)."""
    import csv
    import io

    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["Tag", "Type", "Direction", "Group", "Description"])
    for s in TAG_SPECS:
        writer.writerow([s.name, s.dtype, s.direction, s.group, s.description])
    return buf.getvalue()
