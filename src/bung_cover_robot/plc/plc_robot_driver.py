"""PLC-backed RobotDriver: manual jog/home over EtherNet/IP.

Implements the absolute-incremental model (Claude.md §7, §11, §15):

  * enable()  -> write Manual.Enable, wait Status.Enabled.
  * home()    -> pulse Manual.HomeRequest, wait Status.Homed (PLC runs the
                 switch-referencing routine and reports the reference angles).
  * move_to_angles(l, r) -> write Target*Deg, bump CommandID, pulse
                 MoveToTarget, wait CompleteCommandID == id AND InPosition.
                 The angles were already workspace-validated upstream.

Waits poll status bits with a timeout — never a blind fixed sleep (Claude.md
§15). All targets are gated by WorkspaceValidator before they reach this driver.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Optional

from ..robot.driver import Angles, RobotDriver, RobotDriverError
from . import tags as T
from .compactlogix_client import PlcClient, PlcError

logger = logging.getLogger(__name__)


class PlcRobotDriver(RobotDriver):
    def __init__(
        self,
        client: PlcClient,
        command_timeout_s: float = 10.0,
        poll_interval_s: float = 0.02,
    ) -> None:
        self.client = client
        self.command_timeout_s = command_timeout_s
        self.poll_interval_s = poll_interval_s
        self._command_id = 0

    # --- lifecycle ----------------------------------------------------------
    def connect(self) -> "PlcRobotDriver":
        self.client.connect()
        return self

    def close(self) -> None:
        try:
            self.disable()
        finally:
            self.client.close()

    # --- RobotDriver --------------------------------------------------------
    @property
    def is_enabled(self) -> bool:
        try:
            return bool(self._read(T.Status.ENABLED))
        except PlcError:
            return False

    def enable(self) -> None:
        self._clear_fault_guard()
        self._write(T.Manual.ENABLE, True)
        self._wait(lambda: bool(self._read(T.Status.ENABLED)), "enable drives")

    def disable(self) -> None:
        try:
            self._write(T.Manual.ENABLE, False)
        except PlcError:  # pragma: no cover - best-effort on shutdown
            pass

    def move_to_angles(self, left_deg: float, right_deg: float) -> None:
        if not self.is_enabled:
            raise RobotDriverError("cannot move: drives are disabled")
        cid = self._next_command_id()
        self._write(T.Manual.TARGET_LEFT_DEG, float(left_deg))
        self._write(T.Manual.TARGET_RIGHT_DEG, float(right_deg))
        self._write(T.Manual.COMMAND_ID, cid)
        self._pulse(T.Manual.MOVE_TO_TARGET)
        self._wait(
            lambda: int(self._read(T.Status.COMPLETE_COMMAND_ID)) == cid
            and bool(self._read(T.Status.IN_POSITION)),
            f"move to L={left_deg:.2f} R={right_deg:.2f}",
        )
        logger.info("moved -> L=%.3f R=%.3f (cmd %d)", left_deg, right_deg, cid)

    def read_angles(self) -> Optional[Angles]:
        try:
            if not bool(self._read(T.Status.HOMED)):
                return None
            return (
                float(self._read(T.Status.ACTUAL_LEFT_DEG)),
                float(self._read(T.Status.ACTUAL_RIGHT_DEG)),
            )
        except PlcError:
            return None

    def home(self) -> None:
        if not self.is_enabled:
            raise RobotDriverError("cannot home: drives are disabled")
        self._pulse(T.Manual.HOME_REQUEST)
        self._wait(lambda: bool(self._read(T.Status.HOMED)), "home / find reference")
        logger.info("homed -> %s", self.read_angles())

    def stop(self) -> None:
        self._pulse(T.Manual.ABORT)

    # --- helpers ------------------------------------------------------------
    def _next_command_id(self) -> int:
        self._command_id += 1
        return self._command_id

    def _read(self, tag: str):
        return self.client.read(tag)

    def _write(self, tag: str, value) -> None:
        self.client.write(tag, value)

    def _pulse(self, tag: str) -> None:
        """Rising edge then clear — the PLC latches on the rising edge."""
        self._write(tag, True)
        self._write(tag, False)

    def _clear_fault_guard(self) -> None:
        if bool(self._read_safe(T.Status.FAULTED)):
            raise RobotDriverError(
                f"PLC is faulted (code {self._read_safe(T.Status.FAULT_CODE)}); "
                "reset before enabling"
            )

    def _read_safe(self, tag: str):
        try:
            return self._read(tag)
        except PlcError:
            return 0

    def _wait(self, condition: Callable[[], bool], what: str) -> None:
        """Poll ``condition`` until true or timeout; fault -> raise. Checks
        before sleeping, so an already-satisfied condition returns at once."""
        deadline = time.monotonic() + self.command_timeout_s
        while True:
            if bool(self._read_safe(T.Status.FAULTED)):
                raise RobotDriverError(
                    f"{what}: PLC faulted (code {self._read_safe(T.Status.FAULT_CODE)})"
                )
            if condition():
                return
            if time.monotonic() >= deadline:
                raise RobotDriverError(
                    f"{what}: timed out after {self.command_timeout_s:.1f}s"
                )
            time.sleep(self.poll_interval_s)
