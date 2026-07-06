"""Automatic pick/place handshake (Claude.md §11).

``send_job_and_wait`` loads the pick/drop target angles, bumps ``Cmd.CommandID``,
pulses ``Cmd.RequestPickPlace``, then waits for ``Status.CompleteCommandID`` to
echo that id. The PLC owns the whole pick/place choreography — camera-clear move,
cylinder down/up, vacuum on, verify, blow-off (state machine §11). Python only
loads targets and waits on real status bits.

Two failure modes are handled explicitly (both were gaps in the original draft):
  * **Fault** — ``Status.Faulted`` goes true: stop and report ``FaultCode``.
  * **Timeout/recovery** — neither Complete nor a clean Fault within
    ``command_timeout_s``: pulse ``Cmd.Abort`` and return a recoverable error
    instead of hanging.

The angles handed in were already workspace-validated by robot.planner, so this
layer never re-decides reachability — it only drives the tag handshake. Kept free
of any robot.planner import so the PLC layer stays independent of planning types.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, Tuple

from . import tags as T
from .compactlogix_client import PlcClient, PlcError

logger = logging.getLogger(__name__)

Angles = Tuple[float, float]  # (left_deg, right_deg)


@dataclass(frozen=True)
class JobResult:
    ok: bool
    reason: str
    command_id: int

    def __bool__(self) -> bool:
        return self.ok


class PickPlaceHandshake:
    def __init__(
        self,
        client: PlcClient,
        command_timeout_s: float = 30.0,
        ready_timeout_s: float = 10.0,
        poll_interval_s: float = 0.02,
    ) -> None:
        self.client = client
        self.command_timeout_s = command_timeout_s
        self.ready_timeout_s = ready_timeout_s
        self.poll_interval_s = poll_interval_s
        self._command_id = 0

    def send_job_and_wait(
        self, pick: Angles, drop: Angles, hole_index: int, cover_id: int
    ) -> JobResult:
        """Run one pick/place job to completion. Never raises for a rejected job —
        the outcome (fault, timeout, done) is returned as a JobResult."""
        wait_err = self._wait_ready()
        if wait_err is not None:
            return JobResult(False, wait_err, self._command_id)

        cid = self._next_command_id()
        try:
            self._write(T.Target.PICK_LEFT_DEG, float(pick[0]))
            self._write(T.Target.PICK_RIGHT_DEG, float(pick[1]))
            self._write(T.Target.DROP_LEFT_DEG, float(drop[0]))
            self._write(T.Target.DROP_RIGHT_DEG, float(drop[1]))
            self._write(T.Target.HOLE_INDEX, int(hole_index))
            self._write(T.Target.COVER_ID, int(cover_id))
            self._write(T.Cmd.COMMAND_ID, cid)
            self._pulse(T.Cmd.REQUEST_PICK_PLACE)
        except PlcError as exc:
            return JobResult(False, f"tag write failed: {exc}", cid)

        return self._wait_complete(cid)

    # --- waits --------------------------------------------------------------
    def _wait_ready(self) -> "str | None":
        """Wait until the PLC is Ready (and not Busy/Faulted). Returns an error
        string on fault/timeout, or None when ready."""
        deadline = time.monotonic() + self.ready_timeout_s
        while True:
            if bool(self._read_safe(T.Status.FAULTED)):
                return f"PLC faulted (code {self._read_safe(T.Status.FAULT_CODE)}); reset first"
            if bool(self._read_safe(T.Status.READY)) and not bool(
                self._read_safe(T.Status.BUSY)
            ):
                return None
            if time.monotonic() >= deadline:
                return f"PLC not ready after {self.ready_timeout_s:.1f}s"
            time.sleep(self.poll_interval_s)

    def _wait_complete(self, cid: int) -> JobResult:
        deadline = time.monotonic() + self.command_timeout_s
        while True:
            if bool(self._read_safe(T.Status.FAULTED)):
                code = self._read_safe(T.Status.FAULT_CODE)
                return JobResult(False, f"PLC faulted (code {code})", cid)
            if int(self._read_safe(T.Status.FAILED_COMMAND_ID)) == cid:
                return JobResult(False, "PLC reported the job failed", cid)
            if int(self._read_safe(T.Status.COMPLETE_COMMAND_ID)) == cid:
                logger.info("pick/place job %d complete", cid)
                return JobResult(True, "ok", cid)
            if time.monotonic() >= deadline:
                # Recovery: don't hang — abort the stuck job and surface it.
                try:
                    self._pulse(T.Cmd.ABORT)
                except PlcError:
                    pass
                return JobResult(
                    False, f"timed out after {self.command_timeout_s:.1f}s (aborted)", cid
                )
            time.sleep(self.poll_interval_s)

    # --- helpers ------------------------------------------------------------
    def _next_command_id(self) -> int:
        self._command_id += 1
        return self._command_id

    def _write(self, tag: str, value) -> None:
        self.client.write(tag, value)

    def _pulse(self, tag: str) -> None:
        self._write(tag, True)
        self._write(tag, False)

    def _read_safe(self, tag: str):
        try:
            return self.client.read(tag)
        except PlcError:
            return 0
