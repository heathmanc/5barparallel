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
import threading
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
        pulse_hold_s: float = 0.1,
        heartbeat_interval_s: float = 0.2,
    ) -> None:
        self.client = client
        self.command_timeout_s = command_timeout_s
        self.poll_interval_s = poll_interval_s
        # A command bit must stay true long enough for the PLC (10-20 ms scan) to
        # catch the rising edge; a true-then-immediately-false write can slip
        # between scans and never be seen. 0 = no dwell (e.g. against the sim).
        self.pulse_hold_s = pulse_hold_s
        self._command_id = 0
        # Heartbeat: a background thread increments Cmd.Heartbeat so the PLC's
        # watchdog (R10) knows the app is alive; if it stalls the PLC drops the
        # drives + faults code 10. Set 0 to disable (tests that don't want a
        # background thread). Started now if the client is already connected
        # (sim path), otherwise on connect() (real path).
        self.heartbeat_interval_s = heartbeat_interval_s
        self._hb_value = 0
        self._hb_stop = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None
        if getattr(client, "is_connected", False):
            self.start_heartbeat()

    # --- lifecycle ----------------------------------------------------------
    def connect(self) -> "PlcRobotDriver":
        self.client.connect()
        self.start_heartbeat()
        return self

    def close(self) -> None:
        self.stop_heartbeat()
        try:
            self.disable()
        finally:
            self.client.close()

    # --- heartbeat ----------------------------------------------------------
    def start_heartbeat(self) -> None:
        if self.heartbeat_interval_s <= 0:
            return
        if self._hb_thread is not None and self._hb_thread.is_alive():
            return
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, name="plc-heartbeat", daemon=True
        )
        self._hb_thread.start()

    def stop_heartbeat(self) -> None:
        self._hb_stop.set()
        t = self._hb_thread
        if t is not None:
            t.join(timeout=1.0)
        self._hb_thread = None

    def _heartbeat_loop(self) -> None:
        while not self._hb_stop.is_set():
            self._hb_value = (self._hb_value + 1) & 0x3FFFFFFF
            try:
                self.client.write(T.Cmd.HEARTBEAT, self._hb_value)
            except Exception:  # noqa: BLE001 - a comms hiccup must not kill the beat
                pass
            self._hb_stop.wait(self.heartbeat_interval_s)

    @property
    def is_pc_alive(self) -> bool:
        """The PLC's view of our heartbeat (Status.PcAlive)."""
        return bool(self._read_safe(T.Status.PC_ALIVE))

    def plc_heartbeat(self) -> Optional[int]:
        """The PLC's own heartbeat counter, or None on a read error."""
        try:
            return int(self._read_safe(T.Status.HEARTBEAT))
        except (TypeError, ValueError):
            return None

    # --- RobotDriver --------------------------------------------------------
    @property
    def is_enabled(self) -> bool:
        try:
            return bool(self._read(T.Status.ENABLED))
        except PlcError:
            return False

    @property
    def is_faulted(self) -> bool:
        return bool(self._read_safe(T.Status.FAULTED))

    def fault_code(self) -> Optional[int]:
        if not self.is_faulted:
            return None
        try:
            return int(self._read_safe(T.Status.FAULT_CODE))
        except (TypeError, ValueError):
            return None

    def reset(self) -> None:
        """Pulse Cmd.Reset and wait for the latched fault to clear."""
        try:
            self._pulse(T.Cmd.RESET)
            self._wait(
                lambda: not bool(self._read(T.Status.FAULTED)), "clear fault"
            )
        except PlcError as exc:
            raise RobotDriverError(f"reset: PLC comms error: {exc}") from exc

    def enable(self) -> None:
        self._clear_fault_guard()
        try:
            self._write(T.Manual.ENABLE, True)
            self._wait(lambda: bool(self._read(T.Status.ENABLED)), "enable drives")
        except PlcError as exc:
            raise RobotDriverError(f"enable: PLC comms error: {exc}") from exc

    def disable(self) -> None:
        try:
            self._write(T.Manual.ENABLE, False)
        except PlcError:  # pragma: no cover - best-effort on shutdown
            pass

    def move_to_angles(self, left_deg: float, right_deg: float) -> None:
        if not self.is_enabled:
            raise RobotDriverError("cannot move: drives are disabled")
        cid = self._next_command_id()
        try:
            self._write(T.Manual.TARGET_LEFT_DEG, float(left_deg))
            self._write(T.Manual.TARGET_RIGHT_DEG, float(right_deg))
            self._write(T.Manual.COMMAND_ID, cid)
            self._pulse(T.Manual.MOVE_TO_TARGET)
            self._wait(
                lambda: int(self._read(T.Status.COMPLETE_COMMAND_ID)) == cid
                and bool(self._read(T.Status.IN_POSITION)),
                f"move to L={left_deg:.2f} R={right_deg:.2f}",
            )
        except PlcError as exc:
            raise RobotDriverError(f"move: PLC comms error: {exc}") from exc
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
        try:
            self._pulse(T.Manual.HOME_REQUEST)
            self._wait(
                lambda: bool(self._read(T.Status.HOMED)), "home / find reference"
            )
        except PlcError as exc:
            raise RobotDriverError(f"home: PLC comms error: {exc}") from exc
        logger.info("homed -> %s", self.read_angles())

    def stop(self) -> None:
        try:
            self._pulse(T.Manual.ABORT)
        except PlcError as exc:
            raise RobotDriverError(f"stop: PLC comms error: {exc}") from exc

    # --- helpers ------------------------------------------------------------
    def _next_command_id(self) -> int:
        self._command_id += 1
        return self._command_id

    def _read(self, tag: str):
        return self.client.read(tag)

    def _write(self, tag: str, value) -> None:
        self.client.write(tag, value)

    def _pulse(self, tag: str) -> None:
        """Rising edge, hold, then clear — the PLC latches on the rising edge but
        only if the bit stays true across at least one scan. Holding for
        ``pulse_hold_s`` (several scans) makes the edge reliably visible; the PLC's
        ONS ensures it still triggers exactly once."""
        self._write(tag, True)
        if self.pulse_hold_s > 0:
            time.sleep(self.pulse_hold_s)
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
