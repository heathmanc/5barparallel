"""EtherNet/IP client for the CompactLogix, plus a reactive simulator.

Claude.md §11/§14: Python talks to the PLC via pycomm3. Everything goes through
the PlcClient interface so the driver is testable with no hardware:

  * CompactLogixClient — thin pycomm3 (LogixDriver) wrapper. pycomm3 is imported
    lazily so this module works without it installed.
  * SimulatedPlcClient — an in-memory PLC that *reacts* to the manual jog/home
    handshake exactly as the real ladder should (enable -> Enabled, MoveToTarget
    -> InPosition + Actual=Target + CompleteCommandID, HomeRequest -> Homed).
    Lets PlcRobotDriver run end-to-end in tests and gives the GUI a working
    "live" mode with nothing connected.
"""

from __future__ import annotations

import logging
import threading
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Tuple

from . import tags as T

logger = logging.getLogger(__name__)


class PlcError(Exception):
    """PLC connection or I/O failure."""


class PlcClient(ABC):
    """Minimal tag read/write surface."""

    @abstractmethod
    def connect(self) -> "PlcClient":
        ...

    @abstractmethod
    def close(self) -> None:
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        ...

    @abstractmethod
    def read(self, tag: str) -> Any:
        ...

    @abstractmethod
    def write(self, tag: str, value: Any) -> None:
        ...

    def read_many(self, tag_list) -> Dict[str, Any]:
        """Read several tags at once. Default loops ``read``; subclasses may
        batch. A tag that errors maps to ``None`` instead of aborting the batch,
        so a diagnostics poller shows every other tag."""
        out: Dict[str, Any] = {}
        for tag in tag_list:
            try:
                out[tag] = self.read(tag)
            except PlcError:
                out[tag] = None
        return out

    def __enter__(self) -> "PlcClient":
        return self.connect()

    def __exit__(self, *exc: object) -> None:
        self.close()


class CompactLogixClient(PlcClient):
    """pycomm3-backed client. ``path`` is e.g. '192.168.1.10/0' (IP/slot)."""

    def __init__(self, path: str) -> None:
        self.path = path
        self._plc = None
        # pycomm3's LogixDriver is not thread-safe; serialize all I/O so a
        # background diagnostics poller can read while the main thread commands.
        self._lock = threading.Lock()

    @staticmethod
    def _import_driver():
        try:
            from pycomm3 import LogixDriver
        except ImportError as exc:  # pragma: no cover - depends on host
            raise PlcError(
                "pycomm3 is not installed. `pip install pycomm3` for a real PLC, "
                "or use SimulatedPlcClient / --dry-run."
            ) from exc
        return LogixDriver

    def connect(self) -> "CompactLogixClient":
        if self.is_connected:
            return self
        LogixDriver = self._import_driver()
        try:
            self._plc = LogixDriver(self.path)
            self._plc.open()
        except Exception as exc:  # pragma: no cover - hardware path
            self._plc = None
            raise PlcError(f"failed to connect to PLC at {self.path}: {exc}") from exc
        logger.info("connected to CompactLogix at %s", self.path)
        return self

    def close(self) -> None:
        if self._plc is not None:
            try:
                self._plc.close()
            finally:
                self._plc = None

    @property
    def is_connected(self) -> bool:
        return self._plc is not None

    def read(self, tag: str) -> Any:  # pragma: no cover - hardware path
        if not self.is_connected:
            raise PlcError("not connected")
        with self._lock:
            result = self._plc.read(tag)
        if result is None or getattr(result, "error", None):
            raise PlcError(f"read {tag} failed: {getattr(result, 'error', 'no result')}")
        return result.value

    def read_many(self, tag_list) -> Dict[str, Any]:  # pragma: no cover - hardware path
        tag_list = list(tag_list)
        if not self.is_connected:
            raise PlcError("not connected")
        if not tag_list:
            return {}
        with self._lock:
            results = self._plc.read(*tag_list)
        if len(tag_list) == 1:
            results = [results]
        out: Dict[str, Any] = {}
        for tag, res in zip(tag_list, results):
            out[tag] = None if res is None or getattr(res, "error", None) else res.value
        return out

    def write(self, tag: str, value: Any) -> None:  # pragma: no cover - hardware path
        if not self.is_connected:
            raise PlcError("not connected")
        with self._lock:
            result = self._plc.write((tag, value))
        if result is None or getattr(result, "error", None):
            raise PlcError(f"write {tag}={value!r} failed: "
                           f"{getattr(result, 'error', 'no result')}")


class SimulatedPlcClient(PlcClient):
    """In-memory PLC that emulates both the manual jog/home ladder and the
    automatic pick/place handshake.

    ``home_angles`` is what the (simulated) homing routine reports as the
    reference position. Manual moves complete instantly and echo Target ->
    Actual; an automatic pick/place request runs the whole §11 job in one step
    (ending at the drop pose) and echoes CommandID -> CompleteCommandID.
    """

    def __init__(self, home_angles: Tuple[float, float] = (0.0, 0.0)) -> None:
        self._store: Dict[str, Any] = {}
        self._connected = False
        self._home_angles = home_angles
        self._seed()

    def _seed(self) -> None:
        for tag in T.all_tags():
            self._store.setdefault(tag, 0)
        # Idle-state defaults for the automatic handshake.
        self._store[T.Status.READY] = True
        self._store[T.Status.READY_FOR_VISION] = True

    def connect(self) -> "SimulatedPlcClient":
        self._connected = True
        return self

    def close(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def read(self, tag: str) -> Any:
        if not self._connected:
            raise PlcError("not connected")
        return self._store.get(tag, 0)

    def write(self, tag: str, value: Any) -> None:
        if not self._connected:
            raise PlcError("not connected")
        self._store[tag] = value
        self._react(tag, value)

    # --- emulated ladder ----------------------------------------------------
    def _react(self, tag: str, value: Any) -> None:
        if tag == T.Manual.ENABLE:
            self._store[T.Status.ENABLED] = bool(value)
            if not value:
                self._store[T.Status.MOVING] = False
        elif tag == T.Manual.HOME_REQUEST and value:
            if not self._store.get(T.Status.ENABLED):
                self._fault(1)  # cannot home while disabled
                return
            self._store[T.Status.ACTUAL_LEFT_DEG] = self._home_angles[0]
            self._store[T.Status.ACTUAL_RIGHT_DEG] = self._home_angles[1]
            self._store[T.Status.HOMED] = True
            self._store[T.Status.IN_POSITION] = True
        elif tag == T.Manual.MOVE_TO_TARGET and value:
            if not self._store.get(T.Status.ENABLED):
                self._fault(2)  # cannot move while disabled
                return
            self._store[T.Status.ACTUAL_LEFT_DEG] = self._store.get(
                T.Manual.TARGET_LEFT_DEG, 0.0
            )
            self._store[T.Status.ACTUAL_RIGHT_DEG] = self._store.get(
                T.Manual.TARGET_RIGHT_DEG, 0.0
            )
            self._store[T.Status.IN_POSITION] = True
            self._store[T.Status.MOVING] = False
            self._store[T.Status.COMPLETE_COMMAND_ID] = self._store.get(
                T.Manual.COMMAND_ID, 0
            )
        elif tag == T.Manual.ABORT and value:
            self._store[T.Status.MOVING] = False
        elif tag == T.Cmd.REQUEST_PICK_PLACE and value:
            self._run_pick_place()
        elif tag == T.Cmd.ABORT and value:
            self._store[T.Status.BUSY] = False
            self._store[T.Status.FAILED_COMMAND_ID] = self._store.get(
                T.Status.ACTIVE_COMMAND_ID, 0
            )
            self._store[T.Status.READY] = True
        elif tag == T.Cmd.RESET and value:
            self._store[T.Status.FAULTED] = False
            self._store[T.Status.FAULT_CODE] = 0
            self._store[T.Status.DONE] = False
            self._store[T.Status.BUSY] = False
            self._store[T.Status.READY] = True

    def _run_pick_place(self) -> None:
        """Emulate the §11 auto state machine end-to-end in one step."""
        if self._store.get(T.Status.FAULTED):
            return  # inhibited until reset
        if not self._store.get(T.Status.ENABLED) or not self._store.get(T.Status.HOMED):
            self._fault(3)  # auto job needs enabled + homed drives
            return
        cid = self._store.get(T.Cmd.COMMAND_ID, 0)
        self._store[T.Status.ACTIVE_COMMAND_ID] = cid
        self._store[T.Status.BUSY] = True
        self._store[T.Status.READY] = False
        # ... pick (vacuum on) -> place -> blow-off; ends at the drop pose.
        self._store[T.Status.ACTUAL_LEFT_DEG] = self._store.get(T.Target.DROP_LEFT_DEG, 0.0)
        self._store[T.Status.ACTUAL_RIGHT_DEG] = self._store.get(T.Target.DROP_RIGHT_DEG, 0.0)
        self._store[T.Status.VACUUM_OK] = False  # released after blow-off
        self._store[T.Status.COMPLETE_COMMAND_ID] = cid
        self._store[T.Status.DONE] = True
        self._store[T.Status.BUSY] = False
        self._store[T.Status.READY] = True

    def _fault(self, code: int) -> None:
        self._store[T.Status.FAULTED] = True
        self._store[T.Status.FAULT_CODE] = code
