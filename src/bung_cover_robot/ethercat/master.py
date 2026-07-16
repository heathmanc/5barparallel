"""EtherCAT master abstraction + an in-memory simulator.

The driver talks to the servo drives only through this interface, so the whole
motion stack runs in tests and in the GUI with nothing connected — exactly the
role the old ``SimulatedPlcClient`` played for the PLC.

The model is EtherCAT's own: each drive exposes a small block of **process data**
that the PC reads and writes, and one ``exchange()`` performs a single PDO cycle
(flush the outputs to the drives, latch the drives' inputs back). Outputs are the
CiA 402 controlword, the mode of operation, and the CSP target position; inputs
are the statusword, the mode display, and the actual position.

  * ``SimulatedEtherCatMaster`` — in-memory A6 drives that run the CiA 402 state
    machine and, in CSP, follow the streamed target position. Used by tests and
    the ``--sim-ec`` backend.
  * ``PysoemMaster`` (Stage 4) — the real master over ``pysoem``; the PDO map,
    distributed-clock sync, and real-time streamer land there. Imported lazily so
    this module works without pysoem installed.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List

from . import cia402


@dataclass
class DriveProcessData:
    """One drive's cyclic process image (the PC's read/write view).

    Outputs (PC -> drive): controlword, mode_of_operation, target_position.
    Inputs  (drive -> PC): statusword, mode_display, actual_position.
    """

    # outputs
    controlword: int = 0
    mode_of_operation: int = cia402.MODE_CSP
    target_position: int = 0
    # inputs
    statusword: int = 0
    mode_display: int = 0
    actual_position: int = 0


class EtherCatMaster(ABC):
    """Cyclic process-data exchange with a fixed set of CiA 402 drives."""

    cycle_dt_s: float = 0.002

    @property
    @abstractmethod
    def drives(self) -> List[DriveProcessData]:
        """The per-drive process images, index 0 = left, 1 = right."""

    @abstractmethod
    def open(self) -> "EtherCatMaster":
        ...

    @abstractmethod
    def close(self) -> None:
        ...

    @property
    @abstractmethod
    def is_open(self) -> bool:
        ...

    @abstractmethod
    def exchange(self) -> None:
        """Perform one PDO cycle: send outputs, receive inputs. On real hardware
        this blocks on the distributed-clock sync, which paces the loop."""

    @property
    def num_drives(self) -> int:
        return len(self.drives)


# --------------------------------------------------------------------------- #
# Simulator
# --------------------------------------------------------------------------- #
class _SimDrive:
    """One emulated A6: a CiA 402 state machine + a perfectly-following CSP axis.

    A compliant drive advances one CiA 402 transition per PDO cycle in response to
    the controlword, so the driver's arm/home loops (which poll the statusword
    between steps) behave just like real hardware. In CSP with Operation Enabled
    the actual position tracks the streamed target exactly (an ideal servo)."""

    def __init__(self) -> None:
        self.state = cia402.Cia402State.SWITCH_ON_DISABLED
        self.actual_position = 0
        self._home_offset = 0
        self._faulted = False
        self._prev_cw = 0

    def inject_fault(self) -> None:
        self._faulted = True

    def step(self, pd: DriveProcessData) -> None:
        cw, prev = pd.controlword & 0xFFFF, self._prev_cw
        pd.mode_display = pd.mode_of_operation

        if self._faulted:
            self.state = cia402.Cia402State.FAULT
            # Fault Reset is edge-triggered (bit 7 low -> high) and clears the fault.
            if (cw & cia402.CW_FAULT_RESET) and not (prev & cia402.CW_FAULT_RESET):
                self._faulted = False
                self.state = cia402.Cia402State.SWITCH_ON_DISABLED
        else:
            self.state = self._advance(self.state, cw)

        # CSP motion: an enabled drive in CSP follows the commanded target exactly.
        if (self.state is cia402.Cia402State.OPERATION_ENABLED
                and pd.mode_of_operation == cia402.MODE_CSP):
            self.actual_position = pd.target_position

        # Homing: an enabled drive in Homing mode with the start bit set finds its
        # switch immediately (sim), zeroing position at the home offset.
        if (self.state is cia402.Cia402State.OPERATION_ENABLED
                and pd.mode_of_operation == cia402.MODE_HOMING
                and (cw & (1 << 4))):
            self.actual_position = self._home_offset

        pd.statusword = self._statusword()
        pd.actual_position = self.actual_position
        self._prev_cw = cw

    @staticmethod
    def _advance(state, cw: int):
        S = cia402.Cia402State
        low = cw & 0x0F
        if (cw & 0x02) == 0 and (cw & cia402.CW_FAULT_RESET) == 0:
            return S.SWITCH_ON_DISABLED            # disable voltage (bit1 low)
        if low == 0x06:                            # Shutdown
            return S.READY_TO_SWITCH_ON
        if low == 0x07:                            # Switch On / Disable Operation
            if state in (S.READY_TO_SWITCH_ON, S.SWITCHED_ON, S.OPERATION_ENABLED):
                return S.SWITCHED_ON
            return state
        if low == 0x0F:                            # Enable Operation
            if state in (S.SWITCHED_ON, S.OPERATION_ENABLED, S.READY_TO_SWITCH_ON):
                return S.OPERATION_ENABLED
            return state
        return state

    def _statusword(self) -> int:
        S = cia402.Cia402State
        base = cia402.SW_VOLTAGE_ENABLED          # mains present
        word = {
            S.SWITCH_ON_DISABLED: cia402.SW_SWITCH_ON_DISABLED,
            S.READY_TO_SWITCH_ON: cia402.SW_READY_TO_SWITCH_ON | cia402.SW_QUICK_STOP,
            S.SWITCHED_ON: (cia402.SW_READY_TO_SWITCH_ON | cia402.SW_SWITCHED_ON
                            | cia402.SW_QUICK_STOP),
            S.OPERATION_ENABLED: (cia402.SW_READY_TO_SWITCH_ON | cia402.SW_SWITCHED_ON
                                  | cia402.SW_OPERATION_ENABLED | cia402.SW_QUICK_STOP),
            S.FAULT: cia402.SW_FAULT,
        }.get(self.state, 0)
        return base | word


class SimulatedEtherCatMaster(EtherCatMaster):
    """In-memory two-drive A6 network for tests and the ``--sim-ec`` backend."""

    def __init__(self, num_drives: int = 2, cycle_dt_s: float = 0.002) -> None:
        self.cycle_dt_s = cycle_dt_s
        self._drives = [DriveProcessData() for _ in range(num_drives)]
        self._sim = [_SimDrive() for _ in range(num_drives)]
        self._open = False

    @property
    def drives(self) -> List[DriveProcessData]:
        return self._drives

    def open(self) -> "SimulatedEtherCatMaster":
        self._open = True
        # Seed one exchange so statuswords read a real state immediately.
        self.exchange()
        return self

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def exchange(self) -> None:
        if not self._open:
            raise RuntimeError("master is not open")
        for pd, sim in zip(self._drives, self._sim):
            sim.step(pd)

    # --- test / bench helpers ---------------------------------------------- #
    def inject_fault(self, drive: int = 0) -> None:
        """Force a drive into FAULT (models a drive alarm / following error)."""
        self._sim[drive].inject_fault()


class PysoemMaster(EtherCatMaster):  # pragma: no cover - real hardware (Stage 4)
    """Real EtherCAT master over pysoem — the PDO map, distributed-clock sync, and
    SCHED_FIFO/mlockall real-time streamer land here in Stage 4 (needs the actual
    A6-EC drives and a PREEMPT_RT kernel to validate). Present now so the backend
    wiring is complete and fails with a clear message instead of an ImportError."""

    def __init__(self, ifname: str = "eth0", cycle_dt_s: float = 0.002) -> None:
        self.ifname = ifname
        self.cycle_dt_s = cycle_dt_s
        self._drives: List[DriveProcessData] = []

    @property
    def drives(self) -> List[DriveProcessData]:
        return self._drives

    def open(self) -> "PysoemMaster":
        raise NotImplementedError(
            "PysoemMaster is not implemented yet (Stage 4: pysoem PDO map + DC sync "
            "+ RT streamer). Use SimulatedEtherCatMaster / sim_ec for now."
        )

    def close(self) -> None:
        pass

    @property
    def is_open(self) -> bool:
        return False

    def exchange(self) -> None:
        raise NotImplementedError("PysoemMaster is not implemented yet (Stage 4).")
