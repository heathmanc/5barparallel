"""EtherCAT master abstraction, an in-memory simulator, and the pysoem master.

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
  * ``PysoemMaster`` — the real master over ``pysoem`` (Stage 4): a free-running
    real-time thread owns the cyclic PDO exchange and the CSP setpoint streaming,
    synchronized to the EtherCAT distributed clock. BENCH-UNTESTED — it needs the
    actual A6-EC drives and a PREEMPT_RT kernel; see docs/ethercat_bringup.md.

``run_csp`` is the streaming primitive: give it the per-cycle (left, right) drive
targets and it plays them out one per PDO cycle. The simulator runs it
synchronously; the real master hands it to the RT thread. Either raises
``MasterError`` if a drive faults mid-stream.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, NamedTuple, Sequence, Tuple

from . import cia402

logger = logging.getLogger(__name__)

CspTargets = Sequence[Tuple[int, int]]


class MasterError(Exception):
    """EtherCAT master / drive failure (fault mid-stream, lost frames, timeout)."""


# --------------------------------------------------------------------------- #
# PDO layout  (pure, testable — no pysoem needed)
# --------------------------------------------------------------------------- #
# Matches the ANCTL AS715N (StepperOnline A6-EC) *native* fixed PDO map, verified
# on the bench with scripts/ec_inspect.py. Little-endian, packed:
#
#   RxPDO 0x1701 (PC->drive, 12 B):
#       Controlword(0x6040,U16) | TargetPosition(0x607A,S32)
#       | TouchProbe(0x60B8,U16) | DigitalOutputs(0x60FE:1,U32)
#   TxPDO 0x1B01 (drive->PC, 28 B):
#       ErrorCode(0x603F,U16) | Statusword(0x6041,U16) | PositionActual(0x6064,S32)
#       | TorqueActual(0x6077,S16) | FollowingError(0x60F4,S32)
#       | TouchProbeStatus(0x60B9,U16) | TouchProbe1(0x60BA,S32)
#       | TouchProbe2(0x60BC,S32) | DigitalInputs(0x60FD,U32)
#
# Mode of operation (0x6060) is NOT cyclic in this map — it's written once over
# SDO in _configure_slave (the drive powers up in CSP=8). The touch-probe and
# touch-probe-status words are unused here (packed/ignored as 0).
_RX_FMT = "<HiHI"        # controlword, target_position, touch_probe, digital_outputs
_TX_FMT = "<HHihiHiiI"   # errcode, statusword, actual, torque, foll_err, tp_status, tp1, tp2, dig_in
RX_SIZE = struct.calcsize(_RX_FMT)   # 12 bytes
TX_SIZE = struct.calcsize(_TX_FMT)   # 28 bytes

# CiA 402 object dictionary indices (documented for bring-up / SDO access).
OD_CONTROLWORD = 0x6040
OD_STATUSWORD = 0x6041
OD_MODES_OF_OPERATION = 0x6060
OD_MODES_DISPLAY = 0x6061
OD_TARGET_POSITION = 0x607A
OD_POSITION_ACTUAL = 0x6064
OD_TORQUE_ACTUAL = 0x6077
OD_HOMING_METHOD = 0x6098
OD_FOLLOWING_ERROR = 0x60F4
OD_ERROR_CODE = 0x603F
OD_DIGITAL_INPUTS = 0x60FD
OD_DIGITAL_OUTPUTS = 0x60FE


class DriveInputs(NamedTuple):
    """Decoded TxPDO fields the app cares about (the touch-probe words are dropped)."""

    statusword: int
    actual_position: int
    following_error: int
    digital_inputs: int
    error_code: int
    torque_actual: int


def pack_outputs(controlword: int, target_position: int,
                 digital_outputs: int = 0, touch_probe: int = 0) -> bytes:
    """Pack a drive's RxPDO (0x1701) output image to bytes."""
    return struct.pack(_RX_FMT, controlword & 0xFFFF, int(target_position),
                       touch_probe & 0xFFFF, digital_outputs & 0xFFFFFFFF)


def unpack_inputs(data: bytes) -> DriveInputs:
    """Unpack a drive's TxPDO (0x1B01) input image. Signed fields (position,
    following error, torque) come back signed straight from struct."""
    (errcode, status, actual, torque, foll_err,
     _tp_status, _tp1, _tp2, dig_in) = struct.unpack(_TX_FMT, data[:TX_SIZE])
    return DriveInputs(statusword=status, actual_position=actual,
                       following_error=foll_err, digital_inputs=dig_in,
                       error_code=errcode, torque_actual=torque)


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
    digital_outputs: int = 0        # RxPDO 0x60FE:1 (drive DOs — e.g. vacuum, later)
    # inputs
    statusword: int = 0
    mode_display: int = 0
    actual_position: int = 0
    following_error: int = 0        # TxPDO 0x60F4
    error_code: int = 0             # TxPDO 0x603F (0 = healthy)
    torque_actual: int = 0          # TxPDO 0x6077
    # CiA 402 digital inputs (0x60FD): bit0 = negative limit, bit1 = positive
    # limit, bit2 = home switch. Live in the AS715N TxPDO, so the Drives page
    # shows real switch states.
    digital_inputs: int = 0


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
        """Perform (or wait for) one PDO cycle so fresh inputs are visible. Used
        for the low-rate CiA 402 state-machine steps (enable/home/reset)."""

    @abstractmethod
    def run_csp(self, targets: CspTargets) -> None:
        """Stream per-cycle (left_counts, right_counts) CSP targets, one per PDO
        cycle, returning when the last has been applied. Raises MasterError if a
        drive faults mid-stream."""

    @property
    def num_drives(self) -> int:
        return len(self.drives)

    def _faulted(self) -> bool:
        return any(cia402.is_fault(d.statusword) for d in self.drives)


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
            if (cw & cia402.CW_FAULT_RESET) and not (prev & cia402.CW_FAULT_RESET):
                self._faulted = False
                self.state = cia402.Cia402State.SWITCH_ON_DISABLED
        else:
            self.state = self._advance(self.state, cw)

        if (self.state is cia402.Cia402State.OPERATION_ENABLED
                and pd.mode_of_operation == cia402.MODE_CSP):
            self.actual_position = pd.target_position

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
            return S.SWITCH_ON_DISABLED
        if low == 0x06:
            return S.READY_TO_SWITCH_ON
        if low == 0x07:
            if state in (S.READY_TO_SWITCH_ON, S.SWITCHED_ON, S.OPERATION_ENABLED):
                return S.SWITCHED_ON
            return state
        if low == 0x0F:
            if state in (S.SWITCHED_ON, S.OPERATION_ENABLED, S.READY_TO_SWITCH_ON):
                return S.OPERATION_ENABLED
            return state
        return state

    def _statusword(self) -> int:
        S = cia402.Cia402State
        base = cia402.SW_VOLTAGE_ENABLED
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

    def run_csp(self, targets: CspTargets) -> None:
        for t0, t1 in targets:
            self._drives[0].target_position = int(t0)
            self._drives[1].target_position = int(t1)
            self.exchange()
            if self._faulted():
                raise MasterError("drive faulted during CSP stream")

    # --- test / bench helpers ---------------------------------------------- #
    def inject_fault(self, drive: int = 0) -> None:
        """Force a drive into FAULT (models a drive alarm / following error)."""
        self._sim[drive].inject_fault()


# --------------------------------------------------------------------------- #
# Real hardware — pysoem  (BENCH-UNTESTED scaffolding, Stage 4)
# --------------------------------------------------------------------------- #
def set_realtime(priority: int = 80) -> bool:
    """Best-effort: pin this thread to SCHED_FIFO and lock memory (mlockall) so
    the CSP loop isn't preempted or paged. Needs CAP_SYS_NICE / root. Returns
    True if both succeeded; logs and returns False otherwise (dev machines)."""
    ok = True
    try:
        import os
        param = os.sched_param(priority)
        os.sched_setscheduler(0, os.SCHED_FIFO, param)  # type: ignore[attr-defined]
    except (OSError, AttributeError, PermissionError) as exc:  # pragma: no cover
        logger.warning("SCHED_FIFO not set (%s) — jitter will be higher", exc)
        ok = False
    try:  # pragma: no cover - platform dependent
        import ctypes
        MCL_CURRENT, MCL_FUTURE = 1, 2
        if ctypes.CDLL("libc.so.6", use_errno=True).mlockall(MCL_CURRENT | MCL_FUTURE) != 0:
            raise OSError(ctypes.get_errno(), "mlockall failed")
    except (OSError, AttributeError) as exc:  # pragma: no cover
        logger.warning("mlockall failed (%s) — page faults may cause jitter", exc)
        ok = False
    return ok


class PysoemMaster(EtherCatMaster):  # pragma: no cover - needs real drives + RT
    """Real EtherCAT master over pysoem. BENCH-UNTESTED — validate on hardware.

    A background real-time thread owns the whole cyclic exchange: every DC cycle
    it packs each drive's output image, ``send_processdata`` / ``receive_processdata``,
    and unpacks the inputs back. The main thread only mutates the drive images
    (controlword / target) and reads status; ``run_csp`` loads a setpoint array
    the RT thread plays out one entry per cycle, so the tight loop does no
    kinematics and no allocation.

    See docs/ethercat_bringup.md for the PDO map, DC, homing, and STO steps.
    """

    def __init__(
        self,
        ifname: str = "eth0",
        cycle_dt_s: float = 0.002,
        num_drives: int = 2,
        rt_priority: int = 80,
        recv_timeout_us: int = 2000,
    ) -> None:
        self.ifname = ifname
        self.cycle_dt_s = cycle_dt_s
        self.rt_priority = rt_priority
        self.recv_timeout_us = recv_timeout_us
        self._num = num_drives
        self._drives = [DriveProcessData() for _ in range(num_drives)]
        self._master = None
        self._slaves: List[object] = []
        self._open = False
        # RT-thread control (single-writer / single-reader on simple fields).
        self._rt_thread = None
        self._rt_stop = threading.Event()
        self._cycle_count = 0
        self._csp: List[Tuple[int, int]] = []
        self._csp_index = 0
        self._csp_running = False
        self._fault = False
        self._wkc_bad = 0

    @property
    def drives(self) -> List[DriveProcessData]:
        return self._drives

    @property
    def is_open(self) -> bool:
        return self._open

    # --- lifecycle --------------------------------------------------------- #
    def open(self) -> "PysoemMaster":
        try:
            import pysoem
        except ImportError as exc:
            raise MasterError(
                "pysoem is not installed. `pip install pysoem` on the control PC, "
                "or use SimulatedEtherCatMaster / --sim-ec."
            ) from exc

        m = pysoem.Master()
        try:
            m.open(self.ifname)
            found = m.config_init()
        except Exception as exc:
            raise MasterError(
                f"failed to open EtherCAT on {self.ifname!r}: {exc} "
                "(check the interface name and that you have raw-socket privileges)"
            ) from exc
        if found < self._num:
            m.close()
            raise MasterError(
                f"expected {self._num} EtherCAT slaves, found {found} on {self.ifname}")
        self._slaves = list(m.slaves)[: self._num]
        for s in self._slaves:
            s.config_func = self._configure_slave      # PDO map + CSP setup per drive
        m.config_map()
        if m.config_dc():                              # distributed clocks
            logger.info("EtherCAT DC configured")
        # SAFE_OP -> OP; OP requires processdata already flowing, so prime it.
        m.state_check(pysoem.SAFEOP_STATE, 50_000)
        m.send_processdata()
        m.receive_processdata(self.recv_timeout_us)
        m.state = pysoem.OP_STATE
        m.write_state()
        if m.state_check(pysoem.OP_STATE, 50_000) != pysoem.OP_STATE:
            m.close()
            raise MasterError("slaves did not reach OP state")
        self._master = m
        self._open = True
        self._rt_stop.clear()
        self._rt_thread = threading.Thread(
            target=self._rt_loop, name="ethercat-rt", daemon=True)
        self._rt_thread.start()
        return self

    def close(self) -> None:
        self._rt_stop.set()
        if self._rt_thread is not None:
            self._rt_thread.join(timeout=1.0)
            self._rt_thread = None
        if self._master is not None:
            try:
                import pysoem
                self._master.state = pysoem.INIT_STATE
                self._master.write_state()
            finally:
                self._master.close()
                self._master = None
        self._open = False

    def _configure_slave(self, slave_pos: int) -> None:
        """Per-drive SDO setup run by pysoem during config_map. The AS715N's native
        fixed PDO map (0x1701 / 0x1B01) already carries what we need, so we don't
        remap it — we just select CSP over SDO, since mode-of-operation (0x6060) is
        not a cyclic object in this map. (Verified with scripts/ec_inspect.py; see
        docs/ethercat_bringup.md §3.)"""
        s = self._master.slaves[slave_pos]
        s.sdo_write(OD_MODES_OF_OPERATION, 0, bytes([cia402.MODE_CSP]))

    # --- real-time loop ---------------------------------------------------- #
    def _rt_loop(self) -> None:
        import pysoem
        set_realtime(self.rt_priority)
        next_t = time.perf_counter()
        while not self._rt_stop.is_set():
            # advance the CSP stream (RT-owned: no allocation here)
            if self._csp_running:
                if self._csp_index < len(self._csp):
                    t0, t1 = self._csp[self._csp_index]
                    self._drives[0].target_position = t0
                    self._drives[1].target_position = t1
                    self._csp_index += 1
                else:
                    self._csp_running = False
            # pack outputs -> send -> receive -> unpack inputs
            for pd, s in zip(self._drives, self._slaves):
                s.output = pack_outputs(pd.controlword, pd.target_position,
                                        pd.digital_outputs)
            self._master.send_processdata()
            wkc = self._master.receive_processdata(self.recv_timeout_us)
            if wkc < len(self._slaves):
                self._wkc_bad += 1
            for pd, s in zip(self._drives, self._slaves):
                inp = unpack_inputs(bytes(s.input))
                pd.statusword = inp.statusword
                pd.actual_position = inp.actual_position
                pd.following_error = inp.following_error
                pd.digital_inputs = inp.digital_inputs
                pd.error_code = inp.error_code
                pd.torque_actual = inp.torque_actual
                pd.mode_display = pd.mode_of_operation   # mode isn't cyclic in this map
            if self._faulted():
                self._fault = True
                self._csp_running = False
            self._cycle_count += 1
            next_t += self.cycle_dt_s
            slack = next_t - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.perf_counter()   # we overran; resync the phase

    # --- EtherCatMaster API ------------------------------------------------ #
    def exchange(self) -> None:
        """Block until the RT thread completes one more cycle, so the caller's
        next status read is fresh. Used by the low-rate enable/home/reset loops."""
        if not self._open:
            raise MasterError("master is not open")
        start = self._cycle_count
        deadline = time.perf_counter() + max(0.05, 20 * self.cycle_dt_s)
        while self._cycle_count == start:
            if time.perf_counter() > deadline:
                raise MasterError("RT thread stalled (no PDO cycle)")
            time.sleep(self.cycle_dt_s / 2)

    def run_csp(self, targets: CspTargets) -> None:
        if not self._open:
            raise MasterError("master is not open")
        self._fault = False
        self._csp = list(targets)
        self._csp_index = 0
        self._csp_running = True
        deadline = time.perf_counter() + len(self._csp) * self.cycle_dt_s + 1.0
        while self._csp_running:
            if self._fault:
                raise MasterError(f"drive faulted during CSP stream (code {self.fault_code()})")
            if time.perf_counter() > deadline:
                self._csp_running = False
                raise MasterError("CSP stream did not complete in time")
            time.sleep(self.cycle_dt_s)

    def fault_code(self) -> int:
        return self._wkc_bad
