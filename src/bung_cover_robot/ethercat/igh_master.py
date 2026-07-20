"""IgH EtherCAT master backend — Python side of the C RT daemon.

pysoem cannot generate the DC SYNC0 the ANCTL AS715N requires (it faults Er74.1
"No sync signal"); the IgH EtherLab master can (proven by igh/igh_test). So the
real-time DC loop lives in a small C daemon (``igh/ec_master_daemon``) that owns
the cyclic exchange and exposes each drive's process image + a CSP setpoint
buffer in POSIX shared memory. ``IgHMaster`` maps that memory and implements the
same ``EtherCatMaster`` interface as ``PysoemMaster`` / ``SimulatedEtherCatMaster``
— so ``EtherCatRobotDriver`` and the whole stack above the master seam are
unchanged, and the RT loop runs in C, off the Python GIL.

The shared-memory layout mirrors ``igh/ec_master_daemon.c`` byte-for-byte
(packed, little-endian). Keep the two in sync.
"""

from __future__ import annotations

import mmap
import os
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import List

from . import cia402
from .master import CspTargets, DriveProcessData, EtherCatMaster, MasterError

_SHM_PATH = "/dev/shm/bcr_ethercat"
_SHM_MAGIC = 0x42435231
_SHM_ABI = 4
_MAX_DRIVES = 2
_CSP_MAX = 65536

# header field offsets (see shm_layout_t)
_H_MAGIC, _H_ABI, _H_NUM, _H_CYCLE = 0, 4, 8, 16
_H_OP, _H_STOP = 28, 32
_H_CSP_START, _H_CSP_RUN, _H_CSP_LEN = 36, 40, 44
_DRIVE_BASE, _DRIVE_SZ = 52, 32
_CSP_BASE = _DRIVE_BASE + _MAX_DRIVES * _DRIVE_SZ
# SDO channel appended after the CSP buffer (existing offsets unchanged).
_SDO_BASE = _CSP_BASE + _MAX_DRIVES * _CSP_MAX * 4
_SDO_REQ, _SDO_DONE, _SDO_RESULT = _SDO_BASE, _SDO_BASE + 4, _SDO_BASE + 8
_SDO_INDEX, _SDO_SUB, _SDO_SIZE, _SDO_VALUE = (
    _SDO_BASE + 12, _SDO_BASE + 16, _SDO_BASE + 20, _SDO_BASE + 24)
_SDO_DRIVE, _SDO_OP = _SDO_BASE + 28, _SDO_BASE + 32
# ESC link/CRC error counters (ABI 4): raw register block 0x0300..0x0313 per
# drive, published ~1 Hz by the daemon; link_reset=1 asks it to zero them.
_LINK_RESET = _SDO_BASE + 36
_LINK_SEQ = _SDO_BASE + 40                  # u32 per drive
_LINK_RAW = _SDO_BASE + 40 + 4 * _MAX_DRIVES
_LINK_RAW_SZ = 20
_SHM_SIZE = _LINK_RAW + _MAX_DRIVES * _LINK_RAW_SZ

# drive_shm_t sub-offsets
_O_CTRL, _O_MODE, _O_TARGET, _O_DOUT = 0, 2, 4, 8
_I_STATUS, _I_MODED, _I_ACTUAL, _I_FERR, _I_ERR, _I_TORQ, _I_DIN = 12, 14, 16, 20, 24, 26, 28

_DEFAULT_DAEMON = Path(__file__).resolve().parents[3] / "igh" / "ec_master_daemon"


def parse_link_raw(raw: bytes) -> dict:
    """Decode the ESC error-counter register block (0x0300..0x0313, 20 bytes)
    into named counters. Byte map per the ET1100 register set:
    [0]=invalid frame p0, [1]=RX error p0, [2]/[3]=port 1, [8]/[9]=forwarded
    p0/p1, [12]=processing-unit err, [13]=PDI err, [16]/[17]=lost link p0/p1."""
    b = bytes(raw) + b"\x00" * _LINK_RAW_SZ            # tolerate short input
    ports = [
        {"invalid_frame": b[0], "rx_error": b[1], "forwarded": b[8],
         "lost_link": b[16]},
        {"invalid_frame": b[2], "rx_error": b[3], "forwarded": b[9],
         "lost_link": b[17]},
    ]
    return {"ports": ports, "pu_error": b[12], "pdi_error": b[13]}


def link_error_total(counters: dict) -> int:
    """Sum of every physical-layer error counter (0 == clean link)."""
    t = counters.get("pu_error", 0) + counters.get("pdi_error", 0)
    for p in counters.get("ports", []):
        t += p["rx_error"] + p["invalid_frame"] + p["forwarded"] + p["lost_link"]
    return t


class IgHMaster(EtherCatMaster):
    """EtherCAT master over the IgH RT daemon via shared memory.

    ``open()`` maps an already-running daemon's shared memory, or launches the
    daemon itself (``auto_launch``). ``exchange()`` syncs the per-drive images
    with shared memory; ``run_csp()`` loads the setpoint buffer the daemon plays
    out one entry per DC cycle.
    """

    #: seconds to wait for the daemon to climb INIT->...->OP (DC settle takes ~5 s)
    op_timeout_s = 20.0
    #: where the auto-launched daemon's stderr goes, for diagnostics
    log_path = "/tmp/bcr_ec_daemon.log"

    def __init__(self, num_drives: int = 2, cycle_dt_s: float = 0.002,
                 auto_launch: bool = True,
                 daemon_path: str | Path = _DEFAULT_DAEMON) -> None:
        self.cycle_dt_s = cycle_dt_s
        self._num = num_drives
        self._drives = [DriveProcessData() for _ in range(num_drives)]
        self.auto_launch = auto_launch
        self.daemon_path = Path(daemon_path)
        self._proc: subprocess.Popen | None = None
        self._logf = None
        self._mm: mmap.mmap | None = None
        self._open = False
        # background reader keeps `drives` live from shared memory (the C daemon
        # owns the RT loop, so nothing else refreshes the Python-side images).
        self._reader: threading.Thread | None = None
        self._reader_stop = threading.Event()

    # --- EtherCatMaster interface ------------------------------------------ #
    @property
    def drives(self) -> List[DriveProcessData]:
        return self._drives

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> "IgHMaster":
        if not os.path.exists(_SHM_PATH) and self.auto_launch:
            self._launch_daemon()
        self._map_shm()
        running_n = self._u32(_H_NUM)
        if running_n != self._num:
            self.close()
            raise MasterError(
                f"a daemon is already running with {running_n} drive(s), not "
                f"{self._num}. Stop it first: sudo pkill ec_master_daemon")
        # wait for the daemon to reach OP (INIT->PREOP->SAFEOP->OP + DC settle)
        deadline = time.perf_counter() + self.op_timeout_s
        while self._u32(_H_OP) != 1:
            if time.perf_counter() > deadline:
                self.close()
                raise MasterError(
                    f"IgH daemon did not reach OP within {self.op_timeout_s:.0f} s "
                    f"(see {self.log_path}; check drive power / the ethercat master)")
            time.sleep(0.05)
        self._open = True
        self.exchange()
        self._reader_stop.clear()
        self._reader = threading.Thread(target=self._reader_loop,
                                        name="igh-reader", daemon=True)
        self._reader.start()
        return self

    def _reader_loop(self) -> None:
        while not self._reader_stop.is_set():
            try:
                for d, pd in enumerate(self._drives):
                    self._read_inputs(d, pd)
                    # Never command a stale target: while the drive is disabled,
                    # hold target = actual so enabling can't jump (drive Er87.1).
                    if not cia402.is_operation_enabled(pd.statusword):
                        pd.target_position = pd.actual_position
            except Exception:  # noqa: BLE001 - display refresh must not die
                pass
            self._reader_stop.wait(0.01)     # ~100 Hz

    def close(self) -> None:
        self._reader_stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._mm is not None:
            try:
                self._u32_set(_H_STOP, 1)            # always ask the daemon to exit
            finally:
                self._mm.close()
                self._mm = None
        if self._proc is not None:
            try:
                self._proc.wait(timeout=2.0)
            except Exception:  # noqa: BLE001
                self._proc.terminate()
            self._proc = None
        if self._logf not in (None, subprocess.DEVNULL):
            try:
                self._logf.close()
            except Exception:  # noqa: BLE001
                pass
        self._logf = None
        self._open = False

    def exchange(self) -> None:
        if not self._open and self._mm is None:
            raise MasterError("master is not open")
        for d, pd in enumerate(self._drives):
            self._write_outputs(d, pd)
        start = self._u64(_H_CYCLE)
        deadline = time.perf_counter() + max(0.05, 20 * self.cycle_dt_s)
        while self._u64(_H_CYCLE) == start:
            if time.perf_counter() > deadline:
                raise MasterError("IgH daemon stalled (no DC cycle)")
            time.sleep(self.cycle_dt_s / 2)
        for d, pd in enumerate(self._drives):
            self._read_inputs(d, pd)

    def run_csp(self, targets: CspTargets) -> None:
        if not self._open:
            raise MasterError("master is not open")
        targets = list(targets)
        n = len(targets)
        if n == 0:
            return
        if n > _CSP_MAX:
            raise MasterError(f"CSP stream too long ({n} > {_CSP_MAX})")
        mm = self._mm
        for i, row in enumerate(targets):
            for d in range(self._num):
                off = _CSP_BASE + (d * _CSP_MAX + i) * 4
                struct.pack_into("<i", mm, off, int(row[d]))
        self._u32_set(_H_CSP_LEN, n)
        self._u32_set(_H_CSP_START, 1)               # daemon latches + streams
        deadline = time.perf_counter() + n * self.cycle_dt_s + 1.0
        # wait until the daemon finishes playing the buffer (csp_running -> 0)
        fault_hits = 0
        while self._u32(_H_CSP_RUN) == 1 or self._u32(_H_CSP_START) == 1:
            # A real fault LATCHES — require it on two consecutive samples so a
            # single torn/transient statusword read can't abort a good stream.
            if self._faulted():
                fault_hits += 1
                if fault_hits >= 2:
                    self.exchange()
                    detail = ", ".join(
                        f"drive {i}: sw=0x{pd.statusword:04X} err=0x{pd.error_code:04X}"
                        for i, pd in enumerate(self._drives))
                    raise MasterError(f"drive faulted during CSP stream ({detail})")
            else:
                fault_hits = 0
            if time.perf_counter() > deadline:
                raise MasterError("CSP stream did not complete in time")
            time.sleep(self.cycle_dt_s)
        # Hold at the FINAL streamed position: otherwise the closing exchange()
        # would write the stale pre-move target and the drive would drive back.
        last = targets[-1]
        for d in range(self._num):
            self._drives[d].target_position = int(last[d])
        self.exchange()

    def _sdo_request(self, op: int, index: int, sub: int, drive: int,
                     value: int, size: int, timeout_s: float) -> int:
        """Run one SDO mailbox op via the daemon's worker (off the RT loop) and
        return the result value (meaningful for reads). Raises on abort/timeout."""
        if self._mm is None:
            raise MasterError("master is not open")
        self._u32_set(_SDO_INDEX, index & 0xFFFF)
        self._u32_set(_SDO_SUB, sub & 0xFF)
        self._u32_set(_SDO_SIZE, size)
        self._u32_set(_SDO_DRIVE, drive & 0xFFFF)
        self._u32_set(_SDO_OP, op & 0x1)
        struct.pack_into("<i", self._mm, _SDO_VALUE, int(value))
        self._u32_set(_SDO_DONE, 0)
        self._u32_set(_SDO_REQ, 1)
        verb = "read" if op == 1 else "write"
        deadline = time.perf_counter() + timeout_s
        while self._u32(_SDO_DONE) != 1:
            if time.perf_counter() > deadline:
                self._u32_set(_SDO_REQ, 0)
                raise MasterError(f"SDO {verb} 0x{index:04X}:{sub} (drive {drive}) timed out")
            time.sleep(0.002)
        result = struct.unpack_from("<i", self._mm, _SDO_RESULT)[0]
        readback = struct.unpack_from("<i", self._mm, _SDO_VALUE)[0]
        self._u32_set(_SDO_REQ, 0)
        if result != 0:
            raise MasterError(
                f"SDO {verb} 0x{index:04X}:{sub} (drive {drive}) failed "
                f"(code 0x{result & 0xFFFFFFFF:08X})")
        return readback

    def sdo_write(self, index: int, sub: int, value: int, size: int = 4,
                  drive: int = 0, timeout_s: float = 2.0) -> None:
        """Download an SDO to ``drive`` via the daemon's SDO worker. Raises
        MasterError on abort/timeout. This is what makes the Drives-tab 'Apply'
        write parameters/gains to the live drive."""
        self._sdo_request(0, index, sub, drive, int(value), size, timeout_s)

    def sdo_read(self, index: int, sub: int, size: int = 4,
                 drive: int = 0, timeout_s: float = 2.0) -> int:
        """Upload (read) an SDO from ``drive`` and return its value. Used by the
        Drives-tab Refresh to show each drive's actual parameter values."""
        return self._sdo_request(1, index, sub, drive, 0, size, timeout_s)

    # --- link/CRC error counters -------------------------------------------- #
    def link_counters(self) -> List[dict]:
        """Per-drive ESC error counters (parsed), fresh from shared memory."""
        out = []
        for d in range(self._num):
            raw = self._mm[_LINK_RAW + d * _LINK_RAW_SZ:
                           _LINK_RAW + (d + 1) * _LINK_RAW_SZ]
            out.append(parse_link_raw(raw))
        return out

    def reset_link_counters(self) -> None:
        """Ask the daemon to zero every slave's hardware error counters."""
        if self._mm is not None:
            self._u32_set(_LINK_RESET, 1)

    # --- shared memory helpers --------------------------------------------- #
    def _launch_daemon(self) -> None:
        if not self.daemon_path.exists():
            raise MasterError(
                f"IgH daemon not built: {self.daemon_path} "
                "(run `make -C igh ETHERLAB=/opt/etherlab`)")
        cmd = [str(self.daemon_path), "--drives", str(self._num),
               "--cycle-ns", str(int(round(self.cycle_dt_s * 1e9)))]
        if os.geteuid() != 0:
            # -n: NEVER prompt. From a GUI there is no TTY, so an expired sudo
            # credential cache used to make the launch hang silently; now it
            # fails fast with a clear message instead.
            cmd = ["sudo", "-n"] + cmd
        try:
            self._logf = open(self.log_path, "w")
        except OSError:
            self._logf = subprocess.DEVNULL
        self._proc = subprocess.Popen(cmd, stderr=self._logf, stdout=self._logf)
        deadline = time.perf_counter() + 5.0
        while not os.path.exists(_SHM_PATH):
            if self._proc.poll() is not None:
                raise MasterError(
                    "IgH daemon exited immediately. If the log says 'sudo: a "
                    "password is required', run `sudo -v` in a terminal and "
                    "launch the app from it (or start the daemon yourself: "
                    "sudo igh/ec_master_daemon --drives N)."
                    + self._log_tail())
            if time.perf_counter() > deadline:
                raise MasterError("IgH daemon did not create shared memory"
                                  + self._log_tail())
            time.sleep(0.05)

    def _log_tail(self, lines: int = 6) -> str:
        try:
            txt = Path(self.log_path).read_text().strip().splitlines()
        except OSError:
            return ""
        if not txt:
            return ""
        return "\n--- daemon log ---\n" + "\n".join(txt[-lines:])

    def _map_shm(self) -> None:
        try:
            fd = os.open(_SHM_PATH, os.O_RDWR)
        except FileNotFoundError as exc:
            raise MasterError(
                f"IgH shared memory {_SHM_PATH} not found — start the daemon "
                "(`sudo igh/ec_master_daemon --drives 1`) or use auto_launch"
            ) from exc
        try:
            self._mm = mmap.mmap(fd, _SHM_SIZE, mmap.MAP_SHARED,
                                 mmap.PROT_READ | mmap.PROT_WRITE)
        finally:
            os.close(fd)
        if self._u32(_H_MAGIC) != _SHM_MAGIC:
            raise MasterError("shared-memory magic mismatch — daemon/Python ABI skew")
        abi = self._u32(_H_ABI)
        if abi != _SHM_ABI:
            self._mm.close()
            self._mm = None
            raise MasterError(
                f"daemon ABI {abi} != {_SHM_ABI} — rebuild it (make -C igh) and stop "
                "the old one (sudo pkill ec_master_daemon)")

    def _u32(self, off: int) -> int:
        return struct.unpack_from("<I", self._mm, off)[0]

    def _u32_set(self, off: int, val: int) -> None:
        struct.pack_into("<I", self._mm, off, val & 0xFFFFFFFF)

    def _u64(self, off: int) -> int:
        return struct.unpack_from("<Q", self._mm, off)[0]

    def _write_outputs(self, d: int, pd: DriveProcessData) -> None:
        base = _DRIVE_BASE + d * _DRIVE_SZ
        struct.pack_into("<H", self._mm, base + _O_CTRL, pd.controlword & 0xFFFF)
        struct.pack_into("<b", self._mm, base + _O_MODE, pd.mode_of_operation)
        struct.pack_into("<i", self._mm, base + _O_TARGET, int(pd.target_position))
        struct.pack_into("<I", self._mm, base + _O_DOUT, pd.digital_outputs & 0xFFFFFFFF)

    def _read_inputs(self, d: int, pd: DriveProcessData) -> None:
        base = _DRIVE_BASE + d * _DRIVE_SZ
        pd.statusword = struct.unpack_from("<H", self._mm, base + _I_STATUS)[0]
        pd.actual_position = struct.unpack_from("<i", self._mm, base + _I_ACTUAL)[0]
        pd.following_error = struct.unpack_from("<i", self._mm, base + _I_FERR)[0]
        pd.error_code = struct.unpack_from("<H", self._mm, base + _I_ERR)[0]
        pd.torque_actual = struct.unpack_from("<h", self._mm, base + _I_TORQ)[0]
        pd.digital_inputs = struct.unpack_from("<I", self._mm, base + _I_DIN)[0]
        pd.mode_display = pd.mode_of_operation
        raw = self._mm[_LINK_RAW + d * _LINK_RAW_SZ:
                       _LINK_RAW + (d + 1) * _LINK_RAW_SZ]
        pd.link_errors = parse_link_raw(raw)

    def _faulted(self) -> bool:
        for d, pd in enumerate(self._drives):
            self._read_inputs(d, pd)
        return any(cia402.is_fault(pd.statusword) for pd in self._drives)
