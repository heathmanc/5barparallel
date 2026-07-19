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
_MAX_DRIVES = 2
_CSP_MAX = 65536

# header field offsets (see shm_layout_t)
_H_MAGIC, _H_NUM, _H_CYCLE = 0, 8, 16
_H_OP, _H_STOP = 28, 32
_H_CSP_START, _H_CSP_RUN, _H_CSP_LEN = 36, 40, 44
_DRIVE_BASE, _DRIVE_SZ = 52, 32
_CSP_BASE = _DRIVE_BASE + _MAX_DRIVES * _DRIVE_SZ
_SHM_SIZE = _CSP_BASE + _MAX_DRIVES * _CSP_MAX * 4

# drive_shm_t sub-offsets
_O_CTRL, _O_MODE, _O_TARGET, _O_DOUT = 0, 2, 4, 8
_I_STATUS, _I_MODED, _I_ACTUAL, _I_FERR, _I_ERR, _I_TORQ, _I_DIN = 12, 14, 16, 20, 24, 26, 28

_DEFAULT_DAEMON = Path(__file__).resolve().parents[3] / "igh" / "ec_master_daemon"


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
                if self._proc is not None:
                    self._u32_set(_H_STOP, 1)        # ask the daemon to exit
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
        while self._u32(_H_CSP_RUN) == 1 or self._u32(_H_CSP_START) == 1:
            if self._faulted():
                self.exchange()
                raise MasterError("drive faulted during CSP stream")
            if time.perf_counter() > deadline:
                raise MasterError("CSP stream did not complete in time")
            time.sleep(self.cycle_dt_s)
        self.exchange()

    # --- shared memory helpers --------------------------------------------- #
    def _launch_daemon(self) -> None:
        if not self.daemon_path.exists():
            raise MasterError(
                f"IgH daemon not built: {self.daemon_path} "
                "(run `make -C igh ETHERLAB=/opt/etherlab`)")
        cmd = [str(self.daemon_path), "--drives", str(self._num),
               "--cycle-ns", str(int(round(self.cycle_dt_s * 1e9)))]
        if os.geteuid() != 0:
            cmd = ["sudo"] + cmd
        try:
            self._logf = open(self.log_path, "w")
        except OSError:
            self._logf = subprocess.DEVNULL
        self._proc = subprocess.Popen(cmd, stderr=self._logf, stdout=self._logf)
        deadline = time.perf_counter() + 5.0
        while not os.path.exists(_SHM_PATH):
            if self._proc.poll() is not None:
                raise MasterError("IgH daemon exited immediately "
                                  "(is the IgH master running? `ethercat master`)")
            if time.perf_counter() > deadline:
                raise MasterError("IgH daemon did not create shared memory")
            time.sleep(0.05)

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

    def _faulted(self) -> bool:
        for d, pd in enumerate(self._drives):
            self._read_inputs(d, pd)
        return any(cia402.is_fault(pd.statusword) for pd in self._drives)
