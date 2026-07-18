#!/usr/bin/env python3
"""Read-only EtherCAT drive inspector — a one-time commissioning diagnostic.

Enumerates the drives on an interface and, for each, reads its identity, live
CiA 402 state / position / error over SDO, and — the important part for bring-up —
dumps its **actual PDO assignment** (0x1C12/0x1C13 -> mapped objects). That tells
us whether the drive's default RxPDO/TxPDO match what ``ethercat/master.py`` packs
(``<HbI``: controlword|mode|target  /  statusword|mode|actual). If they don't, the
cyclic exchange would send/receive garbage, so this check comes *before* any OP /
cyclic traffic.

SAFE: SDO reads work in PRE-OP; this script never enables a drive, never writes,
and never commands motion. Run it with the venv's Python under sudo (raw socket):

    sudo .venv/bin/python scripts/ec_inspect.py --ifname ecat0

Nothing here produces torque.
"""

from __future__ import annotations

import argparse
import sys

from bung_cover_robot.ethercat import cia402
from bung_cover_robot.ethercat.master import RX_SIZE, TX_SIZE


# --- object dictionary entries we probe (index, sub, label) ----------------- #
_STATE_OBJECTS = [
    (0x1000, 0, "Device type"),
    (0x6041, 0, "Statusword"),
    (0x6061, 0, "Modes of operation (display)"),
    (0x6060, 0, "Modes of operation (set)"),
    (0x6064, 0, "Position actual value"),
    (0x6077, 0, "Torque actual value"),
    (0x603F, 0, "Error code"),
    (0x6098, 0, "Homing method"),
    (0x6091, 1, "Gear ratio: motor revolutions"),
    (0x6091, 2, "Gear ratio: shaft revolutions"),
    (0x608F, 1, "Encoder increments"),
]

# Human names for objects that commonly appear in a servo PDO map.
_OD_NAMES = {
    0x6040: "Controlword",
    0x6041: "Statusword",
    0x6060: "Modes of operation",
    0x6061: "Modes of operation display",
    0x606C: "Velocity actual value",
    0x6064: "Position actual value",
    0x607A: "Target position",
    0x60FF: "Target velocity",
    0x6071: "Target torque",
    0x6077: "Torque actual value",
    0x60B1: "Velocity offset",
    0x60B2: "Torque offset",
    0x60FD: "Digital inputs",
    0x60FE: "Digital outputs",
    0x603F: "Error code",
    0x60F4: "Following error actual value",
}


def _read_int(slave, index, sub, signed=False):
    """SDO-read an object and return it as an int (or None if it isn't there)."""
    try:
        data = slave.sdo_read(index, sub)
    except Exception as exc:  # noqa: BLE001 - object may simply not exist
        return None, str(exc)
    return int.from_bytes(data, "little", signed=signed), None


def _dump_state(slave) -> None:
    print("  Live objects (SDO reads):")
    for index, sub, label in _STATE_OBJECTS:
        val, err = _read_int(slave, index, sub)
        if val is None:
            print(f"    0x{index:04X}:{sub}  {label:<32}  --  (not present)")
            continue
        extra = ""
        if index == 0x6041:
            extra = f"  -> {cia402.decode_state(val).value}"
        if index == 0x603F and val:
            extra = "  <-- NONZERO ERROR CODE"
        print(f"    0x{index:04X}:{sub}  {label:<32}  {val}  (0x{val & 0xFFFFFFFF:X}){extra}")


def _dump_pdo(slave, assign_index: int, direction: str, expect_bytes: int) -> None:
    """Dump one PDO-assign object (0x1C12 Rx / 0x1C13 Tx) and its mapped entries."""
    count, err = _read_int(slave, assign_index, 0)
    if count is None:
        print(f"  {direction} assign 0x{assign_index:04X}: unreadable ({err})")
        return
    print(f"  {direction} assignment (0x{assign_index:04X}): {count} PDO(s)")
    total_bits = 0
    for i in range(1, count + 1):
        pdo_idx, _ = _read_int(slave, assign_index, i)
        if pdo_idx is None:
            continue
        n_entries, _ = _read_int(slave, pdo_idx, 0)
        print(f"    PDO 0x{pdo_idx:04X}: {n_entries} entr{'y' if n_entries == 1 else 'ies'}")
        for e in range(1, (n_entries or 0) + 1):
            entry, _ = _read_int(slave, pdo_idx, e)
            if entry is None:
                continue
            obj = (entry >> 16) & 0xFFFF
            osub = (entry >> 8) & 0xFF
            bits = entry & 0xFF
            total_bits += bits
            name = _OD_NAMES.get(obj, "?")
            print(f"      0x{obj:04X}:{osub}  {bits:>2} bits  {name}")
    total_bytes = total_bits // 8
    flag = "OK" if total_bytes == expect_bytes else "!! MISMATCH"
    print(f"    => {total_bits} bits = {total_bytes} bytes "
          f"(master.py expects {expect_bytes})  [{flag}]")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ifname", default="ecat0", help="EtherCAT interface (default ecat0)")
    args = ap.parse_args()

    try:
        import pysoem
    except ImportError:
        print("pysoem not installed — `pip install pysoem` in the venv.", file=sys.stderr)
        return 2

    master = pysoem.Master()
    master.open(args.ifname)
    try:
        found = master.config_init()
        print(f"Interface {args.ifname!r}: {found} slave(s) found\n")
        if found <= 0:
            print("No slaves — check cabling direction (ecat0 -> Drive IN) and control power.")
            return 1
        for i, s in enumerate(master.slaves):
            print(f"[{i}] {s.name}   man=0x{s.man:08X}  id=0x{s.id:08X}  rev=0x{s.rev:08X}")
            _dump_state(s)
            _dump_pdo(s, 0x1C12, "RxPDO (PC->drive)", RX_SIZE)
            _dump_pdo(s, 0x1C13, "TxPDO (drive->PC)", TX_SIZE)
            print()
        print("Read-only inspection complete. No drive was enabled; no motion commanded.")
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
