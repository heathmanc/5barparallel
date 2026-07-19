#!/usr/bin/env python3
"""Minimal RAW-pysoem DC/SYNC0 probe — isolates pysoem from our master.

The drive throws Er74.1 "No sync signal" (0x8700): it sees no SYNC0 pulse. This
strips away all of PysoemMaster's machinery and runs the barest canonical pysoem
DC sequence, so we can tell whether raw pysoem can make SYNC0 fire on this drive
at all — and it dumps pysoem's real DC/sync API surface so we're not guessing at
method names/signatures.

    sudo .venv/bin/python scripts/ec_dc_probe.py --ifname ecat0

No motion: the drive is never enabled (controlword stays 0).
"""

from __future__ import annotations

import argparse
import time


def rd(slave, index, sub=0):
    try:
        return int.from_bytes(slave.sdo_read(index, sub), "little")
    except Exception as exc:  # noqa: BLE001
        return f"<err {exc}>"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ifname", default="ecat0")
    ap.add_argument("--cycle-ns", type=int, default=2_000_000)
    ap.add_argument("--shift-ns", type=int, default=0)
    ap.add_argument("--mode", type=int, default=8, help="6060 mode (8=CSP, 1=PP)")
    ap.add_argument("--seconds", type=int, default=8)
    args = ap.parse_args()

    import pysoem
    print("pysoem", pysoem.__version__)
    m = pysoem.Master()
    print("master DC/sync API:", [a for a in dir(m) if "dc" in a.lower() or "sync" in a.lower()])

    m.open(args.ifname)
    n = m.config_init()
    print("slaves:", n)
    if n < 1:
        return 1
    s = m.slaves[0]
    print("slave  DC/sync API:", [a for a in dir(s) if "dc" in a.lower() or "sync" in a.lower()])

    m.config_map()
    print("config_dc() ->", m.config_dc())

    # mode of operation via SDO (not cyclic in this drive's map)
    try:
        s.sdo_write(0x6060, 0, bytes([args.mode & 0xFF]))
    except Exception as exc:  # noqa: BLE001
        print("6060 write:", exc)

    m.state_check(pysoem.SAFEOP_STATE, 50_000)

    # A few frames so the DC system time is distributed before arming SYNC0.
    for _ in range(50):
        m.send_processdata()
        m.receive_processdata(2000)
        time.sleep(0.002)

    print(f"arming SYNC0: cycle={args.cycle_ns} shift={args.shift_ns}")
    s.dc_sync(True, args.cycle_ns, args.shift_ns)

    # Request OP, pump.
    m.state = pysoem.OP_STATE
    m.write_state()
    reached = False
    for _ in range(500):
        m.send_processdata()
        m.receive_processdata(2000)
        if m.state_check(pysoem.OP_STATE, 2000) == pysoem.OP_STATE:
            reached = True
            break
        time.sleep(0.002)
    print("OP reached:", reached, "state=0x%02X" % s.state)

    # Hold, keep frames flowing, watch the fault code.
    for t in range(args.seconds):
        for _ in range(500):
            m.send_processdata()
            m.receive_processdata(2000)
            time.sleep(0.002)
        print(f"t={t}s  0x603F={rd(s,0x603F):#06x}  0x6041={rd(s,0x6041):#06x}  state=0x%02X" % s.state)

    try:
        s.dc_sync(False, 0)
    except Exception:  # noqa: BLE001
        pass
    m.state = pysoem.INIT_STATE
    m.write_state()
    m.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
