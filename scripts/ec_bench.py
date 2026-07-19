#!/usr/bin/env python3
"""Headless single-drive bench console — no Qt, no GUI, no GIL contention.

Brings one drive to OP via the real PysoemMaster (full DC/SYNC0 sequence) and
prints its CiA 402 state / position / error once a second. The point is to run
the RT master *without* the Qt GUI sharing the process, to tell whether a
persistent sync fault (0x8700 / Er741) comes from GUI/GIL timing jitter or from
the DC configuration itself.

    sudo .venv/bin/python scripts/ec_bench.py --ifname ecat0 --seconds 20
    sudo .venv/bin/python scripts/ec_bench.py --cycle 0.004      # try a slower cycle

No motion is commanded — the drive stays in switch-on-disabled the whole time.
"""

from __future__ import annotations

import argparse
import logging
import time

from bung_cover_robot.ethercat import cia402
from bung_cover_robot.ethercat.master import MasterError, PysoemMaster


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ifname", default="ecat0")
    ap.add_argument("--seconds", type=int, default=20)
    ap.add_argument("--cycle", type=float, default=0.002, help="cycle time s (try 0.004/0.008)")
    ap.add_argument("--no-dc", action="store_true", help="disable DC (expected to fault on a DC-only drive)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    m = PysoemMaster(ifname=args.ifname, num_drives=1,
                     mode=cia402.MODE_PROFILE_POSITION,
                     cycle_dt_s=args.cycle, use_dc=not args.no_dc)
    try:
        m.open()
    except MasterError as exc:
        print(f"open failed: {exc}")
        return 1

    print(f"\nOP reached on {args.ifname}. Holding {args.seconds}s (no motion). "
          f"Turn the shaft by hand to watch position track.\n")
    faulted_at = None
    try:
        for t in range(args.seconds):
            time.sleep(1.0)
            d = m.drives[0]
            st = cia402.decode_state(d.statusword)
            fault = cia402.is_fault(d.statusword)
            print(f"t={t:2d}s  sw=0x{d.statusword:04X} {st.value:<20} "
                  f"err=0x{d.error_code:04X}  pos={d.actual_position:>10d}  "
                  f"follerr={d.following_error}  wkc_bad={m.fault_code()}")
            if fault and faulted_at is None:
                faulted_at = t
    finally:
        m.close()

    if faulted_at is None:
        print("\nRESULT: held OP with NO fault for the whole run. "
              "The DC/SYNC0 config is good headless — a GUI-process fault is "
              "GIL/timing jitter, so run the master out-of-process.")
        return 0
    print(f"\nRESULT: faulted at t={faulted_at}s even headless. "
          "Not a GUI problem — DC timing/config needs more work "
          "(try --cycle 0.004, or tune the SYNC0 shift).")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
