#!/usr/bin/env python3
"""EtherCAT physical-layer error-counter dump — is the cable / EMI the problem?

Reads each slave's ESC (EtherCAT Slave Controller) error counters via the IgH
``ethercat`` CLI. It works **while the RT daemon is running its cyclic traffic**
(the CLI multiplexes low-priority register datagrams onto the running master),
so you can watch the counters live during a speed trial.

These counters are the definitive tell for a bad cable / connector / EMI: a
real drive fault latches and shows on the drive; a marginal physical layer
instead makes these counters *climb under load*, and enough lost/corrupt
frames kick a drive from OP down to SAFEOP — which the CiA 402 side sees as a
silent drop to SWITCH ON DISABLED with **no fault**. If any of these increment
while you run, fix the cabling before chasing anything else.

    # the IgH `ethercat` CLI must be installed and /dev/EtherCAT0 up (daemon running)
    python scripts/ec_crc.py                 # one snapshot, all slaves
    python scripts/ec_crc.py --reset         # zero the counters (clean baseline)
    python scripts/ec_crc.py --watch 1       # live: run a trial, watch the deltas
    python scripts/ec_crc.py --reset --watch 1   # zero, then watch from clean

Per-port ESC counters (ET1100/AX58100 register map):
  RX error      0x0300+2p bit[15:8]  PHY-level RX errors (bad signal integrity)
  invalid frame 0x0300+2p bit[7:0]   a frame with a bad CRC arrived at this port
  forwarded RX  0x0308+p            error arrived already flagged from upstream
  lost link     0x0310+p            the link physically dropped (intermittent plug)
  ECAT PU err   0x030C              processing-unit datagram errors
  PDI err       0x030D              process-data-interface errors

RX error / invalid-frame climbing on a port points at the cable segment feeding
THAT port; lost-link points at an intermittent connector. Forwarded errors mean
the fault is upstream of this slave (nearer the PC / on the previous segment).

Read-only unless ``--reset`` (which only zeros the counters). Never touches a
drive's state, never commands motion.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from typing import Dict, List, Optional

# The IgH EtherLab `ethercat` CLI. It rarely sits on a default PATH (it installs
# under the EtherLab prefix), and `sudo` strips PATH via secure_path — so we
# auto-discover it and accept an explicit path / env var. Resolved in main().
ETHERCAT = "ethercat"
_ETHERCAT_CANDIDATES = (
    "/opt/etherlab/bin/ethercat",       # this project's build prefix
    "/usr/local/etherlab/bin/ethercat",
    "/usr/local/bin/ethercat",
    "/usr/bin/ethercat",
)


def resolve_ethercat(explicit: Optional[str] = None) -> Optional[str]:
    """Find the `ethercat` CLI: an explicit path/name, then $ETHERCAT_BIN, then
    PATH, then the usual EtherLab install locations. Returns None if not found."""
    for cand in (explicit, os.environ.get("ETHERCAT_BIN")):
        if cand and (shutil.which(cand) or os.path.isfile(cand)):
            return cand
    onpath = shutil.which("ethercat")
    if onpath:
        return onpath
    for path in _ETHERCAT_CANDIDATES:
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    return None

# ESC error-counter registers.
_RX_ERR_BASE = 0x0300       # +2*port, uint16: [15:8] rx error, [7:0] invalid frame
_FWD_ERR_BASE = 0x0308      # +port, uint8
_PU_ERR = 0x030C            # uint8
_PDI_ERR = 0x030D           # uint8
_LOST_LINK_BASE = 0x0310    # +port, uint8


# --------------------------------------------------------------------------- #
# Pure helpers (unit-tested; no hardware / subprocess)
# --------------------------------------------------------------------------- #
def split_rx(raw: int) -> Dict[str, int]:
    """0x0300+2p is uint16: high byte = RX error, low byte = invalid frame."""
    return {"rx_error": (raw >> 8) & 0xFF, "invalid_frame": raw & 0xFF}


def total_errors(counters: dict) -> int:
    """Sum every physical-layer error on a slave (0 == a clean link)."""
    t = counters.get("pu_error", 0) + counters.get("pdi_error", 0)
    for p in counters.get("ports", []):
        t += p["rx_error"] + p["invalid_frame"] + p["forwarded"] + p["lost_link"]
    return t


def diff_counters(now: dict, base: Optional[dict]) -> dict:
    """now - base, field by field (base None -> now unchanged)."""
    if base is None:
        return now
    out = {"pu_error": now["pu_error"] - base["pu_error"],
           "pdi_error": now["pdi_error"] - base["pdi_error"], "ports": []}
    for pn, pb in zip(now["ports"], base["ports"]):
        out["ports"].append({k: pn[k] - pb[k] for k in pn})
    return out


def render(snaps: List[dict], base: Optional[List[dict]] = None) -> str:
    """A compact per-slave / per-port table. If ``base`` is given, values are
    the DELTA since the baseline (what climbed during the run)."""
    hdr = ("slave port   rx_err  inv_frm  fwd_err  lost_lk        "
           "  pu_err  pdi_err   TOTAL")
    lines = [hdr, "-" * len(hdr)]
    grand = 0
    for i, snap in enumerate(snaps):
        b = base[i] if base else None
        shown = diff_counters(snap, b)
        tot = total_errors(shown)
        grand += tot
        flag = "  <-- ERRORS" if tot else ""
        for p, port in enumerate(shown["ports"]):
            head = f"{i:>5} {p:>4}" if p == 0 else f"{'':>5} {p:>4}"
            lines.append(
                f"{head} {port['rx_error']:>8} {port['invalid_frame']:>8} "
                f"{port['forwarded']:>8} {port['lost_link']:>8}"
                + (f"        {shown['pu_error']:>7} {shown['pdi_error']:>8} "
                   f"{tot:>7}{flag}" if p == 0 else ""))
    lines.append("-" * len(hdr))
    verdict = ("CLEAN — no physical-layer errors"
               if grand == 0 else
               f"{grand} error(s) — physical layer (cable / connector / EMI)")
    lines.append(("delta since baseline: " if base else "cumulative: ") + verdict)
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# Bus I/O via the `ethercat` CLI
# --------------------------------------------------------------------------- #
def _run(args: List[str]) -> str:
    out = subprocess.run([ETHERCAT, *args], capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError((out.stderr or out.stdout).strip()
                           or f"`ethercat {' '.join(args)}` failed")
    return out.stdout.strip()


def _reg_read(slave: int, addr: int, typ: str) -> int:
    return int(_run(["reg_read", "--position", str(slave),
                     "--type", typ, hex(addr)]).split()[-1], 0)


def _reg_write(slave: int, addr: int, typ: str, value: int) -> None:
    _run(["reg_write", "--position", str(slave), "--type", typ,
          hex(addr), hex(value)])


def slave_count() -> int:
    n = 0
    for line in _run(["slaves"]).splitlines():
        tok = line.split()
        if tok and tok[0].isdigit():
            n += 1
    return n


def read_counters(slave: int, nports: int = 2) -> dict:
    ports = []
    for p in range(nports):
        rx = split_rx(_reg_read(slave, _RX_ERR_BASE + 2 * p, "uint16"))
        rx["forwarded"] = _reg_read(slave, _FWD_ERR_BASE + p, "uint8")
        rx["lost_link"] = _reg_read(slave, _LOST_LINK_BASE + p, "uint8")
        ports.append(rx)
    return {"ports": ports,
            "pu_error": _reg_read(slave, _PU_ERR, "uint8"),
            "pdi_error": _reg_read(slave, _PDI_ERR, "uint8")}


def reset_counters(slave: int, nports: int = 2) -> None:
    for p in range(nports):
        _reg_write(slave, _RX_ERR_BASE + 2 * p, "uint16", 0)
        _reg_write(slave, _FWD_ERR_BASE + p, "uint8", 0)
        _reg_write(slave, _LOST_LINK_BASE + p, "uint8", 0)
    _reg_write(slave, _PU_ERR, "uint8", 0)
    _reg_write(slave, _PDI_ERR, "uint8", 0)


# --------------------------------------------------------------------------- #
def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", type=float, metavar="SEC", default=None,
                    help="poll every SEC seconds, showing the delta since start")
    ap.add_argument("--reset", action="store_true",
                    help="zero the counters first (clean baseline)")
    ap.add_argument("--ports", type=int, default=2,
                    help="ports to read per slave (default 2: IN + OUT)")
    ap.add_argument("--slaves", type=int, default=None,
                    help="override slave count (default: auto from `ethercat slaves`)")
    ap.add_argument("--ethercat", metavar="PATH", default=None,
                    help="path to the IgH `ethercat` CLI (default: auto-discover)")
    args = ap.parse_args(argv)

    global ETHERCAT
    tool = resolve_ethercat(args.ethercat)
    if tool is None:
        print(
            "error: the IgH `ethercat` CLI was not found.\n"
            "  looked on PATH and at: " + ", ".join(_ETHERCAT_CANDIDATES) + "\n"
            "It ships with IgH EtherLab (this project builds against "
            "/opt/etherlab), so it is usually /opt/etherlab/bin/ethercat.\n"
            "Fix any one of:\n"
            "  * pass it:      python scripts/ec_crc.py --ethercat /opt/etherlab/bin/ethercat\n"
            "  * set the env:  export ETHERCAT_BIN=/opt/etherlab/bin/ethercat\n"
            "  * add to PATH:  export PATH=$PATH:/opt/etherlab/bin\n"
            "Running under sudo? sudo resets PATH — use --ethercat with the full\n"
            "path, or `sudo env \"PATH=$PATH\" python scripts/ec_crc.py ...`.",
            file=sys.stderr)
        return 2
    ETHERCAT = tool

    try:
        n = args.slaves if args.slaves is not None else slave_count()
        if n == 0:
            print("no slaves found — is the daemon running and the bus up?",
                  file=sys.stderr)
            return 2
        if args.reset:
            for s in range(n):
                reset_counters(s, args.ports)
            print(f"zeroed error counters on {n} slave(s).")

        if args.watch is None:
            snaps = [read_counters(s, args.ports) for s in range(n)]
            print(render(snaps))
            return 0

        base = [read_counters(s, args.ports) for s in range(n)]
        print("watching (Ctrl-C to stop) — run your speed trial now...\n")
        while True:
            time.sleep(args.watch)
            snaps = [read_counters(s, args.ports) for s in range(n)]
            print(f"\n=== t+{time.strftime('%H:%M:%S')} ===")
            print(render(snaps, base))
    except KeyboardInterrupt:
        return 0
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
