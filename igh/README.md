# IgH EtherCAT master — DC go/no-go test

Purpose: prove whether the **IgH EtherLab** master can bring the ANCTL AS715N
(StepperOnline A6-EC) to OP with **distributed clocks** and no `Er74.1`/`0x8700`
"No sync signal" — the fault pysoem can't get past. This is a *test only*; the
full `IgHMaster` backend comes after it passes.

`igh_test.c` is the direct analog of `scripts/ec_dc_probe.py`, in IgH's C API.

---

## 1. Install the IgH master (once)

Use the **maintained** fork — the old etherlab.org 1.5.2 won't build on a 6.12
kernel. Build with the **generic** driver so it rides the existing `e1000e` NIC
(no special EtherCAT NIC driver needed).

```bash
sudo apt install -y build-essential autoconf libtool pkg-config \
     linux-headers-$(uname -r) git

cd ~
git clone https://gitlab.com/etherlab.org/ethercat.git igh-ethercat
cd igh-ethercat
./bootstrap
./configure --prefix=/opt/etherlab \
            --enable-generic --enable-userlib \
            --disable-8139too --disable-eoe
make -j"$(nproc)"
make modules
sudo make install
sudo make modules_install
sudo depmod
```

Point the master at the EtherCAT NIC (the MAC we pinned to `ecat0`) with the
generic driver. **The config lives under the install prefix**, i.e.
`/opt/etherlab/etc/sysconfig/ethercat` (NOT `/etc/sysconfig`):

```bash
sudo tee /opt/etherlab/etc/sysconfig/ethercat >/dev/null <<'EOF'
MASTER0_DEVICE="04:92:26:bd:5f:fe"
DEVICE_MODULES="generic"
EOF
```

The `ethercat` tool is at `/opt/etherlab/bin` — add it to PATH. Take the EtherCAT
port off the IP stack (IgH's generic driver talks to the MAC directly), then start
the master and confirm the drive enumerates:

```bash
export PATH=$PATH:/opt/etherlab/bin
sudo ip link set ecat0 down
sudo /etc/init.d/ethercat start    # (install put the script at /etc/init.d/ethercat)
sudo ethercat master               # master "up", link "UP"
sudo ethercat slaves               # <-- must list the AS715N (0x00400000 / 0x0715)
```

> **`ethercat slaves` showing the drive is milestone #1.** If it doesn't appear:
> the MAC is wrong, the interface is down, or the generic module didn't bind —
> `dmesg | grep -i ethercat` will say which.

## 2. Build + run the DC test

```bash
make -C ~/5barparallel/igh ETHERLAB=/opt/etherlab
sudo ~/5barparallel/igh/igh_test
```

It reaches OP with DC and prints once a second for ~12 s:

```
t= 0s  status=0x0250  err=0x0000  pos=...  al_states=0x08  cycles=...
```

**Interpretation (milestone #2 — the real go/no-go):**

- **`err=0x0000`, `status` = switch-on-disabled (`0x0250`), `al_states=0x08` (OP)
  held for the whole run** → IgH drives this drive's DC correctly. **Path proven**
  — we build the `IgHMaster` backend behind the `EtherCatMaster` interface.
- **`err=0x8700`** → the sync fault persists even on IgH. Then it's not a master
  problem and we rethink (drive/ESI, or hardware).

## Knobs if it doesn't reach OP / DC won't lock

- **`ASSIGN_ACT` (0x0300)** in `igh_test.c` is the DC *AssignActivate* word. It's
  from the drive's **ESI XML** (`<Dc><OpMode><AssignActivate>`). 0x0300 is the
  common CoE value; if DC won't activate, get the real value from the A6-EC ESI
  file and set it here.
- **`SYNC0_SHIFT`** — try `CYCLE_NS/2` if the drive wants margin.
- **`CYCLE_NS`** — 2 ms is safe; the drive's SYNC0 range is mode-dependent.
- If `ethercat slaves` shows the drive but `slave_config` fails, double-check
  `VENDOR`/`PRODUCT` against `ethercat slaves -v`.

---

Report back the per-second lines (especially `err=` and `al_states=`) and whether
`ethercat slaves` listed the AS715N. That tells us go or no-go on IgH.
