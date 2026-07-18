# Control PC setup — Debian + PREEMPT_RT for the EtherCAT master

The reference/commissioning machine for the `pysoem` EtherCAT master. This is the
OS-and-platform half of bring-up; the fieldbus/drive half lives in
[`ethercat_bringup.md`](ethercat_bringup.md).

## Reference machine (validated)

| | |
|---|---|
| CPU | Intel **i7-9700K** — 8 cores / 8 threads, **no hyperthreading** |
| RAM | 16 GB (ample; vision + pysoem + HMI don't approach it) |
| OS | **Debian 13 (Trixie)**, 13.6, **XFCE on X11** |
| Kernel | `6.12.94+deb13-rt-amd64` — mainline **PREEMPT_RT** (`linux-image-rt-amd64`) |
| EtherCAT NIC | Intel **I219-V**, PCI `00:1f.6`, iface **`enp0s31f6`**, driver `e1000e` |
| LAN / dev | onboard **WiFi** (EtherCAT and IP traffic stay on separate interfaces) |

The **no-HT** part is a real advantage: every core is standalone, so an isolated
RT core has no SMT sibling contending for execution units — nothing to disable,
cleaner isolation than an HT part.

Why this NIC split: EtherCAT is raw L2 (no IP). The Intel port carries **only**
the fieldbus; all IP traffic (apt, git, SSH, browser) rides WiFi. A single Intel
I219-V on `e1000e` drives both A6 servos fine. An add-in **Intel i210** PCIe card
is optional — add it only if a soak test says you want the "known-good" fieldbus
NIC; then EtherCAT moves to the i210 and the I219-V becomes the wired LAN.

## 1. RT kernel

```
sudo apt install linux-image-rt-amd64      # mainline-RT 6.12 on Trixie
# reboot into it, then confirm:
uname -r                                   # ...-rt-amd64
uname -v                                   # contains PREEMPT_RT
```

## 2. NIC — dedicate the Intel port to EtherCAT

Find the interface and confirm the driver:

```
lspci | grep -i ethernet                   # Intel I219-V at 00:1f.6
ip -br link                                # its iface name (Z390: enp0s31f6)
ethtool -i enp0s31f6                        # driver: e1000e
```

`enp0s31f6` is the `ifname` for `PysoemMaster` and the value for the Drives tab
`ethercat_ifname` field. Bring it **up with no IP** (EtherCAT doesn't use IP):

```
sudo ip link set enp0s31f6 up
```

No `dhclient`/static IP on this interface. Get WiFi associated and internet
working **before** dedicating the Intel port, or you'll lose apt/git on the box.
Disable WiFi power-save so the dev link doesn't hiccup on long sessions:

```
sudo iw dev <wlan> set power_save off
```

## 3. Core isolation + power tuning

Reserve cores **6 and 7** for the RT thread (one for the pysoem cyclic thread,
one spare); cores 0–5 run the OS, XFCE, IDE, HMI, and OpenCV vision.

Edit `/etc/default/grub`, append to `GRUB_CMDLINE_LINUX_DEFAULT`:

```
isolcpus=6,7 nohz_full=6,7 rcu_nocbs=6,7 intel_idle.max_cstate=1 processor.max_cstate=1
```

```
sudo update-grub && sudo reboot
cat /proc/cmdline                          # verify the flags took
```

Pin the CPU governor to performance (belt-and-suspenders with the BIOS):

```
sudo apt install linux-cpupower
sudo cpupower frequency-set -g performance
```

**BIOS settings** (K-series board exposes them all):

- Disable **C-states** (or cap at C1) — deep-sleep wakeup is the classic tail spike.
- Disable **SpeedStep / EIST** and set a fixed core frequency.
- Disable **Turbo Boost** — its frequency transitions add jitter; determinism
  beats a few % peak clock here.
- Leave hyperthreading alone — the 9700K has none.

## 4. Validate with cyclictest

```
sudo apt install rt-tests
```

**Baseline (all cores, loaded):**

```
sudo cyclictest -m -S -p 90 -i 200 -d 0 -D 30s
```

**RT-core soak (pin to isolated core 6, run long, under load):**

```
sudo cyclictest -m -S -p 90 -i 200 -d 0 -a 6 -t 1 -D 30m
```

Run a build or the vision stack *while it runs* — that's the realistic worst case.

**Acceptance:** worst-case **Max** must sit well under the CSP cycle time
(`cycle_dt_s` = **2 ms** by default). Comfortable = Max under ~100 µs; this
machine measured **~28 µs Max at loadavg 6.7 with no isolation**, i.e. **1.4% of
a 2 ms cycle** — isolation only tightens the tail from there. If a long soak stays
double-digit µs, the platform is not the limiting factor and you could shorten the
cycle (1 ms / 500 µs) if ever needed.

## 5. pysoem

```
pip install pysoem                         # in the project venv
```

Keep the whole Python stack (pysoem, PySide6, OpenCV, PyYAML) in a **venv** so an
`apt` upgrade never disturbs the robot's interpreter. Grant the master
`CAP_SYS_NICE` (or run as root) so `SCHED_FIFO` + `mlockall` take — `set_realtime()`
logs a warning if they don't.

---

Continue with the fieldbus/drive checklist in
[`ethercat_bringup.md`](ethercat_bringup.md) (§2 drive parameters onward).
