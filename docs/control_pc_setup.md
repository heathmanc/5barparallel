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
| EtherCAT NIC | Intel **I219-V**, PCI `00:1f.6`, driver `e1000e`, pinned to iface **`ecat0`** |
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

Find the interface and confirm the driver. **Don't trust a predicted name** — the
kernel may use the onboard-index form (`eno2`) or the PCI-path form
(`enp0s31f6`), and it can flip across BIOS/kernel updates. Ask the tools:

```
lspci | grep -i ethernet                   # Intel I219-V at 00:1f.6
ip -br link                                # the real name (here it was eno2)
sudo .venv/bin/python -c "import pysoem; [print(a.name) for a in pysoem.find_adapters()]"
ethtool -i eno2                            # driver: e1000e, bus-info 0000:00:1f.6
```

**Pin it to a stable name** so `ethercat_ifname` can never drift. A systemd
`.link` file renames the port by MAC to `ecat0` for good:

```
cat /sys/class/net/eno2/address            # the port's MAC, e.g. 04:92:26:bd:5f:fe
sudo tee /etc/systemd/network/10-ecat.link >/dev/null <<'EOF'
[Match]
MACAddress=04:92:26:bd:5f:fe

[Link]
Name=ecat0
EOF
sudo update-initramfs -u
sudo reboot
# after reboot:
ip -br link                                # now: ecat0 ...
ethtool -i ecat0                           # driver: e1000e, 00:1f.6
```

`ecat0` is the `ifname` for `PysoemMaster` and the value for the Drives tab
`ethercat_ifname` field. It self-documents the port's job and survives renaming.
Bring it **up with no IP** (EtherCAT doesn't use IP):

```
sudo ip link set ecat0 up
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

**BIOS settings** (K-series board exposes them all). These are the biggest lever
for the latency *tail* — kernel flags can't substitute, because the spikes come
from firmware the OS can't see:

- Disable **C-states** (or cap at C1) — deep-sleep wakeup is the classic tail spike.
- Disable **Turbo Boost** — its frequency transitions add jitter; determinism
  beats a few % peak clock here.
- Disable **SpeedStep / EIST** and set a fixed core frequency.
- Leave hyperthreading alone — the 9700K has none.

> **Measured on the reference machine** (isolated-core soak, stepping the BIOS):
> **Turbo on + C-states auto → ~84 µs**; **Turbo off + C-states off → ~47 µs**;
> **+ SpeedStep off → ~32 µs**. Each setting tightened the tail — it was
> firmware, exactly what these fix. Do the full BIOS pass; don't chase it in
> software.

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
sudo cyclictest -m -p 90 -i 200 -d 0 -a 6 -t 1 -D 30m
```

> Do **not** combine `-S` (SMP mode) with `-a`/`-t` — `-S` forces one thread per
> CPU and *ignores* the affinity flags ("ignored due to smp mode"). Drop `-S` to
> pin a single thread to one core.

Load cores 0–5 *while it runs* — that's the realistic worst case:

```
stress-ng --cpu 6 --io 2 --timeout 30m       # or run the vision stack + a build
```

(Heavy `--vm` memory load is pessimistic: it contends on the shared memory
controller/L3, which isolation does **not** cover, so it overstates the real
vision workload. `--cpu` load or the actual vision stack is the honest test.)

If you want to confirm whether a residual spike is firmware, `hwlatdetect
--duration=5m` measures SMI/hardware latency directly — what cyclictest attributes
but can't fix.

**Acceptance:** worst-case **Max** must sit well under the CSP cycle time
(`cycle_dt_s` = **2 ms** by default). Comfortable = Max under ~100 µs.

**Reference machine result:** ~28–53 µs unisolated (30 s windows) → **84 µs**
once soaked with Turbo/C-states still on → **47 µs** after Turbo + C-states off →
**32 µs** after SpeedStep off too, on isolated core 6. That final **~1.6% of a
2 ms cycle** is the sign-off. A longer soak always finds a higher Max than a short
one, so judge on the long isolated run under load, not a 30 s window. With this
much headroom the platform is not the limiting factor; the cycle could be
shortened (1 ms / 500 µs) if ever needed.

## 5. pysoem

```
pip install pysoem                         # in the project venv
```

Keep the whole Python stack (pysoem, PySide6, OpenCV, PyYAML) in a **venv** so an
`apt` upgrade never disturbs the robot's interpreter.

pysoem opens a **raw L2 socket** and the RT thread needs scheduling privilege, so
the master needs two capabilities:

- **`CAP_NET_RAW`** — to open the EtherCAT socket on `ecat0`.
- **`CAP_SYS_NICE`** — so `SCHED_FIFO` + `mlockall` take (`set_realtime()` logs a
  warning if they don't).

Quickest for the bench is to run under sudo with the venv's interpreter
(`sudo .venv/bin/python …`) — plain `sudo python` uses system Python and won't
find pysoem. To run the HMI as your normal user without sudo, grant the caps to a
**copied** venv interpreter (`python -m venv --copies .venv`, then
`sudo setcap cap_net_raw,cap_sys_nice+eip .venv/bin/python3`) so the caps scope to
the venv, not the shared system interpreter.

Empty-bus sanity check (returns 0 slaves, no exception = the NIC/socket are good):

```
sudo ip link set ecat0 up
sudo .venv/bin/python -c "import pysoem; m=pysoem.Master(); m.open('ecat0'); print(m.config_init()); m.close()"
```

---

Continue with the fieldbus/drive checklist in
[`ethercat_bringup.md`](ethercat_bringup.md) (§2 drive parameters onward).
