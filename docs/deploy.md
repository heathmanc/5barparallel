# Deploying to a control PC

How to move this project onto a new **control PC** — the Linux machine that
drives the two A6 servos over EtherCAT and runs the HMI. (For a sim/office PC
with no robot, skip everything EtherCAT: `scripts/install.sh --no-daemon`, then
`python -m bung_cover_robot --sim-ec`.)

The move splits into three kinds of thing, and only the first is a real
"packaging" step:

| Piece | How it moves | Notes |
|-------|--------------|-------|
| **Python app** (`src/`) | `git clone` + `scripts/install.sh` | Standard package; a venv + `pip install` per machine. |
| **`config/`** — robot geometry, home, **calibration**, recipes, tuning | **Copy the folder** | Your machine's *state*. Copy it from the old PC (or recalibrate on a different robot). |
| **IgH RT daemon** (`igh/ec_master_daemon`) | **Rebuilt on the target** | A C binary linked against this machine's IgH EtherLab master — never copied. `install.sh` builds it. |
| **IgH EtherLab master + RT kernel + NIC** | **Provisioned once** | OS-level platform build, not something you package. See the references below. |

There is no single shippable bundle, because half of it is compiled against the
target's kernel + EtherCAT master. Don't reach for PyInstaller: it would bloat
with PySide6/OpenCV and still can't bundle the IgH-linked daemon or the master.

---

## 0. Platform prerequisites (one-time, per machine)

Do these first — they're the OS build the app sits on. Both are already
documented; don't duplicate them, follow them:

- **RT kernel, NIC pinned to `ecat0`, CPU isolation** → `docs/ethercat_bringup.md` §1.
- **Install the IgH EtherLab master** (the maintained fork, generic driver) and
  prove the drive enumerates (`ethercat slaves` lists the AS715N) →
  `igh/README.md` §1. **Milestone: `ethercat slaves` shows the drive.** Until
  that works, nothing below will move the robot.

You also need the build toolchain: `sudo apt install -y build-essential python3-venv`.

---

## 1. Get the code + install

```bash
git clone <repo-url> 5barparallel        # or copy the repo over
cd 5barparallel
scripts/install.sh                       # venv + app + build the daemon
#   --etherlab /opt/etherlab   (default)  IgH install prefix
#   --from-lock                           install pinned versions (see §5)
#   --test                                run the suite after installing
```

`install.sh` creates `.venv`, installs the app with the GUI + vision extras
(`pip install -e .[gui,vision]`), builds `igh/ec_master_daemon` against the IgH
master, freezes a `requirements.lock`, and prints the remaining manual steps.
Re-run it after every `git pull` — it rebuilds the daemon, which you **must** do
whenever its shared-memory ABI changes (the app refuses a stale daemon with
`daemon ABI N != M — rebuild it`).

---

## 2. Let the app launch the daemon (passwordless sudo)

The RT daemon must run as root (raw sockets + `mlockall` + SCHED_FIFO). The app
launches it for you with `sudo -n ec_master_daemon …` — the `-n` means it will
**never prompt**, so from the GUI (no TTY) it fails fast unless sudo is
password-free for exactly that binary. Add a drop-in:

```bash
sudo tee /etc/sudoers.d/bcr-ethercat >/dev/null <<EOF
# Let the HMI user start the EtherCAT RT daemon without a password.
$USER ALL=(root) NOPASSWD: $PWD/igh/ec_master_daemon
EOF
sudo chmod 440 /etc/sudoers.d/bcr-ethercat
sudo visudo -c        # validate
```

(If you'd rather not use sudo at all, start the daemon yourself before the app —
`sudo igh/ec_master_daemon --drives 2` — and the app attaches to its shared
memory. The sudo path is just the convenience default.)

---

## 3. Bring your calibration/config across

`config/` holds everything machine-specific — geometry, the home datum, the
per-recipe pixel→robot **calibration** and ROI, detection sliders, camera and
motion tuning. Copy it from the old PC:

```bash
rsync -a old-pc:/path/to/5barparallel/config/ ./config/
```

- **Same robot, new PC** → keep the calibration; it's tied to the arm geometry
  and camera mount, not the PC.
- **Different robot** → recalibrate (Calibration tab) and re-teach home; the
  geometry and encoder datum differ.

Encoder note: this build uses **bounded-linear multi-turn absolute** encoders,
so the home datum survives power-off — but the software still requires a **Set
Home on each program launch** (`is_referenced` starts false). First launch on the
new PC, re-home once. (See `docs/ethercat_bringup.md` on `C00.07` / linear mode.)

---

## 4. First run (do it in this order)

```bash
source .venv/bin/activate
python -m bung_cover_robot --sim-ec                    # 1) no hardware — proves the stack
python -m bung_cover_robot --ethercat --camera basler # 2) real drives + camera
#   --config /path/to/config   if config lives outside the repo
```

Then walk the **first-motion checklist** in `docs/ethercat_bringup.md` §6 (enable
→ Set Home → a slow single move → the demo) before running a production cycle.
Reminder on Stop behavior: the main-screen **Cycle Stop is graceful** (finishes
the current pick, then halts); the Drives-tab bench **demo Stop is a hard stop**.

---

## 5. Reproducible re-installs (the lockfile)

`pyproject.toml` uses `>=` minimums, so a fresh `pip install` could pull newer
PySide6/OpenCV than you validated. `install.sh` writes a `requirements.lock`
(exact `pip freeze`) on install. **Commit it.** To reproduce that exact
environment on the next PC — or to roll back a bad upgrade:

```bash
scripts/install.sh --from-lock
```

Regenerate the lock deliberately (not accidentally) when you *want* to move
versions: run a plain `scripts/install.sh`, test, then commit the new lock.

---

## 6. Optional: start the HMI on boot (kiosk)

For a fixed control station, autostart the HMI as the logged-in operator user
(it needs the graphical session for Qt + a display). Example systemd **user**
unit — `~/.config/systemd/user/bcr-hmi.service`:

```ini
[Unit]
Description=Bung-cover robot HMI
After=graphical-session.target
PartOf=graphical-session.target

[Service]
Type=simple
WorkingDirectory=%h/5barparallel
ExecStart=%h/5barparallel/.venv/bin/python -m bung_cover_robot --ethercat --camera basler
Restart=on-failure

[Install]
WantedBy=graphical-session.target
```

```bash
systemctl --user enable --now bcr-hmi.service
```

Leave the **daemon** to the app's auto-launch (§2) rather than a separate unit —
`IgHMaster` owns its lifecycle (start, ABI check, shutdown) and refuses to run
against a mismatched one.

---

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `IgH daemon exited immediately … sudo: a password is required` | The sudoers drop-in (§2) is missing or the path doesn't match `$PWD/igh/ec_master_daemon`. |
| `daemon ABI N != M — rebuild it` | Stale daemon after a pull. `make -C igh ETHERLAB=/opt/etherlab` (or re-run `install.sh`), then `sudo pkill ec_master_daemon`. |
| `IgH daemon not built` | Run `scripts/install.sh` (or `make -C igh …`). Needs libethercat — see §0. |
| `ethercat slaves` doesn't list the drive | Master/NIC problem, not the app — MAC/interface/generic-driver. `igh/README.md` §1, `dmesg \| grep -i ethercat`. |
| Move ends "did not settle", off by a multiple of 131072 counts | Multi-turn datum offset — set the encoder to **bounded linear** mode. `docs/ethercat_bringup.md` (C00.07). |
| Drive drops to SWITCH ON DISABLED, no fault | Power-stage/E-stop chain blip — `docs/ethercat_bringup.md` §4d. |
