# Home-offset calibration (`HOME_OFFSET_L` / `HOME_OFFSET_R`)

How to measure and set the two home-offset constants so the PLC reports the
**true** shoulder angles after homing. Do this once per machine at
commissioning (and again only if you move a home switch or change the homing
approach). Backed up and restored with the **PLC tab → Commissioning
constants** panel (see `plc_setup.md`).

---

## 1. What the offset is, and why it exists

The ClearLink zeroes each motor's position **at the home-switch (prox) trip
point**, not at a meaningful shoulder angle. So right after homing,
`Motor0/1_CommandedPosn = 0` even though the arm is sitting at some real angle.
The PLC converts commanded steps to a published angle with:

```
ActualLeftDeg  = (Motor0_CommandedPosn + HOME_OFFSET_L) / STEPS_PER_DEG
ActualRightDeg = (Motor1_CommandedPosn + HOME_OFFSET_R) / STEPS_PER_DEG
```

* **Motor0 = left** shoulder, **Motor1 = right** shoulder.
* `STEPS_PER_DEG = 26.66667` (3200 pulses/rev × 3:1 / 360).

`HOME_OFFSET` is therefore **the true shoulder angle at the trip point, in
steps**. If the switch tripped exactly at the design home pose
(L 140.5406° / R 39.4594°) the offsets would be the shipped nominals
**3748 / 1052**. In practice the switch is a few tenths off, so you measure the
real value.

> **Why not a digital level?** This is a horizontal-plane SCARA — the arms sweep
> in a plane parallel to the floor. A gravity inclinometer reads 0° everywhere
> in that plane, so it can't measure the in-plane shoulder angle. The offset is
> calibrated from a **known TCP position** instead (Method 1).

---

## 2. Before you start

Calibrate the offset **last**, after these are done — an offset measured on a
drifting or mis-scaled axis is worthless:

1. **Homing is repeatable.** `HOME_VEL_0/1` sign drives *toward* the prox, the
   back-off/re-approach is tuned, and re-homing several times lands the same
   trip point (watch `M0/M1 CommandedPosn` return to 0 consistently).
2. **`STEPS_PER_DEG` is verified.** Command a 90° shoulder jog and measure the
   real sweep; fix step wiring / microstepping until it matches 26.66667.
3. You have a **locating fixture** that seats the tool at one known robot-frame
   coordinate (a dowel/pin the gripper mates, or a hard stop at a measured
   point).

Open the app: **Diagnostics tab** (shows `M0 CommandedPosn`, `M1
CommandedPosn`, `ActualLeftDeg`, `ActualRightDeg`) and the **PLC tab →
Commissioning constants** panel (to enter/push the values).

---

## 3. Method 1 — locating fixture at a known TCP (recommended)

Each axis is independent, so **one** known TCP position gives you both offsets.

1. **Home** the robot (Robot Test → Home). Confirm `Has Homed` and that both
   `CommandedPosn` read ~0.
2. **Seat the tool** on the fixture at a known robot-frame point `(x, y)` mm.
   Jog there in small steps — you do **not** need correct angles yet; drive
   until the gripper physically mates the pin/stop. Pick a point well inside the
   work envelope (e.g. the fixture near `(0, 250)`).
3. **Read** `M0 CommandedPosn` and `M1 CommandedPosn` from the Diagnostics tab
   at that seated pose. Call them `posn_L`, `posn_R` (steps).
4. **Compute the true angles** for that TCP with the project kinematics:

   ```
   python -c "from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics as K; \
   jt=K().inverse(0.0, 250.0); print(jt.left_deg, jt.right_deg)"
   ```

   Call the results `theta_L`, `theta_R` (deg).
5. **Solve the offsets** (from `ActualDeg = (posn + OFFSET)/STEPS_PER_DEG = theta`):

   ```
   HOME_OFFSET_L = round(theta_L * 26.66667 - posn_L)
   HOME_OFFSET_R = round(theta_R * 26.66667 - posn_R)
   ```

### Worked example

Fixture at `(0, 250)` → `theta_L = 140.5406°`, `theta_R = 39.4594°`, so
`theta*STEPS_PER_DEG` = **3747.75 / 1052.25 steps**. Suppose at the seated pose
the Diagnostics tab shows `posn_L = +42`, `posn_R = −33`:

```
HOME_OFFSET_L = round(3747.75 - 42)   = 3706
HOME_OFFSET_R = round(1052.25 - (-33)) = 1085
```

Check: `ActualLeftDeg = (42 + 3706)/26.66667 = 140.55°`,
`ActualRightDeg = (-33 + 1085)/26.66667 = 39.45°` — both match the fixture
angles. ✔

> **Cross-check (optional):** repeat at a second fixture point and confirm the
> two offset solves agree within a step or two. A large disagreement means the
> homing trip point isn't repeatable (go back to §2.1) or `STEPS_PER_DEG` is off
> (§2.2).

---

## 4. Method 2 — geometric measurement of the home pose (alternative)

If you can't fixture a TCP point but can measure the arm geometry directly:

1. Home the robot (so `CommandedPosn = 0` — you're measuring *at the trip
   point*).
2. With calipers/CMM, measure each **elbow** position relative to its shoulder
   pivot in the robot frame, or the TCP relative to the base, and back out the
   true shoulder angle `theta` from `+X`.
3. Because `CommandedPosn = 0` at home, the offset is simply:

   ```
   HOME_OFFSET_L = round(theta_L_home * 26.66667)
   HOME_OFFSET_R = round(theta_R_home * 26.66667)
   ```

This is less convenient than Method 1 and only as good as the physical
measurement; prefer Method 1 when you have a fixture.

---

## 5. Set, push, and verify

1. **PLC tab → Commissioning constants.** Type the two values into the
   `HOME_OFFSET_L` / `HOME_OFFSET_R` rows.
2. Click **Push to PLC…** (confirm). The panel writes them live.
3. **Verify:** re-home, then jog the tool back onto the fixture and confirm the
   Diagnostics tab now reads `ActualLeftDeg ≈ theta_L`,
   `ActualRightDeg ≈ theta_R`. Move to a second known point and confirm it
   tracks there too.

---

## 6. Back it up (disaster recovery)

Once verified, click **Read from PLC (snapshot)** in the same panel. It saves
the whole set-by-hand set — including your calibrated offsets — to
`config/plc_constants.yaml`. If the controller is ever reloaded or cleared,
**Push to PLC** restores it in one shot. (Studio 5000 does not restore these on
a program download; the snapshot file is your backup. It is git-ignored because
the values are specific to your physical machine.)

---

## 7. When to re-calibrate

Re-run this procedure if you:

* move or replace a home switch / prox sensor,
* change the homing approach direction or `HOME_VEL` sign,
* rebuild an arm (new links, re-set the elbow assembly), or
* change `STEPS_PER_DEG` (microstepping or belt ratio).

A stale offset shows up as the robot reaching poses that are consistently
rotated from the commanded angle, or the two arms disagreeing about where the
TCP is.
