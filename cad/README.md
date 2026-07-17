# cad/ — parametric mechanical design (CadQuery)

`generate.py` regenerates every STEP model and preview image from source. The
geometry is tied to the software's source of truth: link lengths, base spacing,
and the home pose come from `FiveBarConfig` / `FiveBarKinematics`, and the
link-collision proof sweeps the same `WorkspaceValidator` the runtime uses — so
the mechanical design cannot silently drift from the motion software.

```
pip install cadquery        # only needed to regenerate CAD (not for the runtime)
python cad/generate.py      # -> cad/step/*.step, docs/cad/*.png
```

## Level assignment (the "cross" arrangement)

The distal of each side pairs with the proximal of the *other* side:

| plane | z (mm) | members |
|---|---|---|
| A (lower) | 111–141 | proximal **L** + distal **R** |
| B (upper) | 146–176 | proximal **R** + distal **L** |

Why: the two proximals sit on different planes so they can **never** collide;
the two distals land on adjacent planes so they **stack naturally at the TCP**
(full-height bosses, shared Ø8 pin, one 5 mm spacer — no lap machining). Each
distal shares a plane only with the opposite proximal, and `generate.py`
asserts at build time that those cross pairs keep **> 80 mm** centerline
clearance everywhere the workspace validator allows the TCP to be (beams are
20–22 mm wide; anything above ~24 mm is safe).

Identical parts both sides: the right proximal simply clamps higher on the
shoulder shaft's long D-flat; the distals are the same part.

## Design table

| Item | Spec |
|---|---|
| Links | 6061-T6 pocketed-I from 30 mm plate (or PA12/PA-CF print for bring-up) |
| Proximal (L1=200) | I 30×22, fl 3.5 / web 4.0 — Ø25 D-bore + M5 set screw at shoulder, 688 pockets at elbow. ~214 g Al / 80 g PA12 |
| Distal (L2=230) | I 30×20, fl 3.0 / web 3.5 — Ø8 split clamp at elbow (slit into the bore, 2× M3 pinch bolts), 688 pockets at TCP. ~174 g Al / 65 g PA12 |
| Shoulder shafts | Ø25 h6 × 180, one D-flat z109–178 (serves both clamp heights) |
| Shoulder bearings | 4× **7005 angular-contact** (25×47×12), back-to-back pair per shaft, light preload |
| Elbow / TCP bearings | 688-2RS (8×16×5) in the pockets; Ø8 pins (elbow pins 65 mm) |
| Drive | HTD-5M 3:1 — 20T (Ø19 bore, on the motor) → 60T (Ø25 bore); belt **450-5M-15**, C = 120.8 mm, ±4 mm tension slots |
| Belt planes | left z5–20, right z25–40 (60T pulleys stagger to pass at 80 mm spacing) |
| Motors | 2× StepperOnline A6M80-750 under the base plate, shafts up, splayed outboard |
| Motor mounts | 110×110×8 plates (Ø70 pilot, Ø90 BCD, 4×Ø6.6) on standoffs: **left 44 mm, right 24 mm** |
| Plates | bottom 430×110×12 (bearing bores, shaft clearance, slots), top 210×96×10 |

## Placeholders to resolve at build time

- Motor flange modeled as the standard 80-frame pattern (Ø70 pilot / Ø90 BCD /
  4×Ø6.6, Ø19×35 shaft, body length approximate) — **verify against the boxed
  A6M80 datasheet before drilling**.
- Pulleys and belts are dimensionally-correct blanks — swap vendor STEPs.
- Bearings are plain rings (no rolling elements); 7005 pairs need back-to-back
  orientation and preload via shim + locknut.
- Plate sizes / standoff length between plates set the bearing span — tune to
  the machine frame. Thrust washers at arm plane gaps are not modeled.
