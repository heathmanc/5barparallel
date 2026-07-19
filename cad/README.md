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
| A (lower) | 131–161 | proximal **L** + distal **R** |
| B (upper) | 166–196 | proximal **R** + distal **L** |

(The arm planes were raised from the earlier 111/146 so the shoulder bearing
caps + preload locknut fit under the lower arm — the `generate.py` Z-stack is
now derived bottom-up so nothing collides silently.)

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
| Shoulder shafts | Ø25 h6 × 205, one D-flat from below plane A through the top (serves both clamp heights) |
| Shoulder bearings | 4× **7005 angular-contact** (25×47×12), back-to-back pair per shaft, light preload. Each race is trapped by a **printed cap on the top AND bottom of its plate**, bolted together on a Ø58 BCD (4× M4); shaft preload via a locknut above the upper cap. |
| Elbow bearings | 688-2RS (8×16×5) in the proximal elbow pockets; Ø8 × ~75 mm pins with a **bottom head + printed top clip** so no link can walk off (the flipped lower distal included) |
| TCP joint | Hollow spindle Ø20 OD / Ø16 bore in **2× 6804-2RS (20×32×7)** — one in each distal's outer face for max span. The spindle's **bottom flange + top collar** capture both stacked distals. A miniature air cylinder (default ISO 6432 Ø10, Ø15 barrel — set `CYL_BARREL_OD`) drops through the spindle; the cup sits ON the joint axis, immune to the platform's free spin. |
| Drive | HTD-5M 3:1 — 20T (Ø19 bore, on the motor) → 60T (Ø25 bore); belt **450-5M-15**, C = 120.8 mm |
| Belt planes | left z5–20, right z25–40 (60T pulleys stagger to pass at 80 mm spacing) |
| Motors | 2× StepperOnline A6M80-750 under the deck, shafts up, **behind the shoulders** (belts run rearward). Splayed out by 16° from −Y so the two 80-frame bodies clear each other — approximately-behind, not exactly on each shaft axis. |
| Motor mounts | **Jackscrew slider:** each motor bolts to a carriage that slides on two rails (along the belt axis, taking belt-tension shear) fixed to the deck underside. A jackscrew through a fixed block on the shoulder side pushes the carriage away to tension; lock bolts through ±6 mm slots then clamp. Rails and lock bolts sit **outside the belt corridor** (asserted at build time). |
| Plates | bottom **deck** hexagon ~250 × 255 × 12 (fits a 300×300 print bed; shoulder bearing bores + cap/standoff/rail bolts only — no stray holes), top plate 150 × 92 × 10 |

## Placeholders to resolve at build time

- Motor flange modeled as the standard 80-frame pattern (Ø70 pilot / Ø90 BCD /
  4×Ø6.6, Ø19×38 shaft, body length approximate) — **verify against the boxed
  A6M80 datasheet before drilling**.
- Pulleys and belts are dimensionally-correct blanks — swap vendor STEPs.
- Bearings are plain rings (no rolling elements); 7005 pairs need back-to-back
  orientation and preload via shim + locknut.
- The jackscrew-slider carriage, rails, and tension block are functional
  placeholders (boxes + bores). Confirm the carriage bolt pattern against the
  boxed motor, and pick the jackscrew thread (default M6) + heat-set inserts.
- The bearing caps trap the outer race; set the actual preload with a ground
  shim under the bottom cap + the shaft locknut. Cap register is Ø46.6 into a
  Ø47 bore — tune the print fit.
- The deck is a single ~250×255 piece that fits a 300×300 bed with margin; if
  your bed is smaller, split it on the mid-line (the widen point) with a bolted
  lap — the hexagon outline leaves a flat there.
