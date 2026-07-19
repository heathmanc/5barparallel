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
| Shoulder bearings | 4× **7005 angular-contact** (25×47×12), back-to-back pair per shaft, light preload. Each race sits **flush** in its 12-thick plate (plate = race width) and is trapped by a **flat printed Ø72 cap on the top AND bottom face**, bolted together on a Ø58 BCD (4× M4×30 + nyloc, ≥1.5d rim); each cap's 0.3 mm proud pad presses the outer race only, so the disc seats on the plate with a defined clamp crush. Shaft preload via a locknut above the upper cap. |
| Elbow bearings | 688-2RS (8×16×5) in the proximal elbow pockets; Ø8 × ~75 mm pins with a **bottom head + printed top clip** so no link can walk off (the flipped lower distal included) |
| TCP joint | Hollow spindle Ø20 OD / Ø16 bore in **2× 6804-2RS (20×32×7)** — one in each distal's outer face for max span. The spindle's **bottom flange + top collar** capture both stacked distals. A miniature air cylinder (default ISO 6432 Ø10, Ø15 barrel — set `CYL_BARREL_OD`) drops through the spindle; the cup sits ON the joint axis, immune to the platform's free spin. |
| Drive | HTD-5M 3:1 — 24T (Ø19 bore, on the motor) → 72T (Ø25 bore); belt **450-5M-15**, C = 97.5 mm. 24T (up from 20T) leaves ~6 mm hub wall over the motor-shaft keyway (asserted ≥ 3.5). |
| Belt planes | left z5–20, right z25–40 (72T pulleys stagger to pass at 80 mm spacing) |
| Motors | 2× StepperOnline A6M80-750 under the deck, shafts up, **behind the shoulders** (belts run rearward). Splayed out by 16° from −Y so the two 80-frame bodies clear each other — approximately-behind, not exactly on each shaft axis. |
| Motor mounts | **Jackscrew slider on a cradle:** each motor bolts (from below) to a carriage that slides on a fixed window-frame **cradle** directly beneath it. The cradle's two **solid shear-fin walls** (9.5 thick, guide face to frame edge) rise to the deck underside and bolt to it (3× M5 per fin, full-depth heat-sets); four M5 lock bolts pass through ±6 mm slots in the carriage into tapped bosses in the cradle frame **right below** the slots. An M6 jackscrew through the cradle's integral block pushes the carriage away from the shoulder to tension. Fins/bolts/block stay outside the belt corridor and **inside the deck outline** (all asserted in `params.check_layout()`). Machine-shop sheets: `cad/drawings/bcr_drawing_set.pdf`. |
| Plates | bottom **deck** 294 × 240 × 12 (fits a 300×300 bed; covers every underside part; only deliberate holes — full coordinate table on drawing BCR-04), top plate 156 × 92 × 12 (12 thick = race width; standoffs moved to ±64, ±36 to clear the Ø72 caps) |

## Placeholders to resolve at build time

- Motor flange modeled as the standard 80-frame pattern (Ø70 pilot / Ø90 BCD /
  4×Ø6.6, Ø19×38 shaft, body length approximate) — **verify against the boxed
  A6M80 datasheet before drilling**.
- Pulleys and belts are dimensionally-correct blanks — swap vendor STEPs.
- Bearings are plain rings (no rolling elements); 7005 pairs need back-to-back
  orientation and preload via shim + locknut.
- The carriage + cradle are dimensioned on drawing sheets BCR-02/03. Confirm
  the carriage bolt pattern against the boxed motor before fitting inserts.
- The bearing caps trap the outer race (flush race, 0.3 mm pad crush); set the
  running preload with a ground shim + the shaft locknut.
- The deck is a single 294×240 piece that fits a 300×300 bed; if your bed is
  smaller, split it on the mid-line with a bolted lap.
