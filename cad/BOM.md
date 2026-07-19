# Mechanical BOM — 5-bar arm hardware

Quantities are **per robot** (two shoulders, two arms of each link). Small
bearings/pins are cheap in 10-packs — buy spares. Links are *starting points*
(commodity parts, many equivalent vendors); confirm price/stock/tolerance before
ordering. Prices omitted deliberately — they drift.

The **motors** (2× StepperOnline A6M80-750, EtherCAT) are specced separately and
are not in this list.

## Bearings

| Qty | Part | Size | Where it goes | Link |
|---|---|---|---|---|
| **4** | **7005** angular-contact | 25×47×12 | Shoulder axles — a back-to-back (DB) **pair per shaft**, lightly preloaded | [VXB 7005B](https://vxb.com/products/7005b-bearing-angular-contact-ball-bearing-25x47x1) · [Amazon](https://www.amazon.com/Bearing-Angular-25x47x12-Bearings-VXB/dp/B002UE8CW2) |
| **4** | **688-2RS** | 8×16×5 | Proximal elbow bosses (2 per elbow) on the Ø8 elbow pins | [Amazon 10-pk](https://www.amazon.com/Bearing-688-2RS-Miniature-Bearings-VXB/dp/B002BBQCMQ) |
| **2** | **6804-2RS** | 20×32×7 | TCP — one in each distal boss, on the hollow spindle | [Amazon (NSK)](https://www.amazon.com/NSK-6804-2RS-Bearing-20x32x7-Shielded/dp/B07WTQ9WMN) · [FastEddy](https://www.fasteddybearings.com/20x32x7-rubber-sealed-bearing-6804-2rs/) |

> **7005 preload:** the pair must mount **back-to-back** with a small axial
> preload (a ground spacer + locknut, or buy a *matched DB set*). A random pair
> of singles gives you no defined preload — that's where the shoulder stiffness
> comes from. For the printed prototype, plain 6005-2RS deep-groove (25×47×12,
> same envelope) is a cheaper stand-in if you don't want to fuss preload yet.

## Shafts & pins

| Qty | Part | Spec | Note | Link |
|---|---|---|---|---|
| **2** | Shoulder shaft | Ø25 h6 × ~205 mm, steel/SS | **Semi-custom:** cut from 25 h6 ground stock, machine the long D-flat + a locknut thread at the top (see `shoulder_shaft.step`) | [Motedis 25 h6](https://www.motedis.com/en/Precision-shaft-25-mm-h6-steel-hardened-and-ground?products_id=6500) · [Ondrives](https://www.ondrivesus.com/precision-ground-stock-saw-cut-ends/metric) |
| **2** | Elbow pin | Ø8 × ~75 mm, hardened, w/ head | Ø8 shoulder screw (head = bottom retention) or a DIN 6325 dowel + a clip both ends | [McMaster 8 mm dowel](https://www.mcmaster.com/products/dowel-pins/diameter~8-mm/) · [Huyett DIN 6325](https://www.huyett.com/dowmh-080-060) |
| **1** | TCP spindle | Ø20 OD / Ø16 bore × ~75 mm, w/ flange | **Custom turned part** — see `dual_base_full.step` (`tcp_spindle`). Start from Ø20 tube/bar | (machine shop / lathe) |
| — | TCP collar | Ø28 clamp collar, Ø20 bore | Off-the-shelf shaft collar retains the spindle | [McMaster shaft collars](https://www.mcmaster.com/products/shaft-collars/) |

## Drive train (HTD-5M, 3:1) — Beltingonline (bepltd) has the whole set

| Qty | Part | Spec | Note | Link |
|---|---|---|---|---|
| **2** | 72T shoulder pulley | 72-5M-15, pilot bore | Bore to Ø25 + keyway to the shoulder shaft | [72-5M-15](https://bepltd.com/products/72-5m-15-htd-pilot-bore-5m-timing-belt-pulley-72-tooth-x-15mm-wide) |
| **2** | 24T motor pulley | 24-5M-15, pilot bore | Bore to Ø19 + keyway to the A6 shaft | [24-5M-15](https://bepltd.com/products/24-5m-15-htd-pilot-bore-5m-timing-belt-pulley-24-tooth-x-15mm-wide) |
| **2** | Belt | 450-5M-15 (90T) | The stock length the layout was solved to (C = 97.5 mm with 24/72) | [450-5M-15](https://bepltd.com/products/450-5m-15-htd-5m-timing-belt-450mm-long-x-15mm-wide) |

> **Motor-pulley keyway:** the drive was moved from 20T/60T to **24T/72T** (same
> 3:1) specifically to give the Ø19 motor-shaft pulley room for the keyway. The
> 24-5M-15's larger pitch dia (38.2 mm) leaves ~6 mm of hub wall over the keyseat
> vs the 20T's ~3 mm — `cad/generate.py` asserts this wall stays ≥ 3.5 mm, so a
> future tooth-count change can't silently reintroduce a paper-thin boss. Still
> confirm the vendor's actual boss OD when ordering the pilot-bore blank.

## Printed structure (see `cad/drawings/bcr_drawing_set.pdf` for shop sheets)

| Qty | Part | STEP / sheet | Note |
|---|---|---|---|
| 2 | Motor carriage | `motor_carriage.step` / BCR-02 | Slides on the cradle; M6 inserts for the motor, 4 lock slots |
| 1+1 | Motor cradle L + R | `motor_cradle_L/R.step` / BCR-03 | RH mirrors LH; frame + 2 shear fins + jack block, one print each |
| 8 | Bearing cap Ø72 | `bearing_cap.step` / BCR-01 | Top + bottom of each 7005 bore, bolted through the plate |
| 1 | Bottom deck | BCR-04 | 294 × 240 × 12 — hole coordinate table on the sheet |
| 1 | Top plate | BCR-05 | 156 × 92 × 10 (+4 Ø12×40 standoffs) — widened so the Ø72 caps sit fully on it |

## Fasteners (McMaster-Carr — stable catalog)

| Qty | Part | Use |
|---|---|---|
| 8 | M6 × 18 SHCS (Ø90 BCD) | Motor flange → carriage, fitted **from below** through the flange into M6 inserts/taps in the carriage (full 8 mm engagement) |
| 2 | M6 × 40 jackscrew + jam nut | Belt tensioner — through the cradle's block, tip pushes the carriage pad |
| 8 | M5 × 16 SHCS + washer | Carriage lock bolts — through the ±6 mm slots into M5 inserts in the cradle frame (4 per side) |
| 12 | M5 × 20 SHCS | Cradle fin flanges → deck underside (3 per fin, deck holes Ø5.2 thru, 10-deep flange with full M5 heat-set) |
| 16 | M4 × 35 SHCS + nyloc + washers | Bearing caps — 4 per bore, top+bottom cap bolted **through** each plate on the Ø58 BCD (2 bores × 2 plates) |
| 2 | M25×1.5 shaft locknut (or KM5 + tab washer) | Shoulder bearing preload, above the upper cap |
| 4 | M5 × 70 SHCS + nyloc | Top plate → standoff → deck through-bolts (one per standoff — the Ø12 standoffs are clearance-bored Ø5.2, no threads) |
| 4 | M3 SHCS + nut | Distal elbow split-clamp pinch bolts (2 per clamp) |
| 2 | M5 set screw (cup point) | Shoulder D-bore axial retention onto the shaft flat |
| 2 | Ø8 shaft clip / printed clip | Elbow-pin top retention (bottom head is integral to the pin) |
| — | M5/M6 brass heat-set inserts | All printed bosses (cradle lock/fin/jack, carriage motor bosses) |
| — | assorted M4/M5/M6 washers | as needed |

[McMaster metric SHCS](https://www.mcmaster.com/products/screws/socket-head-screws~/system-of-measurement~metric/) · [set screws](https://www.mcmaster.com/products/set-screws/system-of-measurement~metric/)

## Quick shopping strategy

- **One order covers the drive:** all pulleys + belts from Beltingonline.
- **One order covers small bearings/pins/fasteners:** McMaster (688s, dowels,
  all screws, shaft collar) — fastest single-source.
- **Shoulder bearings + 25 h6 stock** are the two items worth spending a minute
  on: get a **matched 7005 DB pair** (or accept the 6005 stand-in for the
  prototype), and enough 25 h6 bar to make both shafts.
- **Two custom parts** need a lathe: the **shoulder shaft** (flat) and the
  **TCP spindle** (hollow + flange). Everything else is off-the-shelf or a
  pilot-bore + keyway job.
