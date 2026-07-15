# ipod-sd-udma - v2 board (native microSD, compact 4-layer)

The v2 realization of [docs/v2-native-sd.md](../docs/v2-native-sd.md): the CF
socket is gone, the FPGA *is* the disk, sectors live on a microSD card, and
the board is as small and as simple as it can be. Derived **entirely by
script** from the proven v1 board - the flow below replays from
`../ipod-cf-udma.kicad_pcb`.

```
 iPod 1.8" 50-pin (J1)  <->  LCMXO2-7000HC-4TG144 (U1, front)  <->  microSD (J2, back)
```

* **38 × 36 mm, 4-layer** - down from v1's 53 × 72 mm 2-layer (48% of the area).
* **microSD on the BACK**, stacked under the FPGA, card slot at the top edge.
* **0 unconnected, 0 electrical DRC violations.** (Two copper-to-edge items
  remain: the microSD's shield tabs at the top edge, inherent to a socket that
  sits at the board edge for card insertion - v1 shipped the identical benign
  overlap on its CF socket.)

## The whole BOM

| Ref | Part | Side |
|---|---|---|
| U1 | LCMXO2-7000HC-4TG144C - the only active component | front |
| J1 | iPod 50-pin card edge (custom fp, v1-proven, untouched) | edge |
| J2 | Hirose DM3AT-SF-PEJM5 microSD push-push | back |
| J3 | Tag-Connect TC2030 pads (JTAG) | front |
| J5 | 4-pin 1.27 mm **through-hole** UART header (v1 errata #3) | front |
| C1–C16 | 12× 100 nF + 4.7 µF + 2× 10 µF + 1 µF, all decoupling | back |

Zero resistors, zero LEDs, zero regulators, no USB. What got deleted vs v1
and why it's safe:

* **USB-C + LDO + J6 source select** - single supply: the iPod's 3.3 V IS the
  rail. Bench power for bring-up: J5 pin 4 or TC2030 pad 1.
* **SD series resistors** - 16.6 MHz over ~2 cm; LPF drive/slew does the job.
* **PROGRAMN/DONE pull-ups (R1/R2)** - plain user I/O with SYSCONFIG off;
  JTAG_PORT=ENABLE is the recovery path.
* **Heartbeat LED (D1/R7)** - the UART `B=B0` boot banner is the alive-signal.
* **CF strapping saga** - gone with the socket. Card-detect switch feeds
  SD_CD_N; no card ⇒ the ATA face holds BSY.

## Why 4 layers

A 144-pin 0.5 mm QFP with 100+ signals in a compact outline is beyond a clean
2-layer route (v1 got away with 2 layers only because it was a 100-pin part on
a board 3× the size). The stackup is **Sig / GND / 3V3 / Sig**: dedicated inner
GND + 3V3 planes turn signal routing back into a 2-layer problem (both outer
layers reference a plane) and every power/GND pin vias straight to its plane -
no power traces, no stitching drama. 4 layers is a fab attribute; the BOM is
unchanged.

## FPGA pin assignment

Geometry-driven (`assign_pins.py`, the v1 trick): the FPGA's I/O pins are
free, so they're paired **in order** to the connector fanout - HD0–15 map to
U1's bottom-edge pins in the same x-order as J1's fingers, SD to the top edge,
UART to the left. That crossing-free fanout is what makes the compact board
routable. The LPF (`../diamond-sd/ipodboard_sd.lpf`) is back-annotated to
match, and Diamond closes timing at **67.1 MHz** (vs 66.5 needed) with it.
Supply map cross-validated three ways (Diamond PAD + spreadsheet + DS Table
4.6): 12 GND, 4 VCC, 12 VCCIO, pin 129 NC. Full table: `tg144_pins.csv`.

## Flow (all rerunnable)

| Step | File |
|---|---|
| derive + place (outline, 4-layer, flip parts to back) | `build_v2.py` |
| geometry pin assignment + LPF back-annotation | `assign_pins.py` |
| Specctra DSN export / SES import | `route_v2.py` |
| freerouting with a fail-fast load-hang guard | `fr.sh` |
| rip stuck signal nets & re-export (if needed) | `riproute.py` |
| hand-route residual congested nets | `hand_route.py` |
| stitch any power pad the router missed | `stitch_v2.py` |
| gerbers/drill/pos + renders / BOM | `finalize_v2.sh` / `gen_bom.py` |

Typical run: `build_v2 → assign_pins → route_v2 export → fr.sh 80 →
route_v2 import → DRC`. freerouting closes all 121 nets (signals + power-to-
plane) in seconds on the 4-layer board; only occasionally does it need a
`hand_route`/`stitch` top-up.

## Bring-up notes

* Gateware: `../diamond-sd/` → `impl1/ipodboard_sd_impl1.jed`, flash via
  Diamond Programmer + TC2030 on J3.
* `SD_FAST=0` for first silicon (16.6 MHz SD clock, fat margins); enable the
  33.25 MHz CMD6 high-speed path only after scoping DAT.
* UART logger (J5) speaks the v1 5-char event language - `M`/`m` = SD init
  ok/fail, `G` = SD retry counter, `B=B0` = board alive.

## Schematic

Not redrawn - `build_v2.py` + `assign_pins.py` (and the LPF) are the netlist
ground truth. Redraw `ipod-sd-udma.kicad_sch` from the board before a respin
if schematic-parity matters.
