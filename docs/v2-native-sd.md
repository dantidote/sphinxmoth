# v2: native SD (design doc)

Decision 2026-07-09: v2 replaces the CF socket with a native microSD slot and
the FPGA absorbs the disk. The FC1307A stack works (256GB SD syncing today)
but is mechanically bulky and a black box; native SD removes the middleman.

## Architecture shift

v1 (shipping): the FPGA is a *translator*. Both iPod masters talk ATA to the
CF card; the bridge re-times UDMA bursts, generates the CRC the iSphynxII
omits, and passes PIO through untouched. Everything hard about "being a
disk" is delegated to the card.

v2: the FPGA is *the disk*.

```
 iPod 50-pin  <->  ata_device (full ATA target)  <->  block backend  <->  sd_host  <->  microSD
                   |-- taskfile, IDENTIFY ROM,        (512B sector       (CMD/DAT engines,
                       command decode, INTRQ,          streams +          CRC7/CRC16, init,
                       signatures, error model)        flow control)      CMD17/18/24/25, HS)
```

## What carries over from v1 (hardware-proven, do not re-learn)

- HS-side electrical wisdom: staged data loads (no DD move on a strobe
  edge), DSTROBE-return-after-DMARQ order (S_FIN), tSS before STOP,
  dead-time strobe filter, capture gated by DMACK.
- The three-personality split: iSphynxII = UDMA2 (paces itself, never
  checks/sends usable CRC -- WE originate/ignore per direction); PP5002
  retail OS = demote to PIO via IDENTIFY (its write strobes are not
  sampleable at our clock; that fact is independent of the backend);
  loader = simple PIO reads.
- BSY for status polls while a command is in flight; the iSphynx treats a
  floating bus as success.
- Flow-control temperament: the iSphynx abandons bursts on long clean
  DDMARDY pauses but duty-cycles happily against a bouncing throttle.
- sync_fifo + hysteretic/raw dual flow-control pattern.
- The UART event logger. Non-negotiable. It closed fourteen bugs.

## Command inventory (from v1 bring-up UART logs -- this is the whole spec)

| Cmd | Who | Notes |
|---|---|---|
| EC IDENTIFY | all | serve from ROM/generated block; words 63/88 zeroed in pp_sess; checksum word 255 |
| 20 READ SECTORS | loader, OS | PIO data-in, SC=0 means 256 |
| C4/C5 READ/WRITE MULTIPLE | OS | PIO block mode, honors C6 |
| C6 SET MULTIPLE | OS | latch block size |
| C8/CA READ/WRITE DMA | iSphynx | UDMA2 bursts, multi-burst per command |
| EF SET FEATURES | all | 03=xfer mode (accept, mostly ignore), 05/A0/FE APM (abort w/ S51 -- OS tolerates), 66 |
| 90 EXECUTE DIAGNOSTIC | OS only | pp_sess discriminator! set err=01 |
| B0 SMART D8/D4/D0 | loader, OS | stub: accept enable, return canned data |
| E0 STANDBY IMMEDIATE | OS | ack, no-op |
| E7 FLUSH CACHE | iSphynx | flush backend write path, then ready |
| SRST via devctl | all | full device reset + signature (SC=01, LBA 01/00/00) |

## sd_host scope

- Init: CMD0, CMD8, ACMD41 (HCS), CMD2/3, CMD7, ACMD6 (4-bit), CMD16,
  optional CMD6 High Speed (50MHz). 3.3V signaling only -- no UHS. 25-50MB/s
  ceiling >> iPod's appetite.
- Data: CMD17/18 reads, CMD24/25 writes with multi-block + CMD12 stop;
  CRC16 per DAT line; busy handling on DAT0; sector-granular interface to
  the FIFO. SDHC/SDXC block addressing (v1 rig already runs 256GB).
- Sim: SD card BFM in the TB family, same hardware-calibrated-cruelty
  philosophy as the v1 models.

## FPGA sizing

v1 sits at 944/1056 SLICEs (89%) on the LCMXO2-2000HC-4TG100C -- B does not
fit. ~~v2 specs LCMXO2-7000HC-4TG100~~ **CORRECTION (2026-07-10): the
XO2-4000/7000 do not exist in TQFP-100** (package DB + DS both say TQFP-144
and BGAs only). v2 uses **LCMXO2-7000HC-4TG144C** -- where this project's
first draft started. Built: 55% SLICEs, 11/26 EBR, 66.8MHz max vs 66.5
required with the board pinout locked.

## De-risk plan (phases)

0. This document + repo scaffolding.                          [done]
1. `ata_device` core behind a `blk_*` backend interface.      [done 2026-07-10]
   Written + sim-verified against the v1 hardware-calibrated host models
   (PIO session, UDMA read/write with ringing + parking + garbage CRC).
   The optional backend-CF shim for v1-board validation was skipped -- the
   sim torture plus the unchanged udma_device engine carry the risk instead.
2. `sd_host` + BFM, sim-complete.                             [done 2026-07-10]
   Plus backend_sd: verify-then-release read buffering (SD CRC error =
   bounded re-read, never bad data on the ATA bus), private write sector
   buffer (host reset cannot tear an SD block), fault-injection TB proves
   both retry paths. SD_FAST (33.25MHz + CMD6 HS) exists but ships OFF
   until the sample point is validated on silicon.
3. v2 board: microSD, XO2-7000 TG144, CF socket deleted.      [done 2026-07-10]
   board-v2/ipod-sd-udma.kicad_pcb, **38×36mm 4-layer**, 0 unconnected +
   0 electrical DRC. Derived ENTIRELY by script from the v1 board. Simplified
   to one active part per user direction (USB/LED/config-pulls/SD-R's gone);
   microSD + all decoupling flipped to the BACK, stacked under the FPGA.
   4-layer (Sig/GND/3V3/Sig) because a 144-pin 0.5mm QFP + 100 signals in a
   compact outline is past a clean 2-layer route. Geometry-aware FPGA pin
   assignment (assign_pins.py) makes the connector fanout crossing-free so
   freerouting closes all 121 nets; LPF back-annotated, Diamond holds 67.1MHz.
4. Integration: v2 silicon bring-up with the UART microscope.  [pending fab]

## Board evolution note (v1 53×72mm 2-layer -> v2 38×36mm 4-layer)

The compaction pass (user: "smallest board") drove two structural changes the
original doc didn't anticipate: (a) microSD + decoupling move to the back and
stack under the FPGA, and (b) the board goes 4-layer -- a 144-pin 0.5mm QFP
with 100+ signals cannot be cleanly 2-layer-routed in a compact outline (v1
only managed 2 layers as a 100-pin part on a 3x-larger board). The FPGA pin
assignment is now geometry-driven (pins paired in-order to the fanout), which
trades a little internal Fmax (recovered via timing-driven MAP + a registered
last_blk) for a routable board.

## Board notes for phase 3

- Keep: 50-pin edge, TC2030 JTAG, UART header (strengthened pads), USB-C
  bench power + J6 select.
- Drop: CF socket (and its True-IDE strapping saga), CSEL/DASP/PDIAG ties.
- Add: microSD push-push socket (top side now -- no CF overhang to dodge),
  6 FPGA pins (CLK/CMD/DAT0-3) with series terminations, guarded routing.
- Consider: keep the FC1307A footprint DNP'd as a plan-B backend? Decided
  against -- v1 boards remain the FC1307A fallback.
