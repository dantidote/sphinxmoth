#!/usr/bin/env python
# build_v2.py -- derive the v2 (native microSD) board from the v1 board.
# Run with KiCad's python:  "C:\Program Files\KiCad\10.0\bin\python.exe" build_v2.py
#
# Keeps: outline, J1 (proven iPod edge geometry), USB-C/LDO/J6 power section
#        with its routing, TC2030, UART header (upgraded to through-hole per
#        v1 errata #3), LED, bulk caps.
# Drops: CF socket + all CF nets, TQFP-100 U1, all FPGA-adjacent routing.
# Adds:  LCMXO2-7000HC-4TG144C (TQFP-144), Hirose DM3AT microSD (top side),
#        6x 33R SD series terminations, 2 extra 100nF, SD nets.
#
# Pin map + supply map validated against three sources: Diamond PAD report,
# Diamond spreadsheet export, official DS FPGA-DS-02056 Table 4.6 counts.

import pcbnew
import os

SRC = os.path.join(REPO, r"ipod-cf-udma.kicad_pcb")
DST = os.path.join(REPO, r"board-v2\ipod-sd-udma.kicad_pcb")
FPLIB = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"

def mm(v): return pcbnew.FromMM(v)
def P(x, y): return pcbnew.VECTOR2I(mm(x), mm(y))

# ---------------------------------------------------------------------------
# the validated TG144 map
# ---------------------------------------------------------------------------
GND_PINS  = [8, 18, 29, 46, 53, 64, 80, 90, 101, 116, 124, 134]
P3V3_PINS = [36, 72, 108, 144,            # VCC core (int. regulator, HC)
             7, 16, 30, 37, 51, 66, 79, 88, 102, 118, 123, 135]  # VCCIO0-5
NC_PINS   = [129]

SIG = {
    # SD, left (banks 4/3) -- through 33R series resistors to the socket
    13: "SDD2", 14: "SDD3", 15: "SDCMD", 17: "SDCLK",
    20: "SDD0", 21: "SDD1", 24: "SDCD",
    # HS, bottom (bank 2), left->right matching J1 pad order
    38: "HRESET",
    39: "HD7", 41: "HD8", 42: "HD6", 43: "HD9", 47: "HD5", 48: "HD10",
    49: "HD4", 50: "HD11", 52: "HD3", 54: "HD12", 55: "HD2", 56: "HD13",
    57: "HD1", 58: "HD14", 59: "HD0", 60: "HD15",
    61: "HDMARQ", 62: "HDIOW", 63: "HDIOR", 65: "HIORDY",
    67: "HDMACK", 68: "HINTRQ", 69: "HDA1",
    # right (bank 1)
    73: "HDA0", 74: "HDA2", 75: "HCS0", 76: "HCS1",
    99: "U_TX", 100: "U_RX",
    # top (bank 0) -- JTAG only; LED gone, PROGRAMN/DONE are plain unused IO
    130: "TMS", 131: "TCK", 136: "TDI", 137: "TDO",
}

# microSD socket J2 (Hirose DM3AT): 1=DAT2 2=CD/DAT3 3=CMD 4=VDD 5=CLK
# 6=VSS 7=DAT0 8=DAT1, 9/10 = detect switch, SH = shield.
# SD lines run DIRECT to the FPGA: at 16.6MHz over ~2cm, programmable drive
# (8mA) + slew control in the LPF replace discrete series terminations.
J2_NETS = {
    "1": "SDD2", "2": "SDD3", "3": "SDCMD", "4": "P3V3",
    "5": "SDCLK", "6": "GND", "7": "SDD0", "8": "SDD1",
    "9": "SDCD", "10": "GND", "SH": "GND",
}

# v2 drops the USB-C bench-power section entirely (J4/U2/J6 + friends);
# the iPod's 3.3V is the only source and VIPOD collapses into P3V3.
# Bench power for bring-up: J5 pin 4 or TC2030 pad 1 (both P3V3).
# v2 compaction re-routes everything from scratch, so keep NO tracks.
KEEP_TRACK_NETS = set()

# ---- compact outline (smallest board: J1 card-edge width sets the floor) -----
# J1 stays put (its pad-to-edge geometry mates the iPod); the board hugs it.
LEFT, RIGHT = 107.5, 145.5            # 38 mm, centered on J1 (x=126.46)
TOP, BOT    = 126.53, 162.53          # 36 mm; BOT = J1 card-edge contact edge
U1_C  = (126.5, 141.0)                # FPGA up, leaving a J1<->U1 fanout channel
J2_C  = (126.5, 137.0)                # microSD on the BACK, slot at TOP edge,
                                      # body overlapping U1 in the Z-stack

DEAD_NETS = set()
for pfx in ("CD",):
    DEAD_NETS |= {"%s%d" % (pfx, i) for i in range(16)}
DEAD_NETS |= {"CCS0", "CCS1", "CDA0", "CDA1", "CDA2", "CDIOR", "CDIOW",
              "CDMACK", "CDMARQ", "CFRESET", "CINTRQ", "CIORDY"}

# ---------------------------------------------------------------------------
import shutil
import os
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # repo root
shutil.copyfile(SRC, DST)
b = pcbnew.LoadBoard(DST)

def net(name):
    n = b.FindNet(name)
    if n is None:
        n = pcbnew.NETINFO_ITEM(b, name)
        b.Add(n)
    return n

report = []

# ---- 1. remove old U1 + J2 ------------------------------------------------
# (keep the python proxies alive: SWIG has no FOOTPRINT dtor and freeing a
# removed footprint's proxy mid-script corrupts later API calls)
_graveyard = []
for ref in ("U1", "J2"):
    fp = b.FindFootprintByReference(ref)
    assert fp, ref
    b.Remove(fp)
    _graveyard.append(fp)
report.append("removed old U1, J2")

# ---- 1b. remove the USB-C bench-power section --------------------------------
for ref in ("J4", "U2", "C17", "C18", "R8", "R9", "J6"):
    fp = b.FindFootprintByReference(ref)
    assert fp, ref
    b.Remove(fp)
    _graveyard.append(fp)
report.append("removed USB section: J4 U2 C17 C18 R8 R9 J6")

# ---- 1c. simplify: no LED, no PROGRAMN/DONE pulls ----------------------------
# D1/R7: the UART boot banner is the alive-signal. R1/R2: PROGRAMN(119) and
# DONE(109) are plain user IO unless SYSCONFIG enables them (it doesn't);
# JTAG_PORT=ENABLE is the recovery path.
for ref in ("D1", "R7", "R1", "R2"):
    fp = b.FindFootprintByReference(ref)
    assert fp, ref
    b.Remove(fp)
    _graveyard.append(fp)
report.append("removed LED + config pull-ups: D1 R7 R1 R2")

# ---- 2. rip ALL tracks (full re-route on the compact outline) ----------------
killed = 0
for t in list(b.GetTracks()):
    b.Remove(t); _graveyard.append(t); killed += 1
report.append("tracks: killed all %d" % killed)

# ---- 2b. VIPOD collapses into P3V3 (single supply, no source mux) ------------
p3v3 = None
def _p3v3():
    return net("P3V3")
vipod_pads = 0
for fp in b.GetFootprints():
    for pad in fp.Pads():
        if pad.GetNetname() == "VIPOD":
            pad.SetNet(_p3v3()); vipod_pads += 1
vipod_trks = 0
for t in b.GetTracks():
    if t.GetNetname() == "VIPOD":
        t.SetNet(_p3v3()); vipod_trks += 1
report.append("VIPOD -> P3V3: %d pads, %d track segments" % (vipod_pads, vipod_trks))

# ---- 2c. redraw Edge.Cuts as the tight rectangle -----------------------------
for d in list(b.GetDrawings()):
    if b.GetLayerName(d.GetLayer()) == "Edge.Cuts":
        b.Remove(d); _graveyard.append(d)
edge = pcbnew.PCB_SHAPE(b)
edge.SetShape(pcbnew.SHAPE_T_RECT)
edge.SetStart(P(LEFT, TOP))
edge.SetEnd(P(RIGHT, BOT))
edge.SetLayer(pcbnew.Edge_Cuts)
edge.SetWidth(mm(0.1))
b.Add(edge)
report.append("outline -> %.0f x %.0f mm" % (RIGHT-LEFT, BOT-TOP))

# ---- 2d. go 4-LAYER: Sig / GND / 3V3 / Sig -----------------------------------
# A 144-pin 0.5mm QFP with 100+ signals in a compact outline is beyond a clean
# 2-layer route (v1 was a 100-pin part on a 3x-larger board). Dedicated inner
# GND + 3V3 planes make signal routing a 2-layer problem again (F.Cu + B.Cu,
# each referencing a plane) and every power/GND pin vias straight to its plane.
# No BOM change -- 4 layers is a fab attribute.
b.SetCopperLayerCount(4)
IN1, IN2 = pcbnew.In1_Cu, pcbnew.In2_Cu
b.SetLayerName(IN1, "GND")
b.SetLayerName(IN2, "PWR")
# compact board: 0.2mm copper-to-edge (JLCPCB-safe) so the tight J1<->U1
# fanout channel doesn't trip the default 0.5mm rule
b.GetDesignSettings().m_CopperEdgeClearance = mm(0.2)
# retarget the two existing pours to the inner planes, resized to the outline
zones = list(b.Zones())
for z in zones:
    op = z.Outline()
    op.RemoveAllContours()
    op.NewOutline()
    for (x, y) in [(LEFT, TOP), (RIGHT, TOP), (RIGHT, BOT), (LEFT, BOT)]:
        op.Append(mm(x), mm(y))
    if b.GetLayerName(z.GetLayer()) == "F.Cu":
        z.SetLayer(IN1); z.SetNet(net("GND"))
    else:
        z.SetLayer(IN2); z.SetNet(net("P3V3"))
    z.HatchBorder()
report.append("4-layer: In1=GND plane, In2=3V3 plane; F.Cu/B.Cu = signal")

# ---- 3. place the new FPGA (centered above J1) -------------------------------
u1 = pcbnew.FootprintLoad(os.path.join(FPLIB, "Package_QFP.pretty"),
                          "TQFP-144_20x20mm_P0.5mm")
u1.SetReference("U1")
u1.SetValue("LCMXO2-7000HC-4TG144C")
u1.SetPosition(P(*U1_C))
b.Add(u1)

for pad in u1.Pads():
    n = int(pad.GetNumber())
    if n in SIG:
        pad.SetNet(net(SIG[n]))
    elif n in GND_PINS:
        pad.SetNet(net("GND"))
    elif n in P3V3_PINS:
        pad.SetNet(net("P3V3"))
report.append("U1 TQFP-144 placed + bound")

# ---- 4. microSD socket: BACK side, card slot at the TOP edge ----------------
# Flipping to B.Cu lets the card body overlap U1 in the Z-stack; the DM3AT's
# insertion opening sits at the top board edge so the card slides in from
# outside and lands on the back, under the FPGA.
j2 = pcbnew.FootprintLoad(os.path.join(FPLIB, "Connector_Card.pretty"),
                          "microSD_HC_Hirose_DM3AT-SF-PEJM5")
j2.SetReference("J2")
j2.SetValue("microSD DM3AT-SF-PEJM5")
j2.SetPosition(P(*J2_C))
b.Add(j2)

# fit on the FRONT first (real pad geometry), then Flip() to the back so the
# PADS actually move to B.Cu -- SetLayer alone leaves pads on F.Cu (they then
# collide with U1). Want the card contacts (pads 1..8) at the DEEP end (+y),
# so the insertion opening is at the top board edge; long axis vertical.
def pad_ext(fp):
    xs = [p.GetPosition().x for p in fp.Pads()]
    ys = [p.GetPosition().y for p in fp.Pads()]
    return min(xs), max(xs), min(ys), max(ys)
best = None
for rot in (0, 90, 180, 270):
    j2.SetOrientationDegrees(rot)
    x0, x1, y0, y1 = pad_ext(j2)
    padyc = sum(j2.FindPadByNumber(str(k)).GetPosition().y for k in range(1, 9)) / 8
    tall = (y1 - y0) >= (x1 - x0)
    if tall and padyc > (y0 + y1) / 2:            # contacts deep (+y)
        best = rot
        break
j2.SetOrientationDegrees(best if best is not None else 0)
x0, x1, y0, y1 = pad_ext(j2)
dx = mm((LEFT + RIGHT) / 2) - (x0 + x1) // 2
dy = mm(TOP + 1.0) - y0
j2.SetPosition(pcbnew.VECTOR2I(j2.GetPosition().x + dx, j2.GetPosition().y + dy))
j2.Flip(j2.GetPosition(), False)                  # -> B.Cu, pads mirror to back
x0, x1, y0, y1 = pad_ext(j2)
report.append("J2 microSD FLIPPED to %s rot %s pad-extent x %.1f..%.1f y %.1f..%.1f" %
              (b.GetLayerName(j2.GetLayer()), best, x0/1e6, x1/1e6, y0/1e6, y1/1e6))
for num, nn in J2_NETS.items():
    pad = j2.FindPadByNumber(num)
    if pad is None:
        report.append("  !! J2 pad %s missing" % num)
        continue
    pad.SetNet(net(nn))
for pad in j2.Pads():
    if pad.GetNumber() == "SH":
        pad.SetNet(net("GND"))
report.append("J2 microSD placed + bound (back)")

# ---- 5. (series terminations deleted -- SD lines run direct) -----------------

# ---- 6. ALL caps on the BACK, under U1, clear of the microSD -----------------
# A 22mm QFP in a 31mm board leaves no front room around it, so decoupling
# goes on the back directly beneath U1 (shortest loop) -- the classic place
# for it. J2 owns the top-center of the back; caps fill the L/R columns and
# the strip below J2. Sub-mm from the VCC/VCCIO pins they bypass, through the
# short via the pin already needs.
ux, uy = U1_C
j2fp = b.FindFootprintByReference("J2")
jx = [p.GetPosition().x for p in j2fp.Pads()]
jy = [p.GetPosition().y for p in j2fp.Pads()]
J2BOX = (min(jx) - mm(0.8), min(jy) - mm(0.8), max(jx) + mm(0.8), max(jy) + mm(0.8))
def hits_j2(x, y):
    return J2BOX[0] <= mm(x) <= J2BOX[2] and J2BOX[1] <= mm(y) <= J2BOX[3]

ALLCAPS = ["C1", "C2", "C3", "C4", "C6", "C7", "C8", "C9",
           "C11", "C12", "C13", "C14",           # 12x 100nF
           "C5", "C10", "C16", "C15"]            # bulk + VCCAUX
# candidate grid on the back, inside U1's pad ring (x 116.5..136.5, y 134..154)
slots = []
yy = 135.0
while yy <= 154.0:
    xx = 117.5
    while xx <= 135.5:
        if not hits_j2(xx, yy):
            slots.append((xx, yy))
        xx += 3.6
    yy += 3.0
placed = 0
for ref, (x, y) in zip(ALLCAPS, slots):
    fp = b.FindFootprintByReference(ref)
    fp.SetPosition(P(x, y))
    fp.SetOrientationDegrees(0)
    fp.Flip(fp.GetPosition(), False)      # real flip: pads move to B.Cu
    placed += 1
report.append("caps -> back (flipped), %d placed under U1 (of %d), %d slots"
              % (placed, len(ALLCAPS), len(slots)))

# ---- 7. (bulk caps folded into the back cap field above) ---------------------

# ---- 8. UART header: through-hole (v1 errata #3), left margin ----------------
j5old = b.FindFootprintByReference("J5")
j5nets = {p.GetNumber(): p.GetNetname() for p in j5old.Pads()}
b.Remove(j5old)
_graveyard.append(j5old)
j5 = pcbnew.FootprintLoad(os.path.join(FPLIB, "Connector_PinHeader_1.27mm.pretty"),
                          "PinHeader_1x04_P1.27mm_Vertical")
j5.SetReference("J5")
j5.SetValue("UART")
b.Add(j5)
# orient VERTICAL (pads stacked in y) so the 4-pin header fits the ~5mm left
# margin without touching U1's left pad column
for rot in (0, 90):
    j5.SetOrientationDegrees(rot)
    xs = [p.GetPosition().x for p in j5.Pads()]
    ys = [p.GetPosition().y for p in j5.Pads()]
    if (max(ys) - min(ys)) > (max(xs) - min(xs)):
        break
j5.SetPosition(P(LEFT + 1.8, uy + 2.0))
for num, nn in j5nets.items():
    if nn:
        j5.FindPadByNumber(num).SetNet(net(nn))
report.append("J5 UART -> through-hole, left margin")

# ---- 8a. TC2030 JTAG: top-right, near U1's top-edge JTAG pins ----------------
j3 = b.FindFootprintByReference("J3")
j3.SetOrientationDegrees(0)                        # keep tooling-hole clearance
j3.SetPosition(P(RIGHT - 5.0, TOP + 3.5))
report.append("J3 TC2030 -> top-right, near JTAG pins")

cleared = 0
report.append("cleared %d kept tracks under new parts" % cleared)

# ---- 9. dead CF nets stay in the net table -----------------------------------
# (pcbnew's python Remove() cannot take NETINFO_ITEMs; padless nets are
# harmless -- KiCad prunes them on the next GUI save)
report.append("dead CF nets left padless (pruned by KiCad on save)")

# ---- 9b. prune dangling stubs left by the region clears -----------------------
# a segment is dangling if an endpoint touches neither a pad nor another track
def endpoints_index():
    from collections import defaultdict
    idx = defaultdict(int)
    for t in b.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            idx[(t.GetPosition().x, t.GetPosition().y)] += 2
        else:
            idx[(t.GetStart().x, t.GetStart().y)] += 1
            idx[(t.GetEnd().x, t.GetEnd().y)] += 1
    return idx
pad_pts = set()
for fp in b.GetFootprints():
    for pad in fp.Pads():
        pad_pts.add((pad.GetPosition().x, pad.GetPosition().y))
for _pass in range(4):
    idx = endpoints_index()
    drop = []
    for t in b.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            continue
        loose = 0
        for p in (t.GetStart(), t.GetEnd()):
            k = (p.x, p.y)
            if idx[k] <= 1 and k not in pad_pts:
                loose += 1
        if loose:
            drop.append(t)
    if not drop:
        break
    for t in drop:
        b.Remove(t); _graveyard.append(t)
report.append("pruned dangling stubs")

# ---- 10. refill the pours (stale after all the surgery) ----------------------
filler = pcbnew.ZONE_FILLER(b)
filler.Fill(b.Zones())
report.append("zones refilled")

pcbnew.SaveBoard(DST, b)
print("\n".join(report))
print("saved", DST)

# ---- 10. verify (guard against silent net-binding drops) --------------------
b2 = pcbnew.LoadBoard(DST)
u1v = b2.FindFootprintByReference("U1")
bad = []
for pad in u1v.Pads():
    n = int(pad.GetNumber())
    want = SIG.get(n) or ("GND" if n in GND_PINS else
                          "P3V3" if n in P3V3_PINS else "")
    got = pad.GetNetname()
    if got != want:
        bad.append((n, want, got))
j2v = b2.FindFootprintByReference("J2")
for num, want in J2_NETS.items():
    p = j2v.FindPadByNumber(num)
    got = p.GetNetname() if p else "<no pad>"
    if got != want:
        bad.append(("J2." + num, want, got))
if bad:
    print("BINDING FAILURES:")
    for x in bad:
        print("  ", x)
    raise SystemExit(1)
print("VERIFY: all %d U1 pads + %d J2 pads bound correctly after reload"
      % (u1v.Pads().size(), j2v.Pads().size()))
