#!/usr/bin/env python
# assign_pins.py -- geometry-aware FPGA pin assignment (the v1 method: assign
# I/O pins to suit the layout, then back-annotate the LPF). Signals get the
# nearest free U1 I/O pin, paired IN ORDER along each edge so the connector
# fanout has no crossings -- which is what freerouting needs to finish.
#
# Reads the placed board, re-binds U1's signal pads, saves, and rewrites the
# Diamond LPF LOCATEs to match. Power/JTAG pins are fixed and never moved.
import pcbnew
import os
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # repo root

PCB = os.path.join(REPO, r"board-v2\ipod-sd-udma.kicad_pcb")
LPF = os.path.join(REPO, r"diamond-sd\ipodboard_sd.lpf")
b = pcbnew.LoadBoard(PCB)

GND_PINS  = {8,18,29,46,53,64,80,90,101,116,124,134}
P3V3_PINS = {36,72,108,144,7,16,30,37,51,66,79,88,102,118,123,135}
JTAG      = {130,131,136,137}
FIXED     = GND_PINS | P3V3_PINS | JTAG | {129}      # 129 = NC

u1 = b.FindFootprintByReference("U1")
uc = u1.GetPosition()
# free I/O pins with positions
pins = {}
for pad in u1.Pads():
    n = int(pad.GetNumber())
    if n not in FIXED:
        p = pad.GetPosition()
        pins[n] = (p.x, p.y)

def edge(n):
    x, y = pins[n]
    dx, dy = x - uc.x, y - uc.y
    if abs(dy) >= abs(dx):
        return "B" if dy > 0 else "T"
    return "R" if dx > 0 else "L"
edges = {"T": [], "B": [], "L": [], "R": []}
for n in pins:
    edges[edge(n)].append(n)

# connector signal pads: net -> (x, y)
def conn_nets(ref):
    fp = b.FindFootprintByReference(ref)
    out = {}
    for pad in fp.Pads():
        nn = pad.GetNetname()
        if nn and nn not in ("GND", "P3V3", ""):
            p = pad.GetPosition()
            out[nn] = (p.x, p.y)
    return out
j1 = conn_nets("J1")     # bottom edge
j2 = conn_nets("J2")     # top edge (back)
j5 = conn_nets("J5")     # left margin

assign = {}              # net -> pin
used = set()

def pair_in_order(targets, edge_pins, secondary=None):
    # targets: list of (net,(x,y)); pair to edge_pins sorted by x, in x-order.
    # overflow spills into `secondary` edge pins (also x-sorted).
    tgt = sorted(targets, key=lambda t: t[1][0])
    avail = sorted((p for p in edge_pins if p not in used),
                   key=lambda p: pins[p][0])
    if secondary:
        avail += sorted((p for p in secondary if p not in used),
                        key=lambda p: pins[p][0])
    for (net, _), pin in zip(tgt, avail):
        assign[net] = pin
        used.add(pin)

# J1 -> bottom edge, x-ordered (overflow to lower L/R handled by extra pins)
pair_in_order(list(j1.items()), edges["B"], secondary=edges["R"] + edges["L"])
# J2 -> top edge, x-ordered
pair_in_order(list(j2.items()), edges["T"], secondary=edges["L"] + edges["R"])
# J5 (U_TX/U_RX) -> nearest free left-edge pins
for net, (tx, ty) in sorted(j5.items()):
    cand = sorted((p for p in edges["L"] if p not in used),
                  key=lambda p: (pins[p][0]-tx)**2 + (pins[p][1]-ty)**2)
    if cand:
        assign[net] = cand[0]; used.add(cand[0])

missing = (set(j1) | set(j2) | set(j5)) - set(assign)
if missing:
    print("UNASSIGNED:", missing)
    raise SystemExit(1)
print("assigned %d signal nets" % len(assign))

# re-bind U1 pads: apply the new assignment; leave unused I/O pins on the
# board's UNCONNECTED net (netcode 0) -- creating fresh empty NETINFO_ITEMs
# that aren't added to the board leaves dangling refs that corrupt the DSN
# export (freerouting then hangs at parse).
pin_net = {pin: net for net, pin in assign.items()}
unconn = b.FindNet(0)
for pad in u1.Pads():
    n = int(pad.GetNumber())
    if n in FIXED:
        continue
    if n in pin_net:
        pad.SetNet(b.FindNet(pin_net[n]))
    else:
        pad.SetNet(unconn)
pcbnew.SaveBoard(PCB, b)
print("re-bound U1, saved board", flush=True)

# ---- rewrite the LPF LOCATEs from the new assignment -------------------------
NET2PORT = {}
for i in range(16):
    NET2PORT["HD%d" % i] = "HS_DD[%d]" % i
    if i < 4:
        NET2PORT["SDD%d" % i] = "SD_DAT[%d]" % i
NET2PORT.update({
    "HCS0":"HS_CS0_N","HCS1":"HS_CS1_N","HDA0":"HS_A0","HDA1":"HS_A1",
    "HDA2":"HS_A2","HDIOR":"HS_IOR_N","HDIOW":"HS_IOW_N","HDMARQ":"HS_DMARQ",
    "HDMACK":"HS_DMACK_N","HRESET":"HS_RESET_N","HINTRQ":"HS_INTRQ",
    "HIORDY":"HS_IORDY","SDCLK":"SD_CLK","SDCMD":"SD_CMD","SDCD":"SD_CD_N",
    "U_TX":"UART_TX","U_RX":"UART_RX",
})
locates = "\n".join(
    'LOCATE COMP "%s"  SITE "%d";' % (NET2PORT[net], pin)
    for net, pin in sorted(assign.items(), key=lambda kv: NET2PORT[kv[0]]))

lpf = '''# LCMXO2-7000HC-4TG144C -- v2 (compact native-SD board) pin LOCATEs.
# GEOMETRY-ASSIGNED to board-v2/ipod-sd-udma.kicad_pcb by assign_pins.py:
# I/O pins paired in-order to the connector fanout (v1 method). Supply map
# cross-validated (Diamond PAD + spreadsheet + DS Table 4.6): 12 GND, 4 VCC,
# 12 VCCIO, pin 129 NC. Port names match rtl/fpga_top_sd.v.

BLOCK RESETPATHS;
BLOCK ASYNCPATHS;

FREQUENCY NET "clk" 66.500000 MHz;

SYSCONFIG JTAG_PORT=ENABLE MASTER_SPI_PORT=DISABLE SLAVE_SPI_PORT=DISABLE I2C_PORT=DISABLE;

IOBUF ALLPORTS IO_TYPE=LVCMOS33;

# ---- pin LOCATEs (geometry-assigned) ----------------------------------------
%s

# ---- pulls / drive ----------------------------------------------------------
IOBUF PORT "HS_IOR_N"   PULLMODE=UP;
IOBUF PORT "HS_IOW_N"   PULLMODE=UP;
IOBUF PORT "HS_DMACK_N" PULLMODE=UP;
IOBUF PORT "HS_CS0_N"   PULLMODE=UP;
IOBUF PORT "HS_CS1_N"   PULLMODE=UP;
IOBUF PORT "HS_RESET_N" PULLMODE=UP;
IOBUF PORT "UART_RX"    PULLMODE=UP;
IOBUF PORT "SD_CMD"     PULLMODE=UP;
IOBUF GROUP "SD_DAT*"   PULLMODE=UP;
IOBUF PORT "SD_CD_N"    PULLMODE=UP;

IOBUF PORT "HS_IORDY"   DRIVE=8 SLEWRATE=FAST;
IOBUF PORT "HS_DMARQ"   DRIVE=8 SLEWRATE=FAST;
IOBUF GROUP "HS_DD*"    DRIVE=8 SLEWRATE=FAST;
IOBUF PORT "SD_CLK"     DRIVE=8 SLEWRATE=FAST;
IOBUF PORT "SD_CMD"     DRIVE=8 SLEWRATE=FAST;
IOBUF GROUP "SD_DAT*"   DRIVE=8 SLEWRATE=FAST;
''' % locates
open(LPF, "w").write(lpf)
print("rewrote LPF (%d LOCATEs)" % len(assign))
