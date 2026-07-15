#!/usr/bin/env python
# gen_bom.py -- BOM.csv for board-v2, v1 format (grouped refs, MPNs)
import pcbnew
from collections import defaultdict
import os
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # repo root

MPN = {
    ("C", "100nF"): "CL05B104KO5NNNC",
    ("C", "4.7uF"): "CL10A475KO8NNNC",
    ("C", "10uF"): "C2012X5R1E106K125AB",
    ("C", "1uF"): "CL05A105KA5NNNC",
    ("U", "LCMXO2-7000HC-4TG144C"): "LCMXO2-7000HC-4TG144C",
    ("J", "iPod_1.8in_50pin"): "SPECIALTY-iPod-1.8in-50pin",
    ("J", "microSD DM3AT-SF-PEJM5"): "DM3AT-SF-PEJM5",
    ("J", "UART"): "20021311-00004T4LF",
}

b = pcbnew.LoadBoard(os.path.join(REPO, r"board-v2\ipod-sd-udma.kicad_pcb"))
groups = defaultdict(list)
for fp in b.GetFootprints():
    ref = fp.GetReference()
    if ref.startswith("J3") or ref.startswith("J6"):   # pad-only / shunt
        pass
    val = fp.GetValue()
    fpname = str(fp.GetFPID().GetLibItemName())
    lib = str(fp.GetFPID().GetLibNickname())
    groups[(val, (lib + ":" + fpname) if lib else fpname)].append(ref)

def refrange(refs):
    import re
    def key(r):
        m = re.match(r"([A-Z]+)(\d+)", r)
        return (m.group(1), int(m.group(2)))
    refs = sorted(refs, key=key)
    out, run = [], [refs[0]]
    for r in refs[1:]:
        p, c = key(run[-1]), key(r)
        if p[0] == c[0] and c[1] == p[1] + 1:
            run.append(r)
        else:
            out.append(run); run = [r]
    out.append(run)
    return ",".join(("%s-%s" % (r[0], r[-1])) if len(r) > 2 else ",".join(r)
                    for r in out)

with open(os.path.join(REPO, r"board-v2\BOM.csv"), "w") as f:
    f.write('"Refs","Value","Footprint","MPN","Qty"\n')
    for (val, fpn), refs in sorted(groups.items(), key=lambda kv: kv[1][0]):
        pfx = refs[0][0]
        mpn = MPN.get((pfx, val), "")
        f.write('"%s","%s","%s","%s","%d"\n'
                % (refrange(refs), val, fpn, mpn, len(refs)))
print("BOM.csv written")
