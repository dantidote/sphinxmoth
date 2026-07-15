#!/bin/bash
# finalize_v2.sh -- fab outputs for board-v2 (run from board-v2/, bash)
set -e
KCLI="/c/Program Files/KiCad/10.0/bin/kicad-cli.exe"
PCB=ipod-sd-udma.kicad_pcb

mkdir -p fab
"$KCLI" pcb export gerbers --output fab/ \
    --layers F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts \
    --subtract-soldermask "$PCB"
"$KCLI" pcb export drill --output fab/ --format excellon --drill-origin absolute "$PCB"
"$KCLI" pcb export pos --output fab/ipod-sd-udma-pos.csv --format csv --units mm "$PCB"
(cd fab && rm -f ../ipod-sd-udma-gerbers.zip && "/c/Program Files/7-Zip/7z.exe" a ../ipod-sd-udma-gerbers.zip . >/dev/null 2>&1 || zip -qr ../ipod-sd-udma-gerbers.zip .)
"$KCLI" pcb render --output board-top.png --side top --width 1600 --height 1600 "$PCB"
"$KCLI" pcb render --output board-bottom.png --side bottom --width 1600 --height 1600 "$PCB"
echo "fab outputs + renders done"
