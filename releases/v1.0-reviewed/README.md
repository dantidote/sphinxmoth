# v1.0-reviewed

Gateware bitstream built 2026-07-18 21:56, after the RTL review.

| | |
|---|---|
| device | **LCMXO2-2000HC-4TQFP100** |
| file | `ipodboard_impl1.jed`, 501,447 bytes |
| md5 | `de032270081ca885...` |
| source | `gateware/rtl/`, constraints `diamond/ipodboard.lpf` |
| rebuild | `diamond/rebuild.tcl` |

## This is a TQFP-100 image

It fits the **v1** board only. A JEDEC file is device and package specific, so
this will not program the v1.1 csBGA-132 board. There is currently no MG132
bitstream in the repository; v1.1 needs a fresh build against
`diamond/ipodboard-mg132.lpf` with the project device set to
`LCMXO2-2000HC-4MG132C`.

## Relationship to v1.0-first-music

`v1.0-first-music` is the image that first played music and is a different
build (md5 `000f28fa...`). This one is newer and carries the fixes made after
it, including the SET MULTIPLE restore after re-init, the stale-word fix in the
UDMA paths, and the debug-snoop correction.

Intermediate builds from the same period exist outside version control:
`multifix`, `noshim` and `debugless`. Only this one is archived, because it is
the reviewed end state and the one matching `diamond/impl1/`.
