# v1.0-reviewed

Gateware bitstream built 2026-07-18 21:56, after the RTL review.

## Provenance

| | |
|---|---|
| device | **LCMXO2-2000HC-4TQFP100** |
| file | `ipodboard_impl1.jed`, 501,447 bytes |
| md5 | `de032270081ca885822dff2d49f05c70` |
| sha256 | `36ef423596756234f5e3773df50c4cd068b5e796a753395a18aed14dd29a16b5` |
| **source commit** | **`f5257df5e41c92ab1ef7da07768f5348f4bd4693`** (`f5257df`, 2026-07-11) |
| RTL tree object | `63c55f91cfcf518eec7d312586dc703629c5ab5a` (`f5257df:gateware/rtl`) |
| constraints | `diamond/ipodboard.lpf`, blob `492b603845768389bb7653aa1acff15848e5ebb9`, 60 LOCATEs |
| project | `diamond/ipodboard.ldf`, device `LCMXO2-2000HC-4TG100C` |
| rebuild | `diamond/rebuild.tcl` |

To reproduce: check out `f5257df`, open `diamond/ipodboard.ldf` in Diamond 3.14
and run `diamond/rebuild.tcl`.

The RTL was verified unchanged between that commit and the build. The source
files carry an 18 July mtime because a checkout rewrote them, but their content
is identical to `f5257df`, so nothing was edited after committing and before
building.

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
