# Rockbox on Sphinxmoth: source patch and automated builds

`pp5002-sphinxmoth.patch` is a three-file change to Rockbox's PP5002 (iPod
1G/2G/3G) ATA code:

- `firmware/target/arm/pp/ata-pp5002.c`: select the IDE controller timing
  from the current CPU clock, using the retail firmware's own values
  (0x10 / 0x80002150 at 30 MHz, 0x11c1 / 0x80003261 with config bit 2 at
  80 MHz), instead of one fixed value.
- `firmware/target/arm/pp/system-pp5002.c`: retune that timing on every
  clock change.
- `firmware/target/arm/pp/ata-target.h`: wait for the controller's idle bit
  after each PIO data word on writes, as the driver already does for
  register writes, so words are not dropped when the bus is slower than the
  CPU's store rate.

Together these let unmodified Rockbox run on boards with the
`v1.1-mg132-fix3b` bridge image, whose read-path latency is more than
Rockbox's stock timing tolerates. Boards on the raw-pad PIO passthrough image
do not need it. `upstream-commit-message.txt` is the message for submitting
the same change to Rockbox; once merged, this directory becomes unnecessary.

## The workflow

`.github/workflows/rockbox.yml` builds Rockbox for `ipod1g2g` and `ipod3g`
with the patch applied:

- **Weekly** (Monday 06:17 UTC): finds the newest upstream release tag
  (`vX.Y` or `vX.Y-final`), and if this repo has no
  `rockbox-<tag>-sphinxmoth` release yet, builds it and publishes one.
- **On demand** (Actions tab, "Run workflow"): builds any Rockbox ref;
  `publish` controls whether a release is created. Building `master`
  publishes a rolling prerelease.

The ARM cross toolchain (`tools/rockboxdev.sh --target=a`, gcc 9.5) is built
once and cached; the first run takes 20 to 30 minutes, later runs a few.

Each release carries `rockbox-ipod1g2g-sphinxmoth.ipod`,
`rockbox-ipod3g-sphinxmoth.ipod`, the full `.zip` install for each, and
`SHA256SUMS.txt`. The `.ipod` file is copied over `.rockbox/rockbox.ipod`
on a matching Rockbox install (same version, or codecs will not load); the
`.zip` is a complete install.

## What the workflow does not do

It does not test anything on hardware. A build that compiles is not a build
that works: verify each published build on a fix3b board (boot, a write,
playback) before pointing customers at it. The tested, hand-patched 4.0
binaries in `releases/rockbox-4.0-ipod1g2g/` and `releases/rockbox-4.0-ipod3g/`
remain the known-good downloads until a workflow build has been through that
check.

If upstream changes the driver so the patch no longer applies, the workflow
fails at the "Apply the Sphinxmoth patch" step; that is the signal to rebase
the patch or, better, to check whether upstream now carries the fix.
