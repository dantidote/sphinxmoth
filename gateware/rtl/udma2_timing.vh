// ============================================================================
// udma2_timing.vh  --  Ultra-DMA mode 2 timing, in nanoseconds
// ----------------------------------------------------------------------------
// Included by the UDMA engines. `NS2T converts ns -> system-clock ticks
// (ceiling); the including module must define a CLK_MHZ parameter.
//
// TWORD is certain (it falls straight out of 33.3 MB/s: 2 bytes / 60 ns). The
// rest are the ATA Ultra-DMA "Data Burst Timing Requirements" figures for
// mode 2 -- they are spec values, not iSphynx-specific, so they can be set
// here without the capture. Cross-check exact numbers against the ATA/ATAPI
// timing table for your reference revision; they are deliberately conservative
// (typicals/maxes) so a compliant CF is comfortable.
// ============================================================================

`ifndef UDMA2_TIMING_VH
`define UDMA2_TIMING_VH

// ns -> ticks, rounded up. CLK_MHZ resolves to the including module's parameter.
`define NS2T(ns) ((((ns)*CLK_MHZ)+999)/1000)

`define UDMA2_TWORD_NS  60   // per-word time = STROBE edge spacing (tCYC typ; min 48)
`define UDMA2_TACK_NS   20   // tACK   DMACK setup/hold (min)
`define UDMA2_TENV_NS   35   // tENV   envelope: DMACK -> STOP/strobe (20..70)
`define UDMA2_TMLI_NS   20   // tMLI   interlock minimum
`define UDMA2_TRFS_NS   75   // tRFS   ready -> final STROBE (max)
`define UDMA2_TRP_NS   100   // tRP    ready -> pause (min)
`define UDMA2_TSS_NS    50   // tSS    last STROBE edge -> STOP assertion (min)
`define UDMA2_TCRC_NS   60   // CRC word valid/hold around the final STROBE edge
`define UDMA2_TZAH_NS   20   // bus turnaround (tAZ/tZAH family) before we drive DD

`endif
