// ============================================================================
// pio_passthru.v  --  transparent DD bridge for all non-DMA (PIO) cycles
// ----------------------------------------------------------------------------
// The PP5002 talks to the CF only in PIO (taskfile registers + PIO data) and
// NEVER asserts DMACK. So whenever we are not in a DMA burst, the data bus is
// simply bridged across the split: on a host read (IOR# low) we forward CF->host
// data; on a host write (IOW# low) we forward host->CF data. All address /
// chip-select / control lines are passed straight through at the top level; only
// DD is steered here because DD is the one bus split through the FPGA.
//
// This keeps every byte of normal iPod operation (boot, playback, IDENTIFY,
// SET FEATURES, taskfile polling) transparent and untouched.
// ============================================================================

module pio_passthru (
    input             active,        // high when NOT in a DMA burst (DMACK# high)

    input             ior_n,         // host read strobe (DIOR#)
    input             iow_n,         // host write strobe (DIOW#)

    // host (iSphynx/PP5002) side DD
    input      [15:0] hs_dd_in,
    output     [15:0] hs_dd_out,
    output            hs_dd_oe,

    // CF side DD
    input      [15:0] cf_dd_in,
    output     [15:0] cf_dd_out,
    output            cf_dd_oe
);

    wire reading = active & ~ior_n;   // CF -> host
    wire writing = active & ~iow_n;   // host -> CF

    // host read: present CF data to the host
    assign hs_dd_out = cf_dd_in;
    assign hs_dd_oe  = reading;

    // host write: present host data to the CF
    assign cf_dd_out = hs_dd_in;
    assign cf_dd_oe  = writing;

endmodule
