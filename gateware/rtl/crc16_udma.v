// ============================================================================
// crc16_udma.v  --  ATA Ultra-DMA CRC-16 (the load-bearing module)
// ----------------------------------------------------------------------------
// The iSphynxII (TSB43AA82) never emits this CRC; the Toshiba HDD tolerated the
// omission, the CF does not -> ICRC -> the no-timeout firmware hangs. This core
// computes the CRC the bridge is missing so the host-side (CF-facing) UDMA2
// engine can insert it at burst termination.
//
//   polynomial : x^16 + x^12 + x^5 + 1   (0x1021, CRC-CCITT)
//   seed       : 0x4ABA                  (ATA Ultra-DMA initial value)
//
// One 16-bit data word is folded into the running CRC per asserted STROBE edge.
// The surrounding capture logic presents exactly one `data_valid` pulse per word
// (it has already de-multiplexed both STROBE edges), so this core is edge-
// agnostic and purely clk-synchronous.
//
//   +-----------------------------------------------------------------------+
//   | !!! BIT/WORD ORDER IS THE #1 FAILURE MODE -- VALIDATE WITH A CAPTURE   |
//   |                                                                        |
//   | The polynomial (0x1021) and seed (0x4ABA) are certain. The order in    |
//   | which the 16 data bits are folded (DD15-first here) is the single      |
//   | thing most likely to be wrong, and this project's entire history says  |
//   | so. Do NOT trust this against the CF until tb proves it reproduces the  |
//   | CRC a known-good UDMA2 drive sends on a real captured burst. The feed   |
//   | order is isolated to crc_next() below and parameterized by MSB_FIRST.   |
//   +-----------------------------------------------------------------------+
// ============================================================================

module crc16_udma #(
    parameter MSB_FIRST = 1   // 1: fold DD15 first (default).  0: fold DD0 first.
                              //    Flip this if capture validation fails; it is
                              //    the only knob that changes the bit order.
) (
    input              clk,
    input              rst_n,       // async, active-low
    input              seed_load,   // pulse: (re)load 0x4ABA at burst start
    input              data_valid,  // one data word valid this cycle
    input      [15:0]  data,        // DD[15:0] for this word
    output reg [15:0]  crc          // running CRC; updated the cycle after data_valid
);

    localparam [15:0] POLY = 16'h1021;
    localparam [15:0] SEED = 16'h4ABA;

    // Fold one 16-bit word into the running CRC. Implemented as an unrolled
    // serial CRC-CCITT shift (combinational); synthesises to a flat XOR tree.
    function [15:0] crc_next;
        input [15:0] crc_in;
        input [15:0] d;
        integer i;
        reg [15:0] c;
        reg        fb;
        reg        bit_in;
        begin
            c = crc_in;
            for (i = 0; i < 16; i = i + 1) begin
                bit_in = MSB_FIRST ? d[15 - i] : d[i];
                fb     = c[15] ^ bit_in;
                c      = {c[14:0], 1'b0};
                if (fb) c = c ^ POLY;
            end
            crc_next = c;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            crc <= SEED;
        else if (seed_load)
            crc <= SEED;
        else if (data_valid)
            crc <= crc_next(crc, data);
    end

endmodule
