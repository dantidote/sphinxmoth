// ============================================================================
// tb_crc16_udma.v  --  self-checking testbench for crc16_udma
// ----------------------------------------------------------------------------
// Two things are checked here:
//
//   1. STRUCTURAL: an independent, differently-written reference model of the
//      SAME algorithm is cross-checked against the DUT over random vectors.
//      This catches transcription typos in the RTL XOR tree. If this passes,
//      the RTL faithfully implements "CRC-CCITT(0x1021), seed 0x4ABA, DD15-
//      first" -- whatever that turns out to mean on the wire.
//
//   2. SPEC INTERPRETATION (the part that needs silicon): the bench prints the
//      CRC for a few fixed word sequences. These are the numbers you compare
//      against a logic-analyzer capture of a known-good UDMA2 drive's burst.
//      Drop the captured words into GOLDEN_* below and set GOLDEN_CHECK=1 to
//      turn the capture into a hard pass/fail. Until then the structural check
//      is the only assertion; the printed values are reference output.
//
// Run (Icarus Verilog):
//   iverilog -g2012 -o tb.vvp rtl/crc16_udma.v sim/tb_crc16_udma.v
//   vvp tb.vvp
// ============================================================================

`timescale 1ns/1ps

module tb_crc16_udma;

    localparam MSB_FIRST = 1;          // keep in sync with the DUT instance

    reg         clk = 0;
    reg         rst_n = 0;
    reg         seed_load = 0;
    reg         data_valid = 0;
    reg  [15:0] data = 16'h0000;
    wire [15:0] crc;

    integer     errors = 0;

    crc16_udma #(.MSB_FIRST(MSB_FIRST)) dut (
        .clk(clk), .rst_n(rst_n), .seed_load(seed_load),
        .data_valid(data_valid), .data(data), .crc(crc)
    );

    always #5 clk = ~clk;              // 100 MHz

    // ---- independent reference model (written deliberately differently) ----
    // Software CRC: shift register init to seed, fold each word bit-by-bit.
    function [15:0] ref_fold;
        input [15:0] crc_in;
        input [15:0] word;
        reg   [15:0] r;
        integer      k, idx;
        reg          inbit, topbit;
        begin
            r = crc_in;
            for (k = 0; k < 16; k = k + 1) begin
                idx    = MSB_FIRST ? (15 - k) : k;
                inbit  = word[idx];
                topbit = r[15] ^ inbit;
                r      = r << 1;
                r[0]   = 1'b0;
                if (topbit) r = r ^ 16'h1021;   // taps 0,5,12 == x16+x12+x5+1
            end
            ref_fold = r;
        end
    endfunction

    // Drive one word into the DUT and step a clock.
    task feed_word;
        input [15:0] w;
        begin
            @(negedge clk);
            data       = w;
            data_valid = 1'b1;
            @(negedge clk);
            data_valid = 1'b0;
        end
    endtask

    task load_seed;
        begin
            @(negedge clk);
            seed_load = 1'b1;
            @(negedge clk);
            seed_load = 1'b0;
        end
    endtask

    task check_crc;
        input [8*32-1:0] name;
        input [15:0]     got;
        input [15:0]     exp;
        begin
            if (got !== exp) begin
                $display("FAIL: %0s -> CRC=%04x (expected %04x)", name, got, exp);
                errors = errors + 1;
            end else
                $display("ok  : %0s -> CRC=%04x", name, got);
        end
    endtask

    // ---- structural cross-check: random streams, compare DUT vs ref ----
    integer t, n, w;
    reg [15:0] ref_crc;
    reg [15:0] stream [0:63];

    // ---- captured golden vector (fill from a real UDMA2 burst) ----
    localparam GOLDEN_CHECK = 0;       // set 1 once you have a real capture
    localparam GOLDEN_N     = 4;
    reg [15:0] golden_words [0:3];
    localparam [15:0] GOLDEN_CRC = 16'h0000;   // <- captured end-of-burst CRC

    initial begin
        // ---------------------------------------------------------------
        // reset
        rst_n = 0; #20; rst_n = 1; #20;
        if (crc !== 16'h4ABA) begin
            $display("FAIL: seed after reset = %04x (expected 4ABA)", crc);
            errors = errors + 1;
        end else
            $display("ok  : seed after reset = 4ABA");

        // ---------------------------------------------------------------
        // structural cross-check over many random streams
        for (t = 0; t < 2000; t = t + 1) begin
            n = (t % 32) + 1;                 // 1..32 words
            load_seed;
            ref_crc = 16'h4ABA;
            for (w = 0; w < n; w = w + 1) begin
                stream[w] = $random;
                ref_crc   = ref_fold(ref_crc, stream[w]);
                feed_word(stream[w]);
            end
            @(negedge clk);                   // let final update settle
            if (crc !== ref_crc) begin
                $display("FAIL[%0d]: n=%0d  dut=%04x  ref=%04x", t, n, crc, ref_crc);
                errors = errors + 1;
            end
        end
        if (errors == 0)
            $display("ok  : 2000 random streams, DUT == independent reference");

        // ---------------------------------------------------------------
        // fixed-vector checks against the independent Python reference
        // (sim/crc_ref.py). These confirm the RTL implements the algorithm;
        // the bit ORDER itself is silicon-validated via GOLDEN_* below.
        load_seed; feed_word(16'h0000); @(negedge clk);
        check_crc("seed,0x0000",          crc, 16'hE496);
        load_seed; feed_word(16'hFFFF); @(negedge clk);
        check_crc("seed,0xFFFF",          crc, 16'hF999);
        load_seed;
        feed_word(16'h0102); feed_word(16'h0304);
        feed_word(16'h0506); feed_word(16'h0708); @(negedge clk);
        check_crc("seed,0102,0304,0506,0708", crc, 16'h103A);

        // ---------------------------------------------------------------
        // hard golden check (enable once captured)
        if (GOLDEN_CHECK) begin
            golden_words[0]=16'h0000; golden_words[1]=16'h0000;  // <- fill
            golden_words[2]=16'h0000; golden_words[3]=16'h0000;  // <- fill
            load_seed;
            for (w = 0; w < GOLDEN_N; w = w + 1) feed_word(golden_words[w]);
            @(negedge clk);
            if (crc !== GOLDEN_CRC) begin
                $display("FAIL: golden capture dut=%04x expected=%04x", crc, GOLDEN_CRC);
                $display("      -> try flipping MSB_FIRST, then re-check.");
                errors = errors + 1;
            end else
                $display("ok  : GOLDEN capture matches -- bit order CONFIRMED on silicon");
        end else begin
            $display("note: GOLDEN_CHECK disabled -- bit order NOT yet silicon-validated");
        end

        // ---------------------------------------------------------------
        if (errors == 0) $display("\n==== PASS ====");
        else             $display("\n==== %0d FAILURE(S) ====", errors);
        $finish;
    end

endmodule
