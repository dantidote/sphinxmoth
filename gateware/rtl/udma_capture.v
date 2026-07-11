// ============================================================================
// udma_capture.v  --  source-synchronous Ultra-DMA word de-multiplexer
// ----------------------------------------------------------------------------
// Ultra-DMA is double-data-rate: one 16-bit word is valid on EACH edge of
// STROBE (pin 27 family: -DSTROBE on reads, HSTROBE on writes). This module
// turns that DDR wire stream into a single-rate word stream in the system
// clock domain: one `word`+`word_valid` pulse per captured edge, which is
// exactly what crc16_udma and sync_fifo consume.
//
// IMPLEMENTATION NOTE (skeleton): this uses oversampling edge-detection on a
// fast `clk` (>= ~8x the UDMA2 strobe, i.e. >= ~120 MHz). At UDMA2 the strobe
// period is ~120 ns so this has margin, but for production on MachXO2 you would
// normally capture with the strobe itself via the input DDR / DQS primitives
// (IDDRXD) and cross into `clk` through the FIFO, rather than oversample. The
// CRC/FIFO interface below is identical either way, so swapping the front end
// does not disturb the rest of the design.
// ============================================================================

module udma_capture (
    input             clk,         // fast system clock
    input             rst_n,
    input             enable,      // high while a burst is active (gated by DMACK)
    input             strobe,      // raw UDMA strobe (already sync'd to clk)
    input      [15:0] dd_in,       // raw DD[15:0] (already sync'd to clk)
    output reg [15:0] word,        // captured word
    output reg        word_valid   // 1-clk pulse per captured edge
);

    reg [2:0] strobe_q;            // 3-deep for sync + edge detect
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) strobe_q <= 3'b000;
        else        strobe_q <= {strobe_q[1:0], strobe};
    end

    // delay-match DD to the strobe pipeline: the edge is detected 3 FFs after
    // the pad, but dd_in is only 1 FF deep -- without these two stages the
    // captured data is from ~2 clocks AFTER the edge (the NEXT word).
    reg [15:0] dd_q1, dd_q2, dd_q3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin dd_q1 <= 16'h0; dd_q2 <= 16'h0; dd_q3 <= 16'h0; end
        else        begin dd_q1 <= dd_in; dd_q2 <= dd_q1; dd_q3 <= dd_q2; end
    end

    wire edge_rise = (strobe_q[2:1] == 2'b01);
    wire edge_fall = (strobe_q[2:1] == 2'b10);

    // dead-time filter: real UDMA2 edges are >=~55ns apart; transitions within
    // 2 clks (30ns) of an accepted edge are ringing/runts, NOT data. (The
    // heavier debounce variant added a clock of latency and broke proven
    // UDMA2 traffic -- this mild filter is the keeper.)
    reg [1:0] dead;
    wire any_edge  = enable & (edge_rise | edge_fall) & (dead == 2'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            word       <= 16'h0000;
            word_valid <= 1'b0;
            dead       <= 2'd0;
        end else if (any_edge) begin
            // sample from just BEFORE the edge: UDMA data has ~70ns setup but
            // only ~6ns hold, so the pre-edge sample is the only safe one
            word       <= dd_q3;
            word_valid <= 1'b1;
            dead       <= 2'd2;
        end else begin
            word_valid <= 1'b0;
            if (dead != 2'd0) dead <= dead - 2'd1;
        end
    end

endmodule
