// ============================================================================
// udma_device.v  --  iSphynx-facing Ultra-DMA mode 2 DEVICE engine
// ----------------------------------------------------------------------------
// Presents the bridge to the iSphynxII as a normal UDMA2 device and shuttles
// the iSphynx leg into/out of the FIFO. The data-phase timing is standard UDMA2
// (we generate DSTROBE when the iSphynx reads us). The ONE thing that is not
// spec-clean is the iSphynx's burst TERMINATION (it omits/garbles the CRC) --
// that handling is isolated to S_TERM and marked TODO_CAP; everything else is
// closed.
//
//   dir_write=1  iSphynx WRITES to CF : iSphynx drives HSTROBE+data, we capture
//                into the FIFO and IGNORE its end-of-burst CRC (never ICRC).
//   dir_write=0  iSphynx READS  from CF: we source FIFO words, drive DD+DSTROBE.
//                The iSphynx does not check a returning CRC, so the CRC we send
//                is cosmetic.
// ============================================================================

`include "udma2_timing.vh"

module udma_device #(
    parameter CLK_MHZ = 150
) (
    input             clk,
    input             rst_n,

    input             go,
    input             dir_write,
    output reg        busy,
    output reg        done,

    // FIFO
    output reg        fifo_wr,
    output reg [15:0] fifo_wr_data,
    input             fifo_full,
    input             fifo_afull,      // almost-full: pause the iSphynx (write dir)
    output reg        fifo_rd,
    input      [15:0] fifo_rd_data,
    input             fifo_empty,

    // iSphynx-side control
    output reg        hs_dmarq,
    input             hs_dmack_n,
    input             hs_stop,
    output reg        hs_ddmardy_n,   // our device-ready (write) / DSTROBE (read)
    input             hs_hstrobe,     // iSphynx HSTROBE (write) / HDMARDY (read)

    // iSphynx-side DD (split)
    input      [15:0] hs_dd_in,
    output reg [15:0] hs_dd_out,
    output reg        hs_dd_oe,

    input             hs_strobe_in    // captured HSTROBE (write dir), pre-sync'd
);

    localparam T_WORD = `NS2T(`UDMA2_TWORD_NS);
    localparam T_ACK  = `NS2T(`UDMA2_TACK_NS);
    localparam T_ENV  = `NS2T(`UDMA2_TENV_NS);
    localparam T_MLI  = `NS2T(`UDMA2_TMLI_NS);

    // ---- CRC core (read-dir: running CRC over words sourced to the host) ----
    // MSB_FIRST(0) = canonical UDMA fold (LSB-first); see udma_host note.
    reg         crc_seed, crc_dvalid;
    reg  [15:0] crc_data;
    wire [15:0] crc_value;
    crc16_udma #(.MSB_FIRST(0)) u_crc (
        .clk(clk), .rst_n(rst_n),
        .seed_load(crc_seed), .data_valid(crc_dvalid),
        .data(crc_data), .crc(crc_value)
    );

    // ---- write-direction capture (iSphynx -> us) ---------------------------
    wire [15:0] cap_word;
    wire        cap_valid;
    // strobes are data ONLY inside the grant (parked-host status polls also
    // ride the IOR line), but the 3-FF sync means the final edge is still in
    // flight at DMACK negation -- keep capture alive 3 clks past the gate.
    wire cap_gate = busy & dir_write & ~hs_dmack_n;
    reg [2:0] cap_gate_q;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) cap_gate_q <= 3'b000;
        else        cap_gate_q <= {cap_gate_q[1:0], cap_gate};
    udma_capture u_cap (
        .clk(clk), .rst_n(rst_n),
        .enable(cap_gate | (|cap_gate_q)),
        .strobe(hs_strobe_in), .dd_in(hs_dd_in),
        .word(cap_word), .word_valid(cap_valid)
    );

    localparam S_IDLE   = 4'd0,
               S_ARB    = 4'd1,
               S_ENV    = 4'd2,
               S_WR     = 4'd3,   // iSphynx writing -> capture into FIFO
               S_RD_PRM = 4'd4,   // iSphynx reading -> prime first word
               S_RD     = 4'd5,   // drive data + DSTROBE
               S_TERM   = 4'd6,   // <-- iSphynx-specific (TODO_CAP)
               S_DONE   = 4'd7,
               S_WR_DRAIN=4'd8,   // flush capture pipeline after iSphynx STOP
               S_FIN    = 4'd9;   // DSTROBE return AFTER DMARQ negation (spec order)

    reg [3:0]  st;
    reg [15:0] tmr;
    reg [15:0] cur;
    reg        ld_r;         // staged word on DD not yet strobed (read dir)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 0; done <= 0;
            hs_dmarq <= 0; hs_ddmardy_n <= 1'b1;
            hs_dd_oe <= 0; hs_dd_out <= 0;
            fifo_wr <= 0; fifo_wr_data <= 0; fifo_rd <= 0;
            crc_seed <= 0; crc_dvalid <= 0; crc_data <= 0;
            tmr <= 0; cur <= 0; ld_r <= 0;
        end else begin
            done <= 0; fifo_wr <= 0; fifo_rd <= 0;
            crc_seed <= 0; crc_dvalid <= 0;

            case (st)
            S_IDLE: begin
                busy <= 0; hs_dd_oe <= 0; hs_dmarq <= 0;
                if (go) begin
                    busy     <= 1'b1;
                    hs_dmarq <= 1'b1;            // request DMA from iSphynx
                    crc_seed <= 1'b1;
                    st       <= S_ARB;
                end
            end
            S_ARB: begin                          // wait DMACK + tACK setup
                if (~hs_dmack_n) begin
                    tmr <= T_ENV;
                    st  <= S_ENV;
                end
            end
            S_ENV: begin
                if (tmr != 0) tmr <= tmr - 1'b1;
                else if (dir_write) begin
                    hs_ddmardy_n <= 1'b0;        // we're ready to receive
                    hs_dd_oe     <= 1'b0;        // iSphynx drives DD
                    st <= S_WR;
                end else begin
                    hs_dd_oe <= 1'b1;            // we drive DD to the iSphynx
                    st <= S_RD_PRM;
                end
            end
            // ============ iSphynx WRITES -> we capture =======================
            S_WR: begin
                if (cap_valid && !fifo_full) begin
                    fifo_wr      <= 1'b1;
                    fifo_wr_data <= cap_word;
                end
                hs_ddmardy_n <= fifo_afull;      // flow control: pause iSphynx when full
                if (hs_stop || hs_dmack_n) begin // iSphynx ends the burst
                    hs_ddmardy_n <= 1'b1;
                    tmr <= 16'd6;                // drain the capture pipeline first
                    st  <= S_WR_DRAIN;
                end
            end
            S_WR_DRAIN: begin                     // grab the trailing captured word(s)
                if (cap_valid && !fifo_full) begin
                    fifo_wr      <= 1'b1;
                    fifo_wr_data <= cap_word;
                end
                if (tmr != 0) tmr <= tmr - 1'b1;
                else st <= S_TERM;
            end
            // ============ HS-side host READS -> we source ====================
            // Staged-load discipline (same as the CF write leg): the word on DD
            // is strobed only after T_WORD of setup; the NEXT word is staged no
            // sooner than 2 ticks AFTER the edge (tDH). The iSphynx's input
            // stage tolerated same-edge data swaps; the PP5002's does not.
            S_RD_PRM: begin
                if (ld_r) begin                   // carry-over from prior chunk
                    hs_dd_out <= cur;
                    tmr <= T_WORD;
                    st  <= S_RD;
                end else if (!fifo_empty) begin
                    fifo_rd  <= 1'b1;
                    cur      <= fifo_rd_data;
                    hs_dd_out<= fifo_rd_data;
                    ld_r     <= 1'b1;
                    tmr      <= T_WORD;
                    st       <= S_RD;
                end else st <= S_TERM;
            end
            S_RD: begin
                hs_dd_oe <= 1'b1;
                if (hs_dmack_n) begin            // host aborted/closed the burst
                    st <= S_TERM;
                end else if (tmr != 0) begin
                    tmr <= tmr - 1'b1;
                    // tDH elapsed (>=2 ticks past the edge): stage next word
                    if (!ld_r && (tmr <= T_WORD-2) && !fifo_empty) begin
                        fifo_rd   <= 1'b1;
                        cur       <= fifo_rd_data;
                        hs_dd_out <= fifo_rd_data;
                        ld_r      <= 1'b1;
                        if (tmr < 16'd2) tmr <= 16'd2;   // late stage: full tDS
                    end
                end else if (hs_hstrobe) begin
                    // HDMARDY# (on the DIOR# line) negated = host pause request:
                    // hold data and DSTROBE until the host is ready again
                end else if (!ld_r) begin
                    st <= S_TERM;                 // FIFO dry: end chunk, re-arm later
                end else begin
                    hs_ddmardy_n <= ~hs_ddmardy_n; // DSTROBE edge (on IORDY line)
                    crc_data     <= cur;
                    crc_dvalid   <= 1'b1;
                    ld_r         <= 1'b0;
                    tmr          <= T_WORD;
                end
            end
            // ============ termination (iSphynx quirk lives here) =============
            S_TERM: begin
                // TODO_CAP: from the capture, set EXACTLY how the iSphynx ends a
                // burst and whether it drives a CRC word here:
                //   write-dir: the iSphynx may drive a (garbage) CRC for ~1 word
                //     time after STOP. We must hold off / tristate and simply
                //     NOT compare it. Add a `T_CRC` swallow wait here if it does.
                //   read-dir : optionally drive crc_value for one word (the
                //     iSphynx ignores it). Harmless either way.
                // Until the capture, fall straight through -- functionally safe
                // because we never assert ICRC regardless.
                tmr <= T_MLI;
                st  <= S_DONE;
            end
            S_DONE: begin
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    hs_dmarq <= 1'b0;             // FIRST: end the burst
                    hs_dd_oe <= 1'b0;
                    tmr <= T_MLI;
                    st  <= S_FIN;
                end
            end
            S_FIN: begin
                // spec: DSTROBE returns to idle AFTER DMARQ negation -- a return
                // edge while DMARQ is high reads as one phantom data word to a
                // strict host (odd-length chunks park DSTROBE low)
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    hs_ddmardy_n <= 1'b1;
                    done         <= 1'b1;
                    busy         <= 1'b0;
                    st           <= S_IDLE;
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
