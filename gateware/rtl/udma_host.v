// ============================================================================
// udma_host.v  --  CF-facing Ultra-DMA mode 2 HOST engine  (THE CRC FIX)
// ----------------------------------------------------------------------------
// Originates the CF-side UDMA2 burst from/to the FIFO and drives the CF the
// correct end-of-burst CRC the iSphynx omits. The CF is a spec-compliant UDMA2
// device, so this engine's entire protocol is defined by the ATA Ultra-DMA
// timing table (see udma2_timing.vh) -- NONE of it depends on the iSphynx
// quirk, which is why it can be closed without the capture. Remaining work is
// board-level SI tuning, exact tick rounding, and multi-burst/pause (noted).
//
// Direction (data flow on the CF leg):
//   dir_write=1  FireWire WRITE : FIFO -> CF   (we drive DD + HSTROBE)
//   dir_write=0  FireWire READ  : CF   -> FIFO (CF drives DD + DSTROBE)
// In BOTH cases the HOST sends the CRC to the device at termination -> we drive
// crc_value onto cf_dd and clock it with a final strobe edge.
// ============================================================================

`include "udma2_timing.vh"

module udma_host #(
    parameter CLK_MHZ = 150
) (
    input             clk,
    input             rst_n,

    input             go,
    input             cmd_start,      // pulse: new ATA command (resets wsent)
    output            wr_pending,     // staged word not yet strobed (write dir)
    input             dir_write,
    output reg        busy,
    output reg        done,

    // FIFO
    output reg        fifo_rd,
    input      [15:0] fifo_rd_data,
    input             fifo_empty,
    output reg        fifo_wr,
    output reg [15:0] fifo_wr_data,
    input             fifo_full,
    input             fifo_afull,     // read dir: throttle the card via HDMARDY#

    // CF-side control
    input             cf_dmarq,
    output reg        cf_dmack_n,
    output reg        cf_stop,        // STOP line (active high here; invert at pad if needed)
    input             cf_ddmardy_n,   // device ready (write dir flow control), active low
    output reg        cf_hstrobe,     // HSTROBE (write) / HDMARDY (read) on the DIOR# family line

    // CF-side DD (split)
    input      [15:0] cf_dd_in,
    output reg [15:0] cf_dd_out,
    output reg        cf_dd_oe,

    input             cf_strobe_in,   // DSTROBE (read dir), pre-sync'd

    input      [16:0] words_total,    // WRITE: command length; one continuous burst
    output     [15:0] dbg_crc,        // running/final burst CRC
    output reg [15:0] dbg_wcnt,       // words captured this burst (read dir)
    output     [3:0]  dbg_st,         // FSM state (autopsy)
    output     [15:0] dbg_wsent       // words sent to CF this WRITE command
);

    // ---- timing (ticks) ----------------------------------------------------
    localparam T_WORD = `NS2T(`UDMA2_TWORD_NS);
    localparam T_ACK  = `NS2T(`UDMA2_TACK_NS);
    localparam T_ENV  = `NS2T(`UDMA2_TENV_NS);
    localparam T_MLI  = `NS2T(`UDMA2_TMLI_NS);
    localparam T_SS   = `NS2T(`UDMA2_TSS_NS);
    localparam T_RFS  = `NS2T(`UDMA2_TRFS_NS);
    localparam T_CRC  = `NS2T(`UDMA2_TCRC_NS);
    localparam T_ZAH  = `NS2T(`UDMA2_TZAH_NS);

    // ---- CRC core ----------------------------------------------------------
    // MSB_FIRST(0): the dialect hunt's "mode 2" (bit-reverse each word, fold
    // MSB-first) IS an LSB-first fold of the raw word -- canonical UDMA CRC
    // was one parameter away the whole time. The runtime dialect mux is gone.
    reg         crc_seed, crc_dvalid;
    reg  [15:0] crc_data;   // driven by the fold pipeline below
    wire [15:0] crc_value;
    crc16_udma #(.MSB_FIRST(0)) u_crc (
        .clk(clk), .rst_n(rst_n),
        .seed_load(crc_seed), .data_valid(crc_dvalid),
        .data(crc_data), .crc(crc_value)
    );
    assign dbg_crc = crc_value;

    // hold stage: cap_word/cur may advance before the fold consumes them
    reg [15:0] cap_x, cur_x;
    reg        fold_cap, fold_cur;
    always @(posedge clk) begin
        cap_x <= cap_word;
        cur_x <= cur;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_data <= 16'h0; crc_dvalid <= 1'b0;
        end else begin
            crc_dvalid <= fold_cap | fold_cur;
            crc_data   <= fold_cap ? cap_x : cur_x;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      dbg_wcnt <= 16'd0;
        else if (go)     dbg_wcnt <= 16'd0;
        else if (cap_valid) dbg_wcnt <= dbg_wcnt + 16'd1;   // every word the CF strobed
    end

    // ---- read-direction capture (CF -> us) ---------------------------------
    wire [15:0] cap_word;
    wire        cap_valid;
    udma_capture u_cap (
        .clk(clk), .rst_n(rst_n),
        .enable(busy & ~dir_write),
        .strobe(cf_strobe_in), .dd_in(cf_dd_in),
        .word(cap_word), .word_valid(cap_valid)
    );

    // ---- FSM ---------------------------------------------------------------
    localparam S_IDLE    = 4'd0,
               S_ACK     = 4'd1,
               S_ENV     = 4'd2,
               S_WR_PRIME= 4'd3,
               S_WR_DATA = 4'd4,
               S_WR_TERM = 4'd5,
               S_RD_DATA = 4'd6,
               S_RD_TERM = 4'd7,
               S_TURN    = 4'd8,
               S_CRC     = 4'd9,
               S_DONE    = 4'd10,
               S_ACK2    = 4'd11,
               S_DONE2   = 4'd12,
               S_RD_DRAIN= 4'd13,
               S_WR_WAIT = 4'd14;   // write underrun: hold burst open, await FIFO

    reg [3:0]  st;
    assign dbg_st = st;
    reg [15:0] tmr;          // tick countdown
    reg [15:0] cur;          // word currently presented (write dir)
    reg        have;         // cur holds a valid word
    reg        ld;           // staged word on DD not yet strobed (write dir)
    assign wr_pending = ld;
    reg [16:0] wsent;        // words sent to CF this write command
    assign dbg_wsent = wsent[15:0];
    reg        dmarq_fresh;  // DMARQ has been LOW since the last grant

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 0; done <= 0;
            cf_dmack_n <= 1'b1; cf_stop <= 1'b1; cf_hstrobe <= 1'b1;   // DIOR# family idles HIGH
            cf_dd_oe <= 1'b0; cf_dd_out <= 16'h0;
            fifo_rd <= 0; fifo_wr <= 0; fifo_wr_data <= 0;
            crc_seed <= 0; fold_cap <= 0; fold_cur <= 0;
            tmr <= 0; cur <= 0; have <= 0; ld <= 0; dmarq_fresh <= 1'b1; wsent <= 17'd0;
        end else begin
            done <= 0; fifo_rd <= 0; fifo_wr <= 0;
            crc_seed <= 0; fold_cap <= 0; fold_cur <= 0;
            if (!cf_dmarq) dmarq_fresh <= 1'b1;    // re-arms whenever DMARQ drops
            if (cmd_start) begin
                wsent <= 17'd0;                    // per-COMMAND cumulative count
                dmarq_fresh <= 1'b1;               // new command = new context: a
                                                   // card stranded mid-aborted-cmd
                                                   // holds DMARQ forever otherwise
            end

            case (st)
            // ----------------------------------------------------------------
            S_IDLE: begin
                busy <= 0; cf_dd_oe <= 0; cf_dmack_n <= 1'b1; cf_stop <= 1'b1;
                cf_hstrobe <= 1'b1; have <= 0;
                if (go) begin
                    busy     <= 1'b1;
                    crc_seed <= 1'b1;            // seed 0x4ABA for this burst
                    st       <= S_ACK;
                end
            end
            // ----------------------------------------------------------------
            S_ACK: begin                          // grant only a FRESH DMARQ assertion
                if (cf_dmarq && dmarq_fresh) begin
                    cf_dmack_n  <= 1'b0;
                    dmarq_fresh <= 1'b0;           // consumed
                    tmr <= T_ACK;
                    st  <= S_ACK2;
                end
            end
            S_ACK2: begin                         // DMACK setup time
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    cf_stop <= 1'b0;              // negate STOP -> data may flow
                    tmr     <= T_ENV;
                    st      <= S_ENV;
                end
            end
            // ----------------------------------------------------------------
            S_ENV: begin                          // envelope before first strobe
                if (tmr != 0) tmr <= tmr - 1'b1;
                else if (dir_write) begin
                    cf_dd_oe <= 1'b1;
                    st <= S_WR_PRIME;
                end else begin
                    cf_hstrobe <= 1'b0;          // assert HDMARDY# (ACTIVE LOW) = ready
                    cf_dd_oe   <= 1'b0;          // device drives DD
                    st <= S_RD_DATA;
                end
            end
            // ================= WRITE (FIFO -> CF): multi-burst =================
            // Data/strobe discipline (real cards check setup+hold, sim didn't):
            // the word on DD is strobed only after T_WORD of setup; the NEXT
            // word is staged no sooner than 2 ticks AFTER the edge (tDH), so
            // DD never moves on the edge itself.
            // Sector-buffered cards TERMINATE the burst (drop DMARQ) after each
            // sector; we close (STOP+CRC over words sent), the sequencer
            // re-arms us on their next DMARQ. wsent is cumulative per COMMAND
            // (cmd_start resets it); a staged-unstrobed word carries over (ld).
            S_WR_PRIME: begin                     // stage first word
                if (ld) begin                     // carry-over from prior burst
                    cf_dd_out <= cur;
                    tmr <= T_WORD;
                    st  <= S_WR_DATA;
                end else if (!fifo_empty) begin
                    fifo_rd <= 1'b1;
                    cur     <= fifo_rd_data;
                    cf_dd_out <= fifo_rd_data;
                    ld      <= 1'b1;
                    tmr     <= T_WORD;
                    st      <= S_WR_DATA;
                end else begin
                    st <= S_WR_WAIT;             // no data yet -> hold burst, wait
                end
            end
            S_WR_DATA: begin
                cf_dd_oe <= 1'b1;
                if (!cf_dmarq) begin
                    tmr <= T_SS;                  // device terminated the burst:
                    st  <= S_WR_TERM;             // close with CRC over words sent
                end else if (cf_ddmardy_n) begin
                    // CF not ready -> pause (hold data + strobe)
                end else if (tmr != 0) begin
                    tmr <= tmr - 1'b1;
                    // tDH elapsed (>=2 ticks past the edge): stage next word
                    if (!ld && (tmr <= T_WORD-2) && !fifo_empty) begin
                        fifo_rd   <= 1'b1;
                        cur       <= fifo_rd_data;
                        cf_dd_out <= fifo_rd_data;
                        ld        <= 1'b1;
                        if (tmr < 16'd2) tmr <= 16'd2;   // late stage: full tDS
                    end
                end else if (!ld) begin
                    st <= S_WR_WAIT;              // underrun: hold burst open, wait
                end else begin
                    // strobe EDGE: clocks the staged word into the CF + CRC
                    cf_hstrobe <= ~cf_hstrobe;
                    fold_cur   <= 1'b1;
                    wsent      <= wsent + 17'd1;
                    ld         <= 1'b0;
                    if (wsent + 17'd1 >= words_total) begin
                        tmr <= T_SS;              // hold DD/strobe tSS before STOP
                        st  <= S_WR_TERM;
                    end else begin
                        tmr <= T_WORD;
                    end
                end
            end
            S_WR_WAIT: begin                      // FIFO underran mid-command
                cf_dd_oe <= 1'b1;                 // keep driving DD (host pause)
                if (!cf_dmarq) begin
                    tmr <= T_SS;                  // device terminated: close burst
                    st  <= S_WR_TERM;
                end else if (!fifo_empty) begin
                    fifo_rd   <= 1'b1;
                    cur       <= fifo_rd_data;
                    cf_dd_out <= fifo_rd_data;
                    ld        <= 1'b1;
                    tmr       <= T_WORD;          // full setup before the edge
                    st        <= S_WR_DATA;      // resume
                end
            end
            S_WR_TERM: begin                      // tSS after last edge, then STOP
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    cf_stop <= 1'b1;
                    tmr <= T_MLI;
                    st  <= S_CRC;                  // host already drives DD -> straight to CRC
                end
            end
            // ================= READ (CF -> FIFO) =============================
            S_RD_DATA: begin
                // HDMARDY# flow control: when the consumer lags (slow HS host,
                // grant gaps) the FIFO fills; without this pause the card keeps
                // streaming and words are silently DROPPED at fifo_full ->
                // short burst -> ICRC. afull leaves 512 words for in-flight.
                cf_hstrobe <= fifo_afull;         // low=ready, high=pause
                if (cap_valid && !fifo_full) begin
                    fifo_wr      <= 1'b1;
                    fifo_wr_data <= cap_word;
                    fold_cap     <= 1'b1;
                end
                if (!cf_dmarq) begin              // device ends the burst
                    tmr <= 16'd6;                // FIRST drain the capture pipeline:
                    st  <= S_RD_DRAIN;           // the final word is still in flight
                end
            end
            S_RD_DRAIN: begin                     // grab trailing words after DMARQ drop
                if (cap_valid && !fifo_full) begin
                    fifo_wr      <= 1'b1;
                    fifo_wr_data <= cap_word;
                    fold_cap     <= 1'b1;
                end
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    cf_hstrobe <= 1'b1;          // negate HDMARDY# (high)
                    tmr <= T_RFS;
                    st  <= S_RD_TERM;
                end
            end
            S_RD_TERM: begin                      // wait tRFS, then turn the bus around
                cf_stop <= 1'b1;
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    tmr <= T_ZAH;
                    st  <= S_TURN;
                end
            end
            S_TURN: begin                         // bus turnaround before we drive DD
                cf_dd_oe <= 1'b0;
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    cf_dd_oe  <= 1'b1;
                    cf_dd_out <= crc_value;
                    tmr <= T_MLI;
                    st  <= S_CRC;
                end
            end
            // ================= CRC OUT (both directions) =====================
            S_CRC: begin
                // THE FIX: present our computed CRC; per ATA the device latches
                // it on the NEGATING edge of DMACK# (no strobe edge here!).
                cf_dd_oe  <= 1'b1;
                cf_dd_out <= crc_value;
                if (tmr != 0) begin
                    tmr <= tmr - 1'b1;           // CRC setup before DMACK negation
                end else begin
                    tmr <= T_CRC;
                    st  <= S_DONE;
                end
            end
            S_DONE: begin
                if (tmr != 0) tmr <= tmr - 1'b1;  // hold CRC valid (tCRC)
                else begin
                    cf_dmack_n <= 1'b1;           // <-- this edge latches the CRC
                    tmr <= T_ACK;
                    st  <= S_DONE2;
                end
            end
            S_DONE2: begin                        // hold CRC through DMACK hold time
                if (tmr != 0) tmr <= tmr - 1'b1;
                else begin
                    cf_dd_oe   <= 1'b0;
                    cf_stop    <= 1'b1;
                    cf_hstrobe <= 1'b1;
                    done       <= 1'b1;
                    busy       <= 1'b0;
                    st         <= S_IDLE;
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    // NOTE (multi-burst): a command longer than one burst pauses (DMARQ negate
    // without all sectors done) and resumes. This FSM handles a single burst;
    // the top sequencer should re-arm `go` per burst using the snooped sector
    // count, keeping the FIFO + CRC running across the gap (don't re-seed CRC
    // mid-command -- the CRC spans the whole burst, reseed only at burst start).
    // For UDMA the CRC is per-BURST, so re-seed at each burst start is correct.

endmodule
