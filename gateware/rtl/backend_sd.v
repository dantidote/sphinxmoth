// ============================================================================
// backend_sd.v  --  blk_* backend policy over sd_host (v2)
// ----------------------------------------------------------------------------
// Sits between ata_device's FIFO face and the sd_host protocol engine.
// Owns everything sd_host refuses to decide:
//
//   READS : ping-pong sector buffers, VERIFY-THEN-RELEASE. A block is copied
//           into the ata FIFO only after all four DAT CRC16s matched, so a
//           flipped bit on the SD bus becomes a bounded re-read (up to 3 per
//           block: CMD12 + reopen CMD18 at the failed LBA), never corrupt
//           data on the ATA bus. Fill(N+1) overlaps drain(N): full rate.
//   WRITES: one private sector buffer. A sector is pulled from the ata FIFO
//           ONLY when a complete one is available, and the SD block streams
//           from the private copy -- a host reset that guts the FIFO can
//           never tear an SD block mid-flight. Rejected CRC token = CMD12,
//           reopen CMD25, resend from the intact buffer (3 tries).
//   FLUSH : trivially honest -- every accepted block already waited out the
//           card's program busy, so flush = wait for op idle.
//   ABORT : close the SD op cleanly (finish nothing new, CMD12, busy out),
//           swallow the in-flight blk op without a done pulse.
//
// Reset domain: POR only (the card must survive host resets); host resets
// arrive as blk_abort from the top level.
// ============================================================================

module backend_sd (
    input             clk,
    input             rst_n,           // POR only

    // ---- blk_* face (toward ata_device) -------------------------------------
    input             blk_req,         // pulse: new operation
    input             blk_write,
    input      [31:0] blk_lba,
    input      [16:0] blk_nsec,        // 1..65536
    output reg        blk_busy,
    output reg        blk_done,        // pulse
    output reg        blk_err,         // valid with blk_done
    input             blk_flush,       // pulse
    output reg        blk_flush_done,  // pulse
    input             blk_abort,       // level: host reset / SRST / watchdog
    output            blk_ready,
    output     [31:0] blk_capacity,

    // ---- ata FIFO face -------------------------------------------------------
    output reg        brd_wr,          // read dir: we produce into the ata FIFO
    output reg [15:0] brd_data,
    input             brd_full,        // ata FIFO full: pause word-by-word
    input             bwr_avail,       // write dir: >=1 full sector is waiting
    output reg        bwr_rd,          // FWFT consume
    input      [15:0] bwr_data,

    // ---- sd_host -------------------------------------------------------------
    input             sd_init_done,
    input      [31:0] sd_capacity,
    output reg        sd_op_go,
    output reg        sd_op_write,
    output reg [31:0] sd_op_lba,
    input             sd_op_open,
    output reg        sd_blk_go,
    input             sd_blk_done,
    input             sd_blk_crc_ok,
    input             sd_wr_acc,
    output reg        sd_op_end,
    input             sd_op_idle,
    input             sd_op_err,
    input             sd_rd_v,
    input      [15:0] sd_rd_w,
    input      [7:0]  sd_wr_idx,
    output reg [15:0] sd_wr_word,

    // ---- debug ---------------------------------------------------------------
    output reg        dbg_retry_ev,    // 1-cycle: a block was re-tried
    output reg [7:0]  dbg_retries     // total retries since POR (saturating)
);

    assign blk_ready    = sd_init_done;
    assign blk_capacity = sd_capacity;

    // ------------------------------------------------------------------------
    // buffers: read ping-pong = 512x16 (one EBR), write sector = 256x16
    // ------------------------------------------------------------------------
    reg [15:0] rbuf [0:511];
    reg [15:0] wbuf [0:255];
    reg        rhalf_full [0:1];
    reg        fill_sel, drain_sel;
    reg [8:0]  fill_cnt;               // words landed in the filling half
    reg [8:0]  drain_cnt;

    // sd_host write feed: registered read of the private buffer
    always @(posedge clk) sd_wr_word <= wbuf[sd_wr_idx];

    // ------------------------------------------------------------------------
    // main FSM
    // ------------------------------------------------------------------------
    localparam B_IDLE    = 4'd0,
               B_RD_OPEN = 4'd1,  B_RD_RUN  = 4'd2,  B_RD_REOPEN = 4'd3,
               B_WR_OPEN = 4'd4,  B_WR_PULL = 4'd5,  B_WR_BLK    = 4'd6,
               B_WR_REOPEN = 4'd7,
               B_CLOSE   = 4'd8,                     // CMD12, then done/err
               B_ABORT   = 4'd9;                     // close, no done pulse
    reg [3:0]  b;

    reg        wr_l;                   // latched op direction
    reg [31:0] lba_l;                  // op start LBA
    reg [16:0] nsec_l;
    reg [16:0] sd_seen;                // blocks sd_host finished (read: into rbuf)
    reg [16:0] released;               // read: blocks drained to the FIFO
    reg [16:0] wr_sent;                // write: blocks accepted by the card
    reg [1:0]  tries;
    reg        rd_inflight;            // a DE block is running
    reg        end_err;                // close with error
    reg        drain_run;
    reg [8:0]  pull_cnt;
    reg        pull_ph;                // FWFT cadence: capture / let it advance
    reg        req_pend;               // blk_req landed while we were closing
    reg        flush_pend;

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b <= B_IDLE;
            blk_busy <= 0; blk_done <= 0; blk_err <= 0; blk_flush_done <= 0;
            brd_wr <= 0; brd_data <= 16'h0; bwr_rd <= 0;
            sd_op_go <= 0; sd_op_write <= 0; sd_op_lba <= 32'h0;
            sd_blk_go <= 0; sd_op_end <= 0;
            fill_sel <= 0; drain_sel <= 0; fill_cnt <= 9'd0; drain_cnt <= 9'd0;
            rhalf_full[0] <= 0; rhalf_full[1] <= 0;
            wr_l <= 0; lba_l <= 32'h0; nsec_l <= 17'd0;
            sd_seen <= 17'd0; released <= 17'd0; wr_sent <= 17'd0;
            tries <= 2'd0; rd_inflight <= 0; end_err <= 0; drain_run <= 0;
            pull_cnt <= 9'd0; pull_ph <= 0; req_pend <= 0; flush_pend <= 0;
            dbg_retry_ev <= 0; dbg_retries <= 8'd0;
        end else begin
            blk_done <= 1'b0; blk_flush_done <= 1'b0;
            sd_op_go <= 1'b0; sd_blk_go <= 1'b0; sd_op_end <= 1'b0;
            brd_wr <= 1'b0; bwr_rd <= 1'b0;
            dbg_retry_ev <= 1'b0;

            // pulses land whenever the ata side pleases; latch them
            if (blk_req)   req_pend   <= 1'b1;
            if (blk_flush) flush_pend <= 1'b1;
            if (blk_abort) req_pend   <= 1'b0;           // aborted before service

            // ---------------- read fill: capture the SD word stream ----------
            if (sd_rd_v && rd_inflight) begin
                rbuf[{fill_sel, fill_cnt[7:0]}] <= sd_rd_w;
                fill_cnt <= fill_cnt + 9'd1;
            end

            // ---------------- read drain: verified half -> ata FIFO ----------
            if (end_err || blk_abort) begin
                drain_run <= 1'b0;                 // never feed a dead command
            end else if (drain_run) begin
                if (!brd_full) begin
                    brd_data  <= rbuf[{drain_sel, drain_cnt[7:0]}];
                    brd_wr    <= 1'b1;
                    if (drain_cnt == 9'd255) begin
                        drain_run           <= 1'b0;
                        rhalf_full[drain_sel] <= 1'b0;
                        drain_sel           <= ~drain_sel;
                        released            <= released + 17'd1;
                        drain_cnt           <= 9'd0;
                    end else
                        drain_cnt <= drain_cnt + 9'd1;
                end
            end else if (rhalf_full[drain_sel] && b != B_IDLE && !wr_l) begin
                drain_run <= 1'b1;
                drain_cnt <= 9'd0;
            end

            // ---------------- op FSM ----------------------------------------
            case (b)
            B_IDLE: begin
                blk_busy <= 1'b0;
                rhalf_full[0] <= 1'b0; rhalf_full[1] <= 1'b0;
                drain_run <= 1'b0; rd_inflight <= 1'b0; end_err <= 1'b0;
                if (flush_pend) begin                        // writes are through
                    blk_flush_done <= 1'b1;
                    flush_pend     <= 1'b0;
                end
                if (req_pend && sd_init_done && !blk_abort) begin
                    req_pend <= 1'b0;
                    blk_busy <= 1'b1;
                    wr_l     <= blk_write;
                    lba_l    <= blk_lba;
                    nsec_l   <= blk_nsec;
                    sd_seen  <= 17'd0; released <= 17'd0; wr_sent <= 17'd0;
                    tries    <= 2'd0;
                    fill_sel <= 1'b0; drain_sel <= 1'b0; fill_cnt <= 9'd0;
                    sd_op_write <= blk_write;
                    sd_op_lba   <= blk_lba;
                    sd_op_go    <= 1'b1;
                    b <= blk_write ? B_WR_OPEN : B_RD_OPEN;
                end
            end
            // ================= READ =================
            B_RD_OPEN:
                if (blk_abort)      begin end_err <= 1'b1; b <= B_ABORT; end
                else if (sd_op_err) begin end_err <= 1'b1; b <= B_CLOSE; end
                else if (sd_op_open) begin
                    fill_cnt   <= 9'd0;
                    sd_blk_go  <= 1'b1;                      // block 0
                    rd_inflight <= 1'b1;
                    b <= B_RD_RUN;
                end
            B_RD_RUN: begin
                if (blk_abort) begin end_err <= 1'b1; b <= B_ABORT; end
                else begin
                    if (sd_blk_done) begin
                        rd_inflight <= 1'b0;
                        if (sd_blk_crc_ok) begin
                            rhalf_full[fill_sel] <= 1'b1;
                            fill_sel <= ~fill_sel;
                            fill_cnt <= 9'd0;
                            sd_seen  <= sd_seen + 17'd1;
                            tries    <= 2'd0;
                        end else begin
                            // bad block: close and re-read it
                            dbg_retry_ev <= 1'b1;
                            if (dbg_retries != 8'hFF) dbg_retries <= dbg_retries + 8'd1;
                            if (tries == 2'd3) begin end_err <= 1'b1; b <= B_CLOSE; end
                            else begin
                                tries <= tries + 2'd1;
                                sd_op_end <= 1'b1;
                                b <= B_RD_REOPEN;
                            end
                        end
                    end
                    // launch the next fill when its half is free
                    if (!rd_inflight && !sd_blk_done && b == B_RD_RUN
                        && sd_seen != nsec_l && !rhalf_full[fill_sel]) begin
                        fill_cnt   <= 9'd0;
                        sd_blk_go  <= 1'b1;
                        rd_inflight <= 1'b1;
                    end
                    // all blocks fetched AND drained -> close
                    if (sd_seen == nsec_l && released == nsec_l && !drain_run) begin
                        sd_op_end <= 1'b1;
                        b <= B_CLOSE;
                    end
                end
            end
            B_RD_REOPEN:                                     // reopen at the bad LBA
                if (blk_abort) begin end_err <= 1'b1; b <= B_ABORT; end
                else if (sd_op_idle) begin
                    sd_op_write <= 1'b0;
                    sd_op_lba   <= lba_l + {15'b0, sd_seen};
                    sd_op_go    <= 1'b1;
                    b <= B_RD_OPEN;
                end
            // ================= WRITE =================
            B_WR_OPEN:
                if (blk_abort)      begin end_err <= 1'b1; b <= B_ABORT; end
                else if (sd_op_err) begin end_err <= 1'b1; b <= B_CLOSE; end
                else if (sd_op_open) begin
                    pull_cnt <= 9'd0;
                    b <= B_WR_PULL;
                end
            B_WR_PULL:                                       // ata FIFO -> private buffer
                if (blk_abort) begin end_err <= 1'b1; b <= B_ABORT; end
                else if (tries != 2'd0) begin                // resend: buffer already loaded
                    sd_blk_go <= 1'b1;
                    b <= B_WR_BLK;
                end else if (pull_cnt == 9'd256) begin
                    sd_blk_go <= 1'b1;
                    pull_ph   <= 1'b0;
                    b <= B_WR_BLK;
                end else if (pull_ph) begin
                    pull_ph <= 1'b0;                         // FWFT head advances now
                end else if (bwr_avail || pull_cnt != 9'd0) begin
                    // once a sector is promised (bwr_avail), all 256 words exist
                    wbuf[pull_cnt[7:0]] <= bwr_data;
                    bwr_rd   <= 1'b1;
                    pull_cnt <= pull_cnt + 9'd1;
                    pull_ph  <= 1'b1;
                end
            B_WR_BLK:
                if (sd_blk_done) begin
                    if (sd_wr_acc) begin
                        wr_sent <= wr_sent + 17'd1;
                        tries   <= 2'd0;
                        if (wr_sent + 17'd1 == nsec_l) begin
                            sd_op_end <= 1'b1;
                            b <= B_CLOSE;
                        end else begin
                            pull_cnt <= 9'd0;
                            b <= B_WR_PULL;
                        end
                    end else begin
                        dbg_retry_ev <= 1'b1;
                        if (dbg_retries != 8'hFF) dbg_retries <= dbg_retries + 8'd1;
                        if (tries == 2'd3) begin end_err <= 1'b1; b <= B_CLOSE; end
                        else begin
                            tries <= tries + 2'd1;
                            sd_op_end <= 1'b1;
                            b <= B_WR_REOPEN;
                        end
                    end
                end else if (blk_abort) begin
                    // let the in-flight block finish from the private buffer
                    // (it cannot depend on the FIFO), then close
                    end_err <= 1'b1;
                end
            B_WR_REOPEN:
                if (blk_abort) begin end_err <= 1'b1; b <= B_ABORT; end
                else if (sd_op_idle) begin
                    sd_op_write <= 1'b1;
                    sd_op_lba   <= lba_l + {15'b0, wr_sent};
                    sd_op_go    <= 1'b1;
                    b <= B_WR_OPEN;
                end
            // ================= CLOSE / ABORT =================
            B_CLOSE:
                if (sd_op_idle) begin
                    blk_done <= 1'b1;
                    blk_err  <= end_err;
                    b <= B_IDLE;
                end
            B_ABORT: begin
                sd_op_end <= 1'b1;                           // idempotent-ish: sd_host
                if (sd_op_idle) b <= B_IDLE;                 // latches end_pend
            end
            default: b <= B_IDLE;
            endcase

            // global abort trump: any state, once idle work is closed out
            if (blk_abort && b != B_IDLE && b != B_ABORT && b != B_WR_BLK
                && b != B_CLOSE)
                b <= B_ABORT;
        end
    end

endmodule
