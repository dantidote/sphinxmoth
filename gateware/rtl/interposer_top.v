// ============================================================================
// interposer_top.v  --  iPod 50-pin (iSphynxII / PP5002)  <->  CF interposer
// ----------------------------------------------------------------------------
// Sits at the 1.8" 50-pin drive header, downstream of the iPod's CN211 bus
// switcher, so it transparently serves whichever master the switcher selects:
//
//   * PP5002   : PIO only, never issues a DMA command -> pio_passthru bridges
//                DD, control/IORDY/INTRQ pass through, everything transparent.
//   * iSphynxII: UDMA2 only. cmd_snoop catches the DMA command in the taskfile,
//                the sequencer takes the bus, the two UDMA2 engines store-and-
//                forward through a FIFO, and the CF-facing host engine INSERTS
//                the CRC the iSphynx omits.
//
// SEQUENCING (this build = SEQUENTIAL store-and-forward):
//   READ  : host engine reads the whole CF burst into the FIFO, THEN the device
//           engine drains it to the iSphynx.
//   WRITE : device engine captures the whole iSphynx burst into the FIFO, THEN
//           the host engine drains it to the CF.
//   Sequential needs a FIFO big enough for one burst (hence DEPTH=4096) and adds
//   one-burst latency. The PRODUCTION choice is CONCURRENT (engines pipelined
//   through a small FIFO with peer-done + flow-control); it shrinks the FIFO to
//   ~256 and removes the latency. Sequential is chosen here because it is the
//   simplest thing that is correct end-to-end for first simulation.
//
// PORT DIRECTIONS: re-origination means WE are the device on the iPod side and
// the host on the CF side, so DMARQ and IORDY are DRIVEN toward the iPod (we
// source them), not passed straight through.
// ============================================================================

module interposer_top #(
    parameter CLK_MHZ   = 66,      // system clock; must match the board oscillator/PLL
                                   // 66 = MachXO2 OSCH tap (66.5MHz), closes timing,
                                   // T_WORD=4 ticks=~60ns = still full UDMA2 speed
    parameter FIFO_AW   = 12,      // 4096 words (one burst); see sequencing note
    parameter FIFO_DEPTH= 4096
) (
    // ---- host side: iPod 50-pin (carries iSphynx OR PP5002 via CN211) ------
    inout  [15:0] HS_DD,
    input         HS_CS0_N, HS_CS1_N,
    input         HS_A0, HS_A1, HS_A2,
    input         HS_IOR_N,      // DIOR# / HDMARDY / HSTROBE family
    input         HS_IOW_N,      // DIOW# / STOP
    output        HS_DMARQ,      // we (device) drive DMARQ toward the iPod
    input         HS_DMACK_N,    // iSphynx asserts during UDMA burst
    input         HS_RESET_N,
    output        HS_INTRQ,
    output        HS_IORDY,      // we drive IORDY/-DDMARDY/-DSTROBE toward the iPod

    // ---- CF side (True-IDE) ------------------------------------------------
    inout  [15:0] CF_DD,
    output        CF_CS0_N, CF_CS1_N,
    output        CF_A0, CF_A1, CF_A2,
    output        CF_IOR_N,
    output        CF_IOW_N,
    input         CF_DMARQ,
    output        CF_DMACK_N,
    output        CF_RESET_N,
    input         CF_INTRQ,
    input         CF_IORDY,      // CF drives IORDY/-DDMARDY/-DSTROBE

    // ---- local ----
    input         CLK,
    input         RST_N,

    // ---- debug (to UART logger) ----
    output        DBG_ABORT,     // 1-cycle: watchdog killed a stuck burst
    output        DBG_END,       // 1-cycle: burst completed normally
    output        DBG_START,     // 1-cycle: sequencer engaged a DMA burst
    output        DBG_INIT_OK,   // 1-cycle: cf_init selected UDMA2 on the card
    output        DBG_INIT_BAD,  // 1-cycle: cf_init gave up (timeout)
    output [7:0]  DBG_INIT_STAT, // CF status cf_init last saw
    output [7:0]  DBG_STAT,      // {seq[1:0], dir, host_busy, dev_busy, fifo_empty, fifo_full, 0}
    output [15:0] DBG_CRC,       // CF-leg burst CRC (as sent)
    output [15:0] DBG_WCNT,      // CF-leg words captured last burst
    output        DBG_CHUNK,     // 1-cycle: producer chunk done (multi-burst)
    output [7:0]  DBG_CHUNKV,    // FIFO fill / 32
    output [15:0] DBG_WCAP,      // words captured into FIFO this command
    output [7:0]  DBG_DMACKF,    // HS DMACK falling edges this command (iSphynx burst count)
    output [7:0]  DBG_HOSTQ,    // {host_st, cf_dmarq_seen, cf_rdy_seen, 00}
    output [15:0] DBG_WSENT     // words the host sent to CF (write)
);

    // ------------------------------------------------------------------------
    // input synchronizers
    // ------------------------------------------------------------------------
    reg [15:0] hs_dd_s, cf_dd_s;
    reg        hs_strobe_s, cf_strobe_s;
    reg        cs0_s, cs1_s, a0_s, a1_s, a2_s, iow_s;
    reg  [1:0] cf_intrq_q;
    wire       cf_intrq_s = cf_intrq_q[1];
    // DMARQ/DMACK: plain 2-FF sync, the config every byte of proven traffic
    // ran on. Debouncing either one adds recognition latency that costs real
    // protocol margin (DMACK: loses first write word at min tENV; DMARQ at
    // card-UDMA2: strobes an extra word into a closed burst -> ICRC).
    reg  [1:0] cf_dmarq_q;
    reg        dmack_s;
    wire       cf_dmarq_s = cf_dmarq_q[1];
    always @(posedge CLK) begin
        cf_dmarq_q  <= {cf_dmarq_q[0], CF_DMARQ};
        dmack_s     <= HS_DMACK_N;
        cf_intrq_q  <= {cf_intrq_q[0], CF_INTRQ};
        hs_dd_s     <= HS_DD;
        cf_dd_s     <= CF_DD;
        hs_strobe_s <= HS_IOR_N;     // HSTROBE rides the DIOR# family (write dir)
        cf_strobe_s <= CF_IORDY;     // DSTROBE/DDMARDY ride the IORDY family
        cs0_s <= HS_CS0_N; cs1_s <= HS_CS1_N;
        a0_s  <= HS_A0;    a1_s  <= HS_A1;   a2_s <= HS_A2;
        iow_s <= HS_IOW_N;
    end

    // ------------------------------------------------------------------------
    // post-reset CF mode fixup: select UDMA2 on the card (the iSphynx never
    // does). While active we own the CF bus and answer the host with BSY.
    // ------------------------------------------------------------------------
    wire        inj_active, inj_cs0, inj_ior, inj_iow, inj_dd_oe, inj_rst;
    wire [2:0]  inj_addr;
    wire [15:0] inj_dd;
    cf_init #(.CLK_MHZ(CLK_MHZ)) u_init (
        .clk(CLK), .rst_n(RST_N),
        .active(inj_active), .rst_drive(inj_rst),
        .cs0_n(inj_cs0), .addr(inj_addr),
        .ior_n(inj_ior), .iow_n(inj_iow),
        .dd_out(inj_dd), .dd_oe(inj_dd_oe),
        .dd_in(cf_dd_s),
        .done_ev(DBG_INIT_OK), .fail_ev(DBG_INIT_BAD),
        .last_status(DBG_INIT_STAT)
    );

    // ------------------------------------------------------------------------
    // taskfile write snoop: feat_last feeds the SET FEATURES SC rewrite,
    // sc_last feeds the sequencer's words_total.
    // (The old FC1307A APM fake-success shim lived here; hardware proved the
    // disk-mode app tolerates the S51 abort, so the shim is gone.)
    // ------------------------------------------------------------------------
    wire [2:0] adr_s = {a2_s, a1_s, a0_s};
    reg        iow_q2, ior_q2;
    reg [7:0]  feat_last, sc_last;
    always @(posedge CLK or negedge RST_N) begin
        if (!RST_N) begin
            iow_q2 <= 1'b1; ior_q2 <= 1'b1; feat_last <= 8'h00; sc_last <= 8'h00;
        end else begin
            iow_q2 <= iow_s;
            ior_q2 <= hs_strobe_s;                              // synced HS_IOR_N
            if (iow_s & ~iow_q2 & ~cs0_s & cs1_s) begin        // IOW# rising, cmd block
                if (adr_s == 3'd1) feat_last <= hs_dd_s[7:0];
                if (adr_s == 3'd2) sc_last   <= hs_dd_s[7:0];
            end
        end
    end

    // ------------------------------------------------------------------------
    // IDENTIFY word-88 patch: advertise UDMA0 only. The PP5002 retail OS sets
    // the highest advertised UDMA mode and strobes its writes at that pace --
    // its UDMA2 HSTROBE is too fast (+ringing) for our 66.5MHz sampler. At
    // UDMA0 (240ns/word) everything has margin, both legs. The iSphynx sets
    // UDMA2 blindly without reading word 88, so FireWire is unaffected.
    // If the card signs the block (word 255 low byte 0xA5), fix the checksum.
    // ------------------------------------------------------------------------
    // pp_sess: the retail OS opens with EXECUTE DIAGNOSTIC (C90); the iSphynx
    // never sends it. In a PP5002 session the IDENTIFY advertises NO DMA at
    // all (words 63+88 zeroed): its write strobes never sample cleanly at
    // 66.5MHz (phantoms unfiltered, dropped words filtered), and every
    // standalone boot corrupted the volume through them. PIO via transparent
    // passthrough is the most-proven path in the design, and the OS sets a
    // full PIO fallback (F03 T0C seen every boot). iSphynx sessions keep UDMA.
    reg        pp_sess;
    reg        id_active;
    reg [8:0]  id_wcnt;
    reg [7:0]  id_csum_delta;      // sum(orig bytes) - sum(patched bytes), mod 256
    // word 0 "general configuration": bit 7 = removable media, and CFA cards
    // answer with the 0x848A signature. Either one trips the iPod/iTunes
    // fixed-disk gate (and the CFA signature is what the TSB43AA82 keys its
    // PIO fallback on). Replace with 0x045A, a classic fixed-disk value.
    wire       id_w0_bad = cf_dd_s[7] | (cf_dd_s == 16'h848A);
    reg [15:0] id_orig;            // card's word, LATCHED mid-strobe (the bus is
                                   // released by completion time)
    wire       id_orig_bad = id_orig[7] | (id_orig == 16'h848A);
    always @(posedge CLK or negedge RST_N) begin
        if (!RST_N) begin
            id_active <= 1'b0; id_wcnt <= 9'd0; id_csum_delta <= 8'd0;
            id_orig   <= 16'h0;
            pp_sess   <= 1'b0;
        end else begin
            if (cmd_valid & (cmd_byte == 8'h90)) pp_sess <= 1'b1;
            if (id_active & ~hs_strobe_s & ~cs0_s & cs1_s & (adr_s == 3'd0))
                id_orig <= cf_dd_s;                    // stable while strobe low
            if (cmd_valid) begin
                id_active     <= (cmd_byte == 8'hEC);
                id_wcnt       <= 9'd0;
                id_csum_delta <= 8'd0;
            end else if (id_active & hs_strobe_s & ~ior_q2 & ~cs0_s & cs1_s
                         & (adr_s == 3'd0)) begin      // data-reg read completed
                // accumulate the byte-sum delta of every word we patched, so
                // the word-255 checksum can be corrected in one place
                if ((id_wcnt == 9'd0) & id_orig_bad)
                    id_csum_delta <= id_csum_delta
                                   + id_orig[7:0] + id_orig[15:8] - 8'h5A - 8'h04;
                if (((id_wcnt == 9'd63) | (id_wcnt == 9'd88)) & pp_sess)
                    id_csum_delta <= id_csum_delta
                                   + id_orig[7:0] + id_orig[15:8];
                if (id_wcnt == 9'd255) id_active <= 1'b0;
                id_wcnt <= id_wcnt + 9'd1;
            end
        end
    end
    // Companion: hosts that honor the patched word 88 set UDMA0 (F03 T40) --
    // but ONLY the host's own strobe pace needs slowing (PP5002 SI). Rewrite
    // the SC byte to 0x42 on its way to the card so the CF leg stays at
    // UDMA2, the proven regime; a UDMA0-slow card makes the iSphynx overrun
    // the FIFO into long DDMARDY pauses, which it answers by abandoning the
    // burst (broke the updater's big writes).
    wire        mode_sc_hit = ~cs0_s & cs1_s & (adr_s == 3'd2)
                            & (feat_last == 8'h03) & (hs_dd_s[7:4] == 4'h4)
                            & ~burst_active & ~inj_active;

    // Power-command neutering: the retail OS issues STANDBY (E0) after idle
    // stretches. The original Toshiba wakes transparently; adapters and
    // industrial CF ignore it; a retail card HONORS it and then playback
    // reads fail until every song has been skipped. SLEEP (E6) is worse: it
    // requires a reset to wake, which the OS will never send mid-session.
    // Rewrite E0/E2/E6 to E5 (CHECK POWER MODE): same completion choreography
    // (BSY, INTRQ, status), card never powers down. IDLE (E1/E3) passes, it
    // wakes transparently by spec.
    wire        pm_cmd_hit = ~cs0_s & cs1_s & (adr_s == 3'd7)
                           & ((hs_dd_s[7:0] == 8'hE0) | (hs_dd_s[7:0] == 8'hE2)
                            | (hs_dd_s[7:0] == 8'hE6))
                           & ~burst_active & ~inj_active;

    // Word 0 (fixed-disk) patches for EVERY master: iTunes checks the media
    // type through the iSphynx too. Words 63/88 (DMA caps) patch in PP5002
    // sessions only -- the iSphynx reads true DMA caps and self-paces UDMA2
    // (the SC rewrite pins the card regardless). Word 255 fixes the CFA
    // block checksum by the accumulated delta when the card signs (0xA5).
    wire        id_rd     = id_active & ~cs0_s & cs1_s & (adr_s == 3'd0);
    wire        id0_hit   = id_rd & (id_wcnt == 9'd0) & id_w0_bad;   // fixed disk
    wire        id63_hit  = id_rd & pp_sess & (id_wcnt == 9'd63);    // MWDMA: none
    wire        id88_hit  = id_rd & pp_sess & (id_wcnt == 9'd88);    // UDMA:  none
    wire        id255_hit = id_rd & (id_wcnt == 9'd255) & (cf_dd_s[7:0] == 8'hA5);
    wire [15:0] id_val    = id0_hit ? 16'h045A
                          : (id63_hit | id88_hit) ? 16'h0000
                          : {cf_dd_s[15:8] + id_csum_delta, 8'hA5};
    wire        id_hit    = id0_hit | id63_hit | id88_hit | id255_hit;

    // ------------------------------------------------------------------------
    // command snoop
    // ------------------------------------------------------------------------
    wire       cmd_valid, cmd_is_dma, cmd_dir_write;
    wire [7:0] cmd_byte;
    cmd_snoop u_snoop (
        .clk(CLK), .rst_n(RST_N),
        .cs0_n(cs0_s), .cs1_n(cs1_s), .a2(a2_s), .a1(a1_s), .a0(a0_s),
        .iow_n(iow_s), .dd(hs_dd_s[7:0]),
        .cmd_valid(cmd_valid), .cmd(cmd_byte),
        .is_dma(cmd_is_dma), .dir_write(cmd_dir_write)
    );

    // ------------------------------------------------------------------------
    // sequencer (CONCURRENT: producer fills FIFO while consumer drains it, so a
    // command of any length streams through a fixed FIFO -- no 16-sector cap).
    //   READ : producer=host (CF->FIFO), consumer=device (FIFO->iSphynx)
    //   WRITE: producer=device (iSphynx->FIFO), consumer=host (FIFO->CF)
    // ------------------------------------------------------------------------
    localparam SEQ_IDLE = 2'd0, SEQ_RUN = 2'd1;
    reg  [1:0] seq;
    reg        dir_w_l;
    reg        want_host, want_dev;                     // request, HELD until ack
    wire       host_busy, host_done, dev_busy, dev_done;
    wire       host_go = want_host & ~host_busy;
    wire       dev_go  = want_dev  & ~dev_busy;

    // role mapping by direction
    wire       prod_busy = dir_w_l ? dev_busy  : host_busy;
    wire       cons_busy = dir_w_l ? host_busy : dev_busy;
    wire       want_prod = dir_w_l ? want_dev  : want_host;
    wire       want_cons = dir_w_l ? want_host : want_dev;

    reg dbg_end_r, dbg_start_r, dbg_chunk_r;
    reg [16:0] words_total, words_cap;                  // command length / words produced
    reg        prod_complete, fifo_room, cons_ready, fifo_afull, fifo_afull_raw;  // registered compares
    always @(posedge CLK or negedge RST_N)
        if (!RST_N) begin prod_complete<=1'b0; fifo_room<=1'b1; cons_ready<=1'b0;
                          fifo_afull<=1'b0; fifo_afull_raw<=1'b0; end
        else begin
            prod_complete <= (words_cap >= words_total);
            fifo_room     <= ({4'b0,fifo_count} < (FIFO_DEPTH - 512));
            // TWO flow-control flavors, one per consumer temperament:
            //  - afull_raw (single-threshold, "bouncy"): DDMARDY# to the
            //    iSphynx. The bounce duty-cycles it to the drain rate -- the
            //    config every proven sync ran on. A long HYSTERETIC pause
            //    makes the iSphynxII ABANDON the burst (killed big writes).
            //  - fifo_afull (hysteretic): HDMARDY# to the CARD on reads.
            //    Cards tolerate long clean pauses; bouncing violates tRP.
            fifo_afull_raw <= ({4'b0,fifo_count} > (FIFO_DEPTH - 512));
            fifo_afull    <= fifo_afull
                           ? ({4'b0,fifo_count} > (FIFO_DEPTH - 1024))
                           : ({4'b0,fifo_count} > (FIFO_DEPTH - 512));
            // WRITES run store-and-forward: the CF burst opens only after the
            // iSphynx delivery is complete (hardware: concurrent write bursts
            // stall with the card never asserting DDMARDY#; 1-sector writes --
            // which are inherently sequential -- work). fifo_afull is the
            // >FIFO-sized-command safety valve. wr_pend re-arms the engine when
            // a device burst-termination stranded a staged word. READS stay
            // concurrent.
            cons_ready    <= dir_w_l ? (((words_cap >= words_total) & (~fifo_empty | wr_pend))
                                        | ({4'b0,fifo_count} > (FIFO_DEPTH - 512)))
                                     : (({4'b0,fifo_count} >= 13'd256)
                                        | ((words_cap >= words_total) & ~fifo_empty));
        end

    always @(posedge CLK or negedge RST_N) begin
        if (!RST_N) begin
            seq <= SEQ_IDLE; dir_w_l <= 1'b0; want_host <= 1'b0; want_dev <= 1'b0;
            dbg_end_r <= 1'b0; dbg_start_r <= 1'b0; dbg_chunk_r <= 1'b0;
            words_total <= 17'd0; words_cap <= 17'd0;
        end else begin
            dbg_end_r <= 1'b0; dbg_start_r <= 1'b0; dbg_chunk_r <= 1'b0;
            if (want_host & host_busy) want_host <= 1'b0;   // engine acked -> drop request
            if (want_dev  & dev_busy ) want_dev  <= 1'b0;
            if (fifo_wr) words_cap <= words_cap + 1'b1;      // words produced into FIFO
            case (seq)
            SEQ_IDLE:
                if (cmd_valid & cmd_is_dma & ~inj_active) begin
                    dbg_start_r <= 1'b1;
                    dir_w_l <= cmd_dir_write;
                    words_total <= (sc_last == 8'h00) ? 17'd65536
                                                      : {1'b0, sc_last, 8'b0};  // sectors*256
                    words_cap <= 17'd0;
                    if (cmd_dir_write) want_dev  <= 1'b1; // arm producer = device
                    else               want_host <= 1'b1; // arm producer = host
                    seq <= SEQ_RUN;
                end
            SEQ_RUN: begin
                // keep the PRODUCER busy while more to capture and FIFO has room
                if (~prod_busy & ~want_prod & ~prod_complete & fifo_room) begin
                    if (dir_w_l) want_dev  <= 1'b1;
                    else         want_host <= 1'b1;
                    dbg_chunk_r <= 1'b1;
                end
                // keep the CONSUMER busy while the FIFO has a burst's worth (or tail)
                if (~cons_busy & ~want_cons & cons_ready) begin
                    if (dir_w_l) want_host <= 1'b1;
                    else         want_dev  <= 1'b1;
                end
                // done: whole command captured AND fully drained
                if (prod_complete & fifo_empty & ~prod_busy & ~cons_busy
                                  & ~want_prod & ~want_cons
                                  & ~(dir_w_l & wr_pend)) begin
                    seq <= SEQ_IDLE;
                    dbg_end_r <= 1'b1;
                end
            end
            default: seq <= SEQ_IDLE;
            endcase
            if (wd_fire) begin
                seq <= SEQ_IDLE; want_host <= 1'b0; want_dev <= 1'b0;
            end
        end
    end

    wire burst_active = (seq != SEQ_IDLE);
    reg [7:0] hs_dmackf; reg dmack_p;
    always @(posedge CLK or negedge RST_N)
        if (!RST_N) begin hs_dmackf<=8'd0; dmack_p<=1'b1; end
        else begin
            dmack_p <= dmack_s;
            if (dbg_start_r) hs_dmackf <= 8'd0;
            else if (burst_active & dmack_p & ~dmack_s) hs_dmackf <= hs_dmackf + 8'd1;
        end
    wire [3:0] host_st;
    reg  cf_dmarq_seen;
    reg [3:0] host_st_last;                 // last NON-idle host state this cmd (survives abort reset)
    reg cf_rdy_seen;                        // CF asserted DDMARDY# (IORDY low) during a WRITE
    always @(posedge CLK or negedge RST_N)
        if (!RST_N) begin cf_dmarq_seen <= 1'b0; host_st_last <= 4'hF; end
        else begin
            if (dbg_start_r) begin cf_dmarq_seen <= 1'b0; host_st_last <= 4'hF; end
            else begin
                if (burst_active & cf_dmarq_s) cf_dmarq_seen <= 1'b1;
                if (host_st != 4'd0)           host_st_last  <= host_st;
                if (burst_active & dir_w_l & ~cf_strobe_s) cf_rdy_seen <= 1'b1;
            end
        end
    assign DBG_WCAP   = words_cap[15:0];
    assign DBG_DMACKF = hs_dmackf;
    assign DBG_HOSTQ  = {host_st_last, cf_dmarq_seen, cf_rdy_seen, 2'b0};

    // ------------------------------------------------------------------------
    // burst watchdog: no burst may hold the bus longer than ~126ms. On fire,
    // the sequencer returns to IDLE and both engines get a hard reset pulse,
    // restoring transparent PIO so the host's retries still see a drive.
    // ------------------------------------------------------------------------
    reg [28:0] wd;
    reg [1:0]  abort_sh;
    wire wd_fire = burst_active & wd[28];   // ~4s no-progress (card program pauses
                                            // must not trip it; Mac gives up at 10s)
    always @(posedge CLK or negedge RST_N) begin
        if (!RST_N) begin wd <= 29'd0; abort_sh <= 2'b00; end
        else begin
            abort_sh <= {abort_sh[0], wd_fire};
            if (!burst_active | fifo_wr | fifo_rd | dbg_chunk_r)
                                wd <= 29'd0;      // watchdog = no PROGRESS
            else if (!wd[28])   wd <= wd + 29'd1;
        end
    end
    wire eng_rst_n = RST_N & ~(wd_fire | abort_sh[0] | abort_sh[1]);

    assign DBG_ABORT = abort_sh[0] & ~abort_sh[1];
    assign DBG_END   = dbg_end_r;
    assign DBG_START = dbg_start_r;
    assign DBG_CHUNK = dbg_chunk_r;
    assign DBG_CHUNKV = fifo_count[12:5];

    // ------------------------------------------------------------------------
    // FIFO between the engines
    //   READ  (dir_w_l=0): host writes FIFO, device reads FIFO
    //   WRITE (dir_w_l=1): device writes FIFO, host reads FIFO
    // ------------------------------------------------------------------------
    wire        h_fifo_wr, h_fifo_rd, d_fifo_wr, d_fifo_rd;
    wire [15:0] h_fifo_wr_data, d_fifo_wr_data;
    wire        fifo_full, fifo_empty;
    wire [FIFO_AW:0] fifo_count;
    wire [15:0] fifo_rd_data;

    wire        fifo_wr      = dir_w_l ? d_fifo_wr      : h_fifo_wr;
    wire [15:0] fifo_wr_data = dir_w_l ? d_fifo_wr_data : h_fifo_wr_data;
    wire        fifo_rd      = dir_w_l ? h_fifo_rd      : d_fifo_rd;

    sync_fifo #(.WIDTH(16), .DEPTH(FIFO_DEPTH), .AW(FIFO_AW)) u_fifo (
        .clk(CLK), .rst_n(eng_rst_n),   // abort flushes stale words: leftovers
                                        // would be strobed into the NEXT command
        .wr_en(fifo_wr), .wr_data(fifo_wr_data), .full(fifo_full),
        .rd_en(fifo_rd), .rd_data(fifo_rd_data), .empty(fifo_empty),
        .count(fifo_count)
    );

    // ------------------------------------------------------------------------
    // engines
    // ------------------------------------------------------------------------
    wire [15:0] dev_hs_dd_out;  wire dev_hs_dd_oe;
    wire        dev_dmarq, dev_ddmardy_n;

    udma_device #(.CLK_MHZ(CLK_MHZ)) u_dev (
        .clk(CLK), .rst_n(eng_rst_n),
        .go(dev_go), .dir_write(dir_w_l), .busy(dev_busy), .done(dev_done),
        .fifo_wr(d_fifo_wr), .fifo_wr_data(d_fifo_wr_data), .fifo_full(fifo_full), .fifo_afull(fifo_afull_raw),
        .fifo_rd(d_fifo_rd), .fifo_rd_data(fifo_rd_data), .fifo_empty(fifo_empty),
        .hs_dmarq(dev_dmarq), .hs_dmack_n(dmack_s), .hs_stop(iow_s),
        .hs_ddmardy_n(dev_ddmardy_n), .hs_hstrobe(hs_strobe_s),
        .hs_dd_in(hs_dd_s), .hs_dd_out(dev_hs_dd_out), .hs_dd_oe(dev_hs_dd_oe),
        .hs_strobe_in(hs_strobe_s)
    );

    wire [15:0] host_cf_dd_out; wire host_cf_dd_oe;
    wire        host_dmack_n, host_stop, host_hstrobe, wr_pend;

    udma_host #(.CLK_MHZ(CLK_MHZ)) u_host (
        .clk(CLK), .rst_n(eng_rst_n),
        .go(host_go), .cmd_start(dbg_start_r), .wr_pending(wr_pend),
        .dir_write(dir_w_l), .busy(host_busy), .done(host_done),
        .fifo_rd(h_fifo_rd), .fifo_rd_data(fifo_rd_data), .fifo_empty(fifo_empty),
        .fifo_wr(h_fifo_wr), .fifo_wr_data(h_fifo_wr_data), .fifo_full(fifo_full),
        .fifo_afull(fifo_afull),
        .cf_dmarq(cf_dmarq_s), .cf_dmack_n(host_dmack_n), .cf_stop(host_stop),
        .cf_ddmardy_n(cf_strobe_s), .cf_hstrobe(host_hstrobe),
        .cf_dd_in(cf_dd_s), .cf_dd_out(host_cf_dd_out), .cf_dd_oe(host_cf_dd_oe),
        .cf_strobe_in(cf_strobe_s),
        .words_total(words_total),
        .dbg_crc(DBG_CRC), .dbg_wcnt(DBG_WCNT), .dbg_st(host_st), .dbg_wsent(DBG_WSENT)
    );

    // ------------------------------------------------------------------------
    // PIO bridge (owns DD when not in a DMA burst)
    // ------------------------------------------------------------------------
    wire [15:0] pio_hs_dd_out, pio_cf_dd_out;
    wire        pio_hs_dd_oe,  pio_cf_dd_oe;
    pio_passthru u_pio (
        .active(~burst_active), .ior_n(HS_IOR_N), .iow_n(HS_IOW_N),
        .hs_dd_in(hs_dd_s), .hs_dd_out(pio_hs_dd_out), .hs_dd_oe(pio_hs_dd_oe),
        .cf_dd_in(cf_dd_s), .cf_dd_out(pio_cf_dd_out), .cf_dd_oe(pio_cf_dd_oe)
    );

    // ------------------------------------------------------------------------
    // DD ownership mux + tristate pads
    // ------------------------------------------------------------------------
    // during cf_init: we answer every host read with BSY, and drive the CF bus.
    // during OUR bursts: status/alt-status reads also get BSY -- the passthru
    // is cut off, and the iSphynx reading a floating bus mid-command mistakes
    // garbage for completion and moves on.
    wire hs_stat_addr = (~cs0_s & cs1_s & (adr_s == 3'd7))    // status
                      | (cs0_s & ~cs1_s & (adr_s == 3'd6));   // alt-status
    wire [15:0] hs_dd_drv = inj_active ? 16'h0080
                          : burst_active ? (dev_hs_dd_oe ? dev_hs_dd_out : 16'h0080)
                          : id_hit       ? id_val         : pio_hs_dd_out;
    wire        hs_dd_oe  = inj_active ? ~HS_IOR_N
                          : burst_active ? (dev_hs_dd_oe | (hs_stat_addr & ~HS_IOR_N))
                          : pio_hs_dd_oe;
    wire [15:0] cf_dd_drv = inj_active ? inj_dd
                          : burst_active ? host_cf_dd_out
                          : mode_sc_hit  ? 16'h0042
                          : pm_cmd_hit   ? 16'h00E5       : pio_cf_dd_out;
    wire        cf_dd_oe  = inj_active ? inj_dd_oe
                          : burst_active ? host_cf_dd_oe  : pio_cf_dd_oe;

    assign HS_DD = hs_dd_oe ? hs_dd_drv : 16'bz;
    assign CF_DD = cf_dd_oe ? cf_dd_drv : 16'bz;

    // ------------------------------------------------------------------------
    // control-line ownership
    // ------------------------------------------------------------------------
    // during our re-originated bursts the card must see a LEGAL UDMA bus:
    // CS0/CS1 negated, DA2:0 = 0 (ATA requires this throughout DMACK-). The
    // iSphynx parks on the status reg between its own bursts; passing that
    // through mid-burst makes compliant cards withhold DDMARDY# -> deadlock.
    assign CF_CS0_N  = inj_active ? inj_cs0     : burst_active ? 1'b1 : HS_CS0_N;
    assign CF_CS1_N  = (inj_active | burst_active) ? 1'b1             : HS_CS1_N;
    assign CF_A0     = inj_active ? inj_addr[0] : burst_active ? 1'b0 : HS_A0;
    assign CF_A1     = inj_active ? inj_addr[1] : burst_active ? 1'b0 : HS_A1;
    assign CF_A2     = inj_active ? inj_addr[2] : burst_active ? 1'b0 : HS_A2;
    assign CF_RESET_N= HS_RESET_N & ~inj_rst;
    assign HS_INTRQ  = inj_active ? 1'b0 : cf_intrq_s;

    assign CF_DMACK_N = inj_active ? 1'b1
                      : burst_active ? host_dmack_n : HS_DMACK_N;
    assign CF_IOR_N   = inj_active ? inj_ior
                      : burst_active ? host_hstrobe : HS_IOR_N;   // HSTROBE/HDMARDY to CF
    assign CF_IOW_N   = inj_active ? inj_iow
                      : burst_active ? host_stop    : HS_IOW_N;   // STOP to CF

    assign HS_DMARQ   = dev_dmarq;                                 // device-sourced
    assign HS_IORDY   = inj_active ? 1'b1
                      : burst_active ? dev_ddmardy_n : cf_strobe_s;// DSTROBE/DDMARDY in burst; CF IORDY pass-through in PIO

    // STOP polarity RESOLVED (2026-07-08): per ATA UDMA, STOP rides DIOW# with
    // asserted = HIGH (line idles high in PIO, host drives it LOW to open the
    // burst). cf_stop is the raw line level: 1 = stopped/idle, 0 = burst open.
    // Passing it straight to CF_IOW_N is correct; no inversion needed.

    // snapshot engine state at the moment the watchdog fires (before the abort
    // pulse resets the engines), so the 'X' event carries real evidence
    wire [7:0] live_stat = {seq, dir_w_l, host_busy, dev_busy, fifo_empty, fifo_full, 1'b0};
    reg  [7:0] stat_wd;
    always @(posedge CLK or negedge RST_N)
        if (!RST_N) stat_wd <= 8'h00;
        else if (wd_fire) stat_wd <= live_stat;

    assign DBG_STAT = DBG_ABORT ? stat_wd : live_stat;

endmodule
