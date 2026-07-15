// ============================================================================
// ata_device.v  --  full ATA target: the FPGA *is* the disk (v2)
// ----------------------------------------------------------------------------
// v1 bridged two ATA buses and let the CF do the being-a-disk. v2 has no CF:
// this module owns the taskfile, decodes the command inventory captured
// during v1 bring-up (docs/v2-native-sd.md), serves IDENTIFY from an on-chip
// image, runs PIO data phases itself, and reuses the hardware-proven
// udma_device engine for the iSphynxII's UDMA2 bursts. Sectors move through
// the same FWFT FIFO pattern as v1, but the far side is now a blk_* backend
// (backend_sd) instead of the CF-facing UDMA host engine.
//
// Everything v1 bring-up taught about the two masters is preserved:
//   * iSphynxII: UDMA2 via udma_device -- bouncy DDMARDY throttle (fifo
//     afull_raw), BSY answered to status polls mid-command, CRC ignored on
//     writes / cosmetic on reads.
//   * PP5002: PIO only. pp_sess latches on EXECUTE DIAGNOSTIC (90h); the
//     IDENTIFY image advertises NO DMA at all in that session (words 63+88
//     zero). The iSphynx never reads word 88 and blindly runs UDMA2.
//   * The retail OS tolerates ABRT (S51) on the APM SET FEATURES subcodes.
//
// Reset domain: rst_n includes the host ATA reset line (ATA-visible state
// resets with the host). The backend/SD live on POR only; `abort_req` (reset
// value 1) tells them to close out cleanly whenever we come up or die.
// ============================================================================

module ata_device #(
    parameter CLK_MHZ    = 66,
    parameter FIFO_AW    = 12,
    parameter FIFO_DEPTH = 4096
) (
    input             clk,
    input             rst_n,          // POR & host ATA reset

    // ---- host bus (iPod 50-pin) ---------------------------------------------
    input      [15:0] hs_dd_in,       // raw pad value
    output     [15:0] hs_dd_out,
    output            hs_dd_oe,
    input             hs_cs0_n, hs_cs1_n,
    input             hs_a0, hs_a1, hs_a2,
    input             hs_ior_n,       // DIOR# / HDMARDY / HSTROBE family
    input             hs_iow_n,       // DIOW# / STOP
    output            hs_dmarq,
    input             hs_dmack_n,
    output            hs_intrq,
    output            hs_iordy,

    // ---- blk_* backend --------------------------------------------------------
    output reg        blk_req,        // pulse (backend latches)
    output reg        blk_write,
    output reg [31:0] blk_lba,
    output reg [16:0] blk_nsec,
    input             blk_busy,
    input             blk_done,       // pulse
    input             blk_err,        // valid with blk_done
    output reg        blk_flush,      // pulse (backend latches)
    input             blk_flush_done,
    output reg        abort_req,      // level; RESETS TO 1 (see header)
    input             blk_ready,
    input      [31:0] blk_capacity,

    // ---- backend FIFO face -----------------------------------------------------
    input             brd_wr,         // backend -> FIFO (read commands)
    input      [15:0] brd_data,
    output            brd_full,
    output            bwr_avail,      // FIFO -> backend: full sector waiting
    input             bwr_rd,
    output     [15:0] bwr_data,

    // ---- debug ------------------------------------------------------------------
    output            dbg_start,      // 1-cycle: data command dispatched
    output            dbg_end,        // 1-cycle: command completed
    output            dbg_abort,      // 1-cycle: watchdog abort
    output     [7:0]  dbg_stat,
    output     [15:0] dbg_wcap,
    output     [7:0]  dbg_dmackf,
    output            pp_sess_o
);

    // ------------------------------------------------------------------------
    // input synchronizers (v1 discipline)
    // ------------------------------------------------------------------------
    reg [15:0] hs_dd_s;
    reg        cs0_s, cs1_s, a0_s, a1_s, a2_s, ior_s, iow_s, dmack_s;
    always @(posedge clk) begin
        hs_dd_s <= hs_dd_in;
        cs0_s   <= hs_cs0_n;  cs1_s <= hs_cs1_n;
        a0_s    <= hs_a0;     a1_s  <= hs_a1;    a2_s <= hs_a2;
        ior_s   <= hs_ior_n;  iow_s <= hs_iow_n;
        dmack_s <= hs_dmack_n;
    end
    reg ior_q2, iow_q2;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ior_q2 <= 1'b1; iow_q2 <= 1'b1; end
        else        begin ior_q2 <= ior_s; iow_q2 <= iow_s; end

    wire [2:0] adr      = {a2_s, a1_s, a0_s};
    wire       sel_cmd  = ~cs0_s & cs1_s;
    wire       sel_ctl  = cs0_s & ~cs1_s;
    wire       iow_rise = iow_s & ~iow_q2;
    wire       ior_rise = ior_s & ~ior_q2;

    // dead-time filter on DATA-PORT strobes only (crosstalk armor; taskfile
    // events stay bare exactly as v1 proved them)
    reg [1:0] dp_dead;
    wire dp_rd_raw = ior_rise & sel_cmd & (adr == 3'd0);
    wire dp_wr_raw = iow_rise & sel_cmd & (adr == 3'd0);
    wire dp_rd_ev  = dp_rd_raw & (dp_dead == 2'd0);
    wire dp_wr_ev  = dp_wr_raw & (dp_dead == 2'd0);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) dp_dead <= 2'd0;
        else if (dp_rd_ev | dp_wr_ev) dp_dead <= 2'd2;
        else if (dp_dead != 2'd0)     dp_dead <= dp_dead - 2'd1;

    // ------------------------------------------------------------------------
    // forward declarations (FSM <-> register file crosstalk)
    // ------------------------------------------------------------------------
    reg        st_bsy, st_drdy, st_drq, st_err;
    reg        sig_set, smart_sig_set, pwr_sc_set;
    reg        fifo_rd_pin, fifo_wr_pout;
    reg [15:0] fifo_wr_pout_data;
    reg        scrub;
    reg [28:0] wd;
    reg [1:0]  abort_sh;

    // ------------------------------------------------------------------------
    // taskfile registers
    // ------------------------------------------------------------------------
    reg [7:0] feat_r, sc_r, lba0_r, lba1_r, lba2_r, dev_r, err_r;
    reg       nien_r, srst_r;
    reg [7:0] cmd_r;
    reg       cmd_ev;
    wire      dev1 = dev_r[4];
    wire [7:0] status = {st_bsy, st_drdy, 1'b0, 1'b1, st_drq, 2'b00, st_err};

    reg intrq_r;
    assign hs_intrq = intrq_r & ~nien_r & ~dev1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            feat_r <= 8'h00; sc_r <= 8'h01; lba0_r <= 8'h01;
            lba1_r <= 8'h00; lba2_r <= 8'h00; dev_r <= 8'h00;
            nien_r <= 1'b0; srst_r <= 1'b0;
            cmd_r <= 8'h00; cmd_ev <= 1'b0;
        end else begin
            cmd_ev <= 1'b0;
            if (iow_rise & sel_cmd & ~st_bsy & ~st_drq) begin
                case (adr)
                3'd1: feat_r <= hs_dd_s[7:0];
                3'd2: sc_r   <= hs_dd_s[7:0];
                3'd3: lba0_r <= hs_dd_s[7:0];
                3'd4: lba1_r <= hs_dd_s[7:0];
                3'd5: lba2_r <= hs_dd_s[7:0];
                3'd6: dev_r  <= hs_dd_s[7:0];
                3'd7: if (!dev1 || hs_dd_s[7:0] == 8'h90) begin
                    cmd_r  <= hs_dd_s[7:0];
                    cmd_ev <= 1'b1;
                end
                default: ;
                endcase
            end
            if (iow_rise & sel_ctl & (adr == 3'd6)) begin
                nien_r <= hs_dd_s[1];
                srst_r <= hs_dd_s[2];
            end
            if (sig_set) begin
                sc_r <= 8'h01; lba0_r <= 8'h01; lba1_r <= 8'h00; lba2_r <= 8'h00;
                dev_r <= 8'h00;
            end
            if (smart_sig_set) begin lba1_r <= 8'h4F; lba2_r <= 8'hC2; end
            if (pwr_sc_set)    sc_r <= 8'hFF;
        end
    end

    // ------------------------------------------------------------------------
    // pp_sess: PP5002 session discriminator (v1-proven). Cleared only by
    // rst_n (POR/host reset); survives SRST like the v1 latch did.
    // ------------------------------------------------------------------------
    reg pp_sess;
    assign pp_sess_o = pp_sess;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) pp_sess <= 1'b0;
        else if (iow_rise & sel_cmd & (adr == 3'd7) & (hs_dd_s[7:0] == 8'h90)
                 & ~st_bsy & ~st_drq)
            pp_sess <= 1'b1;

    // ------------------------------------------------------------------------
    // MULTIPLE mode + CHS geometry state (FSM-owned)
    // ------------------------------------------------------------------------
    reg [4:0] mult_r;                  // 0 = MULTIPLE disabled
    reg [4:0] chs_heads;
    reg [5:0] chs_spt;

    wire [31:0] cap_lba = (blk_capacity > 32'h0FFFFFFF) ? 32'h0FFFFFFF
                                                        : blk_capacity;

    // ------------------------------------------------------------------------
    // display-cylinder divider: cyl_disp = min(16383, cap_lba / 1008)
    // restoring division, one bit per clk, kicked once when capacity lands
    // ------------------------------------------------------------------------
    reg         div_start, div_run, div_done_q;
    reg  [5:0]  div_i;
    reg  [10:0] div_r;
    reg  [31:0] div_dq;                // dividend in, quotient out
    reg  [13:0] cyl_disp;
    wire [10:0] div_t = {div_r[9:0], div_dq[31]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_run <= 1'b0; div_i <= 6'd0; div_r <= 11'd0; div_dq <= 32'd0;
            cyl_disp <= 14'd16383; div_done_q <= 1'b0;
        end else begin
            div_done_q <= 1'b0;
            if (div_start) begin
                div_run <= 1'b1; div_i <= 6'd32;
                div_r   <= 11'd0;
                div_dq  <= cap_lba;
            end else if (div_run) begin
                if (div_t >= 11'd1008) begin
                    div_r  <= div_t - 11'd1008;
                    div_dq <= {div_dq[30:0], 1'b1};
                end else begin
                    div_r  <= div_t;
                    div_dq <= {div_dq[30:0], 1'b0};
                end
                if (div_i == 6'd1) begin
                    div_run    <= 1'b0;
                    div_done_q <= 1'b1;
                end
                div_i <= div_i - 6'd1;
            end
            if (div_done_q)
                cyl_disp <= (div_dq > 32'd16383) ? 14'd16383 : div_dq[13:0];
        end
    end

    // current-geometry capacity = cyl_disp * 1008 (= <<10 - <<4)
    wire [31:0] cap_chs = ({18'b0, cyl_disp} << 10) - ({18'b0, cyl_disp} << 4);

    // ------------------------------------------------------------------------
    // IDENTIFY image: 256x16 RAM, refilled whenever its inputs change
    // (pp_sess, MULTIPLE, capacity). The pass computes the word-255 signature
    // checksum as it goes.
    // ------------------------------------------------------------------------
    function [15:0] id_str2;           // ATA strings: first char in HIGH byte
        input [7:0] c0, c1;
        id_str2 = {c0, c1};
    endfunction

    function [15:0] id_word;
        input [7:0] i;
        input       pp;
        input [4:0] mult;
        reg [15:0] w;
        begin
            case (i)
            8'd0:   w = 16'h0040;                        // fixed device
            8'd1:   w = {2'b00, cyl_disp};
            8'd3:   w = 16'd16;                          // heads
            8'd6:   w = 16'd63;                          // sectors/track
            // serial "IPODSD-FPGA-V2      "
            8'd10:  w = id_str2("I","P");
            8'd11:  w = id_str2("O","D");
            8'd12:  w = id_str2("S","D");
            8'd13:  w = id_str2("-","F");
            8'd14:  w = id_str2("P","G");
            8'd15:  w = id_str2("A","-");
            8'd16:  w = id_str2("V","2");
            8'd17:  w = id_str2(" "," ");
            8'd18:  w = id_str2(" "," ");
            8'd19:  w = id_str2(" "," ");
            // firmware "2.0     "
            8'd23:  w = id_str2("2",".");
            8'd24:  w = id_str2("0"," ");
            8'd25:  w = id_str2(" "," ");
            8'd26:  w = id_str2(" "," ");
            // model "iPod SD interposer v2" padded to 40 chars
            8'd27:  w = id_str2("i","P");
            8'd28:  w = id_str2("o","d");
            8'd29:  w = id_str2(" ","S");
            8'd30:  w = id_str2("D"," ");
            8'd31:  w = id_str2("i","n");
            8'd32:  w = id_str2("t","e");
            8'd33:  w = id_str2("r","p");
            8'd34:  w = id_str2("o","s");
            8'd35:  w = id_str2("e","r");
            8'd36:  w = id_str2(" ","v");
            8'd37:  w = id_str2("2"," ");
            8'd38, 8'd39, 8'd40, 8'd41, 8'd42,
            8'd43, 8'd44, 8'd45, 8'd46: w = id_str2(" "," ");
            8'd47:  w = 16'h8010;                        // MULTIPLE max = 16
            8'd49:  w = 16'h0B00;                        // LBA + DMA + IORDY
            8'd50:  w = 16'h4000;
            8'd51:  w = 16'h0200;                        // legacy PIO timing
            8'd53:  w = 16'h0007;                        // 54-58, 64-70, 88 valid
            8'd54:  w = {2'b00, cyl_disp};               // current geometry
            8'd55:  w = 16'd16;
            8'd56:  w = 16'd63;
            8'd57:  w = cap_chs[15:0];
            8'd58:  w = cap_chs[31:16];
            8'd59:  w = (mult != 5'd0) ? {8'h01, 3'b000, mult} : 16'h0000;
            8'd60:  w = cap_lba[15:0];
            8'd61:  w = cap_lba[31:16];
            8'd63:  w = 16'h0000;                        // no MWDMA, ever
            8'd64:  w = 16'h0003;                        // PIO3/4
            8'd65:  w = 16'd120;
            8'd66:  w = 16'd120;
            8'd67:  w = 16'd120;
            8'd68:  w = 16'd120;
            8'd80:  w = 16'h001E;                        // ATA-1..4
            8'd82:  w = 16'h0021;                        // SMART + write cache
            8'd83:  w = 16'h5000;                        // bit14 + FLUSH CACHE
            8'd84:  w = 16'h4000;
            8'd85:  w = 16'h0021;
            8'd86:  w = 16'h1000;
            8'd87:  w = 16'h4000;
            8'd88:  w = pp ? 16'h0000 : 16'h0407;        // UDMA0-2, mode 2 active
            default: w = 16'h0000;
            endcase
            id_word = w;
        end
    endfunction

    // single write port (id_we/id_wa/id_wd) -- LSE refuses to see a RAM if
    // the array has more than one write statement, and 256x16 in registers
    // is 4096 flip-flops (found the hard way: 162% overflow on the 7000)
    reg  [15:0] id_ram [0:255];
    reg         id_valid, id_fill_run;
    reg         id_we;
    reg  [7:0]  id_wa;
    reg  [15:0] id_wd;
    reg  [8:0]  id_idx;
    reg  [7:0]  id_csum;
    reg         pp_l, rdy_l;
    reg  [4:0]  mult_l;
    wire [15:0] id_w_cur  = id_word(id_idx[7:0], pp_l, mult_l);
    wire        id_stale  = (pp_l != pp_sess) | (mult_l != mult_r)
                          | (rdy_l != blk_ready);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_valid <= 1'b0; id_fill_run <= 1'b0; id_idx <= 9'd0; id_csum <= 8'd0;
            pp_l <= 1'b0; mult_l <= 5'd0; rdy_l <= 1'b0; div_start <= 1'b0;
            id_we <= 1'b0; id_wa <= 8'd0; id_wd <= 16'h0;
        end else begin
            div_start <= 1'b0;
            id_we     <= 1'b0;
            if (!id_fill_run) begin
                if (blk_ready & (id_stale | ~id_valid)) begin
                    if (rdy_l != blk_ready) div_start <= 1'b1;   // capacity landed
                    pp_l <= pp_sess; mult_l <= mult_r; rdy_l <= blk_ready;
                    id_fill_run <= 1'b1;
                    id_valid    <= 1'b0;
                    id_idx      <= 9'd0;
                    id_csum     <= 8'd0;
                end
            end else if (div_start | div_run | div_done_q) begin
                ;                                        // wait out the divider
            end else if (id_idx == 9'd256) begin
                id_fill_run <= 1'b0;
                id_valid    <= 1'b1;
            end else begin
                id_we <= 1'b1;
                id_wa <= id_idx[7:0];
                id_wd <= (id_idx == 9'd255)
                       ? {(8'h00 - (id_csum + 8'hA5)), 8'hA5}
                       : id_w_cur;
                if (id_idx != 9'd255)
                    id_csum <= id_csum + id_w_cur[7:0] + id_w_cur[15:8];
                id_idx <= id_idx + 9'd1;
            end
        end
    end
    always @(posedge clk) if (id_we) id_ram[id_wa] <= id_wd;

    reg [7:0]  id_addr;
    reg [15:0] id_out;
    always @(posedge clk) id_out <= id_ram[id_addr];

    // ------------------------------------------------------------------------
    // FIFO (v1 sync_fifo) + ownership mux
    // ------------------------------------------------------------------------
    wire        fifo_full, fifo_empty;
    wire [FIFO_AW:0] fifo_count;
    wire [15:0] fifo_rd_data;
    reg         fifo_wr_mux, fifo_rd_mux;
    reg  [15:0] fifo_wr_data_mux;
    wire        eng_rst_n = rst_n & ~(wd_fire | abort_sh[0] | abort_sh[1]
                                      | srst_r | scrub);

    sync_fifo #(.WIDTH(16), .DEPTH(FIFO_DEPTH), .AW(FIFO_AW)) u_fifo (
        .clk(clk), .rst_n(eng_rst_n),
        .wr_en(fifo_wr_mux), .wr_data(fifo_wr_data_mux), .full(fifo_full),
        .rd_en(fifo_rd_mux), .rd_data(fifo_rd_data), .empty(fifo_empty),
        .count(fifo_count)
    );

    assign brd_full  = fifo_full;
    assign bwr_data  = fifo_rd_data;

    // ---- registered compares (v1 idiom, one step further: the pointer
    // subtract AND the compare are each a full carry chain, so the count is
    // registered first and every compare runs off the registered copy) -----
    reg [FIFO_AW:0] fifo_count_r;
    reg fifo_afull_raw;                // v1 "bouncy" throttle flavor
    reg pin_ready_r, pout_room_r, fifo_room_r, bwr_avail_r, fifo_full_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            fifo_count_r <= 0;
            fifo_afull_raw <= 1'b0;
            pin_ready_r <= 1'b0; pout_room_r <= 1'b0; fifo_room_r <= 1'b0;
            bwr_avail_r <= 1'b0; fifo_full_r <= 1'b0;
        end else begin
            fifo_count_r   <= fifo_count;
            fifo_afull_raw <= ({4'b0,fifo_count_r} > (FIFO_DEPTH - 512));
            pin_ready_r    <= ({4'b0,fifo_count_r} >= {4'b0,blk_words_w});
            pout_room_r    <= ((FIFO_DEPTH - {4'b0,fifo_count_r}) >= {4'b0,blk_words_w});
            fifo_room_r    <= ({4'b0,fifo_count_r} < (FIFO_DEPTH - 512));
            bwr_avail_r    <= (fifo_count_r >= 13'd256);
            fifo_full_r    <= fifo_full;
        end
    assign bwr_avail = bwr_avail_r;

    // registered compares are 1 cycle stale; at block boundaries the wait
    // states must ignore them until they reflect wait-state-era counts
    // (in the wait states the count moves only in the safe direction)
    reg [1:0] wsettle;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) wsettle <= 2'd0;
        else if (st == ST_PIN_W || st == ST_POUT_W) begin
            if (wsettle != 2'd2) wsettle <= wsettle + 2'd1;
        end else wsettle <= 2'd0;

    // ------------------------------------------------------------------------
    // UDMA2 device engine (v1, verbatim instance)
    // ------------------------------------------------------------------------
    reg         dev_go_req;
    wire        dev_busy, dev_done;
    wire        dev_go = dev_go_req & ~dev_busy;
    wire        d_fifo_wr, d_fifo_rd;
    wire [15:0] d_fifo_wr_data;
    wire [15:0] dev_hs_dd_out;
    wire        dev_hs_dd_oe;
    wire        dev_dmarq, dev_ddmardy_n;
    reg         dma_dir_write;

    udma_device #(.CLK_MHZ(CLK_MHZ)) u_dev (
        .clk(clk), .rst_n(eng_rst_n),
        .go(dev_go), .dir_write(dma_dir_write), .busy(dev_busy), .done(dev_done),
        .fifo_wr(d_fifo_wr), .fifo_wr_data(d_fifo_wr_data),
        .fifo_full(fifo_full), .fifo_afull(fifo_afull_raw),
        .fifo_rd(d_fifo_rd), .fifo_rd_data(fifo_rd_data), .fifo_empty(fifo_empty),
        .hs_dmarq(dev_dmarq), .hs_dmack_n(dmack_s), .hs_stop(iow_s),
        .hs_ddmardy_n(dev_ddmardy_n), .hs_hstrobe(ior_s),
        .hs_dd_in(hs_dd_s), .hs_dd_out(dev_hs_dd_out), .hs_dd_oe(dev_hs_dd_oe),
        .hs_strobe_in(ior_s)
    );

    // ------------------------------------------------------------------------
    // command FSM
    // ------------------------------------------------------------------------
    localparam ST_BOOT   = 5'd0,
               ST_IDLE   = 5'd1,
               ST_SETUP  = 5'd2,
               ST_XLATE  = 5'd3,
               ST_DISP   = 5'd4,
               ST_PIN_W  = 5'd5,
               ST_PIN_D  = 5'd6,
               ST_POUT_W = 5'd7,
               ST_POUT_D = 5'd8,
               ST_POUT_E = 5'd9,
               ST_DMA    = 5'd10,
               ST_DMA_E  = 5'd11,
               ST_FLUSH  = 5'd12,
               ST_DIAG   = 5'd13,
               ST_FIN    = 5'd14,
               ST_FINERR = 5'd15,
               ST_SRST   = 5'd16,
               ST_IDWAIT = 5'd17,
               ST_GO     = 5'd18;   // dispatch (bounds check registered in DISP)

    reg [4:0]  st;
    reg        cmdclass_dma;
    reg [1:0]  pin_src;
    localparam SRC_FIFO = 2'd0, SRC_ID = 2'd1, SRC_ZERO = 2'd2;

    reg [16:0] sec_left;
    reg [4:0]  blk_secs;
    reg [12:0] word_cnt;
    // "is this the last DRQ block?" registered off sec_left -- sec_left is
    // constant through a block (changes only at block completion), so the
    // registered copy is stable by the time the block finishes. Breaks the
    // sec_left -> compare -> FSM critical path (last 0.24ns to close 66.5MHz).
    reg        last_blk;
    // words in the current DRQ block -- combinational so wait states never
    // race a stale registered copy
    wire [12:0] blk_words_w = (sec_left < {12'b0, blk_secs})
                            ? {sec_left[4:0], 8'b0} : {blk_secs, 8'b0};
    always @(posedge clk or negedge rst_n)
        if (!rst_n) last_blk <= 1'b1;
        else        last_blk <= (sec_left <= {12'b0, blk_secs});
    reg [16:0] words_total, words_prod, words_cap;
    reg        first_blk;
    reg [31:0] lba_acc;
    reg [31:0] xl_a;
    reg [5:0]  xl_b;
    reg        xl_phase;
    reg        end_irq_mask;           // PIN already interrupted per block
    reg        blk_done_seen, blk_err_seen;
    reg        bounds_bad;
    reg [7:0]  diag_tmr;
    reg        dbg_start_r, dbg_end_r;

    // watchdog (v1 constants: wd[28] ~ 4s @ 66.5MHz)
    wire cmd_active = (st != ST_IDLE) && (st != ST_BOOT) && (st != ST_SRST);
    wire wd_fire    = cmd_active & wd[28];
    wire progress   = fifo_wr_mux | fifo_rd_mux | blk_done | dp_rd_ev | dp_wr_ev
                    | blk_flush_done | dev_done;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin wd <= 29'd0; abort_sh <= 2'b00; end
        else begin
            abort_sh <= {abort_sh[0], wd_fire};
            if (!cmd_active | progress) wd <= 29'd0;
            else if (!wd[28])           wd <= wd + 29'd1;
        end
    end

    // backend completion latches (pulses may land in any state); cleared at
    // every command acceptance so stale closes can't bleed into the next one
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin blk_done_seen <= 1'b0; blk_err_seen <= 1'b0; end
        else if (cmd_ev) begin blk_done_seen <= 1'b0; blk_err_seen <= 1'b0; end
        else if (blk_done) begin
            blk_done_seen <= 1'b1;
            if (blk_err) blk_err_seen <= 1'b1;
        end
    end

    // producer bookkeeping
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin words_prod <= 17'd0; words_cap <= 17'd0; end
        else if (st == ST_SETUP) begin words_prod <= 17'd0; words_cap <= 17'd0; end
        else begin
            if (brd_wr & ~fifo_full)    words_prod <= words_prod + 17'd1;
            if (d_fifo_wr & ~fifo_full) words_cap  <= words_cap + 17'd1;
        end
    end
    reg prod_complete;                 // registered (v1 idiom)
    reg cons_ready;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin prod_complete <= 1'b0; cons_ready <= 1'b0; end
        else begin
            prod_complete <= dma_dir_write ? (words_cap  >= words_total)
                                           : (words_prod >= words_total);
            cons_ready <= ({4'b0,fifo_count_r} >= 17'd256)
                        | ((words_prod >= words_total) & ~fifo_empty);
        end

    reg [7:0] dmackf; reg dmack_p;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin dmackf <= 8'd0; dmack_p <= 1'b1; end
        else begin
            dmack_p <= dmack_s;
            if (st == ST_SETUP) dmackf <= 8'd0;
            else if (cmd_active & dmack_p & ~dmack_s) dmackf <= dmackf + 8'd1;
        end

    assign dbg_start  = dbg_start_r;
    assign dbg_end    = dbg_end_r;
    assign dbg_abort  = abort_sh[0] & ~abort_sh[1];
    assign dbg_stat   = {st[4:0], fifo_empty, fifo_full_r, st_drq};
    assign dbg_wcap   = dma_dir_write ? words_cap[15:0] : words_prod[15:0];
    assign dbg_dmackf = dmackf;

    // decode helpers
    wire [7:0] c = cmd_r;
    wire c_rd_sec   = (c == 8'h20) | (c == 8'h21);
    wire c_wr_sec   = (c == 8'h30) | (c == 8'h31);
    wire c_rd_mul   = (c == 8'hC4);
    wire c_wr_mul   = (c == 8'hC5);
    wire c_rd_dma   = (c == 8'hC8) | (c == 8'hC9);
    wire c_wr_dma   = (c == 8'hCA) | (c == 8'hCB);
    wire c_verify   = (c == 8'h40) | (c == 8'h41);
    wire c_seek     = (c[7:4] == 4'h7) | (c[7:4] == 4'h1);
    wire c_pwr_ok   = (c == 8'hE0) | (c == 8'hE1) | (c == 8'hE2) | (c == 8'hE3)
                    | (c == 8'hE6);
    wire c_data_cmd = c_rd_sec | c_wr_sec | c_rd_mul | c_wr_mul | c_rd_dma | c_wr_dma;

    wire [16:0] nsec_dec = (sc_r == 8'h00) ? 17'd256 : {9'b0, sc_r};
    wire        use_lba  = dev_r[6];
    wire [31:0] lba28    = {4'b0, dev_r[3:0], lba2_r, lba1_r, lba0_r};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= ST_BOOT;
            st_bsy <= 1'b1; st_drdy <= 1'b0; st_drq <= 1'b0; st_err <= 1'b0;
            err_r <= 8'h01;
            intrq_r <= 1'b0;
            blk_req <= 1'b0; blk_write <= 1'b0; blk_lba <= 32'h0; blk_nsec <= 17'd0;
            blk_flush <= 1'b0;
            abort_req <= 1'b1;
            dev_go_req <= 1'b0; dma_dir_write <= 1'b0;
            cmdclass_dma <= 1'b0; pin_src <= SRC_FIFO;
            sec_left <= 17'd0; blk_secs <= 5'd1; word_cnt <= 13'd0;
            words_total <= 17'd0; first_blk <= 1'b0;
            lba_acc <= 32'h0; xl_a <= 32'h0; xl_b <= 6'd0; xl_phase <= 1'b0;
            mult_r <= 5'd16; chs_heads <= 5'd16; chs_spt <= 6'd63;
            sig_set <= 1'b0; smart_sig_set <= 1'b0; pwr_sc_set <= 1'b0;
            scrub <= 1'b0; diag_tmr <= 8'd0; bounds_bad <= 1'b0;
            dbg_start_r <= 1'b0; dbg_end_r <= 1'b0;
            id_addr <= 8'd0;
            end_irq_mask <= 1'b0;
            fifo_rd_pin <= 1'b0; fifo_wr_pout <= 1'b0; fifo_wr_pout_data <= 16'h0;
        end else begin
            blk_req <= 1'b0; blk_flush <= 1'b0;
            sig_set <= 1'b0; smart_sig_set <= 1'b0; pwr_sc_set <= 1'b0;
            scrub <= 1'b0;
            dbg_start_r <= 1'b0; dbg_end_r <= 1'b0;
            fifo_rd_pin <= 1'b0; fifo_wr_pout <= 1'b0;
            if (dev_go_req & dev_busy) dev_go_req <= 1'b0;

            // INTRQ clear: status read or new command
            if ((ior_rise & sel_cmd & (adr == 3'd7) & ~dev1) | cmd_ev)
                intrq_r <= 1'b0;

            case (st)
            // ----------------------------------------------------------------
            ST_BOOT: begin
                st_bsy <= 1'b1; st_drdy <= 1'b0;
                abort_req <= abort_req & blk_busy;
                if (blk_ready & id_valid) begin
                    sig_set <= 1'b1;
                    err_r   <= 8'h01;
                    st_bsy  <= 1'b0; st_drdy <= 1'b1;
                    st <= ST_IDLE;
                end
            end
            // ----------------------------------------------------------------
            ST_SRST: begin
                abort_req <= 1'b1;
                if (!srst_r) begin
                    sig_set <= 1'b1;
                    err_r   <= 8'h01;
                    st_bsy  <= 1'b0; st_drdy <= 1'b1; st_err <= 1'b0; st_drq <= 1'b0;
                    st <= ST_IDLE;
                end
            end
            // ----------------------------------------------------------------
            ST_IDLE: begin
                abort_req <= abort_req & blk_busy;   // hold until the backend idles
                if (cmd_ev) begin
                    st_err <= 1'b0; err_r <= 8'h00;
                    end_irq_mask <= 1'b0;
                    if (c_data_cmd) begin
                        st_bsy <= 1'b1; st_drq <= 1'b0;
                        scrub  <= 1'b1;
                        st <= ST_SETUP;
                    end else begin
                        case (cmd_r)
                        8'h90: begin
                            st_bsy <= 1'b1; st_drq <= 1'b0;
                            diag_tmr <= 8'd200;
                            st <= ST_DIAG;
                        end
                        8'hEC: begin
                            st_bsy <= 1'b1; st_drq <= 1'b0;
                            pin_src  <= SRC_ID;
                            sec_left <= 17'd1; blk_secs <= 5'd1;
                            first_blk <= 1'b1;
                            id_addr <= 8'd0;
                            st <= ST_IDWAIT;
                        end
                        8'hC6: begin
                            st_bsy <= 1'b1;
                            if (sc_r == 8'd0 || sc_r == 8'd1 || sc_r == 8'd2 ||
                                sc_r == 8'd4 || sc_r == 8'd8 || sc_r == 8'd16) begin
                                mult_r <= sc_r[4:0];
                                st <= ST_FIN;
                            end else begin
                                err_r <= 8'h04; st <= ST_FINERR;
                            end
                        end
                        8'h91: begin
                            chs_heads <= {1'b0, dev_r[3:0]} + 5'd1;
                            if (sc_r != 8'd0) chs_spt <= sc_r[5:0];
                            st_bsy <= 1'b1; st <= ST_FIN;
                        end
                        8'hE5: begin
                            pwr_sc_set <= 1'b1;
                            st_bsy <= 1'b1; st <= ST_FIN;
                        end
                        8'hE7: begin
                            st_bsy <= 1'b1;
                            blk_flush <= 1'b1;
                            st <= ST_FLUSH;
                        end
                        8'hEF: begin
                            st_bsy <= 1'b1;
                            case (feat_r)
                            8'h03, 8'h66, 8'h02, 8'h82,
                            8'h55, 8'hAA: st <= ST_FIN;
                            default: begin err_r <= 8'h04; st <= ST_FINERR; end
                            endcase
                        end
                        8'hB0: begin
                            st_bsy <= 1'b1;
                            if (feat_r == 8'hD0) begin
                                pin_src  <= SRC_ZERO;
                                sec_left <= 17'd1; blk_secs <= 5'd1;
                                first_blk <= 1'b1;
                                st <= ST_PIN_W;
                            end else begin
                                if (feat_r == 8'hDA) smart_sig_set <= 1'b1;
                                st <= ST_FIN;
                            end
                        end
                        default: begin
                            st_bsy <= 1'b1;
                            if (c_verify | c_seek | c_pwr_ok) st <= ST_FIN;
                            else begin err_r <= 8'h04; st <= ST_FINERR; end
                        end
                        endcase
                    end
                end
            end
            // ---------------- data command path ------------------------------
            ST_SETUP: begin
                cmdclass_dma  <= c_rd_dma | c_wr_dma;
                dma_dir_write <= c_wr_dma;
                pin_src   <= SRC_FIFO;
                sec_left  <= nsec_dec;
                blk_secs  <= (c_rd_mul | c_wr_mul) ? mult_r : 5'd1;
                first_blk <= 1'b1;
                words_total <= {nsec_dec[8:0], 8'b0};
                if ((c_rd_mul | c_wr_mul) && mult_r == 5'd0) begin
                    err_r <= 8'h04; st <= ST_FINERR;
                end else if (use_lba) begin
                    lba_acc <= lba28;
                    st <= ST_DISP;
                end else begin
                    xl_a <= {16'b0, lba2_r, lba1_r};     // cylinder
                    xl_b <= {1'b0, chs_heads};
                    xl_phase <= 1'b0;
                    lba_acc <= 32'd0;
                    st <= ST_XLATE;
                end
            end
            ST_XLATE: begin
                if (xl_b[0]) lba_acc <= lba_acc + xl_a;
                xl_a <= {xl_a[30:0], 1'b0};
                xl_b <= {1'b0, xl_b[5:1]};
                if (xl_b == 6'd0) begin
                    if (!xl_phase) begin
                        xl_phase <= 1'b1;
                        xl_a <= lba_acc + {28'b0, dev_r[3:0]};   // + head
                        lba_acc <= 32'd0;
                        xl_b <= chs_spt;
                    end else begin
                        lba_acc <= lba_acc + {24'b0, lba0_r} - 32'd1;  // + sect-1
                        st <= ST_DISP;
                    end
                end
            end
            ST_DISP: begin                               // bounds check, registered
                bounds_bad <= ((lba_acc + {15'b0, sec_left}) > {1'b0, cap_lba});
                st <= ST_GO;
            end
            ST_GO: begin
                if (bounds_bad) begin
                    err_r <= 8'h10;                      // IDNF
                    st <= ST_FINERR;
                end else begin
                    dbg_start_r <= 1'b1;
                    blk_lba   <= lba_acc;
                    blk_nsec  <= sec_left;
                    blk_write <= c_wr_sec | c_wr_mul | c_wr_dma;
                    blk_req   <= 1'b1;
                    word_cnt  <= 13'd0;
                    if (c_rd_dma | c_wr_dma) begin
                        st_bsy <= 1'b1;
                        if (c_wr_dma) dev_go_req <= 1'b1;
                        st <= ST_DMA;
                    end else if (c_rd_sec | c_rd_mul) begin
                        st <= ST_PIN_W;
                    end else begin
                        st <= ST_POUT_W;
                    end
                end
            end
            // ---------------- PIO data-in -----------------------------------
            ST_PIN_W: begin
                st_bsy <= 1'b1; st_drq <= 1'b0;
                word_cnt <= 13'd0;
                if (blk_err_seen) begin
                    err_r <= 8'h40; st <= ST_FINERR;     // UNC from the media
                end else if (pin_src != SRC_FIFO
                    || (pin_ready_r && wsettle == 2'd2)) begin
                    st_bsy <= 1'b0; st_drq <= 1'b1;
                    intrq_r <= 1'b1;
                    st <= ST_PIN_D;
                end
            end
            ST_PIN_D: begin
                if (dp_rd_ev) begin
                    word_cnt <= word_cnt + 13'd1;
                    if (pin_src == SRC_FIFO) fifo_rd_pin <= 1'b1;
                    if (pin_src == SRC_ID)   id_addr <= id_addr + 8'd1;
                    if (word_cnt + 13'd1 == blk_words_w) begin
                        st_drq <= 1'b0;
                        if (last_blk) begin
                            sec_left <= 17'd0;
                            end_irq_mask <= 1'b1;        // per-block IRQs already given
                            st <= ST_FIN;
                        end else begin
                            sec_left <= sec_left - {12'b0, blk_secs};
                            st <= ST_PIN_W;
                        end
                    end
                end
            end
            // ---------------- PIO data-out ----------------------------------
            ST_POUT_W: begin
                st_bsy <= 1'b1; st_drq <= 1'b0;
                word_cnt <= 13'd0;
                if (blk_err_seen) begin
                    err_r <= 8'h40; st <= ST_FINERR;
                end else if (pout_room_r && wsettle == 2'd2) begin
                    st_bsy <= 1'b0; st_drq <= 1'b1;
                    if (!first_blk) intrq_r <= 1'b1;     // no IRQ for block 0
                    first_blk <= 1'b0;
                    st <= ST_POUT_D;
                end
            end
            ST_POUT_D: begin
                if (dp_wr_ev) begin
                    fifo_wr_pout      <= 1'b1;
                    fifo_wr_pout_data <= hs_dd_s;
                    word_cnt <= word_cnt + 13'd1;
                    if (word_cnt + 13'd1 == blk_words_w) begin
                        st_drq <= 1'b0; st_bsy <= 1'b1;
                        if (last_blk) begin
                            sec_left <= 17'd0;
                            st <= ST_POUT_E;
                        end else begin
                            sec_left <= sec_left - {12'b0, blk_secs};
                            st <= ST_POUT_W;
                        end
                    end
                end
            end
            ST_POUT_E: begin
                st_bsy <= 1'b1;
                if (blk_done_seen) begin
                    if (blk_err_seen) begin err_r <= 8'h40; st <= ST_FINERR; end
                    else st <= ST_FIN;
                end
            end
            // ---------------- UDMA -------------------------------------------
            ST_DMA: begin
                st_bsy <= 1'b1;
                if (blk_err_seen) begin
                    err_r <= 8'h40; dev_go_req <= 1'b0; st <= ST_FINERR;
                end else if (!dma_dir_write) begin
                    if (~dev_busy & ~dev_go_req & cons_ready)
                        dev_go_req <= 1'b1;
                    if (prod_complete & fifo_empty & ~dev_busy & ~dev_go_req)
                        st <= ST_DMA_E;
                end else begin
                    if (~dev_busy & ~dev_go_req & ~prod_complete & fifo_room_r)
                        dev_go_req <= 1'b1;
                    if (prod_complete & fifo_empty & ~dev_busy & ~dev_go_req)
                        st <= ST_DMA_E;
                end
            end
            ST_DMA_E: begin
                st_bsy <= 1'b1;
                if (blk_done_seen) begin
                    if (blk_err_seen) begin err_r <= 8'h40; st <= ST_FINERR; end
                    else st <= ST_FIN;
                end
            end
            // ---------------- misc -------------------------------------------
            ST_FLUSH:
                if (blk_flush_done) st <= ST_FIN;
            ST_DIAG:
                if (diag_tmr == 8'd0) begin
                    err_r <= 8'h01;
                    sig_set <= 1'b1;
                    st <= ST_FIN;
                end else diag_tmr <= diag_tmr - 8'd1;
            ST_IDWAIT:
                if (id_valid & ~id_fill_run) st <= ST_PIN_W;
            // ---------------- epilogues --------------------------------------
            ST_FIN: begin
                st_bsy <= 1'b0; st_drdy <= 1'b1; st_drq <= 1'b0; st_err <= 1'b0;
                if (!end_irq_mask) intrq_r <= 1'b1;
                end_irq_mask <= 1'b0;
                dbg_end_r <= 1'b1;
                st <= ST_IDLE;
            end
            ST_FINERR: begin
                st_bsy <= 1'b0; st_drdy <= 1'b1; st_drq <= 1'b0; st_err <= 1'b1;
                intrq_r <= 1'b1;
                abort_req <= 1'b1;                       // scrub any pending blk op
                dbg_end_r <= 1'b1;
                st <= ST_IDLE;
            end
            default: st <= ST_IDLE;
            endcase

            // trumps, strongest last
            if (wd_fire) begin
                err_r <= 8'h04; st_err <= 1'b1;
                st_bsy <= 1'b0; st_drdy <= 1'b1; st_drq <= 1'b0;
                dev_go_req <= 1'b0;
                abort_req <= 1'b1;
                st <= ST_IDLE;
            end
            if (srst_r && st != ST_SRST) begin
                st_bsy <= 1'b1; st_drq <= 1'b0; st_err <= 1'b0;
                abort_req <= 1'b1;
                dev_go_req <= 1'b0;
                st <= ST_SRST;
            end
        end
    end

    // ------------------------------------------------------------------------
    // FIFO ownership mux
    // ------------------------------------------------------------------------
    always @(*) begin
        if (cmdclass_dma & dma_dir_write) begin      // host -> engine -> FIFO -> backend
            fifo_wr_mux      = d_fifo_wr;
            fifo_wr_data_mux = d_fifo_wr_data;
            fifo_rd_mux      = bwr_rd;
        end else if (cmdclass_dma) begin             // backend -> FIFO -> engine -> host
            fifo_wr_mux      = brd_wr;
            fifo_wr_data_mux = brd_data;
            fifo_rd_mux      = d_fifo_rd;
        end else if (blk_write) begin                // PIO out
            fifo_wr_mux      = fifo_wr_pout;
            fifo_wr_data_mux = fifo_wr_pout_data;
            fifo_rd_mux      = bwr_rd;
        end else begin                               // PIO in
            fifo_wr_mux      = brd_wr;
            fifo_wr_data_mux = brd_data;
            fifo_rd_mux      = fifo_rd_pin;
        end
    end

    // ------------------------------------------------------------------------
    // register-file read mux + DD drive
    // ------------------------------------------------------------------------
    wire [15:0] pin_word = (pin_src == SRC_ID)   ? id_out
                         : (pin_src == SRC_ZERO) ? 16'h0000
                         :                         fifo_rd_data;

    reg [15:0] rd_val;
    always @(*) begin
        if (dev1) rd_val = 16'h0000;
        else if (sel_cmd) begin
            case (adr)
            3'd0: rd_val = st_drq ? pin_word : 16'h0000;
            3'd1: rd_val = {8'h00, err_r};
            3'd2: rd_val = {8'h00, sc_r};
            3'd3: rd_val = {8'h00, lba0_r};
            3'd4: rd_val = {8'h00, lba1_r};
            3'd5: rd_val = {8'h00, lba2_r};
            3'd6: rd_val = {8'h00, dev_r};
            default: rd_val = {8'h00, status};
            endcase
        end else begin
            rd_val = (adr == 3'd6) ? {8'h00, status} : 16'h0000;
        end
    end

    // During a UDMA command the engine owns DD when it drives; status polls
    // between grants read BSY (v1-proven). PIO register reads use the raw
    // IOR# for OE timing exactly like v1's passthrough did.
    wire dma_phase = (st == ST_DMA);
    wire stat_addr = (sel_cmd & (adr == 3'd7)) | (sel_ctl & (adr == 3'd6));
    wire reg_sel   = (sel_cmd | sel_ctl);
    assign hs_dd_out = dma_phase ? (dev_hs_dd_oe ? dev_hs_dd_out : 16'h0080)
                                 : rd_val;
    assign hs_dd_oe  = dma_phase ? (dev_hs_dd_oe | (stat_addr & ~hs_ior_n & dmack_s))
                                 : (reg_sel & ~hs_ior_n & ~dev1);

    assign hs_dmarq = dev_dmarq;
    assign hs_iordy = dma_phase ? dev_ddmardy_n : 1'b1;

endmodule
