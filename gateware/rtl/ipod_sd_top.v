// ============================================================================
// ipod_sd_top.v  --  v2 core: iPod 50-pin ATA target over a native microSD
// ----------------------------------------------------------------------------
//   iPod 50-pin <-> ata_device (full ATA target) <-> backend_sd <-> sd_host
//
// Two reset domains (the v2 design doc's one structural rule):
//   rst_ata_n = POR & HS_RESET_N  -> ata_device (ATA-visible state)
//   rst_n     = POR only          -> backend_sd + sd_host (card must survive
//                                    the iPod's boot-time reset storms)
// ata_device.abort_req (reset value 1) is the bridge between them: any ata
// death tells the backend to close its SD op cleanly.
// ============================================================================

module ipod_sd_top #(
    parameter CLK_MHZ  = 66,
    parameter SD_FAST  = 0,
    parameter DIV_INIT = 84,
    parameter DIV_XFER = 2,
    parameter SIM_FAST = 0
) (
    // ---- host side: iPod 50-pin --------------------------------------------
    inout  [15:0] HS_DD,
    input         HS_CS0_N, HS_CS1_N,
    input         HS_A0, HS_A1, HS_A2,
    input         HS_IOR_N,
    input         HS_IOW_N,
    output        HS_DMARQ,
    input         HS_DMACK_N,
    input         HS_RESET_N,
    output        HS_INTRQ,
    output        HS_IORDY,

    // ---- microSD -------------------------------------------------------------
    output        SD_CLK,
    inout         SD_CMD,
    inout  [3:0]  SD_DAT,
    input         SD_CD_N,

    // ---- local ----------------------------------------------------------------
    input         CLK,
    input         RST_N,          // POR only

    // ---- debug taps (for the UART logger) ---------------------------------------
    output        DBG_ABORT,
    output        DBG_END,
    output        DBG_START,
    output        DBG_INIT_OK,    // 1-cycle: SD init completed
    output        DBG_INIT_BAD,   // 1-cycle: SD init failed (will retry)
    output [7:0]  DBG_INIT_STAT,  // {ccs, hs_on, init_stage, 2'b00}
    output [7:0]  DBG_STAT,
    output [15:0] DBG_WCAP,
    output [7:0]  DBG_DMACKF,
    output        DBG_RETRY,      // 1-cycle: SD block retried
    output [7:0]  DBG_RETRIES
);

    // ------------------------------------------------------------------------
    // reset domains
    // ------------------------------------------------------------------------
    reg [1:0] hrst_q;
    always @(posedge CLK or negedge RST_N)
        if (!RST_N) hrst_q <= 2'b00;
        else        hrst_q <= {hrst_q[0], HS_RESET_N};
    wire rst_ata_n = RST_N & hrst_q[1];

    // ------------------------------------------------------------------------
    // ata_device
    // ------------------------------------------------------------------------
    wire [15:0] ata_dd_out;
    wire        ata_dd_oe;
    wire        blk_req, blk_write, blk_busy, blk_done, blk_err;
    wire [31:0] blk_lba;
    wire [16:0] blk_nsec;
    wire        blk_flush, blk_flush_done, blk_ready;
    wire [31:0] blk_capacity;
    wire        abort_req;
    wire        brd_wr, brd_full, bwr_avail, bwr_rd;
    wire [15:0] brd_data, bwr_data;

    ata_device #(.CLK_MHZ(CLK_MHZ)) u_ata (
        .clk(CLK), .rst_n(rst_ata_n),
        .hs_dd_in(HS_DD), .hs_dd_out(ata_dd_out), .hs_dd_oe(ata_dd_oe),
        .hs_cs0_n(HS_CS0_N), .hs_cs1_n(HS_CS1_N),
        .hs_a0(HS_A0), .hs_a1(HS_A1), .hs_a2(HS_A2),
        .hs_ior_n(HS_IOR_N), .hs_iow_n(HS_IOW_N),
        .hs_dmarq(HS_DMARQ), .hs_dmack_n(HS_DMACK_N),
        .hs_intrq(HS_INTRQ), .hs_iordy(HS_IORDY),
        .blk_req(blk_req), .blk_write(blk_write),
        .blk_lba(blk_lba), .blk_nsec(blk_nsec),
        .blk_busy(blk_busy), .blk_done(blk_done), .blk_err(blk_err),
        .blk_flush(blk_flush), .blk_flush_done(blk_flush_done),
        .abort_req(abort_req),
        .blk_ready(blk_ready), .blk_capacity(blk_capacity),
        .brd_wr(brd_wr), .brd_data(brd_data), .brd_full(brd_full),
        .bwr_avail(bwr_avail), .bwr_rd(bwr_rd), .bwr_data(bwr_data),
        .dbg_start(DBG_START), .dbg_end(DBG_END), .dbg_abort(DBG_ABORT),
        .dbg_stat(DBG_STAT), .dbg_wcap(DBG_WCAP), .dbg_dmackf(DBG_DMACKF),
        .pp_sess_o()
    );

    assign HS_DD = ata_dd_oe ? ata_dd_out : 16'bz;

    // abort covers: ata-side aborts AND the window where the ata core itself
    // is held in reset by the host reset line
    wire blk_abort = abort_req | ~rst_ata_n;

    // ------------------------------------------------------------------------
    // sd_host + backend
    // ------------------------------------------------------------------------
    wire        sd_cmd_out, sd_cmd_oe, sd_cmd_in;
    wire [3:0]  sd_dat_out, sd_dat_in;
    wire        sd_dat_oe;
    wire        sd_init_done, sd_init_fail;
    wire [3:0]  sd_init_stage;
    wire [31:0] sd_capacity;
    wire        sd_ccs, sd_hs_on;
    wire        sd_op_go, sd_op_write, sd_op_open, sd_blk_go, sd_blk_done;
    wire        sd_blk_crc_ok, sd_wr_acc, sd_op_end, sd_op_idle, sd_op_err;
    wire [31:0] sd_op_lba;
    wire        sd_rd_v;
    wire [15:0] sd_rd_w;
    wire [7:0]  sd_wr_idx;
    wire [15:0] sd_wr_word;

    sd_host #(
        .CLK_MHZ(CLK_MHZ), .SD_FAST(SD_FAST),
        .DIV_INIT(DIV_INIT), .DIV_XFER(DIV_XFER), .SIM_FAST(SIM_FAST)
    ) u_sd (
        .clk(CLK), .rst_n(RST_N),
        .sd_clk(SD_CLK),
        .sd_cmd_out(sd_cmd_out), .sd_cmd_oe(sd_cmd_oe), .sd_cmd_in(sd_cmd_in),
        .sd_dat_out(sd_dat_out), .sd_dat_oe(sd_dat_oe), .sd_dat_in(sd_dat_in),
        .sd_cd_n(SD_CD_N),
        .init_done(sd_init_done), .init_fail(sd_init_fail),
        .init_stage(sd_init_stage),
        .capacity(sd_capacity), .ccs(sd_ccs), .hs_on(sd_hs_on),
        .op_go(sd_op_go), .op_write(sd_op_write), .op_lba(sd_op_lba),
        .op_open(sd_op_open),
        .blk_go(sd_blk_go), .blk_done(sd_blk_done), .blk_crc_ok(sd_blk_crc_ok),
        .wr_acc(sd_wr_acc),
        .op_end(sd_op_end), .op_idle(sd_op_idle), .op_err(sd_op_err),
        .rd_v(sd_rd_v), .rd_w(sd_rd_w),
        .wr_idx(sd_wr_idx), .wr_word(sd_wr_word)
    );

    assign SD_CMD = sd_cmd_oe ? sd_cmd_out : 1'bz;
    assign sd_cmd_in = SD_CMD;
    assign SD_DAT = sd_dat_oe ? sd_dat_out : 4'bz;
    assign sd_dat_in = SD_DAT;

    backend_sd u_be (
        .clk(CLK), .rst_n(RST_N),
        .blk_req(blk_req), .blk_write(blk_write),
        .blk_lba(blk_lba), .blk_nsec(blk_nsec),
        .blk_busy(blk_busy), .blk_done(blk_done), .blk_err(blk_err),
        .blk_flush(blk_flush), .blk_flush_done(blk_flush_done),
        .blk_abort(blk_abort),
        .blk_ready(blk_ready), .blk_capacity(blk_capacity),
        .brd_wr(brd_wr), .brd_data(brd_data), .brd_full(brd_full),
        .bwr_avail(bwr_avail), .bwr_rd(bwr_rd), .bwr_data(bwr_data),
        .sd_init_done(sd_init_done), .sd_capacity(sd_capacity),
        .sd_op_go(sd_op_go), .sd_op_write(sd_op_write), .sd_op_lba(sd_op_lba),
        .sd_op_open(sd_op_open),
        .sd_blk_go(sd_blk_go), .sd_blk_done(sd_blk_done),
        .sd_blk_crc_ok(sd_blk_crc_ok), .sd_wr_acc(sd_wr_acc),
        .sd_op_end(sd_op_end), .sd_op_idle(sd_op_idle), .sd_op_err(sd_op_err),
        .sd_rd_v(sd_rd_v), .sd_rd_w(sd_rd_w),
        .sd_wr_idx(sd_wr_idx), .sd_wr_word(sd_wr_word),
        .dbg_retry_ev(DBG_RETRY), .dbg_retries(DBG_RETRIES)
    );

    // ------------------------------------------------------------------------
    // SD init edge events for the logger
    // ------------------------------------------------------------------------
    reg init_q, fail_q;
    always @(posedge CLK or negedge RST_N)
        if (!RST_N) begin init_q <= 1'b0; fail_q <= 1'b0; end
        else        begin init_q <= sd_init_done; fail_q <= sd_init_fail; end
    assign DBG_INIT_OK   = sd_init_done & ~init_q;
    assign DBG_INIT_BAD  = sd_init_fail & ~fail_q;
    assign DBG_INIT_STAT = {sd_ccs, sd_hs_on, sd_init_stage, 2'b00};

endmodule
