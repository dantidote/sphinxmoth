// ============================================================================
// fpga_top_sd.v -- v2 synthesis wrapper for the LCMXO2-7000HC-4TG100 board
// (ipod-sd-udma). Internal OSCH @66.5 MHz, power-on reset counter, heartbeat
// LED, dbg_snoop UART logger on the host bus (POR-only reset, as v1).
// Pin LOCATEs: constraints/sd_tg100.lpf.
// ============================================================================

module fpga_top_sd (
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

    output        SD_CLK,
    inout         SD_CMD,
    inout  [3:0]  SD_DAT,
    input         SD_CD_N,

    output        LED,
    output        UART_TX,
    input         UART_RX
);

    wire clk;
`ifdef SIM
    reg sim_clk = 0;
    always #7.5 sim_clk = ~sim_clk;
    assign clk = sim_clk;
`else
    OSCH #(.NOM_FREQ("66.5")) osc (
        .STDBY   (1'b0),
        .OSC     (clk),
        .SEDSTDBY()
    );
`endif

    reg [7:0] por = 8'd0;
    wire rst_n = por[7];
    always @(posedge clk)
        if (!por[7]) por <= por + 8'd1;

    wire        dbg_abort, dbg_end, dbg_start, dbg_init_ok, dbg_init_bad;
    wire [7:0]  dbg_init_stat, dbg_stat, dbg_dmackf, dbg_retries;
    wire [15:0] dbg_wcap;
    wire        dbg_retry;

    ipod_sd_top #(
        .CLK_MHZ (66),
        .SD_FAST (0)                 // enable after the 33MHz sample point is
    ) core (                         // validated on silicon (see sd_host.v)
        .HS_DD      (HS_DD),
        .HS_CS0_N   (HS_CS0_N),
        .HS_CS1_N   (HS_CS1_N),
        .HS_A0      (HS_A0),
        .HS_A1      (HS_A1),
        .HS_A2      (HS_A2),
        .HS_IOR_N   (HS_IOR_N),
        .HS_IOW_N   (HS_IOW_N),
        .HS_DMARQ   (HS_DMARQ),
        .HS_DMACK_N (HS_DMACK_N),
        .HS_RESET_N (HS_RESET_N),
        .HS_INTRQ   (HS_INTRQ),
        .HS_IORDY   (HS_IORDY),
        .SD_CLK     (SD_CLK),
        .SD_CMD     (SD_CMD),
        .SD_DAT     (SD_DAT),
        .SD_CD_N    (SD_CD_N),
        .CLK        (clk),
        .RST_N      (rst_n),
        .DBG_ABORT  (dbg_abort),
        .DBG_END    (dbg_end),
        .DBG_START  (dbg_start),
        .DBG_INIT_OK (dbg_init_ok),
        .DBG_INIT_BAD(dbg_init_bad),
        .DBG_INIT_STAT(dbg_init_stat),
        .DBG_STAT   (dbg_stat),
        .DBG_WCAP   (dbg_wcap),
        .DBG_DMACKF (dbg_dmackf),
        .DBG_RETRY  (dbg_retry),
        .DBG_RETRIES(dbg_retries)
    );

    reg [26:0] hb = 27'd0;
    always @(posedge clk) hb <= hb + 27'd1;
    assign LED = hb[24];

    // taskfile logger on the host bus; POR-only reset so host resets are
    // logged. v2 event mapping: M/m = SD init ok/fail, G = SD retry count,
    // h = unused, K/k/W/w carry the FIFO word counters.
    dbg_snoop #(.DIV(577)) u_dbg (
        .clk   (clk),
        .rst_n (por[7]),
        .cs0_n (HS_CS0_N), .cs1_n(HS_CS1_N),
        .a0(HS_A0), .a1(HS_A1), .a2(HS_A2),
        .ior_n (HS_IOR_N), .iow_n(HS_IOW_N),
        .hrst_n(HS_RESET_N),
        .dd    (HS_DD),
        .abort_ev(dbg_abort), .end_ev(dbg_end), .start_ev(dbg_start), .stat(dbg_stat),
        .init_ok(dbg_init_ok), .init_bad(dbg_init_bad), .istat(dbg_init_stat),
        .bcrc(16'h0000), .bwcnt(dbg_wcap),
        .mode_ev(dbg_retry), .mode_val(dbg_retries),
        .chunk_ev(1'b0), .chunk_val(8'h00),
        .wcap(dbg_wcap), .dmackf(dbg_dmackf), .hostq(8'h00), .wsent(16'h0000),
        .txd   (UART_TX)
    );

endmodule
