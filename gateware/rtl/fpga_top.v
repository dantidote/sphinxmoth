// ============================================================================
// fpga_top.v -- synthesis wrapper for the LCMXO2-2000HC-4TG100C board
// (workspace/ipod-cf-udma). Internal OSCH @66.5 MHz, power-on reset counter,
// heartbeat LED, UART held idle (debug console gateware TBD).
// Pin LOCATEs: constraints/interposer_tg100.lpf.
// ============================================================================

module fpga_top (
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

    inout  [15:0] CF_DD,
    output        CF_CS0_N, CF_CS1_N,
    output        CF_A0, CF_A1, CF_A2,
    output        CF_IOR_N,
    output        CF_IOW_N,
    input         CF_DMARQ,
    output        CF_DMACK_N,
    output        CF_RESET_N,
    input         CF_INTRQ,
    input         CF_IORDY,

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

    wire       dbg_abort, dbg_end, dbg_start, dbg_init_ok, dbg_init_bad;

    wire [7:0] dbg_stat, dbg_init_stat;
    wire [15:0] dbg_crc, dbg_wcnt, dbg_wcap;
    wire        dbg_chunk;
    wire [7:0]  dbg_chunkv, dbg_dmackf, dbg_hostq;
    wire [15:0] dbg_wsent;

    interposer_top #(
        .CLK_MHZ (66)
    ) core (
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
        .CF_DD      (CF_DD),
        .CF_CS0_N   (CF_CS0_N),
        .CF_CS1_N   (CF_CS1_N),
        .CF_A0      (CF_A0),
        .CF_A1      (CF_A1),
        .CF_A2      (CF_A2),
        .CF_IOR_N   (CF_IOR_N),
        .CF_IOW_N   (CF_IOW_N),
        .CF_DMARQ   (CF_DMARQ),
        .CF_DMACK_N (CF_DMACK_N),
        .CF_RESET_N (CF_RESET_N),
        .CF_INTRQ   (CF_INTRQ),
        .CF_IORDY   (CF_IORDY),
        .CLK        (clk),
        .RST_N      (rst_n & HS_RESET_N),
        .DBG_ABORT  (dbg_abort),
        .DBG_END    (dbg_end),
        .DBG_START  (dbg_start),
        .DBG_INIT_OK (dbg_init_ok),
        .DBG_INIT_BAD(dbg_init_bad),
        .DBG_INIT_STAT(dbg_init_stat),
        .DBG_STAT   (dbg_stat),
        .DBG_CRC    (dbg_crc),
        .DBG_WCNT   (dbg_wcnt),
        .DBG_CHUNK  (dbg_chunk),
        .DBG_CHUNKV (dbg_chunkv),
        .DBG_WCAP   (dbg_wcap),
        .DBG_DMACKF (dbg_dmackf),
        .DBG_HOSTQ  (dbg_hostq),
        .DBG_WSENT  (dbg_wsent)
    );

    reg [26:0] hb = 27'd0;
    always @(posedge clk) hb <= hb + 27'd1;

    // LED: ~8 Hz flash while data moves (DMA bursts or data-register PIO,
    // held ~63 ms across burst gaps), heartbeat blip when idle
    wire cf_xfer = ~CF_DMACK_N
                 | (~CF_CS0_N & ({CF_A2,CF_A1,CF_A0} == 3'b000)
                    & (~CF_IOR_N | ~CF_IOW_N));
    reg [21:0] act_hold = 22'd0;
    always @(posedge clk)
        act_hold <= cf_xfer ? {22{1'b1}}
                  : (act_hold != 22'd0) ? act_hold - 22'd1 : 22'd0;
    assign LED = (act_hold != 22'd0) ? hb[22]
               : (hb[24] & hb[23] & hb[22]);

    // taskfile logger on the host bus; POR-only reset so host resets are logged
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
        .bcrc(dbg_crc), .bwcnt(dbg_wcnt),
        .mode_ev(1'b0), .mode_val(8'h00),
        .chunk_ev(dbg_chunk), .chunk_val(dbg_chunkv),
        .wcap(dbg_wcap), .dmackf(dbg_dmackf), .hostq(dbg_hostq), .wsent(dbg_wsent),
        .txd   (UART_TX)
    );

endmodule
