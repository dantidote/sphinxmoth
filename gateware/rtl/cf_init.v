// ============================================================================
// cf_init.v -- post-reset CF mode fixup. The iSphynx never issues SET FEATURES
// (the original Toshiba drive powers up with UDMA2 already selected), so after
// every CF reset WE select UDMA2 on the card: SET FEATURES 03h, count 42h.
// While `active`, the top holds the host off with BSY and we own the CF bus.
// ============================================================================

module cf_init #(
    parameter CLK_MHZ = 66
) (
    input             clk,
    input             rst_n,        // includes host reset: re-arms on every reset

    output reg        active,       // we own the CF bus / host sees BSY
    output reg        rst_drive,    // assert CF_RESET_N low (clean post-power reset)

    // CF bus (valid while active)
    output reg        cs0_n,
    output reg [2:0]  addr,
    output reg        ior_n,
    output reg        iow_n,
    output reg [15:0] dd_out,
    output reg        dd_oe,
    input      [15:0] dd_in,        // synced CF_DD

    output reg        done_ev,      // 1-cycle: mode set, card ready
    output reg        fail_ev,      // 1-cycle: gave up (timeout)
    output reg [7:0]  last_status   // final CF status we saw
);

    // ---- single PIO cycle (slow, mode-0-ish) ----
    localparam XC_SETUP = 8'd12, XC_STROBE = 8'd40, XC_HOLD = 8'd8, XC_REC = 8'd20;   // ticks @66.5MHz
    reg        xgo, xop;            // op: 0=read 1=write
    reg  [2:0] xaddr;
    reg [15:0] xwdata, xrdata;
    reg        xbusy;
    reg  [7:0] xc;
    reg  [2:0] xs;
    localparam X_IDLE=0, X_SETUP=1, X_STROBE=2, X_HOLD=3, X_REC=4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xs <= X_IDLE; xbusy <= 0; xc <= 0;
            cs0_n <= 1'b1; ior_n <= 1'b1; iow_n <= 1'b1; addr <= 3'd0;
            dd_oe <= 1'b0; dd_out <= 16'h0; xrdata <= 16'h0;
        end else begin
            case (xs)
            X_IDLE: if (xgo) begin
                xbusy <= 1'b1; cs0_n <= 1'b0; addr <= xaddr;
                if (xop) begin dd_out <= xwdata; dd_oe <= 1'b1; end
                xc <= XC_SETUP; xs <= X_SETUP;
            end
            X_SETUP: if (xc==0) begin
                if (xop) iow_n <= 1'b0; else ior_n <= 1'b0;
                xc <= XC_STROBE; xs <= X_STROBE;
            end else xc <= xc - 1'b1;
            X_STROBE: if (xc==0) begin
                if (!xop) xrdata <= dd_in;
                ior_n <= 1'b1; iow_n <= 1'b1;
                xc <= XC_HOLD; xs <= X_HOLD;
            end else xc <= xc - 1'b1;
            X_HOLD: if (xc==0) begin
                cs0_n <= 1'b1; dd_oe <= 1'b0;
                xc <= XC_REC; xs <= X_REC;
            end else xc <= xc - 1'b1;
            X_REC: if (xc==0) begin xbusy <= 1'b0; xs <= X_IDLE; end
                   else xc <= xc - 1'b1;
            default: xs <= X_IDLE;
            endcase
        end
    end

    // ---- sequencer ----
    // wait 2ms after reset -> clean 1ms CF reset pulse -> poll status (2ms
    // cadence) until ready -> Features=03 -> Count -> Command=EF -> poll -> done
    localparam T_2MS   = 24'd133000;  // @66.5MHz
    localparam T_1MS   = 24'd66500;
    localparam BUDGET1 = 15'd2500;    // 2500 * 2ms = 5s: cards init <1s; short
                                      // enough that a no-card boot fails fast
    localparam BUDGET2 = 15'd50;      // 100ms

    localparam I_WAIT=0, I_PGO=1, I_PWAIT=2, I_PEVAL=3,
               I_F=4, I_FW=5, I_C=6, I_CW=7, I_E=8, I_EW=9,
               I_P2GO=10, I_P2W=11, I_P2EVAL=12, I_END=13,
               I_SG2=14, I_SG2W=15, I_SG3=16, I_SG3W=17,
               I_SG4=18, I_SG4W=19, I_SG5=20, I_SG5W=21,
               I_DGO=22, I_DW=23, I_RST0=24, I_RST=25;
    reg [4:0]  is_;
    reg [23:0] tick;
    reg [14:0] budget;
    reg [2:0]  dump_idx;   // fail autopsy: regs 1..6 (err, SC, LBA0/1/2, dev)

`ifdef SIM
    // simulation: skip the whole CF init handshake, release the bus immediately
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin active <= 1'b1; done_ev <= 0; rst_drive <= 1'b0; end
        else begin active <= 1'b0; done_ev <= 1'b1; rst_drive <= 1'b0; end
    end
    // tie off the PIO engine
    always @(*) begin xgo=0; xop=0; xaddr=0; xwdata=0; end
`else
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_ <= I_RST0; tick <= T_2MS; budget <= BUDGET1;
            active <= 1'b1; done_ev <= 0; fail_ev <= 0; last_status <= 8'h00;
            xgo <= 0; xop <= 0; xaddr <= 0; xwdata <= 0; rst_drive <= 1'b0;
        end else begin
            done_ev <= 0; fail_ev <= 0; xgo <= 0;
            case (is_)
            // clean CF reset after the rail settles (card may never have seen a
            // good one: the iPod's pulse fires during power-up chaos)
            I_RST0: if (tick==0) begin rst_drive <= 1'b1; tick <= T_1MS; is_ <= I_RST; end
                    else tick <= tick - 1'b1;
            I_RST:  if (tick==0) begin rst_drive <= 1'b0; tick <= T_2MS; is_ <= I_WAIT; end
                    else tick <= tick - 1'b1;
            I_WAIT: if (tick==0) is_ <= I_PGO;
                    else tick <= tick - 1'b1;
            I_PGO:  begin xop<=0; xaddr<=3'd7; xgo<=1; is_ <= I_PWAIT; end
            I_PWAIT: if (!xgo && !xbusy) begin last_status <= xrdata[7:0]; is_ <= I_PEVAL; end
            I_PEVAL: begin
                if (!last_status[7] && last_status[6]) is_ <= I_F;       // ready
                else if (budget==0) begin fail_ev<=1; dump_idx<=3'd1; is_<=I_DGO; end
                else begin budget <= budget-1'b1; tick <= T_2MS; is_ <= I_WAIT; end
            end
            I_F:  begin xop<=1; xaddr<=3'd1; xwdata<=16'h0003; xgo<=1; is_<=I_FW; end
            I_FW: if (!xgo && !xbusy) is_ <= I_C;
            I_C:  begin xop<=1; xaddr<=3'd2; xwdata<=16'h0040; xgo<=1; is_<=I_CW; end  // UDMA0: capture-safe at 66.5MHz
            I_CW: if (!xgo && !xbusy) is_ <= I_E;
            I_E:  begin xop<=1; xaddr<=3'd7; xwdata<=16'h00EF; xgo<=1; is_<=I_EW; end  // SET FEATURES
            I_EW: if (!xgo && !xbusy) begin budget <= BUDGET2; tick <= T_2MS; is_ <= I_P2GO; end
            I_P2GO: if (tick==0) begin xop<=0; xaddr<=3'd7; xgo<=1; is_<=I_P2W; end
                    else tick <= tick - 1'b1;
            I_P2W: if (!xgo && !xbusy) begin last_status <= xrdata[7:0]; is_ <= I_P2EVAL; end
            I_P2EVAL: begin
                if (!last_status[7]) is_ <= I_SG2;                   // BSY clear -> restore signature
                else if (budget==0) begin fail_ev<=1; dump_idx<=3'd1; is_<=I_DGO; end
                else begin budget <= budget-1'b1; tick <= T_2MS; is_ <= I_P2GO; end
            end
            // restore the post-reset ATA signature the FC1307A fails to reload
            // (our EF also clobbered Sector Count): SC=01, LBA 01/00/00
            I_SG2:  begin xop<=1; xaddr<=3'd2; xwdata<=16'h0001; xgo<=1; is_<=I_SG2W; end
            I_SG2W: if (!xgo && !xbusy) is_ <= I_SG3;
            I_SG3:  begin xop<=1; xaddr<=3'd3; xwdata<=16'h0001; xgo<=1; is_<=I_SG3W; end
            I_SG3W: if (!xgo && !xbusy) is_ <= I_SG4;
            I_SG4:  begin xop<=1; xaddr<=3'd4; xwdata<=16'h0000; xgo<=1; is_<=I_SG4W; end
            I_SG4W: if (!xgo && !xbusy) is_ <= I_SG5;
            I_SG5:  begin xop<=1; xaddr<=3'd5; xwdata<=16'h0000; xgo<=1; is_<=I_SG5W; end
            I_SG5W: if (!xgo && !xbusy) begin done_ev<=1; active<=0; is_<=I_END; end
            // fail autopsy: dump regs 1..6 as extra 'm' events (err, SC, LBA0/1/2, dev).
            // dead/floating bus -> all bytes identical junk; live-but-stuck card -> real signature.
            I_DGO: begin xop<=0; xaddr<=dump_idx; xgo<=1; is_<=I_DW; end
            I_DW:  if (!xgo && !xbusy) begin
                       last_status <= xrdata[7:0]; fail_ev <= 1;
                       if (dump_idx==3'd6) begin active<=0; is_<=I_END; end
                       else begin dump_idx <= dump_idx+3'd1; is_ <= I_DGO; end
                   end
            I_END: ;                                   // parked until next reset
            default: is_ <= I_END;
            endcase
        end
    end
`endif

endmodule
