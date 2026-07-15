// tb_v2_dbg.v -- init-path microscope: watches sd_host master/CE/DE states.
`timescale 1ns/1ps

module tb_v2_dbg;
    reg clk = 0; always #7.5 clk = ~clk;
    reg rst_n = 0;

    wire [15:0] HS_DD;
    reg HS_CS0_N=1, HS_CS1_N=1, HS_A0=0, HS_A1=0, HS_A2=0;
    reg HS_IOR_N=1, HS_IOW_N=1, HS_DMACK_N=1, HS_RESET_N=1;
    wire HS_DMARQ, HS_INTRQ, HS_IORDY;
    wire SD_CLK, SD_CMD;
    wire [3:0] SD_DAT;

    ipod_sd_top #(
        .CLK_MHZ(66), .SD_FAST(0), .DIV_INIT(4), .DIV_XFER(2), .SIM_FAST(1)
    ) dut (
        .HS_DD(HS_DD),.HS_CS0_N(HS_CS0_N),.HS_CS1_N(HS_CS1_N),
        .HS_A0(HS_A0),.HS_A1(HS_A1),.HS_A2(HS_A2),
        .HS_IOR_N(HS_IOR_N),.HS_IOW_N(HS_IOW_N),.HS_DMARQ(HS_DMARQ),
        .HS_DMACK_N(HS_DMACK_N),.HS_RESET_N(HS_RESET_N),
        .HS_INTRQ(HS_INTRQ),.HS_IORDY(HS_IORDY),
        .SD_CLK(SD_CLK),.SD_CMD(SD_CMD),.SD_DAT(SD_DAT),.SD_CD_N(1'b0),
        .CLK(clk),.RST_N(rst_n),
        .DBG_ABORT(),.DBG_END(),.DBG_START(),.DBG_INIT_OK(),.DBG_INIT_BAD(),
        .DBG_INIT_STAT(),.DBG_STAT(),.DBG_WCAP(),.DBG_DMACKF(),
        .DBG_RETRY(),.DBG_RETRIES());

    sd_bfm #(.CAP_SECTORS(65536), .MEM_SECTORS(16)) u_bfm (
        .sd_clk(SD_CLK), .sd_cmd(SD_CMD), .sd_dat(SD_DAT));

    reg [4:0] m_p = 5'h1F;
    always @(posedge clk) begin
        if (dut.u_sd.m != m_p) begin
            $display("[%0t] sd m %0d -> %0d  (ce=%0d de=%0d stage=%0d fail=%b done=%b)",
                     $time, m_p, dut.u_sd.m, dut.u_sd.ce, dut.u_sd.de,
                     dut.u_sd.init_stage, dut.u_sd.init_fail, dut.u_sd.init_done);
            m_p = dut.u_sd.m;
        end
    end
    always @(posedge clk) if (dut.u_sd.ce_err)
        $display("[%0t] CE_ERR in m=%0d", $time, dut.u_sd.m);
    always @(posedge clk) if (u_bfm.cmd_crc_errs != crcp) begin
        crcp = u_bfm.cmd_crc_errs;
    end
    integer crcp = 0;

    reg [4:0] ast_p = 5'h1F;
    always @(posedge clk) begin
        if (dut.u_ata.st != ast_p) begin
            $display("[%0t] ata st %0d -> %0d (blk_ready=%b id_valid=%b)",
                     $time, ast_p, dut.u_ata.st, dut.blk_ready, dut.u_ata.id_valid);
            ast_p = dut.u_ata.st;
        end
    end

    initial begin
        #200 rst_n = 1;
        #6_000_000;
        $display("END: m=%0d init_done=%b fail=%b stage=%0d cap=%0d ccs=%b",
                 dut.u_sd.m, dut.u_sd.init_done, dut.u_sd.init_fail,
                 dut.u_sd.init_stage, dut.u_sd.capacity, dut.u_sd.ccs);
        $display("     ata st=%0d id_valid=%b id_fill_run=%b div_run=%b",
                 dut.u_ata.st, dut.u_ata.id_valid, dut.u_ata.id_fill_run,
                 dut.u_ata.div_run);
        $display("     bfm cmd_crc_errs=%0d cstate=%0d a41=%0d",
                 u_bfm.cmd_crc_errs, u_bfm.cstate, u_bfm.a41);
        $finish;
    end
endmodule
