// ============================================================================
// tb_v2_err.v -- v2 fault paths: an SD read block with a corrupted CRC must
// be re-read (verify-then-release: the host NEVER sees the bad data), and a
// write block rejected by the card's CRC status token must be resent from
// the backend's private buffer. Both must leave the data perfect and only
// bump the retry counter.
// ============================================================================
`timescale 1ns/1ps

module tb_v2_err;
    reg clk = 0; always #7.5 clk = ~clk;
    reg rst_n = 0;

    wire [15:0] HS_DD;
    reg  [15:0] hs_drv; reg hs_oe;
    assign HS_DD = hs_oe ? hs_drv : 16'bz;
    reg HS_CS0_N=1, HS_CS1_N=1, HS_A0=0, HS_A1=0, HS_A2=0;
    reg HS_IOR_N=1, HS_IOW_N=1, HS_DMACK_N=1, HS_RESET_N=1;
    wire HS_DMARQ, HS_INTRQ, HS_IORDY;
    wire SD_CLK, SD_CMD;
    wire [3:0] SD_DAT;
    wire dbg_retry;
    wire [7:0] dbg_retries;

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
        .DBG_RETRY(dbg_retry),.DBG_RETRIES(dbg_retries));

    sd_bfm #(.CAP_SECTORS(65536), .MEM_SECTORS(1024)) u_bfm (
        .sd_clk(SD_CLK), .sd_cmd(SD_CMD), .sd_dat(SD_DAT));

    integer errs = 0;

    task hs_write(input [2:0] a, input [15:0] d);
        begin
            @(posedge clk); HS_CS0_N=0; HS_CS1_N=1; {HS_A2,HS_A1,HS_A0}=a;
            hs_drv=d; hs_oe=1; @(posedge clk); HS_IOW_N=0;
            repeat(4) @(posedge clk); HS_IOW_N=1;
            repeat(2) @(posedge clk); HS_CS0_N=1; hs_oe=0; {HS_A2,HS_A1,HS_A0}=0;
            repeat(2) @(posedge clk);
        end
    endtask
    task hs_read(input [2:0] a, output [15:0] d);
        begin
            @(posedge clk); HS_CS0_N=0; HS_CS1_N=1; {HS_A2,HS_A1,HS_A0}=a;
            @(posedge clk); HS_IOR_N=0;
            repeat(5) @(posedge clk);
            d = HS_DD;
            HS_IOR_N=1;
            repeat(2) @(posedge clk); HS_CS0_N=1; {HS_A2,HS_A1,HS_A0}=0;
            @(posedge clk);
        end
    endtask
    reg [15:0] s, d;
    task wait_ready;
        integer guard;
        begin
            guard = 0; s = 16'h80;
            while ((s[7] || !s[6]) && guard < 8000000) begin
                hs_read(3'd7, s); guard = guard + 1;
            end
            if (guard >= 8000000) begin errs=errs+1; $display("  FAIL wait_ready timeout"); end
        end
    endtask
    task wait_drq;
        integer guard;
        begin
            guard = 0; s = 16'h80;
            while ((s[7] || !s[3]) && guard < 8000000) begin
                hs_read(3'd7, s); guard = guard + 1;
            end
            if (guard >= 8000000) begin errs=errs+1; $display("  FAIL wait_drq timeout"); end
        end
    endtask

    integer i, k;
    initial begin
        $dumpfile("tb_v2_err.vcd"); $dumpvars(0, tb_v2_err);
        hs_oe = 0;
        for (i = 0; i < 1024*256; i = i + 1) u_bfm.mem[i] = (i*16'h0031) ^ 16'h0F1E;
        #200 rst_n = 1;
        wait_ready;

        // ---- read with an injected CRC error on the 2nd block ----
        u_bfm.inj_rd_crc_countdown = 2;
        hs_write(3'd2, 16'h0004);
        hs_write(3'd3, 16'h0014);                     // LBA 20
        hs_write(3'd4, 16'h0000);
        hs_write(3'd5, 16'h0000);
        hs_write(3'd6, 16'h00E0);
        hs_write(3'd7, 16'h0020);                     // READ SECTORS x4
        for (k = 0; k < 4; k = k + 1) begin
            wait_drq;
            for (i = 0; i < 256; i = i + 1) begin
                hs_read(3'd0, d);
                if (d !== u_bfm.mem[(20+k)*256+i]) begin
                    if (errs < 8) $display("  FAIL rd s%0d w%0d: %04x exp %04x",
                                           k, i, d, u_bfm.mem[(20+k)*256+i]);
                    errs = errs + 1;
                end
            end
        end
        wait_ready;
        hs_read(3'd7, s);
        if (s[7:0] !== 8'h50) begin errs=errs+1; $display("  FAIL rd status %02x", s[7:0]); end
        if (dbg_retries !== 8'd1) begin
            errs = errs + 1;
            $display("  FAIL: expected 1 retry after read CRC injection, got %0d", dbg_retries);
        end
        $display("[%0t] read CRC-error retry ok (retries=%0d)", $time, dbg_retries);

        // ---- write with the 1st block rejected by the CRC token ----
        u_bfm.inj_wr_reject_countdown = 1;
        hs_write(3'd2, 16'h0002);
        hs_write(3'd3, 16'h0030);                     // LBA 48
        hs_write(3'd4, 16'h0000);
        hs_write(3'd5, 16'h0000);
        hs_write(3'd6, 16'h00E0);
        hs_write(3'd7, 16'h0030);                     // WRITE SECTORS x2
        for (k = 0; k < 2; k = k + 1) begin
            wait_drq;
            for (i = 0; i < 256; i = i + 1)
                hs_write(3'd0, (k*256+i) ^ 16'hC33C);
        end
        wait_ready;
        hs_read(3'd7, s);
        if (s[7:0] !== 8'h50) begin errs=errs+1; $display("  FAIL wr status %02x", s[7:0]); end
        for (i = 0; i < 512; i = i + 1)
            if (u_bfm.mem[48*256+i] !== (i ^ 16'hC33C)) begin
                if (errs < 8) $display("  FAIL wr w%0d: %04x exp %04x",
                                       i, u_bfm.mem[48*256+i], i ^ 16'hC33C);
                errs = errs + 1;
            end
        if (dbg_retries !== 8'd2) begin
            errs = errs + 1;
            $display("  FAIL: expected 2 total retries after write reject, got %0d", dbg_retries);
        end
        $display("[%0t] write-reject retry ok (retries=%0d)", $time, dbg_retries);

        if (errs == 0) $display("PASS: v2 fault paths (verify-then-release + resend)");
        else           $display("FAIL: %0d errors", errs);
        $finish;
    end

    initial begin #200_000_000 $display("TIMEOUT"); $finish; end
endmodule
