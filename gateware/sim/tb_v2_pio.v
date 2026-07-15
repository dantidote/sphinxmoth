// ============================================================================
// tb_v2_pio.v -- v2 PP5002-flavored session: boot, SD init, signature,
// IDENTIFY (pristine + pp_sess variants, checksum), READ SECTORS, SET/WRITE
// MULTIPLE + readback, SET FEATURES accept/abort, CHECK POWER MODE, FLUSH.
// Host model = PIO only, taskfile pacing from the v1 calibrated models.
// ============================================================================
`timescale 1ns/1ps

module tb_v2_pio;
    reg clk = 0; always #7.5 clk = ~clk;
    reg rst_n = 0;

    // ---- host bus ----
    wire [15:0] HS_DD;
    reg  [15:0] hs_drv; reg hs_oe;
    assign HS_DD = hs_oe ? hs_drv : 16'bz;
    reg HS_CS0_N=1, HS_CS1_N=1, HS_A0=0, HS_A1=0, HS_A2=0;
    reg HS_IOR_N=1, HS_IOW_N=1, HS_DMACK_N=1, HS_RESET_N=1;
    wire HS_DMARQ, HS_INTRQ, HS_IORDY;

    // ---- SD bus ----
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

    sd_bfm #(.CAP_SECTORS(65536), .MEM_SECTORS(1024)) u_bfm (
        .sd_clk(SD_CLK), .sd_cmd(SD_CMD), .sd_dat(SD_DAT));

    integer errs = 0;
    task expect16(input [255:0] what, input [15:0] got, input [15:0] exp);
        if (got !== exp) begin
            errs = errs + 1;
            $display("  FAIL %0s: got %04x exp %04x", what, got, exp);
        end
    endtask

    // ---- PIO host primitives (v1 pacing) ----
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
    task hs_read_alt(output [15:0] d);       // alt-status: no INTRQ side effect
        begin
            @(posedge clk); HS_CS0_N=1; HS_CS1_N=0; {HS_A2,HS_A1,HS_A0}=3'd6;
            @(posedge clk); HS_IOR_N=0;
            repeat(5) @(posedge clk);
            d = HS_DD;
            HS_IOR_N=1;
            repeat(2) @(posedge clk); HS_CS1_N=1; {HS_A2,HS_A1,HS_A0}=0;
            @(posedge clk);
        end
    endtask

    reg [15:0] s;
    task wait_ready;                      // BSY=0 DRDY=1
        integer guard;
        begin
            guard = 0;
            s = 16'h80;
            while ((s[7] || !s[6]) && guard < 4000000) begin
                hs_read(3'd7, s); guard = guard + 1;
            end
            if (guard >= 4000000) begin errs=errs+1; $display("  FAIL wait_ready timeout"); end
        end
    endtask
    task wait_drq;
        integer guard;
        begin
            guard = 0;
            s = 16'h80;
            while ((s[7] || !s[3]) && guard < 4000000) begin
                hs_read(3'd7, s); guard = guard + 1;
            end
            if (guard >= 4000000) begin errs=errs+1; $display("  FAIL wait_drq timeout"); end
        end
    endtask

    reg [15:0] idbuf [0:255];
    reg [15:0] d;
    integer i, k, sum;

    task read_id_block;                   // 256 words from the data port
        begin
            for (i = 0; i < 256; i = i + 1) begin
                hs_read(3'd0, d);
                idbuf[i] = d;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_v2_pio.vcd"); $dumpvars(0, tb_v2_pio);
        hs_oe = 0;
        // BFM backing pattern
        for (i = 0; i < 1024*256; i = i + 1) u_bfm.mem[i] = (i*16'h0007) ^ 16'hA5C3;
        #200 rst_n = 1;

        // ---- boot: BSY until SD init, then signature ----
        $display("[%0t] waiting for boot...", $time);
        wait_ready;
        hs_read(3'd2, d); expect16("sig SC", d[7:0], 8'h01);
        hs_read(3'd3, d); expect16("sig LBA0", d[7:0], 8'h01);
        hs_read(3'd4, d); expect16("sig LBA1", d[7:0], 8'h00);
        hs_read(3'd5, d); expect16("sig LBA2", d[7:0], 8'h00);
        hs_read(3'd1, d); expect16("sig ERR", d[7:0], 8'h01);
        $display("[%0t] boot + signature ok", $time);

        // ---- IDENTIFY, pristine session ----
        // poll via ALT-STATUS: DRQ visible with INTRQ still pending; one
        // status read must then clear INTRQ (spec semantics)
        hs_write(3'd7, 16'h00EC);
        begin : id_altpoll
            integer guard;
            guard = 0; s = 16'h80;
            while ((s[7] || !s[3]) && guard < 4000000) begin
                hs_read_alt(s); guard = guard + 1;
            end
        end
        if (HS_INTRQ !== 1'b1) begin errs=errs+1; $display("  FAIL: no INTRQ at ID DRQ"); end
        hs_read(3'd7, s);
        if (HS_INTRQ !== 1'b0) begin errs=errs+1; $display("  FAIL: status read did not clear INTRQ"); end
        read_id_block;
        expect16("ID w0",  idbuf[0],  16'h0040);
        expect16("ID w1 (cyl 65536/1008=65)", idbuf[1], 16'd65);
        expect16("ID w47", idbuf[47], 16'h8010);
        expect16("ID w59 (mult=16 default)", idbuf[59], 16'h0110);
        expect16("ID w60", idbuf[60], 16'h0000);
        expect16("ID w61", idbuf[61], 16'h0001);
        expect16("ID w63", idbuf[63], 16'h0000);
        expect16("ID w88 pristine", idbuf[88], 16'h0407);
        if (idbuf[255][7:0] !== 8'hA5) begin errs=errs+1; $display("  FAIL: w255 sig"); end
        sum = 0;
        for (i = 0; i < 256; i = i + 1) sum = sum + idbuf[i][7:0] + idbuf[i][15:8];
        if (sum[7:0] !== 8'h00) begin errs=errs+1; $display("  FAIL: ID checksum %02x", sum[7:0]); end
        wait_ready;
        $display("[%0t] IDENTIFY pristine ok", $time);

        // ---- EXECUTE DIAGNOSTIC -> pp_sess ----
        hs_write(3'd7, 16'h0090);
        wait_ready;
        hs_read(3'd1, d); expect16("diag err", d[7:0], 8'h01);
        hs_write(3'd7, 16'h00EC);
        wait_drq;
        read_id_block;
        expect16("ID w63 pp", idbuf[63], 16'h0000);
        expect16("ID w88 pp", idbuf[88], 16'h0000);
        sum = 0;
        for (i = 0; i < 256; i = i + 1) sum = sum + idbuf[i][7:0] + idbuf[i][15:8];
        if (sum[7:0] !== 8'h00) begin errs=errs+1; $display("  FAIL: pp ID checksum %02x", sum[7:0]); end
        wait_ready;
        $display("[%0t] pp_sess IDENTIFY ok", $time);

        // ---- READ SECTORS: 4 sectors at LBA 3 ----
        hs_write(3'd2, 16'h0004);
        hs_write(3'd3, 16'h0003);
        hs_write(3'd4, 16'h0000);
        hs_write(3'd5, 16'h0000);
        hs_write(3'd6, 16'h00E0);                     // LBA | dev0
        hs_write(3'd7, 16'h0020);
        for (k = 0; k < 4; k = k + 1) begin
            wait_drq;
            for (i = 0; i < 256; i = i + 1) begin
                hs_read(3'd0, d);
                if (d !== u_bfm.mem[(3+k)*256+i]) begin
                    if (errs < 8) $display("  FAIL rd s%0d w%0d: %04x exp %04x",
                                           k, i, d, u_bfm.mem[(3+k)*256+i]);
                    errs = errs + 1;
                end
            end
        end
        wait_ready;
        hs_read(3'd7, s); expect16("post-read status", s[7:0], 8'h50);
        $display("[%0t] READ SECTORS ok", $time);

        // ---- SET MULTIPLE 4 + WRITE MULTIPLE 8 sectors at LBA 16 ----
        hs_write(3'd2, 16'h0004);
        hs_write(3'd7, 16'h00C6);
        wait_ready;
        hs_read(3'd7, s); expect16("SET MULTIPLE status", s[7:0], 8'h50);
        hs_write(3'd2, 16'h0008);
        hs_write(3'd3, 16'h0010);
        hs_write(3'd4, 16'h0000);
        hs_write(3'd5, 16'h0000);
        hs_write(3'd6, 16'h00E0);
        hs_write(3'd7, 16'h00C5);
        for (k = 0; k < 2; k = k + 1) begin          // 2 DRQ blocks of 4 sectors
            wait_drq;
            for (i = 0; i < 1024; i = i + 1)
                hs_write(3'd0, (k*1024+i) ^ 16'h5AA5);
        end
        wait_ready;
        hs_read(3'd7, s); expect16("post-write status", s[7:0], 8'h50);
        for (i = 0; i < 2048; i = i + 1)
            if (u_bfm.mem[16*256+i] !== (i ^ 16'h5AA5)) begin
                if (errs < 8) $display("  FAIL wr w%0d: %04x exp %04x",
                                       i, u_bfm.mem[16*256+i], i ^ 16'h5AA5);
                errs = errs + 1;
            end
        $display("[%0t] WRITE MULTIPLE ok (%0d blocks written)", $time,
                 u_bfm.blocks_written);

        // ---- SET FEATURES: 03 accepted, 05 (APM) aborted with S51 ----
        hs_write(3'd1, 16'h0003);
        hs_write(3'd2, 16'h000C);
        hs_write(3'd7, 16'h00EF);
        wait_ready;
        hs_read(3'd7, s); expect16("SF03 status", s[7:0], 8'h50);
        hs_write(3'd1, 16'h0005);
        hs_write(3'd2, 16'h0080);
        hs_write(3'd7, 16'h00EF);
        wait_ready;
        hs_read(3'd7, s); expect16("SF05 status (S51)", s[7:0], 8'h51);
        hs_read(3'd1, d); expect16("SF05 err (ABRT)", d[7:0], 8'h04);

        // ---- CHECK POWER MODE / STANDBY / FLUSH ----
        hs_write(3'd7, 16'h00E5);
        wait_ready;
        hs_read(3'd2, d); expect16("E5 SC", d[7:0], 8'hFF);
        hs_write(3'd7, 16'h00E0);
        wait_ready;
        hs_read(3'd7, s); expect16("E0 status", s[7:0], 8'h50);
        hs_write(3'd7, 16'h00E7);
        wait_ready;
        hs_read(3'd7, s); expect16("E7 status", s[7:0], 8'h50);

        // ---- unsupported command aborts ----
        hs_write(3'd7, 16'h00A1);                    // IDENTIFY PACKET
        wait_ready;
        hs_read(3'd7, s); expect16("A1 status (S51)", s[7:0], 8'h51);

        if (u_bfm.cmd_crc_errs != 0) begin
            errs = errs + 1;
            $display("  FAIL: BFM saw %0d CMD CRC7 errors", u_bfm.cmd_crc_errs);
        end
        if (u_bfm.wr_crc_errs != 0) begin
            errs = errs + 1;
            $display("  FAIL: BFM saw %0d write CRC16 errors", u_bfm.wr_crc_errs);
        end

        if (errs == 0) $display("PASS: v2 PIO session complete");
        else           $display("FAIL: %0d errors", errs);
        $finish;
    end

    initial begin
        #80_000_000 $display("TIMEOUT"); $finish;
    end
endmodule
