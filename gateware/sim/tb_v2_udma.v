// ============================================================================
// tb_v2_udma.v -- v2 iSphynxII-flavored session: READ DMA 32 sectors (2x the
// FIFO -> streams + DDMARDY bounce) and WRITE DMA 8 sectors with the v1
// hardware-calibrated cruelty: strobe ringing, status-poll parking, grants
// paced by polls. The iSphynx never sends EXECUTE DIAGNOSTIC (pristine
// session -> UDMA stays advertised) and never sends/needs a usable CRC.
// ============================================================================
`timescale 1ns/1ps

module tb_v2_udma;
    localparam NSEC_RD  = 32;
    localparam NWORD_RD = NSEC_RD*256;
    localparam NSEC_WR  = 8;
    localparam NWORD_WR = NSEC_WR*256;

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
    reg [15:0] s;
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

    // ---- iSphynx UDMA receive (v1 model, verbatim pacing) ----
    reg [15:0] rx [0:NWORD_RD-1];
    integer    rxn;
    task hs_udma_receive;
        begin
            rxn = 0;
            while (rxn < NWORD_RD) begin
                @(posedge clk);
                if (HS_DMARQ) begin
                    if (rxn > 512) #450000;            // slow-host grant gaps
                    HS_DMACK_N = 0;
                    HS_IOR_N   = 0;                    // HDMARDY# = ready
                    while (HS_DMARQ && rxn < NWORD_RD) begin
                        @(HS_IORDY);                   // DSTROBE edge
                        if (HS_DMARQ) begin
                            #3;
                            rx[rxn] = HS_DD; rxn = rxn + 1;
                        end
                    end
                    HS_IOR_N   = 1;
                    HS_DMACK_N = 1;
                    repeat(4) @(posedge clk);
                end
            end
        end
    endtask

    // ---- iSphynx UDMA send with ringing + status-poll parking (v1) ----
    reg [15:0] src [0:NWORD_WR-1];
    reg park_en = 0;
    always @(*) begin
        if (park_en && HS_DMACK_N) begin
            HS_CS0_N = 1'b0; {HS_A2,HS_A1,HS_A0} = 3'd7;
        end else if (park_en) begin
            HS_CS0_N = 1'b1; {HS_A2,HS_A1,HS_A0} = 3'd0;
        end
    end
    reg polling = 0;
    always begin
        #1730;
        if (park_en && HS_DMACK_N) begin
            polling = 1;
            HS_IOR_N = 0; #200; HS_IOR_N = 1;
            #50 polling = 0;
        end
    end
    task hs_udma_send;
        integer n;
        begin
            n = 0;
            while (n < NWORD_WR) begin
                @(posedge clk);
                if (HS_DMARQ) begin
                    #15000;
                    wait (!polling);
                    HS_DMACK_N = 0; HS_IOW_N = 0;
                    while (n < NWORD_WR) begin
                        if (HS_IORDY) begin @(posedge clk); end   // paused
                        else begin
                            hs_drv = src[n]; hs_oe = 1;
                            repeat(6) @(posedge clk);
                            HS_IOR_N = ~HS_IOR_N;                 // HSTROBE edge
                            n = n + 1;
                            if ((n % 128) == 64) begin
                                #8  HS_IOR_N = ~HS_IOR_N;         // ring (bug #9)
                                #12 HS_IOR_N = ~HS_IOR_N;
                            end
                            repeat(2) @(posedge clk);
                        end
                    end
                    repeat(4) @(posedge clk);                     // tSS
                    HS_IOW_N = 1;
                    hs_drv = 16'hBAD0; hs_oe = 1;                 // garbage CRC slot
                    repeat(4) @(posedge clk);
                    HS_DMACK_N = 1; hs_oe = 0;
                end
            end
        end
    endtask

    integer i;
    initial begin
        $dumpfile("tb_v2_udma.vcd"); $dumpvars(0, tb_v2_udma);
        hs_oe = 0;
        for (i = 0; i < 1024*256; i = i + 1) u_bfm.mem[i] = (i*16'h0101) ^ 16'h1234;
        for (i = 0; i < NWORD_WR; i = i + 1) src[i] = (i*16'h0101) ^ 16'hBEEF;
        #200 rst_n = 1;

        wait_ready;
        $display("[%0t] boot ok, issuing READ DMA %0d sectors @ LBA 8", $time, NSEC_RD);

        // ---- READ DMA ----
        hs_write(3'd2, NSEC_RD[15:0]);
        hs_write(3'd3, 16'h0008);
        hs_write(3'd4, 16'h0000);
        hs_write(3'd5, 16'h0000);
        hs_write(3'd6, 16'h00E0);
        fork
            hs_write(3'd7, 16'h00C8);
            hs_udma_receive;
        join
        wait_ready;
        hs_read(3'd7, s);
        if (s[7:0] !== 8'h50) begin errs=errs+1; $display("  FAIL rd status %02x", s[7:0]); end
        for (i = 0; i < NWORD_RD; i = i + 1)
            if (rx[i] !== u_bfm.mem[8*256+i]) begin
                if (errs < 8) $display("  MISMATCH rd w%0d: %04x exp %04x",
                                       i, rx[i], u_bfm.mem[8*256+i]);
                errs = errs + 1;
            end
        $display("[%0t] READ DMA done: %0d/%0d words, errs so far %0d",
                 $time, rxn, NWORD_RD, errs);

        // ---- WRITE DMA ----
        $display("[%0t] WRITE DMA %0d sectors @ LBA 64", $time, NSEC_WR);
        hs_write(3'd2, NSEC_WR[15:0]);
        hs_write(3'd3, 16'h0040);
        hs_write(3'd4, 16'h0000);
        hs_write(3'd5, 16'h0000);
        hs_write(3'd6, 16'h00E0);
        hs_write(3'd7, 16'h00CA);
        park_en = 1;
        hs_udma_send;
        park_en = 0;
        wait_ready;
        hs_read(3'd7, s);
        if (s[7:0] !== 8'h50) begin errs=errs+1; $display("  FAIL wr status %02x", s[7:0]); end
        for (i = 0; i < NWORD_WR; i = i + 1)
            if (u_bfm.mem[64*256+i] !== src[i]) begin
                if (errs < 8) $display("  MISMATCH wr w%0d: %04x exp %04x",
                                       i, u_bfm.mem[64*256+i], src[i]);
                errs = errs + 1;
            end
        if (u_bfm.wr_crc_errs != 0) begin
            errs = errs + 1;
            $display("  FAIL: BFM saw %0d write CRC16 errors", u_bfm.wr_crc_errs);
        end

        if (errs == 0) $display("PASS: v2 UDMA read+write streamed through the SD backend");
        else           $display("FAIL: %0d errors", errs);
        $finish;
    end

    initial begin #200_000_000 $display("TIMEOUT"); $finish; end
endmodule
