// ============================================================================
// tb_identify.v -- IDENTIFY patcher coverage (word 0 / 63 / 88 / 255).
// A CFA-flavored card (word0=0x848A removable signature, signed checksum)
// serves IDENTIFY over PIO passthrough. Checks:
//   iSphynx session (no C90): word0 -> 0x045A (fixed disk), DMA words intact,
//                             block checksum still valid.
//   PP5002 session (C90):     word0 -> 0x045A, words 63+88 -> 0, checksum valid.
// Compile with -DSIM.
// ============================================================================
`timescale 1ns/1ps

module tb_identify;
    reg clk = 0; always #7.5 clk = ~clk;
    reg rst_n = 0;

    wire [15:0] HS_DD, CF_DD;
    reg  [15:0] hs_drv, cf_drv; reg hs_oe, cf_oe;
    assign HS_DD = hs_oe ? hs_drv : 16'bz;
    assign CF_DD = cf_oe ? cf_drv : 16'bz;

    reg HS_CS0_N=1, HS_CS1_N=1, HS_A0=0, HS_A1=0, HS_A2=0;
    reg HS_IOR_N=1, HS_IOW_N=1, HS_DMACK_N=1, HS_RESET_N=1;
    wire HS_DMARQ, HS_INTRQ, HS_IORDY;

    reg CF_DMARQ=0, CF_INTRQ=0, CF_IORDY=0;
    wire CF_CS0_N, CF_CS1_N, CF_A0, CF_A1, CF_A2, CF_IOR_N, CF_IOW_N, CF_DMACK_N, CF_RESET_N;

    interposer_top #(.CLK_MHZ(66)) dut (
        .HS_DD(HS_DD),.HS_CS0_N(HS_CS0_N),.HS_CS1_N(HS_CS1_N),
        .HS_A0(HS_A0),.HS_A1(HS_A1),.HS_A2(HS_A2),
        .HS_IOR_N(HS_IOR_N),.HS_IOW_N(HS_IOW_N),.HS_DMARQ(HS_DMARQ),
        .HS_DMACK_N(HS_DMACK_N),.HS_RESET_N(HS_RESET_N),.HS_INTRQ(HS_INTRQ),.HS_IORDY(HS_IORDY),
        .CF_DD(CF_DD),.CF_CS0_N(CF_CS0_N),.CF_CS1_N(CF_CS1_N),
        .CF_A0(CF_A0),.CF_A1(CF_A1),.CF_A2(CF_A2),
        .CF_IOR_N(CF_IOR_N),.CF_IOW_N(CF_IOW_N),.CF_DMARQ(CF_DMARQ),
        .CF_DMACK_N(CF_DMACK_N),.CF_RESET_N(CF_RESET_N),.CF_INTRQ(CF_INTRQ),.CF_IORDY(CF_IORDY),
        .CLK(clk),.RST_N(rst_n),
        .DBG_ABORT(),.DBG_END(),.DBG_START(),.DBG_INIT_OK(),.DBG_INIT_BAD(),
        .DBG_INIT_STAT(),.DBG_STAT(),.DBG_CRC(),.DBG_WCNT(),.DBG_CHUNK(),
        .DBG_CHUNKV(),.DBG_WCAP(),.DBG_DMACKF(),.DBG_HOSTQ(),.DBG_WSENT());

    // ---- CFA-flavored card: 256-word IDENTIFY with a signed checksum -------
    reg [15:0] idblk [0:255];
    integer widx = 0;
    integer bi;
    reg [7:0] csum;

    // serve data-reg reads combinationally (PIO passthrough is transparent)
    always @(*) begin
        cf_oe  = (!CF_CS0_N && !CF_IOR_N && {CF_A2,CF_A1,CF_A0} == 3'd0);
        cf_drv = idblk[widx & 255];
    end
    // advance on strobe rise; restart the block on an EC command write
    always @(posedge CF_IOR_N)
        if (!CF_CS0_N && {CF_A2,CF_A1,CF_A0} == 3'd0) widx = widx + 1;
    reg [7:0] cf_last_cmd = 0;
    always @(posedge CF_IOW_N)
        if (!CF_CS0_N && {CF_A2,CF_A1,CF_A0} == 3'd7) begin
            cf_last_cmd = CF_DD[7:0];
            if (CF_DD[7:0] == 8'hEC) widx = 0;
        end

    // ---- host-side PIO primitives -------------------------------------------
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
            repeat(6) @(posedge clk); d = HS_DD;
            HS_IOR_N=1; repeat(2) @(posedge clk);
            HS_CS0_N=1; {HS_A2,HS_A1,HS_A0}=0; repeat(2) @(posedge clk);
        end
    endtask

    // ---- read a whole IDENTIFY block and validate ---------------------------
    reg [15:0] rx [0:255];
    integer errs = 0;
    integer bsum;
    task read_identify(input pp, input [8*16-1:0] label);
        integer k;
        begin
            hs_write(3'd7, 16'h00EC);
            for (k = 0; k < 256; k = k + 1) hs_read(3'd0, rx[k]);
            // word 0 always patched to fixed-disk
            if (rx[0] !== 16'h045A) begin
                errs = errs + 1;
                $display("FAIL %0s: word0 = %04x (want 045A)", label, rx[0]);
            end
            // DMA words: zeroed only in the PP5002 session
            if (pp) begin
                if (rx[63] !== 16'h0000 || rx[88] !== 16'h0000) begin
                    errs = errs + 1;
                    $display("FAIL %0s: w63=%04x w88=%04x (want 0)", label, rx[63], rx[88]);
                end
            end else begin
                if (rx[63] !== idblk[63] || rx[88] !== idblk[88]) begin
                    errs = errs + 1;
                    $display("FAIL %0s: w63=%04x w88=%04x (want originals %04x/%04x)",
                             label, rx[63], rx[88], idblk[63], idblk[88]);
                end
            end
            // signature + checksum: byte-sum of the whole block must be 0
            bsum = 0;
            for (k = 0; k < 256; k = k + 1)
                bsum = bsum + rx[k][7:0] + rx[k][15:8];
            if (rx[255][7:0] !== 8'hA5 || (bsum & 255) != 0) begin
                errs = errs + 1;
                $display("FAIL %0s: w255=%04x bytesum=%0d (want A5 sig, sum 0 mod 256)",
                         label, rx[255], bsum & 255);
            end
            $display("%0s: w0=%04x w63=%04x w88=%04x w255=%04x sum%%256=%0d",
                     label, rx[0], rx[63], rx[88], rx[255], bsum & 255);
        end
    endtask

    initial begin
        // build the card's IDENTIFY: CFA signature, some DMA caps, signed
        for (bi = 0; bi < 256; bi = bi + 1) idblk[bi] = (bi * 16'h0123) ^ 16'h55AA;
        idblk[0]  = 16'h848A;      // CFA signature (removable-class)
        idblk[63] = 16'h0407;      // MWDMA 0-2 supported, 2 selected
        idblk[88] = 16'h0407;      // UDMA 0-2 supported, 2 selected
        idblk[255] = 16'h00A5;
        csum = 0;
        for (bi = 0; bi < 256; bi = bi + 1)
            csum = csum + idblk[bi][7:0] + idblk[bi][15:8];
        idblk[255][15:8] = 8'h00 - csum;   // make the block byte-sum 0
        csum = 0;
        for (bi = 0; bi < 256; bi = bi + 1)
            csum = csum + idblk[bi][7:0] + idblk[bi][15:8];
        if (csum != 0) $display("TB bug: card block checksum %0d", csum);

        hs_oe = 0; cf_oe = 0;
        #100 rst_n = 1; #500;

        // session 1: iSphynx (no C90) -- word0 fixed, DMA words untouched
        read_identify(1'b0, "isphynx ");

        // session 2: PP5002 (C90 first) -- word0 fixed, DMA words zeroed
        hs_write(3'd7, 16'h0090);
        read_identify(1'b1, "pp5002  ");

        // power-command neutering: E0/E2/E6 must reach the card as E5;
        // E1/E3 (IDLE family) must pass through untouched
        hs_write(3'd7, 16'h00E0); if (cf_last_cmd !== 8'hE5) begin errs=errs+1; $display("FAIL: E0 -> %02x (want E5)", cf_last_cmd); end
        hs_write(3'd7, 16'h00E2); if (cf_last_cmd !== 8'hE5) begin errs=errs+1; $display("FAIL: E2 -> %02x (want E5)", cf_last_cmd); end
        hs_write(3'd7, 16'h00E6); if (cf_last_cmd !== 8'hE5) begin errs=errs+1; $display("FAIL: E6 -> %02x (want E5)", cf_last_cmd); end
        hs_write(3'd7, 16'h00E1); if (cf_last_cmd !== 8'hE1) begin errs=errs+1; $display("FAIL: E1 -> %02x (want E1)", cf_last_cmd); end
        hs_write(3'd7, 16'h00E3); if (cf_last_cmd !== 8'hE3) begin errs=errs+1; $display("FAIL: E3 -> %02x (want E3)", cf_last_cmd); end
        $display("power cmds: E0/E2/E6 rewritten, E1/E3 passed");

        if (errs == 0) $display("PASS: IDENTIFY patcher + power-command neutering");
        else           $display("FAIL: %0d errors", errs);
        $finish;
    end
    initial begin #10_000_000 $display("TIMEOUT"); $finish; end
endmodule
