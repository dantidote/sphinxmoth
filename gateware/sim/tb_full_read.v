// ============================================================================
// tb_full_read.v -- full-path read test: iSphynx host + CF device around
// interposer_top. Issues a 32-sector READ DMA (8192 words, 2x the FIFO) to
// prove the CONCURRENT sequencer streams a >FIFO command with no deadlock.
// Compile with -DSIM (cf_init short-circuits; fpga_top not used).
// ============================================================================
`timescale 1ns/1ps

module tb_full_read;
    localparam NSEC  = 32;
    localparam NWORD = NSEC*256;

    reg clk = 0; always #7.5 clk = ~clk;
    reg rst_n = 0;

    // ---- tristate data buses ----
    wire [15:0] HS_DD, CF_DD;
    reg  [15:0] hs_drv, cf_drv; reg hs_oe, cf_oe;
    assign HS_DD = hs_oe ? hs_drv : 16'bz;
    assign CF_DD = cf_oe ? cf_drv : 16'bz;

    // ---- iSphynx-driven control (DUT inputs) ----
    reg HS_CS0_N=1, HS_CS1_N=1, HS_A0=0, HS_A1=0, HS_A2=0;
    reg HS_IOR_N=1, HS_IOW_N=1, HS_DMACK_N=1, HS_RESET_N=1;
    wire HS_DMARQ, HS_INTRQ, HS_IORDY;

    // ---- CF-driven control (DUT inputs) ----
    reg CF_DMARQ=0, CF_INTRQ=0, CF_IORDY=0;
    wire CF_CS0_N, CF_CS1_N, CF_A0, CF_A1, CF_A2, CF_IOR_N, CF_IOW_N, CF_DMACK_N, CF_RESET_N;

    wire        DBG_ABORT, DBG_END, DBG_START, DBG_INIT_OK, DBG_INIT_BAD, DBG_CHUNK;
    wire [7:0]  DBG_INIT_STAT, DBG_STAT, DBG_CHUNKV;
    wire [15:0] DBG_CRC, DBG_WCNT;

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
        .DBG_ABORT(DBG_ABORT),.DBG_END(DBG_END),.DBG_START(DBG_START),
        .DBG_INIT_OK(DBG_INIT_OK),.DBG_INIT_BAD(DBG_INIT_BAD),.DBG_INIT_STAT(DBG_INIT_STAT),
        .DBG_STAT(DBG_STAT),.DBG_CRC(DBG_CRC),.DBG_WCNT(DBG_WCNT),
        .DBG_CHUNK(DBG_CHUNK),.DBG_CHUNKV(DBG_CHUNKV));

    reg [15:0] src   [0:NWORD-1];    // what the CF will send
    reg [15:0] rx    [0:NWORD-1];    // what the iSphynx captured
    integer    rxn;
    integer    i;

    // ================= iSphynx host model =================
    task hs_write(input [2:0] a, input [15:0] d);
        begin
            @(posedge clk); HS_CS0_N=0; HS_CS1_N=1; {HS_A2,HS_A1,HS_A0}=a;
            hs_drv=d; hs_oe=1; @(posedge clk); HS_IOW_N=0;
            repeat(4) @(posedge clk); HS_IOW_N=1;      // rising edge = commit
            repeat(2) @(posedge clk); HS_CS0_N=1; hs_oe=0; {HS_A2,HS_A1,HS_A0}=0;
            repeat(2) @(posedge clk);
        end
    endtask

    task hs_udma_receive;
        begin
            rxn=0;
            while (rxn < NWORD) begin
                @(posedge clk);
                if (HS_DMARQ) begin
                    if (rxn > 512) #450000;            // slow host (UDMA0 PP5002):
                                                       // grant gaps fill the FIFO
                    HS_DMACK_N = 0;                    // grant DMA
                    HS_IOR_N   = 0;                    // assert HDMARDY# = ready to receive
                    while (HS_DMARQ && rxn<NWORD) begin
                        @(HS_IORDY);                  // DSTROBE edge = one word
                        if (HS_DMARQ) begin           // post-DMARQ return edge != data
                            #3;                       // real latch: data-path delay
                            rx[rxn] = HS_DD; rxn=rxn+1;
                        end
                    end
                    HS_IOR_N   = 1;                    // negate HDMARDY#
                    HS_DMACK_N = 1;                    // burst paused/ended
                    repeat(4) @(posedge clk);
                end
            end
        end
    endtask

    // ================= CF device model (pausing UDMA source) =================
    // Real-card fidelity: honors HDMARDY# pause mid-burst (CF_IOR_N high =
    // host pause), and CHECKS the CRC the host sends at DMACK negation --
    // mismatch = the E84 the Transcend reports.
    function [15:0] tb_bitrev16; input [15:0] w; integer bi;
        for (bi = 0; bi < 16; bi = bi + 1) tb_bitrev16[bi] = w[15-bi];
    endfunction
    function [15:0] tb_crc_fold;            // crc16_udma MSB_FIRST(1) replica
        input [15:0] c; input [15:0] d;
        integer fi; reg fb;
        begin
            tb_crc_fold = c;
            for (fi = 0; fi < 16; fi = fi + 1) begin
                fb = tb_crc_fold[15] ^ d[15-fi];
                tb_crc_fold = {tb_crc_fold[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0000);
            end
        end
    endfunction
    reg [15:0] cf_crc_ref, cf_crc_got;
    integer    cf_crc_errs = 0;
    task cf_sector(input integer s);
        integer k;
        begin
            #60; CF_DMARQ = 1;                          // request
            wait (CF_DMACK_N==0 && CF_IOW_N==1);        // host grants + opens (STOP=IOW# high->low)
            #20;
            cf_crc_ref = 16'h4ABA;                      // per-burst seed
            for (k=0;k<256;k=k+1) begin
                cf_drv = src[s*256+k]; cf_oe = 1;
                repeat(6) @(posedge clk);
                while (CF_IOR_N) @(posedge clk);        // HDMARDY# pause: hold
                CF_IORDY = ~CF_IORDY;                   // DSTROBE edge
                cf_crc_ref = tb_crc_fold(cf_crc_ref, tb_bitrev16(src[s*256+k]));
                // NOTE: mid-interval crosstalk / DMARQ-glitch injections were
                // removed with the debounce revert -- those armors cost more
                // protocol margin than the hazards they covered (see memory:
                // bugs #12/#13 fixes broke proven UDMA2 traffic). If E84s
                // return at card-UDMA2, revisit with a scope, not a filter.
                repeat(2) @(posedge clk);
            end
            CF_DMARQ = 0; #10 cf_oe = 0;
            wait (CF_DMACK_N==1);                       // CRC latched on this edge
            cf_crc_got = CF_DD;
            if (cf_crc_got !== cf_crc_ref) begin
                cf_crc_errs = cf_crc_errs + 1;
                $display("[%0t] CF: BURST CRC MISMATCH (sector %0d): got %04x exp %04x  << E84",
                         $time, s, cf_crc_got, cf_crc_ref);
            end
            #150;
        end
    endtask

    // capture checker
    always @(posedge clk) if (rst_n && dut.fifo_wr) ; // (fifo internal; rely on rx compare)

    // probes
    always @(posedge clk) if (DBG_START) $display("[%0t] DBG_START seq began, sc_last=%0d words_total=%0d", $time, dut.sc_last, dut.words_total);
    reg afull_seen = 0;
    always @(posedge clk) if (dut.fifo_afull && !afull_seen) begin
        afull_seen = 1; $display("[%0t] FIFO afull reached -- HDMARDY# throttle exercised", $time);
    end
    always @(posedge clk) if (dut.cmd_valid) $display("[%0t] cmd_valid cmd=%02x is_dma=%b", $time, dut.cmd_byte, dut.cmd_is_dma);
    reg phdmarq=0, pdmackh=1; reg [3:0] pdst=0;
    integer iordy_edges=0;
    always @(HS_IORDY) iordy_edges=iordy_edges+1;
    always @(posedge clk) begin
        phdmarq<=HS_DMARQ; if (phdmarq!=HS_DMARQ) $display("[%0t] HS_DMARQ=%b devst=%0d hostDMACK=%b iordyEdges=%0d",$time,HS_DMARQ,dut.u_dev.st,HS_DMACK_N,iordy_edges);
        pdmackh<=HS_DMACK_N; if (pdmackh!=HS_DMACK_N) $display("[%0t] HS_DMACK_N=%b (iSphynx grant) devst=%0d",$time,HS_DMACK_N,dut.u_dev.st);
        pdst<=dut.u_dev.st; if (pdst!=dut.u_dev.st) $display("[%0t]   devst %0d->%0d  fifo_empty=%b cons_ready=%b want_dev=%b",$time,pdst,dut.u_dev.st,dut.fifo_empty,dut.cons_ready,dut.want_dev);
    end

    integer errs;
    initial begin
        $dumpfile("tb_full.vcd"); $dumpvars(0, tb_full_read);
        for (i=0;i<NWORD;i=i+1) src[i] = (i*16'h0101) ^ 16'h1234;
        hs_oe=0; cf_oe=0; errs=0;
        #100 rst_n=1; #200;
        // iSphynx issues READ DMA of NSEC sectors: sector count (addr2), command C8 (addr7)
        hs_write(3'd2, NSEC[15:0]);
        fork
            hs_write(3'd7, 16'h00C8);   // command -> cmd_snoop fires
            begin : cf_thread
                for (i=0;i<NSEC;i=i+1) cf_sector(i);
            end
            hs_udma_receive;
        join
        #2000;
        for (i=0;i<NWORD;i=i+1) if (rx[i]!==src[i]) begin
            if (errs<8) $display("  MISMATCH word %0d: got %04x exp %04x", i, rx[i], src[i]);
            errs=errs+1;
        end
        $display("=== RESULT: iSphynx got %0d/%0d words, %0d mismatches ===", rxn, NWORD, errs);
        if (rxn==NWORD && errs==0) $display("PASS: 32-sector read streamed through concurrent bridge");
        else $display("FAIL");
        $finish;
    end
    initial begin #20_000_000 $display("TIMEOUT: deadlock, rxn=%0d", rxn); $finish; end
endmodule
