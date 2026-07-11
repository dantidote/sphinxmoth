// ============================================================================
// tb_full_write.v -- full-path WRITE test. iSphynx drives UDMA data into the
// bridge (respecting our HDMARDY# flow control), host engine writes it to the
// CF; the CF model captures and checks. 32-sector WRITE DMA (2x FIFO).
// Compile with -DSIM.
// ============================================================================
`timescale 1ns/1ps

module tb_full_write;
    localparam NSEC  = 8;   // must fit the FIFO: strict CF model sinks only
                            // after prod_complete (real-card DMARQ timing)
    localparam NWORD = NSEC*256;

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

    reg [15:0] src [0:NWORD-1];
    reg [15:0] cfrx[0:NWORD-1];
    integer    cfn, i, errs, bcnt;
    real       ledge;

    task hs_write(input [2:0] a, input [15:0] d);
        begin
            @(posedge clk); HS_CS0_N=0; HS_CS1_N=1; {HS_A2,HS_A1,HS_A0}=a;
            hs_drv=d; hs_oe=1; @(posedge clk); HS_IOW_N=0;
            repeat(4) @(posedge clk); HS_IOW_N=1;
            repeat(2) @(posedge clk); HS_CS0_N=1; hs_oe=0; {HS_A2,HS_A1,HS_A0}=0;
            repeat(2) @(posedge clk);
        end
    endtask

    // iSphynx UDMA WRITE host: drive all NWORD words, honoring HDMARDY# (HS_IORDY)
    task hs_udma_send;
        integer n;
        begin
            n=0;
            while (n<NWORD) begin
                @(posedge clk);
                if (HS_DMARQ) begin
                    #15000;                           // real hosts poll status
                                                      // before granting (PP5002)
                    wait (!polling);                  // one host: never grant mid-poll
                    HS_DMACK_N=0; HS_IOW_N=0;         // grant + negate STOP
                    while (n<NWORD) begin
                        if (HS_IORDY) begin @(posedge clk); end   // paused (DDMARDY# high)
                        else begin
                            hs_drv=src[n]; hs_oe=1;
                            repeat(6) @(posedge clk);
                            HS_IOR_N = ~HS_IOR_N;    // HSTROBE edge = one word
                            n=n+1;
                            if ((n % 128) == 64) begin
                                #8  HS_IOR_N = ~HS_IOR_N;  // ring: settles <=20ns
                                #12 HS_IOR_N = ~HS_IOR_N;  // (hardware bug #9 repro)
                            end
                            repeat(2) @(posedge clk);
                        end
                    end
                    repeat(4) @(posedge clk);          // tSS: real hosts hold off
                    HS_IOW_N=1;
                    repeat(4) @(posedge clk);          // CRC window before release
                    HS_DMACK_N=1; hs_oe=0;             // STOP + release
                end
            end
        end
    endtask

    // CF device WRITE sink: assert DMARQ, receive host bursts, capture on HSTROBE.
    // STRICT like real silicon: refuses to assert DDMARDY# while CS0/CS1 are
    // asserted (spec requires CS negated + DA=0 throughout a UDMA burst).
    task cf_recv_all;
        begin
            cfn=0;
            // real cards raise DMARQ only when ready to sink -- by then the
            // iSphynx burst is done and the firmware is parked on status
            wait (dut.burst_active === 1'b1);     // command accepted
            wait (dut.prod_complete === 1'b0);    // stale-true clears
            wait (dut.prod_complete === 1'b1);    // iSphynx data all captured
            wait (HS_DMACK_N === 1'b1); #2000;    // iSphynx grant closed, parked
            while (cfn<NWORD) begin
                #60; CF_DMARQ=1; CF_IORDY=1;          // request, NOT ready yet
                wait (CF_DMACK_N==0 && CF_IOW_N==0);  // host opened burst
                $display("[%0t] CF: burst open. CS0=%b CS1=%b A=%b park_en=%b HS_CS0=%b HS_DMACK=%b",
                         $time, CF_CS0_N, CF_CS1_N, {CF_A2,CF_A1,CF_A0}, park_en, HS_CS0_N, HS_DMACK_N);
                wait (CF_CS0_N===1'b1 && CF_CS1_N===1'b1
                      && {CF_A2,CF_A1,CF_A0}===3'd0); // legal bus, or no DDMARDY#
                $display("[%0t] CF: bus legal, asserting DDMARDY#", $time);
                #40; CF_IORDY=0;                      // NOW ready to receive
                bcnt = 0;                             // sector buffer: 256 words max
                while (CF_DMACK_N==0 && CF_IOW_N==0 && cfn<NWORD && bcnt<256) begin
                    @(CF_IOR_N);                      // HSTROBE edge = one word
                    ledge = $realtime;
                    #3;                               // real latch: data-path delay
                    cfrx[cfn]=CF_DD; cfn=cfn+1; bcnt=bcnt+1;
                end
                CF_IORDY=1;
                CF_DMARQ=0;                           // sector buffer full: TERMINATE
                @(posedge CF_IOW_N);                  // host must close (STOP)
                if (($realtime - ledge) < 50.0) begin // tSS: card still digesting
                    $display("[%0t] CF: tSS violation (%0.1fns after last edge) -- last word dropped",
                             $time, $realtime - ledge);
                    cfn = cfn - 1;
                end
                @(posedge CF_DMACK_N);                // CRC latched on this edge
                #3000;                                // sector program time
            end
        end
    endtask

    // iSphynx CS parker: real firmware sits on the status register (CS0 low,
    // addr 7) whenever it hasn't granted its own UDMA burst.
    reg park_en = 0;
    always @(*) begin
        if (park_en && HS_DMACK_N) begin
            HS_CS0_N = 1'b0; {HS_A2,HS_A1,HS_A0} = 3'd7;
        end else if (park_en) begin
            HS_CS0_N = 1'b1; {HS_A2,HS_A1,HS_A0} = 3'd0;
        end
    end
    // PP5002-style STATUS POLLING: while parked (no grant), the host strobes
    // IOR to read status. These edges are PIO, NOT data -- a capture that
    // counts them injects phantom words (hardware bug #9 repro). `polling`
    // keeps the single-host illusion: the sender never grants mid-poll.
    reg polling = 0;
    always begin
        #1730;
        if (park_en && HS_DMACK_N) begin
            polling = 1;
            HS_IOR_N = 0; #200; HS_IOR_N = 1;
            #50 polling = 0;
        end
    end

    always @(posedge clk) if (DBG_START) $display("[%0t] START write, sc=%0d total=%0d",$time,dut.sc_last,dut.words_total);
    always @(posedge clk) if (DBG_ABORT) $display("[%0t] ABORT stat=%02x",$time,DBG_STAT);
    // stall detector: if no fifo activity for a while, dump state once
    reg [15:0] idle_c=0; reg dumped=0;
    always @(posedge clk) begin
        if (dut.fifo_wr | dut.fifo_rd) idle_c<=0; else idle_c<=idle_c+1;
        if (idle_c==16'd4000 && !dumped) begin
            dumped<=1;
            $display("[%0t] STALL: devst=%0d hostst=%0d fifo_cnt=%0d empty=%b afull=%b room=%b consR=%b prodC=%b",
              $time, dut.u_dev.st, dut.u_host.st, dut.fifo_count, dut.fifo_empty, dut.fifo_afull,
              dut.fifo_room, dut.cons_ready, dut.prod_complete);
            $display("        want_host=%b host_busy=%b want_dev=%b dev_busy=%b words_cap=%0d cfn=%0d HS_DMARQ=%b HS_IORDY=%b CF_DMARQ=%b CF_DMACK=%b",
              dut.want_host, dut.host_busy, dut.want_dev, dut.dev_busy, dut.words_cap, cfn, HS_DMARQ, HS_IORDY, CF_DMARQ, CF_DMACK_N);
        end
    end

    initial begin
        $dumpfile("tb_wr.vcd"); $dumpvars(0, tb_full_write);
        for (i=0;i<NWORD;i=i+1) src[i] = (i*16'h0101) ^ 16'hBEEF;
        hs_oe=0; cf_oe=0; errs=0;
        #100 rst_n=1; #200;
        hs_write(3'd2, NSEC[15:0]);
        hs_write(3'd7, 16'h00CA);       // WRITE DMA
        park_en = 1;                    // firmware parks on status reg from here
        fork
            hs_udma_send;
            cf_recv_all;
        join
        park_en = 0;
        #3000;
        for (i=0;i<NWORD;i=i+1) if (cfrx[i]!==src[i]) begin
            if (errs<8) $display("  MISMATCH word %0d: got %04x exp %04x",i,cfrx[i],src[i]);
            errs=errs+1;
        end
        $display("=== RESULT: CF got %0d/%0d words, %0d mismatches ===", cfn, NWORD, errs);
        if (cfn==NWORD && errs==0) $display("PASS: 32-sector write streamed to CF");
        else $display("FAIL");
        $finish;
    end
    initial begin #30_000_000 $display("TIMEOUT: deadlock, cfn=%0d", cfn); $finish; end
endmodule
