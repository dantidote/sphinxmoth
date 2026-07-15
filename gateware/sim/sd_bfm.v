// ============================================================================
// sd_bfm.v -- behavioral SD card (sim only), v2 test family
// ----------------------------------------------------------------------------
// Same philosophy as the v1 CF/iSphynx models: hardware-calibrated cruelty.
// Everything advances on SD_CLK edges, so host-side clock freezing (the
// block-gap backpressure mechanism) is honored automatically. The model:
//   * checks CRC7 on every command frame (validates the host's CRC7 engine)
//   * checks CRC16 per DAT line on writes (validates the host's CRC16s)
//   * responds R1/R1b/R2/R3/R6/R7 with the real state machine transitions
//   * streams CMD18 multi-block reads until CMD12, honoring clock stop
//   * accepts CMD25 multi-block writes with CRC token + programmable busy
//   * injects faults on demand: corrupt read-block CRC, reject write blocks
//
// Drive discipline: card outputs change 2ns after the SDCLK falling edge;
// card samples host outputs on the rising edge. Matches the sd_host cell
// engine's sample points with margin in both directions.
// ============================================================================
`timescale 1ns/1ps

module sd_bfm #(
    parameter CAP_SECTORS = 65536,     // advertised (CSD v2): multiple of 1024
    parameter MEM_SECTORS = 1024,      // backing store (LBA 0..MEM_SECTORS-1)
    parameter ACMD41_TRIES = 2,        // busy responses before ready
    parameter BUSY_CELLS  = 12         // write program-busy length
) (
    input        sd_clk,
    inout        sd_cmd,
    inout  [3:0] sd_dat
);

    // card-side pull-ups (as on real cards)
    pullup pu_cmd (sd_cmd);
    pullup pu_d0 (sd_dat[0]);
    pullup pu_d1 (sd_dat[1]);
    pullup pu_d2 (sd_dat[2]);
    pullup pu_d3 (sd_dat[3]);

    reg        cmd_o = 1'b1, cmd_oe = 1'b0;
    reg [3:0]  dat_o = 4'hF;
    reg        dat_oe = 1'b0;
    assign sd_cmd = cmd_oe ? cmd_o : 1'bz;
    assign sd_dat = dat_oe ? dat_o : 4'bz;

    // backing store
    reg [15:0] mem [0:MEM_SECTORS*256-1];

    // fault injection (poke hierarchically from the TB)
    integer inj_rd_crc_countdown = 0;   // corrupt CRC of the Nth block read (1=next)
    integer inj_wr_reject_countdown = 0;// reject the Nth write block (1=next)

    // scoreboard the TB can check
    integer cmd_crc_errs = 0;           // host sent a bad CRC7
    integer wr_crc_errs  = 0;           // host sent a bad DAT CRC16
    integer blocks_read = 0, blocks_written = 0;

    // card state
    integer  cstate = 0;                // 0 idle,1 ready,2 ident,3 stby,4 tran
    reg        appcmd = 0;
    reg [15:0] rca = 16'h1234;
    integer  a41 = 0;
    reg        reading = 0, writing = 0;
    reg        stop_req = 0;
    reg [31:0] rw_blk;

    // ------------------------------------------------------------------------
    // CRC functions
    // ------------------------------------------------------------------------
    function [6:0] crc7f;
        input [39:0] d;
        integer i; reg fb;
        begin
            crc7f = 7'd0;
            for (i = 39; i >= 0; i = i - 1) begin
                fb = crc7f[6] ^ d[i];
                crc7f = {crc7f[5:3], crc7f[2] ^ fb, crc7f[1:0], fb};
            end
        end
    endfunction
    function [15:0] crc16f;
        input [15:0] c; input b;
        reg fb;
        begin
            fb = c[15] ^ b;
            crc16f = {c[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0000);
        end
    endfunction

    // ------------------------------------------------------------------------
    // CMD receive / respond
    // ------------------------------------------------------------------------
    reg [5:0]  rx_idx;
    reg [31:0] rx_arg;
    reg        rx_ok;

    task recv_cmd;
        reg [46:0] sh;
        integer i;
        begin
            @(posedge sd_clk);
            while (sd_cmd !== 1'b0) @(posedge sd_clk);       // start bit
            sh = 47'h0;
            for (i = 0; i < 47; i = i + 1) begin
                @(posedge sd_clk);
                sh = {sh[45:0], sd_cmd};
            end
            // sh = [dir, idx(6), arg(32), crc7(7), end(1)] -- 47 bits post-start
            rx_idx = sh[45:40];
            rx_arg = sh[39:8];
            rx_ok  = (crc7f({2'b01, sh[45:40], sh[39:8]}) == sh[7:1]) && sh[0];
            if (!rx_ok) begin
                cmd_crc_errs = cmd_crc_errs + 1;
                $display("[%0t] SD_BFM: CMD%0d BAD CRC7 (got %02x exp %02x)",
                         $time, rx_idx, sh[7:1],
                         crc7f({2'b01, sh[45:40], sh[39:8]}));
            end
        end
    endtask

    // send n bits, MSB first, driving 2ns after each falling edge
    task send_bits;
        input [135:0] bits;
        input integer n;
        integer i;
        begin
            repeat (3) @(negedge sd_clk);                    // NCR gap
            for (i = n - 1; i >= 0; i = i - 1) begin
                @(negedge sd_clk);
                #2 cmd_oe = 1'b1; cmd_o = bits[i];
            end
            @(negedge sd_clk);
            #2 cmd_oe = 1'b0; cmd_o = 1'b1;
        end
    endtask

    task send_r1;
        input [5:0] idx;
        input [31:0] status;
        reg [39:0] head;
        begin
            head = {2'b00, idx, status};
            send_bits({88'h0, head, crc7f(head), 1'b1}, 48);
        end
    endtask

    task send_r_ocr;                    // R3: reserved idx/crc
        input [31:0] ocr;
        begin
            send_bits({88'h0, 2'b00, 6'b111111, ocr, 7'h7F, 1'b1}, 48);
        end
    endtask

    task send_r2;                       // 136-bit: reserved header + 128 reg bits
        input [127:0] regbits;
        begin
            send_bits({2'b00, 6'b111111, regbits}, 136);
        end
    endtask

    task drive_busy;                    // R1b program-style busy on DAT0
        input integer cells;
        begin
            @(negedge sd_clk);
            #2 dat_oe = 1'b1; dat_o = 4'hE;                  // DAT0 low
            repeat (cells) @(negedge sd_clk);
            #2 dat_o = 4'hF;
            @(negedge sd_clk);
            #2 dat_oe = 1'b0;
        end
    endtask

    // ------------------------------------------------------------------------
    // CSD (v2) for the advertised capacity
    // ------------------------------------------------------------------------
    wire [21:0] c_size = (CAP_SECTORS / 1024) - 1;
    function [127:0] csd_v2;
        input dummy;
        begin
            csd_v2 = 128'h0;
            csd_v2[127:126] = 2'b01;                         // CSD v2
            csd_v2[69:48]   = c_size;
            csd_v2[0]       = 1'b1;                          // end bit slot
        end
    endfunction

    // ------------------------------------------------------------------------
    // DAT read engine (card -> host), one process; stop via stop_req
    // ------------------------------------------------------------------------
    reg [15:0] tcrc0, tcrc1, tcrc2, tcrc3;
    task send_block;                    // 512B from mem[rw_blk]
        integer n, w, li;
        reg [15:0] wd;
        reg [3:0] nib;
        reg corrupt;
        begin : sb
            corrupt = 0;
            if (inj_rd_crc_countdown == 1) begin
                corrupt = 1;
                $display("[%0t] SD_BFM: injecting read CRC error on block %0d",
                         $time, rw_blk);
            end
            if (inj_rd_crc_countdown > 0)
                inj_rd_crc_countdown = inj_rd_crc_countdown - 1;
            repeat (4) @(negedge sd_clk);                    // NAC
            if (stop_req) disable sb;
            #2 dat_oe = 1'b1; dat_o = 4'h0;                  // start nibble
            @(negedge sd_clk);
            tcrc0 = 0; tcrc1 = 0; tcrc2 = 0; tcrc3 = 0;
            for (n = 0; n < 1024; n = n + 1) begin
                if (stop_req) begin dat_oe = 0; disable sb; end
                wd = mem[{rw_blk[23:0], 8'b0} + n[11:2]];
                case (n[1:0])                                // LE pair, hi nibble first
                2'd0: nib = wd[7:4];
                2'd1: nib = wd[3:0];
                2'd2: nib = wd[15:12];
                2'd3: nib = wd[11:8];
                endcase
                #2 dat_o = nib;
                tcrc0 = crc16f(tcrc0, nib[0]);
                tcrc1 = crc16f(tcrc1, nib[1]);
                tcrc2 = crc16f(tcrc2, nib[2]);
                tcrc3 = crc16f(tcrc3, nib[3]);
                @(negedge sd_clk);
            end
            if (corrupt) tcrc2 = tcrc2 ^ 16'h0100;
            for (n = 15; n >= 0; n = n - 1) begin
                #2 dat_o = {tcrc3[n], tcrc2[n], tcrc1[n], tcrc0[n]};
                @(negedge sd_clk);
            end
            #2 dat_o = 4'hF;                                 // end bit
            @(negedge sd_clk);
            #2 dat_oe = 1'b0;
            blocks_read = blocks_read + 1;
        end
    endtask

    always @(posedge reading) begin : rd_proc
        while (reading && !stop_req) begin
            send_block;
            rw_blk = rw_blk + 1;
        end
        dat_oe = 0;
    end

    // ------------------------------------------------------------------------
    // DAT write engine (host -> card)
    // ------------------------------------------------------------------------
    task recv_block;
        integer n;
        reg [15:0] rcrc0, rcrc1, rcrc2, rcrc3;
        reg [15:0] xcrc0, xcrc1, xcrc2, xcrc3;
        reg [3:0]  nib;
        reg [7:0]  lo;
        reg [15:0] wtmp [0:255];
        integer wi;
        reg reject;
        begin : rb
            // start nibble hunt (rising-edge samples)
            @(posedge sd_clk);
            while (sd_dat[0] !== 1'b0) begin
                if (!writing || stop_req) disable rb;
                @(posedge sd_clk);
            end
            xcrc0 = 0; xcrc1 = 0; xcrc2 = 0; xcrc3 = 0;
            wi = 0; lo = 8'h0;
            for (n = 0; n < 1024; n = n + 1) begin
                @(posedge sd_clk);
                nib = sd_dat;
                xcrc0 = crc16f(xcrc0, nib[0]);
                xcrc1 = crc16f(xcrc1, nib[1]);
                xcrc2 = crc16f(xcrc2, nib[2]);
                xcrc3 = crc16f(xcrc3, nib[3]);
                case (n[1:0])
                2'd0: lo[7:4]  = nib;
                2'd1: lo[3:0]  = nib;
                2'd2: wtmp[wi][15:12] = nib;
                2'd3: begin
                    wtmp[wi][11:8] = nib;
                    wtmp[wi][7:0]  = lo;
                    wi = wi + 1;
                end
                endcase
            end
            rcrc0 = 0; rcrc1 = 0; rcrc2 = 0; rcrc3 = 0;
            for (n = 0; n < 16; n = n + 1) begin
                @(posedge sd_clk);
                rcrc0 = {rcrc0[14:0], sd_dat[0]};
                rcrc1 = {rcrc1[14:0], sd_dat[1]};
                rcrc2 = {rcrc2[14:0], sd_dat[2]};
                rcrc3 = {rcrc3[14:0], sd_dat[3]};
            end
            @(posedge sd_clk);                               // end bit
            reject = 0;
            if (inj_wr_reject_countdown == 1) begin
                reject = 1;
                $display("[%0t] SD_BFM: rejecting write block %0d", $time, rw_blk);
            end
            if (inj_wr_reject_countdown > 0)
                inj_wr_reject_countdown = inj_wr_reject_countdown - 1;
            if (rcrc0 !== xcrc0 || rcrc1 !== xcrc1 ||
                rcrc2 !== xcrc2 || rcrc3 !== xcrc3) begin
                wr_crc_errs = wr_crc_errs + 1;
                reject = 1;
                $display("[%0t] SD_BFM: HOST WRITE CRC MISMATCH block %0d (l0 %04x/%04x)",
                         $time, rw_blk, rcrc0, xcrc0);
            end
            // CRC status token + busy
            repeat (2) @(negedge sd_clk);
            #2 dat_oe = 1'b1;
            dat_o = 4'hE; @(negedge sd_clk);                 // token start (DAT0=0)
            if (reject) begin
                #2 dat_o = 4'hF; @(negedge sd_clk);          // 1
                #2 dat_o = 4'hE; @(negedge sd_clk);          // 0
                #2 dat_o = 4'hF; @(negedge sd_clk);          // 1  -> 101 = CRC error
            end else begin
                #2 dat_o = 4'hE; @(negedge sd_clk);          // 0
                #2 dat_o = 4'hF; @(negedge sd_clk);          // 1
                #2 dat_o = 4'hE; @(negedge sd_clk);          // 0  -> 010 = accepted
            end
            #2 dat_o = 4'hF; @(negedge sd_clk);              // token end bit
            // program busy
            #2 dat_o = 4'hE;
            repeat (BUSY_CELLS) @(negedge sd_clk);
            #2 dat_o = 4'hF;
            @(negedge sd_clk);
            #2 dat_oe = 1'b0;
            if (!reject) begin
                for (n = 0; n < 256; n = n + 1)
                    mem[{rw_blk[23:0], 8'b0} + n] = wtmp[n];
                blocks_written = blocks_written + 1;
                rw_blk = rw_blk + 1;
            end
        end
    endtask

    always @(posedge writing) begin : wr_proc
        while (writing && !stop_req) recv_block;
        dat_oe = 0;
    end

    // 64-byte CMD6 status block (content zeros, honest CRCs)
    task send_block64;
        integer n;
        begin
            repeat (4) @(negedge sd_clk);
            #2 dat_oe = 1'b1; dat_o = 4'h0;
            @(negedge sd_clk);
            tcrc0 = 0; tcrc1 = 0; tcrc2 = 0; tcrc3 = 0;
            for (n = 0; n < 128; n = n + 1) begin
                #2 dat_o = 4'h0;
                tcrc0 = crc16f(tcrc0, 1'b0);
                tcrc1 = crc16f(tcrc1, 1'b0);
                tcrc2 = crc16f(tcrc2, 1'b0);
                tcrc3 = crc16f(tcrc3, 1'b0);
                @(negedge sd_clk);
            end
            for (n = 15; n >= 0; n = n - 1) begin
                #2 dat_o = {tcrc3[n], tcrc2[n], tcrc1[n], tcrc0[n]};
                @(negedge sd_clk);
            end
            #2 dat_o = 4'hF;
            @(negedge sd_clk);
            #2 dat_oe = 1'b0;
        end
    endtask

    // ------------------------------------------------------------------------
    // command dispatcher
    // ------------------------------------------------------------------------
    wire [31:0] r1_status = 32'h00000100 | (cstate << 9);    // READY_FOR_DATA
    initial begin
        forever begin
            recv_cmd;
            if (rx_ok) begin
                case (rx_idx)
                6'd0: begin cstate = 0; appcmd = 0; reading = 0; writing = 0; end
                6'd8: send_bits({88'h0, {2'b00, 6'd8, 20'h0, rx_arg[11:8], rx_arg[7:0]},
                                 crc7f({2'b00, 6'd8, 20'h0, rx_arg[11:8], rx_arg[7:0]}),
                                 1'b1}, 48);
                6'd55: begin send_r1(6'd55, r1_status | 32'h20); appcmd = 1; end
                6'd41: if (appcmd) begin
                    appcmd = 0;
                    a41 = a41 + 1;
                    if (a41 >= ACMD41_TRIES) begin
                        send_r_ocr(32'hC0FF8000);            // ready + CCS
                        cstate = 1;
                    end else
                        send_r_ocr(32'h00FF8000);            // busy
                end
                6'd2: if (cstate == 1) begin
                    send_r2({120'hDEAD_BEEF_0102_0304_0506_0708_0910, 7'h0, 1'b1});
                    cstate = 2;
                end
                6'd3: begin
                    send_bits({88'h0, {2'b00, 6'd3, rca, 16'h0500},
                               crc7f({2'b00, 6'd3, rca, 16'h0500}), 1'b1}, 48);
                    cstate = 3;
                end
                6'd9: send_r2(csd_v2(1'b0));
                6'd7: begin
                    send_r1(6'd7, r1_status);
                    drive_busy(4);
                    cstate = 4;
                end
                6'd6: if (appcmd) begin                       // ACMD6: bus width
                    appcmd = 0;
                    send_r1(6'd6, r1_status);
                end else begin                                // CMD6: switch + block
                    send_r1(6'd6, r1_status);
                    send_block64;
                end
                6'd16: send_r1(6'd16, r1_status);
                6'd13: send_r1(6'd13, r1_status);
                6'd17, 6'd18: begin
                    stop_req = 0;
                    rw_blk = rx_arg;                          // CCS: block address
                    send_r1(rx_idx, r1_status);
                    reading = 1;
                end
                6'd24, 6'd25: begin
                    stop_req = 0;
                    rw_blk = rx_arg;
                    send_r1(rx_idx, r1_status);
                    writing = 1;
                end
                6'd12: begin
                    stop_req = 1;
                    reading = 0; writing = 0;
                    send_r1(6'd12, r1_status);
                    drive_busy(6);
                    stop_req = 0;
                end
                default: send_r1(rx_idx, r1_status | 32'h00400000); // ILLEGAL_CMD
                endcase
            end
        end
    end

endmodule
