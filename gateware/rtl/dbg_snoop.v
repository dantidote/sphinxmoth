// ============================================================================
// dbg_snoop.v -- passive ATA taskfile logger on the HOST (iPod) bus.
// Self-contained (own synchronizers); reset by POR only so it survives host
// resets. Prints one 5-char line per event at 115200 8N1:
//
//   B=B0   power-on banner (logger alive)
//   R=00   HS_RESET_N fell        r=01  HS_RESET_N rose
//   C=xx   command-register write (the ATA opcode)
//   F=xx   features-register write (SET FEATURES subcode; 03 = set xfer mode)
//   T=xx   sector-count write following F=03 (transfer mode: 0C..0D=PIO3/4)
//   E=xx   error-register read (value the card returned)
//   S=xx   status read with ERR set (deduped until value changes)
// ============================================================================

module dbg_snoop #(
    parameter DIV = 577
) (
    input         clk,
    input         rst_n,        // POR only -- NOT gated by host reset
    input         cs0_n, cs1_n,
    input         a0, a1, a2,
    input         ior_n, iow_n,
    input         hrst_n,
    input  [15:0] dd,
    input         abort_ev,     // 1-cycle: UDMA watchdog abort
    input         end_ev,       // 1-cycle: UDMA burst completed
    input         start_ev,     // 1-cycle: UDMA burst engaged
    input  [7:0]  stat,         // sequencer/engine/FIFO snapshot
    input         init_ok,      // 1-cycle: cf_init set UDMA2 on the card
    input         init_bad,     // 1-cycle: cf_init timed out
    input  [7:0]  istat,        // CF status cf_init last saw
    input  [15:0] bcrc,         // burst CRC (valid at end_ev)
    input  [15:0] bwcnt,        // burst word count (valid at end_ev)
    input         chunk_ev,     // 1-cycle: producer chunk landed (multi-burst)
    input  [7:0]  chunk_val,    // FIFO fill, 32-word units
    input  [15:0] wcap,         // words captured into FIFO this command
    input  [7:0]  dmackf,       // HS DMACK falling edges (iSphynx burst count)
    input  [7:0]  hostq,        // autopsy
    input  [15:0] wsent,        // words host sent to CF (write)
    input         mode_ev,      // 1-cycle: CRC dialect changed
    input  [7:0]  mode_val,     // new dialect
    output        txd
);

    // ---- synchronize ----
    reg [1:0] ior_q, iow_q, rst_q;
    reg       cs0_s, cs1_s, a0_s, a1_s, a2_s;
    reg [7:0] dd_s;
    always @(posedge clk) begin
        ior_q <= {ior_q[0], ior_n};
        iow_q <= {iow_q[0], iow_n};
        rst_q <= {rst_q[0], hrst_n};
        cs0_s <= cs0_n; cs1_s <= cs1_n;
        a0_s  <= a0;    a1_s  <= a1;   a2_s <= a2;
        dd_s  <= dd[7:0];
    end
    wire iow_rise = ~iow_q[1] & iow_q[0];
    wire ior_rise = ~ior_q[1] & ior_q[0];
    wire rst_fall =   rst_q[1] & ~rst_q[0];
    wire rst_rise =  ~rst_q[1] & rst_q[0];

    wire blk  = ~cs0_s & cs1_s;               // command block selected
    wire [2:0] adr = {a2_s, a1_s, a0_s};

    // ---- event detect ----
    reg [15:0] bcrc_l, bwcnt_l, wcap_l;         // latched at end_ev/abort
    reg [7:0]  dmackf_l, hostq_l;
    reg [15:0] wsent_l;
    reg [9:0]  xtra;                           // pending K/k/W/w/P/p/N emissions
    reg [7:0] last_s;
    reg       arm_s;                           // log next status read (post-reset)
    reg        ev_wr;
    reg [15:0] ev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ev_wr <= 1'b0; ev <= 16'h0; last_s <= 8'h00;
            arm_s <= 1'b1; xtra <= 10'b0; bcrc_l <= 16'h0; bwcnt_l <= 16'h0; wcap_l <= 16'h0; dmackf_l <= 8'h0;
        end else begin
            ev_wr <= 1'b0;
            if (end_ev | abort_ev) begin bcrc_l <= bcrc; bwcnt_l <= bwcnt; wcap_l <= wcap; dmackf_l <= dmackf; hostq_l <= hostq; wsent_l <= wsent; end
            if (abort_ev)                       begin ev <= {"X", stat};  ev_wr <= 1'b1; xtra <= 7'b1111111; end
            else if (start_ev)                  begin ev <= {"U", stat};  ev_wr <= 1'b1; end
            else if (end_ev)                    begin ev <= {"u", stat};  ev_wr <= 1'b1; xtra <= 7'b1111111; end
            else if (init_ok)                   begin ev <= {"M", istat}; ev_wr <= 1'b1; end
            else if (init_bad)                  begin ev <= {"m", istat}; ev_wr <= 1'b1; end
            else if (mode_ev)                   begin ev <= {"G", mode_val}; ev_wr <= 1'b1; end
            else if (chunk_ev)                  begin ev <= {"h", chunk_val}; ev_wr <= 1'b1; end
            else if (rst_fall)                  begin ev <= {"R", 8'h00}; ev_wr <= 1'b1; end
            else if (rst_rise)                  begin ev <= {"r", 8'h01}; ev_wr <= 1'b1; arm_s <= 1'b1; end
            else if (iow_rise & blk & adr==3'd7) begin ev <= {"C", dd_s}; ev_wr <= 1'b1; last_s <= 8'h00; end
            else if (iow_rise & blk & adr==3'd1) begin ev <= {"F", dd_s}; ev_wr <= 1'b1; end
            else if (iow_rise & blk & adr==3'd2) begin ev <= {"T", dd_s}; ev_wr <= 1'b1; end
            else if (ior_rise & blk & adr==3'd1) begin ev <= {"E", dd_s}; ev_wr <= 1'b1; end
            else if (ior_rise & blk & adr!=3'd0 & adr!=3'd7) begin
                ev <= {(8'h30 + {5'b0, adr}), dd_s}; ev_wr <= 1'b1;   // '2'..'6' = reg reads
            end
            else if (ior_rise & blk & adr==3'd7 & arm_s) begin
                ev <= {"s", dd_s}; ev_wr <= 1'b1; arm_s <= 1'b0;   // first status after reset
            end
            else if (ior_rise & blk & adr==3'd7 & dd_s[0] & (dd_s != last_s)) begin
                ev <= {"S", dd_s}; ev_wr <= 1'b1; last_s <= dd_s;
            end
            // post-burst detail: CRC sent + word count (K/k = CRC hi/lo, W/w = count hi/lo)
            else if (xtra[9]) begin ev <= {"K", bcrc_l[15:8]};  ev_wr <= 1'b1; xtra[9] <= 1'b0; end
            else if (xtra[8]) begin ev <= {"k", bcrc_l[7:0]};   ev_wr <= 1'b1; xtra[8] <= 1'b0; end
            else if (xtra[7]) begin ev <= {"W", bwcnt_l[15:8]}; ev_wr <= 1'b1; xtra[7] <= 1'b0; end
            else if (xtra[6]) begin ev <= {"w", bwcnt_l[7:0]};  ev_wr <= 1'b1; xtra[6] <= 1'b0; end
            else if (xtra[5]) begin ev <= {"P", wcap_l[15:8]};  ev_wr <= 1'b1; xtra[5] <= 1'b0; end
            else if (xtra[4]) begin ev <= {"p", wcap_l[7:0]};   ev_wr <= 1'b1; xtra[4] <= 1'b0; end
            else if (xtra[3]) begin ev <= {"N", dmackf_l};      ev_wr <= 1'b1; xtra[3] <= 1'b0; end
            else if (xtra[2]) begin ev <= {"H", hostq_l};       ev_wr <= 1'b1; xtra[2] <= 1'b0; end
            else if (xtra[1]) begin ev <= {"V", wsent_l[15:8]}; ev_wr <= 1'b1; xtra[1] <= 1'b0; end
            else if (xtra[0]) begin ev <= {"v", wsent_l[7:0]};  ev_wr <= 1'b1; xtra[0] <= 1'b0; end
        end
    end

    // ---- boot banner ----
    reg booted;
    wire [15:0] ev_in  = booted ? ev    : {"B", 8'hB0};
    wire        ev_in_wr = booted ? ev_wr : 1'b1;

    // ---- event FIFO (128 deep, drop on full) ----
    reg [15:0] mem [0:127];
    reg [6:0]  wp, rp;
    reg [7:0]  cnt;
    wire full  = (cnt == 8'd128);
    wire empty = (cnt == 8'd0);
    wire pop;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= 0; rp <= 0; cnt <= 0; booted <= 1'b0;
        end else begin
            booted <= 1'b1;
            if (ev_in_wr & ~full) begin mem[wp] <= ev_in; wp <= wp + 1'b1; end
            if (pop & ~empty)     rp <= rp + 1'b1;
            case ({ev_in_wr & ~full, pop & ~empty})
                2'b10: cnt <= cnt + 1'b1;
                2'b01: cnt <= cnt - 1'b1;
                default: ;
            endcase
        end
    end
    wire [15:0] ev_out = mem[rp];

    // ---- formatter: TAG '=' HH LL CR LF ----
    function [7:0] hexch; input [3:0] n;
        hexch = (n < 4'd10) ? (8'h30 + n) : (8'h37 + n);   // 0-9, A-F
    endfunction

    reg  [2:0] fs;
    reg  [7:0] tx_data;
    reg        tx_wr;
    wire       tx_busy;

    assign pop = (fs == 3'd5) & ~tx_busy & ~tx_wr;   // single-cycle consume after LF sent

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fs <= 3'd0; tx_wr <= 1'b0; tx_data <= 8'h00;
        end else begin
            tx_wr <= 1'b0;
            if (!tx_busy & !tx_wr) begin
                if (fs == 3'd0) begin
                    if (!empty) begin tx_data <= ev_out[15:8]; tx_wr <= 1'b1; fs <= 3'd1; end
                end else begin
                    case (fs)
                        3'd1: begin tx_data <= hexch(ev_out[7:4]); tx_wr <= 1'b1; fs <= 3'd2; end
                        3'd2: begin tx_data <= hexch(ev_out[3:0]); tx_wr <= 1'b1; fs <= 3'd3; end
                        3'd3: begin tx_data <= 8'h0D;              tx_wr <= 1'b1; fs <= 3'd4; end
                        3'd4: begin tx_data <= 8'h0A;              tx_wr <= 1'b1; fs <= 3'd5; end
                        3'd5: fs <= 3'd0;
                        default: fs <= 3'd0;
                    endcase
                end
            end
        end
    end

    uart_tx #(.DIV(DIV)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .wr(tx_wr), .data(tx_data), .busy(tx_busy), .txd(txd)
    );

endmodule
