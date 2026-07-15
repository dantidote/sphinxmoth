// ============================================================================
// sd_host.v  --  native SD/SDHC/SDXC host (v2 backend, replaces the CF leg)
// ----------------------------------------------------------------------------
// Protocol engine ONLY: init handshake, 48/136-bit CMD frames with CRC7,
// 4-bit DAT blocks with per-line CRC16, busy handling on DAT0. Policy
// (retries, sector buffering, the blk_* face toward ata_device) lives in
// backend_sd.v -- this module reports raw outcomes and never decides.
//
// Reset domain: POR ONLY. The iPod strobes ATA reset freely at boot; the card
// must not be re-initialized on each one (init costs real time and the host
// polls status ~immediately). Host resets are handled above via op_end.
//
// Clocking: SDCLK is a fabric register. One "cell" = one SDCLK period =
// 2*div_half clks: LOW for q in [0, div_half-1], HIGH for the rest.
//   - host outputs (CMD/DAT) change at cell start (SDCLK low)      [cell_start]
//   - card samples them on the rise (mid-cell)
//   - card outputs change relative to a rising edge; we sample them just
//     before the NEXT rising edge, through a 1-FF pad register     [sample_now]
//     -> a full SDCLK period minus 2 clks of settle, the safe universal point.
// Freezing the cell counter freezes SDCLK -- legal any time (SD is fully
// static); used for block-granular backpressure between read blocks.
//
//   rate      div_half @66.5MHz   use
//   ~396kHz   84                  init (spec window 100-400kHz)
//   16.6MHz   2                   transfer, default (fat margins)
//   33.25MHz  1                   transfer, SD_FAST=1 + CMD6 HS ok (validate
//                                 the sample point on silicon first)
// ============================================================================

module sd_host #(
    parameter CLK_MHZ   = 66,
    parameter SD_FAST   = 0,      // 1: attempt CMD6 High-Speed + 33.25MHz
    parameter DIV_INIT  = 84,     // div_half during init
    parameter DIV_XFER  = 2,      // div_half after init
    parameter SIM_FAST  = 0       // sim: shrink long timeouts/settle delays
) (
    input             clk,
    input             rst_n,          // POR only

    // ---- pads --------------------------------------------------------------
    output reg        sd_clk,
    output reg        sd_cmd_out,
    output reg        sd_cmd_oe,
    input             sd_cmd_in,
    output reg [3:0]  sd_dat_out,
    output reg        sd_dat_oe,      // all four lines together
    input      [3:0]  sd_dat_in,
    input             sd_cd_n,        // card-detect switch (0 = card present)

    // ---- init results -------------------------------------------------------
    output reg        init_done,
    output reg        init_fail,      // high during the retry backoff
    output reg [3:0]  init_stage,     // where init last was/died (debug)
    output reg [31:0] capacity,       // total 512B sectors
    output reg        ccs,            // 1 = SDHC/SDXC (block addressing)
    output reg        hs_on,          // CMD6 High-Speed engaged

    // ---- block operation (multi-block; policy above) -------------------------
    input             op_go,          // pulse: open CMD18/CMD25 at op_lba
    input             op_write,
    input      [31:0] op_lba,         // sector address (byte-scaled inside for SDSC)
    output reg        op_open,        // transfer phase entered (R1 accepted)
    input             blk_go,         // pulse: run the next 512B block now
    output reg        blk_done,       // pulse: block finished (see flags)
    output reg        blk_crc_ok,     // read: all four DAT CRC16s matched
    output reg        wr_acc,         // write: CRC status token 010 + busy cleared
    input             op_end,         // pulse: close the op (CMD12 + busy)
    output reg        op_idle,        // no op in flight, CMD engine quiet
    output reg        op_err,         // pulse: protocol death (timeout/status)

    // ---- read stream (block body -> backend buffer) -------------------------
    output reg        rd_v,           // one 16-bit word (LE byte pair)
    output reg [15:0] rd_w,

    // ---- write feed (backend buffer -> block body) ---------------------------
    output reg [7:0]  wr_idx,         // word index we are ABOUT to need
    input      [15:0] wr_word         // buf[wr_idx]; must be valid 2+ clks later
);

    // ------------------------------------------------------------------------
    // pad input registers
    // ------------------------------------------------------------------------
    reg        cmd_i;
    reg [3:0]  dat_i;
    always @(posedge clk) begin
        cmd_i <= sd_cmd_in;
        dat_i <= sd_dat_in;
    end

    // ------------------------------------------------------------------------
    // cell engine
    // ------------------------------------------------------------------------
    reg  [7:0] div_half;
    reg  [8:0] q;
    reg        freeze;
    wire [8:0] cell_top   = {div_half, 1'b0} - 9'd1;
    wire [8:0] q_next     = (q == cell_top) ? 9'd0 : q + 9'd1;
    wire       cell_start = (q == 9'd0) & ~freeze;
    wire       sample_now = (q == {1'b0, div_half} - 9'd1);   // just before the rise
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q <= 9'd0; sd_clk <= 1'b0;
        end else if (freeze && q == 9'd0) begin
            sd_clk <= 1'b0;                                   // parked low
        end else begin
            q      <= q_next;
            sd_clk <= (q_next >= {1'b0, div_half});
        end
    end

    // ------------------------------------------------------------------------
    // CRC steppers
    // ------------------------------------------------------------------------
    function [6:0] crc7_step;                 // x^7 + x^3 + 1
        input [6:0] c; input b;
        reg fb;
        begin
            fb = c[6] ^ b;
            crc7_step = {c[5:3], c[2] ^ fb, c[1:0], fb};
        end
    endfunction
    function [15:0] crc16_step;               // x^16 + x^12 + x^5 + 1
        input [15:0] c; input b;
        reg fb;
        begin
            fb = c[15] ^ b;
            crc16_step = {c[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0000);
        end
    endfunction

    // ------------------------------------------------------------------------
    // CMD micro-engine: send one 48-bit frame, collect an rlen-bit response
    // ------------------------------------------------------------------------
    reg          ce_go;
    reg  [5:0]   ce_idx;
    reg  [31:0]  ce_arg;
    reg  [7:0]   ce_rlen;             // 0 / 48 / 136
    reg          ce_busy_wait;        // R1b: wait DAT0 high after the response
    reg          ce_running, ce_done, ce_err;
    reg  [135:0] resp;

    localparam CE_IDLE = 3'd0, CE_TX = 3'd1, CE_GAP = 3'd2, CE_RX = 3'd3,
               CE_BUSY = 3'd4, CE_FIN = 3'd5;
    reg  [2:0]  ce;
    reg  [39:0] ce_sh;
    reg  [6:0]  ce_crc;
    reg  [7:0]  ce_n;
    reg  [7:0]  ncr;
    reg  [25:0] bto;
    reg         ce_bw;                // busy_wait, captured at ce_go
    localparam BUSY_TO = SIM_FAST ? 26'd200000 : 26'd33000000;   // 3ms sim / 500ms real

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ce <= CE_IDLE; ce_running <= 0; ce_done <= 0; ce_err <= 0;
            sd_cmd_out <= 1'b1; sd_cmd_oe <= 1'b0;
            ce_sh <= 40'h0; ce_crc <= 7'd0; ce_n <= 8'd0; ce_bw <= 1'b0;
            ncr <= 8'd0; resp <= 136'h0; bto <= 26'd0;
        end else begin
            ce_done <= 1'b0; ce_err <= 1'b0;
            case (ce)
            CE_IDLE: begin
                sd_cmd_oe <= 1'b0;
                if (ce_go) begin
                    ce_running <= 1'b1;
                    ce_sh  <= {2'b01, ce_idx, ce_arg};
                    ce_crc <= 7'd0;
                    ce_n   <= 8'd48;
                    ce_bw  <= ce_busy_wait;
                    ce     <= CE_TX;
                end
            end
            CE_TX: if (cell_start) begin
                if (ce_n != 8'd0) begin
                    sd_cmd_oe <= 1'b1;
                    if (ce_n > 8'd8) begin                     // 40 payload bits
                        sd_cmd_out <= ce_sh[39];
                        ce_crc     <= crc7_step(ce_crc, ce_sh[39]);
                        ce_sh      <= {ce_sh[38:0], 1'b0};
                    end else if (ce_n > 8'd1) begin            // 7 CRC bits
                        sd_cmd_out <= ce_crc[6];
                        ce_crc     <= {ce_crc[5:0], 1'b0};
                    end else begin
                        sd_cmd_out <= 1'b1;                    // end bit
                    end
                    ce_n <= ce_n - 8'd1;
                end else begin
                    sd_cmd_oe <= 1'b0;                         // release the line
                    if (ce_rlen == 8'd0) ce <= CE_FIN;
                    else begin ncr <= 8'd0; ce <= CE_GAP; end
                end
            end
            CE_GAP:                                            // hunt the start bit
                if (sample_now) begin
                    if (!cmd_i) begin
                        resp <= {135'h0, 1'b0};
                        ce_n <= ce_rlen - 8'd1;
                        ce   <= CE_RX;
                    end else if (ncr == 8'd200) begin
                        ce_err <= 1'b1; ce_running <= 1'b0; ce <= CE_IDLE;
                    end else
                        ncr <= ncr + 8'd1;
                end
            CE_RX:
                if (sample_now) begin
                    resp <= {resp[134:0], cmd_i};
                    if (ce_n == 8'd1) begin
                        if (ce_bw) begin
                            bto <= BUSY_TO;
                            ncr <= 8'd4;             // grace: busy may lag the
                            ce  <= CE_BUSY;          // end bit by ~2 clocks
                        end else ce <= CE_FIN;
                    end
                    ce_n <= ce_n - 8'd1;
                end
            CE_BUSY:                                           // R1b: DAT0 low = busy
                if (sample_now && ncr != 8'd0) ncr <= ncr - 8'd1;
                else if (sample_now && dat_i[0]) ce <= CE_FIN;
                else if (bto == 26'd0) begin
                    ce_err <= 1'b1; ce_running <= 1'b0; ce <= CE_IDLE;
                end else bto <= bto - 26'd1;
            CE_FIN: begin
                ce_done <= 1'b1; ce_running <= 1'b0; ce <= CE_IDLE;
            end
            default: ce <= CE_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // DAT micro-engine: move one block (512B data / 64B CMD6 status), 4-bit
    // ------------------------------------------------------------------------
    reg         de_rd_go, de_wr_go;
    reg  [10:0] de_bytes;
    reg         de_done, de_crc_ok, de_wr_acc;

    localparam DE_IDLE = 4'd0, DE_RSTART = 4'd1, DE_RBODY = 4'd2, DE_RCRC = 4'd3,
               DE_REND = 4'd4, DE_WLEAD = 4'd5, DE_WSTART = 4'd6, DE_WBODY = 4'd7,
               DE_WCRC = 4'd8, DE_WEND = 4'd9, DE_WREL = 4'd10, DE_WTOK = 4'd11,
               DE_WBUSY = 4'd12, DE_FIN = 4'd13;
    reg  [3:0]  de;
    reg  [11:0] de_nib;
    reg  [7:0]  de_lo;
    reg         de_phase;             // 0 = low byte of the pair under assembly
    reg  [3:0]  de_hi_nib;
    reg         de_half;              // 0 = expecting the byte's high nibble
    reg  [21:0] dto;
    localparam DTO_RD = SIM_FAST ? 22'd50000 : 22'd2000000;    // read start (cells)
    reg  [25:0] bto2;
    reg  [2:0]  tok;
    reg  [2:0]  tok_n;
    reg  [15:0] dcrc0, dcrc1, dcrc2, dcrc3;
    reg  [15:0] dref0, dref1, dref2, dref3;
    reg  [15:0] de_wword;
    reg  [1:0]  de_wn;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de <= DE_IDLE; de_done <= 0; de_crc_ok <= 0; de_wr_acc <= 0;
            sd_dat_out <= 4'hF; sd_dat_oe <= 1'b0;
            de_nib <= 12'd0; de_phase <= 1'b0; de_half <= 1'b0;
            de_hi_nib <= 4'h0; de_lo <= 8'h0; dto <= 22'd0; bto2 <= 26'd0;
            tok <= 3'b0; tok_n <= 3'd0;
            rd_v <= 1'b0; rd_w <= 16'h0; wr_idx <= 8'd0;
            de_wword <= 16'h0; de_wn <= 2'd0;
            dcrc0 <= 0; dcrc1 <= 0; dcrc2 <= 0; dcrc3 <= 0;
            dref0 <= 0; dref1 <= 0; dref2 <= 0; dref3 <= 0;
        end else begin
            de_done <= 1'b0; rd_v <= 1'b0;
            case (de)
            DE_IDLE: begin
                sd_dat_oe <= 1'b0;
                if (de_rd_go) begin
                    de_nib  <= {de_bytes, 1'b0};
                    de_half <= 1'b0; de_phase <= 1'b0;
                    dto     <= DTO_RD;
                    dcrc0 <= 0; dcrc1 <= 0; dcrc2 <= 0; dcrc3 <= 0;
                    de <= DE_RSTART;
                end else if (de_wr_go) begin
                    de_nib <= {de_bytes, 1'b0};
                    de_wn  <= 2'd0;
                    wr_idx <= 8'd0;                            // prefetch word 0
                    dcrc0 <= 0; dcrc1 <= 0; dcrc2 <= 0; dcrc3 <= 0;
                    dto   <= 22'd4;                            // Nwr lead-in cells
                    de <= DE_WLEAD;
                end
            end
            // ================= READ =================
            DE_RSTART:
                if (sample_now) begin
                    if (!dat_i[0]) de <= DE_RBODY;             // start nibble
                    else if (dto == 22'd0) begin
                        de_crc_ok <= 1'b0; de <= DE_FIN;       // data-start timeout
                    end else dto <= dto - 22'd1;
                end
            DE_RBODY:
                if (sample_now) begin
                    dcrc0 <= crc16_step(dcrc0, dat_i[0]);
                    dcrc1 <= crc16_step(dcrc1, dat_i[1]);
                    dcrc2 <= crc16_step(dcrc2, dat_i[2]);
                    dcrc3 <= crc16_step(dcrc3, dat_i[3]);
                    if (!de_half) begin
                        de_hi_nib <= dat_i;
                        de_half   <= 1'b1;
                    end else begin
                        de_half <= 1'b0;
                        if (!de_phase) begin
                            de_lo    <= {de_hi_nib, dat_i};
                            de_phase <= 1'b1;
                        end else begin
                            rd_w     <= {{de_hi_nib, dat_i}, de_lo};   // {odd, even}
                            rd_v     <= 1'b1;
                            de_phase <= 1'b0;
                        end
                    end
                    if (de_nib == 12'd1) begin
                        de_nib <= 12'd16;
                        de <= DE_RCRC;
                    end else de_nib <= de_nib - 12'd1;
                end
            DE_RCRC:
                if (sample_now) begin
                    dref0 <= {dref0[14:0], dat_i[0]};
                    dref1 <= {dref1[14:0], dat_i[1]};
                    dref2 <= {dref2[14:0], dat_i[2]};
                    dref3 <= {dref3[14:0], dat_i[3]};
                    if (de_nib == 12'd1) de <= DE_REND;
                    else de_nib <= de_nib - 12'd1;
                end
            DE_REND:
                if (sample_now) begin                          // end bit slot
                    de_crc_ok <= (dcrc0 == dref0) & (dcrc1 == dref1)
                               & (dcrc2 == dref2) & (dcrc3 == dref3);
                    de <= DE_FIN;
                end
            // ================= WRITE =================
            DE_WLEAD:                                          // hold lines high >=2 cells
                if (cell_start) begin
                    sd_dat_oe  <= 1'b1;
                    sd_dat_out <= 4'hF;
                    if (dto == 22'd0) de <= DE_WSTART;
                    else dto <= dto - 22'd1;
                end
            DE_WSTART:
                if (cell_start) begin
                    sd_dat_out <= 4'h0;                        // start nibble
                    de_wword   <= wr_word;                     // word 0 (prefetched)
                    de_wn      <= 2'd0;
                    wr_idx     <= 8'd1;
                    de <= DE_WBODY;
                end
            DE_WBODY:
                if (cell_start) begin
                    // LE byte pair, high nibble of each byte first; DAT3 = nibble MSB
                    case (de_wn)
                    2'd0: begin
                        sd_dat_out <= de_wword[7:4];
                        dcrc0 <= crc16_step(dcrc0, de_wword[4]);
                        dcrc1 <= crc16_step(dcrc1, de_wword[5]);
                        dcrc2 <= crc16_step(dcrc2, de_wword[6]);
                        dcrc3 <= crc16_step(dcrc3, de_wword[7]);
                    end
                    2'd1: begin
                        sd_dat_out <= de_wword[3:0];
                        dcrc0 <= crc16_step(dcrc0, de_wword[0]);
                        dcrc1 <= crc16_step(dcrc1, de_wword[1]);
                        dcrc2 <= crc16_step(dcrc2, de_wword[2]);
                        dcrc3 <= crc16_step(dcrc3, de_wword[3]);
                    end
                    2'd2: begin
                        sd_dat_out <= de_wword[15:12];
                        dcrc0 <= crc16_step(dcrc0, de_wword[12]);
                        dcrc1 <= crc16_step(dcrc1, de_wword[13]);
                        dcrc2 <= crc16_step(dcrc2, de_wword[14]);
                        dcrc3 <= crc16_step(dcrc3, de_wword[15]);
                    end
                    2'd3: begin
                        sd_dat_out <= de_wword[11:8];
                        dcrc0 <= crc16_step(dcrc0, de_wword[8]);
                        dcrc1 <= crc16_step(dcrc1, de_wword[9]);
                        dcrc2 <= crc16_step(dcrc2, de_wword[10]);
                        dcrc3 <= crc16_step(dcrc3, de_wword[11]);
                        de_wword <= wr_word;                   // prefetched next word
                        wr_idx   <= wr_idx + 8'd1;
                    end
                    endcase
                    de_wn <= de_wn + 2'd1;
                    if (de_nib == 12'd1) begin
                        de_nib <= 12'd16;
                        de <= DE_WCRC;
                    end else de_nib <= de_nib - 12'd1;
                end
            DE_WCRC:
                if (cell_start) begin
                    sd_dat_out <= {dcrc3[15], dcrc2[15], dcrc1[15], dcrc0[15]};
                    dcrc0 <= {dcrc0[14:0], 1'b0};
                    dcrc1 <= {dcrc1[14:0], 1'b0};
                    dcrc2 <= {dcrc2[14:0], 1'b0};
                    dcrc3 <= {dcrc3[14:0], 1'b0};
                    if (de_nib == 12'd1) de <= DE_WEND;
                    else de_nib <= de_nib - 12'd1;
                end
            DE_WEND:
                if (cell_start) begin
                    sd_dat_out <= 4'hF;                        // end bit
                    de <= DE_WREL;
                end
            DE_WREL:
                if (cell_start) begin
                    sd_dat_oe <= 1'b0;                         // hand DAT to the card
                    tok_n <= 3'd0;
                    dto   <= 22'd64;                           // token must be prompt
                    de <= DE_WTOK;
                end
            DE_WTOK:                                           // CRC status: 0 s2 s1 s0 1
                if (sample_now) begin
                    if (tok_n == 3'd0) begin
                        if (!dat_i[0]) tok_n <= 3'd1;          // start bit found
                        else if (dto == 22'd0) begin
                            de_wr_acc <= 1'b0; de <= DE_FIN;
                        end else dto <= dto - 22'd1;
                    end else if (tok_n <= 3'd3) begin
                        tok   <= {tok[1:0], dat_i[0]};
                        tok_n <= tok_n + 3'd1;
                    end else begin                             // end bit slot
                        de_wr_acc <= (tok == 3'b010);
                        bto2  <= BUSY_TO;
                        tok_n <= 3'd2;               // grace before busy-sampling
                        de <= DE_WBUSY;
                    end
                end
            DE_WBUSY:                                          // card programs: DAT0 low
                if (sample_now && tok_n != 3'd0) tok_n <= tok_n - 3'd1;
                else if (sample_now && dat_i[0]) de <= DE_FIN;
                else if (bto2 == 26'd0) begin
                    de_wr_acc <= 1'b0; de <= DE_FIN;
                end else bto2 <= bto2 - 26'd1;
            DE_FIN: begin
                de_done <= 1'b1;
                de <= DE_IDLE;
            end
            default: de <= DE_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // master FSM: init sequence, then the op layer
    // ------------------------------------------------------------------------
    localparam M_COLD   = 5'd0,  M_74     = 5'd1,  M_CMD0  = 5'd2,  M_CMD8   = 5'd3,
               M_CMD55A = 5'd4,  M_ACMD41 = 5'd5,  M_A41W  = 5'd6,  M_CMD2   = 5'd7,
               M_CMD3   = 5'd8,  M_CMD9   = 5'd9,  M_CMD7  = 5'd10, M_CMD55B = 5'd11,
               M_ACMD6  = 5'd12, M_CMD16  = 5'd13, M_HSBLK = 5'd14,
               M_READY  = 5'd15, M_OPCMD  = 5'd16,
               M_RDGAP  = 5'd17, M_RDBLK  = 5'd18, M_WRGAP  = 5'd19, M_WRBLK = 5'd20,
               M_STOP   = 5'd21, M_FAIL   = 5'd22, M_RETRY  = 5'd23;
    reg  [4:0]  m;
    reg  [15:0] rca;
    reg  [7:0]  m74;
    reg  [23:0] mdly;
    reg  [119:0] csd;                 // CSD[127:8]
    reg         op_write_l;
    reg         end_pend;
    reg         sdsc_v1;              // no CMD8 echo: SD v1.x, no HCS
    localparam COLD_DLY = SIM_FAST ? 24'd400 : 24'd1400000;    // card-settle after CD

    wire [11:0] v1_csize = csd[65:54];        // CSD[73:62]
    wire [2:0]  v1_cmult = csd[41:39];        // CSD[49:47]
    wire [3:0]  v1_rbl   = csd[75:72];        // CSD[83:80]
    wire [21:0] v2_csize = csd[61:40];        // CSD[69:48]
    wire [4:0]  v1_shift = {2'b0, v1_cmult} + 5'd2 + {1'b0, v1_rbl} - 5'd9;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m <= M_COLD; div_half <= DIV_INIT[7:0]; freeze <= 1'b0;
            init_done <= 0; init_fail <= 0; init_stage <= 4'd0;
            capacity <= 32'd0; ccs <= 0; hs_on <= 0;
            ce_go <= 0; ce_idx <= 6'd0; ce_arg <= 32'h0; ce_rlen <= 8'd0; ce_busy_wait <= 0;
            de_rd_go <= 0; de_wr_go <= 0; de_bytes <= 11'd512;
            op_open <= 0; op_idle <= 0; op_err <= 0; blk_done <= 0;
            blk_crc_ok <= 0; wr_acc <= 0;
            rca <= 16'h0; m74 <= 8'd0; mdly <= 24'd0; csd <= 120'h0;
            op_write_l <= 0; end_pend <= 0; sdsc_v1 <= 0;
        end else begin
            ce_go <= 1'b0; de_rd_go <= 1'b0; de_wr_go <= 1'b0;
            op_err <= 1'b0; blk_done <= 1'b0;
            if (op_end) end_pend <= 1'b1;
            op_idle <= (m == M_READY) && !ce_running && !ce_go;

            case (m)
            // ================= INIT =================
            M_COLD: begin
                init_done <= 1'b0; hs_on <= 1'b0;
                if (!sd_cd_n) begin
                    if (mdly == COLD_DLY) begin m74 <= 8'd80; mdly <= 24'd0; m <= M_74; end
                    else mdly <= mdly + 24'd1;
                end else mdly <= 24'd0;
            end
            M_74: if (cell_start) begin                        // 74+ warm-up clocks
                if (m74 == 8'd0) begin
                    ce_idx <= 6'd0; ce_arg <= 32'h0; ce_rlen <= 8'd0; ce_busy_wait <= 0;
                    ce_go <= 1'b1; init_stage <= 4'd1; m <= M_CMD0;
                end else m74 <= m74 - 8'd1;
            end
            M_CMD0: if (ce_done) begin
                ce_idx <= 6'd8; ce_arg <= 32'h000001AA; ce_rlen <= 8'd48;
                ce_go <= 1'b1; init_stage <= 4'd2; m <= M_CMD8;
            end
            M_CMD8: begin
                if (ce_done) begin
                    if (resp[19:8] != 12'h1AA) m <= M_FAIL;    // bad echo = dead bus
                    else begin
                        sdsc_v1 <= 1'b0;
                        ce_idx <= 6'd55; ce_arg <= 32'h0; ce_rlen <= 8'd48;
                        ce_go <= 1'b1; init_stage <= 4'd3; m <= M_CMD55A;
                    end
                end else if (ce_err) begin                     // SD v1.x: no CMD8
                    sdsc_v1 <= 1'b1;
                    ce_idx <= 6'd55; ce_arg <= 32'h0; ce_rlen <= 8'd48;
                    ce_go <= 1'b1; init_stage <= 4'd3; m <= M_CMD55A;
                end
            end
            M_CMD55A: begin
                if (ce_done) begin
                    ce_idx <= 6'd41;
                    ce_arg <= sdsc_v1 ? 32'h00FF8000 : 32'h40FF8000;
                    ce_rlen <= 8'd48;
                    ce_go <= 1'b1; m <= M_ACMD41;
                end else if (ce_err) m <= M_FAIL;
            end
            M_ACMD41: begin
                if (ce_done) begin
                    if (resp[39]) begin                        // OCR bit31: power-up done
                        ccs <= resp[38];
                        ce_idx <= 6'd2; ce_arg <= 32'h0; ce_rlen <= 8'd136;
                        ce_go <= 1'b1; init_stage <= 4'd4; m <= M_CMD2;
                    end else begin
                        mdly <= SIM_FAST ? 24'd200 : 24'd700000;   // ~10ms
                        m <= M_A41W;
                    end
                end else if (ce_err) m <= M_FAIL;
            end
            M_A41W: if (mdly == 24'd0) begin
                ce_idx <= 6'd55; ce_arg <= 32'h0; ce_rlen <= 8'd48;
                ce_go <= 1'b1; m <= M_CMD55A;
            end else mdly <= mdly - 24'd1;
            M_CMD2: begin
                if (ce_done) begin
                    ce_idx <= 6'd3; ce_arg <= 32'h0; ce_rlen <= 8'd48;
                    ce_go <= 1'b1; init_stage <= 4'd5; m <= M_CMD3;
                end else if (ce_err) m <= M_FAIL;
            end
            M_CMD3: begin
                if (ce_done) begin
                    rca <= resp[39:24];
                    ce_idx <= 6'd9; ce_arg <= {resp[39:24], 16'h0}; ce_rlen <= 8'd136;
                    ce_go <= 1'b1; init_stage <= 4'd6; m <= M_CMD9;
                end else if (ce_err) m <= M_FAIL;
            end
            M_CMD9: begin
                if (ce_done) begin
                    csd <= resp[127:8];
                    ce_idx <= 6'd7; ce_arg <= {rca, 16'h0}; ce_rlen <= 8'd48;
                    ce_busy_wait <= 1'b1;
                    ce_go <= 1'b1; init_stage <= 4'd7; m <= M_CMD7;
                end else if (ce_err) m <= M_FAIL;
            end
            M_CMD7: begin
                if (ce_done) begin
                    ce_busy_wait <= 1'b0;
                    capacity <= (csd[119:118] == 2'b01)
                              ? (({10'b0, v2_csize} + 32'd1) << 10)
                              : (({20'b0, v1_csize} + 32'd1) << v1_shift);
                    ce_idx <= 6'd55; ce_arg <= {rca, 16'h0}; ce_rlen <= 8'd48;
                    ce_go <= 1'b1; init_stage <= 4'd8; m <= M_CMD55B;
                end else if (ce_err) begin ce_busy_wait <= 1'b0; m <= M_FAIL; end
            end
            M_CMD55B: begin
                if (ce_done) begin
                    ce_idx <= 6'd6; ce_arg <= 32'h2; ce_rlen <= 8'd48;  // ACMD6: 4-bit
                    ce_go <= 1'b1; m <= M_ACMD6;
                end else if (ce_err) m <= M_FAIL;
            end
            M_ACMD6: begin
                if (ce_done) begin
                    if (!ccs) begin
                        ce_idx <= 6'd16; ce_arg <= 32'd512; ce_rlen <= 8'd48;
                        ce_go <= 1'b1; init_stage <= 4'd9; m <= M_CMD16;
                    end else if (SD_FAST != 0) begin
                        ce_idx <= 6'd6; ce_arg <= 32'h80FFFFF1; ce_rlen <= 8'd48;
                        ce_go <= 1'b1;
                        de_bytes <= 11'd64; de_rd_go <= 1'b1;
                        init_stage <= 4'd10; m <= M_HSBLK;
                    end else begin
                        div_half <= DIV_XFER[7:0];
                        init_done <= 1'b1; init_stage <= 4'd15; m <= M_READY;
                    end
                end else if (ce_err) m <= M_FAIL;
            end
            M_CMD16: begin
                if (ce_done) begin
                    if (SD_FAST != 0) begin
                        ce_idx <= 6'd6; ce_arg <= 32'h80FFFFF1; ce_rlen <= 8'd48;
                        ce_go <= 1'b1;
                        de_bytes <= 11'd64; de_rd_go <= 1'b1;
                        init_stage <= 4'd10; m <= M_HSBLK;
                    end else begin
                        div_half <= DIV_XFER[7:0];
                        init_done <= 1'b1; init_stage <= 4'd15; m <= M_READY;
                    end
                end else if (ce_err) m <= M_FAIL;
            end
            M_HSBLK: if (de_done) begin                        // CMD6 status block
                hs_on    <= de_crc_ok;
                div_half <= de_crc_ok ? 8'd1 : DIV_XFER[7:0];
                init_done <= 1'b1; init_stage <= 4'd15; m <= M_READY;
            end
            // ================= OP LAYER =================
            M_READY: begin
                op_open <= 1'b0; end_pend <= 1'b0; freeze <= 1'b0;
                ce_busy_wait <= 1'b0;
                de_bytes <= 11'd512;
                if (sd_cd_n) begin                             // card yanked
                    init_done <= 1'b0; mdly <= 24'd0; m <= M_COLD;
                end else if (op_go && !ce_running) begin
                    op_write_l <= op_write;
                    ce_idx <= op_write ? 6'd25 : 6'd18;
                    ce_arg <= ccs ? op_lba : (op_lba << 9);
                    ce_rlen <= 8'd48;
                    ce_go  <= 1'b1;
                    m <= M_OPCMD;
                end
            end
            M_OPCMD: begin
                if (ce_done) begin
                    if ((resp[39:8] & 32'hFDF98008) != 32'h0) begin
                        op_err <= 1'b1; m <= M_READY;
                    end else begin
                        op_open <= 1'b1;
                        m <= op_write_l ? M_WRGAP : M_RDGAP;
                    end
                end else if (ce_err) begin
                    op_err <= 1'b1; m <= M_READY;
                end
            end
            M_RDGAP: begin
                freeze <= 1'b1;                                // hold the stream
                if (end_pend) begin freeze <= 1'b0; m <= M_STOP; end
                else if (blk_go) begin
                    freeze <= 1'b0;
                    de_rd_go <= 1'b1;
                    m <= M_RDBLK;
                end
            end
            M_RDBLK: if (de_done) begin
                blk_crc_ok <= de_crc_ok;
                blk_done   <= 1'b1;
                m <= M_RDGAP;
            end
            M_WRGAP: begin                                     // clock free-runs; card waits
                if (end_pend) m <= M_STOP;
                else if (blk_go) begin
                    de_wr_go <= 1'b1;
                    m <= M_WRBLK;
                end
            end
            M_WRBLK: if (de_done) begin
                wr_acc   <= de_wr_acc;
                blk_done <= 1'b1;
                m <= M_WRGAP;
            end
            M_STOP: begin
                end_pend <= 1'b0;
                op_open  <= 1'b0;
                ce_idx <= 6'd12; ce_arg <= 32'h0; ce_rlen <= 8'd48; ce_busy_wait <= 1'b1;
                ce_go <= 1'b1;
                m <= M_READY;                    // CE finishes CMD12 in the background;
            end                                  // op_idle waits for it (ce_running)
            M_FAIL: begin
                init_done <= 1'b0;
                init_fail <= 1'b1;
                mdly <= SIM_FAST ? 24'd2000 : 24'd6650000;     // ~100ms, then retry
                m <= M_RETRY;
            end
            M_RETRY: if (mdly == 24'd0) begin
                init_fail <= 1'b0;
                div_half <= DIV_INIT[7:0];
                m <= M_COLD;
            end else mdly <= mdly - 24'd1;
            default: m <= M_COLD;
            endcase
        end
    end

endmodule
