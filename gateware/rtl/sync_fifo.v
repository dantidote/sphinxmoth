// ============================================================================
// sync_fifo.v  --  single-clock FIRST-WORD-FALL-THROUGH FIFO (burst buffer)
// ----------------------------------------------------------------------------
// Decouples the iSphynx-facing UDMA2 *device* engine from the CF-facing UDMA2
// *host* engine so the bridge is store-and-forward and immune to the iSphynx's
// non-compliant burst termination.
//
// FWFT semantics (what the engines rely on): when `empty`==0, `rd_data` already
// shows the head word; pulse `rd_en` to CONSUME it and present the next. This
// matches the engines' "present head, pulse to advance" loop. Built over a
// registered-read RAM core (EBR-friendly on MachXO2) + a prefetch output stage.
// ============================================================================

module sync_fifo #(
    parameter WIDTH = 16,
    parameter DEPTH = 512,                 // power of two
    parameter AW    = 9                     // log2(DEPTH)
) (
    input              clk,
    input              rst_n,
    input              wr_en,
    input  [WIDTH-1:0] wr_data,
    output             full,
    input              rd_en,
    output [WIDTH-1:0] rd_data,
    output             empty,
    output [AW:0]      count
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW:0]      wr_ptr = 0;
    reg [AW:0]      rd_ptr = 0;

    wire [AW:0] used       = wr_ptr - rd_ptr;     // items still in the RAM core
    wire        core_empty = (used == 0);
    wire        do_wr      = wr_en & ~full;

    // ---- write ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) wr_ptr <= 0;
        else if (do_wr) begin
            mem[wr_ptr[AW-1:0]] <= wr_data;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    // ---- FWFT prefetch output stage ----
    reg [WIDTH-1:0] out_data;
    reg             out_valid = 0;

    wire consume    = rd_en & out_valid;          // downstream takes the head now
    wire need_fill  = ~out_valid | consume;       // output slot will be free
    wire do_core_rd = need_fill & ~core_empty;    // pull next from the RAM core

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr    <= 0;
            out_valid <= 1'b0;
            out_data  <= 0;
        end else begin
            if (do_core_rd) begin
                out_data  <= mem[rd_ptr[AW-1:0]];
                rd_ptr    <= rd_ptr + 1'b1;
                out_valid <= 1'b1;
            end else if (consume) begin
                out_valid <= 1'b0;                 // consumed, nothing to refill
            end
        end
    end

    assign rd_data = out_data;
    assign empty   = ~out_valid;
    assign full    = (used == DEPTH[AW:0]);
    assign count   = used + {{AW{1'b0}}, out_valid};

endmodule
