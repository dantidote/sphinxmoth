// ============================================================================
// uart_tx.v -- minimal 8N1 transmitter. DIV = clk / baud (66.5e6/115200 = 577).
// ============================================================================

module uart_tx #(
    parameter DIV = 577
) (
    input            clk,
    input            rst_n,
    input            wr,          // pulse with data while !busy
    input      [7:0] data,
    output           busy,
    output reg       txd
);

    reg [9:0]  sh;      // {stop, data[7:0], start}
    reg [3:0]  nbits;
    reg [15:0] div;

    assign busy = (nbits != 4'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txd <= 1'b1; sh <= 10'h3FF; nbits <= 4'd0; div <= 16'd0;
        end else if (!busy) begin
            txd <= 1'b1;
            if (wr) begin
                sh    <= {1'b1, data, 1'b0};
                nbits <= 4'd10;
                div   <= DIV[15:0] - 16'd1;
            end
        end else begin
            if (div == 16'd0) begin
                txd   <= sh[0];
                sh    <= {1'b1, sh[9:1]};
                nbits <= nbits - 4'd1;
                div   <= DIV[15:0] - 16'd1;
            end else
                div <= div - 16'd1;
        end
    end

endmodule
