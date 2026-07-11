// ============================================================================
// cmd_snoop.v  --  ATA taskfile command-register snoop + DMA decode
// ----------------------------------------------------------------------------
// Watches the PIO writes that pass through to the CF and fires a one-cycle
// pulse when a *command* register write completes, telling the sequencer "a
// command just landed, here's whether it's DMA and which direction." The
// command register is the command-block register at CS0#=0, CS1#=1, A[2:0]=7,
// captured on the trailing (rising) edge of IOW#.
//
// is_dma / dir_write are registered together with cmd_valid (decoded from the
// data on the bus at capture time) so they are valid on the same cycle as the
// pulse -- do not decode the registered `cmd` combinationally, it updates a
// cycle later.
// ============================================================================

module cmd_snoop (
    input            clk,
    input            rst_n,
    input            cs0_n,
    input            cs1_n,
    input            a2, a1, a0,
    input            iow_n,
    input      [7:0] dd,

    output reg       cmd_valid,   // 1-cycle pulse: a command-reg write completed
    output reg [7:0] cmd,         // the captured command (for debug/logging)
    output reg       is_dma,      // valid with cmd_valid
    output reg       dir_write    // valid with cmd_valid: 1 = DMA write, 0 = DMA read
);

    // command register = command block (CS0#), offset 7
    wire cmd_sel = ~cs0_n & cs1_n & a2 & a1 & a0;

    // DMA opcode decode straight off the bus
    wire dma_rd = (dd == 8'hC8) | (dd == 8'hC9) |   // READ DMA / (retry variant)
                  (dd == 8'h25) |                    // READ DMA EXT
                  (dd == 8'hC7) | (dd == 8'h26);     // READ DMA QUEUED / EXT
    wire dma_wr = (dd == 8'hCA) | (dd == 8'hCB) |   // WRITE DMA / (retry variant)
                  (dd == 8'h35) |                    // WRITE DMA EXT
                  (dd == 8'hCC) | (dd == 8'h36);     // WRITE DMA QUEUED / EXT

    reg iow_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            iow_q     <= 1'b1;
            cmd_valid <= 1'b0;
            cmd       <= 8'h00;
            is_dma    <= 1'b0;
            dir_write <= 1'b0;
        end else begin
            iow_q     <= iow_n;
            cmd_valid <= 1'b0;
            if (iow_n & ~iow_q & cmd_sel) begin     // rising edge of IOW# at cmd reg
                cmd       <= dd;
                is_dma    <= dma_rd | dma_wr;
                dir_write <= dma_wr;
                cmd_valid <= 1'b1;
            end
        end
    end

endmodule
