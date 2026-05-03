`timescale 1ns / 1ps

module clock_gate (
    input  wire clk_in,   // Xung nhịp gốc liên tục
    input  wire en,       // Tín hiệu cho phép (1: Cấp clock, 0: Cắt clock)
    input  wire test_en,  // Chân dành cho chế độ Test mạch (DFT), thường nối 0
    output wire clk_out   // Xung nhịp đầu ra đã được Gating
);

    reg latch_en;

    // Latch trong suốt (Transparent Latch) khi clock ở mức thấp
    always @(clk_in or en or test_en) begin
        if (!clk_in) begin
            latch_en <= en | test_en;
        end
    end

    // Gating clock an toàn bằng cổng AND sau Latch
    assign clk_out = clk_in & latch_en;

endmodule