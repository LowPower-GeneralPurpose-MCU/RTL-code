`timescale 1ns / 1ps

module clock_gate (
    input  wire clk_in,
    input  wire en,
    input  wire test_en, // Chân này rất quan trọng khi mang chip đi sản xuất (DFT - Design For Test), bình thường nối 0
    output wire clk_out
);

    reg en_latch;

    // Latch trong suốt (transparent) khi Clock ở mức THẤP
    // Điều này đảm bảo khi Clock đang ở mức CAO, dù tín hiệu 'en' có thay đổi lộn xộn
    // thì 'en_latch' vẫn giữ nguyên, tránh tạo ra xung Glitch cưa đôi Clock.
    always @(clk_in or en or test_en) begin
        if (!clk_in) begin
            en_latch = en | test_en;
        end
    end

    // Gating bằng cổng AND
    assign clk_out = clk_in & en_latch;

endmodule