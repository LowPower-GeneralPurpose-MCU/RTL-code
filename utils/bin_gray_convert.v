`timescale 1ns / 1ps

module bin2gray_converter #(
    parameter DATA_WIDTH = 4
)(
    input  wire [DATA_WIDTH-1:0] bin_i,
    output wire [DATA_WIDTH-1:0] gray_o
);

    // Dịch phải 1 bit và XOR với chính nó
    assign gray_o = bin_i ^ (bin_i >> 1);

endmodule

module gray2bin_converter #(
    parameter DATA_WIDTH = 4
)(
    input  wire [DATA_WIDTH-1:0] gray_i,
    output reg  [DATA_WIDTH-1:0] bin_o
);

    integer i;
    always @(*) begin
        // Bit cao nhất (MSB) của Binary luôn bằng bit MSB của Gray
        bin_o[DATA_WIDTH-1] = gray_i[DATA_WIDTH-1];
        
        // Các bit còn lại được dịch và XOR lan truyền xuống
        for (i = DATA_WIDTH-2; i >= 0; i = i - 1) begin
            bin_o[i] = bin_o[i+1] ^ gray_i[i];
        end
    end

endmodule