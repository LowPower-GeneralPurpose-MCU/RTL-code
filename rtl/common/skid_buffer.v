module skid_buffer # (parameter WIDTH = 32) (
    input wire clk, input wire rst_n,
    input wire s_valid, output wire s_ready, input wire [WIDTH-1:0] s_data,
    output wire m_valid, input wire m_ready, output wire [WIDTH-1:0] m_data
);
    reg [WIDTH-1:0] data_reg, skid_reg;
    reg d_valid, s_filled;
    assign s_ready = !s_filled;
    assign m_valid = d_valid || s_filled;
    assign m_data  = s_filled ? skid_reg : data_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin d_valid <= 0; s_filled <= 0; end
        else begin
            if (s_ready && s_valid) begin
                if (m_valid && !m_ready) begin skid_reg <= s_data; s_filled <= 1; end
                else begin data_reg <= s_data; d_valid <= 1; end
            end
            if (m_ready && m_valid) begin
                if (s_filled) begin data_reg <= skid_reg; s_filled <= 0; end
                else d_valid <= 0;
            end
        end
    end
endmodule