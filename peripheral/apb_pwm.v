`timescale 1ns / 1ps

module apb_pwm #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    input  wire                   pclk,
    input  wire                   presetn,
    input  wire [ADDR_WIDTH-1:0]  paddr,
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [DATA_WIDTH-1:0]  pwdata,
    input  wire [3:0]             pstrb,
    output reg                    pready,
    output reg  [DATA_WIDTH-1:0]  prdata,
    output reg                    pslverr,
    
    // Tín hiệu PWM xuất ra chân vật lý
    output reg                    pwm_out
);

    // =========================================
    // REGISTER MAP
    // 0x00: CTRL    [0] Enable PWM
    // 0x04: PERIOD  Chu kỳ tổng cộng của xung
    // 0x08: DUTY    Độ rộng mức cao (phải <= PERIOD)
    // 0x0C: VAL     Giá trị đếm hiện tại (Read-only)
    // =========================================

    reg [31:0] reg_ctrl;
    reg [31:0] reg_period;
    reg [31:0] reg_duty;
    reg [31:0] reg_val;
    
    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite; // Đọc ở pha Setup

    // --- Logic Giao tiếp APB ---
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl   <= 32'b0;
            reg_period <= 32'd1000; // Giá trị mặc định an toàn
            reg_duty   <= 32'd500;
            pready     <= 1'b0;
            prdata     <= 32'b0;
            pslverr    <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;

            // Xử lý Ghi
            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: reg_ctrl   <= pwdata;
                    12'h004: reg_period <= pwdata;
                    12'h008: reg_duty   <= pwdata;
                    12'h00C: pslverr    <= 1'b1; // Read-only
                    default: pslverr    <= 1'b1;
                endcase
            end
            
            // Xử lý Đọc
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= reg_ctrl;
                    12'h004: prdata <= reg_period;
                    12'h008: prdata <= reg_duty;
                    12'h00C: prdata <= reg_val;
                    default: begin prdata <= 32'h0; pslverr <= 1'b1; end
                endcase
            end
        end
    end

    // --- Logic Tạo xung PWM ---
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_val <= 32'b0;
            pwm_out <= 1'b0;
        end else begin
            if (reg_ctrl[0]) begin
                // Bộ đếm
                if (reg_val >= reg_period) begin
                    reg_val <= 32'b0;
                end else begin
                    reg_val <= reg_val + 1'b1;
                end
                
                // So sánh tạo PWM
                if (reg_val < reg_duty) begin
                    pwm_out <= 1'b1;
                end else begin
                    pwm_out <= 1'b0;
                end
            end else begin
                // Khi tắt PWM, reset counter và kéo chân xuống 0
                reg_val <= 32'b0;
                pwm_out <= 1'b0;
            end
        end
    end

endmodule