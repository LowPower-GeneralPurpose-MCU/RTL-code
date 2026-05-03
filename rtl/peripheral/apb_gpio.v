`timescale 1ns / 1ps

module apb_gpio #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    // --- Giao tiếp APB ---
    input  wire                  pclk,
    input  wire                  presetn,
    input  wire                  psel,
    input  wire                  penable,
    input  wire                  pwrite,
    input  wire [ADDR_WIDTH-1:0] paddr,
    input  wire [DATA_WIDTH-1:0] pwdata,
    output reg                   pready,
    output reg  [DATA_WIDTH-1:0] prdata,
    output reg                   pslverr,

    // --- Chân vật lý GPIO ---
    input  wire [31:0]           gpio_in,  // Tín hiệu vào từ Pad
    output reg  [31:0]           gpio_out, // Tín hiệu ra Pad
    output reg  [31:0]           gpio_dir, // Hướng: 1 là Output, 0 là Input
    output wire                  gpio_irq  // Tín hiệu ngắt gửi tới PLIC
);

    // --- Register Map Đơn Giản ---
    // 0x00: DATA_IN  (Read Only)  - Đọc trạng thái thực tế của chân
    // 0x04: DATA_OUT (Read/Write) - Giá trị logic xuất ra chân
    // 0x08: DIR      (Read/Write) - Cấu hình hướng (1: Out, 0: In)
    // 0x0C: INT_MASK (Read/Write) - Mặt nạ ngắt (1: Cho phép ngắt khi chân Input thay đổi)
    // 0x10: INT_STAT (Read/W1C)   - Trạng thái ngắt (Ghi 1 để xóa)

    reg [31:0] reg_int_mask;
    reg [31:0] reg_int_stat;

    // --- Mạch đồng bộ tín hiệu vào (Bắt buộc phải có) ---
    reg [31:0] sync_f1, sync_f2, sync_f3;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            sync_f1 <= 32'd0;
            sync_f2 <= 32'd0;
            sync_f3 <= 32'd0;
        end else begin
            sync_f1 <= gpio_in;
            sync_f2 <= sync_f1; // Tín hiệu đã đồng bộ với PCLK
            sync_f3 <= sync_f2; // Lưu lại trạng thái cũ để bắt cạnh
        end
    end

    // --- Logic tạo ngắt đơn giản (Bắt cạnh thay đổi) ---
    wire [31:0] changed_bits = sync_f2 ^ sync_f3;
    assign gpio_irq = |(reg_int_stat & reg_int_mask);

    // --- Giao tiếp Đọc/Ghi APB ---
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            gpio_out     <= 32'd0;
            gpio_dir     <= 32'd0;
            reg_int_mask <= 32'd0;
            reg_int_stat <= 32'd0;
            pready       <= 1'b0;
            prdata       <= 32'd0;
            pslverr      <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;

            // Tự động bắt ngắt khi có bit Input thay đổi
            reg_int_stat <= reg_int_stat | (changed_bits & ~gpio_dir);

            if (psel && penable) begin
                if (pwrite) begin
                    // Lệnh Ghi (Write)
                    case (paddr[11:0])
                        12'h004: gpio_out     <= pwdata;
                        12'h008: gpio_dir     <= pwdata;
                        12'h00C: reg_int_mask <= pwdata;
                        12'h010: reg_int_stat <= reg_int_stat & ~pwdata; // Clear ngắt bằng cách ghi 1
                        default: pslverr      <= 1'b1;
                    endcase
                end else begin
                    // Lệnh Đọc (Read)
                    case (paddr[11:0])
                        12'h000: prdata <= sync_f2;     // Đọc data từ chân vào
                        12'h004: prdata <= gpio_out;    // Đọc lại giá trị đang xuất
                        12'h008: prdata <= gpio_dir;    // Kiểm tra hướng
                        12'h00C: prdata <= reg_int_mask;
                        12'h010: prdata <= reg_int_stat;
                        default: prdata <= 32'd0;
                    endcase
                end
            end
        end
    end

endmodule