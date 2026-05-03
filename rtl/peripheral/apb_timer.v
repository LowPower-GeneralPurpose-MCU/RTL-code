`timescale 1ns / 1ps

module apb_timer #(
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
    
    output wire                   timer_irq // Ngắt nối vào CPU/PLIC
);

    // =========================================
    // REGISTER MAP
    // 0x00: CTRL    [0] Enable, [1] IRQ_EN
    // 0x04: LOAD    Giá trị nạp lại (Reload)
    // 0x08: VAL     Giá trị đếm hiện tại (Read-only)
    // 0x0C: INT_CLR Viết 1 để xóa ngắt
    // =========================================

    reg [1:0]  reg_ctrl;
    reg [31:0] reg_load;
    reg [31:0] reg_val;
    reg        irq_status;

    assign timer_irq = irq_status & reg_ctrl[1];
    
    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite; // Đọc ở pha Setup

    integer i;

    // --- Logic Giao tiếp APB & Thanh ghi ---
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl   <= 2'b0;
            reg_load   <= 32'b0;
            irq_status <= 1'b0;
            pready     <= 1'b0;
            prdata     <= 32'b0;
            pslverr    <= 1'b0;
        end else begin
            pready  <= psel && penable; // 0-wait state
            pslverr <= 1'b0;
            
            // Xử lý Ghi
            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: begin // CTRL
                        if (pstrb[0]) reg_ctrl <= pwdata[1:0];
                    end
                    12'h004: begin // LOAD
                        for (i=0; i<4; i=i+1)
                            if (pstrb[i]) reg_load[(i*8)+:8] <= pwdata[(i*8)+:8];
                    end
                    12'h00C: begin // INT_CLR
                        if (pstrb[0] && pwdata[0]) irq_status <= 1'b0; // Xóa ngắt
                    end
                    12'h008: pslverr <= 1'b1; // Cố tình ghi vào Read-only
                    default: pslverr <= 1'b1;
                endcase
            end
            
            // Xử lý Đọc
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= {30'b0, reg_ctrl};
                    12'h004: prdata <= reg_load;
                    12'h008: prdata <= reg_val;
                    12'h00C: prdata <= {31'b0, irq_status};
                    default: begin prdata <= 32'h0; pslverr <= 1'b1; end
                endcase
            end
            
            // Đặt lại ngắt từ bên trong Counter logic
            if (reg_ctrl[0] && reg_val == 32'd1) begin
                irq_status <= 1'b1;
            end
        end
    end

    // --- Logic Counter (Core Timer) ---
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_val <= 32'b0;
        end else begin
            if (reg_ctrl[0]) begin // Nếu Timer được bật
                if (reg_val == 32'd0) begin
                    reg_val <= reg_load; // Reload
                end else begin
                    reg_val <= reg_val - 1'b1; // Đếm lùi
                end
            end else begin
                reg_val <= reg_load; // Nếu tắt, giữ giá trị Load
            end
        end
    end

endmodule