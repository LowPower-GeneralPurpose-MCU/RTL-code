`timescale 1ns / 1ps

module apb_syscon #(
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
    output reg                    pready,
    output reg  [DATA_WIDTH-1:0]  prdata,
    output reg                    pslverr,

    // Tín hiệu quản lý Reset Vector ĐÃ ĐƯỢC NÂNG CẤP LÊN 32-BIT
    output reg  [31:0]            o_reset_vector,

    // Clock Gating & Power Management
    input  wire                   i_wfi_sleep,
    input  wire                   i_ext_irq, 
    
    output wire                   o_cpu_clk_en, 
    output wire                   o_dbg_clk_en,
    output wire                   o_pwm_clk_en, 
    output wire                   o_urt_clk_en, 
    output wire                   o_spi_clk_en,
    output wire                   o_i2c_clk_en,
    output wire                   o_gpo_clk_en,
    output wire                   o_acc_clk_en 
);

    // =========================================
    // REGISTER MAP
    // 0x000: RESET_VECTOR (Giờ đã là 32-bit hoàn chỉnh)
    // 0x004: CLK_GATE_CTRL 
    //        [0] PWM, [1] UART, [2] SPI, [3] I2C, 
    //        [4] GPIO, [5] Accel, [6] Debug Module
    // =========================================

    reg [6:0] clk_gate_reg;
    reg       cpu_sleep_state;

    // Logic Quản lý Sleep/Wakeup CPU
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            cpu_sleep_state <= 1'b0;
        end else begin
            if (i_ext_irq) begin
                cpu_sleep_state <= 1'b0; // Thức dậy khi có ngắt
            end else if (i_wfi_sleep) begin
                cpu_sleep_state <= 1'b1; // Ngủ khi có lệnh wfi
            end
        end
    end

    // Clock cho CPU chỉ tắt khi đang trong trạng thái Sleep
    assign o_cpu_clk_en = ~cpu_sleep_state;
    
    // Gán tín hiệu Clock Gating cho các ngoại vi
    assign o_pwm_clk_en = clk_gate_reg[0];
    assign o_urt_clk_en = clk_gate_reg[1];
    assign o_spi_clk_en = clk_gate_reg[2];
    assign o_i2c_clk_en = clk_gate_reg[3];
    assign o_gpo_clk_en = clk_gate_reg[4];
    assign o_acc_clk_en = clk_gate_reg[5];
    assign o_dbg_clk_en = clk_gate_reg[6];

    // APB Logic
    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            // Mặc định địa chỉ Boot trỏ về Base Address của ROM trong SoC (ví dụ 0x00010000)
            o_reset_vector <= 32'h0001_0000;
            // Mặc định lúc Boot: Bật Clock cho Debug, UART, PWM.
            clk_gate_reg   <= 7'b1000011; 
            pready         <= 1'b0;
            prdata         <= 32'b0;
            pslverr        <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;
            
            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: o_reset_vector <= pwdata; // Lấy toàn bộ 32-bit
                    12'h004: clk_gate_reg   <= pwdata[6:0];
                    default: pslverr <= 1'b1;
                endcase
            end
            
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= o_reset_vector; // Xuất toàn bộ 32-bit
                    12'h004: prdata <= {25'b0, clk_gate_reg};
                    default: begin prdata <= 32'h0; pslverr <= 1'b1; end
                endcase
            end
        end
    end
endmodule