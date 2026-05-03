`timescale 1ns / 1ps

module apb_syscon #(
    parameter ADDR_WIDTH = 16,
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

    // Tín hiệu quản lý Reset Vector
    output reg  [15:0]            o_reset_vector,

    // Tín hiệu quản lý Năng lượng (Clock Gating)
    input  wire                   i_wfi_sleep,    // Từ CPU báo muốn ngủ
    input  wire                   i_ext_irq,      // Từ PLIC báo có ngắt (Wakeup)
    
    output wire                   o_cpu_clk_en,   // Kích hoạt Clock CPU
    output wire                   o_dbg_clk_en,   // Kích hoạt Clock Debug/JTAG
    output wire                   o_tmr_clk_en,   // Kích hoạt Clock Timer
    output wire                   o_urt_clk_en,   // Kích hoạt Clock UART
    output wire                   o_spi_clk_en,   // Kích hoạt Clock SPI
    output wire                   o_i2c_clk_en,   // Kích hoạt Clock I2C
    output wire                   o_gpo_clk_en,   // Kích hoạt Clock GPIO
    output wire                   o_acc_clk_en    // Kích hoạt Clock Accelerator
);

    // =========================================
    // REGISTER MAP
    // 0x000: RESET_VECTOR (Địa chỉ Boot mặc định)
    // 0x004: CLK_GATE_CTRL (1: Enable Clock, 0: Disable Clock)
    //        [0] Timer, [1] UART, [2] SPI, [3] I2C, 
    //        [4] GPIO, [5] Accel, [6] Debug Module
    // =========================================

    reg [6:0] clk_gate_reg;

    // Logic Quản lý Sleep/Wakeup CPU
    reg cpu_sleep_state;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            cpu_sleep_state <= 1'b0; // Mặc định thức
        end else begin
            // Nếu có ngắt -> Lập tức thức dậy
            if (i_ext_irq) begin
                cpu_sleep_state <= 1'b0;
            end 
            // Nếu nhận lệnh WFI -> Đi ngủ
            else if (i_wfi_sleep) begin
                cpu_sleep_state <= 1'b1;
            end
        end
    end

    // CPU Clock Enable: Luôn cấp clock trừ khi đang trong state Sleep
    assign o_cpu_clk_en = ~cpu_sleep_state;

    // Xuất tín hiệu Enable cho các ngoại vi từ thanh ghi điều khiển
    assign o_tmr_clk_en = clk_gate_reg[0];
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
            o_reset_vector <= 16'h1000;
            // Mặc định lúc Boot: Bật Clock cho Debug, Timer, UART. Các khối khác tắt để tiết kiệm điện.
            clk_gate_reg   <= 7'b1000011; 
            pready         <= 1'b0;
            prdata         <= 32'b0;
            pslverr        <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;
            
            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: o_reset_vector <= pwdata[15:0];
                    12'h004: clk_gate_reg   <= pwdata[6:0];
                    default: pslverr <= 1'b1;
                endcase
            end
            
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= {16'b0, o_reset_vector};
                    12'h004: prdata <= {25'b0, clk_gate_reg};
                    default: begin prdata <= 32'h0; pslverr <= 1'b1; end
                endcase
            end
        end
    end
endmodule