`timescale 1ns / 1ps

module apb_interconnect #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    // =========================================================================
    // CỔNG TỪ AXI-TO-APB BRIDGE (MASTER)
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  m_paddr,
    input  wire                   m_psel,
    input  wire                   m_penable,
    input  wire                   m_pwrite,
    input  wire [DATA_WIDTH-1:0]  m_pwdata,
    input  wire [3:0]             m_pstrb,
    output reg  [DATA_WIDTH-1:0]  m_prdata,
    output reg                    m_pready,
    output reg                    m_pslverr,

    // =========================================================================
    // SLAVE 0: SYSCON (0x4000_0000)
    // =========================================================================
    output wire                   s0_psel,
    output wire [ADDR_WIDTH-1:0]  s0_paddr,
    output wire                   s0_penable,
    output wire                   s0_pwrite,
    output wire [DATA_WIDTH-1:0]  s0_pwdata,
    output wire [3:0]             s0_pstrb,
    input  wire [DATA_WIDTH-1:0]  s0_prdata,
    input  wire                   s0_pready,
    input  wire                   s0_pslverr,

    // =========================================================================
    // SLAVE 1: WATCHDOG (0x4000_1000)
    // =========================================================================
    output wire                   s1_psel,
    output wire [ADDR_WIDTH-1:0]  s1_paddr,
    output wire                   s1_penable,
    output wire                   s1_pwrite,
    output wire [DATA_WIDTH-1:0]  s1_pwdata,
    output wire [3:0]             s1_pstrb,
    input  wire [DATA_WIDTH-1:0]  s1_prdata,
    input  wire                   s1_pready,
    input  wire                   s1_pslverr,

    // =========================================================================
    // SLAVE 2: TIMER (0x4000_2000)
    // =========================================================================
    output wire                   s2_psel,
    output wire [ADDR_WIDTH-1:0]  s2_paddr,
    output wire                   s2_penable,
    output wire                   s2_pwrite,
    output wire [DATA_WIDTH-1:0]  s2_pwdata,
    output wire [3:0]             s2_pstrb,
    input  wire [DATA_WIDTH-1:0]  s2_prdata,
    input  wire                   s2_pready,
    input  wire                   s2_pslverr,

    // =========================================================================
    // SLAVE 3: UART (0x4000_3000)
    // =========================================================================
    output wire                   s3_psel,
    output wire [ADDR_WIDTH-1:0]  s3_paddr,
    output wire                   s3_penable,
    output wire                   s3_pwrite,
    output wire [DATA_WIDTH-1:0]  s3_pwdata,
    output wire [3:0]             s3_pstrb,
    input  wire [DATA_WIDTH-1:0]  s3_prdata,
    input  wire                   s3_pready,
    input  wire                   s3_pslverr,

    // =========================================================================
    // SLAVE 4: SPI (0x4000_4000)
    // =========================================================================
    output wire                   s4_psel,
    output wire [ADDR_WIDTH-1:0]  s4_paddr,
    output wire                   s4_penable,
    output wire                   s4_pwrite,
    output wire [DATA_WIDTH-1:0]  s4_pwdata,
    output wire [3:0]             s4_pstrb,
    input  wire [DATA_WIDTH-1:0]  s4_prdata,
    input  wire                   s4_pready,
    input  wire                   s4_pslverr,

    // =========================================================================
    // SLAVE 5: I2C (0x4000_5000)
    // =========================================================================
    output wire                   s5_psel,
    output wire [ADDR_WIDTH-1:0]  s5_paddr,
    output wire                   s5_penable,
    output wire                   s5_pwrite,
    output wire [DATA_WIDTH-1:0]  s5_pwdata,
    output wire [3:0]             s5_pstrb,
    input  wire [DATA_WIDTH-1:0]  s5_prdata,
    input  wire                   s5_pready,
    input  wire                   s5_pslverr,

    // =========================================================================
    // SLAVE 6: GPIO (0x4000_6000)
    // =========================================================================
    output wire                   s6_psel,
    output wire [ADDR_WIDTH-1:0]  s6_paddr,
    output wire                   s6_penable,
    output wire                   s6_pwrite,
    output wire [DATA_WIDTH-1:0]  s6_pwdata,
    output wire [3:0]             s6_pstrb,
    input  wire [DATA_WIDTH-1:0]  s6_prdata,
    input  wire                   s6_pready,
    input  wire                   s6_pslverr,

    // =========================================================================
    // SLAVE 7: ACCELERATOR (0x4000_7000)
    // =========================================================================
    output wire                   s7_psel,
    output wire [ADDR_WIDTH-1:0]  s7_paddr,
    output wire                   s7_penable,
    output wire                   s7_pwrite,
    output wire [DATA_WIDTH-1:0]  s7_pwdata,
    output wire [3:0]             s7_pstrb,
    input  wire [DATA_WIDTH-1:0]  s7_prdata,
    input  wire                   s7_pready,
    input  wire                   s7_pslverr
);

    // Gán các tín hiệu chung (Broadcast)
    assign s0_paddr = m_paddr; assign s0_penable = m_penable; assign s0_pwrite = m_pwrite; assign s0_pwdata = m_pwdata; assign s0_pstrb = m_pstrb;
    assign s1_paddr = m_paddr; assign s1_penable = m_penable; assign s1_pwrite = m_pwrite; assign s1_pwdata = m_pwdata; assign s1_pstrb = m_pstrb;
    assign s2_paddr = m_paddr; assign s2_penable = m_penable; assign s2_pwrite = m_pwrite; assign s2_pwdata = m_pwdata; assign s2_pstrb = m_pstrb;
    assign s3_paddr = m_paddr; assign s3_penable = m_penable; assign s3_pwrite = m_pwrite; assign s3_pwdata = m_pwdata; assign s3_pstrb = m_pstrb;
    assign s4_paddr = m_paddr; assign s4_penable = m_penable; assign s4_pwrite = m_pwrite; assign s4_pwdata = m_pwdata; assign s4_pstrb = m_pstrb;
    assign s5_paddr = m_paddr; assign s5_penable = m_penable; assign s5_pwrite = m_pwrite; assign s5_pwdata = m_pwdata; assign s5_pstrb = m_pstrb;
    assign s6_paddr = m_paddr; assign s6_penable = m_penable; assign s6_pwrite = m_pwrite; assign s6_pwdata = m_pwdata; assign s6_pstrb = m_pstrb;
    assign s7_paddr = m_paddr; assign s7_penable = m_penable; assign s7_pwrite = m_pwrite; assign s7_pwdata = m_pwdata; assign s7_pstrb = m_pstrb;

    // Kiểm tra Base Address (0x4000_0000 đến 0x4000_FFFF)
    wire valid_base = (m_paddr[31:16] == 16'h4000);

    // Giải mã địa chỉ bằng bit [15:12] VÀ phải đúng Base Address
    wire dec_s0 = valid_base && (m_paddr[15:12] == 4'h0);
    wire dec_s1 = valid_base && (m_paddr[15:12] == 4'h1);
    wire dec_s2 = valid_base && (m_paddr[15:12] == 4'h2);
    wire dec_s3 = valid_base && (m_paddr[15:12] == 4'h3);
    wire dec_s4 = valid_base && (m_paddr[15:12] == 4'h4);
    wire dec_s5 = valid_base && (m_paddr[15:12] == 4'h5);
    wire dec_s6 = valid_base && (m_paddr[15:12] == 4'h6);
    wire dec_s7 = valid_base && (m_paddr[15:12] == 4'h7);

    // Kích hoạt PSEL tương ứng
    assign s0_psel = m_psel && dec_s0;
    assign s1_psel = m_psel && dec_s1;
    assign s2_psel = m_psel && dec_s2;
    assign s3_psel = m_psel && dec_s3;
    assign s4_psel = m_psel && dec_s4;
    assign s5_psel = m_psel && dec_s5;
    assign s6_psel = m_psel && dec_s6;
    assign s7_psel = m_psel && dec_s7;

    // Bộ dồn kênh (Multiplexer) phản hồi về Master
    always @(*) begin
        if      (dec_s0) begin m_prdata = s0_prdata; m_pready = s0_pready; m_pslverr = s0_pslverr; end
        else if (dec_s1) begin m_prdata = s1_prdata; m_pready = s1_pready; m_pslverr = s1_pslverr; end
        else if (dec_s2) begin m_prdata = s2_prdata; m_pready = s2_pready; m_pslverr = s2_pslverr; end
        else if (dec_s3) begin m_prdata = s3_prdata; m_pready = s3_pready; m_pslverr = s3_pslverr; end
        else if (dec_s4) begin m_prdata = s4_prdata; m_pready = s4_pready; m_pslverr = s4_pslverr; end
        else if (dec_s5) begin m_prdata = s5_prdata; m_pready = s5_pready; m_pslverr = s5_pslverr; end
        else if (dec_s6) begin m_prdata = s6_prdata; m_pready = s6_pready; m_pslverr = s6_pslverr; end
        else if (dec_s7) begin m_prdata = s7_prdata; m_pready = s7_pready; m_pslverr = s7_pslverr; end
        else begin
            m_prdata  = 32'hDEADBEEF;
            m_pready  = 1'b1;
            m_pslverr = m_psel; // Cố tình truy cập địa chỉ không tồn tại -> Báo lỗi
        end
    end

endmodule