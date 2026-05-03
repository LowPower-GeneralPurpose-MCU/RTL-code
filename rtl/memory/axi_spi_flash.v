`timescale 1ns / 1ps

module axi_spi_flash #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst_n,

    // --- Kênh Ghi (Bị từ chối - Slave Error) ---
    input  wire [ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire                  s_axi_awvalid,
    output reg                   s_axi_awready,
    input  wire [DATA_WIDTH-1:0] s_axi_wdata,
    input  wire [3:0]            s_axi_wstrb,
    input  wire                  s_axi_wvalid,
    output reg                   s_axi_wready,
    output reg  [1:0]            s_axi_bresp,
    output reg                   s_axi_bvalid,
    input  wire                  s_axi_bready,

    // --- Kênh Đọc (Chức năng chính) ---
    input  wire [ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire                  s_axi_arvalid,
    output reg                   s_axi_arready,
    output reg  [DATA_WIDTH-1:0] s_axi_rdata,
    output reg  [1:0]            s_axi_rresp,
    output reg                   s_axi_rvalid,
    input  wire                  s_axi_rready,

    // --- SPI Physical Interface ---
    output reg                   spi_cs_n,
    output reg                   spi_sck,
    output wire                  spi_mosi,
    input  wire                  spi_miso
);

    localparam IDLE  = 2'd0;
    localparam SETUP = 2'd1;
    localparam SHIFT = 2'd2;
    localparam DONE  = 2'd3;

    reg [1:0]  state;
    reg [63:0] shift_reg; 
    reg [6:0]  bit_cnt;
    reg        sck_en;

    assign spi_mosi = shift_reg[bit_cnt];

    // --- Logic Kênh Ghi (Báo lỗi SLVERR) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0;
            s_axi_bvalid  <= 1'b0; s_axi_bresp  <= 2'b00;
        end else begin
            if (s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                s_axi_bvalid  <= 1'b1;
                s_axi_bresp   <= 2'b10; // SLVERR: Flash là Read-Only qua kênh này
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                s_axi_bvalid  <= 1'b0;
            end
        end
    end

    // --- Logic Kênh Đọc (SPI FSM) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; s_axi_arready <= 1'b1; s_axi_rvalid <= 1'b0;
            spi_cs_n <= 1'b1; spi_sck <= 1'b0; sck_en <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    spi_cs_n <= 1'b1; spi_sck <= 1'b0;
                    if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
                    if (s_axi_arvalid && s_axi_arready) begin
                        s_axi_arready <= 1'b0;
                        shift_reg <= {8'h03, s_axi_araddr[23:0], 32'h0};
                        bit_cnt   <= 7'd63;
                        state     <= SETUP;
                    end
                end
                SETUP: begin
                    spi_cs_n <= 1'b0;
                    sck_en   <= 1'b1; // SỬA Ở ĐÂY: Khởi tạo bằng 1 để nhịp tiếp theo nhảy thẳng vào Cạnh Lên
                    state    <= SHIFT;
                end
                
                SHIFT: begin
                    if (!sck_en) begin // CẠNH XUỐNG (Falling Edge)
                        spi_sck <= 1'b0;
                        sck_en  <= 1'b1;
                        
                        // SỬA Ở ĐÂY: Trừ bit_cnt ở cạnh xuống để dữ liệu (mosi) thay đổi an toàn
                        bit_cnt <= bit_cnt - 1; 
                        
                    end else begin     // CẠNH LÊN (Rising Edge)
                        spi_sck <= 1'b1;
                        sck_en  <= 1'b0;
                        
                        // Đọc tín hiệu miso từ Flash vào nửa sau
                        if (bit_cnt < 32) shift_reg[bit_cnt] <= spi_miso;
                        
                        if (bit_cnt == 0) state <= DONE;
                        // (Tuyệt đối KHÔNG có lệnh trừ bit_cnt ở đây nữa)
                    end
                end
                DONE: begin
                    spi_cs_n <= 1'b1; spi_sck <= 1'b0;
                    s_axi_rdata <= shift_reg[31:0];
                    s_axi_rvalid <= 1'b1;
                    s_axi_rresp <= 2'b00;
                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid <= 1'b0; s_axi_arready <= 1'b1;
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule