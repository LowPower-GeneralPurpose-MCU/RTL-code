`timescale 1ns / 1ps

module axi_spi_flash #(
    parameter ADDR_WIDTH          = 32,
    parameter DATA_WIDTH          = 32,
    parameter ID_WIDTH            = 7,
    parameter ADDR_MASK           = 32'h00FF_FFFF, // Thêm lại Mask
    parameter SPI_CLK_DIVIDER     = 2,       
    parameter FLASH_FAST_READ_CMD = 8'h0B,   
    parameter FLASH_DUMMY_CYCLES  = 8        
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // KÊNH GHI (BỊ CHẶN)
    input  wire [ID_WIDTH-1:0]      s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire [7:0]               s_axi_awlen,
    input  wire [2:0]               s_axi_awsize,
    input  wire [1:0]               s_axi_awburst,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]s_axi_wstrb,
    input  wire                     s_axi_wlast,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,
    output reg  [ID_WIDTH-1:0]      s_axi_bid,
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    // KÊNH ĐỌC (XIP BURST)
    input  wire [ID_WIDTH-1:0]      s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire [7:0]               s_axi_arlen,
    input  wire [2:0]               s_axi_arsize,
    input  wire [1:0]               s_axi_arburst,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,
    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output reg  [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready,

    // GIAO DIỆN VẬT LÝ SPI
    output reg                      spi_cs_n_o,
    output reg                      spi_clk_o,
    output reg                      spi_io0_o,
    input  wire                     spi_io0_i,
    output reg                      spi_io0_oe,
    output reg                      spi_io1_o,
    input  wire                     spi_io1_i,
    output reg                      spi_io1_oe,
    output reg                      spi_io2_o,
    input  wire                     spi_io2_i,
    output reg                      spi_io2_oe,
    output reg                      spi_io3_o,
    input  wire                     spi_io3_i,
    output reg                      spi_io3_oe
);

    // ==========================================
    // LOGIC CHẶN GHI (SLVERR)
    // ==========================================
    localparam W_IDLE = 2'd0, W_SINK = 2'd1, W_RESP = 2'd2;
    reg [1:0] w_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state       <= W_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bid     <= {ID_WIDTH{1'b0}};
            s_axi_bresp   <= 2'b00;
        end else begin
            case (w_state)
                W_IDLE: begin
                    s_axi_awready <= 1'b1;
                    if (s_axi_awvalid && s_axi_awready) begin
                        s_axi_bid     <= s_axi_awid;
                        s_axi_awready <= 1'b0;
                        s_axi_wready  <= 1'b1;
                        w_state       <= W_SINK;
                    end
                end
                W_SINK: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        if (s_axi_wlast) begin
                            s_axi_wready <= 1'b0;
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= 2'b10; // Báo lỗi
                            w_state      <= W_RESP;
                        end
                    end
                end
                W_RESP: begin
                    if (s_axi_bready && s_axi_bvalid) begin
                        s_axi_bvalid <= 1'b0;
                        w_state      <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // ==========================================
    // SPI CLOCK GENERATOR
    // ==========================================
    reg [7:0] clk_counter;
    reg       spi_clk_enable;
    reg       spi_clk_rising_edge;
    reg       spi_clk_falling_edge;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_counter          <= 8'd0;
            spi_clk_o            <= 1'b0;
            spi_clk_rising_edge  <= 1'b0;
            spi_clk_falling_edge <= 1'b0;
        end else begin
            spi_clk_rising_edge  <= 1'b0;
            spi_clk_falling_edge <= 1'b0;

            if (spi_clk_enable) begin
                if (clk_counter == (SPI_CLK_DIVIDER - 1)) begin
                    clk_counter <= 8'd0;
                    spi_clk_o   <= ~spi_clk_o;
                    if (~spi_clk_o) spi_clk_rising_edge  <= 1'b1;
                    else            spi_clk_falling_edge <= 1'b1;
                end else begin
                    clk_counter <= clk_counter + 1'b1;
                end
            end else begin
                clk_counter <= 8'd0;
                spi_clk_o   <= 1'b0;
            end
        end
    end

    // ==========================================
    // XIP ĐỌC FSM 
    // ==========================================
    localparam R_IDLE         = 3'd0;
    localparam R_SEND_CMD     = 3'd1;
    localparam R_SEND_DUMMY   = 3'd2;
    localparam R_READ_DATA    = 3'd3;
    localparam R_AXI_RESPONSE = 3'd4;

    reg [2:0]  r_state;
    reg [31:0] shift_out_reg;
    reg [31:0] shift_in_reg;
    reg [7:0]  bit_counter;
    reg [7:0]  burst_length_reg;
    reg [7:0]  burst_counter;
    
    wire [ADDR_WIDTH-1:0] masked_araddr = s_axi_araddr & ADDR_MASK;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state          <= R_IDLE;
            spi_clk_enable   <= 1'b0;
            spi_cs_n_o       <= 1'b1;
            spi_io0_o        <= 1'b0; spi_io0_oe <= 1'b0; 
            spi_io1_o        <= 1'b0; spi_io1_oe <= 1'b0; 
            spi_io2_o        <= 1'b1; spi_io2_oe <= 1'b1; 
            spi_io3_o        <= 1'b1; spi_io3_oe <= 1'b1; 
            
            s_axi_arready    <= 1'b0;
            s_axi_rvalid     <= 1'b0;
            s_axi_rlast      <= 1'b0;
            s_axi_rresp      <= 2'b00;
            s_axi_rid        <= {ID_WIDTH{1'b0}};
            s_axi_rdata      <= {DATA_WIDTH{1'b0}};
            
            shift_out_reg    <= 32'd0;
            shift_in_reg     <= 32'd0;
            bit_counter      <= 8'd0;
            burst_length_reg <= 8'd0;
            burst_counter    <= 8'd0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    spi_cs_n_o     <= 1'b1;
                    spi_clk_enable <= 1'b0;
                    spi_io0_oe     <= 1'b0;
                    
                    s_axi_arready  <= 1'b1;
                    if (s_axi_arvalid && s_axi_arready) begin
                        s_axi_arready    <= 1'b0;
                        s_axi_rid        <= s_axi_arid;
                        burst_length_reg <= s_axi_arlen;
                        burst_counter    <= 8'd0;
                        
                        shift_out_reg  <= {FLASH_FAST_READ_CMD, masked_araddr[23:0]};
                        spi_io0_o      <= FLASH_FAST_READ_CMD[7]; 
                        
                        spi_cs_n_o     <= 1'b0; 
                        spi_clk_enable <= 1'b1;
                        spi_io0_oe     <= 1'b1;
                        bit_counter    <= 8'd31; 
                        r_state        <= R_SEND_CMD;
                    end
                end

                R_SEND_CMD: begin
                    if (spi_clk_falling_edge) begin
                        spi_io0_o <= shift_out_reg[30];
                        shift_out_reg <= {shift_out_reg[29:0], 1'b0};
                        bit_counter <= bit_counter - 1'b1;
                        
                        if (bit_counter == 8'd1) begin
                            // SỬA LỖI TẠI ĐÂY: Phải đếm 9 sườn xuống thì mới tương đương 8 chu kỳ đồng hồ
                            bit_counter <= FLASH_DUMMY_CYCLES + 8'd1; 
                            r_state     <= R_SEND_DUMMY;
                        end
                    end
                end

                R_SEND_DUMMY: begin
                    if (spi_clk_falling_edge) begin
                        spi_io0_o <= 1'b0;
                        bit_counter <= bit_counter - 1'b1;
                        
                        if (bit_counter == 8'd1) begin
                            spi_io0_oe  <= 1'b0; // Tắt MOSI ngay khi Slave chuẩn bị đẩy MISO
                            bit_counter <= 8'd32; 
                            r_state     <= R_READ_DATA;
                        end
                    end
                end

                R_READ_DATA: begin
                    if (spi_clk_rising_edge) begin
                        shift_in_reg <= {shift_in_reg[30:0], spi_io1_i};
                        bit_counter  <= bit_counter - 1'b1;
                        
                        if (bit_counter == 8'd1) begin
                            r_state <= R_AXI_RESPONSE;
                        end
                    end
                end

                R_AXI_RESPONSE: begin
                    spi_clk_enable <= 1'b0; 
                    
                    s_axi_rvalid <= 1'b1;
                    // Lắp ráp byte Little-Endian
                    s_axi_rdata  <= {shift_in_reg[7:0], shift_in_reg[15:8], shift_in_reg[23:16], shift_in_reg[31:24]};
                    s_axi_rresp  <= 2'b00; 
                    s_axi_rlast  <= (burst_counter == burst_length_reg);

                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        s_axi_rlast  <= 1'b0;
                        
                        if (burst_counter == burst_length_reg) begin
                            spi_cs_n_o <= 1'b1;
                            r_state    <= R_IDLE;
                        end else begin
                            burst_counter  <= burst_counter + 1'b1;
                            bit_counter    <= 8'd32;
                            spi_clk_enable <= 1'b1; 
                            r_state        <= R_READ_DATA;
                        end
                    end
                end
            endcase
        end
    end

endmodule