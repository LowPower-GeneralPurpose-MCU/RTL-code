`timescale 1ns / 1ps

module apb_spi #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter FIFO_DEPTH = 16
)(
    // --- Miền xung nhịp APB (pclk - 100MHz) ---
    input  wire                   pclk,
    input  wire                   presetn,
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [ADDR_WIDTH-1:0]  paddr,
    input  wire [DATA_WIDTH-1:0]  pwdata,
    input  wire [3:0]             pstrb,
    output reg                    pready, 
    output reg  [DATA_WIDTH-1:0]  prdata,
    output reg                    pslverr,

    // --- Miền xung nhịp Core (spi_clk) ---
    input  wire                   spi_clk,
    input  wire                   spi_rst_n,
    
    // Giao diện vật lý SPI
    output wire                   sclk,
    output wire                   mosi,
    input  wire                   miso,
    output reg                    cs_n,
    
    // Tín hiệu Ngắt và DMA
    output wire                   spi_irq,
    output wire                   dma_tx_req,
    output wire                   dma_rx_req
);
    localparam IDLE = 0, PHASE1 = 1, PHASE2 = 2;
    reg [2:0]  state;

    // --- Miền APB ---
    reg [31:0] reg_div;
    reg [1:0]  reg_ctrl;
    reg [3:0]  reg_dma_int;
    reg        spi_done_flag;

    wire tx_full, rx_empty;
    wire [7:0] rx_data_apb;
    reg  tx_wr_apb, rx_rd_apb;

    // --- Miền SPI Core ---
    wire tx_empty, rx_full;
    wire [7:0] tx_data_core;
    reg  tx_rd_core, rx_wr_core;
    reg [7:0]  spi_tx_shift;
    reg [7:0]  spi_rx_shift;
    reg        core_done;

    wire core_busy_sync, core_done_sync;
    reg  core_done_sync_dly; // Dùng để bắt cạnh lên của core_done_sync
    
    wire core_busy = (state != IDLE) || (!tx_empty);

    // ============================================================
    // 1. APB INTERFACE (Miền pclk)
    // ============================================================
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl <= 0; reg_div <= 2; cs_n <= 1; reg_dma_int <= 0;
            pready <= 0; prdata <= 0; pslverr <= 0; spi_done_flag <= 0;
            tx_wr_apb <= 0; rx_rd_apb <= 0;
        end else begin
            pready <= psel && penable;
            pslverr <= 0;
            tx_wr_apb <= 0; rx_rd_apb <= 0;

            if (psel && penable && pwrite) begin
                case (paddr[11:0])
                    12'h000: reg_ctrl    <= pwdata[1:0];
                    12'h004: cs_n        <= pwdata[0];
                    12'h008: reg_div     <= pwdata;
                    12'h00C: if (!tx_full) tx_wr_apb <= 1;
                    12'h014: if (pwdata[1]) spi_done_flag <= 0; // W1C Clear Done flag
                    12'h018: reg_dma_int <= pwdata[3:0];
                    default: ;
                endcase
            end
            
            if (psel && !penable && !pwrite) begin
                case (paddr[11:0])
                    12'h000: prdata <= {30'b0, reg_ctrl};
                    12'h004: prdata <= {31'b0, cs_n};
                    12'h008: prdata <= reg_div;
                    12'h010: begin 
                        prdata <= {24'b0, rx_data_apb}; 
                        if (!rx_empty) rx_rd_apb <= 1; 
                    end
                    12'h014: prdata <= {28'b0, rx_empty, tx_full, spi_done_flag, core_busy_sync};
                    12'h018: prdata <= {28'b0, reg_dma_int};
                    default: prdata <= 32'b0;
                endcase
            end

            // Cập nhật ngắt khi SPI Core báo xong (chỉ duy trì 1 nhịp từ sync)
            if (core_done_sync && !core_done_sync_dly) spi_done_flag <= 1'b1;
        end
    end

    // ============================================================
    // 2. CDC & FIFOs (Truyền giữa pclk và spi_clk)
    // ============================================================
    
    // Đưa Config tĩnh (CTRL, DIV) sang miền Core
    reg [31:0] reg_div_core;
    reg [1:0]  reg_ctrl_core;
    always @(posedge spi_clk or negedge spi_rst_n) begin
        if (!spi_rst_n) begin
            reg_div_core <= 32'd2;
            reg_ctrl_core <= 2'b00;
        end else begin
            reg_div_core <= reg_div;
            reg_ctrl_core <= reg_ctrl;
        end
    end

    // Đồng bộ tín hiệu Trạng thái
    cdc_sync_bit u_sync_busy (.clk_dst(pclk), .rst_dst_n(presetn), .d_in(core_busy), .q_out(core_busy_sync));
    cdc_sync_bit u_sync_done (.clk_dst(pclk), .rst_dst_n(presetn), .d_in(core_done), .q_out(core_done_sync));
    
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) core_done_sync_dly <= 1'b0;
        else          core_done_sync_dly <= core_done_sync;
    end

    assign spi_irq = (spi_done_flag & reg_dma_int[0]) | (!rx_empty & reg_dma_int[1]);
    assign dma_tx_req = (!tx_full)  & reg_dma_int[2];
    assign dma_rx_req = (!rx_empty) & reg_dma_int[3];

    // Async FIFOs
    async_fifo #(.DATA_WIDTH(8), .FIFO_DEPTH(16)) u_spi_tx_fifo (
        .clk_wr_domain(pclk),       .clk_rd_domain(spi_clk),
        .data_i(pwdata[7:0]),       .data_o(tx_data_core),
        .wr_valid_i(tx_wr_apb),     .rd_valid_i(tx_rd_core),
        .full_o(tx_full),           .empty_o(tx_empty),
        .rst_n(presetn),
        .wr_ready_o(), .rd_ready_o(), .almost_empty_o(), .almost_full_o()
    );

    async_fifo #(.DATA_WIDTH(8), .FIFO_DEPTH(16)) u_spi_rx_fifo (
        .clk_wr_domain(spi_clk),    .clk_rd_domain(pclk),
        .data_i(spi_rx_shift),      .data_o(rx_data_apb),
        .wr_valid_i(rx_wr_core),    .rd_valid_i(rx_rd_apb),
        .full_o(rx_full),           .empty_o(rx_empty),
        .rst_n(presetn),
        .wr_ready_o(), .rd_ready_o(), .almost_empty_o(), .almost_full_o()
    );

    // ============================================================
    // 3. SPI CORE FSM (Miền spi_clk)
    // ============================================================
    
    // Đồng bộ tín hiệu MISO từ ngoài vào (Chống Metastability)
    reg miso_sync1, miso_sync2;
    always @(posedge spi_clk or negedge spi_rst_n) begin
        if (!spi_rst_n) {miso_sync2, miso_sync1} <= 2'b00;
        else            {miso_sync2, miso_sync1} <= {miso_sync1, miso};
    end

    reg [31:0] clk_cnt;
    reg [2:0]  bit_cnt;
    reg        sclk_reg;
    
    assign sclk = (state == IDLE) ? reg_ctrl_core[1] : sclk_reg;
    assign mosi = spi_tx_shift[7];

    always @(posedge spi_clk or negedge spi_rst_n) begin
        if (!spi_rst_n) begin
            state <= IDLE; clk_cnt <= 0; bit_cnt <= 0;
            sclk_reg <= 0; spi_tx_shift <= 0; spi_rx_shift <= 0;
            tx_rd_core <= 0; rx_wr_core <= 0; core_done <= 0;
        end else begin
            tx_rd_core <= 0;
            rx_wr_core <= 0;
            core_done <= 0;

            case (state)
                IDLE: begin
                    sclk_reg <= reg_ctrl_core[1]; // CPOL
                    if (!tx_empty && !tx_rd_core) begin
                        spi_tx_shift <= tx_data_core;
                        tx_rd_core   <= 1;
                        clk_cnt      <= 0;
                        bit_cnt      <= 0;
                        state        <= PHASE1;
                    end
                end
                
                PHASE1: begin 
                    if (clk_cnt == reg_div_core) begin
                        clk_cnt <= 0;
                        sclk_reg <= ~sclk_reg;
                        state <= PHASE2;
                        if (reg_ctrl_core[0] == 0) spi_rx_shift <= {spi_rx_shift[6:0], miso_sync2}; // CPHA=0
                    end else clk_cnt <= clk_cnt + 1;
                end
                
                PHASE2: begin 
                    if (clk_cnt == reg_div_core) begin
                        clk_cnt <= 0;
                        sclk_reg <= ~sclk_reg;
                        if (reg_ctrl_core[0] == 1) spi_rx_shift <= {spi_rx_shift[6:0], miso_sync2}; // CPHA=1
                        
                        if (bit_cnt == 7) begin
                            rx_wr_core <= 1;
                            if (tx_empty) core_done <= 1; // Chỉ báo Done khi TX rỗng
                            state <= IDLE;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                            spi_tx_shift <= {spi_tx_shift[6:0], 1'b0};
                            state <= PHASE1;
                        end
                    end else clk_cnt <= clk_cnt + 1;
                end
            endcase
        end
    end

endmodule