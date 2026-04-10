`timescale 1ns / 1ps

// =============================================================================
// Module: apb_uart (Top-level Wrapper)
// Mục đích: Giao tiếp APB Bus, quản lý CDC bằng asyn_fifo, tạo DMA/IRQ requests
// =============================================================================
module apb_uart #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter FIFO_DEPTH = 16
)(
    // --- APB Interface (pclk domain) ---
    input  wire pclk,
    input  wire presetn,
    input  wire psel,
    input  wire penable,
    input  wire pwrite,
    input  wire [ADDR_WIDTH-1:0] paddr,
    input  wire [DATA_WIDTH-1:0] pwdata,
    output reg  [DATA_WIDTH-1:0] prdata,
    output reg  pready,
    output reg  pslverr,

    // --- UART Core Interface (uart_clk domain) ---
    input  wire uart_clk,
    input  wire uart_rst_n,
    input  wire rxd,
    output wire txd,

    // --- Interrupt & DMA Signals ---
    output wire uart_irq,
    output wire dma_tx_req,
    output wire dma_rx_req
);

    // ==========================================
    // Register Map
    // 0x00: CLK_DIV    [31:16] RX_DIV, [15:0] TX_DIV
    // 0x04: TX_DATA    (Ghi vào TX FIFO)
    // 0x08: RX_DATA    (Đọc từ RX FIFO)
    // 0x0C: STATUS     [5] RX_BUSY, [4] TX_BUSY, [3] RX_EMPTY, [2] TX_FULL, [1] FRAME_ERR, [0] TIMEOUT
    // 0x10: DMA_INT    [3] RX_DMA_EN, [2] TX_DMA_EN, [1] RX_INT_EN, [0] TX_INT_EN
    // ==========================================

    reg [31:0] reg_clk_div;
    reg [3:0]  reg_dma_int;

    // FIFO Signals
    wire tx_fifo_full, tx_fifo_empty;
    wire rx_fifo_full, rx_fifo_empty;
    wire [7:0] tx_fifo_rdata;
    wire [7:0] rx_fifo_rdata;
    
    reg  tx_fifo_wr, rx_fifo_rd;
    wire tx_fifo_rd, rx_fifo_wr;
    
    // Core Status Signals (từ uart_clk domain)
    wire tx_busy, rx_busy, frame_error, timeout_error;

    wire baud_tx_en, baud_rx_en;
    wire tx_core_ready;
    wire [7:0] rx_fifo_rdata_core;
    
    // Đồng bộ hóa Status từ uart_clk sang pclk (2-FF CDC)
    reg [3:0] status_sync1, status_sync2;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) {status_sync2, status_sync1} <= 8'b0;
        else {status_sync2, status_sync1} <= {status_sync1, {rx_busy, tx_busy, frame_error, timeout_error}};
    end

    // ==========================================
    // 1. APB Read/Write Logic (pclk domain)
    // ==========================================
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_clk_div <= 32'h0010_0002; // Giá trị chia mặc định an toàn
            reg_dma_int <= 4'b0;
            pready <= 1'b0; prdata <= 32'b0; pslverr <= 1'b0;
            tx_fifo_wr <= 1'b0; rx_fifo_rd <= 1'b0;
        end else begin
            pready <= psel && penable;
            pslverr <= 1'b0;
            tx_fifo_wr <= 1'b0;
            rx_fifo_rd <= 1'b0;

            // Xử lý Ghi APB
            if (psel && penable && pwrite) begin
                case (paddr[11:0])
                    12'h000: reg_clk_div <= pwdata;
                    12'h004: if (!tx_fifo_full) tx_fifo_wr <= 1'b1; // Push TX FIFO
                    12'h010: reg_dma_int <= pwdata[3:0];
                    default: ; // Không làm gì
                endcase
            end
            
            // Xử lý Đọc APB
            if (psel && !penable && !pwrite) begin
                case (paddr[11:0])
                    12'h000: prdata <= reg_clk_div;
                    12'h008: begin 
                        prdata <= {24'b0, rx_fifo_rdata};
                        if (!rx_fifo_empty) rx_fifo_rd <= 1'b1; // Pop RX FIFO
                    end
                    12'h00C: prdata <= {26'b0, status_sync2[3:2], rx_fifo_empty, tx_fifo_full, status_sync2[1:0]};
                    12'h010: prdata <= {28'b0, reg_dma_int};
                    default: prdata <= 32'b0;
                endcase
            end
        end
    end

    // ==========================================
    // 2. IRQ & DMA Generation
    // ==========================================
    assign dma_tx_req = (!tx_fifo_full)  & reg_dma_int[2];
    assign dma_rx_req = (!rx_fifo_empty) & reg_dma_int[3];
    assign uart_irq   = (tx_fifo_empty   & reg_dma_int[0]) | (!rx_fifo_empty & reg_dma_int[1]);

    // ==========================================
    // 3. Asynchronous FIFOs cho CDC
    // ==========================================
    // TX FIFO: Ghi ở pclk, Đọc ở uart_clk
    async_fifo #(
        .ASFIFO_TYPE(0), .DATA_WIDTH(8), .FIFO_DEPTH(FIFO_DEPTH), .NUM_SYNC_FF(2)
    ) u_tx_fifo (
        .clk_wr_domain(pclk),       .clk_rd_domain(uart_clk),
        .rst_n(presetn & uart_rst_n),
        .data_i(pwdata[7:0]),       .data_o(tx_fifo_rdata),
        .wr_valid_i(tx_fifo_wr),    .rd_valid_i(tx_fifo_rd),
        .wr_ready_o(),              .rd_ready_o(),
        .full_o(tx_fifo_full),      .empty_o(tx_fifo_empty),
        .almost_empty_o(),          .almost_full_o()
    );

    // RX FIFO: Ghi ở uart_clk, Đọc ở pclk
    async_fifo #(
        .ASFIFO_TYPE(0), .DATA_WIDTH(8), .FIFO_DEPTH(FIFO_DEPTH), .NUM_SYNC_FF(2)
    ) u_rx_fifo (
        .clk_wr_domain(uart_clk),   .clk_rd_domain(pclk),
        .rst_n(presetn & uart_rst_n),
        .data_i(rx_fifo_rdata_core),.data_o(rx_fifo_rdata),
        .wr_valid_i(rx_fifo_wr),    .rd_valid_i(rx_fifo_rd),
        .wr_ready_o(),              .rd_ready_o(),
        .full_o(rx_fifo_full),      .empty_o(rx_fifo_empty),
        .almost_empty_o(),          .almost_full_o()
    );

    // ==========================================
    // 4. UART Protocol Cores (uart_clk domain)
    // ==========================================
    baud_gen u_baud_gen (
        .clk(uart_clk),             .rst_n(uart_rst_n),
        .tx_divisor(reg_clk_div[15:0]), .rx_divisor(reg_clk_div[31:16]),
        .baud_tx_en(baud_tx_en),    .baud_rx_en(baud_rx_en)
    );

    assign tx_fifo_rd = tx_core_ready & !tx_fifo_empty;

    uart_tx u_uart_tx (
        .clk(uart_clk),             .rst_n(uart_rst_n),
        .baudrate_clk_en(baud_tx_en),
        .data_i(tx_fifo_rdata),     .data_valid_i(tx_fifo_rd),
        .ready_o(tx_core_ready),    .tx_busy(tx_busy),
        .TX(txd)
    );

    uart_rx u_uart_rx (
        .clk(uart_clk),             .rst_n(uart_rst_n),
        .baudrate_clk_en(baud_rx_en),
        .RX(rxd),
        .data_out_rx(rx_fifo_rdata_core),
        .fifo_wr(rx_fifo_wr),
        .transaction_en(rx_busy),
        .frame_error(frame_error),  .timeout_error(timeout_error)
    );

endmodule


// =============================================================================
// LỚP VẬT LÝ: BAUD GEN, UART TX, UART RX (Đã gộp và rút gọn)
// =============================================================================

module baud_gen (
    input  wire clk, rst_n,
    input  wire [15:0] tx_divisor, rx_divisor,
    output reg  baud_tx_en, baud_rx_en
);
    reg [15:0] tx_cnt, rx_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin tx_cnt <= 0; baud_tx_en <= 0; end
        else if (tx_cnt >= tx_divisor - 1) begin tx_cnt <= 0; baud_tx_en <= 1; end
        else begin tx_cnt <= tx_cnt + 1; baud_tx_en <= 0; end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_cnt <= 0; baud_rx_en <= 0; end
        else if (rx_cnt >= rx_divisor - 1) begin rx_cnt <= 0; baud_rx_en <= 1; end
        else begin rx_cnt <= rx_cnt + 1; baud_rx_en <= 0; end
    end
endmodule

module uart_tx (
    input  wire clk, rst_n, baudrate_clk_en,
    input  wire [7:0] data_i,
    input  wire data_valid_i,
    output reg  ready_o, tx_busy,
    output reg  TX
);
    reg [2:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_reg;
    localparam IDLE=0, START=1, DATA=2, STOP=3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; TX <= 1'b1; ready_o <= 1'b1; tx_busy <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    TX <= 1'b1; ready_o <= 1'b1; tx_busy <= 1'b0;
                    if (data_valid_i) begin
                        shift_reg <= data_i; state <= START; ready_o <= 1'b0; tx_busy <= 1'b1;
                    end
                end
                START: if (baudrate_clk_en) begin TX <= 1'b0; state <= DATA; bit_cnt <= 0; end
                DATA:  if (baudrate_clk_en) begin
                           TX <= shift_reg[0]; shift_reg <= {1'b0, shift_reg[7:1]};
                           if (bit_cnt == 7) state <= STOP;
                           else bit_cnt <= bit_cnt + 1;
                       end
                STOP:  if (baudrate_clk_en) begin TX <= 1'b1; state <= IDLE; end
            endcase
        end
    end
endmodule

module uart_rx (
    input  wire clk, rst_n, baudrate_clk_en, RX,
    output reg  [7:0] data_out_rx,
    output reg  fifo_wr, transaction_en, frame_error, timeout_error
);
    reg [1:0] state;
    reg [3:0] sample_cnt;
    reg [2:0] bit_cnt;
    reg [7:0] shift_reg;
    localparam IDLE=0, START=1, DATA=2, STOP=3;

    // 2-FF Synchronizer cho RX pin để chống Metastability
    reg rx_s1, rx_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {rx_s2, rx_s1} <= 2'b11; // Kéo lên mức 1
        else        {rx_s2, rx_s1} <= {rx_s1, RX};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; fifo_wr <= 0; transaction_en <= 0;
            frame_error <= 0; timeout_error <= 0;
        end else begin
            fifo_wr <= 1'b0; // Default là tạo xung
            case (state)
                IDLE: begin
                    transaction_en <= 0;
                    if (!rx_s2) begin // Cạnh xuống START bit
                        state <= START; sample_cnt <= 7; transaction_en <= 1; frame_error <= 0;
                    end
                end
                START: if (baudrate_clk_en) begin
                           if (sample_cnt == 0) begin state <= DATA; sample_cnt <= 15; bit_cnt <= 0; end
                           else sample_cnt <= sample_cnt - 1;
                       end
                DATA:  if (baudrate_clk_en) begin
                           if (sample_cnt == 0) begin
                               shift_reg <= {rx_s2, shift_reg[7:1]};
                               sample_cnt <= 15;
                               if (bit_cnt == 7) state <= STOP;
                               else bit_cnt <= bit_cnt + 1;
                           end else sample_cnt <= sample_cnt - 1;
                       end
                STOP:  if (baudrate_clk_en) begin
                           if (sample_cnt == 0) begin
                               if (rx_s2 == 1'b1) begin // Lấy mẫu đúng STOP bit
                                   data_out_rx <= shift_reg;
                                   fifo_wr <= 1'b1; 
                               end else frame_error <= 1'b1;
                               state <= IDLE;
                           end else sample_cnt <= sample_cnt - 1;
                       end
            endcase
        end
    end
endmodule