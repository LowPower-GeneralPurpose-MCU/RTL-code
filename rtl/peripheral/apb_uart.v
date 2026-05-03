`timescale 1ns / 1ps

module apb_uart #(
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
    
    // Physical Pins
    input  wire                   rx,
    output wire                   tx,
    output wire                   uart_irq
);

    // --- Register Map ---
    // 0x00: TX_DATA (W)
    // 0x04: RX_DATA (R)
    // 0x08: STATUS  (R) [0] TX_BUSY, [1] RX_VALID
    // 0x0C: DIVISOR (R/W) (System Clock / (Baudrate * 16))

    reg [31:0] reg_div;
    reg [7:0]  tx_reg;
    reg [7:0]  rx_reg;
    reg        tx_start_req;
    reg        rx_valid;
    wire       tx_busy;
    
    assign uart_irq = rx_valid;

    reg [3:0] tx_state;
    reg [3:0] tx_tick_cnt; // Đếm 16 tick
    reg [2:0] tx_bit_cnt;
    reg [7:0] tx_shift_out;
    reg       tx_pin_reg;

    localparam TX_IDLE = 0, TX_START = 1, TX_DATA = 2, TX_STOP = 3;
    
    reg [3:0] rx_state;
    reg [3:0] rx_tick_cnt;
    reg [2:0] rx_bit_cnt;
    reg [7:0] rx_shift_data;
    reg       rx_done_pulse;

    localparam RX_IDLE = 0, RX_START = 1, RX_DATA = 2, RX_STOP = 3;

    // --- CDC cho RX ---
    reg rx_sync1, rx_sync2;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) {rx_sync2, rx_sync1} <= 2'b11; // Line idle high
        else          {rx_sync2, rx_sync1} <= {rx_sync1, rx};
    end

    // ==========================================
    // 1. APB INTERFACE
    // ==========================================
    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_div      <= 32'd54; // Ví dụ mặc định
            tx_start_req <= 1'b0;
            rx_valid     <= 1'b0;
            pready       <= 1'b0;
            prdata       <= 32'b0;
            pslverr      <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;
            tx_start_req <= 1'b0; // Pulse

            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: begin
                        if (!tx_busy) begin 
                            tx_reg <= pwdata[7:0]; 
                            tx_start_req <= 1'b1; 
                        end else pslverr <= 1'b1;
                    end
                    12'h00C: if (pstrb[0]) reg_div <= pwdata;
                    default: pslverr <= 1'b1;
                endcase
            end
            
            if (apb_read) begin
                case (paddr[11:0])
                    12'h004: begin prdata <= {24'b0, rx_reg}; rx_valid <= 1'b0; end // Đọc xong clear cờ
                    12'h008: prdata <= {30'b0, rx_valid, tx_busy};
                    12'h00C: prdata <= reg_div;
                    default: pslverr <= 1'b1;
                endcase
            end
            
            // Nhận tín hiệu báo xong từ RX Core
            if (rx_done_pulse) begin
                rx_reg <= rx_shift_data;
                rx_valid <= 1'b1;
            end
        end
    end

    // ==========================================
    // 2. BAUD RATE GENERATOR (16x Oversampling)
    // ==========================================
    reg [31:0] baud_cnt;
    wire tick_16x = (baud_cnt == 0);

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) baud_cnt <= 0;
        else if (tick_16x) baud_cnt <= reg_div - 1;
        else baud_cnt <= baud_cnt - 1;
    end

    // ==========================================
    // 3. TX CORE FSM
    // ==========================================
    assign tx = tx_pin_reg;
    assign tx_busy = (tx_state != 0);

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            tx_state <= TX_IDLE;
            tx_pin_reg <= 1'b1;
            tx_tick_cnt <= 0;
            tx_bit_cnt <= 0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx_pin_reg <= 1'b1;
                    if (tx_start_req) begin
                        tx_shift_out <= tx_reg;
                        tx_state <= TX_START;
                        tx_tick_cnt <= 0;
                    end
                end
                TX_START: begin
                    tx_pin_reg <= 1'b0;
                    if (tick_16x) begin
                        if (tx_tick_cnt == 15) begin
                            tx_tick_cnt <= 0;
                            tx_bit_cnt <= 0;
                            tx_state <= TX_DATA;
                        end else tx_tick_cnt <= tx_tick_cnt + 1;
                    end
                end
                TX_DATA: begin
                    tx_pin_reg <= tx_shift_out[0];
                    if (tick_16x) begin
                        if (tx_tick_cnt == 15) begin
                            tx_tick_cnt <= 0;
                            tx_shift_out <= {1'b0, tx_shift_out[7:1]};
                            if (tx_bit_cnt == 7) tx_state <= TX_STOP;
                            else tx_bit_cnt <= tx_bit_cnt + 1;
                        end else tx_tick_cnt <= tx_tick_cnt + 1;
                    end
                end
                TX_STOP: begin
                    tx_pin_reg <= 1'b1;
                    if (tick_16x) begin
                        if (tx_tick_cnt == 15) tx_state <= TX_IDLE;
                        else tx_tick_cnt <= tx_tick_cnt + 1;
                    end
                end
            endcase
        end
    end

    // ==========================================
    // 4. RX CORE FSM (Oversampling Logic)
    // ==========================================
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            rx_state <= RX_IDLE;
            rx_tick_cnt <= 0;
            rx_bit_cnt <= 0;
            rx_done_pulse <= 0;
            rx_shift_data <= 0;
        end else begin
            rx_done_pulse <= 0;
            case (rx_state)
                RX_IDLE: begin
                    if (rx_sync2 == 1'b0) begin // Pháthiện bit 0 (Start bit)
                        rx_state <= RX_START;
                        rx_tick_cnt <= 0;
                    end
                end
                RX_START: begin
                    if (tick_16x) begin
                        if (rx_tick_cnt == 7) begin // Lấy mẫu ở giữa bit Start
                            if (rx_sync2 == 1'b0) begin
                                rx_tick_cnt <= 0;
                                rx_bit_cnt <= 0;
                                rx_state <= RX_DATA;
                            end else rx_state <= RX_IDLE; // Nhiễu ảo, quay lại
                        end else rx_tick_cnt <= rx_tick_cnt + 1;
                    end
                end
                RX_DATA: begin
                    if (tick_16x) begin
                        if (rx_tick_cnt == 15) begin
                            rx_tick_cnt <= 0;
                            rx_shift_data <= {rx_sync2, rx_shift_data[7:1]};
                            if (rx_bit_cnt == 7) rx_state <= RX_STOP;
                            else rx_bit_cnt <= rx_bit_cnt + 1;
                        end else rx_tick_cnt <= rx_tick_cnt + 1;
                    end
                end
                RX_STOP: begin
                    if (tick_16x) begin
                        if (rx_tick_cnt == 15) begin
                            rx_done_pulse <= 1'b1;
                            rx_state <= RX_IDLE;
                        end else rx_tick_cnt <= rx_tick_cnt + 1;
                    end
                end
            endcase
        end
    end
endmodule