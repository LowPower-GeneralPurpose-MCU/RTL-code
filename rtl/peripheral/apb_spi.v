`timescale 1ns / 1ps

module apb_spi #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    input  wire pclk, presetn, psel, penable, pwrite,
    input  wire [ADDR_WIDTH-1:0] paddr,
    input  wire [DATA_WIDTH-1:0] pwdata,
    input  wire [3:0] pstrb,
    output reg  pready, prdata, pslverr,

    // SPI Physical Pins
    output wire sclk,
    output wire mosi,
    input  wire miso,
    output reg  cs_n,
    output wire spi_irq
);

    // --- Register Map ---
    // 0x00: CTRL   [1] CPOL, [0] CPHA
    // 0x04: CS     [0] Chip Select 0
    // 0x08: DIV    SCLK Divider
    // 0x0C: TXDATA
    // 0x10: RXDATA
    // 0x14: STATUS [0] BUSY, [1] DONE

    reg [31:0] reg_div;
    reg [1:0]  reg_ctrl;
    reg        reg_cs_n;
    reg [7:0]  tx_reg, rx_reg;
    reg        spi_start;
    wire       spi_busy;
    reg        spi_done;

    reg [2:0]  state;
    reg [31:0] clk_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  spi_tx_shift;
    reg [7:0]  spi_rx_shift;
    reg        sclk_reg;
    reg        spi_done_pulse;
    
    assign spi_irq = spi_done;

    // --- CDC MISO ---
    reg miso_sync1, miso_sync2;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) {miso_sync2, miso_sync1} <= 2'b00;
        else          {miso_sync2, miso_sync1} <= {miso_sync1, miso};
    end

    // ==========================================
    // 1. APB INTERFACE
    // ==========================================
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl <= 0; reg_div <= 2; cs_n <= 1;
            spi_start <= 0; spi_done <= 0;
            pready <= 0; prdata <= 0; pslverr <= 0;
        end else begin
            pready <= psel && penable;
            pslverr <= 0;
            spi_start <= 0; // Pulse

            if (psel && penable && pwrite) begin
                case (paddr[11:0])
                    12'h000: reg_ctrl <= pwdata[1:0];
                    12'h004: cs_n <= pwdata[0];
                    12'h008: reg_div <= pwdata;
                    12'h00C: if (!spi_busy) begin tx_reg <= pwdata[7:0]; spi_start <= 1; spi_done <= 0; end
                             else pslverr <= 1;
                    default: pslverr <= 1;
                endcase
            end
            
            if (psel && !penable && !pwrite) begin
                case (paddr[11:0])
                    12'h000: prdata <= {30'b0, reg_ctrl};
                    12'h004: prdata <= {31'b0, cs_n};
                    12'h008: prdata <= reg_div;
                    12'h010: begin prdata <= {24'b0, rx_reg}; spi_done <= 0; end // Read clears DONE
                    12'h014: prdata <= {30'b0, spi_done, spi_busy};
                    default: pslverr <= 1;
                endcase
            end
            
            if (spi_done_pulse) begin
                rx_reg <= spi_rx_shift;
                spi_done <= 1'b1;
            end
        end
    end

    // ==========================================
    // 2. SPI CORE FSM
    // ==========================================
    assign sclk = (state == 0) ? reg_ctrl[1] : sclk_reg;
    assign mosi = spi_tx_shift[7]; // MSB first
    assign spi_busy = (state != 0);

    localparam IDLE = 0, PHASE1 = 1, PHASE2 = 2;

    wire cpol = reg_ctrl[1];
    wire cpha = reg_ctrl[0];

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            state <= IDLE;
            clk_cnt <= 0;
            bit_cnt <= 0;
            sclk_reg <= 0;
            spi_tx_shift <= 0;
            spi_rx_shift <= 0;
            spi_done_pulse <= 0;
        end else begin
            spi_done_pulse <= 0;
            case (state)
                IDLE: begin
                    sclk_reg <= cpol;
                    if (spi_start) begin
                        spi_tx_shift <= tx_reg;
                        clk_cnt <= 0;
                        bit_cnt <= 0;
                        state <= PHASE1;
                    end
                end
                
                PHASE1: begin // Cạnh đầu tiên
                    if (clk_cnt == reg_div) begin
                        clk_cnt <= 0;
                        sclk_reg <= ~sclk_reg;
                        state <= PHASE2;
                        
                        // Nếu CPHA=0: Lấy mẫu ở Phase 1. CPHA=1: Lấy mẫu ở Phase 2
                        if (cpha == 0) spi_rx_shift <= {spi_rx_shift[6:0], miso_sync2};
                    end else clk_cnt <= clk_cnt + 1;
                end
                
                PHASE2: begin // Cạnh thứ hai
                    if (clk_cnt == reg_div) begin
                        clk_cnt <= 0;
                        sclk_reg <= ~sclk_reg;
                        
                        if (cpha == 1) spi_rx_shift <= {spi_rx_shift[6:0], miso_sync2};
                        
                        if (bit_cnt == 7) begin
                            state <= IDLE;
                            spi_done_pulse <= 1;
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