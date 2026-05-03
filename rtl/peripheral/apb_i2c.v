`timescale 1ns / 1ps

module apb_i2c #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    input wire pclk, presetn, psel, penable, pwrite,
    input wire [ADDR_WIDTH-1:0] paddr,
    input wire [DATA_WIDTH-1:0] pwdata,
    input wire [3:0] pstrb,
    output reg pready, prdata, pslverr,

    // I2C Open-Drain Interface
    output wire scl_o, scl_oen,
    input  wire scl_i,
    output wire sda_o, sda_oen,
    input  wire sda_i,
    output wire i2c_irq
);

    // --- Register Map ---
    // 0x00: PRER  Prescaler
    // 0x04: TXR   Dữ liệu cần truyền
    // 0x08: RXR   Dữ liệu nhận được
    // 0x0C: CMD   [7] STA, [6] STO, [5] RD, [4] WR, [3] ACK (gửi đi)
    // 0x10: STAT  [7] RxACK (nhận), [6] BUSY, [1] Transferring, [0] IRQ

    reg [15:0] reg_prer;
    reg [7:0]  reg_txr;
    reg [7:0]  reg_rxr;
    reg [7:0]  reg_cmd;
    reg [7:0]  reg_stat;

    reg [4:0]  state;
    reg [15:0] tick_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  shift_reg;
    reg        core_done;
    reg        core_rx_ack;
    reg [7:0]  core_rx_data;

    localparam S_IDLE=0, S_START_A=1, S_START_B=2, S_START_C=3;
    localparam S_BIT_A=4, S_BIT_B=5, S_BIT_C=6, S_BIT_D=7;
    localparam S_ACK_A=8, S_ACK_B=9, S_ACK_C=10, S_ACK_D=11;
    localparam S_STOP_A=12, S_STOP_B=13, S_STOP_C=14;

    assign i2c_irq = reg_stat[0];

    // --- CDC cho SCL/SDA Input ---
    reg sda_s1, sda_s2;
    always @(posedge pclk) {sda_s2, sda_s1} <= {sda_s1, sda_i};

    // ==========================================
    // 1. APB INTERFACE
    // ==========================================
    reg cmd_trigger; // Kích hoạt I2C Core

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_prer <= 16'hFFFF; reg_txr <= 0; reg_cmd <= 0; reg_stat <= 0;
            pready <= 0; prdata <= 0; pslverr <= 0; cmd_trigger <= 0;
        end else begin
            pready <= psel && penable;
            pslverr <= 0;
            cmd_trigger <= 0;

            if (psel && penable && pwrite) begin
                case (paddr[11:0])
                    12'h000: reg_prer <= pwdata[15:0];
                    12'h004: reg_txr  <= pwdata[7:0];
                    12'h00C: begin 
                        reg_cmd <= pwdata[7:0];
                        if (pwdata[7] || pwdata[6] || pwdata[5] || pwdata[4]) begin
                            cmd_trigger <= 1'b1;
                            reg_stat[1] <= 1'b1; // Transferring
                        end
                    end
                    12'h010: if (pwdata[0]) reg_stat[0] <= 1'b0; // Xóa ngắt
                    default: pslverr <= 1'b1;
                endcase
            end
            
            if (psel && !penable && !pwrite) begin
                case (paddr[11:0])
                    12'h000: prdata <= reg_prer;
                    12'h004: prdata <= reg_txr;
                    12'h008: prdata <= reg_rxr;
                    12'h00C: prdata <= reg_cmd;
                    12'h010: prdata <= reg_stat;
                    default: pslverr <= 1'b1;
                endcase
            end

            // Phản hồi từ I2C Core
            if (core_done) begin
                reg_stat[1] <= 1'b0; // Clear Transferring
                reg_stat[0] <= 1'b1; // Set IRQ
                reg_stat[7] <= core_rx_ack; // Lưu ACK status
                reg_cmd <= 8'b0;     // Tự động clear CMD
                if (reg_cmd[5]) reg_rxr <= core_rx_data; // Nếu là lệnh RD, cập nhật RXR
            end
        end
    end

    // ==========================================
    // 2. I2C CORE FSM (Bit-Banging State Machine)
    // ==========================================
    // Điều khiển I2C Pads
    reg scl_out, sda_out;
    assign scl_oen = scl_out; // Open drain: 1 = Hi-Z (bus kéo lên 1), 0 = Pull down
    assign sda_oen = sda_out;
    assign scl_o   = 1'b0;    // Output data luôn là 0 để Pull-down
    assign sda_o   = 1'b0;

    wire tick = (tick_cnt == 0);

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            state <= S_IDLE; tick_cnt <= 0; bit_cnt <= 0;
            scl_out <= 1; sda_out <= 1; // Idle
            core_done <= 0; shift_reg <= 0;
        end else begin
            core_done <= 0;
            if (tick) tick_cnt <= reg_prer;
            else if (state != S_IDLE) tick_cnt <= tick_cnt - 1;

            case (state)
                S_IDLE: begin
                    if (cmd_trigger) begin
                        shift_reg <= reg_txr;
                        bit_cnt <= 7;
                        tick_cnt <= reg_prer;
                        if (reg_cmd[7]) state <= S_START_A; // Sinh Start Condition
                        else            state <= S_BIT_A;   // Truyền/Nhận data luôn
                    end
                end

                // --- START CONDITION ---
                // SCL high, SDA falls
                S_START_A: if (tick) begin sda_out <= 1; scl_out <= 1; state <= S_START_B; end
                S_START_B: if (tick) begin sda_out <= 0; scl_out <= 1; state <= S_START_C; end
                S_START_C: if (tick) begin sda_out <= 0; scl_out <= 0; 
                                           if (reg_cmd[4] || reg_cmd[5]) state <= S_BIT_A;
                                           else begin core_done <= 1; state <= S_IDLE; end
                                     end

                // --- BIT TRANSFER (WRITE OR READ) ---
                // Dịch dữ liệu ra ở SCL Low (BIT_A) -> Kéo SCL High (BIT_B/C) -> Lấy mẫu (BIT_D)
                S_BIT_A: if (tick) begin 
                            scl_out <= 0; 
                            sda_out <= reg_cmd[4] ? shift_reg[7] : 1'b1; // Write data hoặc nhả bus để Read
                            state <= S_BIT_B; 
                         end
                S_BIT_B: if (tick) begin scl_out <= 1; state <= S_BIT_C; end
                S_BIT_C: if (tick) begin 
                            shift_reg <= {shift_reg[6:0], sda_s2}; // Lấy mẫu Data
                            state <= S_BIT_D; 
                         end
                S_BIT_D: if (tick) begin 
                            scl_out <= 0;
                            if (bit_cnt == 0) state <= S_ACK_A;
                            else begin bit_cnt <= bit_cnt - 1; state <= S_BIT_A; end
                         end

                // --- ACKNOWLEDGE PHASE ---
                S_ACK_A: if (tick) begin 
                            scl_out <= 0; 
                            sda_out <= reg_cmd[5] ? reg_cmd[3] : 1'b1; // Nếu Read, gửi ACK host. Nếu Write, nhả bus chờ Slave ACK
                            state <= S_ACK_B; 
                         end
                S_ACK_B: if (tick) begin scl_out <= 1; state <= S_ACK_C; end
                S_ACK_C: if (tick) begin 
                            core_rx_ack <= sda_s2; // Lấy mẫu ACK từ Slave
                            core_rx_data <= shift_reg;
                            state <= S_ACK_D; 
                         end
                S_ACK_D: if (tick) begin 
                            scl_out <= 0; sda_out <= 1; 
                            if (reg_cmd[6]) state <= S_STOP_A; // Lệnh yêu cầu STOP
                            else begin core_done <= 1; state <= S_IDLE; end
                         end

                // --- STOP CONDITION ---
                // SCL high, SDA rises
                S_STOP_A: if (tick) begin sda_out <= 0; scl_out <= 0; state <= S_STOP_B; end
                S_STOP_B: if (tick) begin sda_out <= 0; scl_out <= 1; state <= S_STOP_C; end
                S_STOP_C: if (tick) begin sda_out <= 1; scl_out <= 1; core_done <= 1; state <= S_IDLE; end
            endcase
        end
    end
endmodule