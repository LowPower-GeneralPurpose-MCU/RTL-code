`timescale 1ns / 1ps

module apb_cordic #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    // --- Giao tiếp APB ---
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
    output reg                    pslverr
);

    // =========================================
    // BẢNG THANH GHI (REGISTER MAP)
    // 0x00: CTRL    [0] Start (Tự động clear)
    // 0x04: STATUS  [0] Done (Tự clear khi Start), [1] Busy
    // 0x08: X_IN    (Scale factor K chuẩn 16 iter = 39797. Định dạng Q16.16)
    // 0x0C: Y_IN    (Thường = 0. Định dạng Q16.16)
    // 0x10: Z_IN    (Góc đầu vào - ĐỊNH DẠNG BAM 32-BIT. 180 độ = 0x7FFFFFFF)
    // 0x14: X_OUT   (Kết quả Cos - Read Only. Định dạng Q16.16)
    // 0x18: Y_OUT   (Kết quả Sin - Read Only. Định dạng Q16.16)
    // 0x1C: Z_OUT   (Phần dư của góc - Read Only. Định dạng BAM)
    // =========================================

    // Bảng bộ nhớ tra cứu Atan cho 16 vòng lặp (Định dạng BAM 32-bit)
    // Tính bằng công thức: (atan(2^-i) / 180) * (2^31)
    wire signed [31:0] atan_table [0:15];
    assign atan_table[0]  = 32'sd536870912; // 45.00000 deg
    assign atan_table[1]  = 32'sd316933406; // 26.56505 deg
    assign atan_table[2]  = 32'sd167458907; // 14.03624 deg
    assign atan_table[3]  = 32'sd85004756;  // 7.12501 deg
    assign atan_table[4]  = 32'sd42667331;  // 3.57633 deg
    assign atan_table[5]  = 32'sd21354465;  // 1.78991 deg
    assign atan_table[6]  = 32'sd10679838;  // 0.89517 deg
    assign atan_table[7]  = 32'sd5340245;   // 0.44761 deg
    assign atan_table[8]  = 32'sd2670163;   // 0.22381 deg
    assign atan_table[9]  = 32'sd1335087;   // 0.11190 deg
    assign atan_table[10] = 32'sd667544;    // 0.05595 deg
    assign atan_table[11] = 32'sd333772;    // 0.02797 deg
    assign atan_table[12] = 32'sd166886;    // 0.01398 deg
    assign atan_table[13] = 32'sd83443;     // 0.00699 deg
    assign atan_table[14] = 32'sd41722;     // 0.00349 deg
    assign atan_table[15] = 32'sd20861;     // 0.00174 deg

    // Góc 90 độ dùng cho Pre-rotation trong hệ BAM
    localparam signed [31:0] BAM_DEG_90     = 32'sd1073741824; //  90 độ
    localparam signed [31:0] BAM_NEG_DEG_90 = -32'sd1073741824; // -90 độ

    // Thanh ghi nội bộ
    reg [31:0] reg_ctrl;
    reg [31:0] reg_status;
    reg [31:0] reg_xin;
    reg [31:0] reg_yin;
    reg [31:0] reg_zin;
    reg [31:0] reg_xout;
    reg [31:0] reg_yout;
    reg [31:0] reg_zout;

    // FSM States
    localparam IDLE = 2'b00, CALC = 2'b01, DONE = 2'b10;
    reg [1:0]  state;
    reg [4:0]  iter_cnt;
    
    reg signed [31:0] x_reg, y_reg, z_reg;
    reg signed [31:0] x_next, y_next, z_next;

    // --- Logic Giao tiếp APB ---
    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl   <= 32'd0;
            reg_status <= 32'd0;
            reg_xin    <= 32'd39797; // Hằng số K chuẩn cho 16 vòng lặp
            reg_yin    <= 32'd0;
            reg_zin    <= 32'd0;
            pready     <= 1'b0;
            prdata     <= 32'd0;
            pslverr    <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;

            // Xóa bit Start để tạo thành 1 xung duy nhất
            if (reg_ctrl[0]) reg_ctrl[0] <= 1'b0;

            // Quản lý trạng thái từ Core FSM
            if (state == DONE) begin
                reg_status[0] <= 1'b1; // Set Done
                reg_status[1] <= 1'b0; // Clear Busy
            end else if (state == CALC) begin
                reg_status[1] <= 1'b1; // Báo hiệu đang bận
                reg_status[0] <= 1'b0; // Xóa Done khi đang tính
            end

            // Xử lý Ghi APB
            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: reg_ctrl <= pwdata;
                    12'h008: reg_xin  <= pwdata;
                    12'h00C: reg_yin  <= pwdata;
                    12'h010: reg_zin  <= pwdata;
                    12'h014, 12'h018, 12'h01C, 12'h004: pslverr <= 1'b1; // Cố tình ghi vào Read-Only
                    default: ; 
                endcase
            end

            // Xử lý Đọc APB
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= reg_ctrl;
                    12'h004: prdata <= reg_status;
                    12'h008: prdata <= reg_xin;
                    12'h00C: prdata <= reg_yin;
                    12'h010: prdata <= reg_zin;
                    12'h014: prdata <= reg_xout;
                    12'h018: prdata <= reg_yout;
                    12'h01C: prdata <= reg_zout;
                    default: begin prdata <= 32'd0; pslverr <= 1'b1; end
                endcase
            end
        end
    end

    // --- Core CORDIC FSM ---
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            state    <= IDLE;
            iter_cnt <= 5'd0;
            x_reg    <= 32'd0;
            y_reg    <= 32'd0;
            z_reg    <= 32'd0;
            reg_xout <= 32'd0;
            reg_yout <= 32'd0;
            reg_zout <= 32'd0;
        end else begin
            case (state)
                IDLE: begin
                    iter_cnt <= 5'd0;
                    if (reg_ctrl[0]) begin // Nhận lệnh Start từ APB
                        state <= CALC;
                        
                        // LOGIC PRE-ROTATION: Đưa góc về khoảng [-90, 90]
                        if ($signed(reg_zin) > BAM_DEG_90) begin
                            // Góc > 90 độ: Xoay +90 độ trước
                            x_reg <= -$signed(reg_yin);
                            y_reg <= $signed(reg_xin);
                            z_reg <= $signed(reg_zin) - BAM_DEG_90;
                        end else if ($signed(reg_zin) < BAM_NEG_DEG_90) begin
                            // Góc < -90 độ: Xoay -90 độ trước
                            x_reg <= $signed(reg_yin);
                            y_reg <= -$signed(reg_xin);
                            z_reg <= $signed(reg_zin) - BAM_NEG_DEG_90; // Hoặc: + BAM_DEG_90
                        end else begin
                            // Nằm trong vùng an toàn, nạp bình thường
                            x_reg <= $signed(reg_xin);
                            y_reg <= $signed(reg_yin);
                            z_reg <= $signed(reg_zin);
                        end
                    end
                end

                CALC: begin
                    x_reg <= x_next;
                    y_reg <= y_next;
                    z_reg <= z_next;
                    
                    if (iter_cnt == 5'd15) begin
                        state <= DONE;
                    end else begin
                        iter_cnt <= iter_cnt + 1'b1;
                    end
                end

                DONE: begin
                    reg_xout <= x_reg;
                    reg_yout <= y_reg;
                    reg_zout <= z_reg;
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

    // Quyết định hướng quay: d = 1 nếu z >= 0, ngược lại d = 0
    wire d = (z_reg >= 0);

    // Tính toán vòng lặp CORDIC bằng phép dịch bit
    always @(*) begin
        if (d) begin
            x_next = x_reg - (y_reg >>> iter_cnt);
            y_next = y_reg + (x_reg >>> iter_cnt);
            z_next = z_reg - atan_table[iter_cnt];
        end else begin
            x_next = x_reg + (y_reg >>> iter_cnt);
            y_next = y_reg - (x_reg >>> iter_cnt);
            z_next = z_reg + atan_table[iter_cnt];
        end
    end

endmodule