`timescale 1ns / 1ps

module reset_sync (
    input  wire clk,
    input  wire rst_in_n,
    output wire rst_out_n
);
    reg r1, r2;
    always @(posedge clk or negedge rst_in_n) begin
        if (!rst_in_n) begin
            r1 <= 1'b0;
            r2 <= 1'b0;
        end else begin
            r1 <= 1'b1;
            r2 <= r1;
        end
    end
    assign rst_out_n = r2;
endmodule

// =========================================================================
// 1. MODULE: cdc_sync_bit
// Chức năng: Đồng bộ hóa 1 bit tín hiệu điều khiển (Control/Level)
// Vị trí dùng: periph_req từ UART/SPI sang DMA, ngắt từ ngoại vi sang PLIC.
// =========================================================================
module cdc_sync_bit #(
    parameter STAGES = 2 // Khuyên dùng 3 cho các miền clock tốc độ rất cao (>300MHz)
)(
    input  wire clk_dst,
    input  wire rst_dst_n,
    input  wire d_in,
    output wire q_out
);
    reg [STAGES-1:0] sync_reg;
    
    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
            sync_reg <= {STAGES{1'b0}};
        end else begin
            sync_reg <= {sync_reg[STAGES-2:0], d_in}; // Dịch trái
        end
    end
    
    assign q_out = sync_reg[STAGES-1];
endmodule

// =========================================================================
// 2. MODULE: cdc_sync_vector
// Chức năng: Đồng bộ hóa 1 mảng bit độc lập (Không dùng cho bus dữ liệu)
// Vị trí dùng: Đồng bộ các tín hiệu trạng thái (status flags) rời rạc.
// =========================================================================
module cdc_sync_vector #(
    parameter WIDTH  = 8,
    parameter STAGES = 2
)(
    input  wire              clk_dst,
    input  wire              rst_dst_n,
    input  wire [WIDTH-1:0]  d_in,
    output wire [WIDTH-1:0]  q_out
);
    genvar i;
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : gen_sync
            cdc_sync_bit #(.STAGES(STAGES)) u_bit (
                .clk_dst   (clk_dst),
                .rst_dst_n (rst_dst_n),
                .d_in      (d_in[i]),
                .q_out     (q_out[i])
            );
        end
    endgenerate
endmodule

// =========================================================================
// 3. MODULE: cdc_pulse
// Chức năng: Truyền 1 xung (Pulse) an toàn từ clock này sang clock khác
// Cơ chế: Đổi Pulse thành Toggle -> Đồng bộ -> Bắt cạnh để tạo Pulse mới
// Vị trí dùng: Các tín hiệu Clear, Start, Stop (ví dụ từ APB kích hoạt CORDIC)
// =========================================================================
module cdc_pulse (
    input  wire clk_src,
    input  wire rst_src_n,
    input  wire pulse_src,  // Xung đầu vào (1 chu kỳ của clk_src)
    
    input  wire clk_dst,
    input  wire rst_dst_n,
    output wire pulse_dst   // Xung đầu ra (1 chu kỳ của clk_dst)
);
    // Miền Source: Chuyển Pulse thành Toggle
    reg toggle_src;
    always @(posedge clk_src or negedge rst_src_n) begin
        if (!rst_src_n) toggle_src <= 1'b0;
        else if (pulse_src) toggle_src <= ~toggle_src;
    end

    // Đưa Toggle qua Sync_FF ở miền Đích
    wire toggle_dst_sync;
    cdc_sync_bit #(.STAGES(2)) u_sync (
        .clk_dst(clk_dst), .rst_dst_n(rst_dst_n),
        .d_in(toggle_src), .q_out(toggle_dst_sync)
    );

    // Miền Đích: Bắt cạnh (Edge Detection) Toggle để tái tạo Pulse
    reg toggle_dst_dly;
    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) toggle_dst_dly <= 1'b0;
        else            toggle_dst_dly <= toggle_dst_sync;
    end

    assign pulse_dst = toggle_dst_sync ^ toggle_dst_dly; // Kích hoạt khi có sự thay đổi (XOR)
endmodule

// =========================================================================
// 4. MODULE: cdc_handshake
// Chức năng: Truyền 1 gói dữ liệu (Data/Cmd) giữa 2 miền clock chênh lệch lớn
// Cơ chế: Yêu cầu (Req) -> Đồng bộ -> Đích nhận data -> Phản hồi (Ack)
// Vị trí dùng: Giao tiếp JTAG DTM (TCK) với Debug Module (System Clock)
// =========================================================================
module cdc_handshake #(
    parameter DATA_WIDTH = 32
)(
    // Miền Gửi (Source)
    input  wire                   clk_src,
    input  wire                   rst_src_n,
    input  wire [DATA_WIDTH-1:0]  data_src,
    input  wire                   req_src,
    output wire                   ack_src,   // Báo cho Source biết đã truyền xong
    
    // Miền Nhận (Destination)
    input  wire                   clk_dst,
    input  wire                   rst_dst_n,
    output reg  [DATA_WIDTH-1:0]  data_dst,
    output wire                   valid_dst, // Báo cho Dest biết có data mới
    input  wire                   ack_dst    // Dest xác nhận đã lấy data
);
    // --- Latch Data ở miền Source ---
    reg [DATA_WIDTH-1:0] data_hold;
    reg req_src_reg;
    
    always @(posedge clk_src or negedge rst_src_n) begin
        if (!rst_src_n) begin
            req_src_reg <= 1'b0;
            data_hold   <= {DATA_WIDTH{1'b0}};
        end else begin
            if (req_src && !req_src_reg && !ack_src) begin // Bắt đầu gửi
                req_src_reg <= 1'b1;
                data_hold   <= data_src;
            end else if (ack_src) begin // Chờ phản hồi từ Dest để hạ Req
                req_src_reg <= 1'b0;
            end
        end
    end

    // --- Đưa Req sang miền Dest ---
    wire req_dst_sync;
    cdc_sync_bit #(.STAGES(2)) u_sync_req (
        .clk_dst(clk_dst), .rst_dst_n(rst_dst_n),
        .d_in(req_src_reg), .q_out(req_dst_sync)
    );

    // --- Latch Data ở miền Dest & Báo Valid ---
    reg req_dst_dly;
    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
            req_dst_dly <= 1'b0;
            data_dst    <= {DATA_WIDTH{1'b0}};
        end else begin
            req_dst_dly <= req_dst_sync;
            if (req_dst_sync && !req_dst_dly) begin // Cạnh lên của Req_sync
                data_dst <= data_hold; // Lấy Data vì nó đã ổn định (Req đã Sync)
            end
        end
    end
    
    // Valid chỉ duy trì trong lúc Dest chưa Ack
    reg ack_dst_reg;
    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) ack_dst_reg <= 1'b0;
        else if (req_dst_sync && ack_dst) ack_dst_reg <= 1'b1;
        else if (!req_dst_sync)           ack_dst_reg <= 1'b0;
    end
    assign valid_dst = req_dst_sync && !ack_dst_reg;

    // --- Phản hồi Ack về miền Source ---
    cdc_sync_bit #(.STAGES(2)) u_sync_ack (
        .clk_dst(clk_src), .rst_dst_n(rst_src_n),
        .d_in(ack_dst_reg), .q_out(ack_src)
    );
endmodule

// =========================================================================
// 5. MODULE: cdc_async_fifo_wrapper (Tuỳ chọn)
// Chức năng: Bọc lại fifo_async.v để tên module thống nhất trong cdc_bridge
// Vị trí dùng: AXI-to-APB Bridge, UART Core to APB.
// =========================================================================
module cdc_async_fifo_wrapper #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH_LOG2 = 4
)(
    input  wire wclk,
    input  wire wrst_n,
    input  wire wen,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire wfull,
    
    input  wire rclk,
    input  wire rrst_n,
    input  wire ren,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire rempty
);
    // Tính toán Depth từ số bit địa chỉ
    localparam FIFO_DEPTH_VAL = (1 << DEPTH_LOG2);

    // Sử dụng module async_fifo phiên bản mới
    async_fifo #(
        .ASFIFO_TYPE(0),         // 0: Normal FIFO
        .DATA_WIDTH(DATA_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH_VAL)
    ) u_async_fifo (
        .clk_wr_domain (wclk),
        .clk_rd_domain (rclk),
        .data_i        (wdata),
        .data_o        (rdata),
        .wr_valid_i    (wen),
        .rd_valid_i    (ren),
        .empty_o       (rempty),
        .full_o        (wfull),
        .wr_ready_o    (), // Bỏ trống vì wrapper cũ dùng cờ wfull
        .rd_ready_o    (),
        .almost_empty_o(),
        .almost_full_o (),
        .rst_n         (wrst_n) // Dùng reset của miền write làm reset chung
    );
endmodule