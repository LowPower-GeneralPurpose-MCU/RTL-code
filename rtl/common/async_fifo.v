`timescale 1ns / 1ps

module async_fifo #(
    parameter DATA_WIDTH = 52, // Bề rộng packet. VD: {Write(1), Addr(16), Wdata(32), Size(2), Strb(1)}
    parameter ADDR_WIDTH = 4   // Depth = 16
)(
    // Ghi (Write Domain - CPU Clock)
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire                  wfull,
    
    // Đọc (Read Domain - System Bus Clock)
    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rinc,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire                  rempty
);

    reg  [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
    
    reg  [ADDR_WIDTH:0] wptr_bin, wptr_gray;
    reg  [ADDR_WIDTH:0] rptr_bin, rptr_gray;
    
    reg  [ADDR_WIDTH:0] wptr_gray_sync1, wptr_gray_sync2;
    reg  [ADDR_WIDTH:0] rptr_gray_sync1, rptr_gray_sync2;

    // --- Ghi vào RAM ---
    always @(posedge wclk) begin
        if (winc && !wfull) mem[wptr_bin[ADDR_WIDTH-1:0]] <= wdata;
    end
    assign rdata = mem[rptr_bin[ADDR_WIDTH-1:0]];

    // --- Cập nhật con trỏ Ghi (Write Pointer) ---
    wire [ADDR_WIDTH:0] wptr_bin_next = wptr_bin + (winc & ~wfull);
    wire [ADDR_WIDTH:0] wptr_gray_next = (wptr_bin_next >> 1) ^ wptr_bin_next;
    
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wptr_bin  <= 0;
            wptr_gray <= 0;
        end else begin
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
        end
    end

    // --- Cập nhật con trỏ Đọc (Read Pointer) ---
    wire [ADDR_WIDTH:0] rptr_bin_next = rptr_bin + (rinc & ~rempty);
    wire [ADDR_WIDTH:0] rptr_gray_next = (rptr_bin_next >> 1) ^ rptr_bin_next;
    
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rptr_bin  <= 0;
            rptr_gray <= 0;
        end else begin
            rptr_bin  <= rptr_bin_next;
            rptr_gray <= rptr_gray_next;
        end
    end

    // --- Đồng bộ hóa (2-Flop Synchronizer) ---
    // Đồng bộ Rptr sang Wclk để tính Full
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) {rptr_gray_sync2, rptr_gray_sync1} <= 0;
        else         {rptr_gray_sync2, rptr_gray_sync1} <= {rptr_gray_sync1, rptr_gray};
    end
    
    // Đồng bộ Wptr sang Rclk để tính Empty
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) {wptr_gray_sync2, wptr_gray_sync1} <= 0;
        else         {wptr_gray_sync2, wptr_gray_sync1} <= {wptr_gray_sync1, wptr_gray};
    end

    // --- Điều kiện Empty / Full ---
    assign rempty = (rptr_gray == wptr_gray_sync2);
    // Full xảy ra khi MSB và MSB-1 bị đảo ngược, các bit còn lại giống nhau
    assign wfull  = (wptr_gray == {~rptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1], rptr_gray_sync2[ADDR_WIDTH-2:0]});

endmodule

module async_fifo_cdc #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 4 // Sâu 16 words
)(
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire                  wfull,

    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rinc,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire                  rempty
);
    reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
    reg [ADDR_WIDTH:0] wptr, rptr;
    reg [ADDR_WIDTH:0] wq2_rptr, wq1_rptr, rq2_wptr, rq1_wptr;

    wire [ADDR_WIDTH:0] wptr_gray = wptr ^ (wptr >> 1);
    wire [ADDR_WIDTH:0] rptr_gray = rptr ^ (rptr >> 1);

    // Đồng bộ pointer từ miền R sang miền W
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) {wq2_rptr, wq1_rptr} <= 0;
        else         {wq2_rptr, wq1_rptr} <= {wq1_rptr, rptr_gray};
    end

    // Đồng bộ pointer từ miền W sang miền R
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) {rq2_wptr, rq1_wptr} <= 0;
        else         {rq2_wptr, rq1_wptr} <= {rq1_wptr, wptr_gray};
    end

    assign rempty = (rptr_gray == rq2_wptr);
    assign wfull  = (wptr_gray == {~wq2_rptr[ADDR_WIDTH:ADDR_WIDTH-1], wq2_rptr[ADDR_WIDTH-2:0]});

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) wptr <= 0;
        else if (winc && !wfull) begin
            mem[wptr[ADDR_WIDTH-1:0]] <= wdata;
            wptr <= wptr + 1;
        end
    end

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) rptr <= 0;
        else if (rinc && !rempty) rptr <= rptr + 1;
    end

    assign rdata = mem[rptr[ADDR_WIDTH-1:0]];
endmodule