`timescale 1ns / 1ps

module axi_ram #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter BASE_ADDR  = 32'h8000_0000,
    parameter MEM_DEPTH  = 16384 // 16384 words = 64 Kilobytes
)(
    input wire clk,
    input wire rst_n,

    // =========================================================================
    // KÊNH GHI (WRITE CHANNELS)
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  wire                   s_axi_awvalid,
    output reg                    s_axi_awready,
    input  wire [DATA_WIDTH-1:0]  s_axi_wdata,
    input  wire [3:0]             s_axi_wstrb,
    input  wire                   s_axi_wvalid,
    output reg                    s_axi_wready,
    output reg  [1:0]             s_axi_bresp,
    output reg                    s_axi_bvalid,
    input  wire                   s_axi_bready,

    // =========================================================================
    // KÊNH ĐỌC (READ CHANNELS) - CÓ BURST
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire [7:0]             s_axi_arlen,
    input  wire [2:0]             s_axi_arsize,
    input  wire [1:0]             s_axi_arburst,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    
    output wire [DATA_WIDTH-1:0]  s_axi_rdata, // Nối thẳng từ BRAM out, không dùng reg FSM
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rlast,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready
);

    reg [ADDR_WIDTH-1:0] write_addr_latch;
    reg [ADDR_WIDTH-1:0] read_addr_latch;

    // =========================================================================
    // KHỐI 1: TÁCH BIỆT BRAM VẬT LÝ (TUYỆT ĐỐI KHÔNG CÓ RESET Ở ĐÂY)
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] bram_memory [0:MEM_DEPTH-1];

    // Cổng A (Write Port)
    wire [13:0] ram_waddr = (write_addr_latch - BASE_ADDR) >> 2;
    wire [3:0]  ram_we; // Sinh ra từ Logic Ghi
    
    always @(posedge clk) begin
        if (ram_we[0]) bram_memory[ram_waddr][7:0]   <= s_axi_wdata[7:0];
        if (ram_we[1]) bram_memory[ram_waddr][15:8]  <= s_axi_wdata[15:8];
        if (ram_we[2]) bram_memory[ram_waddr][23:16] <= s_axi_wdata[23:16];
        if (ram_we[3]) bram_memory[ram_waddr][31:24] <= s_axi_wdata[31:24];
    end

    // Cổng B (Read Port)
    wire [13:0] ram_raddr = (read_addr_latch - BASE_ADDR) >> 2;
    reg  [DATA_WIDTH-1:0] ram_rdata_reg; // Thanh ghi đầu ra vật lý của BRAM
    reg         ram_ren; // Sinh ra từ Logic Đọc

    always @(posedge clk) begin
        if (ram_ren) begin
            ram_rdata_reg <= bram_memory[ram_raddr];
        end
        // NGUYÊN TẮC: Khi ram_ren = 0, thanh ghi tự động giữ giá trị cũ (Phù hợp với stall rready=0 của AXI)
    end
    
    assign s_axi_rdata = ram_rdata_reg;

    // =========================================================================
    // KHỐI 2: LOGIC ĐIỀU KHIỂN (AXI FSM)
    // =========================================================================
    
    // --- BIẾN CHO KÊNH GHI ---
    reg aw_en;
    
    assign ram_we = (s_axi_wready && s_axi_wvalid && s_axi_awready && s_axi_awvalid) ? s_axi_wstrb : 4'b0000;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0; aw_en <= 1'b1; write_addr_latch <= 32'h0;
        end else begin
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
                s_axi_awready <= 1'b1; aw_en <= 1'b0; write_addr_latch <= s_axi_awaddr;
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_awready <= 1'b0; aw_en <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) s_axi_wready <= 1'b0;
        else if (~s_axi_wready && s_axi_wvalid && s_axi_awvalid && aw_en) s_axi_wready <= 1'b1;
        else s_axi_wready <= 1'b0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s_axi_bvalid <= 1'b0; s_axi_bresp <= 2'b00; end
        else if (s_axi_awready && s_axi_awvalid && s_axi_wready && s_axi_wvalid && ~s_axi_bvalid) begin
            s_axi_bvalid <= 1'b1; s_axi_bresp <= 2'b00;
        end else if (s_axi_bready && s_axi_bvalid) begin
            s_axi_bvalid <= 1'b0;
        end
    end

    // --- BIẾN CHO KÊNH ĐỌC (PIPELINED BURST) ---
    reg [7:0]            burst_len;
    reg [7:0]            fetch_cnt; // Đếm số từ đã đẩy vào BRAM pipeline
    reg [7:0]            send_cnt;  // Đếm số từ Master đã nhận
    
    localparam R_IDLE    = 2'd0;
    localparam R_LATENCY = 2'd1;
    localparam R_DATA    = 2'd2;
    reg [1:0] r_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready   <= 1'b0;
            s_axi_rvalid    <= 1'b0;
            s_axi_rlast     <= 1'b0;
            s_axi_rresp     <= 2'b00;
            read_addr_latch <= 32'h0;
            burst_len       <= 8'd0;
            fetch_cnt       <= 8'd0;
            send_cnt        <= 8'd0;
            ram_ren         <= 1'b0;
            r_state         <= R_IDLE;
        end else begin
            case (r_state)
                R_IDLE: begin
                    s_axi_arready <= 1'b1;
                    s_axi_rvalid  <= 1'b0;
                    ram_ren       <= 1'b0; // Mặc định tắt đọc
                    
                    if (s_axi_arready && s_axi_arvalid) begin
                        s_axi_arready   <= 1'b0; // Chốt địa chỉ
                        read_addr_latch <= s_axi_araddr;
                        burst_len       <= s_axi_arlen;
                        fetch_cnt       <= 8'd0;
                        send_cnt        <= 8'd0;
                        r_state         <= R_LATENCY;
                    end
                end

                R_LATENCY: begin
                    // Chu kỳ này read_addr_latch đã mang giá trị đúng của lần đọc đầu tiên
                    ram_ren <= 1'b1; // Kích hoạt BRAM đọc dữ liệu
                    r_state <= R_DATA;
                end

                R_DATA: begin
                    // Dữ liệu BRAM ở nhịp này đã được cập nhật đúng vào s_axi_rdata
                    s_axi_rvalid <= 1'b1;
                    s_axi_rresp  <= 2'b00;
                    s_axi_rlast  <= (send_cnt == burst_len);
                    
                    if (s_axi_rvalid && s_axi_rready) begin
                        // Master đã nhận dữ liệu
                        if (send_cnt == burst_len) begin
                            // Kết thúc burst
                            s_axi_rvalid  <= 1'b0;
                            s_axi_rlast   <= 1'b0;
                            ram_ren       <= 1'b0;
                            r_state       <= R_IDLE;
                        end else begin
                            send_cnt <= send_cnt + 1;
                            // Tiếp tục nạp nhịp sau vào pipeline
                            if (fetch_cnt < burst_len) begin
                                ram_ren         <= 1'b1;
                                read_addr_latch <= read_addr_latch + 4;
                                fetch_cnt       <= fetch_cnt + 1;
                            end else begin
                                ram_ren <= 1'b0;
                            end
                        end
                    end else begin
                        // Master stall (rready = 0) -> Ngắt tín hiệu ren để BRAM output register không bị ghi đè
                        ram_ren <= 1'b0;
                    end
                end
            endcase
        end
    end

endmodule