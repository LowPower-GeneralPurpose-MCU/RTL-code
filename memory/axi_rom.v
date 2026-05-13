`timescale 1ns / 1ps

module axi_rom #(
    parameter ADDR_WIDTH        = 32,
    parameter DATA_WIDTH        = 32,
    parameter ID_WIDTH          = 7,
    parameter ADDR_MASK         = 32'h0000_3FFF, // Mask 16KB
    parameter MEM_DEPTH         = 4096,          // 16KB / 4 = 4096 Words
    parameter INIT_FILE         = "boot.mem"     // File chứa mã máy
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Kênh Ghi (Bị chặn, trả về lỗi SLVERR)
    input  wire [ID_WIDTH-1:0]      s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire [7:0]               s_axi_awlen,
    input  wire [2:0]               s_axi_awsize,
    input  wire [1:0]               s_axi_awburst,
    input  wire                     s_axi_awlock,
    input  wire [3:0]               s_axi_awcache,
    input  wire [2:0]               s_axi_awprot,
    input  wire [3:0]               s_axi_awqos,
    input  wire [3:0]               s_axi_awregion,
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

    // Kênh Đọc
    input  wire [ID_WIDTH-1:0]      s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire [7:0]               s_axi_arlen,
    input  wire [2:0]               s_axi_arsize,
    input  wire [1:0]               s_axi_arburst,
    input  wire                     s_axi_arlock,
    input  wire [3:0]               s_axi_arcache,
    input  wire [2:0]               s_axi_arprot,
    input  wire [3:0]               s_axi_arqos,
    input  wire [3:0]               s_axi_arregion,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output wire [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready
);

    // =========================================================
    // PHẦN BỘ NHỚ (BLOCK ROM) - CHỈ CÓ CỔNG ĐỌC
    // =========================================================
    (* rom_style = "block" *) reg [DATA_WIDTH-1:0] rom_memory [0:MEM_DEPTH-1];
    reg [DATA_WIDTH-1:0] rdata_out;

    wire                  bram_re;
    wire [ADDR_WIDTH-1:0] bram_raddr;

    // Khởi tạo bộ nhớ ROM từ file
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, rom_memory);
        end
    end

    // Tiến trình Đọc đồng bộ (Không Reset)
    always @(posedge clk) begin
        if (bram_re) begin
            rdata_out <= rom_memory[bram_raddr];
        end
    end

    assign s_axi_rdata = rdata_out;

    // =========================================================
    // PHẦN LOGIC ĐIỀU KHIỂN AXI
    // =========================================================

    // --- Logic Kênh Ghi (LUÔN BÁO LỖI VÌ LÀ ROM) ---
    localparam W_IDLE = 2'd0, W_SINK = 2'd1, W_RESP = 2'd2;
    reg [1:0] w_state;

    always @(posedge clk) begin
        if (!rst_n) begin
            w_state       <= W_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bid     <= 0;
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
                            s_axi_bresp  <= 2'b10; // Lỗi SLVERR
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
                default: w_state <= W_IDLE;
            endcase
        end
    end

    // --- Logic Kênh Đọc ---
    localparam R_IDLE = 2'd0, R_FETCH = 2'd1, R_DATA = 2'd2;

    reg [1:0]            r_state;
    reg [ADDR_WIDTH-1:0] ar_addr_reg;
    reg [7:0]            ar_len_reg;
    reg [2:0]            ar_size_reg;
    reg [7:0]            r_count;

    always @(posedge clk) begin
        if (!rst_n) begin
            r_state       <= R_IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rlast   <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rid     <= 0;
            ar_addr_reg   <= 0;
            ar_len_reg    <= 0;
            ar_size_reg   <= 0;
            r_count       <= 0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    s_axi_arready <= 1'b1;
                    if (s_axi_arvalid && s_axi_arready) begin
                        ar_addr_reg   <= s_axi_araddr;
                        ar_len_reg    <= s_axi_arlen;
                        ar_size_reg   <= s_axi_arsize;
                        s_axi_rid     <= s_axi_arid;
                        r_count       <= 8'd0;
                        
                        s_axi_arready <= 1'b0;
                        r_state       <= R_FETCH;
                    end
                end
                R_FETCH: begin
                    s_axi_rvalid <= 1'b1;
                    s_axi_rresp  <= 2'b00; // OKAY
                    s_axi_rlast  <= (r_count == ar_len_reg);
                    r_state      <= R_DATA;
                end
                R_DATA: begin
                    if (s_axi_rvalid && s_axi_rready) begin
                        if (s_axi_rlast) begin
                            s_axi_rvalid <= 1'b0;
                            s_axi_rlast  <= 1'b0;
                            r_state      <= R_IDLE;
                        end else begin
                            // Back-to-back beats: keep rvalid=1.
                            // BRAM already prefetched next beat (is_active_read fires this cycle),
                            // so rdata_out is valid on the very next clock edge.
                            r_count     <= r_count + 8'd1;
                            ar_addr_reg <= ar_addr_reg + (1 << ar_size_reg);
                            s_axi_rlast <= (r_count + 8'd1 == ar_len_reg);
                            // rvalid stays 1, r_state stays R_DATA
                        end
                    end
                end
                default: r_state <= R_IDLE;
            endcase
        end
    end

    // Móc nối tín hiệu Đọc vào BRAM
    wire is_idle_read   = (r_state == R_IDLE) && s_axi_arvalid && s_axi_arready;
    wire is_active_read = (r_state == R_DATA) && s_axi_rvalid && s_axi_rready && !s_axi_rlast;
    
    assign bram_re    = is_idle_read || is_active_read;
    wire [ADDR_WIDTH-1:0] current_raddr = is_idle_read ? s_axi_araddr : (ar_addr_reg + (1 << ar_size_reg));
    assign bram_raddr = (current_raddr & ADDR_MASK) >> 2;

endmodule