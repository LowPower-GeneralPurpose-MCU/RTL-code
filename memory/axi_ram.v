`timescale 1ns / 1ps

module axi_ram #(
    parameter ADDR_WIDTH        = 32,
    parameter DATA_WIDTH        = 32,
    parameter ID_WIDTH          = 7,
    parameter ADDR_MASK         = 32'h0000_FFFF, // Mask 64KB
    parameter MEM_DEPTH         = 16384,         // 64KB / 4 = 16384 Words
    parameter INIT_FILE         = ""             // Để trống nếu không cần nạp sẵn
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Kênh Ghi - Address
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

    // Kênh Ghi - Data
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]s_axi_wstrb,
    input  wire                     s_axi_wlast,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    // Kênh Ghi - Response
    output reg  [ID_WIDTH-1:0]      s_axi_bid,
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    // Kênh Đọc - Address
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

    // Kênh Đọc - Data
    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output wire [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready
);

    // =========================================================
    // PHẦN BỘ NHỚ (BLOCK RAM) - TÁCH RIÊNG ĐỌC VÀ GHI
    // =========================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram_memory [0:MEM_DEPTH-1];
    reg [DATA_WIDTH-1:0] rdata_out;

    wire                      bram_we;
    wire [ADDR_WIDTH-1:0]     bram_waddr;
    wire [DATA_WIDTH-1:0]     bram_wdata;
    wire [(DATA_WIDTH/8)-1:0] bram_wstrb;

    wire                      bram_re;
    wire [ADDR_WIDTH-1:0]     bram_raddr;

    // Khởi tạo bộ nhớ nếu có file
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, ram_memory);
        end
    end

    // Tiến trình Ghi đồng bộ (Không Reset)
    always @(posedge clk) begin
        if (bram_we) begin
            if (bram_wstrb[0]) ram_memory[bram_waddr][7:0]   <= bram_wdata[7:0];
            if (bram_wstrb[1]) ram_memory[bram_waddr][15:8]  <= bram_wdata[15:8];
            if (bram_wstrb[2]) ram_memory[bram_waddr][23:16] <= bram_wdata[23:16];
            if (bram_wstrb[3]) ram_memory[bram_waddr][31:24] <= bram_wdata[31:24];
        end
    end

    // Tiến trình Đọc đồng bộ (Không Reset)
    always @(posedge clk) begin
        if (bram_re) begin
            rdata_out <= ram_memory[bram_raddr];
        end
    end

    assign s_axi_rdata = rdata_out;

    // =========================================================
    // PHẦN LOGIC ĐIỀU KHIỂN AXI
    // =========================================================

    // --- Logic Kênh Ghi ---
    localparam W_IDLE = 2'd0, W_DATA = 2'd1, W_RESP = 2'd2;

    reg [1:0]            w_state;
    reg [ADDR_WIDTH-1:0] aw_addr_reg;
    reg [2:0]            aw_size_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            w_state       <= W_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            s_axi_bid     <= 0;
            aw_addr_reg   <= 0;
            aw_size_reg   <= 0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    s_axi_awready <= 1'b1;
                    if (s_axi_awvalid && s_axi_awready) begin
                        aw_addr_reg   <= s_axi_awaddr;
                        aw_size_reg   <= s_axi_awsize;
                        s_axi_bid     <= s_axi_awid;
                        
                        s_axi_awready <= 1'b0;
                        s_axi_wready  <= 1'b1;
                        w_state       <= W_DATA;
                    end
                end
                W_DATA: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        aw_addr_reg <= aw_addr_reg + (1 << aw_size_reg);
                        if (s_axi_wlast) begin
                            s_axi_wready <= 1'b0;
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= 2'b00; // OKAY
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

    // Móc nối tín hiệu Ghi vào BRAM
    assign bram_we    = (w_state == W_DATA) && s_axi_wvalid && s_axi_wready;
    assign bram_waddr = (aw_addr_reg & ADDR_MASK) >> 2; 
    assign bram_wdata = s_axi_wdata;
    assign bram_wstrb = s_axi_wstrb;

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
                            r_count     <= r_count + 1;
                            ar_addr_reg <= ar_addr_reg + (1 << ar_size_reg);
                            s_axi_rvalid <= 1'b0; // Hạ xuống để FETCH nhịp tiếp theo
                            r_state      <= R_FETCH;
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