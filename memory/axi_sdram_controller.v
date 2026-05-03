`timescale 1ns / 1ps

module axi_sdram_controller #(
    // ==========================================
    // THAM S�? CẤU HÌNH HỆ TH�?NG
    // ==========================================
    parameter ADDR_WIDTH            = 32,
    parameter DATA_WIDTH            = 32,
    parameter ID_WIDTH              = 5,
    
    // ==========================================
    // THAM S�? CẤU HÌNH SDRAM (Ví dụ cho chip 32-bit data)
    // ==========================================
    parameter SDRAM_ADDR_WIDTH      = 13, // Kích thước Row: 13 bit (8192 rows)
    parameter SDRAM_COL_WIDTH       = 9,  // Kích thước Column: 9 bit (512 cols)
    parameter SDRAM_BANK_WIDTH      = 2,  // 4 Banks
    parameter SDRAM_DATA_WIDTH      = 32, 
    
    // ==========================================
    // THAM S�? THỜI GIAN SDRAM (Tính theo chu kỳ Clock - VD: Hệ thống chạy 100MHz = 10ns)
    // ==========================================
    parameter INIT_DELAY_CYCLES     = 20000, // 200us th�?i gian khởi động ban đầu
    parameter TRP_CYCLES            = 2,     // Th�?i gian Precharge
    parameter TRCD_CYCLES           = 2,     // Th�?i gian trễ từ Activate đến Read/Write
    parameter TCAS_CYCLES           = 3,     // �?ộ trễ CAS (CAS Latency)
    parameter TRFC_CYCLES           = 7,     // Th�?i gian làm tươi (Auto-Refresh)
    parameter TWR_CYCLES            = 2,     // Th�?i gian phục hồi sau khi Ghi (Write Recovery)
    parameter REFRESH_PERIOD_CYCLES = 780    // Tần suất làm tươi 
)(
    input  wire                     clk,      // Xung 100MHz (0 độ) cho Controller
    input  wire                     clk_phy,  // Xung 100MHz (180 độ) cấp ra chip vật lý
    input  wire                     rst_n,

    // ==========================================
    // AXI4 FULL SLAVE INTERFACE
    // ==========================================
    // Kênh �?ịa Chỉ Ghi (Write Address Channel)
    input  wire [ID_WIDTH-1:0]      s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire [7:0]               s_axi_awlen,
    input  wire [2:0]               s_axi_awsize,
    input  wire [1:0]               s_axi_awburst,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,

    // Kênh Dữ Liệu Ghi (Write Data Channel)
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]s_axi_wstrb,
    input  wire                     s_axi_wlast,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    // Kênh Phản Hồi Ghi (Write Response Channel)
    output reg  [ID_WIDTH-1:0]      s_axi_bid,
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    // Kênh �?ịa Chỉ �?�?c (Read Address Channel)
    input  wire [ID_WIDTH-1:0]      s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire [7:0]               s_axi_arlen,
    input  wire [2:0]               s_axi_arsize,
    input  wire [1:0]               s_axi_arburst,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    // Kênh Dữ Liệu �?�?c (Read Data Channel)
    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output reg  [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready,

    // ==========================================
    // GIAO DIỆN VẬT L�? SDRAM (PHY)
    // ==========================================
    output wire                     sdram_clk,
    output wire                     sdram_cke,
    output wire                     sdram_cs_n,
    output wire                     sdram_ras_n,
    output wire                     sdram_cas_n,
    output wire                     sdram_we_n,
    output wire [SDRAM_BANK_WIDTH-1:0] sdram_ba,
    output wire [SDRAM_ADDR_WIDTH-1:0] sdram_addr,
    output wire [(SDRAM_DATA_WIDTH/8)-1:0] sdram_dqm,
    
    input  wire [SDRAM_DATA_WIDTH-1:0] sdram_dq_i,
    output wire [SDRAM_DATA_WIDTH-1:0] sdram_dq_o,
    output wire                        sdram_dq_oe
);

    // ==========================================
    // �?ỊNH NGHĨA C�?C LỆNH SDRAM (CS_N, RAS_N, CAS_N, WE_N)
    // ==========================================
    localparam CMD_LOAD_MODE = 4'b0000;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_ACTIVE    = 4'b0011;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_NOP       = 4'b0111;

    // Cấp Clock và Clock Enable liên tục
    assign sdram_clk = clk;
    assign sdram_cke = 1'b1;

    // ==========================================
    // QUẢN L�? T�?N HIỆU VÀ TRẠNG TH�?I NỘI BỘ
    // ==========================================
    // Trạng thái FSM
    localparam ST_INIT_WAIT  = 4'd0, ST_INIT_PRE   = 4'd1, ST_INIT_REF1  = 4'd2;
    localparam ST_INIT_REF2  = 4'd3, ST_INIT_LMR   = 4'd4, ST_IDLE       = 4'd5;
    localparam ST_PRECHARGE  = 4'd6, ST_ACTIVATE   = 4'd7;
    localparam ST_WRITE_BEAT = 4'd8, ST_WRITE_WAIT = 4'd9;
    localparam ST_READ_BEAT  = 4'd10,ST_READ_WAIT  = 4'd11, ST_READ_SEND = 4'd12;
    localparam ST_REFRESH    = 4'd13;

    reg [3:0]  state;
    reg [15:0] delay_timer;
    
    // Bộ đếm làm tươi (Auto-Refresh Timer)
    reg [11:0] refresh_counter;
    reg        refresh_request;

    // Thanh ghi đi�?u khiển SDRAM PHY
    reg [3:0]  cmd_reg;
    reg [SDRAM_BANK_WIDTH-1:0] ba_reg;
    reg [SDRAM_ADDR_WIDTH-1:0] addr_reg;
    reg [(SDRAM_DATA_WIDTH/8)-1:0] dqm_reg;
    
    // Quản lý Bus Dữ liệu (�?ã tách kh�?i Tri-state buffer)
    reg [SDRAM_DATA_WIDTH-1:0] dq_out_reg;
    reg                        dq_oe_reg;

    // �?�?c dữ liệu từ cổng input
    wire [SDRAM_DATA_WIDTH-1:0] dq_in_wire = sdram_dq_i;

    assign sdram_cs_n  = cmd_reg[3];
    assign sdram_ras_n = cmd_reg[2];
    assign sdram_cas_n = cmd_reg[1];
    assign sdram_we_n  = cmd_reg[0];
    assign sdram_ba    = ba_reg;
    assign sdram_addr  = addr_reg;
    assign sdram_dqm   = dqm_reg;
    
    // �?ẩy tín hiệu ra các cổng thay vì dùng inout nội bộ
    assign sdram_dq_o  = dq_out_reg;
    assign sdram_dq_oe = dq_oe_reg;

    // ==========================================
    // QUẢN L�? HÀNG TRONG C�?C BANK (OPEN-ROW POLICY)
    // ==========================================
    // Lưu trữ thông tin Hàng nào đang được mở ở Bank nào
    reg [SDRAM_ADDR_WIDTH-1:0] open_row_addr [0:3];
    reg                        is_bank_open [0:3];

    // ==========================================
    // THANH GHI LƯU TRỮ GIAO DỊCH AXI (AXI CONTEXT)
    // ==========================================
    reg [ADDR_WIDTH-1:0] current_addr;
    reg [7:0]            burst_len;
    reg [7:0]            burst_count;
    reg [2:0]            burst_size;
    reg                  is_read_transaction;

    // Bóc tách địa chỉ AXI sang địa chỉ SDRAM vật lý (Cho hệ thống 32-bit: [24:23] Bank, [22:10] Row, [9:2] Col)
    wire [SDRAM_BANK_WIDTH-1:0] target_bank = current_addr[24:23];
    wire [SDRAM_ADDR_WIDTH-1:0] target_row  = current_addr[22:10];
    wire [SDRAM_COL_WIDTH-1:0]  target_col  = current_addr[9:2];

    wire row_hit = is_bank_open[target_bank] && (open_row_addr[target_bank] == target_row);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refresh_counter <= 12'd0;
            refresh_request <= 1'b0;
        end else begin
            if (state == ST_INIT_WAIT) begin
                refresh_counter <= 12'd0;
                refresh_request <= 1'b0;
            end else if (refresh_counter >= REFRESH_PERIOD_CYCLES) begin
                refresh_counter <= 12'd0;
                refresh_request <= 1'b1;
            end else begin
                refresh_counter <= refresh_counter + 1'b1;
                if (state == ST_REFRESH && delay_timer == 16'd0) begin
                    refresh_request <= 1'b0;
                end
            end
        end
    end

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_INIT_WAIT;
            delay_timer <= INIT_DELAY_CYCLES;
            
            for (i=0; i<4; i=i+1) begin
                is_bank_open[i] <= 1'b0;
                open_row_addr[i] <= {SDRAM_ADDR_WIDTH{1'b0}};
            end

            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0; s_axi_rlast <= 1'b0;
            
            cmd_reg <= CMD_NOP;
            dq_oe_reg <= 1'b0; dqm_reg <= {(SDRAM_DATA_WIDTH/8){1'b0}};
        end else begin
            // Mặc định cho mỗi chu kỳ
            cmd_reg   <= CMD_NOP;
            dq_oe_reg <= 1'b0;
            dqm_reg   <= {(SDRAM_DATA_WIDTH/8){1'b0}};

            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (s_axi_rvalid && s_axi_rready) begin s_axi_rvalid <= 1'b0; s_axi_rlast <= 1'b0;
            end

            if (delay_timer > 0) begin
                delay_timer <= delay_timer - 1'b1;
            end else begin
                case (state)
                    ST_INIT_WAIT: state <= ST_INIT_PRE;
                    
                    ST_INIT_PRE: begin
                        cmd_reg <= CMD_PRECHARGE;
                        addr_reg[10] <= 1'b1; // Precharge ALL Banks
                        delay_timer <= TRP_CYCLES;
                        state <= ST_INIT_REF1;
                    end
                    
                    ST_INIT_REF1: begin
                        cmd_reg <= CMD_REFRESH;
                        delay_timer <= TRFC_CYCLES;
                        state <= ST_INIT_REF2;
                    end
                    
                    ST_INIT_REF2: begin
                        cmd_reg <= CMD_REFRESH;
                        delay_timer <= TRFC_CYCLES;
                        state <= ST_INIT_LMR;
                    end
                    
                    ST_INIT_LMR: begin
                        cmd_reg <= CMD_LOAD_MODE;
                        // Mode Register: Burst Length = 1, Chế độ Tuần tự, CAS Latency (Ví dụ CAS=3)
                        addr_reg <= {3'b000, 1'b0, 2'b00, 3'b011, 1'b0, 3'b000};
                        delay_timer <= TRP_CYCLES;
                        state <= ST_IDLE;
                    end

                    ST_IDLE: begin
                        if (refresh_request) begin
                            // Phải Precharge tất cả các Bank trước khi Refresh
                            cmd_reg <= CMD_PRECHARGE;
                            addr_reg[10] <= 1'b1; // All banks
                            delay_timer <= TRP_CYCLES - 1;
                            
                            // Xóa toàn bộ trạng thái Open-Row
                            is_bank_open[0]<=0;
                            is_bank_open[1]<=0; is_bank_open[2]<=0; is_bank_open[3]<=0;
                            state <= ST_REFRESH;
                        end 
                        else if (s_axi_awvalid && !s_axi_awready) begin
                            // Chấp nhận Lệnh Ghi
                            s_axi_awready <= 1'b1;
                            current_addr  <= s_axi_awaddr;
                            burst_len     <= s_axi_awlen;
                            burst_size    <= s_axi_awsize;
                            s_axi_bid     <= s_axi_awid;
                            burst_count   <= 8'd0;
                            is_read_transaction <= 1'b0;
                            
                            // Quyết định hướng đi dựa trên Open-Row Policy
                            if (is_bank_open[s_axi_awaddr[24:23]] && (open_row_addr[s_axi_awaddr[24:23]] == s_axi_awaddr[22:10])) begin
                                state <= ST_WRITE_BEAT;
                            end else if (is_bank_open[s_axi_awaddr[24:23]]) begin
                                state <= ST_PRECHARGE;
                            end else begin
                                state <= ST_ACTIVATE;
                            end
                        end 
                        else if (s_axi_arvalid && !s_axi_arready) begin
                            s_axi_arready <= 1'b1;
                            current_addr  <= s_axi_araddr;
                            burst_len     <= s_axi_arlen;
                            burst_size    <= s_axi_arsize;
                            s_axi_rid     <= s_axi_arid;
                            burst_count   <= 8'd0;
                            is_read_transaction <= 1'b1;
                            
                            if (is_bank_open[s_axi_araddr[24:23]] && (open_row_addr[s_axi_araddr[24:23]] == s_axi_araddr[22:10])) begin
                                state <= ST_READ_BEAT;
                            end else if (is_bank_open[s_axi_araddr[24:23]]) begin
                                state <= ST_PRECHARGE;
                            end else begin
                                state <= ST_ACTIVATE;
                            end
                        end
                    end

                    ST_PRECHARGE: begin
                        s_axi_awready <= 1'b0;
                        s_axi_arready <= 1'b0;
                        cmd_reg <= CMD_PRECHARGE;
                        ba_reg <= target_bank;
                        addr_reg[10] <= 1'b0;
                        
                        // Chỉ Precharge Bank mục tiêu
                        is_bank_open[target_bank] <= 1'b0;
                        
                        // Cập nhật trạng thái
                        delay_timer <= TRP_CYCLES - 1;
                        state <= ST_ACTIVATE;
                    end

                    ST_ACTIVATE: begin
                        s_axi_awready <= 1'b0;
                        s_axi_arready <= 1'b0;
                        cmd_reg <= CMD_ACTIVE;
                        ba_reg <= target_bank;
                        addr_reg <= target_row;
                        
                        // Cập nhật trạng thái Row Hit
                        is_bank_open[target_bank] <= 1'b1;
                        open_row_addr[target_bank] <= target_row;
                        
                        delay_timer <= TRCD_CYCLES - 1;
                        if (is_read_transaction) state <= ST_READ_BEAT;
                        else state <= ST_WRITE_BEAT;
                    end

                    // THỰC THI GHI DỮ LIỆU (WRITE BURST)
                    ST_WRITE_BEAT: begin
                        s_axi_awready <= 1'b0;
                        s_axi_wready <= 1'b1;
                        
                        if (s_axi_wvalid && s_axi_wready) begin
                            s_axi_wready <= 1'b0;
                            cmd_reg <= CMD_WRITE;
                            ba_reg <= target_bank;
                            // SDRAM A10 = 0 (B�? qua Auto-Precharge để giữ hàng mở cho beat tiếp theo)
                            addr_reg <= {3'b000, 1'b0, 1'b0, target_col};
                            
                            dq_out_reg <= s_axi_wdata;
                            dqm_reg    <= ~s_axi_wstrb; // AXI WSTRB cao = hợp lệ, SDRAM DQM thấp = hợp lệ
                            dq_oe_reg  <= 1'b1;
                            
                            delay_timer <= TWR_CYCLES - 1;
                            state <= ST_WRITE_WAIT;
                        end
                    end

                    ST_WRITE_WAIT: begin
                        // �?ợi phục hồi ghi (Write Recovery) để tuân thủ th�?i gian an toàn
                        if (burst_count == burst_len) begin
                            // Kết thúc toàn bộ Burst
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= 2'b00; // OKAY
                            state <= ST_IDLE;
                        end else begin
                            // Tiếp tục beat tiếp theo trong Burst
                            burst_count <= burst_count + 1'b1;
                            current_addr <= current_addr + (1 << burst_size);
                            if ((current_addr[9:2] + 1'b1) >= (1 << SDRAM_COL_WIDTH)) begin
                                state <= ST_PRECHARGE;
                            end else begin
                                state <= ST_WRITE_BEAT;
                            end
                        end
                    end

                    ST_READ_BEAT: begin
                        s_axi_arready <= 1'b0;
                        cmd_reg <= CMD_READ;
                        ba_reg <= target_bank;
                        addr_reg <= {3'b000, 1'b0, 1'b0, target_col}; // Auto-precharge = 0
                        
                        delay_timer <= TCAS_CYCLES - 1;
                        state <= ST_READ_WAIT;
                    end

                    ST_READ_WAIT: begin
                        state <= ST_READ_SEND;
                    end

                    ST_READ_SEND: begin
                        s_axi_rvalid <= 1'b1;
                        s_axi_rdata  <= dq_in_wire;
                        s_axi_rresp  <= 2'b00;
                        s_axi_rlast  <= (burst_count == burst_len);
                        
                        if (s_axi_rvalid && s_axi_rready) begin
                            s_axi_rvalid <= 1'b0;
                            s_axi_rlast  <= 1'b0;
                            
                            if (burst_count == burst_len) begin
                                state <= ST_IDLE;
                            end else begin
                                burst_count <= burst_count + 1'b1;
                                current_addr <= current_addr + (1 << burst_size);
                                
                                if ((current_addr[9:2] + 1'b1) >= (1 << SDRAM_COL_WIDTH)) begin
                                    state <= ST_PRECHARGE;
                                end else begin
                                    state <= ST_READ_BEAT;
                                end
                            end
                        end
                    end

                    ST_REFRESH: begin
                        cmd_reg <= CMD_REFRESH;
                        delay_timer <= TRFC_CYCLES - 1;
                        state <= ST_IDLE;
                    end
                endcase
            end
        end
    end

endmodule