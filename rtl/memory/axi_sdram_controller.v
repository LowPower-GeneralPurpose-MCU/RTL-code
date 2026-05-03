`timescale 1ns / 1ps

module axi_sdram_controller #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    // SDRAM Timing Parameters (Tính theo số chu kỳ xung nhịp, giả sử clk = 100MHz -> 10ns)
    parameter INIT_DELAY = 20000, // 200us
    parameter tRP        = 2,     // PRECHARGE command period
    parameter tRCD       = 2,     // ACTIVE to READ/WRITE delay
    parameter tCAS       = 3,     // CAS Latency
    parameter tRFC       = 7,     // AUTO REFRESH period
    parameter tWR        = 2,     // WRITE recovery time
    parameter REFRESH_CYCLES = 780 // Tần suất làm tươi (VD: 7.8us / 10ns = 780)
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // =========================================================================
    // AXI4 SLAVE INTERFACE
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

    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    output reg  [DATA_WIDTH-1:0]  s_axi_rdata,
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready,

    // =========================================================================
    // SDRAM PHYSICAL INTERFACE
    // =========================================================================
    output wire                   sdram_clk,
    output wire                   sdram_cke,
    output wire                   sdram_cs_n,
    output wire                   sdram_ras_n,
    output wire                   sdram_cas_n,
    output wire                   sdram_we_n,
    output wire [1:0]             sdram_ba,    // Bank Address
    output wire [12:0]            sdram_addr,  // Row/Column Address
    output wire [3:0]             sdram_dqm,   // Data Mask
    input  wire [31:0]            sdram_dq_in, // Data Input từ Pad
    output wire [31:0]            sdram_dq_out,// Data Output ra Pad
    output wire                   sdram_dq_oe  // Output Enable (Tri-state control)
);

    // Bật Clock và CKE liên tục (Mặc định cho thiết kế không dùng Low Power mode)
    assign sdram_clk = clk; 
    assign sdram_cke = 1'b1;

    // Các lệnh chuẩn của SDRAM {CS_N, RAS_N, CAS_N, WE_N}
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVE    = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_LOAD_MODE = 4'b0000;

    // FSM States
    localparam ST_INIT_WAIT  = 4'd0;
    localparam ST_INIT_PRE   = 4'd1;
    localparam ST_INIT_REF1  = 4'd2;
    localparam ST_INIT_REF2  = 4'd3;
    localparam ST_INIT_LMR   = 4'd4;
    localparam ST_IDLE       = 4'd5;
    localparam ST_ACTIVATE   = 4'd6;
    localparam ST_READ       = 4'd7;
    localparam ST_READ_WAIT  = 4'd8;
    localparam ST_WRITE      = 4'd9;
    localparam ST_WRITE_WAIT = 4'd10;
    localparam ST_PRECHARGE  = 4'd11;
    localparam ST_REFRESH    = 4'd12;

    reg [3:0]  state, next_state;
    reg [15:0] delay_cnt;
    reg [9:0]  refresh_cnt;
    reg        refresh_req;

    reg [3:0]  sdram_cmd_r;
    reg [12:0] sdram_addr_r;
    reg [1:0]  sdram_ba_r;
    reg [31:0] sdram_dq_out_r;
    reg [3:0]  sdram_dqm_r;
    reg        sdram_dq_oe_r;

    // Các thanh ghi lưu địa chỉ AXI
    reg [ADDR_WIDTH-1:0] current_addr;
    reg [DATA_WIDTH-1:0] current_wdata;
    reg [3:0]            current_wstrb;
    reg is_read_op;

    assign sdram_cs_n  = sdram_cmd_r[3];
    assign sdram_ras_n = sdram_cmd_r[2];
    assign sdram_cas_n = sdram_cmd_r[1];
    assign sdram_we_n  = sdram_cmd_r[0];
    assign sdram_addr  = sdram_addr_r;
    assign sdram_ba    = sdram_ba_r;
    assign sdram_dq_out= sdram_dq_out_r;
    assign sdram_dqm   = sdram_dqm_r;
    assign sdram_dq_oe = sdram_dq_oe_r;

    // Bộ đếm làm tươi (Auto-Refresh Timer)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refresh_cnt <= 0;
            refresh_req <= 1'b0;
        end else begin
            if (state == ST_INIT_WAIT) begin
                refresh_cnt <= 0;
                refresh_req <= 1'b0;
            end else if (refresh_cnt >= REFRESH_CYCLES) begin
                refresh_cnt <= 0;
                refresh_req <= 1'b1;
            end else begin
                refresh_cnt <= refresh_cnt + 1;
                if (state == ST_REFRESH && delay_cnt == 0) refresh_req <= 1'b0;
            end
        end
    end

    // AXI Control Logic & SDRAM FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_INIT_WAIT;
            delay_cnt <= INIT_DELAY;
            sdram_cmd_r <= CMD_NOP;
            sdram_dq_oe_r <= 1'b0;
            
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
        end else begin
            // Mặc định xuất NOP để tránh dội lệnh
            sdram_cmd_r <= CMD_NOP;
            sdram_dq_oe_r <= 1'b0;
            sdram_dqm_r <= 4'b0000;

            if (delay_cnt > 0) begin
                delay_cnt <= delay_cnt - 1;
            end else begin
                case (state)
                    ST_INIT_WAIT: begin
                        state <= ST_INIT_PRE;
                    end
                    ST_INIT_PRE: begin
                        sdram_cmd_r <= CMD_PRECHARGE;
                        sdram_addr_r[10] <= 1'b1; // Precharge All Banks
                        delay_cnt <= tRP;
                        state <= ST_INIT_REF1;
                    end
                    ST_INIT_REF1: begin
                        sdram_cmd_r <= CMD_REFRESH;
                        delay_cnt <= tRFC;
                        state <= ST_INIT_REF2;
                    end
                    ST_INIT_REF2: begin
                        sdram_cmd_r <= CMD_REFRESH;
                        delay_cnt <= tRFC;
                        state <= ST_INIT_LMR;
                    end
                    ST_INIT_LMR: begin
                        sdram_cmd_r <= CMD_LOAD_MODE;
                        // Mode Register: Burst Length = 1, Sequential, CAS = 3
                        sdram_addr_r <= 13'b000_0_00_011_0_000; 
                        delay_cnt <= tRP;
                        state <= ST_IDLE;
                    end
                    
                    ST_IDLE: begin
                        if (refresh_req) begin
                            state <= ST_PRECHARGE;
                            is_read_op <= 1'bx; // Dummy
                        end else if (s_axi_awvalid && !s_axi_awready) begin
                            s_axi_awready <= 1'b1;
                            current_addr <= s_axi_awaddr;
                            is_read_op <= 1'b0;
                            state <= ST_ACTIVATE;
                        end else if (s_axi_arvalid && !s_axi_arready) begin
                            s_axi_arready <= 1'b1;
                            current_addr <= s_axi_araddr;
                            is_read_op <= 1'b1;
                            state <= ST_ACTIVATE;
                        end
                    end

                    ST_ACTIVATE: begin
                        s_axi_awready <= 1'b0;
                        s_axi_arready <= 1'b0;
                        sdram_cmd_r <= CMD_ACTIVE;
                        // Phân rã địa chỉ: [24:23] Bank, [22:10] Row, [9:2] Column (đối với 32-bit word)
                        sdram_ba_r <= current_addr[24:23];
                        sdram_addr_r <= current_addr[22:10]; 
                        delay_cnt <= tRCD - 1;
                        if (is_read_op) state <= ST_READ;
                        else begin
                            s_axi_wready <= 1'b1; // Sẵn sàng nhận data
                            state <= ST_WRITE;
                        end
                    end

                    ST_READ: begin
                        sdram_cmd_r <= CMD_READ;
                        sdram_ba_r <= current_addr[24:23];
                        sdram_addr_r <= {4'b0000, current_addr[9:2], 1'b0}; // Auto-precharge = 0
                        delay_cnt <= tCAS - 1;
                        state <= ST_READ_WAIT;
                    end

                    ST_READ_WAIT: begin
                        s_axi_rvalid <= 1'b1;
                        s_axi_rdata <= sdram_dq_in;
                        s_axi_rresp <= 2'b00;
                        state <= ST_PRECHARGE; // Thực hiện Precharge thủ công sau khi đọc
                    end

                    ST_WRITE: begin
                        if (s_axi_wvalid) begin
                            s_axi_wready <= 1'b0;
                            sdram_cmd_r <= CMD_WRITE;
                            sdram_ba_r <= current_addr[24:23];
                            sdram_addr_r <= {4'b0000, current_addr[9:2], 1'b0};
                            sdram_dq_out_r <= s_axi_wdata;
                            sdram_dqm_r <= ~s_axi_wstrb; // AXI WSTRB tích cực mức cao, DQM tích cực mức thấp
                            sdram_dq_oe_r <= 1'b1;
                            
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp <= 2'b00;
                            delay_cnt <= tWR - 1;
                            state <= ST_WRITE_WAIT;
                        end
                    end

                    ST_WRITE_WAIT: begin
                        // Đợi Write Recovery time
                        state <= ST_PRECHARGE;
                    end

                    ST_PRECHARGE: begin
                        sdram_cmd_r <= CMD_PRECHARGE;
                        sdram_ba_r <= current_addr[24:23];
                        sdram_addr_r[10] <= refresh_req ? 1'b1 : 1'b0; // Nếu là Refresh thì Precharge All
                        delay_cnt <= tRP - 1;
                        if (refresh_req) state <= ST_REFRESH;
                        else state <= ST_IDLE;
                    end

                    ST_REFRESH: begin
                        sdram_cmd_r <= CMD_REFRESH;
                        delay_cnt <= tRFC - 1;
                        state <= ST_IDLE;
                    end
                endcase
            end

            // Xóa cờ Handshake AXI
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end
endmodule