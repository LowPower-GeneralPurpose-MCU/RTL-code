`timescale 1ns / 1ps

module axi_to_apb_bridge #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 5
)(
    // ==========================================
    // CLOCK & RESET (Đã tách thành 2 miền)
    // ==========================================
    input  wire                     clk_axi,
    input  wire                     clk_apb,
    input  wire                     rst_axi_n,
    input  wire                     rst_apb_n,

    // ==========================================
    // AXI4 FULL SLAVE INTERFACE (Miền clk_axi)
    // ==========================================
    // Write Address Channel
    input  wire [ID_WIDTH-1:0]      s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire [7:0]               s_axi_awlen,
    input  wire [2:0]               s_axi_awsize,
    input  wire [1:0]               s_axi_awburst,
    input  wire [2:0]               s_axi_awprot,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,

    // Write Data Channel
    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]s_axi_wstrb,
    input  wire                     s_axi_wlast,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    // Write Response Channel
    output reg  [ID_WIDTH-1:0]      s_axi_bid,
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    // Read Address Channel
    input  wire [ID_WIDTH-1:0]      s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire [7:0]               s_axi_arlen,
    input  wire [2:0]               s_axi_arsize,
    input  wire [1:0]               s_axi_arburst,
    input  wire [2:0]               s_axi_arprot,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    // Read Data Channel
    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output reg  [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready,

    // ==========================================
    // APB4 MASTER INTERFACE (Miền clk_apb)
    // ==========================================
    output reg  [ADDR_WIDTH-1:0]    m_apb_paddr,
    output reg  [2:0]               m_apb_pprot,
    output reg                      m_apb_psel,
    output reg                      m_apb_penable,
    output reg                      m_apb_pwrite,
    output reg  [DATA_WIDTH-1:0]    m_apb_pwdata,
    output reg  [(DATA_WIDTH/8)-1:0]m_apb_pstrb,
    input  wire                     m_apb_pready,
    input  wire [DATA_WIDTH-1:0]    m_apb_prdata,
    input  wire                     m_apb_pslverr
);

    // ==========================================
    // KHAI BÁO TÍN HIỆU NỘI BỘ VÀ TRẠNG THÁI
    // ==========================================
    // Write FSM States
    localparam W_IDLE    = 3'd0;
    localparam W_DATA    = 3'd1;
    localparam W_ARB_REQ = 3'd2;
    localparam W_RESP    = 3'd3;

    // Read FSM States
    localparam R_IDLE    = 3'd0;
    localparam R_ARB_REQ = 3'd1;
    localparam R_DATA    = 3'd2;

    // Write Context Registers
    reg [2:0]            w_state;
    reg [ID_WIDTH-1:0]   w_id_reg;
    reg [ADDR_WIDTH-1:0] w_addr_reg;
    reg [7:0]            w_len_reg;
    reg [2:0]            w_size_reg;
    reg [2:0]            w_prot_reg;
    reg [7:0]            w_beat_cnt;
    reg [1:0]            w_err_accum;
    reg [DATA_WIDTH-1:0] w_data_reg;
    reg [(DATA_WIDTH/8)-1:0] w_strb_reg;

    // Read Context Registers
    reg [2:0]            r_state;
    reg [ID_WIDTH-1:0]   r_id_reg;
    reg [ADDR_WIDTH-1:0] r_addr_reg;
    reg [7:0]            r_len_reg;
    reg [2:0]            r_size_reg;
    reg [2:0]            r_prot_reg;
    reg [7:0]            r_beat_cnt;

    // Arbiter Signals
    reg  req_w;
    reg  ack_w; // Đổi thành reg vì do CDC Arbiter cấp
    reg  req_r;
    reg  ack_r; // Đổi thành reg vì do CDC Arbiter cấp

    // Tín hiệu dữ liệu đã qua cầu CDC trả về miền AXI
    wire [DATA_WIDTH-1:0] cross_prdata;
    wire                  cross_pslverr;

    // ==========================================
    // 1. WRITE FSM (Miền clk_axi - GIỮ NGUYÊN BẢN GỐC)
    // ==========================================
    always @(posedge clk_axi or negedge rst_axi_n) begin
        if (!rst_axi_n) begin
            w_state       <= W_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            s_axi_bid     <= {ID_WIDTH{1'b0}};
            req_w         <= 1'b0;
            w_beat_cnt    <= 8'd0;
            w_err_accum   <= 2'b00;
        end else begin
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            case (w_state)
                W_IDLE: begin
                    s_axi_awready <= 1'b1;
                    if (s_axi_awvalid && s_axi_awready) begin
                        s_axi_awready <= 1'b0;
                        w_id_reg      <= s_axi_awid;
                        w_addr_reg    <= s_axi_awaddr;
                        w_len_reg     <= s_axi_awlen;
                        w_size_reg    <= s_axi_awsize;
                        w_prot_reg    <= s_axi_awprot;
                        w_beat_cnt    <= 8'd0;
                        w_err_accum   <= 2'b00;
                        w_state       <= W_DATA;
                    end
                end

                W_DATA: begin
                    s_axi_wready <= 1'b1;
                    if (s_axi_wvalid && s_axi_wready) begin
                        s_axi_wready <= 1'b0;
                        w_data_reg   <= s_axi_wdata;
                        w_strb_reg   <= s_axi_wstrb;
                        req_w        <= 1'b1;
                        w_state      <= W_ARB_REQ;
                    end
                end

                W_ARB_REQ: begin
                    if (ack_w) begin
                        req_w <= 1'b0;
                        // Lấy lỗi từ CDC thay vì m_apb_pslverr trực tiếp
                        if (cross_pslverr) w_err_accum <= 2'b10;
                        
                        if (w_beat_cnt == w_len_reg) begin
                            w_state <= W_RESP;
                        end else begin
                            w_beat_cnt <= w_beat_cnt + 1'b1;
                            w_addr_reg <= w_addr_reg + (1 << w_size_reg);
                            w_state    <= W_DATA;
                        end
                    end
                end

                W_RESP: begin
                    if (!s_axi_bvalid) begin
                        s_axi_bvalid <= 1'b1;
                        s_axi_bid    <= w_id_reg;
                        s_axi_bresp  <= w_err_accum;
                        w_state      <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // ==========================================
    // 2. READ FSM (Miền clk_axi - GIỮ NGUYÊN BẢN GỐC)
    // ==========================================
    always @(posedge clk_axi or negedge rst_axi_n) begin
        if (!rst_axi_n) begin
            r_state       <= R_IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rlast   <= 1'b0;
            s_axi_rid     <= {ID_WIDTH{1'b0}};
            s_axi_rdata   <= {DATA_WIDTH{1'b0}};
            req_r         <= 1'b0;
            r_beat_cnt    <= 8'd0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    s_axi_arready <= 1'b1;
                    if (s_axi_arvalid && s_axi_arready) begin
                        s_axi_arready <= 1'b0;
                        r_id_reg      <= s_axi_arid;
                        r_addr_reg    <= s_axi_araddr;
                        r_len_reg     <= s_axi_arlen;
                        r_size_reg    <= s_axi_arsize;
                        r_prot_reg    <= s_axi_arprot;
                        r_beat_cnt    <= 8'd0;
                        req_r         <= 1'b1;
                        r_state       <= R_ARB_REQ;
                    end
                end

                R_ARB_REQ: begin
                    if (ack_r) begin
                        req_r        <= 1'b0;
                        s_axi_rvalid <= 1'b1;
                        s_axi_rid    <= r_id_reg;
                        // Lấy data từ CDC thay vì m_apb_prdata trực tiếp
                        s_axi_rdata  <= cross_prdata;
                        s_axi_rresp  <= cross_pslverr ? 2'b10 : 2'b00;
                        s_axi_rlast  <= (r_beat_cnt == r_len_reg);
                        r_state      <= R_DATA;
                    end
                end

                R_DATA: begin
                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        s_axi_rlast  <= 1'b0;
                        if (r_beat_cnt == r_len_reg) begin
                            r_state <= R_IDLE;
                        end else begin
                            r_beat_cnt <= r_beat_cnt + 1'b1;
                            r_addr_reg <= r_addr_reg + (1 << r_size_reg);
                            req_r      <= 1'b1;
                            r_state    <= R_ARB_REQ;
                        end
                    end
                end
            endcase
        end
    end

    // ==========================================
    // 3. ASYNC FIFOS (Cầu nối CDC)
    // ==========================================
    // Width = 1(Wr/Rd) + 32(Addr) + 32(Data) + 4(Strb) + 3(Prot) = 72 bits
    wire        cmd_fifo_full, cmd_fifo_empty;
    reg         cmd_fifo_wr;
    wire        cmd_fifo_rd;
    wire [71:0] cmd_fifo_wdata;
    wire [71:0] cmd_fifo_rdata;

    cdc_async_fifo_wrapper #(.DATA_WIDTH(72), .DEPTH_LOG2(4)) u_cmd_fifo (
        .wclk(clk_axi), .wrst_n(rst_axi_n), .wen(cmd_fifo_wr), .wdata(cmd_fifo_wdata), .wfull(cmd_fifo_full),
        .rclk(clk_apb), .rrst_n(rst_apb_n), .ren(cmd_fifo_rd), .rdata(cmd_fifo_rdata), .rempty(cmd_fifo_empty)
    );

    // Width = 1(Err) + 32(Data) = 33 bits
    wire        resp_fifo_full, resp_fifo_empty;
    reg         resp_fifo_wr;
    reg         resp_fifo_rd;
    reg  [32:0] resp_fifo_wdata;
    wire [32:0] resp_fifo_rdata;

    cdc_async_fifo_wrapper #(.DATA_WIDTH(33), .DEPTH_LOG2(4)) u_resp_fifo (
        .wclk(clk_apb), .wrst_n(rst_apb_n), .wen(resp_fifo_wr), .wdata(resp_fifo_wdata), .wfull(resp_fifo_full),
        .rclk(clk_axi), .rrst_n(rst_axi_n), .ren(resp_fifo_rd), .rdata(resp_fifo_rdata), .rempty(resp_fifo_empty)
    );

    // ==========================================
    // 4. AXI ARBITER (Đẩy vào FIFO - Miền clk_axi)
    // ==========================================
    localparam ARB_IDLE = 2'd0, ARB_PUSH = 2'd1, ARB_WAIT = 2'd2;
    reg [1:0] arb_state;
    reg       apb_grant_read;

    // Lắp ráp dữ liệu Command đẩy qua FIFO
    assign cmd_fifo_wdata = apb_grant_read ? 
                            {1'b0, r_addr_reg, 32'b0, 4'b0, r_prot_reg} :
                            {1'b1, w_addr_reg, w_data_reg, w_strb_reg, w_prot_reg};

    // Giải nén dữ liệu Response từ FIFO cho Read/Write FSM
    assign cross_prdata  = resp_fifo_rdata[31:0];
    assign cross_pslverr = resp_fifo_rdata[32];

    always @(posedge clk_axi or negedge rst_axi_n) begin
        if (!rst_axi_n) begin
            arb_state <= ARB_IDLE;
            cmd_fifo_wr <= 0; resp_fifo_rd <= 0;
            ack_w <= 0; ack_r <= 0;
            apb_grant_read <= 0;
        end else begin
            cmd_fifo_wr <= 0; resp_fifo_rd <= 0;
            ack_w <= 0; ack_r <= 0;

            case (arb_state)
                ARB_IDLE: begin
                    // Ưu tiên Read > Write giống hệt code gốc của bạn
                    if (!ack_r && !ack_w) begin
                        // Avoid replaying a command while the source FSM is dropping req_*.
                        if (req_r) begin
                            apb_grant_read <= 1'b1;
                            arb_state <= ARB_PUSH;
                        end else if (req_w) begin
                            apb_grant_read <= 1'b0;
                            arb_state <= ARB_PUSH;
                        end
                    end
                end

                ARB_PUSH: begin
                    if (!cmd_fifo_full) begin
                        cmd_fifo_wr <= 1'b1;
                        arb_state <= ARB_WAIT;
                    end
                end

                ARB_WAIT: begin
                    // Chờ APB xử lý xong và trả kết quả về resp_fifo
                    if (!resp_fifo_empty) begin
                        resp_fifo_rd <= 1'b1;
                        if (apb_grant_read) ack_r <= 1'b1; // Kích hoạt ack cho Read FSM
                        else                ack_w <= 1'b1; // Kích hoạt ack cho Write FSM
                        arb_state <= ARB_IDLE;
                    end
                end
            endcase
        end
    end

    // ==========================================
    // 5. APB MASTER FSM (Xử lý thực thi - Miền clk_apb)
    // ==========================================
    reg [1:0] apb_fsm;
    localparam P_IDLE = 2'd0, P_SETUP = 2'd1, P_ACCESS = 2'd2;

    assign cmd_fifo_rd = (apb_fsm == P_IDLE) && !cmd_fifo_empty && !resp_fifo_full;

    always @(posedge clk_apb or negedge rst_apb_n) begin
        if (!rst_apb_n) begin
            apb_fsm <= P_IDLE;
            m_apb_psel <= 0; m_apb_penable <= 0; m_apb_pwrite <= 0;
            m_apb_paddr <= 0; m_apb_pwdata <= 0; m_apb_pstrb <= 0; m_apb_pprot <= 0;
            resp_fifo_wr <= 0; resp_fifo_wdata <= 0;
        end else begin
            resp_fifo_wr <= 0;
            case (apb_fsm)
                P_IDLE: begin
                    if (cmd_fifo_rd) begin
                        m_apb_pwrite <= cmd_fifo_rdata[71];
                        m_apb_paddr  <= cmd_fifo_rdata[70:39];
                        m_apb_pwdata <= cmd_fifo_rdata[38:7];
                        m_apb_pstrb  <= cmd_fifo_rdata[6:3];
                        m_apb_pprot  <= cmd_fifo_rdata[2:0];
                        m_apb_psel   <= 1'b1;
                        apb_fsm      <= P_SETUP;
                    end
                end
                
                P_SETUP: begin
                    m_apb_penable <= 1'b1;
                    apb_fsm <= P_ACCESS;
                end
                
                P_ACCESS: begin
                    if (m_apb_pready) begin
                        m_apb_psel    <= 1'b0; 
                        m_apb_penable <= 1'b0;
                        resp_fifo_wdata <= {m_apb_pslverr, m_apb_prdata};
                        resp_fifo_wr  <= 1'b1;
                        apb_fsm       <= P_IDLE;
                    end
                end
            endcase
        end
    end

endmodule
