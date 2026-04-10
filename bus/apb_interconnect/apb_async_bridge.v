`timescale 1ns / 1ps

module apb_async_bridge #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    // --- Miền Slave (Nối vào APB Interconnect - clk_apb) ---
    input  wire                   s_clk,
    input  wire                   s_rst_n,
    input  wire                   s_apb_psel,
    input  wire                   s_apb_penable,
    input  wire                   s_apb_pwrite,
    input  wire [ADDR_WIDTH-1:0]  s_apb_paddr,
    input  wire [DATA_WIDTH-1:0]  s_apb_pwdata,
    output reg  [DATA_WIDTH-1:0]  s_apb_prdata,
    output reg                    s_apb_pslverr,
    output reg                    s_apb_pready,

    // --- Miền Master (Nối vào DMA Core - clk_axi) ---
    input  wire                   m_clk,
    input  wire                   m_rst_n,
    output reg                    m_apb_psel,
    output reg                    m_apb_penable,
    output reg                    m_apb_pwrite,
    output reg  [ADDR_WIDTH-1:0]  m_apb_paddr,
    output reg  [DATA_WIDTH-1:0]  m_apb_pwdata,
    input  wire [DATA_WIDTH-1:0]  m_apb_prdata,
    input  wire                   m_apb_pslverr,
    input  wire                   m_apb_pready
);

    // ==========================================
    // 1. FIFO Request (S_CLK -> M_CLK)
    // Dữ liệu: {pwrite, paddr, pwdata}
    // ==========================================
    localparam REQ_WIDTH = 1 + ADDR_WIDTH + DATA_WIDTH;
    wire                 req_full, req_empty;
    reg                  req_wr;
    wire                 req_rd;
    wire [REQ_WIDTH-1:0] req_din = {s_apb_pwrite, s_apb_paddr, s_apb_pwdata};
    wire [REQ_WIDTH-1:0] req_dout;

    cdc_async_fifo_wrapper #(.DATA_WIDTH(REQ_WIDTH), .DEPTH_LOG2(4)) u_req_fifo (
        .wclk(s_clk), .wrst_n(s_rst_n), .wen(req_wr), .wdata(req_din), .wfull(req_full),
        .rclk(m_clk), .rrst_n(m_rst_n), .ren(req_rd), .rdata(req_dout), .rempty(req_empty)
    );

    // ==========================================
    // 2. FIFO Response (M_CLK -> S_CLK)
    // Dữ liệu: {pslverr, prdata}
    // ==========================================
    localparam RESP_WIDTH = 1 + DATA_WIDTH;
    wire                  resp_full, resp_empty;
    reg                   resp_wr;
    wire                  resp_rd;
    reg  [RESP_WIDTH-1:0] resp_din;
    wire [RESP_WIDTH-1:0] resp_dout;

    cdc_async_fifo_wrapper #(.DATA_WIDTH(RESP_WIDTH), .DEPTH_LOG2(4)) u_resp_fifo (
        .wclk(m_clk), .wrst_n(m_rst_n), .wen(resp_wr), .wdata(resp_din), .wfull(resp_full),
        .rclk(s_clk), .rrst_n(s_rst_n), .ren(resp_rd), .rdata(resp_dout), .rempty(resp_empty)
    );

    // ==========================================
    // 3. FSM Miền S_CLK (Nhận lệnh từ SoC)
    // ==========================================
    reg [1:0] s_state;
    localparam S_IDLE = 2'd0, S_WAIT = 2'd1;

    always @(posedge s_clk or negedge s_rst_n) begin
        if (!s_rst_n) begin
            s_state <= S_IDLE;
            req_wr <= 1'b0;
            s_apb_pready <= 1'b0;
            s_apb_prdata <= {DATA_WIDTH{1'b0}};
            s_apb_pslverr <= 1'b0;
        end else begin
            req_wr <= 1'b0;
            s_apb_pready <= 1'b0;

            case (s_state)
                S_IDLE: begin
                    if (s_apb_psel && s_apb_penable && !req_full) begin
                        req_wr <= 1'b1;
                        s_state <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    if (!resp_empty) begin
                        s_apb_pready <= 1'b1;
                        s_apb_prdata <= resp_dout[DATA_WIDTH-1:0];
                        s_apb_pslverr <= resp_dout[DATA_WIDTH];
                        s_state <= S_IDLE;
                    end
                end
            endcase
        end
    end
    assign resp_rd = (s_state == S_WAIT) && !resp_empty;

    // ==========================================
    // 4. FSM Miền M_CLK (Đẩy lệnh vào DMA)
    // ==========================================
    reg [1:0] m_state;
    localparam M_IDLE = 2'd0, M_SETUP = 2'd1, M_ACCESS = 2'd2;

    assign req_rd = (m_state == M_IDLE) && !req_empty && !resp_full;

    always @(posedge m_clk or negedge m_rst_n) begin
        if (!m_rst_n) begin
            m_state <= M_IDLE;
            m_apb_psel <= 1'b0;
            m_apb_penable <= 1'b0;
            m_apb_pwrite <= 1'b0;
            m_apb_paddr <= {ADDR_WIDTH{1'b0}};
            m_apb_pwdata <= {DATA_WIDTH{1'b0}};
            resp_wr <= 1'b0;
            resp_din <= {RESP_WIDTH{1'b0}};
        end else begin
            resp_wr <= 1'b0;
            case (m_state)
                M_IDLE: begin
                    if (!req_empty && !resp_full) begin
                        m_apb_pwrite <= req_dout[REQ_WIDTH-1];
                        m_apb_paddr  <= req_dout[REQ_WIDTH-2 : DATA_WIDTH];
                        m_apb_pwdata <= req_dout[DATA_WIDTH-1 : 0];
                        m_apb_psel   <= 1'b1;
                        m_state      <= M_SETUP;
                    end
                end
                M_SETUP: begin
                    m_apb_penable <= 1'b1;
                    m_state <= M_ACCESS;
                end
                M_ACCESS: begin
                    if (m_apb_pready) begin
                        m_apb_psel <= 1'b0;
                        m_apb_penable <= 1'b0;
                        resp_din <= {m_apb_pslverr, m_apb_prdata};
                        resp_wr <= 1'b1;
                        m_state <= M_IDLE;
                    end
                end
            endcase
        end
    end

endmodule