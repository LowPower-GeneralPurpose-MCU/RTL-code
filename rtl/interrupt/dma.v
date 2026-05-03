`timescale 1ns / 1ps

module axi_dma #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // =========================================================================
    // AXI SLAVE INTERFACE (CPU cấu hình DMA)
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
    // AXI MASTER INTERFACE (DMA thực hiện truyền dữ liệu)
    // =========================================================================
    output reg  [ADDR_WIDTH-1:0]  m_axi_awaddr,
    output reg                    m_axi_awvalid,
    input  wire                   m_axi_awready,
    output reg  [DATA_WIDTH-1:0]  m_axi_wdata,
    output reg  [3:0]             m_axi_wstrb,
    output reg                    m_axi_wvalid,
    input  wire                   m_axi_wready,
    input  wire [1:0]             m_axi_bresp,
    input  wire                   m_axi_bvalid,
    output reg                    m_axi_bready,
    
    output reg  [ADDR_WIDTH-1:0]  m_axi_araddr,
    output reg                    m_axi_arvalid,
    input  wire                   m_axi_arready,
    input  wire [DATA_WIDTH-1:0]  m_axi_rdata,
    input  wire [1:0]             m_axi_rresp,
    input  wire                   m_axi_rvalid,
    output reg                    m_axi_rready,

    // =========================================================================
    // INTERRUPT TỚI PLIC
    // =========================================================================
    output wire                   dma_irq
);

    // -------------------------------------------------------------------------
    // 1. REGISTER MAP (Slave Logic)
    // -------------------------------------------------------------------------
    // 0x00: CSR  (Control & Status Register)
    //       [0]   : DMA_EN (Write 1 to start, auto clears when done)
    //       [1]   : DMA_DONE (Read only, cleared when DMA_EN is set)
    //       [2]   : INT_EN (Interrupt Enable)
    // 0x04: SAR  (Source Address Register)
    // 0x08: DAR  (Destination Address Register)
    // 0x0C: LEN  (Length in Bytes - must be word aligned)
    
    reg [31:0] reg_csr;
    reg [31:0] reg_sar;
    reg [31:0] reg_dar;
    reg [31:0] reg_len;

    // Các cờ điều khiển nội bộ
    wire dma_start = reg_csr[0];
    wire int_en    = reg_csr[2];
    reg  dma_done_flag;

    assign dma_irq = dma_done_flag & int_en;

    // --- Slave Write Logic ---
    reg [ADDR_WIDTH-1:0] awaddr_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            
            reg_csr <= 32'h0;
            reg_sar <= 32'h0;
            reg_dar <= 32'h0;
            reg_len <= 32'h0;
        end else begin
            // Address Phase
            if (s_axi_awvalid && !s_axi_awready) begin
                s_axi_awready <= 1'b1;
                awaddr_reg    <= s_axi_awaddr;
            end else begin
                s_axi_awready <= 1'b0;
            end

            // Data Phase
            if (s_axi_wvalid && s_axi_awready && !s_axi_wready) begin
                s_axi_wready <= 1'b1;
                case (awaddr_reg[7:0])
                    8'h00: begin 
                        if (s_axi_wstrb[0]) reg_csr[0] <= s_axi_wdata[0];
                        if (s_axi_wstrb[0]) reg_csr[2] <= s_axi_wdata[2];
                        // Không ghi vào cờ DONE từ bus
                    end
                    8'h04: reg_sar <= s_axi_wdata;
                    8'h08: reg_dar <= s_axi_wdata;
                    8'h0C: reg_len <= s_axi_wdata;
                endcase
            end else begin
                s_axi_wready <= 1'b0;
            end

            // Xóa cờ DMA_EN khi hoàn thành (Logic FSM điều khiển)
            if (dma_done_flag) begin
                reg_csr[0] <= 1'b0; // Tự động xóa Enable
                reg_csr[1] <= 1'b1; // Bật cờ Done
            end else if (s_axi_wvalid && s_axi_wready && awaddr_reg[7:0] == 8'h00 && s_axi_wdata[0]) begin
                reg_csr[1] <= 1'b0; // Xóa cờ Done khi bắt đầu transfer mới
            end

            // Response Phase
            if (s_axi_wvalid && s_axi_wready) begin
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // --- Slave Read Logic ---
    reg [ADDR_WIDTH-1:0] araddr_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'h0;
            s_axi_rresp   <= 2'b00;
        end else begin
            if (s_axi_arvalid && !s_axi_arready) begin
                s_axi_arready <= 1'b1;
                araddr_reg    <= s_axi_araddr;
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_arready && s_axi_arvalid) begin
                s_axi_rvalid <= 1'b1;
                case (araddr_reg[7:0])
                    8'h00: s_axi_rdata <= reg_csr;
                    8'h04: s_axi_rdata <= reg_sar;
                    8'h08: s_axi_rdata <= reg_dar;
                    8'h0C: s_axi_rdata <= reg_len;
                    default: s_axi_rdata <= 32'h0;
                endcase
            end else if (s_axi_rready && s_axi_rvalid) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 2. DMA MASTER FSM (Master Logic)
    // -------------------------------------------------------------------------
    localparam ST_IDLE       = 3'd0;
    localparam ST_READ_ADDR  = 3'd1;
    localparam ST_READ_DATA  = 3'd2;
    localparam ST_WRITE_ADDR = 3'd3;
    localparam ST_WRITE_DATA = 3'd4;
    localparam ST_WRITE_RESP = 3'd5;
    localparam ST_DONE       = 3'd6;

    reg [2:0]  state;
    reg [31:0] current_src;
    reg [31:0] current_dst;
    reg [31:0] bytes_left;
    reg [31:0] data_buffer; // Bộ đệm tạm thời chứa dữ liệu đọc được

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;
            m_axi_wstrb   <= 4'b1111;
            dma_done_flag <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    dma_done_flag <= 1'b0;
                    if (dma_start) begin
                        current_src <= reg_sar;
                        current_dst <= reg_dar;
                        bytes_left  <= reg_len;
                        if (reg_len > 0) state <= ST_READ_ADDR;
                        else             state <= ST_DONE;
                    end
                end

                ST_READ_ADDR: begin
                    m_axi_arvalid <= 1'b1;
                    m_axi_araddr  <= current_src;
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        state         <= ST_READ_DATA;
                    end
                end

                ST_READ_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        data_buffer  <= m_axi_rdata;
                        m_axi_rready <= 1'b0;
                        state        <= ST_WRITE_ADDR;
                    end
                end

                ST_WRITE_ADDR: begin
                    m_axi_awvalid <= 1'b1;
                    m_axi_awaddr  <= current_dst;
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b1;
                        m_axi_wdata   <= data_buffer;
                        state         <= ST_WRITE_DATA;
                    end
                end

                ST_WRITE_DATA: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_bready <= 1'b1;
                        state        <= ST_WRITE_RESP;
                    end
                end

                ST_WRITE_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        // Cập nhật con trỏ và số byte còn lại
                        current_src <= current_src + 4;
                        current_dst <= current_dst + 4;
                        bytes_left  <= bytes_left - 4;
                        
                        if (bytes_left <= 4) state <= ST_DONE;
                        else                 state <= ST_READ_ADDR;
                    end
                end

                ST_DONE: begin
                    dma_done_flag <= 1'b1; // Báo hiệu đã xong
                    state         <= ST_IDLE;
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule