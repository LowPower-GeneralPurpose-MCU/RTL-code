`timescale 1ns / 1ps

module axi_master_adapter #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- Interface giao tiếp với CPU Cache ---
    input  wire                   cpu_read_req,
    input  wire                   cpu_write_req,
    input  wire [ADDR_WIDTH-1:0]  cpu_addr,
    input  wire [DATA_WIDTH-1:0]  cpu_wdata,
    input  wire [1:0]             cpu_mem_size,  // 00: Word, 01: Half, 10: Byte
    output reg  [DATA_WIDTH-1:0]  cpu_rdata,
    output reg                    cpu_ready,     // Trả về 1 khi rảnh, 0 khi đang kẹt Bus
    output reg                    cpu_resp_val,

    // --- AXI4-Lite Master Interface ---
    output reg  [ADDR_WIDTH-1:0]  m_axi_awaddr,
    output reg  [2:0]             m_axi_awprot,
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
    output reg  [2:0]             m_axi_arprot,
    output reg                    m_axi_arvalid,
    input  wire                   m_axi_arready,
    
    input  wire [DATA_WIDTH-1:0]  m_axi_rdata,
    input  wire [1:0]             m_axi_rresp,
    input  wire                   m_axi_rvalid,
    output reg                    m_axi_rready
);

    // Trạng thái FSM
    localparam ST_IDLE   = 3'b000;
    localparam ST_W_ADDR = 3'b001;
    localparam ST_W_DATA = 3'b010;
    localparam ST_W_RESP = 3'b011;
    localparam ST_R_ADDR = 3'b100;
    localparam ST_R_DATA = 3'b101;

    reg [2:0] state;

    // Logic sinh tín hiệu WSTRB (Byte Enable)
    reg [3:0] wstrb_calc;
    always @(*) begin
        case (cpu_mem_size)
            2'b10: wstrb_calc = 4'b0001 << cpu_addr[1:0]; // Byte
            2'b01: wstrb_calc = 4'b0011 << cpu_addr[1:0]; // Half-word
            default: wstrb_calc = 4'b1111;                // Word
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            cpu_ready     <= 1'b1;
            cpu_resp_val  <= 1'b0;
            cpu_rdata     <= 32'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            m_axi_awprot  <= 3'b000;
            m_axi_arprot  <= 3'b000;
        end else begin
            cpu_resp_val <= 1'b0;
            case (state)
                ST_IDLE: begin
                    cpu_ready <= 1'b1;
                    if (cpu_write_req) begin
                        state         <= ST_W_ADDR;
                        cpu_ready     <= 1'b0; // Stall CPU
                        m_axi_awaddr  <= cpu_addr;
                        m_axi_awvalid <= 1'b1;
                        m_axi_wdata   <= cpu_wdata;
                        m_axi_wstrb   <= wstrb_calc;
                        m_axi_wvalid  <= 1'b1;
                        m_axi_bready  <= 1'b1;
                    end else if (cpu_read_req) begin
                        state         <= ST_R_ADDR;
                        cpu_ready     <= 1'b0; // Stall CPU
                        m_axi_araddr  <= cpu_addr;
                        m_axi_arvalid <= 1'b1;
                        m_axi_rready  <= 1'b1;
                    end
                end

                // --- WRITE FLOW ---
                ST_W_ADDR: begin
                    if (m_axi_awready) m_axi_awvalid <= 1'b0;
                    if (m_axi_wready)  m_axi_wvalid  <= 1'b0;
                    if ((m_axi_awready || !m_axi_awvalid) && (m_axi_wready || !m_axi_wvalid)) begin
                        state <= ST_W_RESP;
                    end
                end

                ST_W_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        cpu_ready    <= 1'b1; // Giải phóng CPU
                        cpu_resp_val <= 1'b1;
                        state        <= ST_IDLE;
                    end
                end

                // --- READ FLOW ---
                ST_R_ADDR: begin
                    if (m_axi_arready) m_axi_arvalid <= 1'b0;
                    if (m_axi_arready || !m_axi_arvalid) begin
                        state <= ST_R_DATA;
                    end
                end

                ST_R_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        cpu_rdata    <= m_axi_rdata;
                        m_axi_rready <= 1'b0;
                        cpu_ready    <= 1'b1; // Giải phóng CPU
                        cpu_resp_val <= 1'b1;
                        state        <= ST_IDLE;
                    end
                end
            endcase
        end
    end
endmodule