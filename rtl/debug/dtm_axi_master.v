`timescale 1ns / 1ps

module dtm_axi_master #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    input wire clk_sys,
    input wire rst_sys_n,

    // Interface from Debug Module (Sys CLK Domain)
    input  wire                  i_req,
    input  wire [1:0]            i_op,       // 1: READ, 2: WRITE
    input  wire [ADDR_WIDTH-1:0] i_addr,
    input  wire [DATA_WIDTH-1:0] i_wdata,
    
    output reg                   o_ack,      // Signal back to Debug Module
    output reg  [1:0]            o_resp,     // 0: OK, 2/3: ERROR
    output reg  [DATA_WIDTH-1:0] o_rdata,

    // Interface to AXI4-Lite Main Interconnect
    output reg  [ADDR_WIDTH-1:0] m_axi_awaddr,
    output reg  [2:0]            m_axi_awprot,
    output reg                   m_axi_awvalid,
    input  wire                  m_axi_awready,
    
    output reg  [DATA_WIDTH-1:0] m_axi_wdata,
    output reg  [3:0]            m_axi_wstrb,
    output reg                   m_axi_wvalid,
    input  wire                  m_axi_wready,
    
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output reg                   m_axi_bready,

    output reg  [ADDR_WIDTH-1:0] m_axi_araddr,
    output reg  [2:0]            m_axi_arprot,
    output reg                   m_axi_arvalid,
    input  wire                  m_axi_arready,
    
    input  wire [DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rvalid,
    output reg                   m_axi_rready
);

    localparam ST_IDLE   = 3'b000;
    localparam ST_W_ADDR = 3'b001;
    localparam ST_W_RESP = 3'b010;
    localparam ST_R_ADDR = 3'b011;
    localparam ST_R_DATA = 3'b100;
    localparam ST_ACK    = 3'b101;

    reg [2:0] state;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            state <= ST_IDLE;
            o_ack <= 1'b0;
            o_resp <= 2'b0;
            o_rdata <= 32'b0;
            
            m_axi_awvalid <= 1'b0; m_axi_wvalid  <= 1'b0; m_axi_bready  <= 1'b0;
            m_axi_arvalid <= 1'b0; m_axi_rready  <= 1'b0;
            m_axi_awprot  <= 3'b010;
            m_axi_arprot  <= 3'b010;
        end else begin
            case (state)
                ST_IDLE: begin
                    o_ack <= 1'b0;
                    if (i_req) begin
                        if (i_op == 2'd2) begin // WRITE
                            state <= ST_W_ADDR;
                            m_axi_awaddr  <= i_addr;
                            m_axi_awvalid <= 1'b1;
                            m_axi_wdata   <= i_wdata;
                            m_axi_wstrb   <= 4'b1111;
                            m_axi_wvalid  <= 1'b1;
                            m_axi_bready  <= 1'b1;
                        end else if (i_op == 2'd1) begin // READ
                            state <= ST_R_ADDR;
                            m_axi_araddr  <= i_addr;
                            m_axi_arvalid <= 1'b1;
                            m_axi_rready  <= 1'b1;
                        end
                    end
                end

                // --- WRITE AXI ---
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
                        o_resp       <= m_axi_bresp;
                        o_ack        <= 1'b1;
                        state        <= ST_ACK;
                    end
                end

                // --- READ AXI ---
                ST_R_ADDR: begin
                    if (m_axi_arready) m_axi_arvalid <= 1'b0;
                    if (m_axi_arready || !m_axi_arvalid) begin
                        state <= ST_R_DATA;
                    end
                end

                ST_R_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        m_axi_rready <= 1'b0;
                        o_rdata      <= m_axi_rdata;
                        o_resp       <= m_axi_rresp;
                        o_ack        <= 1'b1;
                        state        <= ST_ACK;
                    end
                end
                
                // --- KẾT THÚC HANDSHAKE ---
                ST_ACK: begin
                    if (!i_req) begin
                        o_ack <= 1'b0;
                        state <= ST_IDLE;
                    end
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule