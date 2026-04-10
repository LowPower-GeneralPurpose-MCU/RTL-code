`timescale 1ns / 1ps

module dtm_axi_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 5,
    parameter MASTER_ID  = 5'd2
)(
    input wire clk_sys,
    input wire rst_sys_n,

    // Interface from Debug Module (Sys CLK Domain)
    input  wire                  i_req,
    input  wire [1:0]            i_op,       // 1: READ, 2: WRITE
    input  wire [1:0]            i_size,     // 0: 8-bit, 1: 16-bit, 2: 32-bit
    input  wire [ADDR_WIDTH-1:0] i_addr,
    input  wire [DATA_WIDTH-1:0] i_wdata,
    
    output reg                   o_ack,
    output reg  [1:0]            o_resp,
    output reg  [DATA_WIDTH-1:0] o_rdata,

    // Interface to AXI4 Full Main Interconnect
    output wire [ID_WIDTH-1:0]   m_axi_awid,
    output reg  [ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]            m_axi_awlen,
    output reg  [2:0]            m_axi_awsize,
    output wire [1:0]            m_axi_awburst,
    output wire                  m_axi_awlock,
    output wire [3:0]            m_axi_awcache,
    output wire [2:0]            m_axi_awprot,
    output wire [3:0]            m_axi_awqos,
    output wire [3:0]            m_axi_awregion,
    output reg                   m_axi_awvalid,
    input  wire                  m_axi_awready,
    
    output reg  [DATA_WIDTH-1:0] m_axi_wdata,
    output reg  [3:0]            m_axi_wstrb,
    output wire                  m_axi_wlast,
    output reg                   m_axi_wvalid,
    input  wire                  m_axi_wready,
    
    input  wire [ID_WIDTH-1:0]   m_axi_bid,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output reg                   m_axi_bready,

    output wire [ID_WIDTH-1:0]   m_axi_arid,
    output reg  [ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]            m_axi_arlen,
    output reg  [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output wire                  m_axi_arlock,
    output wire [3:0]            m_axi_arcache,
    output wire [2:0]            m_axi_arprot,
    output wire [3:0]            m_axi_arqos,
    output wire [3:0]            m_axi_arregion,
    output reg                   m_axi_arvalid,
    input  wire                  m_axi_arready,
    
    input  wire [ID_WIDTH-1:0]   m_axi_rid,
    input  wire [DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output reg                   m_axi_rready
);

    assign m_axi_awid     = MASTER_ID;
    assign m_axi_awlen    = 8'd0; // 1 beat per burst
    assign m_axi_awburst  = 2'b01; // INCR
    assign m_axi_awlock   = 1'b0;
    assign m_axi_awcache  = 4'b0000;
    assign m_axi_awprot   = 3'b010;
    assign m_axi_awqos    = 4'b0000;
    assign m_axi_awregion = 4'b0000;
    assign m_axi_wlast    = 1'b1;
    
    assign m_axi_arid     = MASTER_ID;
    assign m_axi_arlen    = 8'd0;
    assign m_axi_arburst  = 2'b01;
    assign m_axi_arlock   = 1'b0;
    assign m_axi_arcache  = 4'b0000;
    assign m_axi_arprot   = 3'b010;
    assign m_axi_arqos    = 4'b0000;
    assign m_axi_arregion = 4'b0000;

    localparam ST_IDLE   = 3'b000;
    localparam ST_W_ADDR = 3'b001;
    localparam ST_W_RESP = 3'b010;
    localparam ST_R_ADDR = 3'b011;
    localparam ST_R_DATA = 3'b100;
    localparam ST_ACK    = 3'b101;

    reg [2:0] state;

    // Hàm tạo WSTRB dựa trên kích thước và LSB của địa chỉ
    function [3:0] get_wstrb;
        input [1:0] size;
        input [1:0] addr_lsb;
        begin
            case (size)
                2'd0: get_wstrb = 4'b0001 << addr_lsb;
                2'd1: get_wstrb = 4'b0011 << addr_lsb[1:0];
                2'd2: get_wstrb = 4'b1111;
                default: get_wstrb = 4'b1111;
            endcase
        end
    endfunction

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            state <= ST_IDLE;
            o_ack <= 1'b0; o_resp <= 2'b0; o_rdata <= 32'b0;
            m_axi_awvalid <= 1'b0; m_axi_wvalid  <= 1'b0; m_axi_bready  <= 1'b0;
            m_axi_arvalid <= 1'b0; m_axi_rready  <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    o_ack <= 1'b0;
                    if (i_req) begin
                        if (i_op == 2'd2) begin // WRITE
                            state <= ST_W_ADDR;
                            m_axi_awaddr  <= i_addr;
                            m_axi_awsize  <= {1'b0, i_size};
                            m_axi_awvalid <= 1'b1;
                            m_axi_wdata   <= i_wdata;
                            m_axi_wstrb   <= get_wstrb(i_size, i_addr[1:0]);
                            m_axi_wvalid  <= 1'b1;
                            m_axi_bready  <= 1'b1;
                        end else if (i_op == 2'd1) begin // READ
                            state <= ST_R_ADDR;
                            m_axi_araddr  <= i_addr;
                            m_axi_arsize  <= {1'b0, i_size};
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
                
                // --- HANDSHAKE END ---
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