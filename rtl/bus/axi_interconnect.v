`timescale 1ns / 1ps

/**
 * MODULE: AXI INTERCONNECT (4 Master - 8 Slave)
 * -------------------------------------------
 * MASTERS:
 * - M0: Instruction Cache (Read-Only, Hỗ trợ Burst)
 * - M1: Data Cache (Read/Write, Single-beat)
 * - M2: Debug Module SBA (Read/Write, Single-beat)
 * - M3: DMA Controller (Read/Write, Single-beat)
 * * SLAVES & ADDRESS MAP:
 * - S0: Boot ROM            - 32'h0000_1000 đến 32'h0000_4FFF (Read-Only)
 * - S1: On-chip SRAM        - 32'h8000_0000 đến 32'h8000_FFFF (Bao gồm Text/Data/Stack)
 * - S2: APB Peripherals     - 32'h4000_0000 đến 32'h4FFF_FFFF
 * - S3: SPI Flash           - 32'h2000_0000 đến 32'h2FFF_FFFF (Read-Only)
 * - S4: CLINT               - 32'h0200_0000 đến 32'h020B_FFFF
 * - S5: PLIC                - 32'h0C00_0000 đến 32'h0FFF_FFFF
 * - S6: Off-chip SDRAM      - 32'hA000_0000 đến 32'hAFFF_FFFF
 * - S7: DMA Config Slave    - 32'h5000_0000 đến 32'h5000_0FFF (Nơi CPU cấu hình DMA)
 */

module axi_interconnect #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst_n,

    // =========================================================================
    // MASTER INTERFACES
    // =========================================================================
    // Master 0: Instruction Cache
    input  wire [ADDR_WIDTH-1:0]  m0_araddr,
    input  wire [7:0]             m0_arlen,
    input  wire [2:0]             m0_arsize,
    input  wire [1:0]             m0_arburst,
    input  wire                   m0_arvalid,
    output reg                    m0_arready,
    output reg  [DATA_WIDTH-1:0]  m0_rdata,
    output reg  [1:0]             m0_rresp,
    output reg                    m0_rlast,
    output reg                    m0_rvalid,
    input  wire                   m0_rready,

    // Master 1: Data Cache
    input  wire [ADDR_WIDTH-1:0]  m1_awaddr,
    input  wire                   m1_awvalid,
    output reg                    m1_awready,
    input  wire [DATA_WIDTH-1:0]  m1_wdata,
    input  wire [3:0]             m1_wstrb,
    input  wire                   m1_wvalid,
    output reg                    m1_wready,
    output reg  [1:0]             m1_bresp,
    output reg                    m1_bvalid,
    input  wire                   m1_bready,
    input  wire [ADDR_WIDTH-1:0]  m1_araddr,
    input  wire                   m1_arvalid,
    output reg                    m1_arready,
    output reg  [DATA_WIDTH-1:0]  m1_rdata,
    output reg  [1:0]             m1_rresp,
    output reg                    m1_rvalid,
    input  wire                   m1_rready,

    // Master 2: Debug Module
    input  wire [ADDR_WIDTH-1:0]  m2_awaddr,
    input  wire                   m2_awvalid,
    output reg                    m2_awready,
    input  wire [DATA_WIDTH-1:0]  m2_wdata,
    input  wire [3:0]             m2_wstrb,
    input  wire                   m2_wvalid,
    output reg                    m2_wready,
    output reg  [1:0]             m2_bresp,
    output reg                    m2_bvalid,
    input  wire                   m2_bready,
    input  wire [ADDR_WIDTH-1:0]  m2_araddr,
    input  wire                   m2_arvalid,
    output reg                    m2_arready,
    output reg  [DATA_WIDTH-1:0]  m2_rdata,
    output reg  [1:0]             m2_rresp,
    output reg                    m2_rvalid,
    input  wire                   m2_rready,

    // Master 3: DMA Controller (Phần tự động đọc/ghi)
    input  wire [ADDR_WIDTH-1:0]  m3_awaddr,
    input  wire                   m3_awvalid,
    output reg                    m3_awready,
    input  wire [DATA_WIDTH-1:0]  m3_wdata,
    input  wire [3:0]             m3_wstrb,
    input  wire                   m3_wvalid,
    output reg                    m3_wready,
    output reg  [1:0]             m3_bresp,
    output reg                    m3_bvalid,
    input  wire                   m3_bready,
    input  wire [ADDR_WIDTH-1:0]  m3_araddr,
    input  wire                   m3_arvalid,
    output reg                    m3_arready,
    output reg  [DATA_WIDTH-1:0]  m3_rdata,
    output reg  [1:0]             m3_rresp,
    output reg                    m3_rvalid,
    input  wire                   m3_rready,

    // =========================================================================
    // SLAVE INTERFACES
    // =========================================================================
    // Slave 0: ROM (Read-Only)
    output reg  [ADDR_WIDTH-1:0]  s0_araddr,
    output reg                    s0_arvalid,
    input  wire                   s0_arready,
    input  wire [DATA_WIDTH-1:0]  s0_rdata,
    input  wire [1:0]             s0_rresp,
    input  wire                   s0_rvalid,
    output reg                    s0_rready,

    // Slave 1: On-chip SRAM (Hỗ trợ Burst cho M0)
    output reg  [ADDR_WIDTH-1:0]  s1_awaddr,
    output reg                    s1_awvalid,
    input  wire                   s1_awready,
    output reg  [DATA_WIDTH-1:0]  s1_wdata,
    output reg  [3:0]             s1_wstrb,
    output reg                    s1_wvalid,
    input  wire                   s1_wready,
    input  wire [1:0]             s1_bresp,
    input  wire                   s1_bvalid,
    output reg                    s1_bready,
    output reg  [ADDR_WIDTH-1:0]  s1_araddr,
    output reg  [7:0]             s1_arlen,
    output reg  [2:0]             s1_arsize,
    output reg  [1:0]             s1_arburst,
    output reg                    s1_arvalid,
    input  wire                   s1_arready,
    input  wire [DATA_WIDTH-1:0]  s1_rdata,
    input  wire [1:0]             s1_rresp,
    input  wire                   s1_rlast,
    input  wire                   s1_rvalid,
    output reg                    s1_rready,

    // Slave 2: APB Peripherals
    output reg  [ADDR_WIDTH-1:0]  s2_awaddr,
    output reg                    s2_awvalid,
    input  wire                   s2_awready,
    output reg  [DATA_WIDTH-1:0]  s2_wdata,
    output reg  [3:0]             s2_wstrb,
    output reg                    s2_wvalid,
    input  wire                   s2_wready,
    input  wire [1:0]             s2_bresp,
    input  wire                   s2_bvalid,
    output reg                    s2_bready,
    output reg  [ADDR_WIDTH-1:0]  s2_araddr,
    output reg                    s2_arvalid,
    input  wire                   s2_arready,
    input  wire [DATA_WIDTH-1:0]  s2_rdata,
    input  wire [1:0]             s2_rresp,
    input  wire                   s2_rvalid,
    output reg                    s2_rready,

    // Slave 3: SPI Flash (Read-Only)
    output reg  [ADDR_WIDTH-1:0]  s3_araddr,
    output reg                    s3_arvalid,
    input  wire                   s3_arready,
    input  wire [DATA_WIDTH-1:0]  s3_rdata,
    input  wire [1:0]             s3_rresp,
    input  wire                   s3_rvalid,
    output reg                    s3_rready,

    // Slave 4: CLINT
    output reg  [ADDR_WIDTH-1:0]  s4_awaddr,
    output reg                    s4_awvalid,
    input  wire                   s4_awready,
    output reg  [DATA_WIDTH-1:0]  s4_wdata,
    output reg  [3:0]             s4_wstrb,
    output reg                    s4_wvalid,
    input  wire                   s4_wready,
    input  wire [1:0]             s4_bresp,
    input  wire                   s4_bvalid,
    output reg                    s4_bready,
    output reg  [ADDR_WIDTH-1:0]  s4_araddr,
    output reg                    s4_arvalid,
    input  wire                   s4_arready,
    input  wire [DATA_WIDTH-1:0]  s4_rdata,
    input  wire [1:0]             s4_rresp,
    input  wire                   s4_rvalid,
    output reg                    s4_rready,

    // Slave 5: PLIC
    output reg  [ADDR_WIDTH-1:0]  s5_awaddr,
    output reg                    s5_awvalid,
    input  wire                   s5_awready,
    output reg  [DATA_WIDTH-1:0]  s5_wdata,
    output reg  [3:0]             s5_wstrb,
    output reg                    s5_wvalid,
    input  wire                   s5_wready,
    input  wire [1:0]             s5_bresp,
    input  wire                   s5_bvalid,
    output reg                    s5_bready,
    output reg  [ADDR_WIDTH-1:0]  s5_araddr,
    output reg                    s5_arvalid,
    input  wire                   s5_arready,
    input  wire [DATA_WIDTH-1:0]  s5_rdata,
    input  wire [1:0]             s5_rresp,
    input  wire                   s5_rvalid,
    output reg                    s5_rready,

    // Slave 6: Off-chip SDRAM (Hỗ trợ Burst cho M0)
    output reg  [ADDR_WIDTH-1:0]  s6_awaddr,
    output reg                    s6_awvalid,
    input  wire                   s6_awready,
    output reg  [DATA_WIDTH-1:0]  s6_wdata,
    output reg  [3:0]             s6_wstrb,
    output reg                    s6_wvalid,
    input  wire                   s6_wready,
    input  wire [1:0]             s6_bresp,
    input  wire                   s6_bvalid,
    output reg                    s6_bready,
    output reg  [ADDR_WIDTH-1:0]  s6_araddr,
    output reg  [7:0]             s6_arlen,
    output reg  [2:0]             s6_arsize,
    output reg  [1:0]             s6_arburst,
    output reg                    s6_arvalid,
    input  wire                   s6_arready,
    input  wire [DATA_WIDTH-1:0]  s6_rdata,
    input  wire [1:0]             s6_rresp,
    input  wire                   s6_rlast,
    input  wire                   s6_rvalid,
    output reg                    s6_rready,

    // Slave 7: DMA Config Slave
    output reg  [ADDR_WIDTH-1:0]  s7_awaddr,
    output reg                    s7_awvalid,
    input  wire                   s7_awready,
    output reg  [DATA_WIDTH-1:0]  s7_wdata,
    output reg  [3:0]             s7_wstrb,
    output reg                    s7_wvalid,
    input  wire                   s7_wready,
    input  wire [1:0]             s7_bresp,
    input  wire                   s7_bvalid,
    output reg                    s7_bready,
    output reg  [ADDR_WIDTH-1:0]  s7_araddr,
    output reg                    s7_arvalid,
    input  wire                   s7_arready,
    input  wire [DATA_WIDTH-1:0]  s7_rdata,
    input  wire [1:0]             s7_rresp,
    input  wire                   s7_rvalid,
    output reg                    s7_rready
);

    // =========================================================================
    // ADDRESS DECODER
    // =========================================================================
    function [3:0] decode_addr;
        input [ADDR_WIDTH-1:0] addr;
        begin
            if      (addr >= 32'h0000_1000 && addr <= 32'h0000_4FFF) decode_addr = 4'd0; // S0: ROM
            else if (addr >= 32'h8000_0000 && addr <= 32'h8000_FFFF) decode_addr = 4'd1; // S1: On-chip SRAM
            else if (addr >= 32'h4000_0000 && addr <= 32'h4FFF_FFFF) decode_addr = 4'd2; // S2: APB Peripherals
            else if (addr >= 32'h2000_0000 && addr <= 32'h2FFF_FFFF) decode_addr = 4'd3; // S3: SPI Flash
            else if (addr >= 32'h0200_0000 && addr <= 32'h020B_FFFF) decode_addr = 4'd4; // S4: CLINT
            else if (addr >= 32'h0C00_0000 && addr <= 32'h0FFF_FFFF) decode_addr = 4'd5; // S5: PLIC
            else if (addr >= 32'hA000_0000 && addr <= 32'hAFFF_FFFF) decode_addr = 4'd6; // S6: SDRAM
            else if (addr >= 32'h5000_0000 && addr <= 32'h5000_0FFF) decode_addr = 4'd7; // S7: DMA Config
            else                                                     decode_addr = 4'd8; // Decode Error
        end
    endfunction

    // =========================================================================
    // READ CHANNEL ARBITRATION (FSM)
    // =========================================================================
    localparam R_IDLE = 2'd0;
    localparam R_ADDR = 2'd1;
    localparam R_DATA = 2'd2;

    reg [1:0] r_state;
    reg [1:0] r_grant; // 0:M0, 1:M1, 2:M2, 3:M3
    reg [1:0] r_token; 
    reg [3:0] r_target;
    
    wire [3:0] r_req = {m3_arvalid, m2_arvalid, m1_arvalid, m0_arvalid};
    
    // Tín hiệu xác định lượt cuối trong Burst Read (Chỉ S1 và S6 dùng Burst)
    wire current_rlast = (r_target == 4'd1) ? s1_rlast :
                         (r_target == 4'd6) ? s6_rlast : 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state  <= R_IDLE;
            r_grant  <= 2'd0;
            r_token  <= 2'd0;
            r_target <= 4'd0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    if (r_req != 4'b0000) begin
                        r_state <= R_ADDR;
                        if (r_token == 2'd0) begin
                            if      (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; r_target <= decode_addr(m0_araddr); end
                            else if (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; r_target <= decode_addr(m1_araddr); end
                            else if (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd3; r_target <= decode_addr(m2_araddr); end
                            else if (r_req[3]) begin r_grant <= 2'd3; r_token <= 2'd0; r_target <= decode_addr(m3_araddr); end
                        end else if (r_token == 2'd1) begin
                            if      (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; r_target <= decode_addr(m1_araddr); end
                            else if (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd3; r_target <= decode_addr(m2_araddr); end
                            else if (r_req[3]) begin r_grant <= 2'd3; r_token <= 2'd0; r_target <= decode_addr(m3_araddr); end
                            else if (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; r_target <= decode_addr(m0_araddr); end
                        end else if (r_token == 2'd2) begin
                            if      (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd3; r_target <= decode_addr(m2_araddr); end
                            else if (r_req[3]) begin r_grant <= 2'd3; r_token <= 2'd0; r_target <= decode_addr(m3_araddr); end
                            else if (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; r_target <= decode_addr(m0_araddr); end
                            else if (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; r_target <= decode_addr(m1_araddr); end
                        end else begin
                            if      (r_req[3]) begin r_grant <= 2'd3; r_token <= 2'd0; r_target <= decode_addr(m3_araddr); end
                            else if (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; r_target <= decode_addr(m0_araddr); end
                            else if (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; r_target <= decode_addr(m1_araddr); end
                            else if (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd3; r_target <= decode_addr(m2_araddr); end
                        end
                    end
                end
                R_ADDR: begin
                    if ((r_grant == 2'd0 && m0_arvalid && m0_arready) ||
                        (r_grant == 2'd1 && m1_arvalid && m1_arready) ||
                        (r_grant == 2'd2 && m2_arvalid && m2_arready) ||
                        (r_grant == 2'd3 && m3_arvalid && m3_arready)) begin
                        r_state <= R_DATA;
                    end
                end
                R_DATA: begin
                    if ((r_grant == 2'd0 && m0_rvalid && m0_rready && current_rlast) ||
                        (r_grant == 2'd1 && m1_rvalid && m1_rready && current_rlast) ||
                        (r_grant == 2'd2 && m2_rvalid && m2_rready && current_rlast) ||
                        (r_grant == 2'd3 && m3_rvalid && m3_rready && current_rlast)) begin
                        r_state <= R_IDLE;
                    end
                end
            endcase
        end
    end

    // READ ROUTING (Combinational Logic)
    always @(*) begin
        // Reset defaults
        m0_arready = 1'b0; m0_rvalid = 1'b0; m0_rdata = 32'h0; m0_rresp = 2'b00; m0_rlast = 1'b0;
        m1_arready = 1'b0; m1_rvalid = 1'b0; m1_rdata = 32'h0; m1_rresp = 2'b00;
        m2_arready = 1'b0; m2_rvalid = 1'b0; m2_rdata = 32'h0; m2_rresp = 2'b00;
        m3_arready = 1'b0; m3_rvalid = 1'b0; m3_rdata = 32'h0; m3_rresp = 2'b00;

        s0_arvalid = 1'b0; s0_araddr = 32'h0; s0_rready = 1'b0;
        s1_arvalid = 1'b0; s1_araddr = 32'h0; s1_arlen = 8'd0; s1_arsize = 3'd0; s1_arburst = 2'd0; s1_rready = 1'b0;
        s2_arvalid = 1'b0; s2_araddr = 32'h0; s2_rready = 1'b0;
        s3_arvalid = 1'b0; s3_araddr = 32'h0; s3_rready = 1'b0;
        s4_arvalid = 1'b0; s4_araddr = 32'h0; s4_rready = 1'b0;
        s5_arvalid = 1'b0; s5_araddr = 32'h0; s5_rready = 1'b0;
        s6_arvalid = 1'b0; s6_araddr = 32'h0; s6_arlen = 8'd0; s6_arsize = 3'd0; s6_arburst = 2'd0; s6_rready = 1'b0;
        s7_arvalid = 1'b0; s7_araddr = 32'h0; s7_rready = 1'b0;

        if (r_state == R_ADDR) begin
            if (r_target == 4'd8) begin // Decode Error
                if (r_grant == 2'd0) m0_arready = 1'b1;
                if (r_grant == 2'd1) m1_arready = 1'b1;
                if (r_grant == 2'd2) m2_arready = 1'b1;
                if (r_grant == 2'd3) m3_arready = 1'b1;
            end else begin
                case (r_target)
                    4'd0: begin
                        s0_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s0_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s0_arready; if (r_grant == 1) m1_arready = s0_arready;
                        if (r_grant == 2) m2_arready = s0_arready; if (r_grant == 3) m3_arready = s0_arready;
                    end
                    4'd1: begin
                        s1_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s1_arlen   = (r_grant == 0) ? m0_arlen   : 8'd0;
                        s1_arsize  = (r_grant == 0) ? m0_arsize  : 3'd2;
                        s1_arburst = (r_grant == 0) ? m0_arburst : 2'd0;
                        s1_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s1_arready; if (r_grant == 1) m1_arready = s1_arready;
                        if (r_grant == 2) m2_arready = s1_arready; if (r_grant == 3) m3_arready = s1_arready;
                    end
                    4'd2: begin
                        s2_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s2_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s2_arready; if (r_grant == 1) m1_arready = s2_arready;
                        if (r_grant == 2) m2_arready = s2_arready; if (r_grant == 3) m3_arready = s2_arready;
                    end
                    4'd3: begin
                        s3_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s3_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s3_arready; if (r_grant == 1) m1_arready = s3_arready;
                        if (r_grant == 2) m2_arready = s3_arready; if (r_grant == 3) m3_arready = s3_arready;
                    end
                    4'd4: begin
                        s4_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s4_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s4_arready; if (r_grant == 1) m1_arready = s4_arready;
                        if (r_grant == 2) m2_arready = s4_arready; if (r_grant == 3) m3_arready = s4_arready;
                    end
                    4'd5: begin
                        s5_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s5_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s5_arready; if (r_grant == 1) m1_arready = s5_arready;
                        if (r_grant == 2) m2_arready = s5_arready; if (r_grant == 3) m3_arready = s5_arready;
                    end
                    4'd6: begin
                        s6_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s6_arlen   = (r_grant == 0) ? m0_arlen   : 8'd0;
                        s6_arsize  = (r_grant == 0) ? m0_arsize  : 3'd2;
                        s6_arburst = (r_grant == 0) ? m0_arburst : 2'd0;
                        s6_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s6_arready; if (r_grant == 1) m1_arready = s6_arready;
                        if (r_grant == 2) m2_arready = s6_arready; if (r_grant == 3) m3_arready = s6_arready;
                    end
                    4'd7: begin
                        s7_araddr  = (r_grant == 0) ? m0_araddr  : (r_grant == 1) ? m1_araddr  : (r_grant == 2) ? m2_araddr  : m3_araddr;
                        s7_arvalid = (r_grant == 0) ? m0_arvalid : (r_grant == 1) ? m1_arvalid : (r_grant == 2) ? m2_arvalid : m3_arvalid;
                        if (r_grant == 0) m0_arready = s7_arready; if (r_grant == 1) m1_arready = s7_arready;
                        if (r_grant == 2) m2_arready = s7_arready; if (r_grant == 3) m3_arready = s7_arready;
                    end
                endcase
            end
        end else if (r_state == R_DATA) begin
            if (r_target == 4'd8) begin
                if (r_grant == 0) begin m0_rvalid = 1'b1; m0_rresp = 2'b11; m0_rlast = 1'b1; end
                if (r_grant == 1) begin m1_rvalid = 1'b1; m1_rresp = 2'b11; end
                if (r_grant == 2) begin m2_rvalid = 1'b1; m2_rresp = 2'b11; end
                if (r_grant == 3) begin m3_rvalid = 1'b1; m3_rresp = 2'b11; end
            end else begin
                case (r_target)
                    4'd0: begin
                        if (r_grant == 0) begin m0_rvalid = s0_rvalid; m0_rdata = s0_rdata; m0_rresp = s0_rresp; m0_rlast = 1'b1; s0_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s0_rvalid; m1_rdata = s0_rdata; m1_rresp = s0_rresp; s0_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s0_rvalid; m2_rdata = s0_rdata; m2_rresp = s0_rresp; s0_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s0_rvalid; m3_rdata = s0_rdata; m3_rresp = s0_rresp; s0_rready = m3_rready; end
                    end
                    4'd1: begin
                        if (r_grant == 0) begin m0_rvalid = s1_rvalid; m0_rdata = s1_rdata; m0_rresp = s1_rresp; m0_rlast = s1_rlast; s1_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s1_rvalid; m1_rdata = s1_rdata; m1_rresp = s1_rresp; s1_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s1_rvalid; m2_rdata = s1_rdata; m2_rresp = s1_rresp; s1_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s1_rvalid; m3_rdata = s1_rdata; m3_rresp = s1_rresp; s1_rready = m3_rready; end
                    end
                    4'd2: begin
                        if (r_grant == 0) begin m0_rvalid = s2_rvalid; m0_rdata = s2_rdata; m0_rresp = s2_rresp; m0_rlast = 1'b1; s2_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s2_rvalid; m1_rdata = s2_rdata; m1_rresp = s2_rresp; s2_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s2_rvalid; m2_rdata = s2_rdata; m2_rresp = s2_rresp; s2_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s2_rvalid; m3_rdata = s2_rdata; m3_rresp = s2_rresp; s2_rready = m3_rready; end
                    end
                    4'd3: begin
                        if (r_grant == 0) begin m0_rvalid = s3_rvalid; m0_rdata = s3_rdata; m0_rresp = s3_rresp; m0_rlast = 1'b1; s3_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s3_rvalid; m1_rdata = s3_rdata; m1_rresp = s3_rresp; s3_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s3_rvalid; m2_rdata = s3_rdata; m2_rresp = s3_rresp; s3_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s3_rvalid; m3_rdata = s3_rdata; m3_rresp = s3_rresp; s3_rready = m3_rready; end
                    end
                    4'd4: begin
                        if (r_grant == 0) begin m0_rvalid = s4_rvalid; m0_rdata = s4_rdata; m0_rresp = s4_rresp; m0_rlast = 1'b1; s4_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s4_rvalid; m1_rdata = s4_rdata; m1_rresp = s4_rresp; s4_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s4_rvalid; m2_rdata = s4_rdata; m2_rresp = s4_rresp; s4_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s4_rvalid; m3_rdata = s4_rdata; m3_rresp = s4_rresp; s4_rready = m3_rready; end
                    end
                    4'd5: begin
                        if (r_grant == 0) begin m0_rvalid = s5_rvalid; m0_rdata = s5_rdata; m0_rresp = s5_rresp; m0_rlast = 1'b1; s5_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s5_rvalid; m1_rdata = s5_rdata; m1_rresp = s5_rresp; s5_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s5_rvalid; m2_rdata = s5_rdata; m2_rresp = s5_rresp; s5_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s5_rvalid; m3_rdata = s5_rdata; m3_rresp = s5_rresp; s5_rready = m3_rready; end
                    end
                    4'd6: begin
                        if (r_grant == 0) begin m0_rvalid = s6_rvalid; m0_rdata = s6_rdata; m0_rresp = s6_rresp; m0_rlast = s6_rlast; s6_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s6_rvalid; m1_rdata = s6_rdata; m1_rresp = s6_rresp; s6_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s6_rvalid; m2_rdata = s6_rdata; m2_rresp = s6_rresp; s6_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s6_rvalid; m3_rdata = s6_rdata; m3_rresp = s6_rresp; s6_rready = m3_rready; end
                    end
                    4'd7: begin
                        if (r_grant == 0) begin m0_rvalid = s7_rvalid; m0_rdata = s7_rdata; m0_rresp = s7_rresp; m0_rlast = 1'b1; s7_rready = m0_rready; end
                        if (r_grant == 1) begin m1_rvalid = s7_rvalid; m1_rdata = s7_rdata; m1_rresp = s7_rresp; s7_rready = m1_rready; end
                        if (r_grant == 2) begin m2_rvalid = s7_rvalid; m2_rdata = s7_rdata; m2_rresp = s7_rresp; s7_rready = m2_rready; end
                        if (r_grant == 3) begin m3_rvalid = s7_rvalid; m3_rdata = s7_rdata; m3_rresp = s7_rresp; s7_rready = m3_rready; end
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // WRITE CHANNEL ARBITRATION (FSM)
    // =========================================================================
    localparam W_IDLE     = 2'd0;
    localparam W_TRANSFER = 2'd1;
    localparam W_RESP     = 2'd2;

    reg [1:0] w_state;
    reg [1:0] w_grant;   // 1:M1, 2:M2, 3:M3 (M0 Read-Only)
    reg [1:0] w_token;   // 0, 1, 2
    reg [3:0] w_target;
    reg       aw_done;
    reg       w_done;
    
    wire [2:0] w_req = {m3_awvalid, m2_awvalid, m1_awvalid};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state  <= W_IDLE;
            w_grant  <= 2'd0;
            w_token  <= 2'd0;
            w_target <= 4'd0;
            aw_done  <= 1'b0;
            w_done   <= 1'b0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    aw_done <= 1'b0;
                    w_done  <= 1'b0;
                    if (w_req != 3'b000) begin
                        w_state <= W_TRANSFER;
                        if (w_token == 2'd0) begin
                            if      (w_req[0]) begin w_grant <= 2'd1; w_token <= 2'd1; w_target <= decode_addr(m1_awaddr); end
                            else if (w_req[1]) begin w_grant <= 2'd2; w_token <= 2'd2; w_target <= decode_addr(m2_awaddr); end
                            else if (w_req[2]) begin w_grant <= 2'd3; w_token <= 2'd0; w_target <= decode_addr(m3_awaddr); end
                        end else if (w_token == 2'd1) begin
                            if      (w_req[1]) begin w_grant <= 2'd2; w_token <= 2'd2; w_target <= decode_addr(m2_awaddr); end
                            else if (w_req[2]) begin w_grant <= 2'd3; w_token <= 2'd0; w_target <= decode_addr(m3_awaddr); end
                            else if (w_req[0]) begin w_grant <= 2'd1; w_token <= 2'd1; w_target <= decode_addr(m1_awaddr); end
                        end else begin
                            if      (w_req[2]) begin w_grant <= 2'd3; w_token <= 2'd0; w_target <= decode_addr(m3_awaddr); end
                            else if (w_req[0]) begin w_grant <= 2'd1; w_token <= 2'd1; w_target <= decode_addr(m1_awaddr); end
                            else if (w_req[1]) begin w_grant <= 2'd2; w_token <= 2'd2; w_target <= decode_addr(m2_awaddr); end
                        end
                    end
                end
                W_TRANSFER: begin
                    // Nếu ghi vào ROM (0), Flash (3), hoặc Vùng lỗi (8) -> Kết thúc sớm báo lỗi
                    if (w_target == 4'd0 || w_target == 4'd3 || w_target == 4'd8) begin
                        aw_done <= 1'b1; w_done <= 1'b1; w_state <= W_RESP;
                    end else begin
                        if (!aw_done) begin
                            if ((w_grant == 2'd1 && m1_awvalid && m1_awready) ||
                                (w_grant == 2'd2 && m2_awvalid && m2_awready) ||
                                (w_grant == 2'd3 && m3_awvalid && m3_awready)) aw_done <= 1'b1;
                        end
                        if (!w_done) begin
                            if ((w_grant == 2'd1 && m1_wvalid && m1_wready) ||
                                (w_grant == 2'd2 && m2_wvalid && m2_wready) ||
                                (w_grant == 2'd3 && m3_wvalid && m3_wready)) w_done <= 1'b1;
                        end
                        if ((aw_done || (w_grant==1 && m1_awready) || (w_grant==2 && m2_awready) || (w_grant==3 && m3_awready)) &&
                            (w_done  || (w_grant==1 && m1_wready)  || (w_grant==2 && m2_wready)  || (w_grant==3 && m3_wready))) begin
                             w_state <= W_RESP;
                        end
                    end
                end
                W_RESP: begin
                    if ((w_grant == 2'd1 && m1_bvalid && m1_bready) ||
                        (w_grant == 2'd2 && m2_bvalid && m2_bready) ||
                        (w_grant == 2'd3 && m3_bvalid && m3_bready)) w_state <= W_IDLE;
                end
            endcase
        end
    end

    // WRITE ROUTING (Combinational Logic)
    always @(*) begin
        // Reset defaults
        m1_awready = 1'b0; m1_wready = 1'b0; m1_bvalid = 1'b0; m1_bresp = 2'b00;
        m2_awready = 1'b0; m2_wready = 1'b0; m2_bvalid = 1'b0; m2_bresp = 2'b00;
        m3_awready = 1'b0; m3_wready = 1'b0; m3_bvalid = 1'b0; m3_bresp = 2'b00;

        s1_awvalid = 1'b0; s1_awaddr = 32'h0; s1_wvalid = 1'b0; s1_wdata = 32'h0; s1_wstrb = 4'b0000; s1_bready = 1'b0;
        s2_awvalid = 1'b0; s2_awaddr = 32'h0; s2_wvalid = 1'b0; s2_wdata = 32'h0; s2_wstrb = 4'b0000; s2_bready = 1'b0;
        s4_awvalid = 1'b0; s4_awaddr = 32'h0; s4_wvalid = 1'b0; s4_wdata = 32'h0; s4_wstrb = 4'b0000; s4_bready = 1'b0;
        s5_awvalid = 1'b0; s5_awaddr = 32'h0; s5_wvalid = 1'b0; s5_wdata = 32'h0; s5_wstrb = 4'b0000; s5_bready = 1'b0;
        s6_awvalid = 1'b0; s6_awaddr = 32'h0; s6_wvalid = 1'b0; s6_wdata = 32'h0; s6_wstrb = 4'b0000; s6_bready = 1'b0;
        s7_awvalid = 1'b0; s7_awaddr = 32'h0; s7_wvalid = 1'b0; s7_wdata = 32'h0; s7_wstrb = 4'b0000; s7_bready = 1'b0;

        if (w_state == W_TRANSFER) begin
            if (w_target == 4'd0 || w_target == 4'd3 || w_target == 4'd8) begin
                if (w_grant == 2'd1) begin m1_awready = !aw_done; m1_wready = !w_done; end
                if (w_grant == 2'd2) begin m2_awready = !aw_done; m2_wready = !w_done; end
                if (w_grant == 2'd3) begin m3_awready = !aw_done; m3_wready = !w_done; end
            end else begin
                case (w_target)
                    4'd1: begin
                        s1_awaddr  = (w_grant == 1) ? m1_awaddr  : (w_grant == 2) ? m2_awaddr  : m3_awaddr;
                        s1_awvalid = !aw_done ? ((w_grant == 1) ? m1_awvalid : (w_grant == 2) ? m2_awvalid : m3_awvalid) : 1'b0;
                        s1_wdata   = (w_grant == 1) ? m1_wdata   : (w_grant == 2) ? m2_wdata   : m3_wdata;
                        s1_wstrb   = (w_grant == 1) ? m1_wstrb   : (w_grant == 2) ? m2_wstrb   : m3_wstrb;
                        s1_wvalid  = !w_done  ? ((w_grant == 1) ? m1_wvalid  : (w_grant == 2) ? m2_wvalid  : m3_wvalid ) : 1'b0;
                        if (w_grant == 1) begin m1_awready = s1_awready && !aw_done; m1_wready = s1_wready && !w_done; end
                        if (w_grant == 2) begin m2_awready = s1_awready && !aw_done; m2_wready = s1_wready && !w_done; end
                        if (w_grant == 3) begin m3_awready = s1_awready && !aw_done; m3_wready = s1_wready && !w_done; end
                    end
                    4'd2: begin
                        s2_awaddr  = (w_grant == 1) ? m1_awaddr  : (w_grant == 2) ? m2_awaddr  : m3_awaddr;
                        s2_awvalid = !aw_done ? ((w_grant == 1) ? m1_awvalid : (w_grant == 2) ? m2_awvalid : m3_awvalid) : 1'b0;
                        s2_wdata   = (w_grant == 1) ? m1_wdata   : (w_grant == 2) ? m2_wdata   : m3_wdata;
                        s2_wstrb   = (w_grant == 1) ? m1_wstrb   : (w_grant == 2) ? m2_wstrb   : m3_wstrb;
                        s2_wvalid  = !w_done  ? ((w_grant == 1) ? m1_wvalid  : (w_grant == 2) ? m2_wvalid  : m3_wvalid ) : 1'b0;
                        if (w_grant == 1) begin m1_awready = s2_awready && !aw_done; m1_wready = s2_wready && !w_done; end
                        if (w_grant == 2) begin m2_awready = s2_awready && !aw_done; m2_wready = s2_wready && !w_done; end
                        if (w_grant == 3) begin m3_awready = s2_awready && !aw_done; m3_wready = s2_wready && !w_done; end
                    end
                    4'd4: begin
                        s4_awaddr  = (w_grant == 1) ? m1_awaddr  : (w_grant == 2) ? m2_awaddr  : m3_awaddr;
                        s4_awvalid = !aw_done ? ((w_grant == 1) ? m1_awvalid : (w_grant == 2) ? m2_awvalid : m3_awvalid) : 1'b0;
                        s4_wdata   = (w_grant == 1) ? m1_wdata   : (w_grant == 2) ? m2_wdata   : m3_wdata;
                        s4_wstrb   = (w_grant == 1) ? m1_wstrb   : (w_grant == 2) ? m2_wstrb   : m3_wstrb;
                        s4_wvalid  = !w_done  ? ((w_grant == 1) ? m1_wvalid  : (w_grant == 2) ? m2_wvalid  : m3_wvalid ) : 1'b0;
                        if (w_grant == 1) begin m1_awready = s4_awready && !aw_done; m1_wready = s4_wready && !w_done; end
                        if (w_grant == 2) begin m2_awready = s4_awready && !aw_done; m2_wready = s4_wready && !w_done; end
                        if (w_grant == 3) begin m3_awready = s4_awready && !aw_done; m3_wready = s4_wready && !w_done; end
                    end
                    4'd5: begin
                        s5_awaddr  = (w_grant == 1) ? m1_awaddr  : (w_grant == 2) ? m2_awaddr  : m3_awaddr;
                        s5_awvalid = !aw_done ? ((w_grant == 1) ? m1_awvalid : (w_grant == 2) ? m2_awvalid : m3_awvalid) : 1'b0;
                        s5_wdata   = (w_grant == 1) ? m1_wdata   : (w_grant == 2) ? m2_wdata   : m3_wdata;
                        s5_wstrb   = (w_grant == 1) ? m1_wstrb   : (w_grant == 2) ? m2_wstrb   : m3_wstrb;
                        s5_wvalid  = !w_done  ? ((w_grant == 1) ? m1_wvalid  : (w_grant == 2) ? m2_wvalid  : m3_wvalid ) : 1'b0;
                        if (w_grant == 1) begin m1_awready = s5_awready && !aw_done; m1_wready = s5_wready && !w_done; end
                        if (w_grant == 2) begin m2_awready = s5_awready && !aw_done; m2_wready = s5_wready && !w_done; end
                        if (w_grant == 3) begin m3_awready = s5_awready && !aw_done; m3_wready = s5_wready && !w_done; end
                    end
                    4'd6: begin
                        s6_awaddr  = (w_grant == 1) ? m1_awaddr  : (w_grant == 2) ? m2_awaddr  : m3_awaddr;
                        s6_awvalid = !aw_done ? ((w_grant == 1) ? m1_awvalid : (w_grant == 2) ? m2_awvalid : m3_awvalid) : 1'b0;
                        s6_wdata   = (w_grant == 1) ? m1_wdata   : (w_grant == 2) ? m2_wdata   : m3_wdata;
                        s6_wstrb   = (w_grant == 1) ? m1_wstrb   : (w_grant == 2) ? m2_wstrb   : m3_wstrb;
                        s6_wvalid  = !w_done  ? ((w_grant == 1) ? m1_wvalid  : (w_grant == 2) ? m2_wvalid  : m3_wvalid ) : 1'b0;
                        if (w_grant == 1) begin m1_awready = s6_awready && !aw_done; m1_wready = s6_wready && !w_done; end
                        if (w_grant == 2) begin m2_awready = s6_awready && !aw_done; m2_wready = s6_wready && !w_done; end
                        if (w_grant == 3) begin m3_awready = s6_awready && !aw_done; m3_wready = s6_wready && !w_done; end
                    end
                    4'd7: begin
                        s7_awaddr  = (w_grant == 1) ? m1_awaddr  : (w_grant == 2) ? m2_awaddr  : m3_awaddr;
                        s7_awvalid = !aw_done ? ((w_grant == 1) ? m1_awvalid : (w_grant == 2) ? m2_awvalid : m3_awvalid) : 1'b0;
                        s7_wdata   = (w_grant == 1) ? m1_wdata   : (w_grant == 2) ? m2_wdata   : m3_wdata;
                        s7_wstrb   = (w_grant == 1) ? m1_wstrb   : (w_grant == 2) ? m2_wstrb   : m3_wstrb;
                        s7_wvalid  = !w_done  ? ((w_grant == 1) ? m1_wvalid  : (w_grant == 2) ? m2_wvalid  : m3_wvalid ) : 1'b0;
                        if (w_grant == 1) begin m1_awready = s7_awready && !aw_done; m1_wready = s7_wready && !w_done; end
                        if (w_grant == 2) begin m2_awready = s7_awready && !aw_done; m2_wready = s7_wready && !w_done; end
                        if (w_grant == 3) begin m3_awready = s7_awready && !aw_done; m3_wready = s7_wready && !w_done; end
                    end
                endcase
            end
        end else if (w_state == W_RESP) begin
            if (w_target == 4'd0 || w_target == 4'd3 || w_target == 4'd8) begin
                if (w_grant == 2'd1) begin m1_bvalid = 1'b1; m1_bresp = 2'b11; end
                if (w_grant == 2'd2) begin m2_bvalid = 1'b1; m2_bresp = 2'b11; end
                if (w_grant == 2'd3) begin m3_bvalid = 1'b1; m3_bresp = 2'b11; end
            end else begin
                case (w_target)
                    4'd1: begin
                        if (w_grant == 2'd1) begin m1_bvalid = s1_bvalid; m1_bresp = s1_bresp; s1_bready = m1_bready; end
                        if (w_grant == 2'd2) begin m2_bvalid = s1_bvalid; m2_bresp = s1_bresp; s1_bready = m2_bready; end
                        if (w_grant == 2'd3) begin m3_bvalid = s1_bvalid; m3_bresp = s1_bresp; s1_bready = m3_bready; end
                    end
                    4'd2: begin
                        if (w_grant == 2'd1) begin m1_bvalid = s2_bvalid; m1_bresp = s2_bresp; s2_bready = m1_bready; end
                        if (w_grant == 2'd2) begin m2_bvalid = s2_bvalid; m2_bresp = s2_bresp; s2_bready = m2_bready; end
                        if (w_grant == 2'd3) begin m3_bvalid = s2_bvalid; m3_bresp = s2_bresp; s2_bready = m3_bready; end
                    end
                    4'd4: begin
                        if (w_grant == 2'd1) begin m1_bvalid = s4_bvalid; m1_bresp = s4_bresp; s4_bready = m1_bready; end
                        if (w_grant == 2'd2) begin m2_bvalid = s4_bvalid; m2_bresp = s4_bresp; s4_bready = m2_bready; end
                        if (w_grant == 2'd3) begin m3_bvalid = s4_bvalid; m3_bresp = s4_bresp; s4_bready = m3_bready; end
                    end
                    4'd5: begin
                        if (w_grant == 2'd1) begin m1_bvalid = s5_bvalid; m1_bresp = s5_bresp; s5_bready = m1_bready; end
                        if (w_grant == 2'd2) begin m2_bvalid = s5_bvalid; m2_bresp = s5_bresp; s5_bready = m2_bready; end
                        if (w_grant == 2'd3) begin m3_bvalid = s5_bvalid; m3_bresp = s5_bresp; s5_bready = m3_bready; end
                    end
                    4'd6: begin
                        if (w_grant == 2'd1) begin m1_bvalid = s6_bvalid; m1_bresp = s6_bresp; s6_bready = m1_bready; end
                        if (w_grant == 2'd2) begin m2_bvalid = s6_bvalid; m2_bresp = s6_bresp; s6_bready = m2_bready; end
                        if (w_grant == 2'd3) begin m3_bvalid = s6_bvalid; m3_bresp = s6_bresp; s6_bready = m3_bready; end
                    end
                    4'd7: begin
                        if (w_grant == 2'd1) begin m1_bvalid = s7_bvalid; m1_bresp = s7_bresp; s7_bready = m1_bready; end
                        if (w_grant == 2'd2) begin m2_bvalid = s7_bvalid; m2_bresp = s7_bresp; s7_bready = m2_bready; end
                        if (w_grant == 2'd3) begin m3_bvalid = s7_bvalid; m3_bresp = s7_bresp; s7_bready = m3_bready; end
                    end
                endcase
            end
        end
    end

endmodule