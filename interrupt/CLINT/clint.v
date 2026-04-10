`timescale 1ns / 1ps
`include "clint_defines.vh"

// ============================================================
//  clint_top.v  —  Top-level CLINT (AXI4 Full, Verilog-2001)
//
//  Kết nối:
//    clint_axi4_slave  reg_bus  clint_regfile
//    clint_regfile     mtime_wr clint_timer
//    clint_regfile     snapshot clint_timer
//    clint_timer       mtime  clint_regfile  (read)
//    clint_timer       mtime  clint_irq_gen
//    clint_regfile     msip/cmp clint_irq_gen
//    clint_irq_gen     irq  CPU hart
//
//  Tích hợp với PLIC:
//    CLINT  -> msip_o, mtip_o -> Hart CSR MIP.MSIP / MIP.MTIP
//    PLIC   -> meip           -> Hart CSR MIP.MEIP
//    Hai hệ thống HOÀN TOÀN ĐỘC LẬP (không kết nối với nhau)
//
//  Parameters:
//    NUM_HARTS      : Số hart (default 1)
//    HART_IDX_W     : ceil(log2(NUM_HARTS)), min 1
//                     Phải truyền thủ công vì $clog2 không khả dụng
//                     ở mọi tool cho Verilog-2001
//    AXI_ADDR_WIDTH : Byte address width của AXI bus
//    AXI_ID_WIDTH   : Transaction ID width
//    PIPELINE_IRQ   : 1 = thêm FF stage trên mtip/msip output
// ============================================================

module axi_clint #(
    parameter NUM_HARTS      = 1,
    parameter HART_IDX_W     = 1,    // = max(1, ceil(log2(NUM_HARTS)))
    parameter AXI_ADDR_WIDTH = 26,
    parameter AXI_ID_WIDTH   = 4,
    parameter PIPELINE_IRQ   = 1
) (
    //  Clocks & Reset ─
    input  wire clk_i,
    input  wire rst_ni,
    input  wire rtc_clk_i,    // RTC clock (bất đồng bộ)

    //  AXI4 Full Slave

    // Write Address
    input  wire  [AXI_ID_WIDTH-1:0]   s_axi_awid,
    input  wire  [AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire  [7:0]                s_axi_awlen,
    input  wire  [2:0]                s_axi_awsize,
    input  wire  [1:0]                s_axi_awburst,
    input  wire                       s_axi_awlock,
    input  wire  [3:0]                s_axi_awcache,
    input  wire  [2:0]                s_axi_awprot,
    input  wire  [3:0]                s_axi_awqos,
    input  wire  [3:0]                s_axi_awregion,
    input  wire                       s_axi_awvalid,
    output wire                       s_axi_awready,

    // Write Data
    input  wire  [31:0]               s_axi_wdata,
    input  wire   [3:0]               s_axi_wstrb,
    input  wire                       s_axi_wlast,
    input  wire                       s_axi_wvalid,
    output wire                       s_axi_wready,

    // Write Response
    output wire  [AXI_ID_WIDTH-1:0]   s_axi_bid,
    output wire  [1:0]                s_axi_bresp,
    output wire                       s_axi_bvalid,
    input  wire                       s_axi_bready,

    // Read Address
    input  wire  [AXI_ID_WIDTH-1:0]   s_axi_arid,
    input  wire  [AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire  [7:0]                s_axi_arlen,
    input  wire  [2:0]                s_axi_arsize,
    input  wire  [1:0]                s_axi_arburst,
    input  wire                       s_axi_arlock,
    input  wire  [3:0]                s_axi_arcache,
    input  wire  [2:0]                s_axi_arprot,
    input  wire  [3:0]                s_axi_arqos,
    input  wire  [3:0]                s_axi_arregion,
    input  wire                       s_axi_arvalid,
    output wire                       s_axi_arready,

    // Read Data
    output wire  [AXI_ID_WIDTH-1:0]   s_axi_rid,
    output wire  [31:0]               s_axi_rdata,
    output wire  [1:0]                s_axi_rresp,
    output wire                       s_axi_rlast,
    output wire                       s_axi_rvalid,
    input  wire                       s_axi_rready,

    //  Interrupt Outputs -> CPU Hart
    //  Connect trực tiếp vào CSR MIP, không qua PLIC
    output wire [NUM_HARTS-1:0] msip_o,
    output wire [NUM_HARTS-1:0] mtip_o
);

    //  Internal register bus 
    wire               reg_req_valid;
    wire               reg_req_write;
    wire [AXI_ADDR_WIDTH-1:0] reg_req_addr;
    wire [31:0]        reg_req_wdata;
    wire  [3:0]        reg_req_wstrb;
    wire [31:0]        reg_rsp_rdata;
    wire               reg_rsp_error;

    //  Timer wires 
    wire               mtime_wr_valid;
    wire               mtime_wr_hi;
    wire [31:0]        mtime_wr_data;
    wire  [3:0]        mtime_wr_strb;
    wire               snapshot_latch;
    wire [31:0]        mtime_lo;
    wire [31:0]        mtime_hi;
    wire [31:0]        mtime_hi_snap;

    //  IRQ gen wires 
    wire [NUM_HARTS-1:0]    msip_q;
    wire [NUM_HARTS*64-1:0] mtimecmp_flat;

    //  AXI4 Slave
    clint_axi4_slave #(
        .ADDR_W  (AXI_ADDR_WIDTH),
        .ID_W    (AXI_ID_WIDTH)
    ) u_slave (
        .clk_i(clk_i),  
        .rst_ni(rst_ni),
        // AW
        .s_axi_awid(s_axi_awid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awlen(s_axi_awlen),
        .s_axi_awsize(s_axi_awsize), .s_axi_awburst(s_axi_awburst), .s_axi_awlock(s_axi_awlock),
        .s_axi_awcache(s_axi_awcache), .s_axi_awprot(s_axi_awprot), .s_axi_awqos(s_axi_awqos),
        .s_axi_awregion(s_axi_awregion), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        // W
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wlast(s_axi_wlast),
        .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        // B
        .s_axi_bid(s_axi_bid), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        // AR
        .s_axi_arid(s_axi_arid), .s_axi_araddr(s_axi_araddr), .s_axi_arlen(s_axi_arlen),
        .s_axi_arsize(s_axi_arsize), .s_axi_arburst(s_axi_arburst), .s_axi_arlock(s_axi_arlock),
        .s_axi_arcache(s_axi_arcache), .s_axi_arprot(s_axi_arprot), .s_axi_arqos(s_axi_arqos),
        .s_axi_arregion(s_axi_arregion), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        // R
        .s_axi_rid(s_axi_rid), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rlast(s_axi_rlast), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        // reg bus
        .reg_req_valid_o (reg_req_valid),
        .reg_req_write_o (reg_req_write),
        .reg_req_addr_o  (reg_req_addr),
        .reg_req_wdata_o (reg_req_wdata),
        .reg_req_wstrb_o (reg_req_wstrb),
        .reg_rsp_rdata_i (reg_rsp_rdata),
        .reg_rsp_error_i (reg_rsp_error)
    );

    //  Register File
    clint_regfile #(
        .NUM_HARTS  (NUM_HARTS),
        .HART_IDX_W (HART_IDX_W),
        .ADDR_W     (AXI_ADDR_WIDTH)
    ) u_regfile (
        .clk_i(clk_i), 
        .rst_ni(rst_ni),
        .req_valid_i  (reg_req_valid),
        .req_write_i  (reg_req_write),
        .req_addr_i   (reg_req_addr),
        .req_wdata_i  (reg_req_wdata),
        .req_wstrb_i  (reg_req_wstrb),
        .rsp_rdata_o  (reg_rsp_rdata),
        .rsp_error_o  (reg_rsp_error),
        .mtime_lo_i         (mtime_lo),
        .mtime_hi_snap_i    (mtime_hi_snap),
        .mtime_wr_valid_o   (mtime_wr_valid),
        .mtime_wr_hi_o      (mtime_wr_hi),
        .mtime_wr_data_o    (mtime_wr_data),
        .mtime_wr_strb_o    (mtime_wr_strb),
        .snapshot_latch_o   (snapshot_latch),
        .msip_o             (msip_q),
        .mtimecmp_flat_o    (mtimecmp_flat)
    );

    //  Timer
    clint_timer u_timer (
        .clk_i(clk_i), 
        .rst_ni(rst_ni), 
        .rtc_clk_i(rtc_clk_i),
        .mtime_wr_valid_i  (mtime_wr_valid),
        .mtime_wr_hi_i     (mtime_wr_hi),
        .mtime_wr_data_i   (mtime_wr_data),
        .mtime_wr_strb_i   (mtime_wr_strb),
        .snapshot_latch_i  (snapshot_latch),
        .mtime_lo_o        (mtime_lo),
        .mtime_hi_o        (mtime_hi),
        .mtime_hi_snap_o   (mtime_hi_snap)
    );

    //  IRQ Generator
    clint_irq_gen #(
        .NUM_HARTS       (NUM_HARTS),
        .PIPELINE_OUTPUT (PIPELINE_IRQ)
    ) u_irq_gen (
        .clk_i(clk_i), 
        .rst_ni(rst_ni),
        .msip_i            (msip_q),
        .mtimecmp_flat_i   (mtimecmp_flat),
        .mtime_lo_i        (mtime_lo),
        .mtime_hi_i        (mtime_hi),
        .msip_o(msip_o),
        .mtip_o(mtip_o)
    );

endmodule
