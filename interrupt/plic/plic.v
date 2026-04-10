`timescale 1ns / 1ps
`include "plic_defines.vh"

module apb_plic #(
    parameter ALGORITHM = "SEQUENTIAL" // Chọn: "SEQUENTIAL", "BINARY_TREE", "MATRIX"
)(
    input  wire        clk_i,
    input  wire        rst_ni,

    // Giao tiếp APB Slave
    input  wire [31:0] paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output wire        pready,
    output wire [31:0] prdata,
    output wire        pslverr,

    // Nguồn ngắt từ ngoại vi
    input  wire [`PLIC_NUM_SRC-1:0] irq_src_i,

    // Tín hiệu External Interrupt báo cho CPU
    output wire        irq_o
);

    wire [(`PLIC_NUM_SRC*`PLIC_PRIO_WIDTH)-1:0] prio_flat;
    wire [`PLIC_NUM_SRC-1:0]                    ie;
    wire [`PLIC_PRIO_WIDTH-1:0]                 threshold;
    wire [`PLIC_NUM_SRC-1:0]                    ip;
    wire [31:0]                                 claim_id;
    wire [31:0]                                 claim_bus;
    wire [31:0]                                 complete_bus;

    plic_apb_reg u_reg_bus (
        .clk_i      (clk_i),
        .rst_ni     (rst_ni),
        .paddr      (paddr),
        .psel       (psel),
        .penable    (penable),
        .pwrite     (pwrite),
        .pwdata     (pwdata),
        .pready     (pready),
        .prdata     (prdata),
        .pslverr    (pslverr),
        
        .prio_o_flat(prio_flat),
        .ie_o       (ie),
        .threshold_o(threshold),
        .ip_i       (ip),
        .claim_id_i (claim_id),
        .claim_o    (claim_bus),
        .complete_o (complete_bus)
    );

    plic_core #(
        .ALGORITHM(ALGORITHM) // Truyền cấu hình xuống Core
    ) u_core (
        .clk_i      (clk_i),
        .rst_ni     (rst_ni),
        .irq_src_i  (irq_src_i),
        
        .prio_i_flat(prio_flat),
        .ie_i       (ie),
        .threshold_i(threshold),
        .claim_i    (claim_bus),
        .complete_i (complete_bus),
        .ip_o       (ip),
        .claim_id_o (claim_id),
        
        .irq_o      (irq_o)
    );

endmodule