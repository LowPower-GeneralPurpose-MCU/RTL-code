`timescale 1ns / 1ps
`include "plic_defines.vh"

module plic_apb_reg (
    input  wire        clk_i,
    input  wire        rst_ni,

    // APB Interface
    input  wire [31:0] paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output wire        pready,
    output reg  [31:0] prdata,
    output wire        pslverr,

    // Tín hiệu giao tiếp với PLIC Core
    output wire [(`PLIC_NUM_SRC*`PLIC_PRIO_WIDTH)-1:0] prio_o_flat,
    output wire [`PLIC_NUM_SRC-1:0]    ie_o,
    output wire [`PLIC_PRIO_WIDTH-1:0] threshold_o,
    input  wire [`PLIC_NUM_SRC-1:0]    ip_i,
    input  wire [31:0]                 claim_id_i,
    output reg  [31:0]                 claim_o,
    output reg  [31:0]                 complete_o
);

    wire apb_write = psel & penable & pwrite;
    wire apb_read  = psel & ~penable & ~pwrite; 
    
    assign pready  = 1'b1;
    assign pslverr = 1'b0;

    reg [`PLIC_PRIO_WIDTH-1:0] prio_reg [0:`PLIC_NUM_SRC-1];
    reg [`PLIC_NUM_SRC-1:0]    ie_reg;
    reg [`PLIC_PRIO_WIDTH-1:0] threshold_reg;

    assign ie_o        = ie_reg;
    assign threshold_o = threshold_reg;

    // Đóng gói mảng 2 chiều thành vector phẳng
    genvar g;
    generate
        for (g = 0; g < `PLIC_NUM_SRC; g = g + 1) begin : gen_prio_pack
            assign prio_o_flat[g*`PLIC_PRIO_WIDTH +: `PLIC_PRIO_WIDTH] = prio_reg[g];
        end
    endgenerate

    integer i;
    reg [31:0] write_idx;
    reg [31:0] read_idx;
    wire [21:0] plic_offset = paddr[21:0];

    // --- WRITE LOGIC ---
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (i = 0; i < `PLIC_NUM_SRC; i = i + 1) prio_reg[i] <= 0;
            ie_reg        <= 0;
            threshold_reg <= 0;
            complete_o    <= 0;
        end else begin
            complete_o <= 0; 
            
            if (apb_write) begin
                if (plic_offset >= 22'h000000 && plic_offset < 22'h000080) begin
                    write_idx = plic_offset[6:2];
                    if (write_idx > 0 && write_idx < `PLIC_NUM_SRC) 
                        prio_reg[write_idx] <= pwdata[`PLIC_PRIO_WIDTH-1:0];
                end
                else if (plic_offset == 22'h002000) ie_reg <= pwdata[`PLIC_NUM_SRC-1:0];
                else if (plic_offset == 22'h200000) threshold_reg <= pwdata[`PLIC_PRIO_WIDTH-1:0];
                else if (plic_offset == 22'h200004) complete_o <= pwdata;
            end
        end
    end

    // --- READ LOGIC ---
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            claim_o <= 0;
            prdata  <= 0;
        end else begin
            claim_o <= 0; 

            if (apb_read) begin
                prdata <= 32'h0; 
                
                if (plic_offset >= 22'h000000 && plic_offset < 22'h000080) begin
                    read_idx = plic_offset[6:2];
                    if (read_idx < `PLIC_NUM_SRC) 
                        prdata <= { {32-`PLIC_PRIO_WIDTH{1'b0}}, prio_reg[read_idx] };
                end
                // Đọc Pending: Offset 0x001000
                else if (plic_offset == 22'h001000) begin
                    prdata <= ip_i;
                end
                // Đọc Enable: Offset 0x002000
                else if (plic_offset == 22'h002000) begin
                    prdata <= ie_reg;
                end
                // Đọc Threshold: Offset 0x200000
                else if (plic_offset == 22'h200000) begin
                    prdata <= { {32-`PLIC_PRIO_WIDTH{1'b0}}, threshold_reg };
                end
                // Đọc Claim: Offset 0x200004
                else if (plic_offset == 22'h200004) begin
                    prdata  <= claim_id_i;
                    claim_o <= claim_id_i; 
                end
            end
        end
    end

endmodule