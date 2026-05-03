`timescale 1ns / 1ps

module debug_module #(
    parameter ADDR_WIDTH = 32, 
    parameter DATA_WIDTH = 32
)(
    // JTAG TCK Domain
    input  wire tck,
    input  wire trst_n,
    input  wire i_shift_dr,
    input  wire i_capture_dr,
    input  wire i_update_dr,
    input  wire i_sel_dmi,
    input  wire i_tdi,
    output wire o_tdo,

    // System CLK Domain
    input  wire clk_sys,
    input  wire rst_sys_n,
    
    // Interface to DTM AXI Master (Sys CLK Domain)
    output reg                   req_sys,
    output reg  [1:0]            op_sys,   // 0: NOP, 1: READ, 2: WRITE
    output reg  [ADDR_WIDTH-1:0] addr_sys,
    output reg  [DATA_WIDTH-1:0] wdata_sys,
    
    input  wire                  ack_sys,
    input  wire [1:0]            resp_sys, // 0: OK, 2: ERROR
    input  wire [DATA_WIDTH-1:0] rdata_sys
);

    // =========================================================
    // JTAG TCK DOMAIN LOGIC
    // =========================================================
    
    reg [65:0] dmi_shift_reg;
    reg req_tck;
    reg [1:0]            cmd_op;
    reg [ADDR_WIDTH-1:0] cmd_addr;
    reg [DATA_WIDTH-1:0] cmd_wdata;
    
    reg ack_sync1_tck, ack_sync2_tck;
    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) {ack_sync2_tck, ack_sync1_tck} <= 2'b00;
        else         {ack_sync2_tck, ack_sync1_tck} <= {ack_sync1_tck, ack_sys};
    end

    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            dmi_shift_reg <= 66'b0;
            req_tck       <= 1'b0;
            cmd_op        <= 2'b0;
            cmd_addr      <= 32'b0;
            cmd_wdata     <= 32'b0;
        end else if (i_sel_dmi) begin
            if (i_capture_dr) begin
                dmi_shift_reg <= {resp_sys, cmd_addr, rdata_sys};
            end
            else if (i_shift_dr) begin
                dmi_shift_reg <= {i_tdi, dmi_shift_reg[65:1]};
            end
            else if (i_update_dr) begin
                if (dmi_shift_reg[65:64] != 2'b00) begin 
                    cmd_op    <= dmi_shift_reg[65:64];
                    cmd_addr  <= dmi_shift_reg[63:32];
                    cmd_wdata <= dmi_shift_reg[31:0];
                    req_tck   <= 1'b1;
                end
            end
            
            if (req_tck && ack_sync2_tck) begin
                req_tck <= 1'b0;
            end
        end
    end

    assign o_tdo = dmi_shift_reg[0];

    // =========================================================
    // SYSTEM CLK DOMAIN LOGIC
    // =========================================================
    
    reg req_sync1_sys, req_sync2_sys, req_sync3_sys;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) {req_sync3_sys, req_sync2_sys, req_sync1_sys} <= 3'b000;
        else            {req_sync3_sys, req_sync2_sys, req_sync1_sys} <= {req_sync2_sys, req_sync1_sys, req_tck};
    end

    wire rising_req_sys = (req_sync2_sys && !req_sync3_sys);

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            req_sys   <= 1'b0;
            op_sys    <= 2'b0;
            addr_sys  <= 32'b0;
            wdata_sys <= 32'b0;
        end else begin
            if (rising_req_sys && !ack_sys) begin
                req_sys   <= 1'b1;
                op_sys    <= cmd_op;       
                addr_sys  <= cmd_addr;
                wdata_sys <= cmd_wdata;
            end 
            else if (ack_sys) begin
                req_sys   <= 1'b0;
            end
        end
    end

endmodule