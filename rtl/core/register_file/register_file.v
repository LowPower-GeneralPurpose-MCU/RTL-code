//==================================================================================================
// File: register_file.v
//==================================================================================================
module register_file (
    input clk,
    input reset_n,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input mem_wb_reg_write,
    input [4:0] mem_wb_rd,
    input [31:0] mem_wb_write_data,
    output [31:0] read_data1,
    output [31:0] read_data2
);

    (* ram_style = "distributed" *) reg [31:0] rf_main [0:31];

    reg [31:0] x2_sp;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            x2_sp <= 32'd4096;
        end else if (mem_wb_reg_write) begin
            if (mem_wb_rd == 5'd2) begin
                x2_sp <= mem_wb_write_data;
            end else if (mem_wb_rd != 5'd0) begin
                rf_main[mem_wb_rd] <= mem_wb_write_data;
            end
        end
    end

    assign read_data1 = (read_reg1 == 5'd0) ? 32'd0 :
                        (read_reg1 == 5'd2) ? x2_sp : rf_main[read_reg1];
                        
    assign read_data2 = (read_reg2 == 5'd0) ? 32'd0 :
                        (read_reg2 == 5'd2) ? x2_sp : rf_main[read_reg2];
                        
endmodule


module f_register_file (
    input clk, 
    input reset_n,
    input [4:0] read_reg1, 
    input [4:0] read_reg2,
    output [31:0] read_data1, 
    output [31:0] read_data2,
    input reg_write_en,   
    input [4:0] write_reg,    
    input [31:0] write_data    
);

    (* ram_style = "distributed" *) reg [31:0] f_regfile [0:31];
    
    assign read_data1 = f_regfile[read_reg1];
    assign read_data2 = f_regfile[read_reg2];
    
    integer i;
    
    always @(posedge clk) begin
        if (!reset_n) begin
            for (i = 0; i < 32; i = i + 1) begin
                f_regfile[i] <= 32'b0;
            end
        end else if (reg_write_en) begin
            f_regfile[write_reg] <= write_data;
        end
    end
    
endmodule


module csr_register_file (
    input clk,
    input reset_n,
    input meip_i, // External Interrupt Pending
    input msip_i, // Software Interrupt Pending
    input mtip_i, // Timer Interrupt Pending
    input [11:0] csr_addr,
    output reg [31:0] csr_read_data,
    input [11:0] csr_write_addr,
    input [31:0] csr_write_data,
    input [1:0] csr_op,
    input csr_write_en,
    input count_en,
    input instret_en,
    input trap_enter,
    input mret_exec,
    input [31:0] trap_cause,
    input [31:0] trap_pc,
    input [31:0] trap_val,
    output [31:0] mtvec_out,
    output [31:0] mepc_out,
    output [31:0] mie_out,
    output mstatus_mie
);

    localparam [31:0] MVENDORID  = 32'h0;
    localparam [31:0] MARCHID    = 32'h0;
    localparam [31:0] MIMPID     = 32'h01000000;
    localparam [31:0] MHARTID    = 32'h0;
    localparam [31:0] MISA       = 32'h40000100;

    reg [31:0] mstatus;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mscratch;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mtval;
    wire [31:0] mip_val = {20'd0, meip_i, 3'd0, mtip_i, 3'd0, msip_i, 3'd0};

    reg [63:0] mcycle;
    reg [63:0] minstret;

    assign mtvec_out = mtvec;
    assign mepc_out = mepc;
    assign mstatus_mie = mstatus[3];
    assign mie_out = mie;
    
    always @(*) begin
        case (csr_addr)
            12'hF11: csr_read_data = MVENDORID;
            12'hF12: csr_read_data = MARCHID;
            12'hF13: csr_read_data = MIMPID;
            12'hF14: csr_read_data = MHARTID;
            12'h301: csr_read_data = MISA;
            12'h300: csr_read_data = mstatus;
            12'h304: csr_read_data = mie;
            12'h305: csr_read_data = mtvec;
            12'h340: csr_read_data = mscratch;
            12'h341: csr_read_data = mepc;
            12'h342: csr_read_data = mcause;
            12'h343: csr_read_data = mtval;
            12'h344: csr_read_data = mip_val;
            12'hB00, 12'hC00, 12'hC01: csr_read_data = mcycle[31:0];
            12'hB80, 12'hC80, 12'hC81: csr_read_data = mcycle[63:32];
            12'hB02, 12'hC02:          csr_read_data = minstret[31:0];
            12'hB82, 12'hC82:          csr_read_data = minstret[63:32];
            default: csr_read_data = 32'b0;
        endcase
    end
    
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            mstatus  <= 32'h00001800;
            mie      <= 32'b0;
            mtvec    <= 32'b0;
            mscratch <= 32'b0;
            mepc     <= 32'b0;
            mcause   <= 32'b0;
            mtval    <= 32'b0;
            mcycle   <= 64'b0;
            minstret <= 64'b0;
        end else begin
            if (count_en) begin
                mcycle <= mcycle + 64'd1;
            end
            
            if (instret_en) begin
                minstret <= minstret + 64'd1;
            end
            
            if (trap_enter) begin
                mepc <= trap_pc;
                mcause <= trap_cause;
                mtval <= trap_val;
                mstatus[7] <= mstatus[3];
                mstatus[3] <= 1'b0;
                mstatus[12:11] <= 2'b11;
            end else if (mret_exec) begin
                mstatus[3] <= mstatus[7];
                mstatus[7] <= 1'b1;
                mstatus[12:11] <= 2'b11;
            end else if (csr_write_en && (csr_op != 2'b00)) begin
                case (csr_write_addr)
                    12'h300: begin
                        mstatus[3] <= csr_write_data[3];
                        mstatus[7] <= csr_write_data[7];
                        mstatus[12:11] <= csr_write_data[12:11];
                    end
                    12'h304: mie <= csr_write_data & 32'h00000888;
                    12'h305: mtvec <= csr_write_data;
                    12'h340: mscratch <= csr_write_data;
                    12'h341: mepc <= csr_write_data;
                    12'h342: mcause <= csr_write_data;
                    12'h343: mtval <= csr_write_data;
                    12'hB00: mcycle[31:0] <= csr_write_data;
                    12'hB80: mcycle[63:32] <= csr_write_data;
                    12'hB02: minstret[31:0] <= csr_write_data;
                    12'hB82: minstret[63:32] <= csr_write_data;
                endcase
            end
        end
    end
    
endmodule