//==================================================================================================
// File: forwarding_unit.v
//==================================================================================================
module forwarding_unit (
    input [31:0] id_ex_read_data1,
    input [31:0] id_ex_read_data2,
    input [31:0] id_ex_ext_imm,
    input [4:0] id_ex_rs1,
    input [4:0] id_ex_rs2,
    input ex_mem_reg_write,
    input mem_wb_reg_write,
    input id_ex_alu_src,
    input [4:0] ex_mem_rd,
    input [4:0] mem_wb_rd,
    input [31:0] ex_mem_alu_result,
    input [31:0] mem_wb_write_data,
    input [31:0] id_ex_read_f_data1,
    input [31:0] id_ex_read_f_data2,
    input ex_mem_f_reg_write,
    input mem_wb_f_reg_write,
    input [31:0] ex_mem_fpu_result,
    input [31:0] mem_wb_f_write_data,
    output [31:0] alu_in1,
    output [31:0] alu_in2,
    output [31:0] mem_write_data,
    output [31:0] fpu_in1,
    output [31:0] fpu_in2
);
    reg [1:0] forward_a;
    reg [1:0] forward_b;
    
    always @(*) begin
        forward_a = 2'b00;
        forward_b = 2'b00;
        
        if (ex_mem_reg_write && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs1)) begin
            forward_a = 2'b10;
        end else if (mem_wb_reg_write && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs1) && 
                !(ex_mem_reg_write && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs1))) begin
            forward_a = 2'b01;
        end
        
        if (ex_mem_reg_write && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs2)) begin
            forward_b = 2'b10;
        end else if (mem_wb_reg_write && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs2) && 
                !(ex_mem_reg_write && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs2))) begin
            forward_b = 2'b01;
        end
    end
    
    assign alu_in1 = (forward_a == 2'b00) ? id_ex_read_data1 :
                     (forward_a == 2'b01) ? mem_wb_write_data :
                     ex_mem_alu_result;
                     
    assign alu_in2 = (id_ex_alu_src) ? id_ex_ext_imm : mem_write_data;
                         
    assign mem_write_data = (forward_b == 2'b00) ? id_ex_read_data2 :
                            (forward_b == 2'b01) ? mem_wb_write_data :
                            ex_mem_alu_result;

    reg [1:0] forward_f_a;
    reg [1:0] forward_f_b;

    always @(*) begin
        forward_f_a = 2'b00;
        forward_f_b = 2'b00;
        
        if (ex_mem_f_reg_write && (ex_mem_rd == id_ex_rs1)) begin
            forward_f_a = 2'b10;
        end else if (mem_wb_f_reg_write && (mem_wb_rd == id_ex_rs1)) begin
            forward_f_a = 2'b01;
        end
        
        if (ex_mem_f_reg_write && (ex_mem_rd == id_ex_rs2)) begin
            forward_f_b = 2'b10;
        end else if (mem_wb_f_reg_write && (mem_wb_rd == id_ex_rs2)) begin
            forward_f_b = 2'b01;
        end
    end

    assign fpu_in1 = (forward_f_a == 2'b00) ? id_ex_read_f_data1 :
                     (forward_f_a == 2'b10) ? ex_mem_fpu_result : mem_wb_f_write_data;
                     
    assign fpu_in2 = (forward_f_b == 2'b00) ? id_ex_read_f_data2 :
                     (forward_f_b == 2'b10) ? ex_mem_fpu_result : mem_wb_f_write_data;
                     
endmodule