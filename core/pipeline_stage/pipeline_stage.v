//==================================================================================================
// File: pipeline_stage.v
//==================================================================================================
module instruction_fetch (
    input wire reset_n,
    input wire flush_temp, 
    input wire trap_enter, 
    input wire mret_exec,
    input wire [31:0] reset_vector_in,
    input wire [31:0] mtvec_in, 
    input wire [31:0] mepc_in,
    input wire [31:0] ex_mem_branch_target, 
    input wire [31:0] id_ex_jal_target, 
    input wire [31:0] pc_in, 
    input wire [31:0] ex_mem_pc_in,
    input wire id_ex_jalr, 
    input wire id_ex_jal, 
    input wire btb_hit,
    input wire [31:0] alu_in1, 
    input wire [31:0] id_ex_ext_imm,
    input wire predict_taken, 
    input wire actual_taken, 
    input wire bpu_correct,
    input wire [31:0] predict_target,
    output reg [31:0] pc_out, 
    output wire [31:0] pc_plus_4,
    output wire [31:0] instr,
    output wire icache_read_req,
    output wire [31:0] icache_addr,
    input wire [31:0] icache_read_data
);

    always @(*) begin
        if (!reset_n) begin
            pc_out = reset_vector_in;
        end else if (trap_enter) begin
            pc_out = mtvec_in;
        end else if (mret_exec) begin
            pc_out = mepc_in;
        end else if (!bpu_correct && actual_taken) begin
            pc_out = ex_mem_branch_target;
        end else if (!bpu_correct && !actual_taken) begin
            pc_out = ex_mem_pc_in + 32'd4;
        end else if (id_ex_jalr) begin
            pc_out = (alu_in1 + id_ex_ext_imm) & 32'hFFFFFFFE;
        end else if (id_ex_jal) begin
            pc_out = id_ex_jal_target;
        end else if (btb_hit && predict_taken) begin
            pc_out = predict_target;
        end else if (!flush_temp) begin
            pc_out = pc_in + 32'd4;
        end else begin
            pc_out = pc_in;
        end
    end
    
    assign icache_read_req = 1'b1;
    assign icache_addr = pc_in;
    assign instr = icache_read_data;
    assign pc_plus_4 = pc_in + 32'd4;

endmodule


module instruction_decode (
    input [31:0] if_id_pc_in,
    input [31:0] if_id_instr,
    output [31:0] ext_imm, 
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output reg [4:0] rd,
    output reg [2:0] funct3,
    output reg [6:0] opcode,
    output reg [6:0] funct7,
    output [31:0] jal_target,
    output [31:0] branch_target,
    output reg_write,
    output alu_src,
    output mem_write,
    output mem_read,
    output mem_to_reg,
    output branch,
    output jal,
    output jalr,
    output lui,
    output auipc,
    output mem_unsigned,
    output [1:0] alu_op,
    output [1:0] mem_size,
    output [3:0] alu_ctrl,
    output md_type,
    output [2:0] md_operation,
    output ecall,
    output ebreak,
    output mret,
    output [11:0] csr_addr,
    output [1:0] csr_op,
    output csr_we,
    output wire wfi_req,
    output fpu_en,
    output f_reg_write,
    output f_mem_to_reg,
    output f_mem_write,
    output f_to_x,
    output x_to_f,
    output [4:0] fpu_operation
);

    reg [19:0] u_imm;
    reg [11:0] i_imm;
    reg [11:0] s_imm;
    reg [11:0] b_imm;
    reg [19:0] j_imm;
    
    always @(*) begin 
        opcode = if_id_instr[6:0];
        funct3 = if_id_instr[14:12];
        funct7 = if_id_instr[31:25];
        rs1 = if_id_instr[19:15];
        rs2 = if_id_instr[24:20];
        rd = if_id_instr[11:7];
        u_imm = if_id_instr[31:12];
        i_imm = if_id_instr[31:20];
        s_imm = {if_id_instr[31:25], if_id_instr[11:7]};
        b_imm = {if_id_instr[31], if_id_instr[7], if_id_instr[30:25], if_id_instr[11:8]};
        j_imm = {if_id_instr[31], if_id_instr[19:12], if_id_instr[20], if_id_instr[30:21]};
    end
    
    wire [31:0] u_imm_ext = {u_imm, 12'b0};
    wire [31:0] i_imm_ext = {{20{i_imm[11]}}, i_imm};
    wire [31:0] s_imm_ext = {{20{s_imm[11]}}, s_imm};
    wire [31:0] b_imm_ext = {{19{b_imm[11]}}, b_imm, 1'b0};
    wire [31:0] j_imm_ext = {{11{j_imm[19]}}, j_imm, 1'b0};
    
    assign ext_imm = (opcode == 7'b0110111 || opcode == 7'b0010111) ? u_imm_ext :
                     (opcode == 7'b0000011 || opcode == 7'b0010011 || opcode == 7'b1100111 || opcode == 7'b0000111) ? i_imm_ext :
                     (opcode == 7'b0100011 || opcode == 7'b0100111) ? s_imm_ext :
                     (opcode == 7'b1100011) ? b_imm_ext :
                     (opcode == 7'b1101111) ? j_imm_ext :
                     32'b0;

    assign md_type = (opcode == 7'b0110011 && funct7 == 7'b0000001);

    assign jal_target = if_id_pc_in + j_imm_ext;
    
    assign branch_target = if_id_pc_in + b_imm_ext;
    
    wire is_system = (opcode == 7'b1110011);
    assign ecall = (if_id_instr == 32'h00000073);
    assign ebreak = (if_id_instr == 32'h00100073);
    assign mret = (if_id_instr == 32'h30200073);
    assign wfi_req = (if_id_instr == 32'h10500073);
    
    assign csr_addr = if_id_instr[31:20];
    assign csr_we = is_system && (funct3 != 3'b000);
    assign csr_op = (is_system && funct3 != 3'b000) ? funct3[1:0] : 2'b00;
    
    main_control_unit MCU (
        .opcode(opcode),
        .funct7(funct7),
        .funct3(funct3),
        .rs2(rs2),
        .reg_write(reg_write),
        .alu_src(alu_src),
        .mem_write(mem_write),
        .mem_read(mem_read),
        .mem_to_reg(mem_to_reg),
        .branch(branch),
        .jal(jal),
        .jalr(jalr),
        .lui(lui),
        .auipc(auipc),
        .mem_unsigned(mem_unsigned),
        .alu_op(alu_op),
        .mem_size(mem_size),
        .md_operation(md_operation),
        .fpu_en(fpu_en),
        .f_reg_write(f_reg_write),
        .f_mem_to_reg(f_mem_to_reg),
        .f_mem_write(f_mem_write),
        .f_to_x(f_to_x),
        .x_to_f(x_to_f),
        .fpu_operation(fpu_operation)
    );
    
    alu_control_unit ACU (
        .alu_op(alu_op),
        .funct3(funct3),
        .funct7(funct7),
        .opcode(opcode),
        .alu_ctrl(alu_ctrl)
    );
    
endmodule


module execute (
    input clk,
    input reset_n,
    input stall_id_ex,
    input [31:0] alu_in1,
    input [31:0] alu_in2,
    input [3:0] id_ex_alu_ctrl,
    input [2:0] id_ex_funct3,
    input id_ex_branch,
    input [31:0] id_ex_instr,
    input id_ex_lui,
    input id_ex_auipc,
    input id_ex_md_type,
    input [2:0] id_ex_md_operation,
    input [31:0] id_ex_pc_in,
    input [31:0] id_ex_ext_imm,
    input [1:0] id_ex_csr_op,
    input id_ex_csr_we,
    input [31:0] csr_read_data,
    input [4:0] id_ex_rs1,
    input id_ex_fpu_en,
    input [4:0] id_ex_fpu_operation,
    input [31:0] id_ex_read_f_data1,
    input [31:0] id_ex_read_f_data2,
    input id_ex_f_to_x,
    input id_ex_x_to_f,
    output reg [31:0] alu_result,
    output reg branch_taken,
    output reg [31:0] csr_write_data,
    output mf_alu_stall,
    output [31:0] fpu_result_out
);  

    wire [31:0] mul_result;
    wire [31:0] div_result;
    wire mul_alu_done;
    wire div_alu_done;
    wire mul_alu_stall;
    wire div_alu_stall;

    wire fpu_stall;
    wire fpu_done;
    wire [31:0] fpu_result;
    
    wire [31:0] fpu_operand_a = id_ex_x_to_f ? alu_in1 : id_ex_read_f_data1;

    multiplier MUL (
        .clk(clk),
        .reset_n(reset_n),
        .stall_id_ex(stall_id_ex),
        .md_type(id_ex_md_type),
        .alu_in1(alu_in1),
        .alu_in2(alu_in2),
        .md_operation(id_ex_md_operation),
        .md_result(mul_result),
        .md_alu_stall(mul_alu_stall),
        .md_alu_done(mul_alu_done)
    );

    divider DIV (
        .clk(clk),
        .reset_n(reset_n),
        .stall_id_ex(stall_id_ex),
        .md_type(id_ex_md_type),
        .alu_in1(alu_in1),
        .alu_in2(alu_in2),
        .md_operation(id_ex_md_operation),
        .md_result(div_result),
        .md_alu_stall(div_alu_stall),
        .md_alu_done(div_alu_done)
    );

    fpu_unit FPU (
        .clk(clk),
        .reset_n(reset_n),
        .stall_id_ex(stall_id_ex),
        .fpu_start(id_ex_fpu_en),
        .fpu_op(id_ex_fpu_operation),
        .operand_a(fpu_operand_a),
        .operand_b(id_ex_read_f_data2),
        .result(fpu_result),
        .fpu_stall(fpu_stall),
        .fpu_done(fpu_done)
    );

    assign fpu_result_out = fpu_result;
    assign mf_alu_stall = mul_alu_stall || div_alu_stall || fpu_stall;

    wire [31:0] csr_rs1_val = id_ex_funct3[2] ? {27'b0, id_ex_rs1} : alu_in1;

    always @(*) begin
        branch_taken = 1'b0;
        csr_write_data = 32'b0;
        
        if (id_ex_csr_we) begin
            alu_result = csr_read_data;
            case (id_ex_csr_op)
                2'b01: csr_write_data = csr_rs1_val;
                2'b10: csr_write_data = csr_read_data | csr_rs1_val;
                2'b11: csr_write_data = csr_read_data & ~csr_rs1_val;
                default: csr_write_data = csr_rs1_val;
            endcase
        end else if (id_ex_f_to_x) begin
            alu_result = fpu_result;
        end else if (id_ex_lui) begin
            alu_result = id_ex_ext_imm;
        end else if (id_ex_auipc) begin
            alu_result = id_ex_pc_in + id_ex_ext_imm;
        end else if (id_ex_md_type) begin
            if (mul_alu_done) begin
                alu_result = mul_result;
            end else if (div_alu_done) begin
                alu_result = div_result;
            end else begin
                alu_result = 32'd0;
            end
        end else begin 
            case (id_ex_alu_ctrl)
                4'b0000: alu_result = alu_in1 & alu_in2;  
                4'b0001: alu_result = alu_in1 | alu_in2;  
                4'b0010: alu_result = alu_in1 + alu_in2;  
                4'b0110: begin 
                    alu_result = alu_in1 - alu_in2;  
                    if (id_ex_branch) begin
                        case (id_ex_funct3)
                            3'b000: branch_taken = (alu_result == 32'd0); 
                            3'b001: branch_taken = (alu_result != 32'd0); 
                            3'b100: branch_taken = ($signed(alu_in1) < $signed(alu_in2)); 
                            3'b101: branch_taken = ($signed(alu_in1) >= $signed(alu_in2)); 
                            3'b110: branch_taken = (alu_in1 < alu_in2); 
                            3'b111: branch_taken = (alu_in1 >= alu_in2); 
                            default: branch_taken = 1'b0;
                        endcase
                    end
                end
                4'b0100: alu_result = alu_in1 ^ alu_in2;  
                4'b0111: begin
                    if ($signed(alu_in1) < $signed(alu_in2)) begin
                        alu_result = 32'd1;
                    end else begin
                        alu_result = 32'd0;
                    end
                end  
                4'b1010: begin
                    if (alu_in1 < alu_in2) begin
                        alu_result = 32'd1;
                    end else begin
                        alu_result = 32'd0;
                    end
                end  
                4'b1000: alu_result = alu_in1 << alu_in2[4:0];  
                4'b1001: alu_result = alu_in1 >> alu_in2[4:0];  
                4'b1011: alu_result = $signed(alu_in1) >>> alu_in2[4:0];  
                default: alu_result = alu_in1 + alu_in2;
            endcase
        end
    end
    
endmodule


module memory_access (
    input [31:0] ex_mem_alu_result,
    input [31:0] ex_mem_mem_write_data,
    input ex_mem_mem_write,
    input ex_mem_mem_read,
    output [31:0] mem_read_data,
    output dcache_read_req,
    output dcache_write_req,
    output [31:0] dcache_addr,
    output [31:0] dcache_write_data,
    input [31:0] dcache_read_data
);

    assign dcache_read_req = ex_mem_mem_read;
    assign dcache_write_req = ex_mem_mem_write;
    assign dcache_addr = ex_mem_alu_result;
    assign dcache_write_data = ex_mem_mem_write_data;
    assign mem_read_data = dcache_read_data;
    
endmodule


module write_back (
    input [31:0] mem_wb_mem_read_data,
    input [31:0] mem_wb_alu_result,
    input [31:0] mem_wb_pc_plus_4,
    input mem_wb_mem_to_reg,
    input mem_wb_jal,
    output [31:0] mem_wb_write_data
);

    assign mem_wb_write_data = (mem_wb_jal) ? mem_wb_pc_plus_4 :
                                mem_wb_mem_to_reg ? mem_wb_mem_read_data : mem_wb_alu_result;
                                
endmodule