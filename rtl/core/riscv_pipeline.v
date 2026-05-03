`timescale 1ns / 1ps

module riscv_pipeline (
    input clk,
    input reset_n,  // Reset mức thấp
    input riscv_start,
    input external_irq_in,
    input [31:0] reset_vector_in,  // Sửa: Thêm chân reset vector
    output reg riscv_done,
    
    // ICache interface
    output icache_read_req,
    output [31:0] icache_addr,
    input [31:0] icache_read_data,
    input icache_hit,
    input icache_stall,
    // output để branch predict
    output bpu_predict_taken_out,
    output bpu_btb_hit_out,
    output [31:0] bpu_predict_target_out,

    
    // DCache interface
    output dcache_read_req,
    output dcache_write_req,
    output [31:0] dcache_addr,
    output [31:0] dcache_write_data,
    input [31:0] dcache_read_data,
    input dcache_hit,
    input dcache_stall,

    output flush_top,
    output [1:0] mem_size_top,
    output mem_unsigned_top,

    output wfi_sleep_out

);

    // Tự động bỏ qua việc tái định nghĩa tất cả các Wire nội bộ bên dưới do dài, vẫn giữ nguyên cấu trúc
    wire [31:0] pc_in;
    wire [31:0] pc_out;
    wire [31:0] pc_plus_4;
    wire [31:0] instr;
    wire predict_taken;
    wire bpu_correct;
    wire btb_hit;
    wire actual_taken;
    wire [31:0] predict_target;
    wire [31:0] if_id_instr;
    wire [31:0] if_id_pc_plus_4;
    wire [31:0] if_id_pc_in;
    wire if_id_predict_taken;
    wire if_id_btb_hit;
    wire [31:0] read_data1;
    wire [31:0] read_data2;
    wire [31:0] ext_imm;
    wire [4:0] rs1;
    wire [4:0] rs2;
    wire [4:0] rd;
    wire [2:0] funct3;
    wire [6:0] opcode;
    wire [6:0] funct7;
    wire [31:0] jal_target;
    wire [31:0] branch_target;
    wire reg_write;
    wire alu_src;
    wire mem_write;
    wire mem_read;
    wire mem_to_reg;
    wire branch;
    wire jal;
    wire jalr;
    wire lui;
    wire auipc;
    wire mem_unsigned;
    wire [1:0] alu_op;
    wire [1:0] mem_size;
    wire [3:0] alu_ctrl;
    wire ecall;
    wire ebreak;
    wire mret; 
    wire [11:0] csr_addr; 
    wire [1:0] csr_op; 
    wire csr_we;
    wire md_type;
    wire [2:0] md_operation;
    wire fpu_en;
    wire f_reg_write;
    wire f_mem_to_reg;
    wire f_mem_write;
    wire f_to_x;
    wire x_to_f;
    wire [4:0] fpu_operation;
    wire [31:0] id_ex_pc_plus_4;
    wire [31:0] id_ex_pc_in;
    wire [31:0] id_ex_instr;
    wire [31:0] id_ex_read_data1;
    wire [31:0] id_ex_read_data2;
    wire [31:0] id_ex_ext_imm;
    wire [4:0] id_ex_rs1;
    wire [4:0] id_ex_rs2;
    wire [4:0] id_ex_rd;
    wire [2:0] id_ex_funct3;
    wire id_ex_reg_write;
    wire id_ex_alu_src;
    wire id_ex_mem_write;
    wire id_ex_mem_read;
    wire id_ex_mem_to_reg;
    wire id_ex_branch;
    wire id_ex_jal;
    wire id_ex_jalr;
    wire id_ex_lui;
    wire id_ex_auipc;
    wire id_ex_mem_unsigned;
    wire [1:0] id_ex_mem_size;
    wire [3:0] id_ex_alu_ctrl;
    wire [31:0] id_ex_branch_target;
    wire [31:0] id_ex_jal_target;
    wire id_ex_predict_taken;
    wire id_ex_btb_hit;
    wire id_ex_ecall;
    wire id_ex_ebreak;
    wire id_ex_mret; 
    wire [11:0] id_ex_csr_addr; 
    wire [1:0] id_ex_csr_op; 
    wire id_ex_csr_we;
    wire id_ex_md_type;
    wire [2:0] id_ex_md_operation;
    wire id_ex_fpu_en;
    wire id_ex_f_reg_write;
    wire id_ex_f_mem_to_reg;
    wire id_ex_f_mem_write;
    wire id_ex_f_to_x;
    wire id_ex_x_to_f;
    wire [4:0] id_ex_fpu_operation;
    wire [31:0] id_ex_read_f_data1;
    wire [31:0] id_ex_read_f_data2;
    wire [31:0] alu_in1;
    wire [31:0] alu_in2;
    wire [31:0] mem_write_data;
    wire [31:0] csr_write_data_ex;
    wire [31:0] fpu_in1;
    wire [31:0] fpu_in2;
    wire [31:0] alu_result;
    wire branch_taken;
    wire md_alu_stall;
    wire [31:0] fpu_result_out;
    wire [31:0] ex_mem_instr;
    wire [31:0] ex_mem_alu_result;
    wire [31:0] ex_mem_mem_write_data;
    wire [31:0] ex_mem_branch_target;
    wire [31:0] ex_mem_pc_plus_4;
    wire [31:0] ex_mem_pc_in;
    wire [4:0] ex_mem_rd;
    wire ex_mem_mem_write;
    wire ex_mem_mem_read;
    wire ex_mem_mem_to_reg;
    wire ex_mem_branch;
    wire ex_mem_branch_taken;
    wire ex_mem_jal;
    wire ex_mem_mem_unsigned;
    wire ex_mem_reg_write;
    wire [1:0] ex_mem_mem_size;
    wire ex_mem_predict_taken;
    wire ex_mem_btb_hit;
    wire ex_mem_ecall;
    wire ex_mem_ebreak;
    wire ex_mem_mret;
    wire [11:0] ex_mem_csr_addr; 
    wire [1:0] ex_mem_csr_op; 
    wire ex_mem_csr_we; 
    wire [31:0] ex_mem_csr_write_data;
    wire [31:0] ex_mem_fpu_result;
    wire [31:0] ex_mem_f_store_data;
    wire ex_mem_f_reg_write;
    wire ex_mem_f_mem_to_reg;
    wire ex_mem_f_mem_write;
    wire [31:0] mem_read_data;
    wire [31:0] mem_wb_mem_read_data;
    wire [31:0] mem_wb_alu_result;
    wire [31:0] mem_wb_pc_plus_4;
    wire mem_wb_mem_to_reg;
    wire mem_wb_reg_write;
    wire mem_wb_jal;
    wire [4:0] mem_wb_rd;
    wire mem_wb_ecall;
    wire [31:0] mem_wb_fpu_result;
    wire mem_wb_f_reg_write;
    wire mem_wb_f_mem_to_reg;
    wire [31:0] mem_wb_write_data;
    wire [31:0] wb_f_write_data;
    wire load_use_stall;
    wire flush_branch;
    wire flush_jal;
    wire flush_trap;
    wire [31:0] read_data1_temp;
    wire [31:0] read_data2_temp;
    wire [31:0] read_f_data1_temp;
    wire [31:0] read_f_data2_temp;
    wire [31:0] read_f_data1;
    wire [31:0] read_f_data2;
    wire [31:0] mie_val;
    wire mstatus_mie_val;

    wire is_external_interrupt = external_irq_in & mie_val[11] & mstatus_mie_val;
    wire trap_enter = ex_mem_ecall | ex_mem_ebreak | is_external_interrupt;
    
    wire [31:0] trap_cause = is_external_interrupt ? 32'h8000000b :
                             ex_mem_ecall           ? 32'd11       :
                             ex_mem_ebreak          ? 32'd3        : 32'd0;

    wire [31:0] mtvec_pc;
    wire [31:0] mepc_pc;
    wire [31:0] csr_read_data_raw;
    wire [31:0] csr_read_data_fwd;

    wire wfi_req_internal;

    assign csr_read_data_fwd = (ex_mem_csr_we && (ex_mem_csr_addr == id_ex_csr_addr)) ? ex_mem_csr_write_data : csr_read_data_raw;

    csr_register_file CSR_RF (
        .clk(clk),
        .reset_n(reset_n),
        .csr_addr(id_ex_csr_addr),
        .csr_read_data(csr_read_data_raw),
        .csr_write_addr(ex_mem_csr_addr),
        .csr_write_data(ex_mem_csr_write_data),
        .csr_op(ex_mem_csr_op),
        .csr_write_en(ex_mem_csr_we),
        .count_en(1'b1),
        .instret_en(!dcache_stall && !icache_stall && !md_alu_stall && !flush_trap && !flush_branch),
        .trap_enter(trap_enter),
        .mret_exec(ex_mem_mret),
        .trap_cause(trap_cause),
        .trap_pc(ex_mem_pc_in),
        .trap_val(32'd0),
        .mtvec_out(mtvec_pc),
        .mepc_out(mepc_pc),
        .mie_out(mie_val),
        .mstatus_mie(mstatus_mie_val)
    );

    reg [31:0] pc_reg;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pc_reg <= reset_vector_in;
        end else if (riscv_start && !riscv_done) begin
            if (flush_branch || flush_jal) begin // Add flush trap ----------------------
                pc_reg <= pc_out;
            end else if (!load_use_stall && !icache_stall && !dcache_stall && !md_alu_stall) begin
                pc_reg <= pc_out;
            end
        end
    end
    
    assign pc_in = pc_reg;

    reg flush_temp;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            flush_temp <= 1'b0;
        end else if (riscv_start && !riscv_done) begin
            flush_temp <= flush_branch || flush_jal;
        end
    end

    assign flush_top = flush_temp;
    assign mem_size_top = ex_mem_mem_size;
    assign mem_unsigned_top = ex_mem_mem_unsigned;

    // Instruction Fetch
    instruction_fetch IF (
        .reset_n(reset_n),
        .flush_temp(flush_temp),
        .trap_enter(trap_enter),
        .mret_exec(ex_mem_mret),
        .reset_vector_in(reset_vector_in), // Sửa: Dùng port mapping thay vì 32'h0
        .mtvec_in(mtvec_pc),
        .mepc_in(mepc_pc),
        .ex_mem_branch_target(ex_mem_branch_target),
        .id_ex_jal_target(id_ex_jal_target),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .id_ex_jalr(id_ex_jalr),
        .id_ex_jal(id_ex_jal),
        .btb_hit(btb_hit),
        .alu_in1(alu_in1),
        .id_ex_ext_imm(id_ex_ext_imm),
        .predict_taken(predict_taken),
        .actual_taken(actual_taken),
        .bpu_correct(bpu_correct),
        .predict_target(predict_target),
        .pc_out(pc_out),
        .pc_plus_4(pc_plus_4),
        .instr(instr),
        .icache_read_req(icache_read_req),
        .icache_addr(icache_addr),
        .icache_read_data(icache_read_data)
    );

    // Phần pipeline phía sau giữ nguyên, nối bình thường do không liên quan tới lỗi
    if_id_register IF_ID ( .clk(clk), .reset_n(reset_n), .icache_stall(icache_stall), .dcache_stall(dcache_stall), .md_alu_stall(md_alu_stall), .load_use_stall(load_use_stall), .flush(flush_branch | flush_jal | flush_trap), .riscv_start(riscv_start), .riscv_done(riscv_done), .instr(instr), .pc_plus_4(pc_plus_4), .pc_in(pc_in), .predict_taken(predict_taken), .btb_hit(btb_hit), .if_id_instr(if_id_instr), .if_id_pc_plus_4(if_id_pc_plus_4), .if_id_pc_in(if_id_pc_in), .if_id_predict_taken(if_id_predict_taken), .if_id_btb_hit(if_id_btb_hit) );
    
    instruction_decode ID ( .if_id_pc_in(if_id_pc_in), .if_id_instr(if_id_instr), .ext_imm(ext_imm), .rs1(rs1), .rs2(rs2), .rd(rd), .funct3(funct3), .opcode(opcode), .funct7(funct7), .jal_target(jal_target), .branch_target(branch_target), .reg_write(reg_write), .alu_src(alu_src), .mem_write(mem_write), .mem_read(mem_read), .mem_to_reg(mem_to_reg), .branch(branch), .jal(jal), .jalr(jalr), .lui(lui), .auipc(auipc), .mem_unsigned(mem_unsigned), .alu_op(alu_op), .mem_size(mem_size), .alu_ctrl(alu_ctrl), .md_type(md_type), .md_operation(md_operation), .ecall(ecall), .ebreak(ebreak), .mret(mret), .csr_addr(csr_addr), .csr_op(csr_op), .csr_we(csr_we), .fpu_en(fpu_en), .f_reg_write(f_reg_write), .f_mem_to_reg(f_mem_to_reg), .f_mem_write(f_mem_write), .f_to_x(f_to_x), .x_to_f(x_to_f), .fpu_operation(fpu_operation), .wfi_req(wfi_req_internal) );
    
    assign wfi_sleep_out = wfi_req_internal;
    
    register_file RF ( .clk(clk), .reset_n(reset_n), .read_reg1(rs1), .read_reg2(rs2), .mem_wb_reg_write(mem_wb_reg_write), .mem_wb_rd(mem_wb_rd), .mem_wb_write_data(mem_wb_write_data), .read_data1(read_data1_temp),  .read_data2(read_data2_temp) );
    assign read_data1 = (rs1 != 5'd0 && rs1 == mem_wb_rd && mem_wb_reg_write) ? mem_wb_write_data : read_data1_temp;
    assign read_data2 = (rs2 != 5'd0 && rs2 == mem_wb_rd && mem_wb_reg_write) ? mem_wb_write_data : read_data2_temp;
    
    f_register_file F_RF ( .clk(clk), .reset_n(reset_n), .read_reg1(rs1), .read_reg2(rs2), .read_data1(read_f_data1_temp), .read_data2(read_f_data2_temp), .reg_write_en(mem_wb_f_reg_write), .write_reg(mem_wb_rd), .write_data(wb_f_write_data) );
    assign read_f_data1 = (rs1 == mem_wb_rd && mem_wb_f_reg_write) ? wb_f_write_data : read_f_data1_temp;
    assign read_f_data2 = (rs2 == mem_wb_rd && mem_wb_f_reg_write) ? wb_f_write_data : read_f_data2_temp;
    
    id_ex_register ID_EX ( .clk(clk), .reset_n(reset_n), .dcache_stall(dcache_stall), .md_alu_stall(md_alu_stall), .load_use_stall(load_use_stall), .flush(flush_branch | flush_jal | flush_trap), .riscv_start(riscv_start), .riscv_done(riscv_done), .if_id_pc_plus_4(if_id_pc_plus_4), .if_id_pc_in(if_id_pc_in), .funct3(funct3), .read_data1(read_data1), .read_data2(read_data2), .ext_imm(ext_imm), .rs1(rs1), .rs2(rs2), .rd(rd), .reg_write(reg_write), .alu_src(alu_src), .mem_write(mem_write), .mem_read(mem_read), .mem_to_reg(mem_to_reg), .branch(branch), .jal(jal), .jalr(jalr), .lui(lui), .auipc(auipc), .mem_unsigned(mem_unsigned), .mem_size(mem_size), .alu_ctrl(alu_ctrl), .branch_target(branch_target), .jal_target(jal_target), .if_id_predict_taken(if_id_predict_taken), .if_id_btb_hit(if_id_btb_hit), .ecall(ecall), .ebreak(ebreak), .mret(mret), .csr_addr(csr_addr), .csr_op(csr_op), .csr_we(csr_we), .md_type(md_type), .md_operation(md_operation), .if_id_instr(if_id_instr), .fpu_en(fpu_en), .f_reg_write(f_reg_write), .f_mem_to_reg(f_mem_to_reg), .f_mem_write(f_mem_write), .f_to_x(f_to_x), .x_to_f(x_to_f), .fpu_operation(fpu_operation), .read_f_data1(read_f_data1), .read_f_data2(read_f_data2), .id_ex_pc_plus_4(id_ex_pc_plus_4), .id_ex_pc_in(id_ex_pc_in), .id_ex_funct3(id_ex_funct3), .id_ex_read_data1(id_ex_read_data1), .id_ex_read_data2(id_ex_read_data2), .id_ex_ext_imm(id_ex_ext_imm), .id_ex_rs1(id_ex_rs1), .id_ex_rs2(id_ex_rs2), .id_ex_rd(id_ex_rd), .id_ex_reg_write(id_ex_reg_write), .id_ex_alu_src(id_ex_alu_src), .id_ex_mem_write(id_ex_mem_write), .id_ex_mem_read(id_ex_mem_read), .id_ex_mem_to_reg(id_ex_mem_to_reg), .id_ex_branch(id_ex_branch), .id_ex_jal(id_ex_jal), .id_ex_jalr(id_ex_jalr), .id_ex_lui(id_ex_lui), .id_ex_auipc(id_ex_auipc), .id_ex_mem_unsigned(id_ex_mem_unsigned), .id_ex_mem_size(id_ex_mem_size), .id_ex_alu_ctrl(id_ex_alu_ctrl), .id_ex_branch_target(id_ex_branch_target), .id_ex_jal_target(id_ex_jal_target), .id_ex_predict_taken(id_ex_predict_taken), .id_ex_btb_hit(id_ex_btb_hit), .id_ex_ecall(id_ex_ecall), .id_ex_ebreak(id_ex_ebreak), .id_ex_mret(id_ex_mret), .id_ex_csr_addr(id_ex_csr_addr), .id_ex_csr_op(id_ex_csr_op), .id_ex_csr_we(id_ex_csr_we), .id_ex_md_type(id_ex_md_type), .id_ex_md_operation(id_ex_md_operation), .id_ex_instr(id_ex_instr), .id_ex_fpu_en(id_ex_fpu_en), .id_ex_f_reg_write(id_ex_f_reg_write), .id_ex_f_mem_to_reg(id_ex_f_mem_to_reg), .id_ex_f_mem_write(id_ex_f_mem_write), .id_ex_f_to_x(id_ex_f_to_x), .id_ex_x_to_f(id_ex_x_to_f), .id_ex_fpu_operation(id_ex_fpu_operation), .id_ex_read_f_data1(id_ex_read_f_data1), .id_ex_read_f_data2(id_ex_read_f_data2) );
    
    forwarding_unit FU ( .id_ex_read_data1(id_ex_read_data1), .id_ex_read_data2(id_ex_read_data2), .id_ex_ext_imm(id_ex_ext_imm), .id_ex_rs1(id_ex_rs1), .id_ex_rs2(id_ex_rs2), .ex_mem_reg_write(ex_mem_reg_write), .mem_wb_reg_write(mem_wb_reg_write), .id_ex_alu_src(id_ex_alu_src), .ex_mem_rd(ex_mem_rd), .mem_wb_rd(mem_wb_rd), .ex_mem_alu_result(ex_mem_alu_result), .mem_wb_write_data(mem_wb_write_data), .alu_in1(alu_in1), .alu_in2(alu_in2), .mem_write_data(mem_write_data), .id_ex_read_f_data1(id_ex_read_f_data1), .id_ex_read_f_data2(id_ex_read_f_data2), .ex_mem_f_reg_write(ex_mem_f_reg_write), .mem_wb_f_reg_write(mem_wb_f_reg_write), .ex_mem_fpu_result(ex_mem_fpu_result), .mem_wb_f_write_data(wb_f_write_data), .fpu_in1(fpu_in1), .fpu_in2(fpu_in2) );
    
    execute EX ( .clk(clk), .reset_n(reset_n), .alu_in1(alu_in1), .alu_in2(alu_in2), .id_ex_alu_ctrl(id_ex_alu_ctrl), .id_ex_branch(id_ex_branch), .id_ex_instr(id_ex_instr), .id_ex_funct3(id_ex_funct3), .id_ex_lui(id_ex_lui), .id_ex_auipc(id_ex_auipc), .id_ex_md_type(id_ex_md_type), .id_ex_md_operation(id_ex_md_operation), .id_ex_pc_in(id_ex_pc_in), .id_ex_ext_imm(id_ex_ext_imm), .id_ex_csr_op(id_ex_csr_op), .id_ex_csr_we(id_ex_csr_we), .csr_read_data(csr_read_data_fwd), .id_ex_rs1(id_ex_rs1), .id_ex_fpu_en(id_ex_fpu_en), .id_ex_fpu_operation(id_ex_fpu_operation), .id_ex_read_f_data1(fpu_in1), .id_ex_read_f_data2(fpu_in2), .id_ex_f_to_x(id_ex_f_to_x), .id_ex_x_to_f(id_ex_x_to_f), .alu_result(alu_result), .branch_taken(branch_taken), .csr_write_data(csr_write_data_ex), .md_alu_stall(md_alu_stall), .fpu_result_out(fpu_result_out) );
    
    ex_mem_register EX_MEM ( .clk(clk), .reset_n(reset_n), .dcache_stall(dcache_stall), .md_alu_stall(md_alu_stall), .flush(flush_branch | flush_trap), .riscv_start(riscv_start), .riscv_done(riscv_done), .alu_result(alu_result), .id_ex_ext_imm(id_ex_ext_imm), .id_ex_rd(id_ex_rd), .id_ex_pc_plus_4(id_ex_pc_plus_4), .id_ex_pc_in(id_ex_pc_in), .id_ex_branch_target(id_ex_branch_target), .id_ex_mem_write(id_ex_mem_write), .id_ex_mem_read(id_ex_mem_read), .id_ex_mem_to_reg(id_ex_mem_to_reg), .id_ex_reg_write(id_ex_reg_write), .id_ex_branch(id_ex_branch), .branch_taken(branch_taken), .id_ex_jal(id_ex_jal), .id_ex_mem_unsigned(id_ex_mem_unsigned), .id_ex_mem_size(id_ex_mem_size), .id_ex_read_data2(id_ex_read_data2), .mem_write_data(mem_write_data), .id_ex_predict_taken(id_ex_predict_taken), .id_ex_btb_hit(id_ex_btb_hit), .id_ex_ecall(id_ex_ecall), .id_ex_ebreak(id_ex_ebreak), .id_ex_mret(id_ex_mret), .id_ex_csr_addr(id_ex_csr_addr), .id_ex_csr_op(id_ex_csr_op), .id_ex_csr_we(id_ex_csr_we), .csr_write_data_in(csr_write_data_ex), .id_ex_instr(id_ex_instr), .fpu_result(fpu_result_out), .id_ex_read_f_data2(fpu_in2), .id_ex_f_reg_write(id_ex_f_reg_write), .id_ex_f_mem_to_reg(id_ex_f_mem_to_reg), .id_ex_f_mem_write(id_ex_f_mem_write), .ex_mem_alu_result(ex_mem_alu_result), .ex_mem_rd(ex_mem_rd), .ex_mem_branch_target(ex_mem_branch_target), .ex_mem_pc_plus_4(ex_mem_pc_plus_4), .ex_mem_pc_in(ex_mem_pc_in), .ex_mem_mem_write(ex_mem_mem_write), .ex_mem_mem_read(ex_mem_mem_read), .ex_mem_mem_to_reg(ex_mem_mem_to_reg), .ex_mem_reg_write(ex_mem_reg_write), .ex_mem_branch(ex_mem_branch), .ex_mem_branch_taken(ex_mem_branch_taken), .ex_mem_jal(ex_mem_jal), .ex_mem_mem_unsigned(ex_mem_mem_unsigned), .ex_mem_mem_size(ex_mem_mem_size), .ex_mem_mem_write_data(ex_mem_mem_write_data), .ex_mem_predict_taken(ex_mem_predict_taken), .ex_mem_btb_hit(ex_mem_btb_hit), .ex_mem_ecall(ex_mem_ecall), .ex_mem_ebreak(ex_mem_ebreak), .ex_mem_mret(ex_mem_mret), .ex_mem_csr_addr(ex_mem_csr_addr), .ex_mem_csr_op(ex_mem_csr_op), .ex_mem_csr_we(ex_mem_csr_we), .ex_mem_csr_write_data(ex_mem_csr_write_data), .ex_mem_instr(ex_mem_instr), .ex_mem_fpu_result(ex_mem_fpu_result), .ex_mem_f_store_data(ex_mem_f_store_data), .ex_mem_f_reg_write(ex_mem_f_reg_write), .ex_mem_f_mem_to_reg(ex_mem_f_mem_to_reg), .ex_mem_f_mem_write(ex_mem_f_mem_write) );
    
    wire [31:0] final_mem_write_data = ex_mem_f_mem_write ? ex_mem_f_store_data : ex_mem_mem_write_data;
    
    memory_access MEM ( .ex_mem_alu_result(ex_mem_alu_result), .ex_mem_mem_write_data(final_mem_write_data), .ex_mem_mem_write(ex_mem_mem_write | ex_mem_f_mem_write), .ex_mem_mem_read(ex_mem_mem_read), .mem_read_data(mem_read_data), .dcache_read_req(dcache_read_req), .dcache_write_req(dcache_write_req), .dcache_addr(dcache_addr), .dcache_write_data(dcache_write_data), .dcache_read_data(dcache_read_data) );
    
    mem_wb_register MEM_WB ( .clk(clk), .reset_n(reset_n), .dcache_stall(dcache_stall), .riscv_start(riscv_start), .riscv_done(riscv_done), .mem_read_data(mem_read_data), .ex_mem_pc_plus_4(ex_mem_pc_plus_4), .ex_mem_mem_to_reg(ex_mem_mem_to_reg), .ex_mem_reg_write(ex_mem_reg_write), .ex_mem_jal(ex_mem_jal), .ex_mem_alu_result(ex_mem_alu_result), .ex_mem_rd(ex_mem_rd), .ex_mem_ecall(ex_mem_ecall), .ex_mem_fpu_result(ex_mem_fpu_result), .ex_mem_f_reg_write(ex_mem_f_reg_write), .ex_mem_f_mem_to_reg(ex_mem_f_mem_to_reg), .mem_wb_mem_read_data(mem_wb_mem_read_data), .mem_wb_pc_plus_4(mem_wb_pc_plus_4), .mem_wb_mem_to_reg(mem_wb_mem_to_reg), .mem_wb_reg_write(mem_wb_reg_write), .mem_wb_jal(mem_wb_jal), .mem_wb_alu_result(mem_wb_alu_result), .mem_wb_rd(mem_wb_rd), .mem_wb_ecall(mem_wb_ecall), .mem_wb_fpu_result(mem_wb_fpu_result), .mem_wb_f_reg_write(mem_wb_f_reg_write), .mem_wb_f_mem_to_reg(mem_wb_f_mem_to_reg) );
    
    write_back WB ( .mem_wb_mem_read_data(mem_wb_mem_read_data), .mem_wb_alu_result(mem_wb_alu_result), .mem_wb_pc_plus_4(mem_wb_pc_plus_4), .mem_wb_mem_to_reg(mem_wb_mem_to_reg), .mem_wb_jal(mem_wb_jal), .mem_wb_write_data(mem_wb_write_data) );
    
    assign wb_f_write_data = mem_wb_f_mem_to_reg ? mem_wb_mem_read_data : mem_wb_fpu_result;
    
    pipeline_control_unit PCU ( .opcode(opcode), .funct3(funct3), .rs1(rs1), .rs2(rs2), .id_ex_mem_read(id_ex_mem_read), .id_ex_jal(id_ex_jal), .id_ex_jalr(id_ex_jalr), .id_ex_rd(id_ex_rd), .bpu_correct(bpu_correct), .trap_enter(trap_enter), .mret_exec(ex_mem_mret), .load_use_stall(load_use_stall), .flush_branch(flush_branch), .flush_jal(flush_jal), .flush_trap(flush_trap) );
    
    branch_prediction_unit BPU ( .clk(clk), .reset_n(reset_n), .pc_in(pc_in), .ex_mem_pc_in(ex_mem_pc_in), .ex_mem_branch(ex_mem_branch), .ex_mem_branch_taken(ex_mem_branch_taken), .ex_mem_predict_taken(ex_mem_predict_taken), .ex_mem_btb_hit(ex_mem_btb_hit), .ex_mem_branch_target(ex_mem_branch_target), .bpu_correct(bpu_correct), .predict_taken(predict_taken), .btb_hit(btb_hit), .actual_taken(actual_taken), .predict_target(predict_target) );
    assign bpu_predict_taken_out  = predict_taken;
    assign bpu_btb_hit_out        = btb_hit;
    assign bpu_predict_target_out = predict_target;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            riscv_done <= 1'b0;
        end else if (riscv_start) begin
            if (mem_wb_ecall) begin
                riscv_done <= 1'b1;
            end
        end
    end
    
endmodule