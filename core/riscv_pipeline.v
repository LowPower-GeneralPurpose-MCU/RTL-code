`timescale 1ns / 1ps

// Scalar RV32IMAC pipeline.
//
// The old top-level was a two-lane superscalar shell with ROB/Tomasulo hooks.
// This replacement keeps the public ports compatible with the existing project,
// but only issues lane 0. Lane 1 fetch/cache ports are tied off.
module riscv_pipeline #(
    parameter ENABLE_TOMASULO_INTEGER = 0
)(
    input  wire        clk,
    input  wire        reset_n,
    input  wire        riscv_start,
    input  wire        meip_i,
    input  wire        msip_i,
    input  wire        mtip_i,
    input  wire [31:0] reset_vector_in,
    output reg         riscv_done,

    // ICache interface
    output wire        icache_read_req,
    output wire [31:0] icache_addr,
    input  wire [31:0] icache_read_data,
    input  wire        icache_hit,
    input  wire        icache_stall,
    output wire        icache_read_req_lane1,
    output wire [31:0] icache_addr_lane1,
    input  wire [31:0] icache_read_data_lane1,
    input  wire        icache_hit_lane1,
    input  wire        icache_stall_lane1,

    // DCache interface
    output wire        dcache_read_req,
    output wire        dcache_write_req,
    output wire [31:0] dcache_addr,
    output wire [31:0] dcache_write_data,
    input  wire [31:0] dcache_read_data,
    input  wire        dcache_hit,
    input  wire        dcache_stall,

    output wire [1:0]  mem_size_top,
    output wire        mem_unsigned_top,

    output wire        wfi_sleep_out,

    // Debug Module Interface
    input  wire        dbg_halt_req,
    input  wire        dbg_resume_req,
    output wire        dbg_halted,

    input  wire [15:0] dbg_reg_read_addr,
    output wire [31:0] dbg_reg_read_data,
    input  wire        dbg_reg_write_en,
    input  wire [15:0] dbg_reg_write_addr,
    input  wire [31:0] dbg_reg_write_data
);

    localparam ROB_TAG_W = 1;
    localparam [31:0] NOP = 32'h00000013;

    // Lane 1 is intentionally disabled in scalar mode.
    assign icache_read_req_lane1 = 1'b0;
    assign icache_addr_lane1     = 32'd0;

    // =========================================================================
    // IF wires and PC
    // =========================================================================
    wire [31:0] pc_in;
    wire [31:0] pc_out;
    wire [31:0] pc_plus_4;
    wire [31:0] pc_plus_8_unused;
    wire [31:0] instr;
    wire [31:0] instr_lane1_unused;
    wire        if_lane1_req_unused;
    wire [31:0] if_lane1_addr_unused;

    reg [31:0] pc_reg;
    reg        flush_temp;

    // =========================================================================
    // IF/ID
    // =========================================================================
    wire [31:0] if_id_instr;
    wire [31:0] if_id_pc_plus_4;
    wire [31:0] if_id_pc_in;
    wire        if_id_predict_taken;
    wire        if_id_btb_hit;

    // =========================================================================
    // ID
    // =========================================================================
    wire [31:0] read_data1;
    wire [31:0] read_data2;
    wire [31:0] read_data1_temp;
    wire [31:0] read_data2_temp;
    wire [31:0] ext_imm;
    wire [4:0]  rs1;
    wire [4:0]  rs2;
    wire [4:0]  rd;
    wire [2:0]  funct3;
    wire [6:0]  opcode;
    wire [6:0]  funct7;
    wire [31:0] jal_target;
    wire [31:0] branch_target;
    wire        reg_write;
    wire        alu_src;
    wire        mem_write;
    wire        mem_read;
    wire        mem_to_reg;
    wire        branch;
    wire        jal;
    wire        jalr;
    wire        lui;
    wire        auipc;
    wire        mem_unsigned;
    wire [1:0]  alu_op;
    wire [1:0]  mem_size;
    wire [3:0]  alu_ctrl;
    wire        ecall;
    wire        ebreak;
    wire        mret;
    wire [11:0] csr_addr;
    wire [1:0]  csr_op;
    wire        csr_we;
    wire        md_type;
    wire [2:0]  md_operation;
    wire        wfi_req_internal;
    wire        fpu_en;
    wire        f_reg_write;
    wire        f_mem_to_reg;
    wire        f_mem_write;
    wire        f_to_x;
    wire        x_to_f;
    wire [4:0]  fpu_operation;

    // =========================================================================
    // ID/EX
    // =========================================================================
    wire [31:0] id_ex_pc_plus_4;
    wire [31:0] id_ex_pc_in;
    wire [31:0] id_ex_instr;
    wire [31:0] id_ex_read_data1;
    wire [31:0] id_ex_read_data2;
    wire [31:0] id_ex_ext_imm;
    wire [4:0]  id_ex_rs1;
    wire [4:0]  id_ex_rs2;
    wire [4:0]  id_ex_rd;
    wire [2:0]  id_ex_funct3;
    wire        id_ex_reg_write;
    wire        id_ex_alu_src;
    wire        id_ex_mem_write;
    wire        id_ex_mem_read;
    wire        id_ex_mem_to_reg;
    wire        id_ex_branch;
    wire        id_ex_jal;
    wire        id_ex_jalr;
    wire        id_ex_lui;
    wire        id_ex_auipc;
    wire        id_ex_mem_unsigned;
    wire [1:0]  id_ex_mem_size;
    wire [3:0]  id_ex_alu_ctrl;
    wire [31:0] id_ex_branch_target;
    wire [31:0] id_ex_jal_target;
    wire        id_ex_predict_taken;
    wire        id_ex_btb_hit;
    wire        id_ex_ecall;
    wire        id_ex_ebreak;
    wire        id_ex_mret;
    wire [11:0] id_ex_csr_addr;
    wire [1:0]  id_ex_csr_op;
    wire        id_ex_csr_we;
    wire        id_ex_md_type;
    wire [2:0]  id_ex_md_operation;
    wire        id_ex_fpu_en;
    wire        id_ex_f_reg_write;
    wire        id_ex_f_mem_to_reg;
    wire        id_ex_f_mem_write;
    wire        id_ex_f_to_x;
    wire        id_ex_x_to_f;
    wire [4:0]  id_ex_fpu_operation;
    wire [31:0] id_ex_read_f_data1;
    wire [31:0] id_ex_read_f_data2;
    wire [ROB_TAG_W-1:0] id_ex_rob_tag;
    wire        id_ex_rob_valid;

    // =========================================================================
    // EX
    // =========================================================================
    wire [31:0] alu_in1;
    wire [31:0] alu_in2;
    wire [31:0] mem_write_data;
    wire [31:0] csr_write_data_ex;
    wire [31:0] fpu_in1;
    wire [31:0] fpu_in2;
    wire [31:0] alu_result;
    wire        branch_taken;
    wire        mf_alu_stall;
    wire [31:0] fpu_result_out;

    // =========================================================================
    // EX/MEM
    // =========================================================================
    wire [31:0] ex_mem_instr;
    wire [31:0] ex_mem_alu_result;
    wire [31:0] ex_mem_mem_write_data;
    wire [31:0] ex_mem_branch_target;
    wire [31:0] ex_mem_pc_plus_4;
    wire [31:0] ex_mem_pc_in;
    wire [4:0]  ex_mem_rd;
    wire        ex_mem_mem_write;
    wire        ex_mem_mem_read;
    wire        ex_mem_mem_to_reg;
    wire        ex_mem_branch;
    wire        ex_mem_branch_taken;
    wire        ex_mem_jal;
    wire        ex_mem_mem_unsigned;
    wire        ex_mem_reg_write;
    wire [1:0]  ex_mem_mem_size;
    wire        ex_mem_predict_taken;
    wire        ex_mem_btb_hit;
    wire        ex_mem_ecall;
    wire        ex_mem_ebreak;
    wire        ex_mem_mret;
    wire [11:0] ex_mem_csr_addr;
    wire [1:0]  ex_mem_csr_op;
    wire        ex_mem_csr_we;
    wire [31:0] ex_mem_csr_write_data;
    wire [31:0] ex_mem_fpu_result;
    wire [31:0] ex_mem_f_store_data;
    wire        ex_mem_f_reg_write;
    wire        ex_mem_f_mem_to_reg;
    wire        ex_mem_f_mem_write;
    wire [ROB_TAG_W-1:0] ex_mem_rob_tag;
    wire        ex_mem_rob_valid;
    wire [31:0] final_mem_write_data;
    wire [31:0] mem_read_data;

    // =========================================================================
    // MEM/WB
    // =========================================================================
    wire [31:0] mem_wb_mem_read_data;
    wire [31:0] mem_wb_alu_result;
    wire [31:0] mem_wb_pc_plus_4;
    wire        mem_wb_mem_to_reg;
    wire        mem_wb_reg_write;
    wire        mem_wb_jal;
    wire [4:0]  mem_wb_rd;
    wire        mem_wb_ecall;
    wire [31:0] mem_wb_fpu_result;
    wire        mem_wb_f_reg_write;
    wire        mem_wb_f_mem_to_reg;
    wire [ROB_TAG_W-1:0] mem_wb_rob_tag;
    wire        mem_wb_rob_valid;
    wire [31:0] wb_write_data;
    wire [31:0] wb_f_write_data;

    // =========================================================================
    // CSR/debug/branch/stall
    // =========================================================================
    wire [31:0] mie_val;
    wire        mstatus_mie_val;
    wire [31:0] mtvec_pc;
    wire [31:0] mepc_pc;
    wire [31:0] csr_read_data_raw;
    wire [31:0] csr_read_data_fwd;
    wire [31:0] csr_dbg_read_data;
    wire [31:0] rf_dbg_read_data;
    wire [31:0] frf_dbg_read_data;
    wire [31:0] dpc_out;
    wire [31:0] dcsr_out;

    wire        predict_taken;
    wire        bpu_correct;
    wire        btb_hit;
    wire        actual_taken;
    wire [31:0] predict_target;

    wire        load_use_stall;
    wire        flush_branch;
    wire        flush_jal;
    wire        flush_trap;
    wire        stall_IF;
    wire        stall_ID;
    wire        stall_EX;
    wire        stall_MEM;
    wire        stall_WB;
    wire        is_sleeping_internal;

    // =========================================================================
    // Trap and interrupt logic
    // =========================================================================
    wire wake_interrupt = (meip_i & mie_val[11]) |
                          (msip_i & mie_val[3])  |
                          (mtip_i & mie_val[7]);

    wire is_external_irq = meip_i & mie_val[11] & mstatus_mie_val;
    wire is_software_irq = msip_i & mie_val[3]  & mstatus_mie_val;
    wire is_timer_irq    = mtip_i & mie_val[7]  & mstatus_mie_val;
    wire trap_interrupt  = is_external_irq | is_software_irq | is_timer_irq;
    wire trap_enter      = ex_mem_ecall | ex_mem_ebreak | trap_interrupt;
    wire mret_exec       = ex_mem_mret;

    wire [31:0] trap_cause = (trap_interrupt && is_external_irq) ? 32'h8000000b :
                             (trap_interrupt && is_software_irq) ? 32'h80000003 :
                             (trap_interrupt && is_timer_irq)    ? 32'h80000007 :
                             ex_mem_ecall                        ? 32'd11       :
                             ex_mem_ebreak                       ? 32'd3        : 32'd0;
    wire [31:0] trap_pc_value = trap_interrupt ? pc_in : ex_mem_pc_in;

    // =========================================================================
    // Stall/flush policy
    // =========================================================================
    wire stall_if_id  = dcache_stall | mf_alu_stall | load_use_stall | stall_ID;
    wire flush_if_id  = flush_trap | flush_branch | flush_jal | icache_stall;
    wire stall_id_ex  = dcache_stall | mf_alu_stall | stall_EX;
    wire flush_id_ex  = flush_trap | flush_branch | flush_jal | load_use_stall;
    wire stall_ex_mem = dcache_stall | stall_MEM;
    wire flush_ex_mem = flush_trap | flush_branch | mf_alu_stall;
    wire stall_mem_wb = dcache_stall | stall_WB;
    wire flush_mem_wb = flush_trap;

    assign pc_in = pc_reg;
    assign mem_size_top     = ex_mem_mem_size;
    assign mem_unsigned_top = ex_mem_mem_unsigned;
    assign wfi_sleep_out    = is_sleeping_internal;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pc_reg <= reset_vector_in;
        end else if (riscv_start && !riscv_done) begin
            if (dbg_halted && !dbg_resume_req) begin
                pc_reg <= dpc_out;
            end else if (flush_trap || flush_branch || flush_jal) begin
                pc_reg <= pc_out;
            end else if (!stall_IF && !load_use_stall &&
                         !icache_stall && !dcache_stall && !mf_alu_stall) begin
                pc_reg <= pc_out;
            end
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            flush_temp <= 1'b0;
        end else if (riscv_start && !riscv_done) begin
            flush_temp <= flush_branch || flush_jal || flush_trap;
        end
    end

    // =========================================================================
    // IF
    // =========================================================================
    instruction_fetch IF (
        .reset_n(reset_n),
        .flush_temp(flush_temp),
        .trap_enter(trap_enter),
        .mret_exec(mret_exec),
        .reset_vector_in(reset_vector_in),
        .mtvec_in(mtvec_pc),
        .mepc_in(mepc_pc),
        .ex_mem_branch_target(ex_mem_branch_target),
        .id_ex_jal_target(id_ex_jal_target),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .ex_mem_pc_plus_4(ex_mem_pc_plus_4),
        .id_ex_jalr(id_ex_jalr),
        .id_ex_jal(id_ex_jal),
        .btb_hit(btb_hit),
        .alu_in1(alu_in1),
        .id_ex_ext_imm(id_ex_ext_imm),
        .predict_taken(predict_taken),
        .actual_taken(actual_taken),
        .bpu_correct(bpu_correct),
        .predict_target(predict_target),
        .fetch_two_valid(1'b0),
        .pc_out(pc_out),
        .pc_plus_4(pc_plus_4),
        .pc_plus_8(pc_plus_8_unused),
        .instr(instr),
        .instr_lane1(instr_lane1_unused),
        .icache_read_req(icache_read_req),
        .icache_read_req_lane1(if_lane1_req_unused),
        .icache_addr(icache_addr),
        .icache_addr_lane1(if_lane1_addr_unused),
        .icache_read_data(icache_read_data),
        .icache_read_data_lane1(icache_read_data_lane1)
    );

    if_id_register IF_ID (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_if_id),
        .flush(flush_if_id),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .instr(instr),
        .pc_plus_4(pc_plus_4),
        .pc_in(pc_in),
        .predict_taken(predict_taken),
        .btb_hit(btb_hit),
        .if_id_instr(if_id_instr),
        .if_id_pc_plus_4(if_id_pc_plus_4),
        .if_id_pc_in(if_id_pc_in),
        .if_id_predict_taken(if_id_predict_taken),
        .if_id_btb_hit(if_id_btb_hit)
    );

    // =========================================================================
    // ID
    // =========================================================================
    instruction_decode ID (
        .if_id_pc_in(if_id_pc_in),
        .if_id_instr(if_id_instr),
        .ext_imm(ext_imm),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .funct3(funct3),
        .opcode(opcode),
        .funct7(funct7),
        .jal_target(jal_target),
        .branch_target(branch_target),
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
        .alu_ctrl(alu_ctrl),
        .md_type(md_type),
        .md_operation(md_operation),
        .ecall(ecall),
        .ebreak(ebreak),
        .mret(mret),
        .csr_addr(csr_addr),
        .csr_op(csr_op),
        .csr_we(csr_we),
        .wfi_req(wfi_req_internal),
        .fpu_en(fpu_en),
        .f_reg_write(f_reg_write),
        .f_mem_to_reg(f_mem_to_reg),
        .f_mem_write(f_mem_write),
        .f_to_x(f_to_x),
        .x_to_f(x_to_f),
        .fpu_operation(fpu_operation)
    );

    wire is_csr = (dbg_reg_read_addr[15:12] == 4'h0);
    wire is_gpr = (dbg_reg_read_addr[15:5]  == 11'h080);
    wire is_fpr = 1'b0;

    wire dbg_gpr_we = dbg_reg_write_en & (dbg_reg_write_addr[15:5]  == 11'h080);
    wire dbg_csr_we = dbg_reg_write_en & (dbg_reg_write_addr[15:12] == 4'h0);

    assign dbg_reg_read_data = is_csr ? csr_dbg_read_data :
                               is_gpr ? rf_dbg_read_data  :
                               is_fpr ? frf_dbg_read_data : 32'd0;

    csr_register_file CSR_RF (
        .clk(clk),
        .reset_n(reset_n),
        .meip_i(meip_i),
        .msip_i(msip_i),
        .mtip_i(mtip_i),
        .csr_addr(id_ex_csr_addr),
        .csr_read_data(csr_read_data_raw),
        .csr_addr_lane1(12'd0),
        .csr_read_data_lane1(),
        .csr_write_addr(ex_mem_csr_addr),
        .csr_write_data(ex_mem_csr_write_data),
        .csr_op(ex_mem_csr_op),
        .csr_write_en(ex_mem_csr_we),
        .count_en(!dbg_halted),
        .instret_en(!dbg_halted),
        .trap_enter(trap_enter),
        .mret_exec(mret_exec),
        .trap_cause(trap_cause),
        .trap_pc(trap_pc_value),
        .trap_val(32'd0),
        .mtvec_out(mtvec_pc),
        .mepc_out(mepc_pc),
        .mie_out(mie_val),
        .mstatus_mie(mstatus_mie_val),
        .dbg_halt_req(dbg_halt_req),
        .dbg_halted(dbg_halted),
        .debug_pc_in(pc_in),
        .dpc_out(dpc_out),
        .dcsr_out(dcsr_out),
        .dbg_reg_read_addr(dbg_reg_read_addr[11:0]),
        .dbg_read_data(csr_dbg_read_data),
        .dbg_reg_write_en(dbg_csr_we),
        .dbg_reg_write_addr(dbg_reg_write_addr[11:0]),
        .dbg_reg_write_data(dbg_reg_write_data)
    );

    assign csr_read_data_fwd =
        (ex_mem_csr_we && (ex_mem_csr_addr == id_ex_csr_addr)) ?
        ex_mem_csr_write_data : csr_read_data_raw;

    register_file RF (
        .clk(clk),
        .reset_n(reset_n),
        .read_reg1(rs1),
        .read_reg2(rs2),
        .read_reg1_lane1(5'd0),
        .read_reg2_lane1(5'd0),
        .mem_wb_reg_write(mem_wb_reg_write),
        .mem_wb_rd(mem_wb_rd),
        .mem_wb_write_data(wb_write_data),
        .mem_wb_reg_write_lane1(1'b0),
        .mem_wb_rd_lane1(5'd0),
        .mem_wb_write_data_lane1(32'd0),
        .ooo_commit_valid0(1'b0),
        .ooo_commit_rd0(5'd0),
        .ooo_commit_data0(32'd0),
        .ooo_commit_valid1(1'b0),
        .ooo_commit_rd1(5'd0),
        .ooo_commit_data1(32'd0),
        .read_data1(read_data1_temp),
        .read_data2(read_data2_temp),
        .read_data1_lane1(),
        .read_data2_lane1(),
        .dbg_mode(dbg_halted),
        .dbg_read_addr(dbg_reg_read_addr[4:0]),
        .dbg_read_data(rf_dbg_read_data),
        .dbg_write_en(dbg_gpr_we),
        .dbg_write_addr(dbg_reg_write_addr[4:0]),
        .dbg_write_data(dbg_reg_write_data)
    );

    assign read_data1 = (rs1 != 5'd0 && rs1 == mem_wb_rd && mem_wb_reg_write) ?
                        wb_write_data : read_data1_temp;
    assign read_data2 = (rs2 != 5'd0 && rs2 == mem_wb_rd && mem_wb_reg_write) ?
                        wb_write_data : read_data2_temp;

    assign frf_dbg_read_data = 32'd0;
    assign wb_f_write_data   = 32'd0;

    id_ex_register #(.ROB_TAG_W(ROB_TAG_W)) ID_EX (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_id_ex),
        .flush(flush_id_ex),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .rob_tag({ROB_TAG_W{1'b0}}),
        .rob_valid(1'b0),
        .if_id_pc_plus_4(if_id_pc_plus_4),
        .if_id_pc_in(if_id_pc_in),
        .funct3(funct3),
        .read_data1(read_data1),
        .read_data2(read_data2),
        .ext_imm(ext_imm),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
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
        .mem_size(mem_size),
        .alu_ctrl(alu_ctrl),
        .branch_target(branch_target),
        .jal_target(jal_target),
        .if_id_predict_taken(if_id_predict_taken),
        .if_id_btb_hit(if_id_btb_hit),
        .ecall(ecall),
        .ebreak(ebreak),
        .mret(mret),
        .csr_addr(csr_addr),
        .csr_op(csr_op),
        .csr_we(csr_we),
        .md_type(md_type),
        .md_operation(md_operation),
        .if_id_instr(if_id_instr),
        .fpu_en(1'b0),
        .f_reg_write(1'b0),
        .f_mem_to_reg(1'b0),
        .f_mem_write(1'b0),
        .f_to_x(1'b0),
        .x_to_f(1'b0),
        .fpu_operation(5'd0),
        .read_f_data1(32'd0),
        .read_f_data2(32'd0),
        .id_ex_pc_plus_4(id_ex_pc_plus_4),
        .id_ex_pc_in(id_ex_pc_in),
        .id_ex_funct3(id_ex_funct3),
        .id_ex_read_data1(id_ex_read_data1),
        .id_ex_read_data2(id_ex_read_data2),
        .id_ex_ext_imm(id_ex_ext_imm),
        .id_ex_rs1(id_ex_rs1),
        .id_ex_rs2(id_ex_rs2),
        .id_ex_rd(id_ex_rd),
        .id_ex_reg_write(id_ex_reg_write),
        .id_ex_alu_src(id_ex_alu_src),
        .id_ex_mem_write(id_ex_mem_write),
        .id_ex_mem_read(id_ex_mem_read),
        .id_ex_mem_to_reg(id_ex_mem_to_reg),
        .id_ex_branch(id_ex_branch),
        .id_ex_jal(id_ex_jal),
        .id_ex_jalr(id_ex_jalr),
        .id_ex_lui(id_ex_lui),
        .id_ex_auipc(id_ex_auipc),
        .id_ex_mem_unsigned(id_ex_mem_unsigned),
        .id_ex_mem_size(id_ex_mem_size),
        .id_ex_alu_ctrl(id_ex_alu_ctrl),
        .id_ex_branch_target(id_ex_branch_target),
        .id_ex_jal_target(id_ex_jal_target),
        .id_ex_predict_taken(id_ex_predict_taken),
        .id_ex_btb_hit(id_ex_btb_hit),
        .id_ex_ecall(id_ex_ecall),
        .id_ex_ebreak(id_ex_ebreak),
        .id_ex_mret(id_ex_mret),
        .id_ex_csr_addr(id_ex_csr_addr),
        .id_ex_csr_op(id_ex_csr_op),
        .id_ex_csr_we(id_ex_csr_we),
        .id_ex_md_type(id_ex_md_type),
        .id_ex_md_operation(id_ex_md_operation),
        .id_ex_instr(id_ex_instr),
        .id_ex_fpu_en(id_ex_fpu_en),
        .id_ex_f_reg_write(id_ex_f_reg_write),
        .id_ex_f_mem_to_reg(id_ex_f_mem_to_reg),
        .id_ex_f_mem_write(id_ex_f_mem_write),
        .id_ex_f_to_x(id_ex_f_to_x),
        .id_ex_x_to_f(id_ex_x_to_f),
        .id_ex_fpu_operation(id_ex_fpu_operation),
        .id_ex_read_f_data1(id_ex_read_f_data1),
        .id_ex_read_f_data2(id_ex_read_f_data2),
        .id_ex_rob_tag(id_ex_rob_tag),
        .id_ex_rob_valid(id_ex_rob_valid)
    );

    // =========================================================================
    // EX
    // =========================================================================
    forwarding_unit FU (
        .id_ex_read_data1(id_ex_read_data1),
        .id_ex_read_data2(id_ex_read_data2),
        .id_ex_ext_imm(id_ex_ext_imm),
        .id_ex_rs1(id_ex_rs1),
        .id_ex_rs2(id_ex_rs2),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_reg_write(mem_wb_reg_write),
        .id_ex_alu_src(id_ex_alu_src),
        .ex_mem_rd(ex_mem_rd),
        .mem_wb_rd(mem_wb_rd),
        .ex_mem_alu_result(ex_mem_alu_result),
        .mem_wb_write_data(wb_write_data),
        .id_ex_read_f_data1(32'd0),
        .id_ex_read_f_data2(32'd0),
        .ex_mem_f_reg_write(1'b0),
        .mem_wb_f_reg_write(1'b0),
        .ex_mem_fpu_result(32'd0),
        .mem_wb_f_write_data(32'd0),
        .alu_in1(alu_in1),
        .alu_in2(alu_in2),
        .mem_write_data(mem_write_data),
        .fpu_in1(fpu_in1),
        .fpu_in2(fpu_in2)
    );

    execute #(
        .ENABLE_MULDIV(1),
        .ENABLE_FPU(0),
        .ENABLE_CSR(1),
        .ENABLE_BRANCH(1)
    ) EX (
        .clk(clk),
        .reset_n(reset_n),
        .stall_id_ex(stall_id_ex),
        .alu_in1(alu_in1),
        .alu_in2(alu_in2),
        .id_ex_alu_ctrl(id_ex_alu_ctrl),
        .id_ex_funct3(id_ex_funct3),
        .id_ex_branch(id_ex_branch),
        .id_ex_instr(id_ex_instr),
        .id_ex_lui(id_ex_lui),
        .id_ex_auipc(id_ex_auipc),
        .id_ex_md_type(id_ex_md_type),
        .id_ex_md_operation(id_ex_md_operation),
        .id_ex_pc_in(id_ex_pc_in),
        .id_ex_ext_imm(id_ex_ext_imm),
        .id_ex_csr_op(id_ex_csr_op),
        .id_ex_csr_we(id_ex_csr_we),
        .csr_read_data(csr_read_data_fwd),
        .id_ex_rs1(id_ex_rs1),
        .id_ex_fpu_en(1'b0),
        .id_ex_fpu_operation(5'd0),
        .id_ex_read_f_data1(32'd0),
        .id_ex_read_f_data2(32'd0),
        .id_ex_f_to_x(1'b0),
        .id_ex_x_to_f(1'b0),
        .alu_result(alu_result),
        .branch_taken(branch_taken),
        .csr_write_data(csr_write_data_ex),
        .mf_alu_stall(mf_alu_stall),
        .fpu_result_out(fpu_result_out)
    );

    ex_mem_register #(.ROB_TAG_W(ROB_TAG_W)) EX_MEM (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_ex_mem),
        .flush(flush_ex_mem),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .id_ex_rob_tag(id_ex_rob_tag),
        .id_ex_rob_valid(id_ex_rob_valid),
        .alu_result(alu_result),
        .id_ex_ext_imm(id_ex_ext_imm),
        .id_ex_rd(id_ex_rd),
        .id_ex_pc_plus_4(id_ex_pc_plus_4),
        .id_ex_pc_in(id_ex_pc_in),
        .id_ex_branch_target(id_ex_branch_target),
        .id_ex_mem_write(id_ex_mem_write),
        .id_ex_mem_read(id_ex_mem_read),
        .id_ex_mem_to_reg(id_ex_mem_to_reg),
        .id_ex_reg_write(id_ex_reg_write),
        .id_ex_branch(id_ex_branch),
        .branch_taken(branch_taken),
        .id_ex_jal(id_ex_jal),
        .id_ex_mem_unsigned(id_ex_mem_unsigned),
        .id_ex_mem_size(id_ex_mem_size),
        .id_ex_read_data2(id_ex_read_data2),
        .mem_write_data(mem_write_data),
        .id_ex_predict_taken(id_ex_predict_taken),
        .id_ex_btb_hit(id_ex_btb_hit),
        .id_ex_ecall(id_ex_ecall),
        .id_ex_ebreak(id_ex_ebreak),
        .id_ex_mret(id_ex_mret),
        .id_ex_csr_addr(id_ex_csr_addr),
        .id_ex_csr_op(id_ex_csr_op),
        .id_ex_csr_we(id_ex_csr_we),
        .csr_write_data_in(csr_write_data_ex),
        .id_ex_instr(id_ex_instr),
        .fpu_result(32'd0),
        .id_ex_read_f_data2(32'd0),
        .id_ex_f_reg_write(1'b0),
        .id_ex_f_mem_to_reg(1'b0),
        .id_ex_f_mem_write(1'b0),
        .ex_mem_alu_result(ex_mem_alu_result),
        .ex_mem_branch_target(ex_mem_branch_target),
        .ex_mem_pc_plus_4(ex_mem_pc_plus_4),
        .ex_mem_pc_in(ex_mem_pc_in),
        .ex_mem_rd(ex_mem_rd),
        .ex_mem_mem_write(ex_mem_mem_write),
        .ex_mem_mem_read(ex_mem_mem_read),
        .ex_mem_mem_to_reg(ex_mem_mem_to_reg),
        .ex_mem_reg_write(ex_mem_reg_write),
        .ex_mem_branch(ex_mem_branch),
        .ex_mem_branch_taken(ex_mem_branch_taken),
        .ex_mem_jal(ex_mem_jal),
        .ex_mem_mem_unsigned(ex_mem_mem_unsigned),
        .ex_mem_mem_size(ex_mem_mem_size),
        .ex_mem_mem_write_data(ex_mem_mem_write_data),
        .ex_mem_predict_taken(ex_mem_predict_taken),
        .ex_mem_btb_hit(ex_mem_btb_hit),
        .ex_mem_ecall(ex_mem_ecall),
        .ex_mem_ebreak(ex_mem_ebreak),
        .ex_mem_mret(ex_mem_mret),
        .ex_mem_csr_addr(ex_mem_csr_addr),
        .ex_mem_csr_op(ex_mem_csr_op),
        .ex_mem_csr_we(ex_mem_csr_we),
        .ex_mem_csr_write_data(ex_mem_csr_write_data),
        .ex_mem_instr(ex_mem_instr),
        .ex_mem_fpu_result(ex_mem_fpu_result),
        .ex_mem_f_store_data(ex_mem_f_store_data),
        .ex_mem_f_reg_write(ex_mem_f_reg_write),
        .ex_mem_f_mem_to_reg(ex_mem_f_mem_to_reg),
        .ex_mem_f_mem_write(ex_mem_f_mem_write),
        .ex_mem_rob_tag(ex_mem_rob_tag),
        .ex_mem_rob_valid(ex_mem_rob_valid)
    );

    // =========================================================================
    // MEM
    // =========================================================================
    assign final_mem_write_data =
        ex_mem_f_mem_write ? ex_mem_f_store_data : ex_mem_mem_write_data;

    memory_access MEM (
        .clk(clk),
        .reset_n(reset_n),
        .ex_mem_alu_result(ex_mem_alu_result),
        .ex_mem_mem_write_data(final_mem_write_data),
        .ex_mem_instr(ex_mem_instr),
        .ex_mem_mem_write(ex_mem_mem_write | ex_mem_f_mem_write),
        .ex_mem_mem_read(ex_mem_mem_read),
        .mem_read_data(mem_read_data),
        .dcache_read_req(dcache_read_req),
        .dcache_write_req(dcache_write_req),
        .dcache_addr(dcache_addr),
        .dcache_write_data(dcache_write_data),
        .dcache_read_data(dcache_read_data)
    );

    mem_wb_register #(.ROB_TAG_W(ROB_TAG_W)) MEM_WB (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_mem_wb),
        .flush(flush_mem_wb),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .ex_mem_rob_tag(ex_mem_rob_tag),
        .ex_mem_rob_valid(ex_mem_rob_valid),
        .mem_read_data(mem_read_data),
        .ex_mem_pc_plus_4(ex_mem_pc_plus_4),
        .ex_mem_mem_to_reg(ex_mem_mem_to_reg),
        .ex_mem_reg_write(ex_mem_reg_write),
        .ex_mem_jal(ex_mem_jal),
        .ex_mem_alu_result(ex_mem_alu_result),
        .ex_mem_rd(ex_mem_rd),
        .ex_mem_ecall(ex_mem_ecall),
        .ex_mem_fpu_result(32'd0),
        .ex_mem_f_reg_write(1'b0),
        .ex_mem_f_mem_to_reg(1'b0),
        .mem_wb_mem_read_data(mem_wb_mem_read_data),
        .mem_wb_pc_plus_4(mem_wb_pc_plus_4),
        .mem_wb_alu_result(mem_wb_alu_result),
        .mem_wb_mem_to_reg(mem_wb_mem_to_reg),
        .mem_wb_reg_write(mem_wb_reg_write),
        .mem_wb_jal(mem_wb_jal),
        .mem_wb_rd(mem_wb_rd),
        .mem_wb_ecall(mem_wb_ecall),
        .mem_wb_fpu_result(mem_wb_fpu_result),
        .mem_wb_f_reg_write(mem_wb_f_reg_write),
        .mem_wb_f_mem_to_reg(mem_wb_f_mem_to_reg),
        .mem_wb_rob_tag(mem_wb_rob_tag),
        .mem_wb_rob_valid(mem_wb_rob_valid)
    );

    write_back WB (
        .mem_wb_mem_read_data(mem_wb_mem_read_data),
        .mem_wb_alu_result(mem_wb_alu_result),
        .mem_wb_pc_plus_4(mem_wb_pc_plus_4),
        .mem_wb_mem_to_reg(mem_wb_mem_to_reg),
        .mem_wb_jal(mem_wb_jal),
        .mem_wb_write_data(wb_write_data)
    );

    // =========================================================================
    // Control and branch prediction
    // =========================================================================
    pipeline_control_unit PCU (
        .clk(clk),
        .reset_n(reset_n),
        .opcode(opcode),
        .funct3(funct3),
        .rs1(rs1),
        .rs2(rs2),
        .id_ex_mem_read(id_ex_mem_read),
        .id_ex_jal(id_ex_jal),
        .id_ex_jalr(id_ex_jalr),
        .id_ex_rd(id_ex_rd),
        .bpu_correct(bpu_correct),
        .trap_enter(trap_enter),
        .mret_exec(mret_exec),
        .icache_stall(icache_stall),
        .dcache_stall(dcache_stall),
        .mf_alu_stall(mf_alu_stall),
        .wfi_req(wfi_req_internal),
        .trap_interrupt(wake_interrupt),
        .is_sleeping(is_sleeping_internal),
        .dbg_halt_req(dbg_halt_req),
        .dbg_resume_req(dbg_resume_req),
        .dcsr_step(dcsr_out[2]),
        .dbg_halted(dbg_halted),
        .load_use_stall(load_use_stall),
        .flush_branch(flush_branch),
        .flush_jal(flush_jal),
        .flush_trap(flush_trap),
        .stall_IF(stall_IF),
        .stall_ID(stall_ID),
        .stall_EX(stall_EX),
        .stall_MEM(stall_MEM),
        .stall_WB(stall_WB)
    );

    branch_prediction_unit BPU (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_mem_wb),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .ex_mem_branch(ex_mem_branch),
        .ex_mem_branch_taken(ex_mem_branch_taken),
        .ex_mem_predict_taken(ex_mem_predict_taken),
        .ex_mem_btb_hit(ex_mem_btb_hit),
        .ex_mem_branch_target(ex_mem_branch_target),
        .bpu_correct(bpu_correct),
        .predict_taken(predict_taken),
        .btb_hit(btb_hit),
        .actual_taken(actual_taken),
        .predict_target(predict_target)
    );

    // =========================================================================
    // Done
    // =========================================================================
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            riscv_done <= 1'b0;
        end else if (riscv_start) begin
            if (ex_mem_ecall || mem_wb_ecall) begin
                riscv_done <= 1'b1;
            end
        end
    end

endmodule
