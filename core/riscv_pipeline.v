`timescale 1ns / 1ps

module riscv_pipeline #(
    parameter ENABLE_TOMASULO_INTEGER = 1
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

    // =========================================================================
    // KHAI BÁO CÁC TÍN HIỆU TRUNG GIAN (WIRES)
    // =========================================================================
    wire [31:0] pc_in;
    wire [31:0] pc_out;
    wire [31:0] pc_plus_4;
    wire [31:0] pc_plus_8;
    wire [31:0] instr;
    wire [31:0] instr_lane1;
    wire        predict_taken;
    wire        bpu_correct;
    wire        bpu_correct_selected;
    wire        btb_hit;
    wire        actual_taken;
    wire        actual_taken_selected;
    wire [31:0] predict_target;

    wire [31:0] if_id_instr;
    wire [31:0] if_id_pc_plus_4;
    wire [31:0] if_id_pc_in;
    wire        if_id_predict_taken;
    wire        if_id_btb_hit;
    wire [31:0] if_id_instr_lane1;
    wire [31:0] if_id_pc_plus_4_lane1;
    wire [31:0] if_id_pc_in_lane1;
    wire        if_id_predict_taken_lane1;
    wire        if_id_btb_hit_lane1;

    wire [31:0] read_data1;
    wire [31:0] read_data2;
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
    wire        fpu_en;
    wire        f_reg_write;
    wire        f_mem_to_reg;
    wire        f_mem_write;
    wire        f_to_x;
    wire        x_to_f;
    wire [4:0]  fpu_operation;

    wire [31:0] ext_imm_lane1;
    wire [4:0]  rs1_lane1;
    wire [4:0]  rs2_lane1;
    wire [4:0]  rd_lane1;
    wire [2:0]  funct3_lane1;
    wire [6:0]  opcode_lane1;
    wire [6:0]  funct7_lane1;
    wire [31:0] jal_target_lane1;
    wire [31:0] branch_target_lane1;
    wire        reg_write_lane1;
    wire        alu_src_lane1;
    wire        mem_write_lane1;
    wire        mem_read_lane1;
    wire        mem_to_reg_lane1;
    wire        branch_lane1;
    wire        jal_lane1;
    wire        jalr_lane1;
    wire        lui_lane1;
    wire        auipc_lane1;
    wire        mem_unsigned_lane1;
    wire [1:0]  alu_op_lane1;
    wire [1:0]  mem_size_lane1;
    wire [3:0]  alu_ctrl_lane1;
    wire        ecall_lane1;
    wire        ebreak_lane1;
    wire        mret_lane1;
    wire [11:0] csr_addr_lane1;
    wire [1:0]  csr_op_lane1;
    wire        csr_we_lane1;
    wire        md_type_lane1;
    wire [2:0]  md_operation_lane1;
    wire        fpu_en_lane1;
    wire        f_reg_write_lane1;
    wire        f_mem_to_reg_lane1;
    wire        f_mem_write_lane1;
    wire        f_to_x_lane1;
    wire        x_to_f_lane1;
    wire [4:0]  fpu_operation_lane1;
    wire        wfi_req_lane1;

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
    wire [31:0] id_ex_pc_plus_4_lane1;
    wire [31:0] id_ex_pc_in_lane1;
    wire [31:0] id_ex_instr_lane1;
    wire [31:0] id_ex_read_data1_lane1;
    wire [31:0] id_ex_read_data2_lane1;
    wire [31:0] id_ex_ext_imm_lane1;
    wire [4:0]  id_ex_rs1_lane1;
    wire [4:0]  id_ex_rs2_lane1;
    wire [4:0]  id_ex_rd_lane1;
    wire [2:0]  id_ex_funct3_lane1;
    wire        id_ex_reg_write_lane1;
    wire        id_ex_alu_src_lane1;
    wire        id_ex_mem_write_lane1;
    wire        id_ex_mem_read_lane1;
    wire        id_ex_mem_to_reg_lane1;
    wire        id_ex_branch_lane1;
    wire        id_ex_jal_lane1;
    wire        id_ex_jalr_lane1;
    wire        id_ex_lui_lane1;
    wire        id_ex_auipc_lane1;
    wire        id_ex_mem_unsigned_lane1;
    wire [1:0]  id_ex_mem_size_lane1;
    wire [3:0]  id_ex_alu_ctrl_lane1;
    wire [31:0] id_ex_branch_target_lane1;
    wire [31:0] id_ex_jal_target_lane1;
    wire        id_ex_predict_taken_lane1;
    wire        id_ex_btb_hit_lane1;
    wire        id_ex_ecall_lane1;
    wire        id_ex_ebreak_lane1;
    wire        id_ex_mret_lane1;
    wire [11:0] id_ex_csr_addr_lane1;
    wire [1:0]  id_ex_csr_op_lane1;
    wire        id_ex_csr_we_lane1;
    wire        id_ex_md_type_lane1;
    wire [2:0]  id_ex_md_operation_lane1;
    wire        id_ex_fpu_en_lane1;
    wire        id_ex_f_reg_write_lane1;
    wire        id_ex_f_mem_to_reg_lane1;
    wire        id_ex_f_mem_write_lane1;
    wire        id_ex_f_to_x_lane1;
    wire        id_ex_x_to_f_lane1;
    wire [4:0]  id_ex_fpu_operation_lane1;
    wire [31:0] id_ex_read_f_data1_lane1;
    wire [31:0] id_ex_read_f_data2_lane1;
    wire [31:0] alu_in1;
    wire [31:0] alu_in2;
    wire [31:0] mem_write_data;
    wire [31:0] csr_write_data_ex;
    wire [31:0] fpu_in1;
    wire [31:0] fpu_in2;
    wire [31:0] alu_in1_base;
    wire [31:0] alu_in2_base;
    wire [31:0] mem_write_data_base;
    wire [31:0] fpu_in1_base;
    wire [31:0] fpu_in2_base;
    wire [31:0] alu_result;
    wire        branch_taken;
    wire        mf_alu_stall;
    wire [31:0] fpu_result_out;
    wire [31:0] alu_in1_lane1;
    wire [31:0] alu_in2_lane1;
    wire [31:0] mem_write_data_lane1;
    wire [31:0] csr_write_data_ex_lane1;
    wire [31:0] fpu_in1_lane1;
    wire [31:0] fpu_in2_lane1;
    wire [31:0] alu_result_lane1;
    wire        branch_taken_lane1;
    wire        mf_alu_stall_lane1;
    wire [31:0] fpu_result_out_lane1;

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
    wire [31:0] ex_mem_instr_lane1;
    wire [31:0] ex_mem_alu_result_lane1;
    wire [31:0] ex_mem_mem_write_data_lane1;
    wire [31:0] ex_mem_branch_target_lane1;
    wire [31:0] ex_mem_pc_plus_4_lane1;
    wire [31:0] ex_mem_pc_in_lane1;
    wire [4:0]  ex_mem_rd_lane1;
    wire        ex_mem_mem_write_lane1;
    wire        ex_mem_mem_read_lane1;
    wire        ex_mem_mem_to_reg_lane1;
    wire        ex_mem_branch_lane1;
    wire        ex_mem_branch_taken_lane1;
    wire        ex_mem_jal_lane1;
    wire        ex_mem_mem_unsigned_lane1;
    wire        ex_mem_reg_write_lane1;
    wire [1:0]  ex_mem_mem_size_lane1;
    wire        ex_mem_predict_taken_lane1;
    wire        ex_mem_btb_hit_lane1;
    wire        ex_mem_ecall_lane1;
    wire        ex_mem_ebreak_lane1;
    wire        ex_mem_mret_lane1;
    wire [11:0] ex_mem_csr_addr_lane1;
    wire [1:0]  ex_mem_csr_op_lane1;
    wire        ex_mem_csr_we_lane1;
    wire [31:0] ex_mem_csr_write_data_lane1;
    wire [31:0] ex_mem_fpu_result_lane1;
    wire [31:0] ex_mem_f_store_data_lane1;
    wire        ex_mem_f_reg_write_lane1;
    wire        ex_mem_f_mem_to_reg_lane1;
    wire        ex_mem_f_mem_write_lane1;
    wire [31:0] mem_read_data;
    wire [31:0] mem_read_data_lane1;
    wire        legacy_lane0_dcache_read_req;
    wire        legacy_lane0_dcache_write_req;
    wire [31:0] legacy_lane0_dcache_addr;
    wire [31:0] legacy_lane0_dcache_write_data;
    wire        legacy_dcache_read_req;
    wire        legacy_dcache_write_req;
    wire [31:0] legacy_dcache_addr;
    wire [31:0] legacy_dcache_write_data;
    wire [1:0]  legacy_dcache_mem_size;
    wire        legacy_dcache_mem_unsigned;
    wire        ooo_dcache_read_req;
    wire        ooo_dcache_write_req;
    wire [31:0] ooo_dcache_addr;
    wire [31:0] ooo_dcache_write_data;
    wire [1:0]  ooo_dcache_mem_size;
    wire        ooo_dcache_mem_unsigned;
    wire        ooo_dcache_active;
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
    wire [31:0] wb_write_data;
    wire [31:0] wb_f_write_data;
    wire [31:0] mem_wb_mem_read_data_lane1;
    wire [31:0] mem_wb_alu_result_lane1;
    wire [31:0] mem_wb_pc_plus_4_lane1;
    wire        mem_wb_mem_to_reg_lane1;
    wire        mem_wb_reg_write_lane1;
    wire        mem_wb_jal_lane1;
    wire [4:0]  mem_wb_rd_lane1;
    wire        mem_wb_ecall_lane1;
    wire [31:0] mem_wb_fpu_result_lane1;
    wire        mem_wb_f_reg_write_lane1;
    wire        mem_wb_f_mem_to_reg_lane1;
    wire [31:0] wb_write_data_lane1;
    wire [31:0] wb_f_write_data_lane1;

    wire        load_use_stall;
    wire        load_use_stall_raw;
    wire        flush_branch;
    wire        flush_jal;
    wire        flush_trap;
    wire        stall_IF;
    wire        stall_ID;
    wire        stall_EX;
    wire        stall_MEM;
    wire        stall_WB;
    wire [31:0] read_data1_temp;
    wire [31:0] read_data2_temp;
    wire [31:0] read_data1_temp_lane1;
    wire [31:0] read_data2_temp_lane1;
    wire [31:0] read_data1_lane1;
    wire [31:0] read_data2_lane1;
    wire [31:0] read_f_data1_temp;
    wire [31:0] read_f_data2_temp;
    wire [31:0] read_f_data1_temp_lane1;
    wire [31:0] read_f_data2_temp_lane1;
    wire [31:0] read_f_data1;
    wire [31:0] read_f_data2;
    wire [31:0] read_f_data1_lane1;
    wire [31:0] read_f_data2_lane1;
    wire [31:0] mie_val;
    wire        mstatus_mie_val;
    wire [31:0] mtvec_pc;
    wire [31:0] mepc_pc;
    wire [31:0] csr_read_data_raw;
    wire [31:0] csr_read_data_fwd;
    wire [31:0] csr_read_data_raw_lane1;
    wire [31:0] csr_read_data_fwd_lane1;
    wire        wfi_req_internal;
    
    wire [31:0] dpc_out;
    wire [31:0] dcsr_out;

    // =========================================================================
    // SUPERSCALAR ROB INTEGRATION
    // =========================================================================
    localparam ROB_DEPTH      = 64;
    localparam ROB_TAG_W      = 6;
    localparam ROB_N_DISPATCH = 2;
    localparam ROB_N_COMMIT   = 2;
    localparam ROB_N_CDB      = 2;
    localparam ROB_PTR_W      = ROB_TAG_W + 1;

    localparam [2:0] ROB_TYPE_ALU    = 3'b000;
    localparam [2:0] ROB_TYPE_LOAD   = 3'b001;
    localparam [2:0] ROB_TYPE_STORE  = 3'b010;
    localparam [2:0] ROB_TYPE_BRANCH = 3'b011;
    localparam [2:0] ROB_TYPE_FP     = 3'b100;

    wire [ROB_TAG_W-1:0] id_ex_rob_tag;
    wire [ROB_TAG_W-1:0] ex_mem_rob_tag;
    wire [ROB_TAG_W-1:0] mem_wb_rob_tag;
    wire                 id_ex_rob_valid;
    wire                 ex_mem_rob_valid;
    wire                 mem_wb_rob_valid;
    wire [ROB_TAG_W-1:0] id_ex_rob_tag_lane1;
    wire [ROB_TAG_W-1:0] ex_mem_rob_tag_lane1;
    wire [ROB_TAG_W-1:0] mem_wb_rob_tag_lane1;
    wire                 id_ex_rob_valid_lane1;
    wire                 ex_mem_rob_valid_lane1;
    wire                 mem_wb_rob_valid_lane1;

    wire [ROB_N_DISPATCH-1:0]           rob_dispatch_valid;
    wire [ROB_N_DISPATCH-1:0]           rob_dispatch_ready;
    wire [ROB_N_DISPATCH*ROB_TAG_W-1:0] rob_dispatch_tag;
    wire [ROB_N_DISPATCH*32-1:0]        rob_dispatch_pc;
    wire [ROB_N_DISPATCH*5-1:0]         rob_dispatch_rd;
    wire [ROB_N_DISPATCH-1:0]           rob_dispatch_rd_valid;
    wire [ROB_N_DISPATCH*3-1:0]         rob_dispatch_type;

    wire [ROB_N_CDB-1:0]                rob_cdb_valid;
    wire [ROB_N_CDB*ROB_TAG_W-1:0]      rob_cdb_tag;
    wire [ROB_N_CDB*32-1:0]             rob_cdb_data;
    wire [ROB_N_CDB*8-1:0]              rob_cdb_exc;

    wire [ROB_N_COMMIT-1:0]             rob_commit_valid;
    wire [ROB_N_COMMIT*5-1:0]           rob_commit_rd;
    wire [ROB_N_COMMIT-1:0]             rob_commit_rd_valid;
    wire [ROB_N_COMMIT*32-1:0]          rob_commit_data;
    wire [ROB_N_COMMIT*32-1:0]          rob_commit_pc;
    wire [ROB_N_COMMIT*3-1:0]           rob_commit_type;
    wire [ROB_N_COMMIT-1:0]             rob_commit_is_store;
    wire                                rob_flush;
    wire [31:0]                         rob_flush_pc;
    wire                                rob_exc_valid;
    wire [7:0]                          rob_exc_code;
    wire                                rob_flush_early;
    wire [31:0]                         rob_flush_pc_early;
    wire                                rob_full;
    wire                                rob_empty;
    wire [ROB_PTR_W-1:0]                rob_count;
    wire [ROB_PTR_W-1:0]                rob_free_slots;
    wire [ROB_DEPTH-1:0]                rob_wakeup_vec;
    wire                                rob_alloc_stall;
    wire                                rob_decode_valid;
    wire                                rob_decode_valid_lane1;

    localparam OOO_N_ARCH_WB = 3;

    wire                                ooo_enabled;
    wire [ROB_N_DISPATCH-1:0]           ooo_dispatch_valid;
    wire [ROB_N_DISPATCH-1:0]           ooo_dispatch_ready;
    wire [ROB_N_COMMIT-1:0]             ooo_commit_valid;
    wire [ROB_N_COMMIT*5-1:0]           ooo_commit_rd;
    wire [ROB_N_COMMIT-1:0]             ooo_commit_rd_valid;
    wire [ROB_N_COMMIT*32-1:0]          ooo_commit_data;
    wire                                ooo_rob_full;
    wire                                ooo_rob_empty;
    wire                                ooo_backend_empty;
    wire                                ooo_lane0_candidate;
    wire                                ooo_lane1_candidate;
    wire                                ooo_lane0_alu_candidate;
    wire                                ooo_lane0_mem_candidate;
    wire                                ooo_lane1_alu_candidate;
    wire                                ooo_lane1_mem_candidate;
    wire                                ooo_lane0_fire;
    wire                                ooo_lane1_fire;
    wire                                legacy_decode_valid_lane0;
    wire                                legacy_decode_valid_lane1;
    wire [ROB_N_DISPATCH-1:0]           legacy_dispatch_valid;
    wire                                legacy_pipe_busy;
    wire                                legacy_rob_alloc_stall;
    wire                                ooo_backend_alloc_stall;
    wire                                ooo_decode_blocked_by_legacy;
    wire                                legacy_decode_waits_for_ooo;
    wire                                ooo_interrupt_drain;

    function automatic is_dual_int_eligible;
        input [31:0] inst;
        reg [6:0] op;
        begin
            op = inst[6:0];
            is_dual_int_eligible = (inst != 32'h00000013) &&
                                   !((op == 7'b0110011) && (inst[31:25] == 7'b0000001)) &&
                                   ((op == 7'b0110011) || // OP
                                   (op == 7'b0010011) || // OP-IMM
                                   (op == 7'b0110111) || // LUI
                                   (op == 7'b0010111));  // AUIPC
        end
    endfunction

    function automatic inst_is_csr;
        input [31:0] inst;
        begin
            inst_is_csr = (inst[6:0] == 7'b1110011) && (inst[14:12] != 3'b000);
        end
    endfunction

    function automatic inst_is_system_barrier;
        input [31:0] inst;
        begin
            inst_is_system_barrier =
                (inst[6:0] == 7'b1110011) && (inst[14:12] == 3'b000);
        end
    endfunction

    function automatic inst_is_mem;
        input [31:0] inst;
        reg [6:0] op;
        begin
            op = inst[6:0];
            inst_is_mem = (op == 7'b0000011) || // LOAD
                          (op == 7'b0100011) || // STORE
                          (op == 7'b0000111) || // LOAD-FP
                          (op == 7'b0100111);   // STORE-FP
        end
    endfunction

    function automatic inst_is_control;
        input [31:0] inst;
        reg [6:0] op;
        begin
            op = inst[6:0];
            inst_is_control = (op == 7'b1100011) || // BRANCH
                              (op == 7'b1100111) || // JALR
                              (op == 7'b1101111);   // JAL
        end
    endfunction

    function automatic is_dual_lane_eligible;
        input [31:0] inst;
        reg [6:0] op;
        begin
            op = inst[6:0];
            is_dual_lane_eligible = (inst != 32'h00000013) &&
                                    !inst_is_system_barrier(inst) &&
                                    ((op == 7'b0110011) || // OP / M
                                     (op == 7'b0010011) || // OP-IMM
                                     (op == 7'b0110111) || // LUI
                                     (op == 7'b0010111) || // AUIPC
                                     (op == 7'b0000011) || // LOAD
                                     (op == 7'b0100011) || // STORE
                                     (op == 7'b1100011) || // BRANCH
                                     (op == 7'b1100111) || // JALR
                                     (op == 7'b1101111) || // JAL
                                     (op == 7'b0000111) || // LOAD-FP
                                     (op == 7'b0100111) || // STORE-FP
                                     (op == 7'b1010011) || // OP-FP
                                     (op == 7'b1000011) ||
                                     (op == 7'b1000111) ||
                                     (op == 7'b1001011) ||
                                     (op == 7'b1001111) ||
                                     inst_is_csr(inst));
        end
    endfunction

    function automatic inst_writes_rd;
        input [31:0] inst;
        reg [6:0] op;
        begin
            op = inst[6:0];
            inst_writes_rd = (op == 7'b0110011) ||
                             (op == 7'b0010011) ||
                             (op == 7'b0110111) ||
                             (op == 7'b0010111) ||
                             (op == 7'b0000011) ||
                             (op == 7'b1100111) ||
                             (op == 7'b1101111) ||
                             inst_is_csr(inst);
        end
    endfunction

    function automatic inst_uses_rs1;
        input [31:0] inst;
        reg [6:0] op;
        begin
            op = inst[6:0];
            inst_uses_rs1 = (op == 7'b0110011) ||
                            (op == 7'b0010011) ||
                            (op == 7'b0000011) ||
                            (op == 7'b0100011) ||
                            (op == 7'b1100011) ||
                            (op == 7'b1100111) ||
                            (op == 7'b0000111) ||
                            (op == 7'b0100111) ||
                            (inst_is_csr(inst) && !inst[14]);
        end
    endfunction

    function automatic inst_uses_rs2;
        input [31:0] inst;
        reg [6:0] op;
        begin
            op = inst[6:0];
            inst_uses_rs2 = (op == 7'b0110011) ||
                            (op == 7'b0100011) ||
                            (op == 7'b1100011);
        end
    endfunction

    wire lane0_fetch_mem = inst_is_mem(instr);
    wire lane1_fetch_mem = inst_is_mem(instr_lane1);
    wire lane0_fetch_csr = inst_is_csr(instr);
    wire lane1_fetch_csr = inst_is_csr(instr_lane1);
    wire lane0_fetch_control = inst_is_control(instr);
    wire lane1_fetch_control = inst_is_control(instr_lane1);
    wire lane0_fetch_writes = inst_writes_rd(instr);
    wire lane1_fetch_writes = inst_writes_rd(instr_lane1);
    wire [4:0] lane0_fetch_rd  = instr[11:7];
    wire [4:0] lane1_fetch_rs1 = instr_lane1[19:15];
    wire [4:0] lane1_fetch_rs2 = instr_lane1[24:20];
    wire [4:0] lane1_fetch_rd  = instr_lane1[11:7];
    wire lane1_fetch_uses_rs1 = inst_uses_rs1(instr_lane1);
    wire lane1_fetch_uses_rs2 = inst_uses_rs2(instr_lane1);
    wire lane1_rs1_dep = lane1_fetch_uses_rs1 & (lane0_fetch_rd == lane1_fetch_rs1);
    wire lane1_rs2_dep = lane1_fetch_uses_rs2 & (lane0_fetch_rd == lane1_fetch_rs2);
    wire lane1_raw_dep = lane0_fetch_writes & (lane0_fetch_rd != 5'd0) &
                         (lane1_rs1_dep | lane1_rs2_dep);
    wire lane1_waw_dep = lane0_fetch_writes & lane1_fetch_writes &
                         (lane0_fetch_rd != 5'd0) & (lane0_fetch_rd == lane1_fetch_rd);
    wire pair_uses_same_special =
        (lane0_fetch_mem && lane1_fetch_mem) ||
        (lane0_fetch_csr && lane1_fetch_csr) ||
        (lane0_fetch_control && lane1_fetch_control);
    wire fetch_two_valid = is_dual_lane_eligible(instr) &&
                           is_dual_lane_eligible(instr_lane1) &&
                           !lane0_fetch_control &&
                           !pair_uses_same_special &&
                           !lane1_raw_dep &&
                           !lane1_waw_dep &&
                           !(btb_hit && predict_taken) &&
                           !icache_stall &&
                           !icache_stall_lane1;

    // =========================================================================
    // LOGIC TRAP & INTERRUPT
    // =========================================================================
    // 1. Điều kiện thức dậy từ WFI (Chỉ cần Pending & Enable, bỏ qua Global MIE)
    wire wake_interrupt = (meip_i & mie_val[11]) | 
                          (msip_i & mie_val[3])  | 
                          (mtip_i & mie_val[7]);

    // 2. Điều kiện thực sự nhảy vào hàm ngắt (Phải có thêm Global MIE)
    wire is_external_irq = meip_i & mie_val[11] & mstatus_mie_val;
    wire is_software_irq = msip_i & mie_val[3]  & mstatus_mie_val;
    wire is_timer_irq    = mtip_i & mie_val[7]  & mstatus_mie_val;
    wire trap_interrupt_raw = is_external_irq | is_software_irq | is_timer_irq;
    assign ooo_interrupt_drain = ooo_enabled && trap_interrupt_raw && !ooo_backend_empty;
    wire trap_interrupt  = trap_interrupt_raw && !ooo_interrupt_drain;
    wire trap_sync_lane0 = ex_mem_ecall | ex_mem_ebreak;
    wire trap_sync_lane1 = !trap_sync_lane0 && (ex_mem_ecall_lane1 | ex_mem_ebreak_lane1);
    wire trap_enter      = trap_sync_lane0 | trap_sync_lane1 | trap_interrupt;
    wire mret_exec_arb   = ex_mem_mret | (!ex_mem_mret && ex_mem_mret_lane1);
    
    wire [31:0] trap_cause = (trap_interrupt && is_external_irq) ? 32'h8000000b :
                               (trap_interrupt && is_software_irq) ? 32'h80000003 :
                               (trap_interrupt && is_timer_irq)    ? 32'h80000007 :
                               ex_mem_ecall                        ? 32'd11       :
                               ex_mem_ebreak                       ? 32'd3        :
                               ex_mem_ecall_lane1                  ? 32'd11       :
                               ex_mem_ebreak_lane1                 ? 32'd3        : 32'd0;
    wire [31:0] trap_pc_value = trap_interrupt ? pc_in :
                                trap_sync_lane1 ? ex_mem_pc_in_lane1 : ex_mem_pc_in;

    // 1. TẦNG IF/ID
    wire icache_stall_any = icache_stall | icache_stall_lane1;
    wire mf_alu_stall_any = mf_alu_stall | mf_alu_stall_lane1;
    wire load_use_stall_lane1_from_lane0 =
        id_ex_mem_read && (id_ex_rd != 5'd0) &&
        ((id_ex_rd == rs1_lane1) || (id_ex_rd == rs2_lane1));
    wire load_use_stall_lane0_from_lane1 =
        id_ex_mem_read_lane1 && (id_ex_rd_lane1 != 5'd0) &&
        ((id_ex_rd_lane1 == rs1) || (id_ex_rd_lane1 == rs2));
    wire load_use_stall_lane1_from_lane1 =
        id_ex_mem_read_lane1 && (id_ex_rd_lane1 != 5'd0) &&
        ((id_ex_rd_lane1 == rs1_lane1) || (id_ex_rd_lane1 == rs2_lane1));
    assign load_use_stall = load_use_stall_raw |
                            load_use_stall_lane1_from_lane0 |
                            load_use_stall_lane0_from_lane1 |
                            load_use_stall_lane1_from_lane1;

    assign ooo_enabled = (ENABLE_TOMASULO_INTEGER != 0);
    assign rob_decode_valid = riscv_start && !riscv_done &&
                              (if_id_instr != 32'h00000013);
    assign rob_decode_valid_lane1 = rob_decode_valid &&
                                    (if_id_instr_lane1 != 32'h00000013);

    assign legacy_pipe_busy =
        id_ex_reg_write | id_ex_mem_write | id_ex_mem_read |
        id_ex_branch | id_ex_jal | id_ex_jalr |
        id_ex_csr_we | id_ex_md_type | id_ex_fpu_en |
        id_ex_f_reg_write | id_ex_f_mem_write |
        id_ex_ecall | id_ex_ebreak | id_ex_mret |
        id_ex_reg_write_lane1 | id_ex_mem_write_lane1 | id_ex_mem_read_lane1 |
        id_ex_branch_lane1 | id_ex_jal_lane1 | id_ex_jalr_lane1 |
        id_ex_csr_we_lane1 | id_ex_md_type_lane1 | id_ex_fpu_en_lane1 |
        id_ex_f_reg_write_lane1 | id_ex_f_mem_write_lane1 |
        id_ex_ecall_lane1 | id_ex_ebreak_lane1 | id_ex_mret_lane1 |
        ex_mem_reg_write | ex_mem_mem_write | ex_mem_mem_read |
        ex_mem_branch | ex_mem_jal | ex_mem_csr_we |
        ex_mem_f_reg_write | ex_mem_f_mem_write |
        ex_mem_ecall | ex_mem_ebreak | ex_mem_mret |
        ex_mem_reg_write_lane1 | ex_mem_mem_write_lane1 | ex_mem_mem_read_lane1 |
        ex_mem_branch_lane1 | ex_mem_jal_lane1 | ex_mem_csr_we_lane1 |
        ex_mem_f_reg_write_lane1 | ex_mem_f_mem_write_lane1 |
        ex_mem_ecall_lane1 | ex_mem_ebreak_lane1 | ex_mem_mret_lane1 |
        mem_wb_reg_write | mem_wb_reg_write_lane1 |
        mem_wb_f_reg_write | mem_wb_f_reg_write_lane1 |
        mem_wb_ecall | mem_wb_ecall_lane1;

    assign ooo_lane0_alu_candidate =
        ooo_enabled && rob_decode_valid &&
        is_dual_int_eligible(if_id_instr) &&
        !md_type && !fpu_en && !csr_we && !mem_read && !mem_write &&
        !branch && !jal && !jalr && !ecall && !ebreak && !mret;
    assign ooo_lane0_mem_candidate =
        ooo_enabled && rob_decode_valid &&
        ((opcode == 7'b0000011) || (opcode == 7'b0100011)) &&
        (mem_read || mem_write) &&
        !f_mem_to_reg && !f_mem_write && !fpu_en && !csr_we &&
        !branch && !jal && !jalr && !ecall && !ebreak && !mret;
    wire ooo_lane1_alu_pre_candidate =
        rob_decode_valid_lane1 &&
        is_dual_int_eligible(if_id_instr_lane1) &&
        !md_type_lane1 && !fpu_en_lane1 && !csr_we_lane1 &&
        !mem_read_lane1 && !mem_write_lane1 &&
        !branch_lane1 && !jal_lane1 && !jalr_lane1 &&
        !ecall_lane1 && !ebreak_lane1 && !mret_lane1;
    wire ooo_lane1_mem_pre_candidate =
        rob_decode_valid_lane1 &&
        ((opcode_lane1 == 7'b0000011) || (opcode_lane1 == 7'b0100011)) &&
        (mem_read_lane1 || mem_write_lane1) &&
        !f_mem_to_reg_lane1 && !f_mem_write_lane1 && !fpu_en_lane1 && !csr_we_lane1 &&
        !branch_lane1 && !jal_lane1 && !jalr_lane1 &&
        !ecall_lane1 && !ebreak_lane1 && !mret_lane1;
    wire ooo_lane1_pre_candidate =
        ooo_lane1_alu_pre_candidate | ooo_lane1_mem_pre_candidate;
    assign ooo_lane0_candidate =
        (ooo_lane0_alu_candidate | ooo_lane0_mem_candidate) &&
        (!rob_decode_valid_lane1 || ooo_lane1_pre_candidate);
    assign ooo_lane1_alu_candidate =
        ooo_lane0_candidate && ooo_lane1_alu_pre_candidate;
    assign ooo_lane1_mem_candidate =
        ooo_lane0_candidate && ooo_lane1_mem_pre_candidate;
    assign ooo_lane1_candidate = ooo_lane1_alu_candidate | ooo_lane1_mem_candidate;

    assign legacy_decode_valid_lane0 = rob_decode_valid && !ooo_lane0_candidate;
    assign legacy_decode_valid_lane1 = rob_decode_valid_lane1 && !ooo_lane1_candidate;

    assign legacy_rob_alloc_stall =
        (legacy_decode_valid_lane0 && !rob_dispatch_ready[0]) ||
        (legacy_decode_valid_lane1 && !rob_dispatch_ready[1]);
    assign ooo_backend_alloc_stall =
        !legacy_pipe_busy &&
        ((ooo_lane0_candidate && !ooo_dispatch_ready[0]) ||
         (ooo_lane1_candidate && !ooo_dispatch_ready[1]));
    assign ooo_decode_blocked_by_legacy =
        (ooo_lane0_candidate || ooo_lane1_candidate) && legacy_pipe_busy;
    assign legacy_decode_waits_for_ooo =
        ooo_enabled && (legacy_decode_valid_lane0 || legacy_decode_valid_lane1) &&
        !ooo_backend_empty;

    wire stall_if_id = dcache_stall | mf_alu_stall_any | load_use_stall | stall_ID | rob_alloc_stall;
    wire flush_if_id = flush_trap | flush_branch | flush_jal | icache_stall_any;

    // 2. TẦNG ID/EX
    wire stall_id_ex = dcache_stall | mf_alu_stall_any | stall_EX;
    wire flush_id_ex_base = flush_trap | flush_branch | flush_jal | load_use_stall;
    wire flush_id_ex = flush_id_ex_base | rob_alloc_stall;

    assign rob_alloc_stall = !flush_id_ex_base &&
                             (legacy_rob_alloc_stall |
                              ooo_backend_alloc_stall |
                              ooo_decode_blocked_by_legacy |
                              legacy_decode_waits_for_ooo |
                              ooo_interrupt_drain);

    assign legacy_dispatch_valid = {
        legacy_decode_valid_lane1 && !stall_id_ex && !flush_id_ex,
        legacy_decode_valid_lane0 && !stall_id_ex && !flush_id_ex
    };

    assign ooo_dispatch_valid = {
        ooo_lane1_candidate && !legacy_pipe_busy && !stall_id_ex && !flush_id_ex,
        ooo_lane0_candidate && !legacy_pipe_busy && !stall_id_ex && !flush_id_ex
    };
    assign ooo_lane0_fire = ooo_dispatch_valid[0] && ooo_dispatch_ready[0];
    assign ooo_lane1_fire = ooo_dispatch_valid[1] && ooo_dispatch_ready[1];

    // 3. TẦNG EX/MEM
    wire stall_ex_mem = dcache_stall | stall_MEM;
    wire flush_ex_mem = flush_trap | mf_alu_stall_any;

    // 4. TẦNG MEM/WB
    wire stall_mem_wb = dcache_stall | stall_WB;
    wire flush_mem_wb = flush_trap;

    // =========================================================================
    // PROGRAM COUNTER & FLUSH LOGIC
    // =========================================================================
    reg [31:0] pc_reg;
    always @(posedge clk) begin
        if (!reset_n) begin
            pc_reg <= reset_vector_in;
        end else if (riscv_start && !riscv_done) begin
            if (dbg_halted && !dbg_resume_req) begin
                pc_reg <= dpc_out;
            end else if (flush_trap) begin
                pc_reg <= pc_out;  // Ưu tiên 1 (Già nhất)
            end else if (flush_branch) begin
                pc_reg <= pc_out;  // Ưu tiên 2 
            end else if (flush_jal) begin
                pc_reg <= pc_out;  // Ưu tiên 3 (Trẻ nhất)
            end else if (!stall_IF && !load_use_stall && !icache_stall_any && !dcache_stall && !mf_alu_stall_any && !rob_alloc_stall) begin
                pc_reg <= pc_out;  // Chạy bình thường (Chỉ tiến lên khi không ai kẹt)
            end
        end
    end
    
    assign pc_in = pc_reg;
    reg flush_temp;
    always @(posedge clk) begin
        if (!reset_n) begin
            flush_temp <= 1'b0;
        end else if (riscv_start && !riscv_done) begin
            flush_temp <= flush_branch || flush_jal || flush_trap;
        end
    end

    wire lane0_jal_redirect = id_ex_jal | id_ex_jalr;
    wire lane1_jal_redirect =
        !lane0_jal_redirect && (id_ex_jal_lane1 | id_ex_jalr_lane1);
    wire id_ex_jal_arb = id_ex_jal | (lane1_jal_redirect & id_ex_jal_lane1);
    wire id_ex_jalr_arb = id_ex_jalr | (lane1_jal_redirect & id_ex_jalr_lane1);
    wire [31:0] id_ex_jal_target_arb =
        lane1_jal_redirect ? id_ex_jal_target_lane1 : id_ex_jal_target;
    wire [31:0] id_ex_ext_imm_jalr_arb =
        lane1_jal_redirect ? id_ex_ext_imm_lane1 : id_ex_ext_imm;
    wire [31:0] alu_in1_jalr_arb =
        lane1_jal_redirect ? alu_in1_lane1 : alu_in1;

    wire bpu_select_lane1 =
        !(ex_mem_branch | ex_mem_jal | ex_mem_predict_taken) &&
        ex_mem_branch_lane1;
    wire [31:0] ex_mem_branch_target_arb =
        bpu_select_lane1 ? ex_mem_branch_target_lane1 : ex_mem_branch_target;
    wire [31:0] ex_mem_pc_in_arb =
        bpu_select_lane1 ? ex_mem_pc_in_lane1 : ex_mem_pc_in;
    wire ex_mem_branch_arb =
        bpu_select_lane1 ? ex_mem_branch_lane1 : ex_mem_branch;
    wire ex_mem_branch_taken_arb =
        bpu_select_lane1 ? ex_mem_branch_taken_lane1 : ex_mem_branch_taken;
    wire ex_mem_predict_taken_arb =
        bpu_select_lane1 ? ex_mem_predict_taken_lane1 : ex_mem_predict_taken;
    wire ex_mem_btb_hit_arb =
        bpu_select_lane1 ? ex_mem_btb_hit_lane1 : ex_mem_btb_hit;
    assign bpu_correct = bpu_correct_selected;
    assign actual_taken = actual_taken_selected;

    wire [31:0] final_mem_write_data =
        ex_mem_f_mem_write ? ex_mem_f_store_data : ex_mem_mem_write_data;
    wire [31:0] final_mem_write_data_lane1 =
        ex_mem_f_mem_write_lane1 ? ex_mem_f_store_data_lane1 :
                                   ex_mem_mem_write_data_lane1;
    wire legacy_lane0_mem_req =
        legacy_lane0_dcache_read_req | legacy_lane0_dcache_write_req;
    wire legacy_lane1_dcache_read_req = ex_mem_mem_read_lane1;
    wire legacy_lane1_dcache_write_req =
        ex_mem_mem_write_lane1 | ex_mem_f_mem_write_lane1;
    wire legacy_lane1_mem_req =
        legacy_lane1_dcache_read_req | legacy_lane1_dcache_write_req;
    wire legacy_lane1_dcache_grant =
        legacy_lane1_mem_req && !legacy_lane0_mem_req;

    assign legacy_dcache_read_req =
        legacy_lane0_dcache_read_req |
        (legacy_lane1_dcache_grant & legacy_lane1_dcache_read_req);
    assign legacy_dcache_write_req =
        legacy_lane0_dcache_write_req |
        (legacy_lane1_dcache_grant & legacy_lane1_dcache_write_req);
    assign legacy_dcache_addr =
        legacy_lane1_dcache_grant ? ex_mem_alu_result_lane1 :
                                    legacy_lane0_dcache_addr;
    assign legacy_dcache_write_data =
        legacy_lane1_dcache_grant ? final_mem_write_data_lane1 :
                                    legacy_lane0_dcache_write_data;
    assign legacy_dcache_mem_size =
        legacy_lane1_dcache_grant ? ex_mem_mem_size_lane1 : ex_mem_mem_size;
    assign legacy_dcache_mem_unsigned =
        legacy_lane1_dcache_grant ? ex_mem_mem_unsigned_lane1 :
                                    ex_mem_mem_unsigned;
    assign mem_read_data_lane1 =
        legacy_lane1_dcache_grant ? dcache_read_data : 32'd0;

    assign ooo_dcache_active = ooo_dcache_read_req | ooo_dcache_write_req;
    assign dcache_read_req = ooo_dcache_read_req | legacy_dcache_read_req;
    assign dcache_write_req = ooo_dcache_write_req | legacy_dcache_write_req;
    assign dcache_addr = ooo_dcache_active ? ooo_dcache_addr : legacy_dcache_addr;
    assign dcache_write_data =
        ooo_dcache_active ? ooo_dcache_write_data : legacy_dcache_write_data;
    assign mem_size_top =
        ooo_dcache_active ? ooo_dcache_mem_size : legacy_dcache_mem_size;
    assign mem_unsigned_top =
        ooo_dcache_active ? ooo_dcache_mem_unsigned : legacy_dcache_mem_unsigned;
    wire is_sleeping_internal;
    assign wfi_sleep_out = is_sleeping_internal;

    // =========================================================================
    // INSTRUCTION FETCH (IF)
    // =========================================================================
    instruction_fetch IF (
        .reset_n(reset_n),
        .flush_temp(flush_temp),
        .trap_enter(trap_enter),
        .mret_exec(mret_exec_arb),
        .reset_vector_in(reset_vector_in),
        .mtvec_in(mtvec_pc),
        .mepc_in(mepc_pc),
        .ex_mem_branch_target(ex_mem_branch_target_arb),
        .id_ex_jal_target(id_ex_jal_target_arb),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in_arb),
        .id_ex_jalr(id_ex_jalr_arb),
        .id_ex_jal(id_ex_jal_arb),
        .btb_hit(btb_hit),
        .alu_in1(alu_in1_jalr_arb),
        .id_ex_ext_imm(id_ex_ext_imm_jalr_arb),
        .predict_taken(predict_taken),
        .actual_taken(actual_taken),
        .bpu_correct(bpu_correct),
        .predict_target(predict_target),
        .fetch_two_valid(fetch_two_valid),
        .pc_out(pc_out),
        .pc_plus_4(pc_plus_4),
        .pc_plus_8(pc_plus_8),
        .instr(instr),
        .instr_lane1(instr_lane1),
        .icache_read_req(icache_read_req),
        .icache_read_req_lane1(icache_read_req_lane1),
        .icache_addr(icache_addr),
        .icache_addr_lane1(icache_addr_lane1),
        .icache_read_data(icache_read_data),
        .icache_read_data_lane1(icache_read_data_lane1)
    );

    // =========================================================================
    // IF/ID REGISTER
    // =========================================================================
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

    if_id_register IF_ID_LANE1 (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_if_id), 
        .flush(flush_if_id),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .instr(fetch_two_valid ? instr_lane1 : 32'h00000013),
        .pc_plus_4(pc_plus_8),
        .pc_in(pc_in + 32'd4),
        .predict_taken(1'b0),
        .btb_hit(1'b0),
        .if_id_instr(if_id_instr_lane1),
        .if_id_pc_plus_4(if_id_pc_plus_4_lane1),
        .if_id_pc_in(if_id_pc_in_lane1),
        .if_id_predict_taken(if_id_predict_taken_lane1),
        .if_id_btb_hit(if_id_btb_hit_lane1)
    );

    // =========================================================================
    // INSTRUCTION DECODE (ID)
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
        .fpu_en(fpu_en),
        .f_reg_write(f_reg_write),
        .f_mem_to_reg(f_mem_to_reg),
        .f_mem_write(f_mem_write),
        .f_to_x(f_to_x),
        .x_to_f(x_to_f),
        .fpu_operation(fpu_operation),
        .wfi_req(wfi_req_internal)
    );

    instruction_decode ID_LANE1 (
        .if_id_pc_in(if_id_pc_in_lane1),
        .if_id_instr(if_id_instr_lane1),
        .ext_imm(ext_imm_lane1),
        .rs1(rs1_lane1),
        .rs2(rs2_lane1),
        .rd(rd_lane1),
        .funct3(funct3_lane1),
        .opcode(opcode_lane1),
        .funct7(funct7_lane1),
        .jal_target(jal_target_lane1),
        .branch_target(branch_target_lane1),
        .reg_write(reg_write_lane1),
        .alu_src(alu_src_lane1),
        .mem_write(mem_write_lane1),
        .mem_read(mem_read_lane1),
        .mem_to_reg(mem_to_reg_lane1),
        .branch(branch_lane1),
        .jal(jal_lane1),
        .jalr(jalr_lane1),
        .lui(lui_lane1),
        .auipc(auipc_lane1),
        .mem_unsigned(mem_unsigned_lane1),
        .alu_op(alu_op_lane1),
        .mem_size(mem_size_lane1),
        .alu_ctrl(alu_ctrl_lane1),
        .md_type(md_type_lane1),
        .md_operation(md_operation_lane1),
        .ecall(ecall_lane1),
        .ebreak(ebreak_lane1),
        .mret(mret_lane1),
        .csr_addr(csr_addr_lane1),
        .csr_op(csr_op_lane1),
        .csr_we(csr_we_lane1),
        .fpu_en(fpu_en_lane1),
        .f_reg_write(f_reg_write_lane1),
        .f_mem_to_reg(f_mem_to_reg_lane1),
        .f_mem_write(f_mem_write_lane1),
        .f_to_x(f_to_x_lane1),
        .x_to_f(x_to_f_lane1),
        .fpu_operation(fpu_operation_lane1),
        .wfi_req(wfi_req_lane1)
    );

    // --- LOGIC PHÂN LUỒNG ĐỊA CHỈ DEBUG (ĐÃ GỘP) ---
    wire [31:0] rf_dbg_read_data;
    wire [31:0] frf_dbg_read_data;
    wire [31:0] csr_dbg_read_data; // Mới: Nhận data từ toàn bộ CSR
    
    // Phân rã theo tiền tố (Prefix) của chuẩn Abstract Command
    wire is_csr   = (dbg_reg_read_addr[15:12] == 4'h0); // Dải 0x0000 - 0x0FFF
    wire is_gpr   = (dbg_reg_read_addr[15:5]  == 11'h080); // Dải 0x1000 - 0x101F
    wire is_fpr   = (dbg_reg_read_addr[15:5]  == 11'h081); // Dải 0x1020 - 0x103F

    // Chỉ sinh tín hiệu write_en khi địa chỉ map đúng khu vực
    wire dbg_gpr_we = dbg_reg_write_en & is_gpr;
    wire dbg_fpr_we = dbg_reg_write_en & is_fpr;
    wire dbg_csr_we = dbg_reg_write_en & is_csr;

    // 1 bộ MUX duy nhất trả về cho OpenOCD
    assign dbg_reg_read_data = is_csr ? csr_dbg_read_data :
                               is_gpr ? rf_dbg_read_data :
                               is_fpr ? frf_dbg_read_data : 
                               32'h0;

    // =========================================================================
    // CSR REGISTER FILE
    // =========================================================================
    wire csr_write_lane0 = ex_mem_csr_we;
    wire csr_write_lane1 = ex_mem_csr_we_lane1 && !csr_write_lane0;
    wire csr_write_en_arb = csr_write_lane0 | csr_write_lane1;
    wire [11:0] csr_write_addr_arb =
        csr_write_lane1 ? ex_mem_csr_addr_lane1 : ex_mem_csr_addr;
    wire [31:0] csr_write_data_arb =
        csr_write_lane1 ? ex_mem_csr_write_data_lane1 : ex_mem_csr_write_data;
    wire [1:0] csr_write_op_arb =
        csr_write_lane1 ? ex_mem_csr_op_lane1 : ex_mem_csr_op;

    csr_register_file CSR_RF (
        .clk(clk),
        .reset_n(reset_n),
        .meip_i(meip_i),
        .msip_i(msip_i),
        .mtip_i(mtip_i),
        .csr_addr(id_ex_csr_addr),
        .csr_read_data(csr_read_data_raw),
        .csr_addr_lane1(id_ex_csr_addr_lane1),
        .csr_read_data_lane1(csr_read_data_raw_lane1),
        .csr_write_addr(csr_write_addr_arb),
        .csr_write_data(csr_write_data_arb),
        .csr_op(csr_write_op_arb),
        .csr_write_en(csr_write_en_arb),
        .count_en(!dbg_halted),
        .instret_en(!dbg_halted),
        .trap_enter(trap_enter),
        .mret_exec(mret_exec_arb),
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
        // --- CÁC PORT MỚI CHO DEBUG CSR ---
        .dbg_reg_read_addr(dbg_reg_read_addr[11:0]),
        .dbg_read_data(csr_dbg_read_data),
        .dbg_reg_write_en(dbg_csr_we),
        .dbg_reg_write_addr(dbg_reg_write_addr[11:0]),
        .dbg_reg_write_data(dbg_reg_write_data)
    );
    assign csr_read_data_fwd =
        (csr_write_en_arb && (csr_write_addr_arb == id_ex_csr_addr)) ?
        csr_write_data_arb : csr_read_data_raw;
    assign csr_read_data_fwd_lane1 =
        (csr_write_en_arb && (csr_write_addr_arb == id_ex_csr_addr_lane1)) ?
        csr_write_data_arb : csr_read_data_raw_lane1;

    // =========================================================================
    // INTEGER REGISTER FILE (WITH DEBUG ACCESS)
    // =========================================================================
    register_file RF (
        .clk(clk),
        .reset_n(reset_n),
        .read_reg1(rs1),
        .read_reg2(rs2),
        .read_reg1_lane1(rs1_lane1),
        .read_reg2_lane1(rs2_lane1),
        .mem_wb_reg_write(mem_wb_reg_write),
        .mem_wb_rd(mem_wb_rd),
        .mem_wb_write_data(wb_write_data),
        .mem_wb_reg_write_lane1(mem_wb_reg_write_lane1),
        .mem_wb_rd_lane1(mem_wb_rd_lane1),
        .mem_wb_write_data_lane1(wb_write_data_lane1),
        .ooo_commit_valid0(ooo_commit_valid[0] & ooo_commit_rd_valid[0]),
        .ooo_commit_rd0(ooo_commit_rd[0*5 +: 5]),
        .ooo_commit_data0(ooo_commit_data[0*32 +: 32]),
        .ooo_commit_valid1(ooo_commit_valid[1] & ooo_commit_rd_valid[1]),
        .ooo_commit_rd1(ooo_commit_rd[1*5 +: 5]),
        .ooo_commit_data1(ooo_commit_data[1*32 +: 32]),
        .read_data1(read_data1_temp),
        .read_data2(read_data2_temp),
        .read_data1_lane1(read_data1_temp_lane1),
        .read_data2_lane1(read_data2_temp_lane1),
        .dbg_mode(dbg_halted),
        .dbg_read_addr(dbg_reg_read_addr[4:0]),
        .dbg_read_data(rf_dbg_read_data),
        .dbg_write_en(dbg_gpr_we),
        .dbg_write_addr(dbg_reg_write_addr[4:0]),
        .dbg_write_data(dbg_reg_write_data)
    );
    assign read_data1 = (rs1 != 5'd0 && rs1 == mem_wb_rd_lane1 && mem_wb_reg_write_lane1) ? wb_write_data_lane1 :
                        (rs1 != 5'd0 && rs1 == mem_wb_rd && mem_wb_reg_write) ? wb_write_data :
                        (rs1 != 5'd0 && rs1 == ooo_commit_rd[1*5 +: 5] &&
                         (ooo_commit_valid[1] & ooo_commit_rd_valid[1])) ? ooo_commit_data[1*32 +: 32] :
                        (rs1 != 5'd0 && rs1 == ooo_commit_rd[0*5 +: 5] &&
                         (ooo_commit_valid[0] & ooo_commit_rd_valid[0])) ? ooo_commit_data[0*32 +: 32] :
                        read_data1_temp;
    assign read_data2 = (rs2 != 5'd0 && rs2 == mem_wb_rd_lane1 && mem_wb_reg_write_lane1) ? wb_write_data_lane1 :
                        (rs2 != 5'd0 && rs2 == mem_wb_rd && mem_wb_reg_write) ? wb_write_data :
                        (rs2 != 5'd0 && rs2 == ooo_commit_rd[1*5 +: 5] &&
                         (ooo_commit_valid[1] & ooo_commit_rd_valid[1])) ? ooo_commit_data[1*32 +: 32] :
                        (rs2 != 5'd0 && rs2 == ooo_commit_rd[0*5 +: 5] &&
                         (ooo_commit_valid[0] & ooo_commit_rd_valid[0])) ? ooo_commit_data[0*32 +: 32] :
                        read_data2_temp;
    assign read_data1_lane1 = (rs1_lane1 != 5'd0 && rs1_lane1 == mem_wb_rd_lane1 && mem_wb_reg_write_lane1) ? wb_write_data_lane1 :
                              (rs1_lane1 != 5'd0 && rs1_lane1 == mem_wb_rd && mem_wb_reg_write) ? wb_write_data :
                              (rs1_lane1 != 5'd0 && rs1_lane1 == ooo_commit_rd[1*5 +: 5] &&
                               (ooo_commit_valid[1] & ooo_commit_rd_valid[1])) ? ooo_commit_data[1*32 +: 32] :
                              (rs1_lane1 != 5'd0 && rs1_lane1 == ooo_commit_rd[0*5 +: 5] &&
                               (ooo_commit_valid[0] & ooo_commit_rd_valid[0])) ? ooo_commit_data[0*32 +: 32] :
                              read_data1_temp_lane1;
    assign read_data2_lane1 = (rs2_lane1 != 5'd0 && rs2_lane1 == mem_wb_rd_lane1 && mem_wb_reg_write_lane1) ? wb_write_data_lane1 :
                              (rs2_lane1 != 5'd0 && rs2_lane1 == mem_wb_rd && mem_wb_reg_write) ? wb_write_data :
                              (rs2_lane1 != 5'd0 && rs2_lane1 == ooo_commit_rd[1*5 +: 5] &&
                               (ooo_commit_valid[1] & ooo_commit_rd_valid[1])) ? ooo_commit_data[1*32 +: 32] :
                              (rs2_lane1 != 5'd0 && rs2_lane1 == ooo_commit_rd[0*5 +: 5] &&
                               (ooo_commit_valid[0] & ooo_commit_rd_valid[0])) ? ooo_commit_data[0*32 +: 32] :
                              read_data2_temp_lane1;

    // =========================================================================
    // FLOATING POINT REGISTER FILE
    // =========================================================================
    f_register_file F_RF (
        .clk(clk),
        .reset_n(reset_n),
        .read_reg1(rs1),
        .read_reg2(rs2),
        .read_reg1_lane1(rs1_lane1),
        .read_reg2_lane1(rs2_lane1),
        .read_data1(read_f_data1_temp),
        .read_data2(read_f_data2_temp),
        .read_data1_lane1(read_f_data1_temp_lane1),
        .read_data2_lane1(read_f_data2_temp_lane1),
        .reg_write_en(mem_wb_f_reg_write),
        .write_reg(mem_wb_rd),
        .write_data(wb_f_write_data),
        .reg_write_en_lane1(mem_wb_f_reg_write_lane1),
        .write_reg_lane1(mem_wb_rd_lane1),
        .write_data_lane1(wb_f_write_data_lane1),
        
        // --- THÊM KẾT NỐI DEBUG ---
        .dbg_mode(dbg_halted),
        .dbg_read_addr(dbg_reg_read_addr[4:0]),
        .dbg_read_data(frf_dbg_read_data),
        .dbg_write_en(dbg_fpr_we),
        .dbg_write_addr(dbg_reg_write_addr[4:0]),
        .dbg_write_data(dbg_reg_write_data)
    );
    assign read_f_data1 = (rs1 == mem_wb_rd && mem_wb_f_reg_write) ? wb_f_write_data : read_f_data1_temp;
    assign read_f_data2 = (rs2 == mem_wb_rd && mem_wb_f_reg_write) ? wb_f_write_data : read_f_data2_temp;
    assign read_f_data1_lane1 =
        (rs1_lane1 == mem_wb_rd_lane1 && mem_wb_f_reg_write_lane1) ? wb_f_write_data_lane1 :
        (rs1_lane1 == mem_wb_rd && mem_wb_f_reg_write) ? wb_f_write_data :
        read_f_data1_temp_lane1;
    assign read_f_data2_lane1 =
        (rs2_lane1 == mem_wb_rd_lane1 && mem_wb_f_reg_write_lane1) ? wb_f_write_data_lane1 :
        (rs2_lane1 == mem_wb_rd && mem_wb_f_reg_write) ? wb_f_write_data :
        read_f_data2_temp_lane1;

    // =========================================================================
    // ID/EX REGISTER
    // =========================================================================
    id_ex_register ID_EX (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_id_ex), 
        .flush(flush_id_ex | ooo_lane0_candidate),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .rob_tag(rob_dispatch_tag[0*ROB_TAG_W +: ROB_TAG_W]),
        .rob_valid(legacy_dispatch_valid[0] & rob_dispatch_ready[0]),
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
        .fpu_en(fpu_en),
        .f_reg_write(f_reg_write),
        .f_mem_to_reg(f_mem_to_reg),
        .f_mem_write(f_mem_write),
        .f_to_x(f_to_x),
        .x_to_f(x_to_f),
        .fpu_operation(fpu_operation),
        .read_f_data1(read_f_data1),
        .read_f_data2(read_f_data2),
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

    id_ex_register ID_EX_LANE1 (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_id_ex), 
        .flush(flush_id_ex | ooo_lane1_candidate),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .rob_tag(rob_dispatch_tag[1*ROB_TAG_W +: ROB_TAG_W]),
        .rob_valid(legacy_dispatch_valid[1] & rob_dispatch_ready[1]),
        .if_id_pc_plus_4(if_id_pc_plus_4_lane1),
        .if_id_pc_in(if_id_pc_in_lane1),
        .funct3(funct3_lane1),
        .read_data1(read_data1_lane1),
        .read_data2(read_data2_lane1),
        .ext_imm(ext_imm_lane1),
        .rs1(rs1_lane1),
        .rs2(rs2_lane1),
        .rd(rd_lane1),
        .reg_write(reg_write_lane1),
        .alu_src(alu_src_lane1),
        .mem_write(mem_write_lane1),
        .mem_read(mem_read_lane1),
        .mem_to_reg(mem_to_reg_lane1),
        .branch(branch_lane1),
        .jal(jal_lane1),
        .jalr(jalr_lane1),
        .lui(lui_lane1),
        .auipc(auipc_lane1),
        .mem_unsigned(mem_unsigned_lane1),
        .mem_size(mem_size_lane1),
        .alu_ctrl(alu_ctrl_lane1),
        .branch_target(branch_target_lane1),
        .jal_target(jal_target_lane1),
        .if_id_predict_taken(if_id_predict_taken_lane1),
        .if_id_btb_hit(if_id_btb_hit_lane1),
        .ecall(ecall_lane1),
        .ebreak(ebreak_lane1),
        .mret(mret_lane1),
        .csr_addr(csr_addr_lane1),
        .csr_op(csr_op_lane1),
        .csr_we(csr_we_lane1),
        .md_type(md_type_lane1),
        .md_operation(md_operation_lane1),
        .if_id_instr(if_id_instr_lane1),
        .fpu_en(fpu_en_lane1),
        .f_reg_write(f_reg_write_lane1),
        .f_mem_to_reg(f_mem_to_reg_lane1),
        .f_mem_write(f_mem_write_lane1),
        .f_to_x(f_to_x_lane1),
        .x_to_f(x_to_f_lane1),
        .fpu_operation(fpu_operation_lane1),
        .read_f_data1(read_f_data1_lane1),
        .read_f_data2(read_f_data2_lane1),
        .id_ex_pc_plus_4(id_ex_pc_plus_4_lane1),
        .id_ex_pc_in(id_ex_pc_in_lane1),
        .id_ex_funct3(id_ex_funct3_lane1),
        .id_ex_read_data1(id_ex_read_data1_lane1),
        .id_ex_read_data2(id_ex_read_data2_lane1),
        .id_ex_ext_imm(id_ex_ext_imm_lane1),
        .id_ex_rs1(id_ex_rs1_lane1),
        .id_ex_rs2(id_ex_rs2_lane1),
        .id_ex_rd(id_ex_rd_lane1),
        .id_ex_reg_write(id_ex_reg_write_lane1),
        .id_ex_alu_src(id_ex_alu_src_lane1),
        .id_ex_mem_write(id_ex_mem_write_lane1),
        .id_ex_mem_read(id_ex_mem_read_lane1),
        .id_ex_mem_to_reg(id_ex_mem_to_reg_lane1),
        .id_ex_branch(id_ex_branch_lane1),
        .id_ex_jal(id_ex_jal_lane1),
        .id_ex_jalr(id_ex_jalr_lane1),
        .id_ex_lui(id_ex_lui_lane1),
        .id_ex_auipc(id_ex_auipc_lane1),
        .id_ex_mem_unsigned(id_ex_mem_unsigned_lane1),
        .id_ex_mem_size(id_ex_mem_size_lane1),
        .id_ex_alu_ctrl(id_ex_alu_ctrl_lane1),
        .id_ex_branch_target(id_ex_branch_target_lane1),
        .id_ex_jal_target(id_ex_jal_target_lane1),
        .id_ex_predict_taken(id_ex_predict_taken_lane1),
        .id_ex_btb_hit(id_ex_btb_hit_lane1),
        .id_ex_ecall(id_ex_ecall_lane1),
        .id_ex_ebreak(id_ex_ebreak_lane1),
        .id_ex_mret(id_ex_mret_lane1),
        .id_ex_csr_addr(id_ex_csr_addr_lane1),
        .id_ex_csr_op(id_ex_csr_op_lane1),
        .id_ex_csr_we(id_ex_csr_we_lane1),
        .id_ex_md_type(id_ex_md_type_lane1),
        .id_ex_md_operation(id_ex_md_operation_lane1),
        .id_ex_instr(id_ex_instr_lane1),
        .id_ex_fpu_en(id_ex_fpu_en_lane1),
        .id_ex_f_reg_write(id_ex_f_reg_write_lane1),
        .id_ex_f_mem_to_reg(id_ex_f_mem_to_reg_lane1),
        .id_ex_f_mem_write(id_ex_f_mem_write_lane1),
        .id_ex_f_to_x(id_ex_f_to_x_lane1),
        .id_ex_x_to_f(id_ex_x_to_f_lane1),
        .id_ex_fpu_operation(id_ex_fpu_operation_lane1),
        .id_ex_read_f_data1(id_ex_read_f_data1_lane1),
        .id_ex_read_f_data2(id_ex_read_f_data2_lane1),
        .id_ex_rob_tag(id_ex_rob_tag_lane1),
        .id_ex_rob_valid(id_ex_rob_valid_lane1)
    );

    // =========================================================================
    // FORWARDING UNIT
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
        .alu_in1(alu_in1_base),
        .alu_in2(alu_in2_base),
        .mem_write_data(mem_write_data_base),
        .id_ex_read_f_data1(id_ex_read_f_data1),
        .id_ex_read_f_data2(id_ex_read_f_data2),
        .ex_mem_f_reg_write(ex_mem_f_reg_write),
        .mem_wb_f_reg_write(mem_wb_f_reg_write),
        .ex_mem_fpu_result(ex_mem_fpu_result),
        .mem_wb_f_write_data(wb_f_write_data),
        .fpu_in1(fpu_in1_base),
        .fpu_in2(fpu_in2_base)
    );

    wire lane0_ex_fwd_hit_a = ex_mem_reg_write_lane1 && (ex_mem_rd_lane1 != 5'd0) &&
                              (ex_mem_rd_lane1 == id_ex_rs1);
    wire lane0_ex_fwd_hit_b = ex_mem_reg_write_lane1 && (ex_mem_rd_lane1 != 5'd0) &&
                              (ex_mem_rd_lane1 == id_ex_rs2);
    wire lane0_wb1_fwd_hit_a = mem_wb_reg_write_lane1 && (mem_wb_rd_lane1 != 5'd0) &&
                               (mem_wb_rd_lane1 == id_ex_rs1);
    wire lane0_wb1_fwd_hit_b = mem_wb_reg_write_lane1 && (mem_wb_rd_lane1 != 5'd0) &&
                               (mem_wb_rd_lane1 == id_ex_rs2);

    assign alu_in1 = lane0_ex_fwd_hit_a ? ex_mem_alu_result_lane1 :
                     lane0_wb1_fwd_hit_a ? wb_write_data_lane1 :
                     alu_in1_base;
    assign mem_write_data = lane0_ex_fwd_hit_b ? ex_mem_alu_result_lane1 :
                            lane0_wb1_fwd_hit_b ? wb_write_data_lane1 :
                            mem_write_data_base;
    assign alu_in2 = id_ex_alu_src ? id_ex_ext_imm : mem_write_data;
    assign fpu_in1 = fpu_in1_base;
    assign fpu_in2 = fpu_in2_base;

    wire [31:0] lane1_src1_ex0 = (ex_mem_reg_write && (ex_mem_rd != 5'd0) &&
                                  (ex_mem_rd == id_ex_rs1_lane1)) ? ex_mem_alu_result :
                                  id_ex_read_data1_lane1;
    wire [31:0] lane1_src1_ex1 = (ex_mem_reg_write_lane1 && (ex_mem_rd_lane1 != 5'd0) &&
                                  (ex_mem_rd_lane1 == id_ex_rs1_lane1)) ? ex_mem_alu_result_lane1 :
                                  lane1_src1_ex0;
    wire [31:0] lane1_src1_wb0 = (mem_wb_reg_write && (mem_wb_rd != 5'd0) &&
                                  (mem_wb_rd == id_ex_rs1_lane1)) ? wb_write_data :
                                  lane1_src1_ex1;
    wire [31:0] lane1_src2_ex0 = (ex_mem_reg_write && (ex_mem_rd != 5'd0) &&
                                  (ex_mem_rd == id_ex_rs2_lane1)) ? ex_mem_alu_result :
                                  id_ex_read_data2_lane1;
    wire [31:0] lane1_src2_ex1 = (ex_mem_reg_write_lane1 && (ex_mem_rd_lane1 != 5'd0) &&
                                  (ex_mem_rd_lane1 == id_ex_rs2_lane1)) ? ex_mem_alu_result_lane1 :
                                  lane1_src2_ex0;
    wire [31:0] lane1_src2_wb0 = (mem_wb_reg_write && (mem_wb_rd != 5'd0) &&
                                  (mem_wb_rd == id_ex_rs2_lane1)) ? wb_write_data :
                                  lane1_src2_ex1;

    assign alu_in1_lane1 = (mem_wb_reg_write_lane1 && (mem_wb_rd_lane1 != 5'd0) &&
                            (mem_wb_rd_lane1 == id_ex_rs1_lane1)) ? wb_write_data_lane1 :
                           lane1_src1_wb0;
    assign mem_write_data_lane1 = (mem_wb_reg_write_lane1 && (mem_wb_rd_lane1 != 5'd0) &&
                                   (mem_wb_rd_lane1 == id_ex_rs2_lane1)) ? wb_write_data_lane1 :
                                  lane1_src2_wb0;
    assign alu_in2_lane1 = id_ex_alu_src_lane1 ? id_ex_ext_imm_lane1 : mem_write_data_lane1;
    assign fpu_in1_lane1 = id_ex_read_f_data1_lane1;
    assign fpu_in2_lane1 = id_ex_read_f_data2_lane1;

    // =========================================================================
    // EXECUTE STAGE (EX)
    // =========================================================================
    execute EX (
        .clk(clk),
        .reset_n(reset_n),
        .stall_id_ex(stall_id_ex),
        .alu_in1(alu_in1),
        .alu_in2(alu_in2),
        .id_ex_alu_ctrl(id_ex_alu_ctrl),
        .id_ex_branch(id_ex_branch),
        .id_ex_instr(id_ex_instr),
        .id_ex_funct3(id_ex_funct3),
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
        .id_ex_fpu_en(id_ex_fpu_en),
        .id_ex_fpu_operation(id_ex_fpu_operation),
        .id_ex_read_f_data1(fpu_in1),
        .id_ex_read_f_data2(fpu_in2),
        .id_ex_f_to_x(id_ex_f_to_x),
        .id_ex_x_to_f(id_ex_x_to_f),
        .alu_result(alu_result),
        .branch_taken(branch_taken),
        .csr_write_data(csr_write_data_ex),
        .mf_alu_stall(mf_alu_stall),
        .fpu_result_out(fpu_result_out)
    );

    execute EX_LANE1 (
        .clk(clk),
        .reset_n(reset_n),
        .stall_id_ex(stall_id_ex),
        .alu_in1(alu_in1_lane1),
        .alu_in2(alu_in2_lane1),
        .id_ex_alu_ctrl(id_ex_alu_ctrl_lane1),
        .id_ex_branch(id_ex_branch_lane1),
        .id_ex_instr(id_ex_instr_lane1),
        .id_ex_funct3(id_ex_funct3_lane1),
        .id_ex_lui(id_ex_lui_lane1),
        .id_ex_auipc(id_ex_auipc_lane1),
        .id_ex_md_type(id_ex_md_type_lane1),
        .id_ex_md_operation(id_ex_md_operation_lane1),
        .id_ex_pc_in(id_ex_pc_in_lane1),
        .id_ex_ext_imm(id_ex_ext_imm_lane1),
        .id_ex_csr_op(id_ex_csr_op_lane1),
        .id_ex_csr_we(id_ex_csr_we_lane1),
        .csr_read_data(csr_read_data_fwd_lane1),
        .id_ex_rs1(id_ex_rs1_lane1),
        .id_ex_fpu_en(id_ex_fpu_en_lane1),
        .id_ex_fpu_operation(id_ex_fpu_operation_lane1),
        .id_ex_read_f_data1(fpu_in1_lane1),
        .id_ex_read_f_data2(fpu_in2_lane1),
        .id_ex_f_to_x(id_ex_f_to_x_lane1),
        .id_ex_x_to_f(id_ex_x_to_f_lane1),
        .alu_result(alu_result_lane1),
        .branch_taken(branch_taken_lane1),
        .csr_write_data(csr_write_data_ex_lane1),
        .mf_alu_stall(mf_alu_stall_lane1),
        .fpu_result_out(fpu_result_out_lane1)
    );

    // =========================================================================
    // EX/MEM REGISTER
    // =========================================================================
    ex_mem_register EX_MEM (
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
        .fpu_result(fpu_result_out),
        .id_ex_read_f_data2(fpu_in2),
        .id_ex_f_reg_write(id_ex_f_reg_write),
        .id_ex_f_mem_to_reg(id_ex_f_mem_to_reg),
        .id_ex_f_mem_write(id_ex_f_mem_write),
        .ex_mem_alu_result(ex_mem_alu_result),
        .ex_mem_rd(ex_mem_rd),
        .ex_mem_branch_target(ex_mem_branch_target),
        .ex_mem_pc_plus_4(ex_mem_pc_plus_4),
        .ex_mem_pc_in(ex_mem_pc_in),
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

    ex_mem_register EX_MEM_LANE1 (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_ex_mem), 
        .flush(flush_ex_mem),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .id_ex_rob_tag(id_ex_rob_tag_lane1),
        .id_ex_rob_valid(id_ex_rob_valid_lane1),
        .alu_result(alu_result_lane1),
        .id_ex_ext_imm(id_ex_ext_imm_lane1),
        .id_ex_rd(id_ex_rd_lane1),
        .id_ex_pc_plus_4(id_ex_pc_plus_4_lane1),
        .id_ex_pc_in(id_ex_pc_in_lane1),
        .id_ex_branch_target(id_ex_branch_target_lane1),
        .id_ex_mem_write(id_ex_mem_write_lane1),
        .id_ex_mem_read(id_ex_mem_read_lane1),
        .id_ex_mem_to_reg(id_ex_mem_to_reg_lane1),
        .id_ex_reg_write(id_ex_reg_write_lane1),
        .id_ex_branch(id_ex_branch_lane1),
        .branch_taken(branch_taken_lane1),
        .id_ex_jal(id_ex_jal_lane1),
        .id_ex_mem_unsigned(id_ex_mem_unsigned_lane1),
        .id_ex_mem_size(id_ex_mem_size_lane1),
        .id_ex_read_data2(id_ex_read_data2_lane1),
        .mem_write_data(mem_write_data_lane1),
        .id_ex_predict_taken(id_ex_predict_taken_lane1),
        .id_ex_btb_hit(id_ex_btb_hit_lane1),
        .id_ex_ecall(id_ex_ecall_lane1),
        .id_ex_ebreak(id_ex_ebreak_lane1),
        .id_ex_mret(id_ex_mret_lane1),
        .id_ex_csr_addr(id_ex_csr_addr_lane1),
        .id_ex_csr_op(id_ex_csr_op_lane1),
        .id_ex_csr_we(id_ex_csr_we_lane1),
        .csr_write_data_in(csr_write_data_ex_lane1),
        .id_ex_instr(id_ex_instr_lane1),
        .fpu_result(fpu_result_out_lane1),
        .id_ex_read_f_data2(fpu_in2_lane1),
        .id_ex_f_reg_write(id_ex_f_reg_write_lane1),
        .id_ex_f_mem_to_reg(id_ex_f_mem_to_reg_lane1),
        .id_ex_f_mem_write(id_ex_f_mem_write_lane1),
        .ex_mem_alu_result(ex_mem_alu_result_lane1),
        .ex_mem_rd(ex_mem_rd_lane1),
        .ex_mem_branch_target(ex_mem_branch_target_lane1),
        .ex_mem_pc_plus_4(ex_mem_pc_plus_4_lane1),
        .ex_mem_pc_in(ex_mem_pc_in_lane1),
        .ex_mem_mem_write(ex_mem_mem_write_lane1),
        .ex_mem_mem_read(ex_mem_mem_read_lane1),
        .ex_mem_mem_to_reg(ex_mem_mem_to_reg_lane1),
        .ex_mem_reg_write(ex_mem_reg_write_lane1),
        .ex_mem_branch(ex_mem_branch_lane1),
        .ex_mem_branch_taken(ex_mem_branch_taken_lane1),
        .ex_mem_jal(ex_mem_jal_lane1),
        .ex_mem_mem_unsigned(ex_mem_mem_unsigned_lane1),
        .ex_mem_mem_size(ex_mem_mem_size_lane1),
        .ex_mem_mem_write_data(ex_mem_mem_write_data_lane1),
        .ex_mem_predict_taken(ex_mem_predict_taken_lane1),
        .ex_mem_btb_hit(ex_mem_btb_hit_lane1),
        .ex_mem_ecall(ex_mem_ecall_lane1),
        .ex_mem_ebreak(ex_mem_ebreak_lane1),
        .ex_mem_mret(ex_mem_mret_lane1),
        .ex_mem_csr_addr(ex_mem_csr_addr_lane1),
        .ex_mem_csr_op(ex_mem_csr_op_lane1),
        .ex_mem_csr_we(ex_mem_csr_we_lane1),
        .ex_mem_csr_write_data(ex_mem_csr_write_data_lane1),
        .ex_mem_instr(ex_mem_instr_lane1),
        .ex_mem_fpu_result(ex_mem_fpu_result_lane1),
        .ex_mem_f_store_data(ex_mem_f_store_data_lane1),
        .ex_mem_f_reg_write(ex_mem_f_reg_write_lane1),
        .ex_mem_f_mem_to_reg(ex_mem_f_mem_to_reg_lane1),
        .ex_mem_f_mem_write(ex_mem_f_mem_write_lane1),
        .ex_mem_rob_tag(ex_mem_rob_tag_lane1),
        .ex_mem_rob_valid(ex_mem_rob_valid_lane1)
    );

    // =========================================================================
    // MEMORY ACCESS STAGE (MEM)
    // =========================================================================
    memory_access MEM (
        .ex_mem_alu_result(ex_mem_alu_result),
        .ex_mem_mem_write_data(final_mem_write_data),
        .ex_mem_mem_write(ex_mem_mem_write | ex_mem_f_mem_write),
        .ex_mem_mem_read(ex_mem_mem_read),
        .mem_read_data(mem_read_data),
        .dcache_read_req(legacy_lane0_dcache_read_req),
        .dcache_write_req(legacy_lane0_dcache_write_req),
        .dcache_addr(legacy_lane0_dcache_addr),
        .dcache_write_data(legacy_lane0_dcache_write_data),
        .dcache_read_data(dcache_read_data)
    );

    // =========================================================================
    // MEM/WB REGISTER
    // =========================================================================
    mem_wb_register MEM_WB (
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
        .ex_mem_fpu_result(ex_mem_fpu_result),
        .ex_mem_f_reg_write(ex_mem_f_reg_write),
        .ex_mem_f_mem_to_reg(ex_mem_f_mem_to_reg),
        .mem_wb_mem_read_data(mem_wb_mem_read_data),
        .mem_wb_pc_plus_4(mem_wb_pc_plus_4),
        .mem_wb_mem_to_reg(mem_wb_mem_to_reg),
        .mem_wb_reg_write(mem_wb_reg_write),
        .mem_wb_jal(mem_wb_jal),
        .mem_wb_alu_result(mem_wb_alu_result),
        .mem_wb_rd(mem_wb_rd),
        .mem_wb_ecall(mem_wb_ecall),
        .mem_wb_fpu_result(mem_wb_fpu_result),
        .mem_wb_f_reg_write(mem_wb_f_reg_write),
        .mem_wb_f_mem_to_reg(mem_wb_f_mem_to_reg),
        .mem_wb_rob_tag(mem_wb_rob_tag),
        .mem_wb_rob_valid(mem_wb_rob_valid)
    );

    mem_wb_register MEM_WB_LANE1 (
        .clk(clk),
        .reset_n(reset_n),
        .stall(stall_mem_wb), 
        .flush(flush_mem_wb),
        .riscv_start(riscv_start),
        .riscv_done(riscv_done),
        .ex_mem_rob_tag(ex_mem_rob_tag_lane1),
        .ex_mem_rob_valid(ex_mem_rob_valid_lane1),
        .mem_read_data(mem_read_data_lane1),
        .ex_mem_pc_plus_4(ex_mem_pc_plus_4_lane1),
        .ex_mem_mem_to_reg(ex_mem_mem_to_reg_lane1),
        .ex_mem_reg_write(ex_mem_reg_write_lane1),
        .ex_mem_jal(ex_mem_jal_lane1),
        .ex_mem_alu_result(ex_mem_alu_result_lane1),
        .ex_mem_rd(ex_mem_rd_lane1),
        .ex_mem_ecall(ex_mem_ecall_lane1),
        .ex_mem_fpu_result(ex_mem_fpu_result_lane1),
        .ex_mem_f_reg_write(ex_mem_f_reg_write_lane1),
        .ex_mem_f_mem_to_reg(ex_mem_f_mem_to_reg_lane1),
        .mem_wb_mem_read_data(mem_wb_mem_read_data_lane1),
        .mem_wb_pc_plus_4(mem_wb_pc_plus_4_lane1),
        .mem_wb_mem_to_reg(mem_wb_mem_to_reg_lane1),
        .mem_wb_reg_write(mem_wb_reg_write_lane1),
        .mem_wb_jal(mem_wb_jal_lane1),
        .mem_wb_alu_result(mem_wb_alu_result_lane1),
        .mem_wb_rd(mem_wb_rd_lane1),
        .mem_wb_ecall(mem_wb_ecall_lane1),
        .mem_wb_fpu_result(mem_wb_fpu_result_lane1),
        .mem_wb_f_reg_write(mem_wb_f_reg_write_lane1),
        .mem_wb_f_mem_to_reg(mem_wb_f_mem_to_reg_lane1),
        .mem_wb_rob_tag(mem_wb_rob_tag_lane1),
        .mem_wb_rob_valid(mem_wb_rob_valid_lane1)
    );

    // =========================================================================
    // WRITE BACK STAGE (WB)
    // =========================================================================
    write_back WB (
        .mem_wb_mem_read_data(mem_wb_mem_read_data),
        .mem_wb_alu_result(mem_wb_alu_result),
        .mem_wb_pc_plus_4(mem_wb_pc_plus_4),
        .mem_wb_mem_to_reg(mem_wb_mem_to_reg),
        .mem_wb_jal(mem_wb_jal),
        .mem_wb_write_data(wb_write_data)
    );
    assign wb_f_write_data = mem_wb_f_mem_to_reg ? mem_wb_mem_read_data : mem_wb_fpu_result;

    write_back WB_LANE1 (
        .mem_wb_mem_read_data(mem_wb_mem_read_data_lane1),
        .mem_wb_alu_result(mem_wb_alu_result_lane1),
        .mem_wb_pc_plus_4(mem_wb_pc_plus_4_lane1),
        .mem_wb_mem_to_reg(mem_wb_mem_to_reg_lane1),
        .mem_wb_jal(mem_wb_jal_lane1),
        .mem_wb_write_data(wb_write_data_lane1)
    );
    assign wb_f_write_data_lane1 = mem_wb_f_mem_to_reg_lane1 ? mem_wb_mem_read_data_lane1 : mem_wb_fpu_result_lane1;

    wire [OOO_N_ARCH_WB-1:0] ooo_arch_wb_valid = {
        ooo_enabled && dbg_halted && dbg_gpr_we,
        ooo_enabled && mem_wb_reg_write_lane1,
        ooo_enabled && mem_wb_reg_write
    };
    wire [OOO_N_ARCH_WB*5-1:0] ooo_arch_wb_rd = {
        dbg_reg_write_addr[4:0],
        mem_wb_rd_lane1,
        mem_wb_rd
    };
    wire [OOO_N_ARCH_WB*32-1:0] ooo_arch_wb_data = {
        dbg_reg_write_data,
        wb_write_data_lane1,
        wb_write_data
    };
    wire ooo_backend_flush = (flush_branch | flush_trap | flush_jal) && !ooo_backend_empty;

    tomasulo_backend_2way #(
        .ROB_DEPTH  (ROB_DEPTH),
        .PHYS_REGS  (64),
        .ARCH_REGS  (32),
        .IQ_ENTRIES (16),
        .DATA_W     (32),
        .N_DISPATCH (ROB_N_DISPATCH),
        .N_COMMIT   (ROB_N_COMMIT),
        .N_CDB      (3),
        .N_ARCH_WB  (OOO_N_ARCH_WB)
    ) OOO_BACKEND (
        .clk                 (clk),
        .rst_n               (reset_n),
        .flush_i             (ooo_backend_flush),
        .dispatch_valid_i    (ooo_dispatch_valid),
        .dispatch_ready_o    (ooo_dispatch_ready),
        .dispatch_pc_i       ({if_id_pc_in_lane1, if_id_pc_in}),
        .dispatch_rs1_i      ({rs1_lane1, rs1}),
        .dispatch_rs2_i      ({rs2_lane1, rs2}),
        .dispatch_rd_i       ({rd_lane1, rd}),
        .dispatch_rd_valid_i ({reg_write_lane1 && (rd_lane1 != 5'd0),
                               reg_write && (rd != 5'd0)}),
        .dispatch_alu_ctrl_i ({alu_ctrl_lane1, alu_ctrl}),
        .dispatch_alu_src_i  ({alu_src_lane1, alu_src}),
        .dispatch_lui_i      ({lui_lane1, lui}),
        .dispatch_auipc_i    ({auipc_lane1, auipc}),
        .dispatch_load_i     ({mem_read_lane1 && !f_mem_to_reg_lane1,
                               mem_read && !f_mem_to_reg}),
        .dispatch_store_i    ({mem_write_lane1 && !f_mem_write_lane1,
                               mem_write && !f_mem_write}),
        .dispatch_mem_size_i ({mem_size_lane1, mem_size}),
        .dispatch_mem_unsigned_i({mem_unsigned_lane1, mem_unsigned}),
        .dispatch_imm_i      ({ext_imm_lane1, ext_imm}),
        .arch_wb_valid_i     (ooo_arch_wb_valid),
        .arch_wb_rd_i        (ooo_arch_wb_rd),
        .arch_wb_data_i      (ooo_arch_wb_data),
        .dcache_read_req_o   (ooo_dcache_read_req),
        .dcache_write_req_o  (ooo_dcache_write_req),
        .dcache_addr_o       (ooo_dcache_addr),
        .dcache_write_data_o (ooo_dcache_write_data),
        .dcache_mem_size_o   (ooo_dcache_mem_size),
        .dcache_mem_unsigned_o(ooo_dcache_mem_unsigned),
        .dcache_read_data_i  (dcache_read_data),
        .dcache_hit_i        (dcache_hit),
        .dcache_stall_i      (dcache_stall),
        .commit_valid_o      (ooo_commit_valid),
        .commit_rd_o         (ooo_commit_rd),
        .commit_rd_valid_o   (ooo_commit_rd_valid),
        .commit_data_o       (ooo_commit_data),
        .rob_full_o          (ooo_rob_full),
        .rob_empty_o         (ooo_rob_empty),
        .backend_empty_o     (ooo_backend_empty)
    );

    // =========================================================================
    // REORDER BUFFER (SUPERSCALAR-READY RETIRE TRACKER)
    // =========================================================================
    wire [2:0] rob_dispatch_type_lane0 =
        (mem_read || f_mem_to_reg)      ? ROB_TYPE_LOAD   :
        (mem_write || f_mem_write)      ? ROB_TYPE_STORE  :
        (branch || jal || jalr)         ? ROB_TYPE_BRANCH :
        (fpu_en || f_reg_write || f_mem_write) ? ROB_TYPE_FP :
                                          ROB_TYPE_ALU;
    wire [2:0] rob_dispatch_type_lane1 =
        (mem_read_lane1 || f_mem_to_reg_lane1)      ? ROB_TYPE_LOAD   :
        (mem_write_lane1 || f_mem_write_lane1)      ? ROB_TYPE_STORE  :
        (branch_lane1 || jal_lane1 || jalr_lane1)   ? ROB_TYPE_BRANCH :
        (fpu_en_lane1 || f_reg_write_lane1 || f_mem_write_lane1) ?
                                                     ROB_TYPE_FP     :
                                                     ROB_TYPE_ALU;

    assign rob_dispatch_valid    = legacy_dispatch_valid;
    assign rob_dispatch_pc       = {if_id_pc_in_lane1, if_id_pc_in};
    assign rob_dispatch_rd       = {rd_lane1, rd};
    assign rob_dispatch_rd_valid = {((reg_write_lane1 && (rd_lane1 != 5'd0)) || f_reg_write_lane1),
                                    ((reg_write && (rd != 5'd0)) || f_reg_write)};
    assign rob_dispatch_type     = {rob_dispatch_type_lane1, rob_dispatch_type_lane0};

    assign rob_cdb_valid = {mem_wb_rob_valid_lane1 && !stall_mem_wb && !flush_mem_wb,
                            mem_wb_rob_valid && !stall_mem_wb && !flush_mem_wb};
    assign rob_cdb_tag   = {mem_wb_rob_tag_lane1, mem_wb_rob_tag};
    assign rob_cdb_data  = {wb_write_data_lane1,
                            mem_wb_f_reg_write ? wb_f_write_data : wb_write_data};
    assign rob_cdb_exc   = {mem_wb_ecall_lane1 ? 8'd11 : 8'd0,
                            mem_wb_ecall ? 8'd11 : 8'd0};

    reorder_buffer_hp #(
        .ROB_DEPTH  (ROB_DEPTH),
        .DATA_W     (32),
        .ARCH_REGS  (32),
        .PC_W       (32),
        .N_CDB      (ROB_N_CDB),
        .N_DISPATCH (ROB_N_DISPATCH),
        .N_COMMIT   (ROB_N_COMMIT),
        .N_BANKS    (2),
        .XCODE_W    (8)
    ) ROB (
        .clk                 (clk),
        .rst_n               (reset_n),
        .dispatch_valid_i    (rob_dispatch_valid),
        .dispatch_pc_i       (rob_dispatch_pc),
        .dispatch_rd_i       (rob_dispatch_rd),
        .dispatch_rd_valid_i (rob_dispatch_rd_valid),
        .dispatch_type_i     (rob_dispatch_type),
        .dispatch_ready_o    (rob_dispatch_ready),
        .dispatch_rob_tag_o  (rob_dispatch_tag),
        .cdb_valid_i         (rob_cdb_valid),
        .cdb_tag_i           (rob_cdb_tag),
        .cdb_data_i          (rob_cdb_data),
        .cdb_exc_i           (rob_cdb_exc),
        .commit_valid_o      (rob_commit_valid),
        .commit_rd_o         (rob_commit_rd),
        .commit_rd_valid_o   (rob_commit_rd_valid),
        .commit_data_o       (rob_commit_data),
        .commit_pc_o         (rob_commit_pc),
        .commit_type_o       (rob_commit_type),
        .commit_is_store_o   (rob_commit_is_store),
        .fwd_tag_a_i         ({ROB_TAG_W{1'b0}}),
        .fwd_valid_a_i       (1'b0),
        .fwd_data_a_o        (),
        .fwd_hit_a_o         (),
        .fwd_tag_b_i         ({ROB_TAG_W{1'b0}}),
        .fwd_valid_b_i       (1'b0),
        .fwd_data_b_o        (),
        .fwd_hit_b_o         (),
        .br_mispredict_i     (flush_branch | flush_trap),
        .br_correct_pc_i     (pc_out),
        .flush_o             (rob_flush),
        .flush_pc_o          (rob_flush_pc),
        .exc_valid_o         (rob_exc_valid),
        .exc_code_o          (rob_exc_code),
        .flush_early_o       (rob_flush_early),
        .flush_pc_early_o    (rob_flush_pc_early),
        .rob_full_o          (rob_full),
        .rob_empty_o         (rob_empty),
        .rob_count_o         (rob_count),
        .rob_free_slots_o    (rob_free_slots),
        .wakeup_vec_o        (rob_wakeup_vec)
    );

    // =========================================================================
    // PIPELINE CONTROL UNIT (HAZARD, STALL, FLUSH, DEBUG)
    // =========================================================================
    pipeline_control_unit PCU ( 
        .clk(clk),
        .reset_n(reset_n),
        .opcode(opcode), 
        .funct3(funct3), 
        .rs1(rs1), 
        .rs2(rs2), 
        .id_ex_mem_read(id_ex_mem_read), 
        .id_ex_jal(id_ex_jal_arb), 
        .id_ex_jalr(id_ex_jalr_arb), 
        .id_ex_rd(id_ex_rd), 
        .bpu_correct(bpu_correct), 
        .trap_enter(trap_enter), 
        .mret_exec(mret_exec_arb),
        .icache_stall(icache_stall_any), 
        .dcache_stall(dcache_stall),
        .mf_alu_stall(mf_alu_stall_any),

        .wfi_req(wfi_req_internal),
        .trap_interrupt(wake_interrupt),
        .is_sleeping(is_sleeping_internal),
        
        .dbg_halt_req(dbg_halt_req),
        .dbg_resume_req(dbg_resume_req),
        .dcsr_step(dcsr_out[2]),
        .dbg_halted(dbg_halted),
        
        .load_use_stall(load_use_stall_raw), 
        .flush_branch(flush_branch), 
        .flush_jal(flush_jal), 
        .flush_trap(flush_trap),
        .stall_IF(stall_IF),
        .stall_ID(stall_ID),
        .stall_EX(stall_EX),
        .stall_MEM(stall_MEM),
        .stall_WB(stall_WB)
    );

    // =========================================================================
    // BRANCH PREDICTION UNIT
    // =========================================================================
    wire bpu_stall = stall_mem_wb;

    branch_prediction_unit BPU (
        .clk(clk),
        .reset_n(reset_n),
        .pc_in(pc_in),
        .stall(bpu_stall),
        .ex_mem_pc_in(ex_mem_pc_in_arb),
        .ex_mem_branch(ex_mem_branch_arb),
        .ex_mem_branch_taken(ex_mem_branch_taken_arb),
        .ex_mem_predict_taken(ex_mem_predict_taken_arb),
        .ex_mem_btb_hit(ex_mem_btb_hit_arb),
        .ex_mem_branch_target(ex_mem_branch_target_arb),
        .bpu_correct(bpu_correct_selected),
        .predict_taken(predict_taken),
        .btb_hit(btb_hit),
        .actual_taken(actual_taken_selected),
        .predict_target(predict_target)
    );

    // =========================================================================
    // DONE LOGIC
    // =========================================================================
    always @(posedge clk) begin
        if (!reset_n) begin
            riscv_done <= 1'b0;
        end else if (riscv_start) begin
            if (ex_mem_ecall | ex_mem_ecall_lane1 | mem_wb_ecall | mem_wb_ecall_lane1) begin
                riscv_done <= 1'b1;
            end
        end
    end
    
endmodule
