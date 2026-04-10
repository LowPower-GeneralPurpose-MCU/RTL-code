//==================================================================================================
// File: pipeline_register.v
//==================================================================================================

module if_id_register (
    input clk, reset_n,
    input stall, input flush,
    input riscv_start, input riscv_done,
    input [31:0] instr, pc_plus_4, pc_in,
    input predict_taken, btb_hit,
    output reg [31:0] if_id_instr, if_id_pc_plus_4, if_id_pc_in,
    output reg if_id_predict_taken, if_id_btb_hit
);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            if_id_pc_in <= 32'd0;
            if_id_instr <= 32'd0;
            if_id_pc_plus_4 <= 32'd0;
            if_id_predict_taken <= 1'b0;
            if_id_btb_hit <= 1'b0;
        end else if (riscv_start && !riscv_done) begin            
            if (stall) begin
                // Đóng băng, không thay đổi giá trị
            end else if (flush) begin
                if_id_instr <= 32'h00000013; // NOP (addi x0, x0, 0)
                if_id_predict_taken <= 1'b0;
                if_id_btb_hit <= 1'b0;
            end else begin
                if_id_pc_in <= pc_in;
                if_id_instr <= instr;
                if_id_pc_plus_4 <= pc_plus_4;
                if_id_predict_taken <= predict_taken;
                if_id_btb_hit <= btb_hit;
            end
        end
    end
endmodule


module id_ex_register (
    input clk, reset_n,
    input stall, input flush,
    input riscv_start, input riscv_done,
    input [31:0] if_id_pc_plus_4, if_id_pc_in,
    input [2:0] funct3,
    input [31:0] read_data1, read_data2, ext_imm,
    input [4:0] rs1, rs2, rd,
    input reg_write, alu_src, mem_write, mem_read, mem_to_reg,
    input branch, jal, jalr, lui, auipc, mem_unsigned,
    input [1:0] mem_size,
    input [3:0] alu_ctrl,
    input [31:0] branch_target, jal_target,
    input if_id_predict_taken, if_id_btb_hit,
    input ecall, ebreak, mret,
    input [11:0] csr_addr,
    input [1:0] csr_op,
    input csr_we, md_type,
    input [2:0] md_operation,
    input [31:0] if_id_instr,
    input fpu_en, f_reg_write, f_mem_to_reg, f_mem_write, f_to_x, x_to_f,
    input [4:0] fpu_operation,
    input [31:0] read_f_data1, read_f_data2,
  
    // Outputs giữ nguyên tên như cũ
    output reg [31:0] id_ex_pc_plus_4, id_ex_pc_in,
    output reg [2:0] id_ex_funct3,
    output reg [31:0] id_ex_read_data1, id_ex_read_data2, id_ex_ext_imm,
    output reg [4:0] id_ex_rs1, id_ex_rs2, id_ex_rd,
    output reg id_ex_reg_write, id_ex_alu_src, id_ex_mem_write, id_ex_mem_read, id_ex_mem_to_reg,
    output reg id_ex_branch, id_ex_jal, id_ex_jalr, id_ex_lui, id_ex_auipc, id_ex_mem_unsigned,
    output reg [1:0] id_ex_mem_size,
    output reg [3:0] id_ex_alu_ctrl,
    output reg [31:0] id_ex_branch_target, id_ex_jal_target,
    output reg id_ex_predict_taken, id_ex_btb_hit, id_ex_ecall, id_ex_ebreak, id_ex_mret,
    output reg [11:0] id_ex_csr_addr,
    output reg [1:0] id_ex_csr_op,
    output reg id_ex_csr_we, id_ex_md_type,
    output reg [2:0] id_ex_md_operation,
    output reg [31:0] id_ex_instr,
    output reg id_ex_fpu_en, id_ex_f_reg_write, id_ex_f_mem_to_reg, id_ex_f_mem_write,
    output reg id_ex_f_to_x, id_ex_x_to_f,
    output reg [4:0] id_ex_fpu_operation,
    output reg [31:0] id_ex_read_f_data1, id_ex_read_f_data2
);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            id_ex_instr <= 32'd0;
            id_ex_reg_write <= 1'b0; id_ex_mem_write <= 1'b0; id_ex_mem_read <= 1'b0;
            id_ex_branch <= 1'b0; id_ex_jal <= 1'b0; id_ex_jalr <= 1'b0; id_ex_csr_we <= 1'b0;
            id_ex_f_reg_write <= 1'b0; id_ex_f_mem_write <= 1'b0;
            id_ex_ecall <= 1'b0; id_ex_mret <= 1'b0; id_ex_ebreak <= 1'b0;
            // (Reset các biến khác nếu cần, nhưng các tín hiệu điều khiển trên là quan trọng nhất)
        end else if (riscv_start && !riscv_done) begin
            if (stall) begin
                // Đóng băng
            end else if (flush) begin
                // Bơm NOP (Chỉ cần tắt các cờ thay đổi trạng thái hệ thống)
                id_ex_instr <= 32'h00000013;
                id_ex_reg_write <= 1'b0;
                id_ex_mem_write <= 1'b0;
                id_ex_mem_read <= 1'b0;
                id_ex_branch <= 1'b0;
                id_ex_jal <= 1'b0;
                id_ex_jalr <= 1'b0;
                id_ex_ecall <= 1'b0; 
                id_ex_ebreak <= 1'b0; 
                id_ex_mret <= 1'b0; 
                id_ex_csr_we <= 1'b0;
                id_ex_f_reg_write <= 1'b0;
                id_ex_f_mem_write <= 1'b0;
                id_ex_f_to_x <= 1'b0;
                id_ex_x_to_f <= 1'b0;
            end else begin
                // Cập nhật bình thường
                id_ex_pc_plus_4 <= if_id_pc_plus_4; id_ex_pc_in <= if_id_pc_in;
                id_ex_funct3 <= funct3;
                id_ex_read_data1 <= read_data1; id_ex_read_data2 <= read_data2;
                id_ex_ext_imm <= ext_imm; id_ex_branch_target <= branch_target;
                id_ex_jal_target <= jal_target;
                id_ex_rs1 <= rs1; id_ex_rs2 <= rs2; id_ex_rd <= rd;
                id_ex_alu_src <= alu_src; id_ex_mem_write <= mem_write;
                id_ex_mem_read <= mem_read; id_ex_mem_to_reg <= mem_to_reg;
                id_ex_reg_write <= reg_write; id_ex_branch <= branch;
                id_ex_jal <= jal; id_ex_jalr <= jalr; id_ex_lui <= lui;
                id_ex_auipc <= auipc; id_ex_mem_unsigned <= mem_unsigned;
                id_ex_mem_size <= mem_size; id_ex_alu_ctrl <= alu_ctrl;
                id_ex_predict_taken <= if_id_predict_taken; id_ex_btb_hit <= if_id_btb_hit;
                id_ex_instr <= if_id_instr; id_ex_ecall <= ecall;
                id_ex_ebreak <= ebreak; id_ex_mret <= mret;
                id_ex_csr_addr <= csr_addr; id_ex_csr_op <= csr_op; id_ex_csr_we <= csr_we;
                id_ex_md_type <= md_type; id_ex_md_operation <= md_operation;
                id_ex_fpu_en <= fpu_en; id_ex_f_reg_write <= f_reg_write;
                id_ex_f_mem_to_reg <= f_mem_to_reg; id_ex_f_mem_write <= f_mem_write;
                id_ex_f_to_x <= f_to_x; id_ex_x_to_f <= x_to_f;
                id_ex_fpu_operation <= fpu_operation;
                id_ex_read_f_data1 <= read_f_data1; id_ex_read_f_data2 <= read_f_data2;
            end
        end
    end
endmodule


module ex_mem_register (
    input clk, reset_n,
    input stall, input flush,
    input riscv_start, input riscv_done,
    input [31:0] alu_result, id_ex_ext_imm,
    input [4:0] id_ex_rd,
    input [31:0] id_ex_pc_plus_4, id_ex_pc_in, id_ex_branch_target,
    input id_ex_mem_write, id_ex_mem_read, id_ex_mem_to_reg, id_ex_reg_write,
    input id_ex_branch, branch_taken, id_ex_jal, id_ex_mem_unsigned,
    input [1:0] id_ex_mem_size,
    input [31:0] id_ex_read_data2, mem_write_data,
    input id_ex_predict_taken, id_ex_btb_hit, id_ex_ecall, id_ex_ebreak, id_ex_mret,
    input [11:0] id_ex_csr_addr, input [1:0] id_ex_csr_op, input id_ex_csr_we,
    input [31:0] csr_write_data_in, id_ex_instr, fpu_result, id_ex_read_f_data2,
    input id_ex_f_reg_write, id_ex_f_mem_to_reg, id_ex_f_mem_write,
    
    // Outputs
    output reg [31:0] ex_mem_alu_result, ex_mem_branch_target, ex_mem_pc_plus_4, ex_mem_pc_in,
    output reg [4:0] ex_mem_rd,
    output reg ex_mem_mem_write, ex_mem_mem_read, ex_mem_mem_to_reg, ex_mem_reg_write,
    output reg ex_mem_branch, ex_mem_branch_taken, ex_mem_jal, ex_mem_mem_unsigned,
    output reg [1:0] ex_mem_mem_size,
    output reg [31:0] ex_mem_mem_write_data,
    output reg ex_mem_predict_taken, ex_mem_btb_hit, ex_mem_ecall, ex_mem_ebreak, ex_mem_mret,
    output reg [11:0] ex_mem_csr_addr,
    output reg [1:0] ex_mem_csr_op,
    output reg ex_mem_csr_we,
    output reg [31:0] ex_mem_csr_write_data, ex_mem_instr, ex_mem_fpu_result, ex_mem_f_store_data,
    output reg ex_mem_f_reg_write, ex_mem_f_mem_to_reg, ex_mem_f_mem_write
);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ex_mem_instr <= 32'd0;
            ex_mem_reg_write <= 1'b0; ex_mem_mem_write <= 1'b0; ex_mem_mem_read <= 1'b0;
            ex_mem_branch <= 1'b0; ex_mem_jal <= 1'b0; ex_mem_csr_we <= 1'b0;
            ex_mem_ecall <= 1'b0; ex_mem_ebreak <= 1'b0; ex_mem_mret <= 1'b0;
            ex_mem_f_reg_write <= 1'b0; ex_mem_f_mem_write <= 1'b0;
        end else if (riscv_start && !riscv_done) begin
            if (stall) begin
                // Đóng băng
            end else if (flush) begin
                ex_mem_instr <= 32'h00000013;
                ex_mem_reg_write <= 1'b0;
                ex_mem_mem_write <= 1'b0;
                ex_mem_mem_read <= 1'b0;
                ex_mem_branch <= 1'b0;
                ex_mem_jal <= 1'b0;
                ex_mem_ecall <= 1'b0;
                ex_mem_ebreak <= 1'b0; 
                ex_mem_mret <= 1'b0; 
                ex_mem_csr_we <= 1'b0;
                ex_mem_f_reg_write <= 1'b0;
                ex_mem_f_mem_write <= 1'b0;
            end else begin
                ex_mem_alu_result <= alu_result; ex_mem_rd <= id_ex_rd;
                ex_mem_branch_target <= id_ex_branch_target; ex_mem_pc_plus_4 <= id_ex_pc_plus_4;
                ex_mem_pc_in <= id_ex_pc_in; ex_mem_branch <= id_ex_branch;
                ex_mem_branch_taken <= branch_taken; ex_mem_jal <= id_ex_jal;
                ex_mem_mem_unsigned <= id_ex_mem_unsigned; ex_mem_mem_write <= id_ex_mem_write;
                ex_mem_mem_read <= id_ex_mem_read; ex_mem_mem_to_reg <= id_ex_mem_to_reg;
                ex_mem_reg_write <= id_ex_reg_write; ex_mem_mem_size <= id_ex_mem_size;
                ex_mem_mem_write_data <= mem_write_data; ex_mem_predict_taken <= id_ex_predict_taken;
                ex_mem_btb_hit <= id_ex_btb_hit; ex_mem_instr <= id_ex_instr;
                ex_mem_ecall <= id_ex_ecall; ex_mem_ebreak <= id_ex_ebreak; ex_mem_mret <= id_ex_mret;
                ex_mem_csr_addr <= id_ex_csr_addr; ex_mem_csr_op <= id_ex_csr_op; 
                ex_mem_csr_we <= id_ex_csr_we; ex_mem_csr_write_data <= csr_write_data_in;
                ex_mem_fpu_result <= fpu_result; ex_mem_f_store_data <= id_ex_read_f_data2;
                ex_mem_f_reg_write <= id_ex_f_reg_write; ex_mem_f_mem_to_reg <= id_ex_f_mem_to_reg;
                ex_mem_f_mem_write <= id_ex_f_mem_write;
            end
        end
    end
endmodule


module mem_wb_register (
    input clk, reset_n,
    input stall, input flush,
    input riscv_start, riscv_done,
    input [31:0] mem_read_data, ex_mem_pc_plus_4,
    input ex_mem_mem_to_reg, ex_mem_reg_write, ex_mem_jal,
    input [31:0] ex_mem_alu_result,
    input [4:0] ex_mem_rd,
    input ex_mem_ecall,
    input [31:0] ex_mem_fpu_result,
    input ex_mem_f_reg_write, ex_mem_f_mem_to_reg,
    
    // Outputs
    output reg [31:0] mem_wb_mem_read_data, mem_wb_pc_plus_4, mem_wb_alu_result, mem_wb_fpu_result,
    output reg mem_wb_mem_to_reg, mem_wb_reg_write, mem_wb_jal, mem_wb_ecall, mem_wb_f_reg_write, mem_wb_f_mem_to_reg,
    output reg [4:0] mem_wb_rd
);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            mem_wb_reg_write <= 1'b0;
            mem_wb_f_reg_write <= 1'b0;
            mem_wb_ecall <= 1'b0;
        end else if (riscv_start && !riscv_done) begin
            if (stall) begin
                // Đóng băng
            end else if (flush) begin
                mem_wb_reg_write <= 1'b0;
                mem_wb_f_reg_write <= 1'b0;
                mem_wb_ecall <= 1'b0;
            end else begin
                mem_wb_mem_read_data <= mem_read_data; mem_wb_pc_plus_4 <= ex_mem_pc_plus_4;
                mem_wb_alu_result <= ex_mem_alu_result; mem_wb_rd <= ex_mem_rd;
                mem_wb_mem_to_reg <= ex_mem_mem_to_reg; mem_wb_reg_write <= ex_mem_reg_write;
                mem_wb_jal <= ex_mem_jal; mem_wb_ecall <= ex_mem_ecall;
                mem_wb_fpu_result <= ex_mem_fpu_result; mem_wb_f_reg_write <= ex_mem_f_reg_write;
                mem_wb_f_mem_to_reg <= ex_mem_f_mem_to_reg;
            end
        end
    end    
endmodule