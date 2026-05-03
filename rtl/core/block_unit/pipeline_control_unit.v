//==================================================================================================
// File: pipeline_control_unit.v
//==================================================================================================
module pipeline_control_unit (
    input [6:0] opcode,
    input [2:0] funct3,
    input [4:0] rs1,
    input [4:0] rs2,
    input id_ex_mem_read, 
    input id_ex_jal,
    input id_ex_jalr,
    input [4:0] id_ex_rd,
    input bpu_correct,
    input trap_enter,
    input mret_exec,
    output reg load_use_stall,
    output reg flush_branch,
    output reg flush_jal,
    output reg flush_trap
);

    reg load_use_hazard;

    always @(*) begin
        load_use_stall = 1'b0;
        flush_branch = 1'b0;
        flush_jal = 1'b0;
        flush_trap = 1'b0;
        load_use_hazard = 1'b0;

        if (id_ex_mem_read && (id_ex_rd != 5'd0)) begin
            case (opcode)
                7'b0110011, 7'b0010011, 7'b0000011, 7'b0100011, 7'b1100011, 7'b1100111: begin
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) begin
                        load_use_hazard = 1'b1;
                    end
                end
                7'b1010011, 7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) begin
                        load_use_hazard = 1'b1;
                    end
                end
                7'b0000111: begin
                    if (id_ex_rd == rs1) begin
                        load_use_hazard = 1'b1;
                    end
                end
                7'b0100111: begin
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) begin
                        load_use_hazard = 1'b1;
                    end
                end
            endcase
        end

        flush_branch = !bpu_correct;
        flush_jal = id_ex_jal || id_ex_jalr;
        flush_trap = trap_enter || mret_exec;

        load_use_stall = load_use_hazard && !flush_branch && !flush_jal;
    end
    
endmodule