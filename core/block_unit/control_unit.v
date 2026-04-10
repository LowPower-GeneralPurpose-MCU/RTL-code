//==================================================================================================
// File: control_unit.v
//==================================================================================================
module main_control_unit (
    input [6:0] opcode,
    input [6:0] funct7,
    input [2:0] funct3,
    input [4:0] rs2,
    output reg reg_write,
    output reg alu_src,
    output reg mem_write,
    output reg mem_read,
    output reg mem_to_reg,
    output reg branch,
    output reg jal,
    output reg jalr,
    output reg lui,
    output reg auipc,
    output reg mem_unsigned,
    output reg [1:0] alu_op,
    output reg [1:0] mem_size,
    output reg [2:0] md_operation,
    output reg fpu_en,
    output reg f_reg_write,
    output reg f_mem_to_reg,
    output reg f_mem_write,
    output reg f_to_x,
    output reg x_to_f,
    output reg [4:0] fpu_operation
);

    always @(*) begin 
        reg_write = 1'b0;
        alu_src = 1'b0;
        mem_write = 1'b0;
        mem_read = 1'b0;
        mem_to_reg = 1'b0;
        branch = 1'b0;
        jal = 1'b0;
        jalr = 1'b0;
        lui = 1'b0;
        auipc = 1'b0;
        alu_op = 2'b00;
        mem_size = 2'b00;
        mem_unsigned = 1'b0;
        md_operation = 3'b000;
        fpu_en = 1'b0;
        f_reg_write = 1'b0;
        f_mem_to_reg = 1'b0;
        f_mem_write = 1'b0;
        f_to_x = 1'b0;
        x_to_f = 1'b0;
        fpu_operation = 5'b00000;
        
        case (opcode)
            7'b0110011: begin
                reg_write = 1'b1;
                alu_op = 2'b10;
                if (funct7 == 7'b0000001) begin
                    case (funct3)
                        3'b000: md_operation = 3'b000;
                        3'b001: md_operation = 3'b001;
                        3'b010: md_operation = 3'b010;
                        3'b011: md_operation = 3'b011;
                        3'b100: md_operation = 3'b100;
                        3'b101: md_operation = 3'b101;
                        3'b110: md_operation = 3'b110;
                        3'b111: md_operation = 3'b111;
                        default: md_operation = 3'b000;
                    endcase
                end
            end
            
            7'b0010011: begin
                reg_write = 1'b1;
                alu_src = 1'b1;
                alu_op = 2'b10;
            end
            
            7'b0000011: begin
                alu_src = 1'b1;
                mem_read = 1'b1;
                mem_to_reg = 1'b1;
                reg_write = 1'b1;
                alu_op = 2'b00;
                case (funct3)
                    3'b000: begin
                        mem_size = 2'b00;
                        mem_unsigned = 1'b0;
                    end
                    3'b001: begin
                        mem_size = 2'b01;
                        mem_unsigned = 1'b0;
                    end
                    3'b010: begin
                        mem_size = 2'b10;
                        mem_unsigned = 1'b0;
                    end
                    3'b100: begin
                        mem_size = 2'b00;
                        mem_unsigned = 1'b1;
                    end
                    3'b101: begin
                        mem_size = 2'b01;
                        mem_unsigned = 1'b1;
                    end
                    default: begin
                        mem_size = 2'b10;
                        mem_unsigned = 1'b0;
                    end
                endcase
            end
            
            7'b0100011: begin
                alu_src = 1'b1;
                mem_write = 1'b1;
                alu_op = 2'b00;
                case (funct3)
                    3'b000: mem_size = 2'b00;
                    3'b001: mem_size = 2'b01;
                    3'b010: mem_size = 2'b10;
                    default: mem_size = 2'b10;
                endcase
            end
            
            7'b1100011: begin
                branch = 1'b1;
                alu_op = 2'b01;
            end
            
            7'b0110111: begin
                lui = 1'b1;
                reg_write = 1'b1;
            end
            
            7'b0010111: begin
                auipc = 1'b1;
                reg_write = 1'b1;
            end
            
            7'b1101111: begin
                jal = 1'b1;
                reg_write = 1'b1;
            end
            
            7'b1100111: begin
                jalr = 1'b1;
                reg_write = 1'b1;
                alu_src = 1'b1;
            end
            
            7'b1110011: begin
                if (funct3 != 3'b000) reg_write = 1'b1;
            end
            
            7'b0000111: begin
                alu_src = 1'b1;
                mem_read = 1'b1;
                f_mem_to_reg = 1'b1;
                f_reg_write = 1'b1;
                alu_op = 2'b00;
                mem_size = 2'b10;
            end
            
            7'b0100111: begin
                alu_src = 1'b1;
                mem_write = 1'b1;
                f_mem_write = 1'b1;
                alu_op = 2'b00;
                mem_size = 2'b10;
            end
            
            7'b1010011: begin
                fpu_en = 1'b1;
                case (funct7)
                    7'b0000000: begin
                        f_reg_write = 1'b1;
                        fpu_operation = 5'b00000;
                    end
                    7'b0000100: begin
                        f_reg_write = 1'b1;
                        fpu_operation = 5'b00001;
                    end
                    7'b0001000: begin
                        f_reg_write = 1'b1;
                        fpu_operation = 5'b00010;
                    end
                    7'b0001100: begin
                        f_reg_write = 1'b1;
                        fpu_operation = 5'b01000;
                    end
                    7'b0101100: begin
                        f_reg_write = 1'b1;
                        fpu_operation = 5'b01001;
                    end
                    7'b0010000: begin
                        f_reg_write = 1'b1;
                        case (funct3)
                            3'b000: fpu_operation = 5'b01100;
                            3'b001: fpu_operation = 5'b01101;
                            3'b010: fpu_operation = 5'b01110;
                            default: fpu_operation = 5'b01100;
                        endcase
                    end
                    7'b0010100: begin
                        f_reg_write = 1'b1;
                        case (funct3)
                            3'b000: fpu_operation = 5'b01010;
                            3'b001: fpu_operation = 5'b01011;
                            default: fpu_operation = 5'b01010;
                        endcase
                    end
                    7'b1010000: begin
                        f_to_x = 1'b1;
                        reg_write = 1'b1;
                        case (funct3)
                            3'b010: fpu_operation = 5'b00101;
                            3'b001: fpu_operation = 5'b00110;
                            3'b000: fpu_operation = 5'b00111;
                            default: fpu_operation = 5'b00101;
                        endcase
                    end
                    7'b1100000: begin
                        f_to_x = 1'b1;
                        reg_write = 1'b1;
                        if (rs2[0]) begin
                            fpu_operation = 5'b10010;
                        end else begin
                            fpu_operation = 5'b00011;
                        end
                    end
                    7'b1101000: begin
                        x_to_f = 1'b1;
                        f_reg_write = 1'b1;
                        if (rs2[0]) begin
                            fpu_operation = 5'b10011;
                        end else begin
                            fpu_operation = 5'b00100;
                        end
                    end
                    7'b1110000: begin
                        f_to_x = 1'b1;
                        reg_write = 1'b1;
                        case (funct3)
                            3'b000: fpu_operation = 5'b01111;
                            3'b001: fpu_operation = 5'b10001;
                            default: fpu_operation = 5'b01111;
                        endcase
                    end
                    7'b1111000: begin
                        x_to_f = 1'b1;
                        f_reg_write = 1'b1;
                        fpu_operation = 5'b10000;
                    end
                    default: fpu_en = 1'b0;
                endcase
            end
            
            7'b1000011: begin
                fpu_en = 1'b1;
                f_reg_write = 1'b1;
                fpu_operation = 5'b10100;
            end
            
            7'b1000111: begin
                fpu_en = 1'b1;
                f_reg_write = 1'b1;
                fpu_operation = 5'b10101;
            end
            
            7'b1001011: begin
                fpu_en = 1'b1;
                f_reg_write = 1'b1;
                fpu_operation = 5'b10110;
            end
            
            7'b1001111: begin
                fpu_en = 1'b1;
                f_reg_write = 1'b1;
                fpu_operation = 5'b10111;
            end
            
            default: begin
            end
        endcase
    end
    
endmodule


module alu_control_unit (
    input [1:0] alu_op,
    input [2:0] funct3,
    input [6:0] funct7,
    input [6:0] opcode,
    output reg [3:0] alu_ctrl
);

    always @(*) begin
        case (alu_op)
            2'b00: alu_ctrl = 4'b0010;
            2'b01: alu_ctrl = 4'b0110;
            2'b10: begin
                if (opcode == 7'b0010011) begin
                    case (funct3)
                        3'b000: alu_ctrl = 4'b0010;
                        3'b010: alu_ctrl = 4'b0111;
                        3'b011: alu_ctrl = 4'b1010;
                        3'b100: alu_ctrl = 4'b0100;
                        3'b110: alu_ctrl = 4'b0001;
                        3'b111: alu_ctrl = 4'b0000;
                        3'b001: alu_ctrl = 4'b1000;
                        3'b101: begin
                            if (funct7[5]) begin
                                alu_ctrl = 4'b1011;
                            end else begin
                                alu_ctrl = 4'b1001;
                            end
                        end
                        default: alu_ctrl = 4'b0010;
                    endcase
                end else begin
                    case (funct3)
                        3'b000: begin
                            if (funct7[5]) begin
                                alu_ctrl = 4'b0110;
                            end else begin
                                alu_ctrl = 4'b0010;
                            end
                        end
                        3'b001: alu_ctrl = 4'b1000;
                        3'b010: alu_ctrl = 4'b0111;
                        3'b011: alu_ctrl = 4'b1010;
                        3'b100: alu_ctrl = 4'b0100;
                        3'b101: begin
                            if (funct7[5]) begin
                                alu_ctrl = 4'b1011;
                            end else begin
                                alu_ctrl = 4'b1001;
                            end
                        end
                        3'b110: alu_ctrl = 4'b0001;
                        3'b111: alu_ctrl = 4'b0000;
                        default: alu_ctrl = 4'b0010;
                    endcase
                end
            end
            default: alu_ctrl = 4'b0010;
        endcase
    end
    
endmodule