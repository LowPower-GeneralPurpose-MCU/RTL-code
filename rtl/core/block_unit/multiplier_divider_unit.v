//==================================================================================================
// File: multiplier_divider_unit.v
//==================================================================================================
module multiplier (
    input clk,
    input reset_n,
    input md_type,
    input [31:0] alu_in1,
    input [31:0] alu_in2,
    input [2:0] md_operation,
    output [31:0] md_result,
    output md_alu_stall,
    output md_alu_done
);

    localparam BITS_PER_CYCLE = 4;
    
    wire mul_inst_w = md_type & (~md_operation[2]);
    wire is_mul = mul_inst_w && (md_operation == 3'b000);
    wire is_mulh = mul_inst_w && (md_operation == 3'b001);
    wire is_mulhsu = mul_inst_w && (md_operation == 3'b010);
    wire is_mulhu = mul_inst_w && (md_operation == 3'b011);
    
    localparam STATE_IDLE = 2'b00;
    localparam STATE_BUSY = 2'b01;
    
    reg [1:0] state;
    reg [1:0] state_next;
    
    reg [31:0] a_val;
    reg [31:0] a_val_next;
    reg [31:0] b_val;
    reg [31:0] b_val_next;
    reg a_signed_flag;
    reg a_signed_flag_next;
    reg b_signed_flag;
    reg b_signed_flag_next;
    reg [2:0] opcode_reg;
    reg [2:0] opcode_reg_next;
    
    reg [63:0] multiplicand;
    reg [63:0] multiplicand_next;
    reg [31:0] multiplier;
    reg [31:0] multiplier_next;
    reg [63:0] product;
    reg [63:0] product_next;
    reg [5:0] counter;
    reg [5:0] counter_next;
    
    reg [31:0] md_result_reg;
    reg [31:0] md_result_next;
    reg md_alu_stall_next;
    reg md_alu_done_next;
    
    wire start_mul = (state == STATE_IDLE) && mul_inst_w;
    
    reg [63:0] prod_temp;
    reg [63:0] mcand_temp;
    reg [31:0] mplier_temp;
    reg [5:0] count_temp;
    
    reg signed [63:0] signed_product;
    reg result_sign;
        
    always @* begin
        state_next = state;
        a_val_next = a_val;
        b_val_next = b_val;
        a_signed_flag_next = a_signed_flag;
        b_signed_flag_next = b_signed_flag;
        multiplicand_next = multiplicand;
        multiplier_next = multiplier;
        product_next = product;
        counter_next = counter;
        opcode_reg_next = opcode_reg;
        
        md_result_next = md_result_reg;
        md_alu_stall_next = 1'b0;
        md_alu_done_next = 1'b0;
        
        prod_temp = 64'b0;
        mcand_temp = 64'b0;
        mplier_temp = 32'b0;
        count_temp = 6'b0;
        signed_product = 64'b0;
        result_sign = 1'b0;
        
        case (state)
            STATE_IDLE: begin
                if (start_mul) begin
                    opcode_reg_next = md_operation;
                    
                    case (md_operation)
                        3'b000: begin
                            a_val_next = alu_in1;
                            b_val_next = alu_in2;
                            a_signed_flag_next = 1'b0;
                            b_signed_flag_next = 1'b0;
                        end
                        3'b001: begin
                            a_val_next = alu_in1;
                            b_val_next = alu_in2;
                            a_signed_flag_next = 1'b1;
                            b_signed_flag_next = 1'b1;
                        end
                        3'b010: begin
                            a_val_next = alu_in1;
                            b_val_next = alu_in2;
                            a_signed_flag_next = 1'b1;
                            b_signed_flag_next = 1'b0;
                        end
                        3'b011: begin
                            a_val_next = alu_in1;
                            b_val_next = alu_in2;
                            a_signed_flag_next = 1'b0;
                            b_signed_flag_next = 1'b0;
                        end
                        default: begin
                            a_val_next = alu_in1;
                            b_val_next = alu_in2;
                            a_signed_flag_next = 1'b0;
                            b_signed_flag_next = 1'b0;
                        end
                    endcase
                    
                    if (a_signed_flag_next && a_val_next[31]) begin
                        multiplicand_next = {32'b0, -a_val_next};
                    end else begin
                        multiplicand_next = {32'b0, a_val_next};
                    end
                    
                    if (b_signed_flag_next && b_val_next[31]) begin
                        multiplier_next = -b_val_next;
                    end else begin
                        multiplier_next = b_val_next;
                    end
                    
                    product_next = 64'b0;
                    counter_next = 6'b0;
                    
                    md_alu_stall_next = 1'b1;
                    state_next = STATE_BUSY;
                end
            end
            
            STATE_BUSY: begin
                md_alu_stall_next = 1'b1;
                
                prod_temp = product;
                mcand_temp = multiplicand;
                mplier_temp = multiplier;
                count_temp = counter;
                
                begin : PROCESS_BITS
                    integer i;
                    for (i = 0; i < BITS_PER_CYCLE; i = i + 1) begin
                        if (count_temp < 32) begin
                            if (mplier_temp[0]) begin
                                prod_temp = prod_temp + mcand_temp;
                            end
                            
                            mplier_temp = mplier_temp >> 1;
                            mcand_temp = mcand_temp << 1;
                            count_temp = count_temp + 1;
                        end
                    end
                end
                
                product_next = prod_temp;
                multiplier_next = mplier_temp;
                multiplicand_next = mcand_temp;
                counter_next = count_temp;
                
                if (count_temp >= 32) begin
                    md_alu_stall_next = 1'b0;
                    md_alu_done_next = 1'b1;
                    state_next = STATE_IDLE;
                    
                    result_sign = 1'b0;
                    if (opcode_reg == 3'b001) begin
                        result_sign = (a_val[31] & a_signed_flag) ^ (b_val[31] & b_signed_flag);
                    end else if (opcode_reg == 3'b010) begin
                        result_sign = a_val[31] & a_signed_flag;
                    end
                    
                    if (result_sign) begin
                        signed_product = -product_next;
                    end else begin
                        signed_product = product_next;
                    end
                    
                    case (opcode_reg)
                        3'b000: begin
                            md_result_next = product_next[31:0];
                        end
                        3'b001: begin
                            md_result_next = signed_product[63:32];
                        end
                        3'b010: begin
                            md_result_next = signed_product[63:32];
                        end
                        3'b011: begin
                            md_result_next = product_next[63:32];
                        end
                        default: begin
                            md_result_next = 32'b0;
                        end
                    endcase
                end
            end
            
            default: begin
                state_next = STATE_IDLE;
            end
        endcase
    end
    
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= STATE_IDLE;
            a_val <= 32'b0;
            b_val <= 32'b0;
            a_signed_flag <= 1'b0;
            b_signed_flag <= 1'b0;
            multiplicand <= 64'b0;
            multiplier <= 32'b0;
            product <= 64'b0;
            counter <= 6'b0;
            opcode_reg <= 3'b0;
            md_result_reg <= 32'b0;
        end else begin
            state <= state_next;
            a_val <= a_val_next;
            b_val <= b_val_next;
            a_signed_flag <= a_signed_flag_next;
            b_signed_flag <= b_signed_flag_next;
            multiplicand <= multiplicand_next;
            multiplier <= multiplier_next;
            product <= product_next;
            counter <= counter_next;
            opcode_reg <= opcode_reg_next;
            md_result_reg <= md_result_next;
        end
    end

    assign md_alu_done = md_alu_done_next;
    assign md_alu_stall = md_alu_stall_next;
    assign md_result = md_result_next;

endmodule


module divider (
    input clk,
    input reset_n,
    input md_type,
    input [31:0] alu_in1,
    input [31:0] alu_in2,
    input [2:0] md_operation,
    output [31:0] md_result,
    output md_alu_stall,
    output md_alu_done
);

    localparam BITS_PER_CYCLE = 2;

    wire is_div_op = md_operation[2];
    wire is_div = is_div_op && (md_operation[1:0] == 2'b00);
    wire is_divu = is_div_op && (md_operation[1:0] == 2'b01);
    wire is_rem = is_div_op && (md_operation[1:0] == 2'b10);
    wire is_remu = is_div_op && (md_operation[1:0] == 2'b11);

    wire signed_op = is_div | is_rem;
    wire div_inst = is_div | is_divu;

    localparam STATE_IDLE = 2'b00;
    localparam STATE_BUSY = 2'b01;

    reg [1:0] state;

    reg [31:0] dividend_orig;
    reg [31:0] dividend_abs;
    reg [31:0] divisor_abs;
    reg [31:0] quotient;
    reg [31:0] remainder;
    reg [31:0] mask;
    reg invert_res;
    reg div_inst_q;
    
    reg [31:0] rem_tmp;
    reg [31:0] quo_tmp;
    reg [31:0] mask_tmp;

    integer step;

    reg [31:0] md_result_reg;
    reg [31:0] md_result_next;
    reg md_alu_stall_next;
    reg md_alu_done_next;

    wire start_div = (state == STATE_IDLE) && md_type && is_div_op;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= STATE_IDLE;
            md_result_reg <= 32'd0;
            dividend_orig <= 32'd0;
            dividend_abs <= 32'd0;
            divisor_abs <= 32'd0;
            quotient <= 32'd0;
            remainder <= 32'd0;
            mask <= 32'd0;
            invert_res <= 1'b0;
            div_inst_q <= 1'b0;
        end else begin
            md_result_reg <= md_result_next;

            case (state)
                STATE_IDLE: begin
                    if (start_div) begin
                        dividend_orig <= alu_in1;

                        if (signed_op) begin
                            if (alu_in1[31]) begin
                                dividend_abs <= -alu_in1;
                            end else begin
                                dividend_abs <= alu_in1;
                            end
                            
                            if (alu_in2[31]) begin
                                divisor_abs <= -alu_in2;
                            end else begin
                                divisor_abs <= alu_in2;
                            end
                        end else begin
                            dividend_abs <= alu_in1;
                            divisor_abs <= alu_in2;
                        end

                        quotient <= 32'd0;
                        remainder <= 32'd0;
                        mask <= 32'h8000_0000;

                        invert_res <= (is_div && (alu_in1[31] ^ alu_in2[31]) && (alu_in2 != 32'd0)) ||
                                      (is_rem && alu_in1[31]);
                        
                        div_inst_q <= div_inst;

                        state <= STATE_BUSY;
                    end
                end

                STATE_BUSY: begin
                    remainder <= rem_tmp;
                    quotient <= quo_tmp;
                    mask <= mask_tmp;

                    if (mask_tmp == 32'd0) begin
                        state <= STATE_IDLE;
                    end
                end

                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

    always @(*) begin
        md_result_next = md_result_reg;
        md_alu_stall_next = 1'b0;
        md_alu_done_next = 1'b0;

        rem_tmp = remainder;
        quo_tmp = quotient;
        mask_tmp = mask;

        case (state)
            STATE_IDLE: begin
                if (start_div) begin
                    md_alu_stall_next = 1'b1;
                    md_alu_done_next = 1'b0;
                end
            end
            
            STATE_BUSY: begin
                for (step = 0; step < BITS_PER_CYCLE; step = step + 1) begin
                    if (mask_tmp != 32'd0) begin
                        if (dividend_abs & mask_tmp) begin
                            rem_tmp = (rem_tmp << 1) | 1'b1;
                        end else begin
                            rem_tmp = (rem_tmp << 1);
                        end

                        if ((rem_tmp >= divisor_abs) && (divisor_abs != 32'd0)) begin
                            rem_tmp = rem_tmp - divisor_abs;
                            quo_tmp = quo_tmp | mask_tmp;
                        end

                        mask_tmp = mask_tmp >> 1;
                    end
                end

                md_alu_stall_next = 1'b1;
                md_alu_done_next = 1'b0;

                if (mask_tmp == 32'd0) begin
                    md_alu_stall_next = 1'b0;
                    md_alu_done_next = 1'b1;

                    if (divisor_abs == 32'd0) begin
                        if (div_inst_q) begin
                            md_result_next = 32'hFFFF_FFFF;
                        end else begin
                            md_result_next = dividend_orig;
                        end
                    end else begin
                        if (div_inst_q) begin
                            if (invert_res) begin
                                md_result_next = -quo_tmp;
                            end else begin
                                md_result_next = quo_tmp;
                            end
                        end else begin
                            if (invert_res) begin
                                md_result_next = -rem_tmp;
                            end else begin
                                md_result_next = rem_tmp;
                            end
                        end
                    end
                end
            end
        endcase
    end
    
    assign md_result = md_result_next;
    assign md_alu_stall = md_alu_stall_next;
    assign md_alu_done = md_alu_done_next;

endmodule