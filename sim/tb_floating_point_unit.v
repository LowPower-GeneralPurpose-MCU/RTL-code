`timescale 1ns / 1ps

module tb_floating_point_unit();

    // =========================================================================
    // SIGNALS & INSTANTIATION
    // =========================================================================
    reg         clk;
    reg         reset_n;
    reg         fpu_start;
    reg  [4:0]  fpu_op;
    reg  [31:0] operand_a;
    reg  [31:0] operand_b;
    wire [31:0] result;
    wire        fpu_stall;
    wire        fpu_done;

    fpu_unit #(
        .BITS_PER_CYCLE(2)
    ) dut (
        .clk(clk),
        .reset_n(reset_n),
        .fpu_start(fpu_start),
        .fpu_op(fpu_op),
        .operand_a(operand_a),
        .operand_b(operand_b),
        .result(result),
        .fpu_stall(fpu_stall),
        .fpu_done(fpu_done)
    );

    // =========================================================================
    // CLOCK GENERATOR
    // =========================================================================
    initial begin clk = 0; forever #5 clk = ~clk; end

    // =========================================================================
    // FPU OPCODES
    // =========================================================================
    localparam FOP_ADD      = 5'b00000;
    localparam FOP_SUB      = 5'b00001;
    localparam FOP_MUL      = 5'b00010;
    localparam FOP_CVT_W_S  = 5'b00011; 
    localparam FOP_CVT_S_W  = 5'b00100; 
    localparam FOP_EQ       = 5'b00101;
    localparam FOP_LT       = 5'b00110;
    localparam FOP_LE       = 5'b00111;
    localparam FOP_DIV      = 5'b01000;
    localparam FOP_SQRT     = 5'b01001;
    localparam FOP_MIN      = 5'b01010;
    localparam FOP_MAX      = 5'b01011;
    localparam FOP_MV_X_W   = 5'b01111; 
    localparam FOP_MV_W_X   = 5'b10000; 
    localparam FOP_CVT_WU_S = 5'b10010; 
    localparam FOP_CVT_S_WU = 5'b10011; 

    // =========================================================================
    // SCOREBOARD & EXECUTION TASK
    // =========================================================================
    integer test_passed = 0;
    integer test_failed = 0;

    task run_fpu_test(input [4:0] op, input [31:0] a, input [31:0] b, input [31:0] exp_res, input [8*60:1] test_name);
        begin
            @(posedge clk);
            fpu_op    = op;
            operand_a = a;
            operand_b = b;
            fpu_start = 1;

            @(posedge clk);
            fpu_start = 0;

            wait(fpu_done == 1'b1);
            
            // Check logic: Exact match OR NaN wildcard
            if (result === exp_res || (exp_res[30:23] == 8'hFF && result[30:23] == 8'hFF && result[22:0] != 0)) begin
                $display("  [PASS] %0s", test_name);
                test_passed = test_passed + 1;
            end else begin
                $display("  [FAIL] %0s", test_name);
                $display("         Expected: %h, Got: %h <--- ERROR!", exp_res, result);
                test_failed = test_failed + 1;
            end
            @(posedge clk);
        end
    endtask

    // =========================================================================
    // 12-PHASE TORTURE SUITE
    // =========================================================================
    initial begin
        fpu_start = 0; fpu_op = 0; operand_a = 0; operand_b = 0;
        reset_n = 0; #50 reset_n = 1; #20;

        $display("\n=========================================================");
        $display(" PHASE 1: BASIC ARITHMETIC (WARM-UP)");
        run_fpu_test(FOP_ADD, 32'h3FC0_0000, 32'h4020_0000, 32'h4080_0000, "1.5 + 2.5 = 4.0");
        run_fpu_test(FOP_SUB, 32'h40A0_0000, 32'h3F80_0000, 32'h4080_0000, "5.0 - 1.0 = 4.0");
        run_fpu_test(FOP_MUL, 32'h4000_0000, 32'h4060_0000, 32'h40E0_0000, "2.0 * 3.5 = 7.0");
        run_fpu_test(FOP_DIV, 32'h4120_0000, 32'h4020_0000, 32'h4080_0000, "10.0 / 2.5 = 4.0");

        $display("\n=========================================================");
        $display(" PHASE 2: MATHEMATICAL FUNCTIONS (SQRT)");
        run_fpu_test(FOP_SQRT, 32'h4180_0000, 0, 32'h4080_0000, "sqrt(16.0) = 4.0");
        run_fpu_test(FOP_SQRT, 32'h4080_0000, 0, 32'h4000_0000, "sqrt(4.0) = 2.0");

        $display("\n=========================================================");
        $display(" PHASE 3: COMPARISONS & MIN/MAX");
        run_fpu_test(FOP_EQ,  32'h40A0_0000, 32'h40A0_0000, 32'd1,        "5.0 == 5.0 -> True");
        run_fpu_test(FOP_LT,  32'h3F80_0000, 32'h40A0_0000, 32'd1,        "1.0 < 5.0  -> True");
        run_fpu_test(FOP_MIN, 32'hBF80_0000, 32'h40A0_0000, 32'hBF80_0000, "min(-1.0, 5.0) = -1.0");
        run_fpu_test(FOP_MAX, 32'h4040_0000, 32'h40E0_0000, 32'h40E0_0000, "max(3.0, 7.0) = 7.0");

        $display("\n=========================================================");
        $display(" PHASE 4: CONVERSIONS (FLOAT <-> INT)");
        run_fpu_test(FOP_CVT_W_S,  32'h40A0_0000, 0, 32'd5,           "Float(5.0) -> Int(5)");
        run_fpu_test(FOP_CVT_S_W,  32'hFFFF_FFFB, 0, 32'hC0A0_0000,  "Int(-5) -> Float(-5.0)"); 
        run_fpu_test(FOP_CVT_WU_S, 32'h4120_0000, 0, 32'd10,          "Float(10.0) -> UInt(10)");
        run_fpu_test(FOP_CVT_S_WU, 32'd16,          0, 32'h4180_0000,  "UInt(16) -> Float(16.0)");

        $display("\n=========================================================");
        $display(" PHASE 5: BITCAST MOVES (RAW BITS)");
        run_fpu_test(FOP_MV_X_W, 32'h4120_0000, 0, 32'h4120_0000, "F->I Bitcast");
        run_fpu_test(FOP_MV_W_X, 32'h4120_0000, 0, 32'h4120_0000, "I->F Bitcast");

        $display("\n=========================================================");
        $display(" PHASE 6: EDGE CASES (ZERO, INF, NAN)");
        run_fpu_test(FOP_DIV,  32'h4120_0000, 0, 32'h7F80_0000, "10.0 / 0.0 = Infinity"); 
        run_fpu_test(FOP_SQRT, 32'hBF80_0000, 0, 32'h7FC0_0000, "sqrt(-1.0) = NaN");
        run_fpu_test(FOP_ADD,  32'h40A0_0000, 0, 32'h40A0_0000, "5.0 + 0.0 = 5.0");

        $display("\n=========================================================");
        $display(" PHASE 7: EXTREME - CATASTROPHIC CANCELLATION");
        // (1 + 2^-23) - 1 = 2^-23. Hex: 34000000
        run_fpu_test(FOP_SUB, 32'h3F80_0001, 32'h3F80_0000, 32'h3400_0000, "1.0000001 - 1.0 = 2^-23");
        run_fpu_test(FOP_SUB, 32'h47C3_5000, 32'h47C3_4FFF, 32'h3C00_0000, "100000 - 99999.99 = 0.0078");

        $display("\n=========================================================");
        $display(" PHASE 8: EXTREME - ABSORPTION & STICKY BIT");
        run_fpu_test(FOP_ADD, 32'h4B80_0000, 32'h3F80_0000, 32'h4B80_0000, "16777216.0 + 1.0 (Absorbed)");
        run_fpu_test(FOP_ADD, 32'h4B7F_FFFF, 32'h3F80_0000, 32'h4B80_0000, "16777215.0 + 1.0 (Carry)");

        $display("\n=========================================================");
        $display(" PHASE 9: EXTREME - IRRATIONAL & REPEATING");
        run_fpu_test(FOP_DIV, 32'h3F80_0000, 32'h4040_0000, 32'h3EAA_AAAB, "1.0 / 3.0 (Round Up)");
        run_fpu_test(FOP_DIV, 32'h43B1_8000, 32'h42E2_0000, 32'h4049_0FDC, "355/113 = Pi (Round Up)");
        run_fpu_test(FOP_SQRT, 32'h4000_0000, 0, 32'h3FB5_04F3, "sqrt(2.0) (Restoring)");
        run_fpu_test(FOP_SQRT, 32'h4040_0000, 0, 32'h3FDD_B3D7, "sqrt(3.0) (Restoring)");

        $display("\n=========================================================");
        $display(" PHASE 10: EXTREME - BOUNDARY MULTIPLICATION");
        run_fpu_test(FOP_MUL, 32'h3FFF_FFFF, 32'h3FFF_FFFF, 32'h407F_FFFE, "MaxMant * MaxMant");
        run_fpu_test(FOP_MUL, 32'h7000_0000, 32'h0F00_0000, 32'h3F80_0000, "2^97 * 2^-97 = 1.0");

        $display("\n=========================================================");
        $display(" PHASE 11: FLOAT-TO-INT TRUNCATION");
        run_fpu_test(FOP_CVT_W_S, 32'h407F_5C29, 0, 32'd3,           "Float(3.99) -> Int(3)");
        run_fpu_test(FOP_CVT_W_S, 32'hC03F_5C29, 0, 32'hFFFF_FFFE, "Float(-2.99) -> Int(-2)");

        $display("\n=========================================================");
        $display(" PHASE 12: CHAIN REACTION & TINY DECIMALS");
        run_fpu_test(FOP_DIV, 32'h3F80_0000, 32'h4120_0000, 32'h3DCC_CCCD, "1.0 / 10.0 = 0.1");
        run_fpu_test(FOP_MUL, 32'h3DCC_CCCD, 32'h4120_0000, 32'h3F80_0000, "0.1 * 10.0 = 1.0");

        #200;
        $display("\n=========================================================");
        $display(" FPU VERIFICATION SCOREBOARD");
        $display("=========================================================");
        $display(" Total Tests Conducted : %0d", test_passed + test_failed);
        $display(" PASSED                : %0d", test_passed);
        $display(" FAILED                : %0d", test_failed);
        
        if (test_failed == 0) 
            $display("\n >>> SUCCESS! ALL TEST PASSED! <<<\n");
        else 
            $display("\n >>> CRITICAL FAILURE DETECTED! <<<\n");
        
        $finish;
    end
endmodule