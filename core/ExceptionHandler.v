`timescale 1ns/1ps

module exception_handler #(
    parameter PC_W    = 64,
    parameter XCODE_W = 8
)(
    input  wire               clk,
    input  wire               rst_n,

    // From ROB head
    input  wire               head_exc_valid_i,
    input  wire [XCODE_W-1:0] head_exc_code_i,
    input  wire [PC_W-1:0]    head_pc_i,
    input  wire               head_commit_ready_i,

    // From branch execution unit
    input  wire               br_mispredict_i,
    input  wire [PC_W-1:0]    br_correct_pc_i,

    // Registered outputs (1-cycle latency, for general pipeline flush)
    output reg                flush_o,
    output reg  [PC_W-1:0]    flush_pc_o,
    output reg                exc_valid_o,
    output reg  [XCODE_W-1:0] exc_code_o,

    // FIX Bug 7: Combinational early outputs (0-cycle latency, for time-critical paths)
    // These were previously computed locally but never connected anywhere.
    output wire               flush_early_o,
    output wire [PC_W-1:0]    flush_pc_early_o
);

    wire precise_exc = head_exc_valid_i && head_commit_ready_i;
    wire need_flush  = precise_exc || br_mispredict_i;

    assign flush_early_o    = need_flush;
    assign flush_pc_early_o = precise_exc ? head_pc_i : br_correct_pc_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flush_o     <= 1'b0;
            flush_pc_o  <= {PC_W{1'b0}};
            exc_valid_o <= 1'b0;
            exc_code_o  <= {XCODE_W{1'b0}};
        end else begin
            flush_o <= need_flush;

            if (precise_exc) begin
                exc_valid_o <= 1'b1;
                exc_code_o  <= head_exc_code_i;
                flush_pc_o  <= head_pc_i;
            end else if (br_mispredict_i) begin
                exc_valid_o <= 1'b0;
                exc_code_o  <= {XCODE_W{1'b0}};
                flush_pc_o  <= br_correct_pc_i;
            end else begin
                flush_o     <= 1'b0;
                exc_valid_o <= 1'b0;
            end
        end
    end

endmodule
