`timescale 1ns/1ps
// ============================================================
//  exception_handler.v  —  Precise Exception & Flush Controller
//
//  BUG ĐÃ SỬA:
//    [Bug 7] flush_immediate và flush_pc_immediate được tính toán
//    (tốn logic) nhưng không kết nối ra port nào và không dùng
//    nội bộ → dead code hoàn toàn. Synthesis tool optimize away,
//    nhưng lint tool sẽ cảnh báo "signal assigned but never read".
//    Quan trọng hơn: ý định của code là cung cấp fast flush path
//    (bypass registered output) cho các stage cần phản hồi ngay
//    mà không thể chờ 1 cycle. Bỏ đi sẽ mất tính năng này.
//
//    Sửa: expose chúng thành output wire port:
//      flush_early_o    — combinational, dùng cho critical flush paths
//      flush_pc_early_o — combinational PC tương ứng
//
//    Sự khác biệt giữa hai cặp output:
//      flush_o / flush_pc_o      : registered (1 cycle trễ), ổn định hơn
//      flush_early_o / flush_pc_early_o : combinational, phản hồi ngay
//
//    Top-level kết nối flush_early_o tới các sub-module cần fast flush.
// ============================================================

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
