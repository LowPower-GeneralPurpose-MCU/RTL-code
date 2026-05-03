`timescale 1ns/1ps


module rob_commit_scanner #(
    parameter ROB_DEPTH  = 64,
    parameter N_COMMIT   = 4,
    parameter ROB_TAG_W  = $clog2(ROB_DEPTH),
    parameter PTR_W      = ROB_TAG_W + 1
)(
    // [Issue 7] clk và rst_n đã bị xóa - module là fully combinational
    input  wire [ROB_TAG_W-1:0]           head_idx_i,  // unused internally but kept for debug
    input  wire [PTR_W-1:0]               count_i,
    input  wire                           empty_i,
    input  wire                           flush_i,

    // 2-bit state per entry from head: 2'b10=WRITEBACK, 2'b01=ISSUED, 2'b00=EMPTY
    input  wire [N_COMMIT*2-1:0]          state_i,
    input  wire [N_COMMIT-1:0]            exc_valid_i,

    output reg  [N_COMMIT-1:0]            commit_ready_o,
    output reg  [$clog2(N_COMMIT+1)-1:0]  commit_count_o,

    // Lookahead = commit_ready_o (same signal, kept for interface compatibility)
    output wire [N_COMMIT-1:0]            commit_ready_next_o
);

    localparam ST_WRITEBACK = 2'b10;
    localparam CNT_W        = $clog2(N_COMMIT + 1);

    integer j_v;
    reg [CNT_W-1:0] cnt_v;

    // =========================================================
    // Fully Combinational Chain Resolution
    //
    // Each slot j reads commit_ready_o[j-1] computed in the SAME
    // always @(*) evaluation pass → correct current-cycle chain.
    // Blocking assignment (=) ensures values propagate within the block.
    // =========================================================
    always @(*) begin
        commit_ready_o = {N_COMMIT{1'b0}};
        commit_count_o = {CNT_W{1'b0}};
        cnt_v          = {CNT_W{1'b0}};

        if (!empty_i && !flush_i) begin
            // Slot 0: head entry
            commit_ready_o[0] = (state_i[0*2 +: 2] == ST_WRITEBACK);
            if (commit_ready_o[0])
                cnt_v = {{(CNT_W-1){1'b0}}, 1'b1};

            // Slots 1..N_COMMIT-1: chain
            for (j_v = 1; j_v < N_COMMIT; j_v = j_v + 1) begin
                if ((state_i[j_v*2 +: 2] == ST_WRITEBACK) &&
                     commit_ready_o[j_v-1]                 &&   // reads THIS cycle's value
                    !exc_valid_i[j_v-1]                    &&   // stop at exception slot
                     (count_i > {{(PTR_W-CNT_W){1'b0}}, j_v[CNT_W-1:0]}))
                begin
                    commit_ready_o[j_v] = 1'b1;
                    cnt_v = cnt_v + {{(CNT_W-1){1'b0}}, 1'b1};
                end
            end
            commit_count_o = cnt_v;
        end
    end

    // Lookahead identical to commit_ready_o (both combinational)
    assign commit_ready_next_o = commit_ready_o;

endmodule
