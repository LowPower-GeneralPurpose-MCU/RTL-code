`timescale 1ns/1ps


module rob_state_onehot #(
    parameter ROB_DEPTH  = 64,
    parameter N_DISPATCH = 4,
    parameter N_CDB      = 4,
    parameter N_COMMIT   = 4,
    parameter ROB_TAG_W  = $clog2(ROB_DEPTH)
)(
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire                             flush_i,

    input  wire [N_DISPATCH-1:0]            dispatch_en_i,
    input  wire [N_DISPATCH*ROB_TAG_W-1:0]  dispatch_tag_i,

    input  wire [N_CDB-1:0]                 cdb_valid_i,
    input  wire [N_CDB*ROB_TAG_W-1:0]       cdb_tag_i,

    input  wire [N_COMMIT-1:0]              commit_en_i,
    input  wire [N_COMMIT*ROB_TAG_W-1:0]    commit_tag_i,

    output wire [ROB_DEPTH-1:0]             state_empty_o,
    output wire [ROB_DEPTH-1:0]             state_issued_o,
    output wire [ROB_DEPTH-1:0]             state_writeback_o,

    input  wire [ROB_TAG_W-1:0]             query_tag_a_i,
    input  wire [ROB_TAG_W-1:0]             query_tag_b_i,
    output wire                             query_a_writeback_o,
    output wire                             query_b_writeback_o
);

    // ROB_DEPTH-wide '1' → shift is always ROB_DEPTH bits wide (Bug 1 fix retained)
    localparam [ROB_DEPTH-1:0] ONE = {{(ROB_DEPTH-1){1'b0}}, 1'b1};

    // ----- State registers (bit planes) -----
    reg [ROB_DEPTH-1:0] bit_empty_r;
    reg [ROB_DEPTH-1:0] bit_issued_r;
    reg [ROB_DEPTH-1:0] bit_writeback_r;

    // =========================================================
    // One-Hot Decode: tag → ROB_DEPTH-wide mask per slot
    // =========================================================
    wire [ROB_DEPTH-1:0] dispatch_mask [0:N_DISPATCH-1];
    wire [ROB_DEPTH-1:0] cdb_mask      [0:N_CDB-1];
    wire [ROB_DEPTH-1:0] commit_mask   [0:N_COMMIT-1];

    genvar di, ci, mi;
    generate
        for (di = 0; di < N_DISPATCH; di = di + 1) begin : dispatch_decode
            wire [ROB_TAG_W-1:0] dtag = dispatch_tag_i[di*ROB_TAG_W +: ROB_TAG_W];
            assign dispatch_mask[di] = dispatch_en_i[di] ? (ONE << dtag)
                                                         : {ROB_DEPTH{1'b0}};
        end
        for (ci = 0; ci < N_CDB; ci = ci + 1) begin : cdb_decode
            wire [ROB_TAG_W-1:0] ctag = cdb_tag_i[ci*ROB_TAG_W +: ROB_TAG_W];
            assign cdb_mask[ci] = cdb_valid_i[ci] ? (ONE << ctag)
                                                   : {ROB_DEPTH{1'b0}};
        end
        for (mi = 0; mi < N_COMMIT; mi = mi + 1) begin : commit_decode
            wire [ROB_TAG_W-1:0] mtag = commit_tag_i[mi*ROB_TAG_W +: ROB_TAG_W];
            assign commit_mask[mi] = commit_en_i[mi] ? (ONE << mtag)
                                                      : {ROB_DEPTH{1'b0}};
        end
    endgenerate


    wire [ROB_DEPTH-1:0] disp_partial   [0:N_DISPATCH];
    wire [ROB_DEPTH-1:0] cdb_partial    [0:N_CDB];
    wire [ROB_DEPTH-1:0] commit_partial [0:N_COMMIT];

    genvar doi, coi, moi;
    generate
        // Dispatch OR tree
        assign disp_partial[0] = {ROB_DEPTH{1'b0}};
        for (doi = 0; doi < N_DISPATCH; doi = doi + 1) begin : disp_or
            assign disp_partial[doi+1] = disp_partial[doi] | dispatch_mask[doi];
        end
        // CDB OR tree
        assign cdb_partial[0] = {ROB_DEPTH{1'b0}};
        for (coi = 0; coi < N_CDB; coi = coi + 1) begin : cdb_or
            assign cdb_partial[coi+1] = cdb_partial[coi] | cdb_mask[coi];
        end
        // Commit OR tree
        assign commit_partial[0] = {ROB_DEPTH{1'b0}};
        for (moi = 0; moi < N_COMMIT; moi = moi + 1) begin : commit_or
            assign commit_partial[moi+1] = commit_partial[moi] | commit_mask[moi];
        end
    endgenerate

    wire [ROB_DEPTH-1:0] all_dispatch_mask_w = disp_partial[N_DISPATCH];
    wire [ROB_DEPTH-1:0] all_cdb_mask_w      = cdb_partial[N_CDB];
    wire [ROB_DEPTH-1:0] all_commit_mask_w   = commit_partial[N_COMMIT];

    // =========================================================
    // State Transition Register
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_empty_r     <= {ROB_DEPTH{1'b1}};
            bit_issued_r    <= {ROB_DEPTH{1'b0}};
            bit_writeback_r <= {ROB_DEPTH{1'b0}};
        end else if (flush_i) begin
            bit_empty_r     <= {ROB_DEPTH{1'b1}};
            bit_issued_r    <= {ROB_DEPTH{1'b0}};
            bit_writeback_r <= {ROB_DEPTH{1'b0}};
        end else begin
            bit_empty_r     <= (bit_empty_r & ~all_dispatch_mask_w) | all_commit_mask_w;
            bit_issued_r    <= ((bit_issued_r | all_dispatch_mask_w) &
                                ~all_cdb_mask_w) & ~all_commit_mask_w;
            bit_writeback_r <= ((bit_writeback_r |
                                 (all_cdb_mask_w & bit_issued_r)) & ~all_commit_mask_w);
        end
    end

    // ----- Outputs -----
    assign state_empty_o       = bit_empty_r;
    assign state_issued_o      = bit_issued_r;
    assign state_writeback_o   = bit_writeback_r;
    assign query_a_writeback_o = bit_writeback_r[query_tag_a_i];
    assign query_b_writeback_o = bit_writeback_r[query_tag_b_i];

endmodule
