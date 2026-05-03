`timescale 1ns/1ps
module rob_ptr_manager_superscalar #(
    parameter DEPTH      = 64,
    parameter PTRW       = $clog2(DEPTH) + 1,
    parameter N_DISPATCH = 4,
    parameter N_COMMIT   = 4
)(
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire [$clog2(N_DISPATCH+1)-1:0]  dispatch_count_i,
    input  wire [$clog2(N_COMMIT+1)-1:0]    commit_count_i,
    input  wire                             flush_i,
    output wire                             full_o,
    output wire                             empty_o,
    output wire [PTRW-2:0]                  head_o,
    output wire [PTRW-2:0]                  tail_o,
    output wire [PTRW-1:0]                  count_o,
    // [Issue 8] New output: available slots accounting for same-cycle commits
    output wire [PTRW-1:0]                  adjusted_free_o
);

    localparam IDX_W  = PTRW - 1;
    localparam CNTW   = $clog2(N_COMMIT   + 1);
    localparam DISPW  = $clog2(N_DISPATCH + 1);

    reg [PTRW-1:0] head_r, tail_r;

    wire [PTRW-1:0] head_next = head_r + {{(PTRW-CNTW){1'b0}},  commit_count_i};
    wire [PTRW-1:0] tail_next = tail_r + {{(PTRW-DISPW){1'b0}}, dispatch_count_i};

    wire [PTRW-1:0] current_count   = tail_r - head_r;
    wire [PTRW-1:0] available_slots = DEPTH   - current_count;
    wire [PTRW-1:0] effective_free_slots = available_slots + {{(PTRW-CNTW){1'b0}}, commit_count_i};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head_r <= {PTRW{1'b0}};
            tail_r <= {PTRW{1'b0}};
        end else if (flush_i) begin
            tail_r <= head_r; // precise flush: tail snaps to head
        end else begin
            // Commit: advance head
            if (commit_count_i != {CNTW{1'b0}} &&
                current_count >= {{(PTRW-CNTW){1'b0}}, commit_count_i})
                head_r <= head_next;

            // Dispatch: advance tail
            // Use adjusted_free_o-based check in top-level, not full_o.
            // Here we still guard with available_slots (pre-commit value).
            if (dispatch_count_i != {DISPW{1'b0}} &&
                effective_free_slots >= {{(PTRW-DISPW){1'b0}}, dispatch_count_i})
                tail_r <= tail_next;
        end
    end

    assign full_o   = (head_r[IDX_W-1:0] == tail_r[IDX_W-1:0]) &&
                      (head_r[IDX_W]     != tail_r[IDX_W]);
    assign empty_o  = (head_r == tail_r);
    assign head_o   = head_r[IDX_W-1:0];
    assign tail_o   = tail_r[IDX_W-1:0];
    assign count_o  = current_count;

    // [Issue 8] adjusted_free: how many slots are actually usable this cycle,
    // counting both currently empty slots AND slots freed by same-cycle commits.
    assign adjusted_free_o = effective_free_slots;

endmodule
