// ============================================================================
// Utility: rob_onehot_mux
// ----------------------------------------------------------------------------
// Thay thế mux-if-chain bằng AND-mask + OR-reduce.
// Synthesizer thấy N cặp AND song song rồi OR-tree → không có carry-chain.
//
//   data_o = OR_i( data_i[i*W +: W] & {W{sel_i[i]}} )
// ============================================================================
module rob_onehot_mux #(
    parameter W = 32,   // bit-width của mỗi phần tử
    parameter N = 8     // số phần tử
)(
    input  wire [N*W-1:0] din,   // flattened: phần tử i tại din[i*W +: W]
    input  wire [N-1:0]   sel,   // one-hot select
    output wire [W-1:0]   dout
);
    // Mỗi masked[i] = 0 nếu ~sel[i], = din[i] nếu sel[i]
    wire [W-1:0] masked [0:N-1];
    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : gen_mask
            assign masked[g] = {W{sel[g]}} & din[g*W +: W];
        end
    endgenerate

    // OR-tree (synthesizer tự flatten thành cây)
    wire [W-1:0] tree [0:N-1];
    generate
        assign tree[0] = masked[0];
        for (g = 1; g < N; g = g + 1) begin : gen_or
            assign tree[g] = tree[g-1] | masked[g];
        end
    endgenerate
    assign dout = tree[N-1];
endmodule

module oh2bin #(
    parameter N = 8,
    parameter W = 3
)(
    input  wire [N-1:0] oh,
    output wire [W-1:0] bin
);
    wire [W-1:0] tmp  [0:N-1];
    wire [W-1:0] tree [0:N-1];
    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : gen_enc
            assign tmp[g] = {W{oh[g]}} & g[W-1:0];
        end
        assign tree[0] = tmp[0];
        for (g = 1; g < N; g = g + 1) begin : gen_or
            assign tree[g] = tree[g-1] | tmp[g];
        end
    endgenerate
    assign bin = tree[N-1];
endmodule


module rot_pri_enc #(
    parameter N     = 8,
    parameter PTR_W = 3
)(
    input  wire [N-1:0]     req,
    input  wire [PTR_W-1:0] start,
    output wire [N-1:0]     grant_oh,
    output wire             grant_vld
);
    wire [2*N-1:0] req2        = {req, req};
    wire [2*N-1:0] start_mask  = ({2*N{1'b1}}) << start;
    wire [2*N-1:0] masked      = req2 & start_mask;
    // lsb = masked & (-masked):  twos-complement negation
    wire [2*N-1:0] masked_neg  = (~masked) + 1'b1;
    wire [2*N-1:0] lsb2        = masked & masked_neg;

    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : gen_fold
            assign grant_oh[g] = lsb2[g] | lsb2[g + N];
        end
    endgenerate

    assign grant_vld = |req;
endmodule


// ============================================================================
// axi_read_reorder_buffer_core  (optimized)
// ============================================================================
// Các thay đổi so với bản gốc:
//
//  A. FREE-SLOT FINDER
//     Gốc: for-loop priority scan → N-stage if-chain.
//     Mới: inv & (-inv) → lsb one-hot → oh2bin.
//          Depth: 1 (inv) + adder(N) + AND + oh2bin-OR-tree ≈ log2(N) LUT.
//
//  B. RESP TAG READY/VALID VECTOR
//     Gốc: always @(*) for-loop viết từng bit.
//     Mới: genvar generate → N comparator song song (không phụ thuộc nhau).
//          Synthesizer map thành LUT riêng biệt, không chia sẻ.
//
//  C. OUTPUT SCHEDULER
//     Gốc: for-loop với integer scan_tag_int và if-chain.
//     Mới:
//       1. req_vec (bitwise genvar): is_head & valid & (err | recv_nonzero)
//       2. rot_pri_enc → grant_oh (one-hot, round-robin từ rr_tag_q)
//       3. oh2bin → choose_tag_c
//       4. onehot_mux → choose_id_c  (không cần mux-if)
//
//  D. FLATTENED REGISTER ARRAYS
//     Gốc: reg [ID_WIDTH-1:0] entry_id_q [0:ROB_DEPTH-1]  → ROM-style index
//          trong always-block gây ra LUT multiplexer cho read.
//     Mới: reg [ID_WIDTH*ROB_DEPTH-1:0] entry_id_flat_q
//          → slice đơn [i*W +: W] cho cả read lẫn write, không cần loop.
//          genvar comparator đọc trực tiếp slice, không qua mux.
//
//  E. BRAM / PER-ID ORDER
//     Beat data stays in a reset-insensitive BRAM process. Per-ID order uses
//     a compact linked list of ROB tags instead of ID_COUNT*ROB_DEPTH queues.
// ============================================================================
module axi_read_reorder_buffer_core #(
    parameter DATA_WIDTH      = 32,
    parameter ID_WIDTH        = 5,
    parameter LEN_WIDTH       = 8,
    parameter RESP_WIDTH      = 2,
    parameter ROB_DEPTH       = 8,
    parameter MAX_BURST_BEATS = 64,
    parameter ROB_TAG_W       = 3,
    parameter BEAT_CNT_W      = 7,
    parameter ID_COUNT        = 32,
    parameter ID_PTR_W        = 3,
    parameter ID_CNT_W        = 4,
    parameter [RESP_WIDTH-1:0] DECERR_RESP = {RESP_WIDTH{1'b1}}
)(
    input  wire                     ACLK_i,
    input  wire                     ARESETn_i,

    // AR allocation
    input  wire                     alloc_valid_i,
    output wire                     alloc_ready_o,
    input  wire [ID_WIDTH-1:0]      alloc_id_i,
    input  wire [LEN_WIDTH-1:0]     alloc_len_i,
    input  wire                     alloc_err_i,
    output wire [ROB_TAG_W-1:0]     alloc_tag_o,
    output wire                     alloc_fire_o,

    // R response input
    input  wire                     r_valid_i,
    output wire                     r_ready_o,
    input  wire [ROB_TAG_W-1:0]     r_tag_i,
    input  wire [DATA_WIDTH-1:0]    r_data_i,
    input  wire [RESP_WIDTH-1:0]    r_resp_i,
    input  wire                     r_last_i,

    // For wrappers with multiple R inputs
    output wire [ROB_DEPTH-1:0]     resp_tag_ready_vec_o,
    output wire [ROB_DEPTH-1:0]     resp_tag_valid_vec_o,

    // R output to master
    output wire                     m_RVALID_o,
    input  wire                     m_RREADY_i,
    output wire [ID_WIDTH-1:0]      m_RID_o,
    output wire [DATA_WIDTH-1:0]    m_RDATA_o,
    output wire [RESP_WIDTH-1:0]    m_RRESP_o,
    output wire                     m_RLAST_o,

    // Status/debug
    output wire                     rob_full_o,
    output wire                     id_order_full_o,
    output reg                      r_unexpected_tag_o,
    output reg                      r_overflow_o,
    output reg                      r_last_mismatch_o
);

    reg [ROB_DEPTH-1:0]                  entry_valid_q;
    reg [ROB_DEPTH-1:0]                  entry_err_q;
    reg [ROB_DEPTH-1:0]                  entry_is_head_q;
    reg [ID_WIDTH*ROB_DEPTH-1:0]         entry_id_flat_q;
    reg [ROB_TAG_W*ROB_DEPTH-1:0]        entry_next_flat_q;
    reg [BEAT_CNT_W*ROB_DEPTH-1:0]      entry_expected_flat_q;
    reg [BEAT_CNT_W*ROB_DEPTH-1:0]      entry_recv_flat_q;

    // BRAM beat memory (reset-insensitive process để tool suy ra BRAM)
    (* ram_style = "block" *)
    reg [DATA_WIDTH+RESP_WIDTH-1:0]      beat_mem_q [0:ROB_DEPTH*MAX_BURST_BEATS-1];

    // Per-ID order tracking: linked ROB entries instead of ID_COUNT*ROB_DEPTH queues.
    reg [ROB_TAG_W-1:0]                  id_tail_tag_q [0:ID_COUNT-1];
    reg [ID_CNT_W-1:0]                   idq_count_q [0:ID_COUNT-1];

    // Active output burst state
    reg                                  active_q;
    reg [ROB_TAG_W-1:0]                  active_tag_q;
    reg [ID_WIDTH-1:0]                   active_id_q;
    reg [ROB_TAG_W-1:0]                  rr_tag_q;
    reg [BEAT_CNT_W-1:0]                active_read_cnt_q;

    // Registered output buffer
    reg                                  out_valid_q;
    reg [ID_WIDTH-1:0]                   out_id_q;
    reg [DATA_WIDTH+RESP_WIDTH-1:0]      out_payload_q;
    reg                                  out_last_q;

    // BRAM read pipeline
    reg                                  read_pending_q;
    reg [DATA_WIDTH+RESP_WIDTH-1:0]      beat_rd_q;
    reg                                  read_last_q;
    reg [ID_WIDTH-1:0]                   read_id_q;

    // Misc
    integer i;
    localparam [LEN_WIDTH-1:0]  MAX_ARLEN_C = MAX_BURST_BEATS - 1;
    localparam [BEAT_CNT_W-1:0] MAX_BEATS_C = MAX_BURST_BEATS;

    // ------------------------------------------------------------------ //
    // Helper functions (giữ nguyên cho các phép tính đơn-entry)
    // ------------------------------------------------------------------ //
    function [ROB_TAG_W-1:0] rob_tag_inc;
        input [ROB_TAG_W-1:0] tag;
        begin
            if (tag == ROB_DEPTH - 1) rob_tag_inc = {ROB_TAG_W{1'b0}};
            else                       rob_tag_inc = tag + {{(ROB_TAG_W-1){1'b0}}, 1'b1};
        end
    endfunction

    function integer rob_mem_idx;
        input [ROB_TAG_W-1:0]  tag;
        input [BEAT_CNT_W-1:0] beat;
        begin rob_mem_idx = (tag * MAX_BURST_BEATS) + beat; end
    endfunction

    // Slice helpers – trích xuất field từ flat arrays cho entry đang dùng
    wire [BEAT_CNT_W-1:0] act_expected =
        entry_expected_flat_q[active_tag_q*BEAT_CNT_W +: BEAT_CNT_W];
    wire [BEAT_CNT_W-1:0] act_recv =
        entry_recv_flat_q[active_tag_q*BEAT_CNT_W +: BEAT_CNT_W];
    wire [BEAT_CNT_W-1:0] rtag_recv =
        entry_recv_flat_q[r_tag_i*BEAT_CNT_W +: BEAT_CNT_W];
    wire [BEAT_CNT_W-1:0] rtag_expected =
        entry_expected_flat_q[r_tag_i*BEAT_CNT_W +: BEAT_CNT_W];

    // alloc_len width-matching
    wire [BEAT_CNT_W-1:0] alloc_len_ext_c;
    generate
        if (BEAT_CNT_W >= LEN_WIDTH)
            assign alloc_len_ext_c = {{(BEAT_CNT_W-LEN_WIDTH){1'b0}}, alloc_len_i};
        else
            assign alloc_len_ext_c = alloc_len_i[BEAT_CNT_W-1:0];
    endgenerate

    wire                   alloc_len_ok_c = (alloc_len_i <= MAX_ARLEN_C);
    wire [BEAT_CNT_W-1:0] alloc_expected_c =
        alloc_len_ok_c ? (alloc_len_ext_c + {{(BEAT_CNT_W-1){1'b0}}, 1'b1}) : MAX_BEATS_C;

    // ------------------------------------------------------------------ //
    // [A] FREE-SLOT FINDER
    //   inv  = ~entry_valid_q          (một-bước, không cần for)
    //   lsb  = inv & (-inv)            (two-complement trick, one-hot)
    //   tag  = oh2bin(lsb)
    // ------------------------------------------------------------------ //
    wire [ROB_DEPTH-1:0] free_oh;
    wire [ROB_TAG_W-1:0] free_tag_c;
    wire                 free_found_c;

    wire [ROB_DEPTH-1:0] inv_valid = ~entry_valid_q;
    assign free_oh      = inv_valid & (~inv_valid + {{(ROB_DEPTH-1){1'b0}}, 1'b1});
    assign free_found_c = |inv_valid; // = |inv_valid

    oh2bin #(.N(ROB_DEPTH), .W(ROB_TAG_W)) u_free_enc (
        .oh (free_oh),
        .bin(free_tag_c)
    );

    // ------------------------------------------------------------------ //
    // Alloc handshake
    // ------------------------------------------------------------------ //
    assign id_order_full_o = (idq_count_q[alloc_id_i] == ROB_DEPTH[ID_CNT_W-1:0]);
    assign rob_full_o      = ~free_found_c;
    assign alloc_ready_o   = free_found_c & ~id_order_full_o & alloc_len_ok_c;
    assign alloc_tag_o     = free_tag_c;
    assign alloc_fire_o    = alloc_valid_i & alloc_ready_o;


    genvar g;
    generate
        for (g = 0; g < ROB_DEPTH; g = g + 1) begin : gen_tag_vec
            wire [BEAT_CNT_W-1:0] recv_g =
                entry_recv_flat_q[g*BEAT_CNT_W +: BEAT_CNT_W];
            wire [BEAT_CNT_W-1:0] exp_g =
                entry_expected_flat_q[g*BEAT_CNT_W +: BEAT_CNT_W];

            assign resp_tag_valid_vec_o[g] =
                entry_valid_q[g] & ~entry_err_q[g];

            assign resp_tag_ready_vec_o[g] =
                resp_tag_valid_vec_o[g] &
                (recv_g < exp_g);
        end
    endgenerate

    wire r_fire_c = r_valid_i & resp_tag_ready_vec_o[r_tag_i];
    assign r_ready_o = resp_tag_ready_vec_o[r_tag_i];

    // recv+1 cho entry r_tag_i
    wire [BEAT_CNT_W-1:0] recv_plus_one =
        rtag_recv + {{(BEAT_CNT_W-1){1'b0}}, 1'b1};

    // ------------------------------------------------------------------ //
    // [C] OUTPUT SCHEDULER
    //   Bước 1: req_vec – bitwise genvar (không có for-if chain).
    // ------------------------------------------------------------------ //
    wire [ROB_DEPTH-1:0] recv_nonzero_vec;
    generate
        for (g = 0; g < ROB_DEPTH; g = g + 1) begin : gen_recv_nonzero
            wire [BEAT_CNT_W-1:0] r_g =
                entry_recv_flat_q[g*BEAT_CNT_W +: BEAT_CNT_W];
            assign recv_nonzero_vec[g] = |r_g;
        end
    endgenerate

    wire [ROB_DEPTH-1:0] sched_req =
        entry_is_head_q & entry_valid_q & (entry_err_q | recv_nonzero_vec);

    //   Bước 2: rot_pri_enc → one-hot grant từ rr_tag_q
    wire [ROB_DEPTH-1:0] choose_oh;
    wire                 choose_valid_c;

    rot_pri_enc #(.N(ROB_DEPTH), .PTR_W(ROB_TAG_W)) u_sched_enc (
        .req      (sched_req),
        .start    (rr_tag_q),
        .grant_oh (choose_oh),
        .grant_vld(choose_valid_c)
    );

    //   Bước 3: oh2bin → choose_tag_c (binary index)
    wire [ROB_TAG_W-1:0] choose_tag_c;
    oh2bin #(.N(ROB_DEPTH), .W(ROB_TAG_W)) u_sched_tag (
        .oh  (choose_oh),
        .bin (choose_tag_c)
    );

    //   Bước 4: rob_onehot_mux → choose_id_c (không cần mux-if trên array)
    wire [ID_WIDTH-1:0] choose_id_c;
    rob_onehot_mux #(.W(ID_WIDTH), .N(ROB_DEPTH)) u_choose_id (
        .din  (entry_id_flat_q),
        .sel  (choose_oh),
        .dout (choose_id_c)
    );

    // ------------------------------------------------------------------ //
    // Master output datapath
    // ------------------------------------------------------------------ //
    assign m_RVALID_o = out_valid_q;
    assign m_RID_o    = out_valid_q ? out_id_q                             : {ID_WIDTH{1'b0}};
    assign m_RDATA_o  = out_valid_q ? out_payload_q[DATA_WIDTH-1:0]        : {DATA_WIDTH{1'b0}};
    assign m_RRESP_o  = out_valid_q ? out_payload_q[DATA_WIDTH +: RESP_WIDTH] : {RESP_WIDTH{1'b0}};
    assign m_RLAST_o  = out_valid_q ? out_last_q                           : 1'b0;

    wire out_fire_c     = out_valid_q & m_RREADY_i;
    wire dealloc_fire_c = out_fire_c & out_last_q;
    wire out_buf_free_c = ~out_valid_q | out_fire_c;

    wire issue_data_read_c =
        active_q &
        ~entry_err_q[active_tag_q] &
        ~read_pending_q &
        out_buf_free_c &
        (active_read_cnt_q < act_recv) &
        (active_read_cnt_q < act_expected);

    wire issue_err_beat_c =
        active_q &
        entry_err_q[active_tag_q] &
        out_buf_free_c &
        (active_read_cnt_q < act_expected);

    // Next-head logic from the per-ID linked list.
    wire [ROB_TAG_W-1:0] next_head_tag_c =
        entry_next_flat_q[active_tag_q*ROB_TAG_W +: ROB_TAG_W];
    wire has_next_c =
        (idq_count_q[active_id_q] > {{(ID_CNT_W-1){1'b0}}, 1'b1});
    wire alloc_same_id_c =
        alloc_fire_o & active_q & (alloc_id_i == active_id_q);

    // ================================================================== //n
    // Sequential logic (control path, reset-sensitive)
    // ================================================================== //
    always @(posedge ACLK_i or negedge ARESETn_i) begin
        if (!ARESETn_i) begin
            active_q           <= 1'b0;
            active_tag_q       <= {ROB_TAG_W{1'b0}};
            active_id_q        <= {ID_WIDTH{1'b0}};
            rr_tag_q           <= {ROB_TAG_W{1'b0}};
            active_read_cnt_q  <= {BEAT_CNT_W{1'b0}};

            out_valid_q        <= 1'b0;
            out_id_q           <= {ID_WIDTH{1'b0}};
            out_payload_q      <= {(DATA_WIDTH+RESP_WIDTH){1'b0}};
            out_last_q         <= 1'b0;

            read_pending_q     <= 1'b0;
            read_last_q        <= 1'b0;
            read_id_q          <= {ID_WIDTH{1'b0}};

            r_unexpected_tag_o <= 1'b0;
            r_overflow_o       <= 1'b0;
            r_last_mismatch_o  <= 1'b0;

            // [D] Reset flattened arrays – dùng concat literal, không cần for-loop
            entry_valid_q         <= {ROB_DEPTH{1'b0}};
            entry_err_q           <= {ROB_DEPTH{1'b0}};
            entry_is_head_q       <= {ROB_DEPTH{1'b0}};
            entry_id_flat_q       <= {(ID_WIDTH*ROB_DEPTH){1'b0}};
            entry_next_flat_q     <= {(ROB_TAG_W*ROB_DEPTH){1'b0}};
            entry_expected_flat_q <= {(BEAT_CNT_W*ROB_DEPTH){1'b0}};
            entry_recv_flat_q     <= {(BEAT_CNT_W*ROB_DEPTH){1'b0}};

            // idq counters/pointers vẫn cần for vì là unpacked array
            for (i = 0; i < ID_COUNT; i = i + 1) begin
                id_tail_tag_q[i] <= {ROB_TAG_W{1'b0}};
                idq_count_q[i]   <= {ID_CNT_W{1'b0}};
            end

        end else begin

            // ----------------------------------------------------------
            // Output register: xoá khi fire
            // ----------------------------------------------------------
            if (out_fire_c)
                out_valid_q <= 1'b0;

            // ----------------------------------------------------------
            // Allocate ROB entry
            // ----------------------------------------------------------
            if (alloc_fire_o) begin
                entry_valid_q[free_tag_c]    <= 1'b1;
                entry_err_q[free_tag_c]      <= alloc_err_i;
                // DECERR entries produce local error beats and do not accept slave R beats.
                entry_is_head_q[free_tag_c]  <=
                    (idq_count_q[alloc_id_i] == {ID_CNT_W{1'b0}});

                // [D] Ghi vào slice của flat array – không cần index-mux
                entry_id_flat_q      [free_tag_c*ID_WIDTH   +: ID_WIDTH]   <= alloc_id_i;
                entry_next_flat_q    [free_tag_c*ROB_TAG_W  +: ROB_TAG_W]  <= {ROB_TAG_W{1'b0}};
                entry_expected_flat_q[free_tag_c*BEAT_CNT_W +: BEAT_CNT_W] <= alloc_expected_c;
                entry_recv_flat_q    [free_tag_c*BEAT_CNT_W +: BEAT_CNT_W] <= {BEAT_CNT_W{1'b0}};

                if (idq_count_q[alloc_id_i] != {ID_CNT_W{1'b0}})
                    entry_next_flat_q[id_tail_tag_q[alloc_id_i]*ROB_TAG_W +: ROB_TAG_W] <= free_tag_c;
                id_tail_tag_q[alloc_id_i] <= free_tag_c;
                if (!(dealloc_fire_c && (alloc_id_i == active_id_q)))
                    idq_count_q[alloc_id_i] <=
                        idq_count_q[alloc_id_i] + {{(ID_CNT_W-1){1'b0}}, 1'b1};
            end

            // Store incoming R beat
            if (r_valid_i && !r_ready_o)
                r_unexpected_tag_o <= 1'b1;

            if (r_fire_c) begin
                if (rtag_recv >= rtag_expected)
                    r_overflow_o <= 1'b1;

                if (recv_plus_one == rtag_expected) begin
                    if (!r_last_i) r_last_mismatch_o <= 1'b1;
                end else begin
                    if (r_last_i)  r_last_mismatch_o <= 1'b1;
                end

                // [D] Cập nhật slice recv counter
                entry_recv_flat_q[r_tag_i*BEAT_CNT_W +: BEAT_CNT_W] <= recv_plus_one;
            end

            // Fill output register từ BRAM read pipeline
            if (read_pending_q) begin
                out_valid_q    <= 1'b1;
                out_id_q       <= read_id_q;
                out_payload_q  <= beat_rd_q;
                out_last_q     <= read_last_q;
                read_pending_q <= 1'b0;
            end

            // Issue synchronous BRAM read
            if (issue_data_read_c) begin
                read_pending_q <= 1'b1;
                read_last_q    <= (active_read_cnt_q ==
                    (act_expected - {{(BEAT_CNT_W-1){1'b0}}, 1'b1}));
                read_id_q      <= active_id_q;
                active_read_cnt_q <= active_read_cnt_q + {{(BEAT_CNT_W-1){1'b0}}, 1'b1};
            end

            // ----------------------------------------------------------
            // Generate DECERR beat (không cần đọc BRAM)
            // ----------------------------------------------------------
            if (issue_err_beat_c) begin
                out_valid_q   <= 1'b1;
                out_id_q      <= active_id_q;
                out_payload_q <= {DECERR_RESP, {DATA_WIDTH{1'b0}}};
                out_last_q    <= (active_read_cnt_q ==
                    (act_expected - {{(BEAT_CNT_W-1){1'b0}}, 1'b1}));
                active_read_cnt_q <= active_read_cnt_q + {{(BEAT_CNT_W-1){1'b0}}, 1'b1};
            end

            // Activate head-of-ID transaction
            if (!active_q && !out_valid_q && !read_pending_q && choose_valid_c) begin
                active_q          <= 1'b1;
                active_tag_q      <= choose_tag_c;
                active_id_q       <= choose_id_c;
                active_read_cnt_q <= {BEAT_CNT_W{1'b0}};
            end

            if (out_fire_c) begin
                if (out_last_q) begin
                    active_q <= 1'b0;

                    // Xoá entry: bit-clear (không cần loop)
                    entry_valid_q[active_tag_q]    <= 1'b0;
                    entry_err_q[active_tag_q]      <= 1'b0;
                    entry_is_head_q[active_tag_q]  <= 1'b0;
                    entry_id_flat_q      [active_tag_q*ID_WIDTH   +: ID_WIDTH]   <= {ID_WIDTH{1'b0}};
                    entry_next_flat_q    [active_tag_q*ROB_TAG_W  +: ROB_TAG_W]  <= {ROB_TAG_W{1'b0}};
                    entry_expected_flat_q[active_tag_q*BEAT_CNT_W +: BEAT_CNT_W] <= {BEAT_CNT_W{1'b0}};
                    entry_recv_flat_q    [active_tag_q*BEAT_CNT_W +: BEAT_CNT_W] <= {BEAT_CNT_W{1'b0}};
                    active_read_cnt_q <= {BEAT_CNT_W{1'b0}};

                    // Promote next head
                    if (has_next_c)
                        entry_is_head_q[next_head_tag_c] <= 1'b1;
                    else if (alloc_same_id_c)
                        entry_is_head_q[free_tag_c] <= 1'b1;

                    if (!(alloc_fire_o && (alloc_id_i == active_id_q)))
                        idq_count_q[active_id_q] <=
                            idq_count_q[active_id_q] - {{(ID_CNT_W-1){1'b0}}, 1'b1};

                    rr_tag_q <= rob_tag_inc(active_tag_q);
                end
            end
        end // else reset
    end // always

    always @(posedge ACLK_i) begin
        if (ARESETn_i) begin
            // Ghi beat memory
            if (r_fire_c)
                beat_mem_q[rob_mem_idx(r_tag_i, rtag_recv)] <= {r_resp_i, r_data_i};

            // Đọc BRAM (synchronous read → 1-cycle latency)
            if (issue_data_read_c)
                beat_rd_q <= beat_mem_q[rob_mem_idx(active_tag_q, active_read_cnt_q)];
        end
    end

endmodule

module axi_read_reorder_buffer #(
    parameter SLV_AMT          = 4,
    parameter DATA_WIDTH       = 32,
    parameter ID_WIDTH         = 5,
    parameter LEN_WIDTH        = 8,
    parameter RESP_WIDTH       = 2,
    parameter ROB_DEPTH        = 8,
    parameter MAX_BURST_BEATS  = 64,

    // Explicit dependent widths for Verilog-2001 portability.
    parameter ROB_TAG_W        = 3,
    parameter BEAT_CNT_W       = 7,
    parameter ID_COUNT         = 32,
    parameter ID_PTR_W         = 3,
    parameter ID_CNT_W         = 4,
    parameter ROB_ID_WIDTH     = ROB_TAG_W + ID_WIDTH,
    parameter SLV_PTR_W        = 2,

    parameter [RESP_WIDTH-1:0] DECERR_RESP = {RESP_WIDTH{1'b1}}
)(
    input  wire                         ACLK_i,
    input  wire                         ARESETn_i,

    // AR allocation side.
    input  wire                         alloc_valid_i,
    output wire                         alloc_ready_o,
    input  wire [ID_WIDTH-1:0]          alloc_id_i,
    input  wire [LEN_WIDTH-1:0]         alloc_len_i,
    input  wire                         alloc_err_i,
    output wire [ROB_TAG_W-1:0]         alloc_tag_o,
    output wire                         alloc_fire_o,

    // R inputs from slave-arbitration blocks to this dispatcher/master.
    input  wire [ROB_ID_WIDTH*SLV_AMT-1:0] sa_RID_i,
    input  wire [DATA_WIDTH*SLV_AMT-1:0]   sa_RDATA_i,
    input  wire [RESP_WIDTH*SLV_AMT-1:0]   sa_RRESP_i,
    input  wire [SLV_AMT-1:0]              sa_RLAST_i,
    input  wire [SLV_AMT-1:0]              sa_RVALID_i,
    output reg  [SLV_AMT-1:0]              sa_RREADY_o,

    // R output to original master.
    output wire                         m_RVALID_o,
    input  wire                         m_RREADY_i,
    output wire [ID_WIDTH-1:0]          m_RID_o,
    output wire [DATA_WIDTH-1:0]        m_RDATA_o,
    output wire [RESP_WIDTH-1:0]        m_RRESP_o,
    output wire                         m_RLAST_o,

    // Status/debug.
    output wire                         rob_full_o,
    output wire                         id_order_full_o,
    output wire                         r_unexpected_tag_o,
    output wire                         r_overflow_o,
    output wire                         r_last_mismatch_o
);

    localparam MERGE_R_INFO_W = ROB_TAG_W + DATA_WIDTH + RESP_WIDTH + 1;

    wire [ROB_DEPTH-1:0]        tag_ready_vec;
    wire [ROB_DEPTH-1:0]        tag_valid_vec_unused;
    reg  [ROB_TAG_W-1:0]        slv_tag_c     [0:SLV_AMT-1];
    reg  [ID_WIDTH-1:0]         slv_orig_id_c [0:SLV_AMT-1];
    reg  [SLV_AMT-1:0]          slv_eligible_c;
    reg  [SLV_AMT-1:0]          slv_grant_c;
    reg  [SLV_PTR_W-1:0]        rr_slv_q;
    reg  [SLV_PTR_W-1:0]        grant_idx_c;
    reg                         grant_valid_c;

    wire                        core_r_valid;
    wire                        core_r_ready;
    wire [ROB_TAG_W-1:0]        core_r_tag;
    wire [DATA_WIDTH-1:0]       core_r_data;
    wire [RESP_WIDTH-1:0]       core_r_resp;
    wire                        core_r_last;
    wire                        core_r_fire;
    wire [MERGE_R_INFO_W-1:0]   merge_bwd_data;
    wire                        merge_bwd_valid;
    wire                        merge_bwd_ready;
    wire                        merge_bwd_fire;
    wire [MERGE_R_INFO_W-1:0]   merge_fwd_data;
    wire                        merge_fwd_valid;
    wire                        merge_fwd_ready;
    wire [ROB_TAG_W-1:0]        merge_fwd_tag;
    wire [DATA_WIDTH-1:0]       merge_fwd_rdata;
    wire [RESP_WIDTH-1:0]       merge_fwd_rresp;
    wire                        merge_fwd_rlast;

    integer s;
    integer scan_s_int;
    integer rid_base_int;

    function [SLV_PTR_W-1:0] slv_ptr_inc;
        input [SLV_PTR_W-1:0] ptr;
        begin
            if (ptr == (SLV_AMT - 1))
                slv_ptr_inc = {SLV_PTR_W{1'b0}};
            else
                slv_ptr_inc = ptr + {{(SLV_PTR_W-1){1'b0}}, 1'b1};
        end
    endfunction

    // Extract ROB tag and original ID from each slave-returned RID.
    // sa_RID_i layout for slave s:
    //   bits [ROB_ID_WIDTH*s +: ID_WIDTH]        = original_arid
    //   bits [ROB_ID_WIDTH*s+ID_WIDTH +: TAG_W]  = rob_tag
    always @(*) begin
        for (s = 0; s < SLV_AMT; s = s + 1) begin
            rid_base_int = ROB_ID_WIDTH * s;
            slv_orig_id_c[s] = sa_RID_i[rid_base_int +: ID_WIDTH];
            slv_tag_c[s]     = sa_RID_i[rid_base_int + ID_WIDTH +: ROB_TAG_W];
            slv_eligible_c[s] = sa_RVALID_i[s];
        end
    end

    // Round-robin merge of multiple slave R inputs.
    always @(*) begin
        slv_grant_c   = {SLV_AMT{1'b0}};
        grant_idx_c   = {SLV_PTR_W{1'b0}};
        grant_valid_c = 1'b0;

        for (s = 0; s < SLV_AMT; s = s + 1) begin
            scan_s_int = rr_slv_q + s;
            if (scan_s_int >= SLV_AMT)
                scan_s_int = scan_s_int - SLV_AMT;

            if (!grant_valid_c && slv_eligible_c[scan_s_int]) begin
                grant_valid_c = 1'b1;
                grant_idx_c   = scan_s_int[SLV_PTR_W-1:0];
                slv_grant_c[scan_s_int] = 1'b1;
            end
        end
    end

    assign merge_bwd_valid = grant_valid_c;
    assign merge_bwd_data  = {
        slv_tag_c[grant_idx_c],
        sa_RDATA_i[DATA_WIDTH*grant_idx_c +: DATA_WIDTH],
        sa_RRESP_i[RESP_WIDTH*grant_idx_c +: RESP_WIDTH],
        sa_RLAST_i[grant_idx_c]
    };
    assign merge_bwd_fire = merge_bwd_valid & merge_bwd_ready;

    skid_buffer #(
        .SBUF_TYPE(1),
        .DATA_WIDTH(MERGE_R_INFO_W)
    ) r_merge_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (merge_bwd_data),
        .bwd_valid_i(merge_bwd_valid),
        .fwd_ready_i(merge_fwd_ready),
        .fwd_data_o (merge_fwd_data),
        .bwd_ready_o(merge_bwd_ready),
        .fwd_valid_o(merge_fwd_valid)
    );

    assign {merge_fwd_tag, merge_fwd_rdata, merge_fwd_rresp, merge_fwd_rlast} = merge_fwd_data;
    assign core_r_valid = merge_fwd_valid;
    assign core_r_tag   = merge_fwd_tag;
    assign core_r_data  = merge_fwd_rdata;
    assign core_r_resp  = merge_fwd_rresp;
    assign core_r_last  = merge_fwd_rlast;
    assign core_r_fire  = core_r_valid & core_r_ready;
    assign merge_fwd_ready = core_r_ready;

    always @(*) begin
        sa_RREADY_o = {SLV_AMT{1'b0}};
        if (grant_valid_c) begin
            sa_RREADY_o[grant_idx_c] = merge_bwd_ready;
        end
    end

    always @(posedge ACLK_i or negedge ARESETn_i) begin
        if (!ARESETn_i) begin
            rr_slv_q <= {SLV_PTR_W{1'b0}};
        end else begin
            if (merge_bwd_fire) begin
                rr_slv_q <= slv_ptr_inc(grant_idx_c);
            end
        end
    end

    axi_read_reorder_buffer_core #(
        .DATA_WIDTH      (DATA_WIDTH),
        .ID_WIDTH        (ID_WIDTH),
        .LEN_WIDTH       (LEN_WIDTH),
        .RESP_WIDTH      (RESP_WIDTH),
        .ROB_DEPTH       (ROB_DEPTH),
        .MAX_BURST_BEATS (MAX_BURST_BEATS),
        .ROB_TAG_W       (ROB_TAG_W),
        .BEAT_CNT_W      (BEAT_CNT_W),
        .ID_COUNT        (ID_COUNT),
        .ID_PTR_W        (ID_PTR_W),
        .ID_CNT_W        (ID_CNT_W),
        .DECERR_RESP     (DECERR_RESP)
    ) u_core (
        .ACLK_i               (ACLK_i),
        .ARESETn_i            (ARESETn_i),

        .alloc_valid_i        (alloc_valid_i),
        .alloc_ready_o        (alloc_ready_o),
        .alloc_id_i           (alloc_id_i),
        .alloc_len_i          (alloc_len_i),
        .alloc_err_i          (alloc_err_i),
        .alloc_tag_o          (alloc_tag_o),
        .alloc_fire_o         (alloc_fire_o),

        .r_valid_i            (core_r_valid),
        .r_ready_o            (core_r_ready),
        .r_tag_i              (core_r_tag),
        .r_data_i             (core_r_data),
        .r_resp_i             (core_r_resp),
        .r_last_i             (core_r_last),
        .resp_tag_ready_vec_o (tag_ready_vec),
        .resp_tag_valid_vec_o (tag_valid_vec_unused),

        .m_RVALID_o           (m_RVALID_o),
        .m_RREADY_i           (m_RREADY_i),
        .m_RID_o              (m_RID_o),
        .m_RDATA_o            (m_RDATA_o),
        .m_RRESP_o            (m_RRESP_o),
        .m_RLAST_o            (m_RLAST_o),

        .rob_full_o           (rob_full_o),
        .id_order_full_o      (id_order_full_o),
        .r_unexpected_tag_o   (r_unexpected_tag_o),
        .r_overflow_o         (r_overflow_o),
        .r_last_mismatch_o    (r_last_mismatch_o)
    );

endmodule
