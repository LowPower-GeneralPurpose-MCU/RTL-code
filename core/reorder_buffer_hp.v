`timescale 1ns/1ps

module reorder_buffer_hp #(
    parameter ROB_DEPTH  = 64,
    parameter DATA_W     = 64,
    parameter ARCH_REGS  = 32,
    parameter PC_W       = 64,
    parameter N_CDB      = 4,
    parameter N_DISPATCH = 4,
    parameter N_COMMIT   = 4,
    parameter N_BANKS    = 4,
    parameter XCODE_W    = 8,

    // Derived — không override
    parameter ROB_TAG_W = $clog2(ROB_DEPTH),
    parameter REG_AW    = $clog2(ARCH_REGS),
    parameter PTR_W     = ROB_TAG_W + 1
)(
    input  wire clk,
    input  wire rst_n,

    // ----------------------------------------------------------
    // [Dispatch] N-way superscalar
    // Precondition: dispatch_valid_i phải là contiguous từ bit 0.
    // Tức là nếu dispatch_valid_i[k]=0 thì dispatch_valid_i[k+1..N-1] phải =0.
    // ----------------------------------------------------------
    input  wire [N_DISPATCH-1:0]           dispatch_valid_i,
    input  wire [N_DISPATCH*PC_W-1:0]      dispatch_pc_i,
    input  wire [N_DISPATCH*REG_AW-1:0]    dispatch_rd_i,
    input  wire [N_DISPATCH-1:0]           dispatch_rd_valid_i,
    input  wire [N_DISPATCH*3-1:0]         dispatch_type_i,
    output wire [N_DISPATCH-1:0]           dispatch_ready_o,
    output wire [N_DISPATCH*ROB_TAG_W-1:0] dispatch_rob_tag_o,

    // [CDB Writeback]
    input  wire [N_CDB-1:0]               cdb_valid_i,
    input  wire [N_CDB*ROB_TAG_W-1:0]     cdb_tag_i,
    input  wire [N_CDB*DATA_W-1:0]        cdb_data_i,
    input  wire [N_CDB*XCODE_W-1:0]       cdb_exc_i,

    // [Commit] N-way superscalar (registered outputs, 1-cycle latency)
    output wire [N_COMMIT-1:0]             commit_valid_o,
    output wire [N_COMMIT*REG_AW-1:0]      commit_rd_o,
    output wire [N_COMMIT-1:0]             commit_rd_valid_o,
    output wire [N_COMMIT*DATA_W-1:0]      commit_data_o,
    output wire [N_COMMIT*PC_W-1:0]        commit_pc_o,
    output wire [N_COMMIT*3-1:0]           commit_type_o,
    output wire [N_COMMIT-1:0]             commit_is_store_o,

    // [Bypass/Forwarding] 2 source operand ports
    input  wire [ROB_TAG_W-1:0]            fwd_tag_a_i,
    input  wire                            fwd_valid_a_i,
    output wire [DATA_W-1:0]               fwd_data_a_o,
    output wire                            fwd_hit_a_o,

    input  wire [ROB_TAG_W-1:0]            fwd_tag_b_i,
    input  wire                            fwd_valid_b_i,
    output wire [DATA_W-1:0]               fwd_data_b_o,
    output wire                            fwd_hit_b_o,

    // [Exception / Flush]
    input  wire                            br_mispredict_i,
    input  wire [PC_W-1:0]                 br_correct_pc_i,
    output wire                            flush_o,       // registered
    output wire [PC_W-1:0]                 flush_pc_o,
    output wire                            exc_valid_o,
    output wire [XCODE_W-1:0]             exc_code_o,
    output wire                            flush_early_o,    // combinational (0-latency)
    output wire [PC_W-1:0]                flush_pc_early_o,

    // [Status]
    output wire                            rob_full_o,
    output wire                            rob_empty_o,
    output wire [PTR_W-1:0]               rob_count_o,
    output wire [PTR_W-1:0]               rob_free_slots_o,

    // [Wakeup] bitvector cho OOO scheduler
    output wire [ROB_DEPTH-1:0]            wakeup_vec_o
);


    localparam [2:0] TYPE_ALU    = 3'b000;
    localparam [2:0] TYPE_LOAD   = 3'b001;
    localparam [2:0] TYPE_STORE  = 3'b010;
    localparam [2:0] TYPE_BRANCH = 3'b011;
    localparam [2:0] TYPE_FP     = 3'b100;

    (* ram_style = "distributed" *) reg [PC_W-1:0]    pc_r       [0:ROB_DEPTH-1];
    (* ram_style = "distributed" *) reg [REG_AW-1:0]  rd_r       [0:ROB_DEPTH-1];
    (* ram_style = "distributed" *) reg               rd_valid_r [0:ROB_DEPTH-1];
    (* ram_style = "distributed" *) reg [2:0]         type_r     [0:ROB_DEPTH-1];
    (* ram_style = "distributed" *) reg [XCODE_W-1:0] exc_code_r [0:ROB_DEPTH-1];
    reg [ROB_DEPTH-1:0] exc_valid_r;

    integer ctrl_disp_idx_v;  // dispatch loop index (replaces 'automatic integer idx')
    integer ctrl_cdb_tag_v;   // CDB loop tag (replaces 'automatic integer tag')
    integer ctrl_init_v;      // reset init loop
    integer d_v, c_v;         // loop variables

    wire                    full_w, empty_w;
    wire [ROB_TAG_W-1:0]    head_idx_w, tail_idx_w;
    wire [PTR_W-1:0]        count_w, adjusted_free_w;

    wire [$clog2(N_COMMIT+1)-1:0] commit_count_w;
    wire [N_COMMIT-1:0]           commit_ready_w;
    wire [N_COMMIT-1:0]           commit_ready_next_w;
    wire                          flush_block_w;

    wire [N_DISPATCH-1:0] dispatch_contiguous_en_w;
    genvar pca;
    generate
        assign dispatch_contiguous_en_w[0] = dispatch_valid_i[0];
        for (pca = 1; pca < N_DISPATCH; pca = pca + 1) begin : contig_and
            assign dispatch_contiguous_en_w[pca] = dispatch_valid_i[pca] &
                                                    dispatch_contiguous_en_w[pca-1];
        end
    endgenerate

    reg [$clog2(N_DISPATCH+1)-1:0] dispatch_count_w;
    integer ddc_v; // loop variable for dispatch count
    always @(*) begin
        dispatch_count_w = {($clog2(N_DISPATCH+1)){1'b0}};
        for (ddc_v = 0; ddc_v < N_DISPATCH; ddc_v = ddc_v + 1) begin
            // [Issue 8] Use adjusted_free_w (accounts for same-cycle commits)
            if (dispatch_contiguous_en_w[ddc_v] &&
                (adjusted_free_w > ddc_v[$clog2(N_DISPATCH+1)-1:0]) &&
                !flush_block_w)
                dispatch_count_w = dispatch_count_w + 1;
        end
    end

    rob_ptr_manager_superscalar #(
        .DEPTH      (ROB_DEPTH),
        .PTRW       (PTR_W),
        .N_DISPATCH (N_DISPATCH),
        .N_COMMIT   (N_COMMIT)
    ) u_ptr_mgr (
        .clk              (clk),
        .rst_n            (rst_n),
        .dispatch_count_i (dispatch_count_w),
        .commit_count_i   (commit_count_w),
        .flush_i          (flush_block_w),
        .full_o           (full_w),
        .empty_o          (empty_w),
        .head_o           (head_idx_w),
        .tail_o           (tail_idx_w),
        .count_o          (count_w),
        .adjusted_free_o  (adjusted_free_w)  // [Issue 8] new port
    );

    wire [ROB_DEPTH-1:0] state_empty_w, state_issued_w, state_writeback_w;
    wire                 query_a_wb_w, query_b_wb_w;

    wire [N_DISPATCH*ROB_TAG_W-1:0] dispatch_tag_vec;
    wire [N_COMMIT*ROB_TAG_W-1:0]   commit_tag_vec;

    genvar dti, cti;
    generate
        for (dti = 0; dti < N_DISPATCH; dti = dti + 1) begin : dtag_gen
            assign dispatch_tag_vec[dti*ROB_TAG_W +: ROB_TAG_W] =
                tail_idx_w + dti[ROB_TAG_W-1:0];
        end
        for (cti = 0; cti < N_COMMIT; cti = cti + 1) begin : ctag_gen
            assign commit_tag_vec[cti*ROB_TAG_W +: ROB_TAG_W] =
                head_idx_w + cti[ROB_TAG_W-1:0];
        end
    endgenerate

    // Dispatch enable per slot (contiguous + free-slot gated)
    wire [N_DISPATCH-1:0] dispatch_en_w;
    genvar dei;
    generate
        for (dei = 0; dei < N_DISPATCH; dei = dei + 1) begin : den_gen
            assign dispatch_en_w[dei] = dispatch_contiguous_en_w[dei] &&
                                        (adjusted_free_w > dei[PTR_W-1:0]) &&
                                        !flush_block_w;
        end
    endgenerate

    rob_state_onehot #(
        .ROB_DEPTH  (ROB_DEPTH),
        .N_DISPATCH (N_DISPATCH),
        .N_CDB      (N_CDB),
        .N_COMMIT   (N_COMMIT),
        .ROB_TAG_W  (ROB_TAG_W)
    ) u_state (
        .clk                 (clk),
        .rst_n               (rst_n),
        .flush_i             (flush_block_w),
        .dispatch_en_i       (dispatch_en_w),
        .dispatch_tag_i      (dispatch_tag_vec),
        .cdb_valid_i         (cdb_valid_i),
        .cdb_tag_i           (cdb_tag_i),
        .commit_en_i         (commit_ready_w),
        .commit_tag_i        (commit_tag_vec),
        .state_empty_o       (state_empty_w),
        .state_issued_o      (state_issued_w),
        .state_writeback_o   (state_writeback_w),
        .query_tag_a_i       (fwd_tag_a_i),
        .query_tag_b_i       (fwd_tag_b_i),
        .query_a_writeback_o (query_a_wb_w),
        .query_b_writeback_o (query_b_wb_w)
    );

    localparam N_DATA_READ = 2 + N_COMMIT; // 2 fwd + N_COMMIT commit ports
    wire [N_CDB-1:0]                   data_wr_en;
    wire [N_BANKS-1:0]                 wr_conflict_w;

    genvar cwi;
    generate
        for (cwi = 0; cwi < N_CDB; cwi = cwi + 1) begin : cdb_wr_gen
            wire [ROB_TAG_W-1:0] cdb_tag_s = cdb_tag_i[cwi*ROB_TAG_W +: ROB_TAG_W];
            assign data_wr_en[cwi] = cdb_valid_i[cwi] && state_issued_w[cdb_tag_s] && !flush_block_w;
        end
    endgenerate

    wire [N_DATA_READ-1:0]           data_rd_en;
    wire [N_DATA_READ*ROB_TAG_W-1:0] data_rd_addr;
    wire [N_DATA_READ*DATA_W-1:0]    data_rd_data;
    wire [N_DATA_READ-1:0]           data_rd_valid;

    assign data_rd_en  [0]                  = fwd_valid_a_i;
    assign data_rd_en  [1]                  = fwd_valid_b_i;
    assign data_rd_addr[0*ROB_TAG_W +: ROB_TAG_W] = fwd_tag_a_i;
    assign data_rd_addr[1*ROB_TAG_W +: ROB_TAG_W] = fwd_tag_b_i;

    genvar cri;
    generate
        for (cri = 0; cri < N_COMMIT; cri = cri + 1) begin : crd_gen
            assign data_rd_en  [2+cri]                           = commit_ready_w[cri];
            assign data_rd_addr[(2+cri)*ROB_TAG_W +: ROB_TAG_W] =
                commit_tag_vec[cri*ROB_TAG_W +: ROB_TAG_W];
        end
    endgenerate

    rob_banked_array #(
        .ROB_DEPTH      (ROB_DEPTH),
        .DATA_W         (DATA_W),
        .N_BANKS        (N_BANKS),
        .N_READ         (N_DATA_READ),
        .N_WRITE        (N_CDB),
        .PIPELINE_OUTPUT(1),
        .ROB_TAG_W      (ROB_TAG_W)
    ) u_data_array (
        .clk          (clk),
        .rst_n        (rst_n),
        .flush_i      (flush_block_w),
        .wr_en_i      (data_wr_en),
        .wr_addr_i    (cdb_tag_i),
        .wr_data_i    (cdb_data_i),
        .rd_en_i      (data_rd_en),
        .rd_addr_i    (data_rd_addr),
        .rd_data_o    (data_rd_data),
        .rd_valid_o   (data_rd_valid),
        .wr_conflict_o(wr_conflict_w)
    );

    wire [N_COMMIT*2-1:0] scanner_state_in;
    wire [N_COMMIT-1:0]   scanner_exc_in;

    genvar sci;
    generate
        for (sci = 0; sci < N_COMMIT; sci = sci + 1) begin : scan_in_gen
            wire [ROB_TAG_W-1:0] chk_idx = head_idx_w + sci[ROB_TAG_W-1:0];
            assign scanner_state_in[sci*2 +: 2] =
                state_writeback_w[chk_idx] ? 2'b10 :
                state_issued_w   [chk_idx] ? 2'b01 : 2'b00;
            assign scanner_exc_in[sci] = exc_valid_r[chk_idx];
        end
    endgenerate

    rob_commit_scanner #(
        .ROB_DEPTH (ROB_DEPTH),
        .N_COMMIT  (N_COMMIT),
        .ROB_TAG_W (ROB_TAG_W),
        .PTR_W     (PTR_W)
    ) u_commit_scanner (
        .head_idx_i          (head_idx_w),
        .count_i             (count_w),
        .empty_i             (empty_w),
        .flush_i             (flush_o),
        .state_i             (scanner_state_in),
        .exc_valid_i         (scanner_exc_in),
        .commit_ready_o      (commit_ready_w),
        .commit_count_o      (commit_count_w),
        .commit_ready_next_o (commit_ready_next_w)
    );

    exception_handler #(
        .PC_W    (PC_W),
        .XCODE_W (XCODE_W)
    ) u_exc_handler (
        .clk                 (clk),
        .rst_n               (rst_n),
        .head_exc_valid_i    (exc_valid_r[head_idx_w]),
        .head_exc_code_i     (exc_code_r [head_idx_w]),
        .head_pc_i           (pc_r       [head_idx_w]),
        .head_commit_ready_i (commit_ready_w[0]),
        .br_mispredict_i     (br_mispredict_i),
        .br_correct_pc_i     (br_correct_pc_i),
        .flush_o             (flush_o),
        .flush_pc_o          (flush_pc_o),
        .exc_valid_o         (exc_valid_o),
        .exc_code_o          (exc_code_o),
        .flush_early_o       (flush_early_o),
        .flush_pc_early_o    (flush_pc_early_o)
    );

    wire [DATA_W-1:0] cdb_fwd_data_a, cdb_fwd_data_b;
    wire              cdb_fwd_hit_a,  cdb_fwd_hit_b;

    assign flush_block_w = flush_o | flush_early_o;

    cdb_bypass_mux #(.DATA_W(DATA_W), .TAG_W(ROB_TAG_W), .N_CDB(N_CDB), .REGISTERED_OUTPUT(0))
    u_fwd_a (
        .clk(clk), .rst_n(rst_n),
        .src_tag_i(fwd_tag_a_i), .src_valid_i(fwd_valid_a_i),
        .cdb_tag_i(cdb_tag_i), .cdb_data_i(cdb_data_i), .cdb_valid_i(cdb_valid_i),
        .fwd_data_o(cdb_fwd_data_a), .fwd_hit_o(cdb_fwd_hit_a)
    );

    cdb_bypass_mux #(.DATA_W(DATA_W), .TAG_W(ROB_TAG_W), .N_CDB(N_CDB), .REGISTERED_OUTPUT(0))
    u_fwd_b (
        .clk(clk), .rst_n(rst_n),
        .src_tag_i(fwd_tag_b_i), .src_valid_i(fwd_valid_b_i),
        .cdb_tag_i(cdb_tag_i), .cdb_data_i(cdb_data_i), .cdb_valid_i(cdb_valid_i),
        .fwd_data_o(cdb_fwd_data_b), .fwd_hit_o(cdb_fwd_hit_b)
    );

    reg rob_fwd_hit_a_q;
    reg rob_fwd_hit_b_q;

    // The ROB data array has registered outputs. Delay the ROB-array hit bit
    // by the same cycle so hit and data refer to the same request. CDB bypass
    // stays combinational for same-cycle writeback forwarding.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_block_w) begin
            rob_fwd_hit_a_q <= 1'b0;
            rob_fwd_hit_b_q <= 1'b0;
        end else begin
            rob_fwd_hit_a_q <= fwd_valid_a_i && query_a_wb_w;
            rob_fwd_hit_b_q <= fwd_valid_b_i && query_b_wb_w;
        end
    end

    assign fwd_hit_a_o  = cdb_fwd_hit_a | (rob_fwd_hit_a_q & data_rd_valid[0]);
    assign fwd_data_a_o = cdb_fwd_hit_a ? cdb_fwd_data_a :
                          (rob_fwd_hit_a_q & data_rd_valid[0]) ?
                              data_rd_data[0*DATA_W +: DATA_W] :
                              {DATA_W{1'b0}};

    assign fwd_hit_b_o  = cdb_fwd_hit_b | (rob_fwd_hit_b_q & data_rd_valid[1]);
    assign fwd_data_b_o = cdb_fwd_hit_b ? cdb_fwd_data_b :
                          (rob_fwd_hit_b_q & data_rd_valid[1]) ?
                              data_rd_data[1*DATA_W +: DATA_W] :
                              {DATA_W{1'b0}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            exc_valid_r <= {ROB_DEPTH{1'b0}};
            for (ctrl_init_v = 0; ctrl_init_v < ROB_DEPTH; ctrl_init_v = ctrl_init_v + 1) begin
                pc_r      [ctrl_init_v] <= {PC_W{1'b0}};
                rd_r      [ctrl_init_v] <= {REG_AW{1'b0}};
                rd_valid_r[ctrl_init_v] <= 1'b0;
                type_r    [ctrl_init_v] <= 3'b000;
                exc_code_r[ctrl_init_v] <= {XCODE_W{1'b0}};
            end
        end else if (flush_block_w) begin
            exc_valid_r <= {ROB_DEPTH{1'b0}};
        end else begin
            // Dispatch writes
            for (d_v = 0; d_v < N_DISPATCH; d_v = d_v + 1) begin
                if (dispatch_en_w[d_v]) begin
                    ctrl_disp_idx_v = (tail_idx_w + d_v) & (ROB_DEPTH - 1);
                    pc_r      [ctrl_disp_idx_v] <= dispatch_pc_i      [d_v*PC_W   +: PC_W];
                    rd_r      [ctrl_disp_idx_v] <= dispatch_rd_i      [d_v*REG_AW +: REG_AW];
                    rd_valid_r[ctrl_disp_idx_v] <= dispatch_rd_valid_i[d_v];
                    type_r    [ctrl_disp_idx_v] <= dispatch_type_i    [d_v*3      +: 3];
                    exc_valid_r[ctrl_disp_idx_v] <= 1'b0;
                    exc_code_r [ctrl_disp_idx_v] <= {XCODE_W{1'b0}};
                end
            end
            // CDB exception capture
            for (c_v = 0; c_v < N_CDB; c_v = c_v + 1) begin
                if (cdb_valid_i[c_v]) begin
                    ctrl_cdb_tag_v = cdb_tag_i[c_v*ROB_TAG_W +: ROB_TAG_W];
                    if (state_issued_w[ctrl_cdb_tag_v]) begin
                        exc_valid_r[ctrl_cdb_tag_v] <= |cdb_exc_i[c_v*XCODE_W +: XCODE_W];
                        exc_code_r [ctrl_cdb_tag_v] <= cdb_exc_i[c_v*XCODE_W +: XCODE_W];
                    end
                end
            end
        end
    end

    genvar di_o;
    generate
        for (di_o = 0; di_o < N_DISPATCH; di_o = di_o + 1) begin : dout_gen
            assign dispatch_ready_o[di_o] =
                !flush_block_w && (adjusted_free_w > di_o[PTR_W-1:0]);
            // [Issue 4] Remove % → natural bit truncation on ROB_TAG_W-bit target
            assign dispatch_rob_tag_o[di_o*ROB_TAG_W +: ROB_TAG_W] =
                tail_idx_w + di_o[ROB_TAG_W-1:0];
        end
    endgenerate

    genvar mi_o;
    generate
        for (mi_o = 0; mi_o < N_COMMIT; mi_o = mi_o + 1) begin : commit_reg_gen
            reg               commit_valid_r;
            reg [REG_AW-1:0]  commit_rd_r;
            reg               commit_rd_valid_r;
            reg [PC_W-1:0]    commit_pc_r;
            reg [2:0]         commit_type_r;
            reg               commit_is_store_r;

            // [Issue 4] no % → natural wrap
            wire [ROB_TAG_W-1:0] cidx = head_idx_w + mi_o[ROB_TAG_W-1:0];

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n || flush_block_w) begin
                    commit_valid_r    <= 1'b0;
                    commit_rd_valid_r <= 1'b0;
                    commit_is_store_r <= 1'b0;
                    commit_rd_r       <= {REG_AW{1'b0}};
                    commit_pc_r       <= {PC_W{1'b0}};
                    commit_type_r     <= 3'b000;
                end else begin
                    commit_valid_r    <= commit_ready_w[mi_o] && !exc_valid_r[cidx];
                    commit_rd_r       <= rd_r      [cidx];
                    commit_rd_valid_r <= commit_ready_w[mi_o] && !exc_valid_r[cidx] && rd_valid_r[cidx];
                    commit_pc_r       <= pc_r      [cidx];
                    commit_type_r     <= type_r    [cidx];
                    commit_is_store_r <= commit_ready_w[mi_o] && !exc_valid_r[cidx] && (type_r[cidx] == TYPE_STORE);
                end
            end

            assign commit_valid_o   [mi_o]                   = commit_valid_r;
            assign commit_rd_o      [mi_o*REG_AW +: REG_AW]  = commit_rd_r;
            assign commit_rd_valid_o[mi_o]                   = commit_rd_valid_r;
            assign commit_pc_o      [mi_o*PC_W   +: PC_W]    = commit_pc_r;
            assign commit_type_o    [mi_o*3      +: 3]        = commit_type_r;
            assign commit_is_store_o[mi_o]                   = commit_is_store_r;
            assign commit_data_o    [mi_o*DATA_W +: DATA_W]  =
                data_rd_data[(2+mi_o)*DATA_W +: DATA_W];
        end
    endgenerate

    assign rob_full_o       = full_w;
    assign rob_empty_o      = empty_w;
    assign rob_count_o      = count_w;
    assign rob_free_slots_o = adjusted_free_w; // expose adjusted value
    assign wakeup_vec_o     = state_writeback_w;

    genvar pca2;
    generate
        for (pca2 = 0; pca2 < N_DISPATCH-1; pca2 = pca2 + 1) begin : contig_assert
            always @(posedge clk) begin
                if (rst_n && !flush_block_w) begin
                    if (!dispatch_valid_i[pca2] && dispatch_valid_i[pca2+1]) begin
                        // $error("[%0t] reorder_buffer_hp: dispatch_valid VIOLATION: "
                        //        "slot %0d=0 but slot %0d=1. Slots must be contiguous from 0.",
                        //        $realtime, pca2, pca2+1);
                    end
                end
            end
        end
    endgenerate


endmodule
