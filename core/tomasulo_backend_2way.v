`timescale 1ns/1ps

// ============================================================================
// Tomasulo 2-way backend building blocks.
//
// This file is intentionally self-contained so the classic pipeline can migrate
// toward a real OoO backend one boundary at a time:
//   decode packet -> rename/RAT -> issue queue -> execute -> CDB -> ROB commit
// ============================================================================

module tomasulo_first_onehot #(
    parameter WIDTH = 16
)(
    input  wire [WIDTH-1:0] req_i,
    output wire [WIDTH-1:0] grant_o,
    output wire             valid_o
);
    wire [WIDTH:0] seen_w;

    assign seen_w[0] = 1'b0;
    genvar i;
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : first_gen
            assign grant_o[i] = req_i[i] & ~seen_w[i];
            assign seen_w[i+1] = seen_w[i] | req_i[i];
        end
    endgenerate

    assign valid_o = seen_w[WIDTH];
endmodule


module tomasulo_onehot_mux #(
    parameter WIDTH  = 16,
    parameter DATA_W = 32
)(
    input  wire [WIDTH-1:0]        sel_i,
    input  wire [WIDTH*DATA_W-1:0] data_i,
    output wire [DATA_W-1:0]       data_o
);
    localparam GROUP_SIZE = (WIDTH < 4) ? WIDTH : 4;
    localparam GROUPS     = (WIDTH + GROUP_SIZE - 1) / GROUP_SIZE;

    wire [GROUPS*DATA_W-1:0] group_data_w;

    genvar gi, gj;
    generate
        for (gi = 0; gi < GROUPS; gi = gi + 1) begin : mux_group_gen
            wire [DATA_W-1:0] group_partial_w [0:GROUP_SIZE];
            assign group_partial_w[0] = {DATA_W{1'b0}};

            for (gj = 0; gj < GROUP_SIZE; gj = gj + 1) begin : lane_gen
                localparam LANE_IDX = gi*GROUP_SIZE + gj;
                if (LANE_IDX < WIDTH) begin : valid_lane
                    assign group_partial_w[gj+1] =
                        group_partial_w[gj] |
                        ({DATA_W{sel_i[LANE_IDX]}} &
                         data_i[LANE_IDX*DATA_W +: DATA_W]);
                end else begin : empty_lane
                    assign group_partial_w[gj+1] = group_partial_w[gj];
                end
            end

            assign group_data_w[gi*DATA_W +: DATA_W] =
                group_partial_w[GROUP_SIZE];
        end
    endgenerate

    wire [DATA_W-1:0] group_reduce_w [0:GROUPS];
    assign group_reduce_w[0] = {DATA_W{1'b0}};

    genvar rg;
    generate
        for (rg = 0; rg < GROUPS; rg = rg + 1) begin : group_reduce_gen
            assign group_reduce_w[rg+1] =
                group_reduce_w[rg] | group_data_w[rg*DATA_W +: DATA_W];
        end
    endgenerate

    assign data_o = group_reduce_w[GROUPS];
endmodule


module tomasulo_phys_regfile #(
    parameter PHYS_REGS = 64,
    parameter DATA_W    = 32,
    parameter N_READ    = 4,
    parameter N_WRITE   = 2,
    parameter PHYS_AW   = $clog2(PHYS_REGS)
)(
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [N_READ*PHYS_AW-1:0]    rd_addr_i,
    output wire [N_READ*DATA_W-1:0]     rd_data_o,

    input  wire [N_WRITE-1:0]           wr_en_i,
    input  wire [N_WRITE*PHYS_AW-1:0]   wr_addr_i,
    input  wire [N_WRITE*DATA_W-1:0]    wr_data_i
);
    (* ram_style = "distributed" *) reg [DATA_W-1:0] preg [0:PHYS_REGS-1];

    integer i;
    integer w;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < PHYS_REGS; i = i + 1)
                preg[i] <= {DATA_W{1'b0}};
            preg[2] <= 32'h8001_0000;
        end else begin
            for (w = 0; w < N_WRITE; w = w + 1) begin
                if (wr_en_i[w] &&
                    (wr_addr_i[w*PHYS_AW +: PHYS_AW] != {PHYS_AW{1'b0}}))
                    preg[wr_addr_i[w*PHYS_AW +: PHYS_AW]] <=
                        wr_data_i[w*DATA_W +: DATA_W];
            end
        end
    end

    genvar r;
    generate
        for (r = 0; r < N_READ; r = r + 1) begin : prf_read_gen
            wire [PHYS_AW-1:0] raddr = rd_addr_i[r*PHYS_AW +: PHYS_AW];
            assign rd_data_o[r*DATA_W +: DATA_W] =
                (raddr == {PHYS_AW{1'b0}}) ? {DATA_W{1'b0}} : preg[raddr];
        end
    endgenerate
endmodule


module tomasulo_free_list #(
    parameter PHYS_REGS = 64,
    parameter ARCH_REGS = 32,
    parameter N_ALLOC   = 2,
    parameter N_FREE    = 2,
    parameter PHYS_AW   = $clog2(PHYS_REGS),
    parameter PTR_W     = PHYS_AW + 1
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         flush_i,

    input  wire [N_ALLOC-1:0]           alloc_req_i,
    output wire [N_ALLOC-1:0]           alloc_ready_o,
    output wire [N_ALLOC*PHYS_AW-1:0]   alloc_phys_o,

    input  wire [N_FREE-1:0]            free_valid_i,
    input  wire [N_FREE*PHYS_AW-1:0]    free_phys_i,
    output wire [PTR_W-1:0]             free_count_o
);
    reg [PHYS_AW-1:0] fifo [0:PHYS_REGS-1];
    reg [PTR_W-1:0] head_r;
    reg [PTR_W-1:0] tail_r;
    reg [PTR_W-1:0] count_r;

    integer init_i;
    integer a_i;
    integer f_i;
    integer alloc_count_v;
    integer free_count_v;
    integer free_pos_v;

    genvar a;
    generate
        for (a = 0; a < N_ALLOC; a = a + 1) begin : alloc_out_gen
            assign alloc_ready_o[a] = (count_r > a[PTR_W-1:0]);
            assign alloc_phys_o[a*PHYS_AW +: PHYS_AW] =
                fifo[(head_r[PHYS_AW-1:0] + a[PHYS_AW-1:0]) & {PHYS_AW{1'b1}}];
        end
    endgenerate

    always @(*) begin
        alloc_count_v = 0;
        for (a_i = 0; a_i < N_ALLOC; a_i = a_i + 1)
            if (alloc_req_i[a_i] && alloc_ready_o[a_i])
                alloc_count_v = alloc_count_v + 1;

        free_count_v = 0;
        for (f_i = 0; f_i < N_FREE; f_i = f_i + 1)
            if (free_valid_i[f_i] &&
                (free_phys_i[f_i*PHYS_AW +: PHYS_AW] != {PHYS_AW{1'b0}}))
                free_count_v = free_count_v + 1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            head_r  <= {PTR_W{1'b0}};
            tail_r  <= (PHYS_REGS - ARCH_REGS);
            count_r <= (PHYS_REGS - ARCH_REGS);
            for (init_i = 0; init_i < PHYS_REGS; init_i = init_i + 1)
                fifo[init_i] <= (init_i + ARCH_REGS) & {PHYS_AW{1'b1}};
        end else begin
            head_r  <= head_r + alloc_count_v[PTR_W-1:0];
            tail_r  <= tail_r + free_count_v[PTR_W-1:0];
            count_r <= count_r - alloc_count_v[PTR_W-1:0] + free_count_v[PTR_W-1:0];

            free_pos_v = 0;
            for (f_i = 0; f_i < N_FREE; f_i = f_i + 1) begin
                if (free_valid_i[f_i] &&
                    (free_phys_i[f_i*PHYS_AW +: PHYS_AW] != {PHYS_AW{1'b0}})) begin
                    fifo[(tail_r[PHYS_AW-1:0] + free_pos_v[PHYS_AW-1:0]) & {PHYS_AW{1'b1}}]
                        <= free_phys_i[f_i*PHYS_AW +: PHYS_AW];
                    free_pos_v = free_pos_v + 1;
                end
            end
        end
    end

    assign free_count_o = count_r;
endmodule


module tomasulo_rat #(
    parameter ARCH_REGS  = 32,
    parameter PHYS_REGS  = 64,
    parameter ROB_DEPTH  = 64,
    parameter CHECKPOINTS = 4,
    parameter N_DISPATCH = 2,
    parameter N_CDB      = 2,
    parameter N_ARCH_WB  = 2,
    parameter ARCH_AW    = $clog2(ARCH_REGS),
    parameter PHYS_AW    = $clog2(PHYS_REGS),
    parameter ROB_TAG_W  = $clog2(ROB_DEPTH),
    parameter CP_AW      = $clog2(CHECKPOINTS)
)(
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           flush_i,

    input  wire [N_DISPATCH-1:0]          dispatch_fire_i,
    input  wire [N_DISPATCH*ARCH_AW-1:0]  dispatch_rs1_i,
    input  wire [N_DISPATCH*ARCH_AW-1:0]  dispatch_rs2_i,
    input  wire [N_DISPATCH*ARCH_AW-1:0]  dispatch_rd_i,
    input  wire [N_DISPATCH-1:0]          dispatch_rd_valid_i,
    input  wire [N_DISPATCH*PHYS_AW-1:0]  dispatch_alloc_phys_i,

    output reg  [N_DISPATCH*PHYS_AW-1:0]  src1_phys_o,
    output reg  [N_DISPATCH*PHYS_AW-1:0]  src2_phys_o,
    output reg  [N_DISPATCH-1:0]          src1_ready_o,
    output reg  [N_DISPATCH-1:0]          src2_ready_o,
    output reg  [N_DISPATCH*PHYS_AW-1:0]  old_phys_o,

    input  wire [N_CDB-1:0]               cdb_valid_i,
    input  wire [N_CDB*PHYS_AW-1:0]       cdb_phys_i,

    input  wire [N_ARCH_WB-1:0]            arch_wb_valid_i,
    input  wire [N_ARCH_WB*ARCH_AW-1:0]    arch_wb_rd_i,
    output reg  [N_ARCH_WB*PHYS_AW-1:0]    arch_wb_phys_o,

    input  wire                            checkpoint_valid_i,
    input  wire [ROB_TAG_W-1:0]            checkpoint_rob_tag_i,
    output wire                            checkpoint_ready_o,
    input  wire                            checkpoint_release_valid_i,
    input  wire [ROB_TAG_W-1:0]            checkpoint_release_rob_tag_i,
    input  wire                            restore_valid_i,
    input  wire [ROB_TAG_W-1:0]            restore_rob_tag_i
);
    reg [PHYS_AW-1:0] map_r [0:ARCH_REGS-1];
    reg [PHYS_REGS-1:0] ready_r;
    reg cp_valid_r [0:CHECKPOINTS-1];
    reg [ROB_TAG_W-1:0] cp_rob_tag_r [0:CHECKPOINTS-1];
    reg [PHYS_AW-1:0] cp_map_r [0:CHECKPOINTS-1][0:ARCH_REGS-1];
    reg [PHYS_REGS-1:0] cp_ready_r [0:CHECKPOINTS-1];

    integer i;
    integer d;
    integer c;
    integer wb;
    integer cp;
    integer ar;

    reg [ARCH_AW-1:0] rs1_v;
    reg [ARCH_AW-1:0] rs2_v;
    reg [ARCH_AW-1:0] rd_v;
    reg [PHYS_AW-1:0] alloc_v;
    reg [CP_AW-1:0] checkpoint_alloc_idx_v;
    reg checkpoint_ready_v;
    reg [CP_AW-1:0] restore_idx_v;
    reg restore_hit_v;

    always @(*) begin
        for (d = 0; d < N_DISPATCH; d = d + 1) begin
            rs1_v = dispatch_rs1_i[d*ARCH_AW +: ARCH_AW];
            rs2_v = dispatch_rs2_i[d*ARCH_AW +: ARCH_AW];
            rd_v  = dispatch_rd_i [d*ARCH_AW +: ARCH_AW];

            src1_phys_o [d*PHYS_AW +: PHYS_AW] = map_r[rs1_v];
            src2_phys_o [d*PHYS_AW +: PHYS_AW] = map_r[rs2_v];
            old_phys_o  [d*PHYS_AW +: PHYS_AW] = map_r[rd_v];
            src1_ready_o[d] = ready_r[map_r[rs1_v]];
            src2_ready_o[d] = ready_r[map_r[rs2_v]];
        end

        if (N_DISPATCH > 1) begin
            if (dispatch_fire_i[0] && dispatch_rd_valid_i[0] &&
                (dispatch_rd_i[0*ARCH_AW +: ARCH_AW] != {ARCH_AW{1'b0}})) begin
                alloc_v = dispatch_alloc_phys_i[0*PHYS_AW +: PHYS_AW];

                if (dispatch_rs1_i[1*ARCH_AW +: ARCH_AW] ==
                    dispatch_rd_i [0*ARCH_AW +: ARCH_AW]) begin
                    src1_phys_o [1*PHYS_AW +: PHYS_AW] = alloc_v;
                    src1_ready_o[1] = 1'b0;
                end
                if (dispatch_rs2_i[1*ARCH_AW +: ARCH_AW] ==
                    dispatch_rd_i [0*ARCH_AW +: ARCH_AW]) begin
                    src2_phys_o [1*PHYS_AW +: PHYS_AW] = alloc_v;
                    src2_ready_o[1] = 1'b0;
                end
                if (dispatch_rd_i[1*ARCH_AW +: ARCH_AW] ==
                    dispatch_rd_i[0*ARCH_AW +: ARCH_AW]) begin
                    old_phys_o[1*PHYS_AW +: PHYS_AW] = alloc_v;
                end
            end
        end

        for (d = 0; d < N_DISPATCH; d = d + 1) begin
            if (dispatch_rs1_i[d*ARCH_AW +: ARCH_AW] == {ARCH_AW{1'b0}}) begin
                src1_phys_o [d*PHYS_AW +: PHYS_AW] = {PHYS_AW{1'b0}};
                src1_ready_o[d] = 1'b1;
            end
            if (dispatch_rs2_i[d*ARCH_AW +: ARCH_AW] == {ARCH_AW{1'b0}}) begin
                src2_phys_o [d*PHYS_AW +: PHYS_AW] = {PHYS_AW{1'b0}};
                src2_ready_o[d] = 1'b1;
            end
        end

        for (wb = 0; wb < N_ARCH_WB; wb = wb + 1) begin
            arch_wb_phys_o[wb*PHYS_AW +: PHYS_AW] =
                map_r[arch_wb_rd_i[wb*ARCH_AW +: ARCH_AW]];
        end

        checkpoint_ready_v = 1'b0;
        checkpoint_alloc_idx_v = {CP_AW{1'b0}};
        restore_hit_v = 1'b0;
        restore_idx_v = {CP_AW{1'b0}};
        for (cp = 0; cp < CHECKPOINTS; cp = cp + 1) begin
            if (!cp_valid_r[cp] && !checkpoint_ready_v) begin
                checkpoint_ready_v = 1'b1;
                checkpoint_alloc_idx_v = cp[CP_AW-1:0];
            end
            if (cp_valid_r[cp] && (cp_rob_tag_r[cp] == restore_rob_tag_i)) begin
                restore_hit_v = 1'b1;
                restore_idx_v = cp[CP_AW-1:0];
            end
        end
    end

    assign checkpoint_ready_o = checkpoint_ready_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            for (i = 0; i < ARCH_REGS; i = i + 1)
                map_r[i] <= i[PHYS_AW-1:0];
            ready_r <= {PHYS_REGS{1'b1}};
            for (cp = 0; cp < CHECKPOINTS; cp = cp + 1) begin
                cp_valid_r[cp] <= 1'b0;
                cp_rob_tag_r[cp] <= {ROB_TAG_W{1'b0}};
                cp_ready_r[cp] <= {PHYS_REGS{1'b1}};
                for (ar = 0; ar < ARCH_REGS; ar = ar + 1)
                    cp_map_r[cp][ar] <= ar[PHYS_AW-1:0];
            end
        end else begin
            if (restore_valid_i && restore_hit_v) begin
                for (i = 0; i < ARCH_REGS; i = i + 1)
                    map_r[i] <= cp_map_r[restore_idx_v][i];
                ready_r <= cp_ready_r[restore_idx_v];
                for (cp = 0; cp < CHECKPOINTS; cp = cp + 1)
                    cp_valid_r[cp] <= 1'b0;
            end else begin
                for (c = 0; c < N_CDB; c = c + 1)
                    if (cdb_valid_i[c])
                        ready_r[cdb_phys_i[c*PHYS_AW +: PHYS_AW]] <= 1'b1;

                for (wb = 0; wb < N_ARCH_WB; wb = wb + 1) begin
                    if (arch_wb_valid_i[wb] &&
                        (arch_wb_rd_i[wb*ARCH_AW +: ARCH_AW] != {ARCH_AW{1'b0}})) begin
                        ready_r[map_r[arch_wb_rd_i[wb*ARCH_AW +: ARCH_AW]]] <= 1'b1;
                    end
                end

                for (d = 0; d < N_DISPATCH; d = d + 1) begin
                    if (dispatch_fire_i[d] && dispatch_rd_valid_i[d] &&
                        (dispatch_rd_i[d*ARCH_AW +: ARCH_AW] != {ARCH_AW{1'b0}})) begin
                        map_r[dispatch_rd_i[d*ARCH_AW +: ARCH_AW]]
                            <= dispatch_alloc_phys_i[d*PHYS_AW +: PHYS_AW];
                        ready_r[dispatch_alloc_phys_i[d*PHYS_AW +: PHYS_AW]] <= 1'b0;
                    end
                end

                if (checkpoint_valid_i && checkpoint_ready_v) begin
                    cp_valid_r[checkpoint_alloc_idx_v] <= 1'b1;
                    cp_rob_tag_r[checkpoint_alloc_idx_v] <= checkpoint_rob_tag_i;
                    cp_ready_r[checkpoint_alloc_idx_v] <= ready_r;
                    for (ar = 0; ar < ARCH_REGS; ar = ar + 1)
                        cp_map_r[checkpoint_alloc_idx_v][ar] <= map_r[ar];
                end

                if (checkpoint_release_valid_i) begin
                    for (cp = 0; cp < CHECKPOINTS; cp = cp + 1) begin
                        if (cp_valid_r[cp] &&
                            (cp_rob_tag_r[cp] == checkpoint_release_rob_tag_i))
                            cp_valid_r[cp] <= 1'b0;
                    end
                end

                ready_r[0] <= 1'b1;
            end
        end
    end
endmodule


module tomasulo_issue_queue_opt #(
    parameter ENTRIES    = 16,
    parameter DATA_W     = 32,
    parameter PHYS_REGS  = 64,
    parameter ROB_DEPTH  = 64,
    parameter N_DISPATCH = 2,
    parameter N_ISSUE    = 2,
    parameter N_CDB      = 2,
    parameter PHYS_AW    = $clog2(PHYS_REGS),
    parameter ROB_TAG_W  = $clog2(ROB_DEPTH)
)(
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           flush_i,

    input  wire [N_DISPATCH-1:0]          dispatch_valid_i,
    output wire [N_DISPATCH-1:0]          dispatch_ready_o,
    input  wire [N_DISPATCH*4-1:0]        dispatch_alu_ctrl_i,
    input  wire [N_DISPATCH-1:0]          dispatch_alu_src_i,
    input  wire [N_DISPATCH-1:0]          dispatch_lui_i,
    input  wire [N_DISPATCH-1:0]          dispatch_auipc_i,
    input  wire [N_DISPATCH*DATA_W-1:0]   dispatch_pc_i,
    input  wire [N_DISPATCH*DATA_W-1:0]   dispatch_imm_i,
    input  wire [N_DISPATCH*ROB_TAG_W-1:0]dispatch_rob_tag_i,
    input  wire [N_DISPATCH*PHYS_AW-1:0]  dispatch_dst_phys_i,
    input  wire [N_DISPATCH-1:0]          dispatch_dst_valid_i,
    input  wire [N_DISPATCH*PHYS_AW-1:0]  dispatch_src1_phys_i,
    input  wire [N_DISPATCH*PHYS_AW-1:0]  dispatch_src2_phys_i,
    input  wire [N_DISPATCH-1:0]          dispatch_src1_ready_i,
    input  wire [N_DISPATCH-1:0]          dispatch_src2_ready_i,
    input  wire [N_DISPATCH*DATA_W-1:0]   dispatch_src1_data_i,
    input  wire [N_DISPATCH*DATA_W-1:0]   dispatch_src2_data_i,

    input  wire [N_CDB-1:0]               cdb_valid_i,
    input  wire [N_CDB*PHYS_AW-1:0]       cdb_phys_i,
    input  wire [N_CDB*DATA_W-1:0]        cdb_data_i,

    output wire [N_ISSUE-1:0]             issue_valid_o,
    output wire [N_ISSUE*4-1:0]           issue_alu_ctrl_o,
    output wire [N_ISSUE-1:0]             issue_alu_src_o,
    output wire [N_ISSUE-1:0]             issue_lui_o,
    output wire [N_ISSUE-1:0]             issue_auipc_o,
    output wire [N_ISSUE*DATA_W-1:0]      issue_pc_o,
    output wire [N_ISSUE*DATA_W-1:0]      issue_imm_o,
    output wire [N_ISSUE*DATA_W-1:0]      issue_src1_data_o,
    output wire [N_ISSUE*DATA_W-1:0]      issue_src2_data_o,
    output wire [N_ISSUE*ROB_TAG_W-1:0]   issue_rob_tag_o,
    output wire [N_ISSUE*PHYS_AW-1:0]     issue_dst_phys_o,
    output wire [N_ISSUE-1:0]             issue_dst_valid_o
);
    localparam PAYLOAD_W = 4 + 1 + 1 + 1 + (4 * DATA_W) + ROB_TAG_W + PHYS_AW + 1;
    localparam P_ALU_CTRL = 0;
    localparam P_ALU_SRC  = P_ALU_CTRL + 4;
    localparam P_LUI      = P_ALU_SRC  + 1;
    localparam P_AUIPC    = P_LUI      + 1;
    localparam P_PC       = P_AUIPC    + 1;
    localparam P_IMM      = P_PC       + DATA_W;
    localparam P_SRC1     = P_IMM      + DATA_W;
    localparam P_SRC2     = P_SRC1     + DATA_W;
    localparam P_ROB      = P_SRC2     + DATA_W;
    localparam P_DST      = P_ROB      + ROB_TAG_W;
    localparam P_DVALID   = P_DST      + PHYS_AW;

    reg valid_r [0:ENTRIES-1];
    reg [3:0] alu_ctrl_r [0:ENTRIES-1];
    reg alu_src_r [0:ENTRIES-1];
    reg lui_r [0:ENTRIES-1];
    reg auipc_r [0:ENTRIES-1];
    reg [DATA_W-1:0] pc_r [0:ENTRIES-1];
    reg [DATA_W-1:0] imm_r [0:ENTRIES-1];
    reg [DATA_W-1:0] src1_data_r [0:ENTRIES-1];
    reg [DATA_W-1:0] src2_data_r [0:ENTRIES-1];
    reg [PHYS_AW-1:0] src1_phys_r [0:ENTRIES-1];
    reg [PHYS_AW-1:0] src2_phys_r [0:ENTRIES-1];
    reg src1_ready_r [0:ENTRIES-1];
    reg src2_ready_r [0:ENTRIES-1];
    reg [ROB_TAG_W-1:0] rob_tag_r [0:ENTRIES-1];
    reg [PHYS_AW-1:0] dst_phys_r [0:ENTRIES-1];
    reg dst_valid_r [0:ENTRIES-1];

    wire [ENTRIES-1:0] valid_vec_w;
    wire [ENTRIES-1:0] ready_vec_w;
    wire [ENTRIES*PAYLOAD_W-1:0] payload_flat_w;

    genvar e_gen;
    generate
        for (e_gen = 0; e_gen < ENTRIES; e_gen = e_gen + 1) begin : payload_pack_gen
            assign valid_vec_w[e_gen] = valid_r[e_gen];
            assign ready_vec_w[e_gen] = valid_r[e_gen] &
                                        (src1_ready_r[e_gen] | lui_r[e_gen] | auipc_r[e_gen]) &
                                        (src2_ready_r[e_gen] | alu_src_r[e_gen] |
                                         lui_r[e_gen] | auipc_r[e_gen]);
            assign payload_flat_w[e_gen*PAYLOAD_W +: PAYLOAD_W] = {
                dst_valid_r[e_gen],
                dst_phys_r[e_gen],
                rob_tag_r[e_gen],
                src2_data_r[e_gen],
                src1_data_r[e_gen],
                imm_r[e_gen],
                pc_r[e_gen],
                auipc_r[e_gen],
                lui_r[e_gen],
                alu_src_r[e_gen],
                alu_ctrl_r[e_gen]
            };
        end
    endgenerate

    wire [ENTRIES-1:0] issue0_sel_w;
    wire [ENTRIES-1:0] issue1_sel_w;
    wire issue0_valid_w;
    wire issue1_valid_w;

    tomasulo_first_onehot #(.WIDTH(ENTRIES)) u_issue0_sel (
        .req_i(ready_vec_w),
        .grant_o(issue0_sel_w),
        .valid_o(issue0_valid_w)
    );

    tomasulo_first_onehot #(.WIDTH(ENTRIES)) u_issue1_sel (
        .req_i(ready_vec_w & ~issue0_sel_w),
        .grant_o(issue1_sel_w),
        .valid_o(issue1_valid_w)
    );

    wire [PAYLOAD_W-1:0] issue0_payload_w;
    wire [PAYLOAD_W-1:0] issue1_payload_w;

    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(PAYLOAD_W)) u_issue0_mux (
        .sel_i(issue0_sel_w),
        .data_i(payload_flat_w),
        .data_o(issue0_payload_w)
    );

    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(PAYLOAD_W)) u_issue1_mux (
        .sel_i(issue1_sel_w),
        .data_i(payload_flat_w),
        .data_o(issue1_payload_w)
    );

    reg [N_ISSUE-1:0]             issue_valid_r;
    reg [N_ISSUE*4-1:0]           issue_alu_ctrl_r;
    reg [N_ISSUE-1:0]             issue_alu_src_r;
    reg [N_ISSUE-1:0]             issue_lui_r;
    reg [N_ISSUE-1:0]             issue_auipc_r;
    reg [N_ISSUE*DATA_W-1:0]      issue_pc_r;
    reg [N_ISSUE*DATA_W-1:0]      issue_imm_r;
    reg [N_ISSUE*DATA_W-1:0]      issue_src1_data_r;
    reg [N_ISSUE*DATA_W-1:0]      issue_src2_data_r;
    reg [N_ISSUE*ROB_TAG_W-1:0]   issue_rob_tag_r;
    reg [N_ISSUE*PHYS_AW-1:0]     issue_dst_phys_r;
    reg [N_ISSUE-1:0]             issue_dst_valid_r;

    assign issue_valid_o    = issue_valid_r;
    assign issue_alu_ctrl_o = issue_alu_ctrl_r;
    assign issue_alu_src_o  = issue_alu_src_r;
    assign issue_lui_o      = issue_lui_r;
    assign issue_auipc_o    = issue_auipc_r;
    assign issue_pc_o       = issue_pc_r;
    assign issue_imm_o      = issue_imm_r;
    assign issue_src1_data_o = issue_src1_data_r;
    assign issue_src2_data_o = issue_src2_data_r;
    assign issue_rob_tag_o   = issue_rob_tag_r;
    assign issue_dst_phys_o  = issue_dst_phys_r;
    assign issue_dst_valid_o = issue_dst_valid_r;

    wire [ENTRIES-1:0] issue_pick_mask_w = issue0_sel_w | issue1_sel_w;
    wire [ENTRIES-1:0] free_vec_w = ~valid_vec_w | issue_pick_mask_w;
    wire [ENTRIES-1:0] free0_sel_w;
    wire [ENTRIES-1:0] free1_sel_w;
    wire free0_valid_w;
    wire free1_valid_w;

    tomasulo_first_onehot #(.WIDTH(ENTRIES)) u_free0_sel (
        .req_i(free_vec_w),
        .grant_o(free0_sel_w),
        .valid_o(free0_valid_w)
    );

    tomasulo_first_onehot #(.WIDTH(ENTRIES)) u_free1_sel (
        .req_i(free_vec_w & ~free0_sel_w),
        .grant_o(free1_sel_w),
        .valid_o(free1_valid_w)
    );

    assign dispatch_ready_o[0] = free0_valid_w;
    assign dispatch_ready_o[1] = free0_valid_w & free1_valid_w;

    wire [ENTRIES-1:0] dispatch0_entry_w =
        {ENTRIES{dispatch_valid_i[0] & dispatch_ready_o[0]}} & free0_sel_w;
    wire [ENTRIES-1:0] dispatch1_entry_w =
        {ENTRIES{dispatch_valid_i[1] & dispatch_ready_o[1]}} & free1_sel_w;

    integer e;
    integer c;
    reg iq_src1_dispatch_ready_v;
    reg iq_src2_dispatch_ready_v;
    reg [DATA_W-1:0] iq_src1_dispatch_data_v;
    reg [DATA_W-1:0] iq_src2_dispatch_data_v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            issue_valid_r <= {N_ISSUE{1'b0}};
            issue_alu_ctrl_r <= {N_ISSUE*4{1'b0}};
            issue_alu_src_r <= {N_ISSUE{1'b0}};
            issue_lui_r <= {N_ISSUE{1'b0}};
            issue_auipc_r <= {N_ISSUE{1'b0}};
            issue_pc_r <= {N_ISSUE*DATA_W{1'b0}};
            issue_imm_r <= {N_ISSUE*DATA_W{1'b0}};
            issue_src1_data_r <= {N_ISSUE*DATA_W{1'b0}};
            issue_src2_data_r <= {N_ISSUE*DATA_W{1'b0}};
            issue_rob_tag_r <= {N_ISSUE*ROB_TAG_W{1'b0}};
            issue_dst_phys_r <= {N_ISSUE*PHYS_AW{1'b0}};
            issue_dst_valid_r <= {N_ISSUE{1'b0}};
            for (e = 0; e < ENTRIES; e = e + 1) begin
                valid_r[e] <= 1'b0;
                src1_ready_r[e] <= 1'b0;
                src2_ready_r[e] <= 1'b0;
            end
        end else begin
            issue_valid_r[0] <= issue0_valid_w;
            issue_alu_ctrl_r[0*4 +: 4] <= issue0_payload_w[P_ALU_CTRL +: 4];
            issue_alu_src_r[0] <= issue0_payload_w[P_ALU_SRC];
            issue_lui_r[0] <= issue0_payload_w[P_LUI];
            issue_auipc_r[0] <= issue0_payload_w[P_AUIPC];
            issue_pc_r[0*DATA_W +: DATA_W] <= issue0_payload_w[P_PC +: DATA_W];
            issue_imm_r[0*DATA_W +: DATA_W] <= issue0_payload_w[P_IMM +: DATA_W];
            issue_src1_data_r[0*DATA_W +: DATA_W] <= issue0_payload_w[P_SRC1 +: DATA_W];
            issue_src2_data_r[0*DATA_W +: DATA_W] <= issue0_payload_w[P_SRC2 +: DATA_W];
            issue_rob_tag_r[0*ROB_TAG_W +: ROB_TAG_W] <= issue0_payload_w[P_ROB +: ROB_TAG_W];
            issue_dst_phys_r[0*PHYS_AW +: PHYS_AW] <= issue0_payload_w[P_DST +: PHYS_AW];
            issue_dst_valid_r[0] <= issue0_payload_w[P_DVALID];

            issue_valid_r[1] <= issue1_valid_w;
            issue_alu_ctrl_r[1*4 +: 4] <= issue1_payload_w[P_ALU_CTRL +: 4];
            issue_alu_src_r[1] <= issue1_payload_w[P_ALU_SRC];
            issue_lui_r[1] <= issue1_payload_w[P_LUI];
            issue_auipc_r[1] <= issue1_payload_w[P_AUIPC];
            issue_pc_r[1*DATA_W +: DATA_W] <= issue1_payload_w[P_PC +: DATA_W];
            issue_imm_r[1*DATA_W +: DATA_W] <= issue1_payload_w[P_IMM +: DATA_W];
            issue_src1_data_r[1*DATA_W +: DATA_W] <= issue1_payload_w[P_SRC1 +: DATA_W];
            issue_src2_data_r[1*DATA_W +: DATA_W] <= issue1_payload_w[P_SRC2 +: DATA_W];
            issue_rob_tag_r[1*ROB_TAG_W +: ROB_TAG_W] <= issue1_payload_w[P_ROB +: ROB_TAG_W];
            issue_dst_phys_r[1*PHYS_AW +: PHYS_AW] <= issue1_payload_w[P_DST +: PHYS_AW];
            issue_dst_valid_r[1] <= issue1_payload_w[P_DVALID];

            for (e = 0; e < ENTRIES; e = e + 1) begin
                if (dispatch0_entry_w[e]) begin
                    iq_src1_dispatch_ready_v = dispatch_src1_ready_i[0];
                    iq_src2_dispatch_ready_v = dispatch_alu_src_i[0] | dispatch_lui_i[0] |
                                               dispatch_auipc_i[0] | dispatch_src2_ready_i[0];
                    iq_src1_dispatch_data_v = dispatch_src1_data_i[0*DATA_W +: DATA_W];
                    iq_src2_dispatch_data_v = dispatch_src2_data_i[0*DATA_W +: DATA_W];
                    for (c = 0; c < N_CDB; c = c + 1) begin
                        if (cdb_valid_i[c] &&
                            (dispatch_src1_phys_i[0*PHYS_AW +: PHYS_AW] ==
                             cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            iq_src1_dispatch_ready_v = 1'b1;
                            iq_src1_dispatch_data_v = cdb_data_i[c*DATA_W +: DATA_W];
                        end
                        if (!dispatch_alu_src_i[0] && !dispatch_lui_i[0] &&
                            !dispatch_auipc_i[0] && cdb_valid_i[c] &&
                            (dispatch_src2_phys_i[0*PHYS_AW +: PHYS_AW] ==
                             cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            iq_src2_dispatch_ready_v = 1'b1;
                            iq_src2_dispatch_data_v = cdb_data_i[c*DATA_W +: DATA_W];
                        end
                    end
                    valid_r[e] <= 1'b1;
                    alu_ctrl_r[e] <= dispatch_alu_ctrl_i[0*4 +: 4];
                    alu_src_r[e] <= dispatch_alu_src_i[0];
                    lui_r[e] <= dispatch_lui_i[0];
                    auipc_r[e] <= dispatch_auipc_i[0];
                    pc_r[e] <= dispatch_pc_i[0*DATA_W +: DATA_W];
                    imm_r[e] <= dispatch_imm_i[0*DATA_W +: DATA_W];
                    src1_phys_r[e] <= dispatch_src1_phys_i[0*PHYS_AW +: PHYS_AW];
                    src2_phys_r[e] <= dispatch_src2_phys_i[0*PHYS_AW +: PHYS_AW];
                    src1_ready_r[e] <= iq_src1_dispatch_ready_v;
                    src2_ready_r[e] <= iq_src2_dispatch_ready_v;
                    src1_data_r[e] <= iq_src1_dispatch_data_v;
                    src2_data_r[e] <= iq_src2_dispatch_data_v;
                    rob_tag_r[e] <= dispatch_rob_tag_i[0*ROB_TAG_W +: ROB_TAG_W];
                    dst_phys_r[e] <= dispatch_dst_phys_i[0*PHYS_AW +: PHYS_AW];
                    dst_valid_r[e] <= dispatch_dst_valid_i[0];
                end else if (dispatch1_entry_w[e]) begin
                    iq_src1_dispatch_ready_v = dispatch_src1_ready_i[1];
                    iq_src2_dispatch_ready_v = dispatch_alu_src_i[1] | dispatch_lui_i[1] |
                                               dispatch_auipc_i[1] | dispatch_src2_ready_i[1];
                    iq_src1_dispatch_data_v = dispatch_src1_data_i[1*DATA_W +: DATA_W];
                    iq_src2_dispatch_data_v = dispatch_src2_data_i[1*DATA_W +: DATA_W];
                    for (c = 0; c < N_CDB; c = c + 1) begin
                        if (cdb_valid_i[c] &&
                            (dispatch_src1_phys_i[1*PHYS_AW +: PHYS_AW] ==
                             cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            iq_src1_dispatch_ready_v = 1'b1;
                            iq_src1_dispatch_data_v = cdb_data_i[c*DATA_W +: DATA_W];
                        end
                        if (!dispatch_alu_src_i[1] && !dispatch_lui_i[1] &&
                            !dispatch_auipc_i[1] && cdb_valid_i[c] &&
                            (dispatch_src2_phys_i[1*PHYS_AW +: PHYS_AW] ==
                             cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            iq_src2_dispatch_ready_v = 1'b1;
                            iq_src2_dispatch_data_v = cdb_data_i[c*DATA_W +: DATA_W];
                        end
                    end
                    valid_r[e] <= 1'b1;
                    alu_ctrl_r[e] <= dispatch_alu_ctrl_i[1*4 +: 4];
                    alu_src_r[e] <= dispatch_alu_src_i[1];
                    lui_r[e] <= dispatch_lui_i[1];
                    auipc_r[e] <= dispatch_auipc_i[1];
                    pc_r[e] <= dispatch_pc_i[1*DATA_W +: DATA_W];
                    imm_r[e] <= dispatch_imm_i[1*DATA_W +: DATA_W];
                    src1_phys_r[e] <= dispatch_src1_phys_i[1*PHYS_AW +: PHYS_AW];
                    src2_phys_r[e] <= dispatch_src2_phys_i[1*PHYS_AW +: PHYS_AW];
                    src1_ready_r[e] <= iq_src1_dispatch_ready_v;
                    src2_ready_r[e] <= iq_src2_dispatch_ready_v;
                    src1_data_r[e] <= iq_src1_dispatch_data_v;
                    src2_data_r[e] <= iq_src2_dispatch_data_v;
                    rob_tag_r[e] <= dispatch_rob_tag_i[1*ROB_TAG_W +: ROB_TAG_W];
                    dst_phys_r[e] <= dispatch_dst_phys_i[1*PHYS_AW +: PHYS_AW];
                    dst_valid_r[e] <= dispatch_dst_valid_i[1];
                end else if (issue_pick_mask_w[e]) begin
                    valid_r[e] <= 1'b0;
                end else if (valid_r[e]) begin
                    for (c = 0; c < N_CDB; c = c + 1) begin
                        if (cdb_valid_i[c]) begin
                            if (!src1_ready_r[e] &&
                                (src1_phys_r[e] == cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                                src1_ready_r[e] <= 1'b1;
                                src1_data_r[e] <= cdb_data_i[c*DATA_W +: DATA_W];
                            end
                            if (!src2_ready_r[e] &&
                                (src2_phys_r[e] == cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                                src2_ready_r[e] <= 1'b1;
                                src2_data_r[e] <= cdb_data_i[c*DATA_W +: DATA_W];
                            end
                        end
                    end
                end
            end
        end
    end
endmodule


module tomasulo_alu_execute_2way #(
    parameter DATA_W    = 32,
    parameter PHYS_REGS = 64,
    parameter ROB_DEPTH = 64,
    parameter N_ISSUE   = 2,
    parameter PHYS_AW   = $clog2(PHYS_REGS),
    parameter ROB_TAG_W = $clog2(ROB_DEPTH)
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         flush_i,

    input  wire [N_ISSUE-1:0]           issue_valid_i,
    input  wire [N_ISSUE*4-1:0]         issue_alu_ctrl_i,
    input  wire [N_ISSUE-1:0]           issue_alu_src_i,
    input  wire [N_ISSUE-1:0]           issue_lui_i,
    input  wire [N_ISSUE-1:0]           issue_auipc_i,
    input  wire [N_ISSUE*DATA_W-1:0]    issue_pc_i,
    input  wire [N_ISSUE*DATA_W-1:0]    issue_imm_i,
    input  wire [N_ISSUE*DATA_W-1:0]    issue_src1_data_i,
    input  wire [N_ISSUE*DATA_W-1:0]    issue_src2_data_i,
    input  wire [N_ISSUE*ROB_TAG_W-1:0] issue_rob_tag_i,
    input  wire [N_ISSUE*PHYS_AW-1:0]   issue_dst_phys_i,
    input  wire [N_ISSUE-1:0]           issue_dst_valid_i,

    output reg  [N_ISSUE-1:0]           fu_valid_o,
    output reg  [N_ISSUE*DATA_W-1:0]    fu_data_o,
    output reg  [N_ISSUE*ROB_TAG_W-1:0] fu_rob_tag_o,
    output reg  [N_ISSUE*PHYS_AW-1:0]   fu_dst_phys_o,
    output reg  [N_ISSUE-1:0]           fu_dst_valid_o
);
    integer i;
    reg [DATA_W-1:0] src1_v;
    reg [DATA_W-1:0] src2_v;
    reg [DATA_W-1:0] op2_v;
    reg [3:0] ctrl_v;
    reg [DATA_W-1:0] result_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            fu_valid_o <= {N_ISSUE{1'b0}};
            fu_data_o <= {N_ISSUE*DATA_W{1'b0}};
            fu_rob_tag_o <= {N_ISSUE*ROB_TAG_W{1'b0}};
            fu_dst_phys_o <= {N_ISSUE*PHYS_AW{1'b0}};
            fu_dst_valid_o <= {N_ISSUE{1'b0}};
        end else begin
            for (i = 0; i < N_ISSUE; i = i + 1) begin
                src1_v = issue_src1_data_i[i*DATA_W +: DATA_W];
                src2_v = issue_src2_data_i[i*DATA_W +: DATA_W];
                op2_v  = issue_alu_src_i[i] ? issue_imm_i[i*DATA_W +: DATA_W] : src2_v;
                ctrl_v = issue_alu_ctrl_i[i*4 +: 4];
                result_v = src1_v + op2_v;

                if (issue_lui_i[i])
                    result_v = issue_imm_i[i*DATA_W +: DATA_W];
                else if (issue_auipc_i[i])
                    result_v = issue_pc_i[i*DATA_W +: DATA_W] +
                               issue_imm_i[i*DATA_W +: DATA_W];
                else begin
                    case (ctrl_v)
                        4'b0000: result_v = src1_v & op2_v;
                        4'b0001: result_v = src1_v | op2_v;
                        4'b0010: result_v = src1_v + op2_v;
                        4'b0110: result_v = src1_v - op2_v;
                        4'b0100: result_v = src1_v ^ op2_v;
                        4'b0111: result_v = ($signed(src1_v) < $signed(op2_v)) ? 32'd1 : 32'd0;
                        4'b1010: result_v = (src1_v < op2_v) ? 32'd1 : 32'd0;
                        4'b1000: result_v = src1_v << op2_v[4:0];
                        4'b1001: result_v = src1_v >> op2_v[4:0];
                        4'b1011: result_v = $signed(src1_v) >>> op2_v[4:0];
                        default: result_v = src1_v + op2_v;
                    endcase
                end

                fu_valid_o[i] <= issue_valid_i[i];
                fu_data_o[i*DATA_W +: DATA_W] <= result_v;
                fu_rob_tag_o[i*ROB_TAG_W +: ROB_TAG_W] <=
                    issue_rob_tag_i[i*ROB_TAG_W +: ROB_TAG_W];
                fu_dst_phys_o[i*PHYS_AW +: PHYS_AW] <=
                    issue_dst_phys_i[i*PHYS_AW +: PHYS_AW];
                fu_dst_valid_o[i] <= issue_dst_valid_i[i];
            end
        end
    end
endmodule


module tomasulo_cdb_arbiter #(
    parameter DATA_W    = 32,
    parameter PHYS_REGS = 64,
    parameter ROB_DEPTH = 64,
    parameter N_FU      = 2,
    parameter N_CDB     = 2,
    parameter PHYS_AW   = $clog2(PHYS_REGS),
    parameter ROB_TAG_W = $clog2(ROB_DEPTH)
)(
    input  wire [N_FU-1:0]               fu_valid_i,
    input  wire [N_FU*DATA_W-1:0]        fu_data_i,
    input  wire [N_FU*ROB_TAG_W-1:0]     fu_rob_tag_i,
    input  wire [N_FU*PHYS_AW-1:0]       fu_dst_phys_i,
    input  wire [N_FU-1:0]               fu_dst_valid_i,

    output wire [N_CDB-1:0]              cdb_valid_o,
    output wire [N_CDB*DATA_W-1:0]       cdb_data_o,
    output wire [N_CDB*ROB_TAG_W-1:0]    cdb_rob_tag_o,
    output wire [N_CDB*PHYS_AW-1:0]      cdb_phys_o,
    output wire [N_CDB-1:0]              cdb_phys_valid_o
);
    wire [N_FU-1:0] grant_w [0:N_CDB-1];
    wire [N_FU-1:0] used_w  [0:N_CDB];

    assign used_w[0] = {N_FU{1'b0}};

    genvar i;
    generate
        for (i = 0; i < N_CDB; i = i + 1) begin : cdb_grant_gen
            wire [N_FU-1:0] req_w = fu_valid_i & ~used_w[i];
            tomasulo_first_onehot #(.WIDTH(N_FU)) u_pick (
                .req_i(req_w),
                .grant_o(grant_w[i]),
                .valid_o(cdb_valid_o[i])
            );
            assign used_w[i+1] = used_w[i] | grant_w[i];

            tomasulo_onehot_mux #(.WIDTH(N_FU), .DATA_W(DATA_W)) u_data_mux (
                .sel_i(grant_w[i]),
                .data_i(fu_data_i),
                .data_o(cdb_data_o[i*DATA_W +: DATA_W])
            );
            tomasulo_onehot_mux #(.WIDTH(N_FU), .DATA_W(ROB_TAG_W)) u_tag_mux (
                .sel_i(grant_w[i]),
                .data_i(fu_rob_tag_i),
                .data_o(cdb_rob_tag_o[i*ROB_TAG_W +: ROB_TAG_W])
            );
            tomasulo_onehot_mux #(.WIDTH(N_FU), .DATA_W(PHYS_AW)) u_phys_mux (
                .sel_i(grant_w[i]),
                .data_i(fu_dst_phys_i),
                .data_o(cdb_phys_o[i*PHYS_AW +: PHYS_AW])
            );
            tomasulo_onehot_mux #(.WIDTH(N_FU), .DATA_W(1)) u_phys_valid_mux (
                .sel_i(grant_w[i]),
                .data_i(fu_dst_valid_i),
                .data_o(cdb_phys_valid_o[i])
            );
        end
    endgenerate
endmodule


module tomasulo_lsq_conservative #(
    parameter ENTRIES    = 8,
    parameter DATA_W     = 32,
    parameter PHYS_REGS  = 64,
    parameter ROB_DEPTH  = 64,
    parameter N_DISPATCH = 2,
    parameter N_COMMIT   = 2,
    parameter N_CDB      = 2,
    parameter PHYS_AW    = $clog2(PHYS_REGS),
    parameter ROB_TAG_W  = $clog2(ROB_DEPTH),
    parameter LSQ_AW     = $clog2(ENTRIES),
    parameter LSQ_SEQ_W  = ROB_TAG_W + LSQ_AW + 1
)(
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           flush_i,

    input  wire [N_DISPATCH-1:0]          dispatch_valid_i,
    output wire [N_DISPATCH-1:0]          dispatch_ready_o,
    input  wire [N_DISPATCH-1:0]          dispatch_load_i,
    input  wire [N_DISPATCH-1:0]          dispatch_store_i,
    input  wire [N_DISPATCH*ROB_TAG_W-1:0]dispatch_rob_tag_i,
    input  wire [N_DISPATCH*PHYS_AW-1:0]  dispatch_dst_phys_i,
    input  wire [N_DISPATCH-1:0]          dispatch_dst_valid_i,
    input  wire [N_DISPATCH*PHYS_AW-1:0]  dispatch_src1_phys_i,
    input  wire [N_DISPATCH*PHYS_AW-1:0]  dispatch_src2_phys_i,
    input  wire [N_DISPATCH-1:0]          dispatch_src1_ready_i,
    input  wire [N_DISPATCH-1:0]          dispatch_src2_ready_i,
    input  wire [N_DISPATCH*DATA_W-1:0]   dispatch_src1_data_i,
    input  wire [N_DISPATCH*DATA_W-1:0]   dispatch_src2_data_i,
    input  wire [N_DISPATCH*DATA_W-1:0]   dispatch_imm_i,
    input  wire [N_DISPATCH*2-1:0]        dispatch_mem_size_i,
    input  wire [N_DISPATCH-1:0]          dispatch_mem_unsigned_i,

    input  wire [N_CDB-1:0]               cdb_valid_i,
    input  wire [N_CDB*PHYS_AW-1:0]       cdb_phys_i,
    input  wire [N_CDB*DATA_W-1:0]        cdb_data_i,

    input  wire [N_COMMIT-1:0]            commit_valid_i,
    input  wire [N_COMMIT-1:0]            commit_is_store_i,
    input  wire [N_COMMIT*ROB_TAG_W-1:0]  commit_rob_tag_i,

    output wire                           dcache_read_req_o,
    output wire                           dcache_write_req_o,
    output wire [DATA_W-1:0]              dcache_addr_o,
    output wire [DATA_W-1:0]              dcache_write_data_o,
    output wire [1:0]                     dcache_mem_size_o,
    output wire                           dcache_mem_unsigned_o,
    input  wire [DATA_W-1:0]              dcache_read_data_i,
    input  wire                           dcache_hit_i,
    input  wire                           dcache_stall_i,

    output wire                           empty_o,
    output reg                            fu_valid_o,
    output reg  [DATA_W-1:0]              fu_data_o,
    output reg  [ROB_TAG_W-1:0]           fu_rob_tag_o,
    output reg  [PHYS_AW-1:0]             fu_dst_phys_o,
    output reg                            fu_dst_valid_o
);
    reg valid_r [0:ENTRIES-1];
    reg load_r [0:ENTRIES-1];
    reg store_r [0:ENTRIES-1];
    reg committed_r [0:ENTRIES-1];
    reg store_ready_sent_r [0:ENTRIES-1];
    reg [ROB_TAG_W-1:0] rob_tag_r [0:ENTRIES-1];
    reg [PHYS_AW-1:0] dst_phys_r [0:ENTRIES-1];
    reg dst_valid_r [0:ENTRIES-1];
    reg [PHYS_AW-1:0] src1_phys_r [0:ENTRIES-1];
    reg [PHYS_AW-1:0] src2_phys_r [0:ENTRIES-1];
    reg src1_ready_r [0:ENTRIES-1];
    reg src2_ready_r [0:ENTRIES-1];
    reg [DATA_W-1:0] src1_data_r [0:ENTRIES-1];
    reg [DATA_W-1:0] src2_data_r [0:ENTRIES-1];
    reg [DATA_W-1:0] imm_r [0:ENTRIES-1];
    reg [1:0] mem_size_r [0:ENTRIES-1];
    reg mem_unsigned_r [0:ENTRIES-1];
    reg [LSQ_SEQ_W-1:0] seq_r [0:ENTRIES-1];
    reg [LSQ_SEQ_W-1:0] seq_next_r;

    reg [N_DISPATCH-1:0] dispatch_ready_r;
    reg [LSQ_AW-1:0] dispatch_idx_r [0:N_DISPATCH-1];
    reg [ENTRIES-1:0] older_store_pending_r;

    integer e;
    integer d;
    integer c;
    integer free_seen_v;
    integer load_scan_i;
    integer store_scan_i;

    always @(*) begin
        dispatch_ready_r = {N_DISPATCH{1'b0}};
        free_seen_v = 0;
        for (d = 0; d < N_DISPATCH; d = d + 1)
            dispatch_idx_r[d] = {LSQ_AW{1'b0}};

        for (e = 0; e < ENTRIES; e = e + 1) begin
            if (!valid_r[e] && (free_seen_v < N_DISPATCH)) begin
                dispatch_ready_r[free_seen_v] = 1'b1;
                dispatch_idx_r[free_seen_v] = e[LSQ_AW-1:0];
                free_seen_v = free_seen_v + 1;
            end
        end
    end

    always @(*) begin
        older_store_pending_r = {ENTRIES{1'b0}};
        for (load_scan_i = 0; load_scan_i < ENTRIES; load_scan_i = load_scan_i + 1) begin
            for (store_scan_i = 0; store_scan_i < ENTRIES; store_scan_i = store_scan_i + 1) begin
                if (valid_r[load_scan_i] && load_r[load_scan_i] &&
                    valid_r[store_scan_i] && store_r[store_scan_i] &&
                    (seq_r[store_scan_i] < seq_r[load_scan_i])) begin
                    older_store_pending_r[load_scan_i] = 1'b1;
                end
            end
        end
    end

    assign dispatch_ready_o = dispatch_ready_r;

    wire [ENTRIES-1:0] valid_vec_w;
    wire [ENTRIES-1:0] store_valid_vec_w;
    wire [ENTRIES-1:0] store_ready_req_w;
    wire [ENTRIES-1:0] store_mem_req_w;
    wire [ENTRIES-1:0] load_mem_req_w;
    wire [ENTRIES-1:0] store_ready_sel_w;
    wire [ENTRIES-1:0] store_mem_sel_w;
    wire [ENTRIES-1:0] load_mem_sel_w;
    wire store_ready_valid_w;
    wire store_mem_valid_w;
    wire load_mem_valid_w;

    genvar g;
    generate
        for (g = 0; g < ENTRIES; g = g + 1) begin : lsq_req_gen
            assign valid_vec_w[g] = valid_r[g];
            assign store_valid_vec_w[g] = valid_r[g] & store_r[g];
            assign store_ready_req_w[g] =
                valid_r[g] & store_r[g] & src1_ready_r[g] & src2_ready_r[g] &
                !store_ready_sent_r[g];
            assign store_mem_req_w[g] =
                valid_r[g] & store_r[g] & committed_r[g] &
                src1_ready_r[g] & src2_ready_r[g];
        end
    endgenerate

    generate
        for (g = 0; g < ENTRIES; g = g + 1) begin : lsq_load_req_gen
            assign load_mem_req_w[g] =
                valid_r[g] & load_r[g] & src1_ready_r[g] &
                !older_store_pending_r[g];
        end
    endgenerate

    tomasulo_first_onehot #(.WIDTH(ENTRIES)) u_store_ready_pick (
        .req_i(store_ready_req_w),
        .grant_o(store_ready_sel_w),
        .valid_o(store_ready_valid_w)
    );
    tomasulo_first_onehot #(.WIDTH(ENTRIES)) u_store_mem_pick (
        .req_i(store_mem_req_w),
        .grant_o(store_mem_sel_w),
        .valid_o(store_mem_valid_w)
    );
    tomasulo_first_onehot #(.WIDTH(ENTRIES)) u_load_mem_pick (
        .req_i(load_mem_req_w),
        .grant_o(load_mem_sel_w),
        .valid_o(load_mem_valid_w)
    );

    wire [ENTRIES-1:0] mem_sel_w =
        store_mem_valid_w ? store_mem_sel_w : load_mem_sel_w;
    wire mem_is_store_w = store_mem_valid_w;
    wire mem_is_load_w = !store_mem_valid_w & load_mem_valid_w;
    wire mem_req_valid_w = (mem_is_store_w | mem_is_load_w) & !store_ready_valid_w;

    wire [ENTRIES*DATA_W-1:0] addr_flat_w;
    wire [ENTRIES*DATA_W-1:0] store_data_flat_w;
    wire [ENTRIES*ROB_TAG_W-1:0] rob_flat_w;
    wire [ENTRIES*PHYS_AW-1:0] dst_phys_flat_w;
    wire [ENTRIES-1:0] dst_valid_flat_w;
    wire [ENTRIES*2-1:0] mem_size_flat_w;
    wire [ENTRIES-1:0] mem_unsigned_flat_w;

    generate
        for (g = 0; g < ENTRIES; g = g + 1) begin : lsq_flat_gen
            assign addr_flat_w[g*DATA_W +: DATA_W] =
                src1_data_r[g] + imm_r[g];
            assign store_data_flat_w[g*DATA_W +: DATA_W] = src2_data_r[g];
            assign rob_flat_w[g*ROB_TAG_W +: ROB_TAG_W] = rob_tag_r[g];
            assign dst_phys_flat_w[g*PHYS_AW +: PHYS_AW] = dst_phys_r[g];
            assign dst_valid_flat_w[g] = dst_valid_r[g];
            assign mem_size_flat_w[g*2 +: 2] = mem_size_r[g];
            assign mem_unsigned_flat_w[g] = mem_unsigned_r[g];
        end
    endgenerate

    wire [DATA_W-1:0] mem_addr_w;
    wire [DATA_W-1:0] mem_store_data_w;
    wire [ROB_TAG_W-1:0] mem_rob_tag_w;
    wire [PHYS_AW-1:0] mem_dst_phys_w;
    wire mem_dst_valid_w;

    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(DATA_W)) u_mem_addr_mux (
        .sel_i(mem_sel_w),
        .data_i(addr_flat_w),
        .data_o(mem_addr_w)
    );
    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(DATA_W)) u_mem_data_mux (
        .sel_i(mem_sel_w),
        .data_i(store_data_flat_w),
        .data_o(mem_store_data_w)
    );
    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(ROB_TAG_W)) u_mem_tag_mux (
        .sel_i(mem_sel_w),
        .data_i(rob_flat_w),
        .data_o(mem_rob_tag_w)
    );
    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(PHYS_AW)) u_mem_dst_mux (
        .sel_i(mem_sel_w),
        .data_i(dst_phys_flat_w),
        .data_o(mem_dst_phys_w)
    );
    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(1)) u_mem_dst_valid_mux (
        .sel_i(mem_sel_w),
        .data_i(dst_valid_flat_w),
        .data_o(mem_dst_valid_w)
    );
    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(2)) u_mem_size_mux (
        .sel_i(mem_sel_w),
        .data_i(mem_size_flat_w),
        .data_o(dcache_mem_size_o)
    );
    tomasulo_onehot_mux #(.WIDTH(ENTRIES), .DATA_W(1)) u_mem_unsigned_mux (
        .sel_i(mem_sel_w),
        .data_i(mem_unsigned_flat_w),
        .data_o(dcache_mem_unsigned_o)
    );

    assign dcache_read_req_o = mem_req_valid_w & mem_is_load_w;
    assign dcache_write_req_o = mem_req_valid_w & mem_is_store_w;
    assign dcache_addr_o = mem_addr_w;
    assign dcache_write_data_o = mem_store_data_w;
    assign empty_o = ~(|valid_vec_w);

    wire mem_complete_w =
        mem_req_valid_w & dcache_hit_i & !dcache_stall_i;

    integer clear_i;
    integer ready_i;
    integer mem_i;
    integer commit_i;
    reg src1_dispatch_ready_v;
    reg src2_dispatch_ready_v;
    reg [DATA_W-1:0] src1_dispatch_data_v;
    reg [DATA_W-1:0] src2_dispatch_data_v;
    reg [LSQ_SEQ_W-1:0] seq_alloc_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            fu_valid_o <= 1'b0;
            fu_data_o <= {DATA_W{1'b0}};
            fu_rob_tag_o <= {ROB_TAG_W{1'b0}};
            fu_dst_phys_o <= {PHYS_AW{1'b0}};
            fu_dst_valid_o <= 1'b0;
            seq_next_r <= {LSQ_SEQ_W{1'b0}};
            for (e = 0; e < ENTRIES; e = e + 1) begin
                valid_r[e] <= 1'b0;
                load_r[e] <= 1'b0;
                store_r[e] <= 1'b0;
                committed_r[e] <= 1'b0;
                store_ready_sent_r[e] <= 1'b0;
                rob_tag_r[e] <= {ROB_TAG_W{1'b0}};
                dst_phys_r[e] <= {PHYS_AW{1'b0}};
                dst_valid_r[e] <= 1'b0;
                src1_phys_r[e] <= {PHYS_AW{1'b0}};
                src2_phys_r[e] <= {PHYS_AW{1'b0}};
                src1_ready_r[e] <= 1'b0;
                src2_ready_r[e] <= 1'b0;
                src1_data_r[e] <= {DATA_W{1'b0}};
                src2_data_r[e] <= {DATA_W{1'b0}};
                imm_r[e] <= {DATA_W{1'b0}};
                mem_size_r[e] <= 2'b00;
                mem_unsigned_r[e] <= 1'b0;
                seq_r[e] <= {LSQ_SEQ_W{1'b0}};
            end
        end else begin
            fu_valid_o <= 1'b0;
            fu_data_o <= {DATA_W{1'b0}};
            fu_rob_tag_o <= {ROB_TAG_W{1'b0}};
            fu_dst_phys_o <= {PHYS_AW{1'b0}};
            fu_dst_valid_o <= 1'b0;

            for (e = 0; e < ENTRIES; e = e + 1) begin
                for (c = 0; c < N_CDB; c = c + 1) begin
                    if (cdb_valid_i[c]) begin
                        if (!src1_ready_r[e] &&
                            (src1_phys_r[e] == cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            src1_ready_r[e] <= 1'b1;
                            src1_data_r[e] <= cdb_data_i[c*DATA_W +: DATA_W];
                        end
                        if (!src2_ready_r[e] &&
                            (src2_phys_r[e] == cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            src2_ready_r[e] <= 1'b1;
                            src2_data_r[e] <= cdb_data_i[c*DATA_W +: DATA_W];
                        end
                    end
                end

                for (commit_i = 0; commit_i < N_COMMIT; commit_i = commit_i + 1) begin
                    if (commit_valid_i[commit_i] && commit_is_store_i[commit_i] &&
                        (rob_tag_r[e] == commit_rob_tag_i[commit_i*ROB_TAG_W +: ROB_TAG_W])) begin
                        committed_r[e] <= 1'b1;
                    end
                end
            end

            if (store_ready_valid_w) begin
                for (ready_i = 0; ready_i < ENTRIES; ready_i = ready_i + 1) begin
                    if (store_ready_sel_w[ready_i]) begin
                        store_ready_sent_r[ready_i] <= 1'b1;
                        fu_valid_o <= 1'b1;
                        fu_data_o <= {DATA_W{1'b0}};
                        fu_rob_tag_o <= rob_tag_r[ready_i];
                        fu_dst_phys_o <= {PHYS_AW{1'b0}};
                        fu_dst_valid_o <= 1'b0;
                    end
                end
            end else if (mem_complete_w && mem_is_load_w) begin
                fu_valid_o <= 1'b1;
                fu_data_o <= dcache_read_data_i;
                fu_rob_tag_o <= mem_rob_tag_w;
                fu_dst_phys_o <= mem_dst_phys_w;
                fu_dst_valid_o <= mem_dst_valid_w;
            end

            if (mem_complete_w) begin
                for (mem_i = 0; mem_i < ENTRIES; mem_i = mem_i + 1) begin
                    if (mem_sel_w[mem_i]) begin
                        valid_r[mem_i] <= 1'b0;
                        load_r[mem_i] <= 1'b0;
                        store_r[mem_i] <= 1'b0;
                        committed_r[mem_i] <= 1'b0;
                        store_ready_sent_r[mem_i] <= 1'b0;
                    end
                end
            end

            seq_alloc_v = seq_next_r;
            for (d = 0; d < N_DISPATCH; d = d + 1) begin
                if (dispatch_valid_i[d] && dispatch_ready_r[d]) begin
                    src1_dispatch_ready_v = dispatch_src1_ready_i[d];
                    src2_dispatch_ready_v =
                        dispatch_load_i[d] ? 1'b1 : dispatch_src2_ready_i[d];
                    src1_dispatch_data_v =
                        dispatch_src1_data_i[d*DATA_W +: DATA_W];
                    src2_dispatch_data_v =
                        dispatch_src2_data_i[d*DATA_W +: DATA_W];
                    for (c = 0; c < N_CDB; c = c + 1) begin
                        if (cdb_valid_i[c] &&
                            (dispatch_src1_phys_i[d*PHYS_AW +: PHYS_AW] ==
                             cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            src1_dispatch_ready_v = 1'b1;
                            src1_dispatch_data_v = cdb_data_i[c*DATA_W +: DATA_W];
                        end
                        if (!dispatch_load_i[d] && cdb_valid_i[c] &&
                            (dispatch_src2_phys_i[d*PHYS_AW +: PHYS_AW] ==
                             cdb_phys_i[c*PHYS_AW +: PHYS_AW])) begin
                            src2_dispatch_ready_v = 1'b1;
                            src2_dispatch_data_v = cdb_data_i[c*DATA_W +: DATA_W];
                        end
                    end
                    valid_r[dispatch_idx_r[d]] <= 1'b1;
                    load_r[dispatch_idx_r[d]] <= dispatch_load_i[d];
                    store_r[dispatch_idx_r[d]] <= dispatch_store_i[d];
                    committed_r[dispatch_idx_r[d]] <= 1'b0;
                    store_ready_sent_r[dispatch_idx_r[d]] <= 1'b0;
                    rob_tag_r[dispatch_idx_r[d]] <=
                        dispatch_rob_tag_i[d*ROB_TAG_W +: ROB_TAG_W];
                    dst_phys_r[dispatch_idx_r[d]] <=
                        dispatch_dst_phys_i[d*PHYS_AW +: PHYS_AW];
                    dst_valid_r[dispatch_idx_r[d]] <= dispatch_dst_valid_i[d];
                    src1_phys_r[dispatch_idx_r[d]] <=
                        dispatch_src1_phys_i[d*PHYS_AW +: PHYS_AW];
                    src2_phys_r[dispatch_idx_r[d]] <=
                        dispatch_src2_phys_i[d*PHYS_AW +: PHYS_AW];
                    src1_ready_r[dispatch_idx_r[d]] <= src1_dispatch_ready_v;
                    src2_ready_r[dispatch_idx_r[d]] <= src2_dispatch_ready_v;
                    src1_data_r[dispatch_idx_r[d]] <= src1_dispatch_data_v;
                    src2_data_r[dispatch_idx_r[d]] <= src2_dispatch_data_v;
                    imm_r[dispatch_idx_r[d]] <=
                        dispatch_imm_i[d*DATA_W +: DATA_W];
                    mem_size_r[dispatch_idx_r[d]] <=
                        dispatch_mem_size_i[d*2 +: 2];
                    mem_unsigned_r[dispatch_idx_r[d]] <= dispatch_mem_unsigned_i[d];
                    seq_r[dispatch_idx_r[d]] <= seq_alloc_v;
                    seq_alloc_v = seq_alloc_v + {{(LSQ_SEQ_W-1){1'b0}}, 1'b1};
                end
            end
            seq_next_r <= seq_alloc_v;
        end
    end
endmodule


module tomasulo_backend_2way #(
    parameter ROB_DEPTH = 64,
    parameter PHYS_REGS = 64,
    parameter ARCH_REGS = 32,
    parameter IQ_ENTRIES = 16,
    parameter LSQ_ENTRIES = 8,
    parameter DATA_W = 32,
    parameter N_DISPATCH = 2,
    parameter N_COMMIT = 2,
    parameter N_CDB = 3,
    parameter N_ARCH_WB = 2,
    parameter ARCH_AW = $clog2(ARCH_REGS),
    parameter PHYS_AW = $clog2(PHYS_REGS),
    parameter ROB_TAG_W = $clog2(ROB_DEPTH),
    parameter PTR_W = ROB_TAG_W + 1
)(
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire                             flush_i,

    input  wire [N_DISPATCH-1:0]            dispatch_valid_i,
    output wire [N_DISPATCH-1:0]            dispatch_ready_o,
    input  wire [N_DISPATCH*DATA_W-1:0]     dispatch_pc_i,
    input  wire [N_DISPATCH*ARCH_AW-1:0]    dispatch_rs1_i,
    input  wire [N_DISPATCH*ARCH_AW-1:0]    dispatch_rs2_i,
    input  wire [N_DISPATCH*ARCH_AW-1:0]    dispatch_rd_i,
    input  wire [N_DISPATCH-1:0]            dispatch_rd_valid_i,
    input  wire [N_DISPATCH*4-1:0]          dispatch_alu_ctrl_i,
    input  wire [N_DISPATCH-1:0]            dispatch_alu_src_i,
    input  wire [N_DISPATCH-1:0]            dispatch_lui_i,
    input  wire [N_DISPATCH-1:0]            dispatch_auipc_i,
    input  wire [N_DISPATCH-1:0]            dispatch_load_i,
    input  wire [N_DISPATCH-1:0]            dispatch_store_i,
    input  wire [N_DISPATCH*2-1:0]          dispatch_mem_size_i,
    input  wire [N_DISPATCH-1:0]            dispatch_mem_unsigned_i,
    input  wire [N_DISPATCH*DATA_W-1:0]     dispatch_imm_i,

    input  wire [N_ARCH_WB-1:0]              arch_wb_valid_i,
    input  wire [N_ARCH_WB*ARCH_AW-1:0]      arch_wb_rd_i,
    input  wire [N_ARCH_WB*DATA_W-1:0]       arch_wb_data_i,

    output wire                             dcache_read_req_o,
    output wire                             dcache_write_req_o,
    output wire [DATA_W-1:0]                dcache_addr_o,
    output wire [DATA_W-1:0]                dcache_write_data_o,
    output wire [1:0]                       dcache_mem_size_o,
    output wire                             dcache_mem_unsigned_o,
    input  wire [DATA_W-1:0]                dcache_read_data_i,
    input  wire                             dcache_hit_i,
    input  wire                             dcache_stall_i,

    output wire [N_COMMIT-1:0]              commit_valid_o,
    output wire [N_COMMIT*ARCH_AW-1:0]      commit_rd_o,
    output wire [N_COMMIT-1:0]              commit_rd_valid_o,
    output wire [N_COMMIT*DATA_W-1:0]       commit_data_o,
    output wire                             rob_full_o,
    output wire                             rob_empty_o,
    output wire                             backend_empty_o
);
    localparam [2:0] TYPE_ALU   = 3'b000;
    localparam [2:0] TYPE_LOAD  = 3'b001;
    localparam [2:0] TYPE_STORE = 3'b010;
    localparam OOO_N_ISSUE = 2;
    localparam OOO_N_FU = OOO_N_ISSUE + 1;

    wire [N_DISPATCH-1:0] alloc_ready_w;
    wire [N_DISPATCH*PHYS_AW-1:0] alloc_phys_w;
    wire [N_DISPATCH-1:0] iq_dispatch_ready_w;
    wire [N_DISPATCH-1:0] lsq_dispatch_ready_w;
    wire [N_DISPATCH-1:0] rob_dispatch_ready_w;
    wire [N_DISPATCH*ROB_TAG_W-1:0] rob_dispatch_tag_w;
    wire [N_DISPATCH-1:0] dispatch_mem_w =
        dispatch_load_i | dispatch_store_i;
    wire [N_DISPATCH-1:0] dispatch_iq_fire_w;
    wire [N_DISPATCH-1:0] dispatch_lsq_fire_w;

    wire [N_DISPATCH-1:0] needs_phys_w;
    genvar nd;
    generate
        for (nd = 0; nd < N_DISPATCH; nd = nd + 1) begin : needs_phys_gen
            assign needs_phys_w[nd] =
                dispatch_rd_valid_i[nd] &&
                (dispatch_rd_i[nd*ARCH_AW +: ARCH_AW] != {ARCH_AW{1'b0}});
        end
    endgenerate

    wire dispatch_resource_ready0_w =
        dispatch_mem_w[0] ? lsq_dispatch_ready_w[0] : iq_dispatch_ready_w[0];
    wire dispatch_resource_ready1_w =
        dispatch_mem_w[1] ? lsq_dispatch_ready_w[1] : iq_dispatch_ready_w[1];

    assign dispatch_ready_o[0] =
        rob_dispatch_ready_w[0] &&
        dispatch_resource_ready0_w &&
        (!needs_phys_w[0] || alloc_ready_w[0]);
    assign dispatch_ready_o[1] =
        dispatch_ready_o[0] &&
        rob_dispatch_ready_w[1] &&
        dispatch_resource_ready1_w &&
        (!needs_phys_w[1] || (needs_phys_w[0] ? alloc_ready_w[1] : alloc_ready_w[0]));

    wire [N_DISPATCH-1:0] dispatch_fire_w =
        dispatch_valid_i & dispatch_ready_o;
    assign dispatch_iq_fire_w = dispatch_fire_w & ~dispatch_mem_w;
    assign dispatch_lsq_fire_w = dispatch_fire_w & dispatch_mem_w;
    wire [N_DISPATCH-1:0] alloc_req_w =
        dispatch_fire_w & needs_phys_w;

    wire [N_DISPATCH*PHYS_AW-1:0] alloc_phys_dispatch_w;
    assign alloc_phys_dispatch_w[0*PHYS_AW +: PHYS_AW] =
        alloc_phys_w[0*PHYS_AW +: PHYS_AW];
    assign alloc_phys_dispatch_w[1*PHYS_AW +: PHYS_AW] =
        needs_phys_w[0] ? alloc_phys_w[1*PHYS_AW +: PHYS_AW] :
                          alloc_phys_w[0*PHYS_AW +: PHYS_AW];

    wire [N_DISPATCH*PHYS_AW-1:0] src1_phys_w;
    wire [N_DISPATCH*PHYS_AW-1:0] src2_phys_w;
    wire [N_DISPATCH-1:0] src1_ready_w;
    wire [N_DISPATCH-1:0] src2_ready_w;
    wire [N_DISPATCH*PHYS_AW-1:0] old_phys_w;
    wire [N_ARCH_WB*PHYS_AW-1:0] arch_wb_phys_w;

    wire [N_CDB-1:0] cdb_valid_w;
    wire [N_CDB*DATA_W-1:0] cdb_data_w;
    wire [N_CDB*ROB_TAG_W-1:0] cdb_rob_tag_w;
    wire [N_CDB*PHYS_AW-1:0] cdb_phys_w;
    wire [N_CDB-1:0] cdb_phys_valid_w;

    tomasulo_rat #(
        .ARCH_REGS(ARCH_REGS),
        .PHYS_REGS(PHYS_REGS),
        .ROB_DEPTH(ROB_DEPTH),
        .N_DISPATCH(N_DISPATCH),
        .N_CDB(N_CDB),
        .N_ARCH_WB(N_ARCH_WB)
    ) u_rat (
        .clk(clk),
        .rst_n(rst_n),
        .flush_i(flush_i),
        .dispatch_fire_i(dispatch_fire_w),
        .dispatch_rs1_i(dispatch_rs1_i),
        .dispatch_rs2_i(dispatch_rs2_i),
        .dispatch_rd_i(dispatch_rd_i),
        .dispatch_rd_valid_i(needs_phys_w),
        .dispatch_alloc_phys_i(alloc_phys_dispatch_w),
        .src1_phys_o(src1_phys_w),
        .src2_phys_o(src2_phys_w),
        .src1_ready_o(src1_ready_w),
        .src2_ready_o(src2_ready_w),
        .old_phys_o(old_phys_w),
        .cdb_valid_i(cdb_phys_valid_w & cdb_valid_w),
        .cdb_phys_i(cdb_phys_w),
        .arch_wb_valid_i(arch_wb_valid_i),
        .arch_wb_rd_i(arch_wb_rd_i),
        .arch_wb_phys_o(arch_wb_phys_w),
        .checkpoint_valid_i(1'b0),
        .checkpoint_rob_tag_i({ROB_TAG_W{1'b0}}),
        .checkpoint_ready_o(),
        .checkpoint_release_valid_i(1'b0),
        .checkpoint_release_rob_tag_i({ROB_TAG_W{1'b0}}),
        .restore_valid_i(1'b0),
        .restore_rob_tag_i({ROB_TAG_W{1'b0}})
    );

    wire [N_COMMIT-1:0] free_valid_w;
    wire [N_COMMIT*PHYS_AW-1:0] free_phys_w;
    wire [PHYS_AW:0] free_count_unused_w;

    tomasulo_free_list #(
        .PHYS_REGS(PHYS_REGS),
        .ARCH_REGS(ARCH_REGS),
        .N_ALLOC(N_DISPATCH),
        .N_FREE(N_COMMIT)
    ) u_free_list (
        .clk(clk),
        .rst_n(rst_n),
        .flush_i(flush_i),
        .alloc_req_i(alloc_req_w),
        .alloc_ready_o(alloc_ready_w),
        .alloc_phys_o(alloc_phys_w),
        .free_valid_i(free_valid_w),
        .free_phys_i(free_phys_w),
        .free_count_o(free_count_unused_w)
    );

    wire [N_DISPATCH*DATA_W-1:0] src_data_w;
    wire [4*PHYS_AW-1:0] prf_rd_addr_w =
        {src2_phys_w[1*PHYS_AW +: PHYS_AW],
         src1_phys_w[1*PHYS_AW +: PHYS_AW],
         src2_phys_w[0*PHYS_AW +: PHYS_AW],
         src1_phys_w[0*PHYS_AW +: PHYS_AW]};
    wire [4*DATA_W-1:0] prf_rd_data_w;

    assign src_data_w[0*DATA_W +: DATA_W] = prf_rd_data_w[0*DATA_W +: DATA_W];
    assign src_data_w[1*DATA_W +: DATA_W] = prf_rd_data_w[2*DATA_W +: DATA_W];
    wire [N_DISPATCH*DATA_W-1:0] src2_data_w =
        {prf_rd_data_w[3*DATA_W +: DATA_W], prf_rd_data_w[1*DATA_W +: DATA_W]};

    // CDB writes already place completed OoO results in the destination
    // physical registers.  Commit only frees old physical registers and writes
    // the architectural RF outside this backend, so it does not need extra PRF
    // write ports here.
    wire [N_CDB+N_ARCH_WB-1:0] prf_wr_en_w =
        {arch_wb_valid_i, (cdb_valid_w & cdb_phys_valid_w)};
    wire [(N_CDB+N_ARCH_WB)*PHYS_AW-1:0] prf_wr_addr_w =
        {arch_wb_phys_w, cdb_phys_w};
    wire [(N_CDB+N_ARCH_WB)*DATA_W-1:0] prf_wr_data_w =
        {arch_wb_data_i, cdb_data_w};

    tomasulo_phys_regfile #(
        .PHYS_REGS(PHYS_REGS),
        .DATA_W(DATA_W),
        .N_READ(4),
        .N_WRITE(N_CDB + N_ARCH_WB)
    ) u_prf (
        .clk(clk),
        .rst_n(rst_n),
        .rd_addr_i(prf_rd_addr_w),
        .rd_data_o(prf_rd_data_w),
        .wr_en_i(prf_wr_en_w),
        .wr_addr_i(prf_wr_addr_w),
        .wr_data_i(prf_wr_data_w)
    );

    wire [N_DISPATCH*3-1:0] rob_dispatch_type_w;
    genvar rdt;
    generate
        for (rdt = 0; rdt < N_DISPATCH; rdt = rdt + 1) begin : rob_dispatch_type_gen
            assign rob_dispatch_type_w[rdt*3 +: 3] =
                dispatch_store_i[rdt] ? TYPE_STORE :
                dispatch_load_i[rdt]  ? TYPE_LOAD  :
                                        TYPE_ALU;
        end
    endgenerate
    wire [N_CDB*8-1:0] rob_cdb_exc_w = {N_CDB*8{1'b0}};
    wire [N_COMMIT*DATA_W-1:0] commit_pc_unused_w;
    wire [N_COMMIT*3-1:0] commit_type_unused_w;
    wire [N_COMMIT-1:0] commit_store_unused_w;
    wire flush_unused_w;
    wire [DATA_W-1:0] flush_pc_unused_w;
    wire exc_valid_unused_w;
    wire [7:0] exc_code_unused_w;
    wire flush_early_unused_w;
    wire [DATA_W-1:0] flush_pc_early_unused_w;
    wire [PTR_W-1:0] rob_count_unused_w;
    wire [PTR_W-1:0] rob_free_slots_unused_w;
    wire [ROB_DEPTH-1:0] wakeup_unused_w;

    reorder_buffer_hp #(
        .ROB_DEPTH(ROB_DEPTH),
        .DATA_W(DATA_W),
        .ARCH_REGS(ARCH_REGS),
        .PC_W(DATA_W),
        .N_CDB(N_CDB),
        .N_DISPATCH(N_DISPATCH),
        .N_COMMIT(N_COMMIT),
        .N_BANKS(2),
        .XCODE_W(8)
    ) u_rob (
        .clk(clk),
        .rst_n(rst_n),
        .dispatch_valid_i(dispatch_fire_w),
        .dispatch_pc_i(dispatch_pc_i),
        .dispatch_rd_i(dispatch_rd_i),
        .dispatch_rd_valid_i(dispatch_rd_valid_i),
        .dispatch_type_i(rob_dispatch_type_w),
        .dispatch_ready_o(rob_dispatch_ready_w),
        .dispatch_rob_tag_o(rob_dispatch_tag_w),
        .cdb_valid_i(cdb_valid_w),
        .cdb_tag_i(cdb_rob_tag_w),
        .cdb_data_i(cdb_data_w),
        .cdb_exc_i(rob_cdb_exc_w),
        .commit_valid_o(commit_valid_o),
        .commit_rd_o(commit_rd_o),
        .commit_rd_valid_o(commit_rd_valid_o),
        .commit_data_o(commit_data_o),
        .commit_pc_o(commit_pc_unused_w),
        .commit_type_o(commit_type_unused_w),
        .commit_is_store_o(commit_store_unused_w),
        .fwd_tag_a_i({ROB_TAG_W{1'b0}}),
        .fwd_valid_a_i(1'b0),
        .fwd_data_a_o(),
        .fwd_hit_a_o(),
        .fwd_tag_b_i({ROB_TAG_W{1'b0}}),
        .fwd_valid_b_i(1'b0),
        .fwd_data_b_o(),
        .fwd_hit_b_o(),
        .br_mispredict_i(flush_i),
        .br_correct_pc_i({DATA_W{1'b0}}),
        .flush_o(flush_unused_w),
        .flush_pc_o(flush_pc_unused_w),
        .exc_valid_o(exc_valid_unused_w),
        .exc_code_o(exc_code_unused_w),
        .flush_early_o(flush_early_unused_w),
        .flush_pc_early_o(flush_pc_early_unused_w),
        .rob_full_o(rob_full_o),
        .rob_empty_o(rob_empty_o),
        .rob_count_o(rob_count_unused_w),
        .rob_free_slots_o(rob_free_slots_unused_w),
        .wakeup_vec_o(wakeup_unused_w)
    );

    reg [PHYS_AW-1:0] old_phys_by_rob [0:ROB_DEPTH-1];
    reg [PTR_W-1:0] commit_head_r;
    integer old_i;

    wire [1:0] commit_count_w = {1'b0, commit_valid_o[0]} +
                                {1'b0, commit_valid_o[1]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            commit_head_r <= {PTR_W{1'b0}};
            for (old_i = 0; old_i < ROB_DEPTH; old_i = old_i + 1)
                old_phys_by_rob[old_i] <= {PHYS_AW{1'b0}};
        end else begin
            for (old_i = 0; old_i < N_DISPATCH; old_i = old_i + 1) begin
                if (dispatch_fire_w[old_i])
                    old_phys_by_rob[rob_dispatch_tag_w[old_i*ROB_TAG_W +: ROB_TAG_W]]
                        <= old_phys_w[old_i*PHYS_AW +: PHYS_AW];
            end
            commit_head_r <= commit_head_r + {{(PTR_W-2){1'b0}}, commit_count_w};
        end
    end

    generate
        for (nd = 0; nd < N_COMMIT; nd = nd + 1) begin : free_old_gen
            wire [ROB_TAG_W-1:0] ctag =
                commit_head_r[ROB_TAG_W-1:0] + nd[ROB_TAG_W-1:0];
            assign free_valid_w[nd] = commit_valid_o[nd] && commit_rd_valid_o[nd];
            assign free_phys_w[nd*PHYS_AW +: PHYS_AW] = old_phys_by_rob[ctag];
        end
    endgenerate

    wire [N_COMMIT*ROB_TAG_W-1:0] commit_rob_tag_w;
    genvar ctag_gen;
    generate
        for (ctag_gen = 0; ctag_gen < N_COMMIT; ctag_gen = ctag_gen + 1) begin : commit_tag_gen
            assign commit_rob_tag_w[ctag_gen*ROB_TAG_W +: ROB_TAG_W] =
                commit_head_r[ROB_TAG_W-1:0] + ctag_gen[ROB_TAG_W-1:0];
        end
    endgenerate

    wire lsq_fu_valid_w;
    wire [DATA_W-1:0] lsq_fu_data_w;
    wire [ROB_TAG_W-1:0] lsq_fu_rob_tag_w;
    wire [PHYS_AW-1:0] lsq_fu_dst_phys_w;
    wire lsq_fu_dst_valid_w;
    wire lsq_empty_w;

    tomasulo_lsq_conservative #(
        .ENTRIES(LSQ_ENTRIES),
        .DATA_W(DATA_W),
        .PHYS_REGS(PHYS_REGS),
        .ROB_DEPTH(ROB_DEPTH),
        .N_DISPATCH(N_DISPATCH),
        .N_COMMIT(N_COMMIT),
        .N_CDB(N_CDB)
    ) u_lsq (
        .clk(clk),
        .rst_n(rst_n),
        .flush_i(flush_i),
        .dispatch_valid_i(dispatch_lsq_fire_w),
        .dispatch_ready_o(lsq_dispatch_ready_w),
        .dispatch_load_i(dispatch_load_i),
        .dispatch_store_i(dispatch_store_i),
        .dispatch_rob_tag_i(rob_dispatch_tag_w),
        .dispatch_dst_phys_i(alloc_phys_dispatch_w),
        .dispatch_dst_valid_i(needs_phys_w),
        .dispatch_src1_phys_i(src1_phys_w),
        .dispatch_src2_phys_i(src2_phys_w),
        .dispatch_src1_ready_i(src1_ready_w),
        .dispatch_src2_ready_i(src2_ready_w),
        .dispatch_src1_data_i(src_data_w),
        .dispatch_src2_data_i(src2_data_w),
        .dispatch_imm_i(dispatch_imm_i),
        .dispatch_mem_size_i(dispatch_mem_size_i),
        .dispatch_mem_unsigned_i(dispatch_mem_unsigned_i),
        .cdb_valid_i(cdb_valid_w & cdb_phys_valid_w),
        .cdb_phys_i(cdb_phys_w),
        .cdb_data_i(cdb_data_w),
        .commit_valid_i(commit_valid_o),
        .commit_is_store_i(commit_store_unused_w),
        .commit_rob_tag_i(commit_rob_tag_w),
        .dcache_read_req_o(dcache_read_req_o),
        .dcache_write_req_o(dcache_write_req_o),
        .dcache_addr_o(dcache_addr_o),
        .dcache_write_data_o(dcache_write_data_o),
        .dcache_mem_size_o(dcache_mem_size_o),
        .dcache_mem_unsigned_o(dcache_mem_unsigned_o),
        .dcache_read_data_i(dcache_read_data_i),
        .dcache_hit_i(dcache_hit_i),
        .dcache_stall_i(dcache_stall_i),
        .empty_o(lsq_empty_w),
        .fu_valid_o(lsq_fu_valid_w),
        .fu_data_o(lsq_fu_data_w),
        .fu_rob_tag_o(lsq_fu_rob_tag_w),
        .fu_dst_phys_o(lsq_fu_dst_phys_w),
        .fu_dst_valid_o(lsq_fu_dst_valid_w)
    );

    assign backend_empty_o = rob_empty_o & lsq_empty_w;

    wire [OOO_N_ISSUE-1:0] issue_valid_w;
    wire [OOO_N_ISSUE*4-1:0] issue_alu_ctrl_w;
    wire [OOO_N_ISSUE-1:0] issue_alu_src_w;
    wire [OOO_N_ISSUE-1:0] issue_lui_w;
    wire [OOO_N_ISSUE-1:0] issue_auipc_w;
    wire [OOO_N_ISSUE*DATA_W-1:0] issue_pc_w;
    wire [OOO_N_ISSUE*DATA_W-1:0] issue_imm_w;
    wire [OOO_N_ISSUE*DATA_W-1:0] issue_src1_data_w;
    wire [OOO_N_ISSUE*DATA_W-1:0] issue_src2_data_w;
    wire [OOO_N_ISSUE*ROB_TAG_W-1:0] issue_rob_tag_w;
    wire [OOO_N_ISSUE*PHYS_AW-1:0] issue_dst_phys_w;
    wire [OOO_N_ISSUE-1:0] issue_dst_valid_w;

    tomasulo_issue_queue_opt #(
        .ENTRIES(IQ_ENTRIES),
        .DATA_W(DATA_W),
        .PHYS_REGS(PHYS_REGS),
        .ROB_DEPTH(ROB_DEPTH),
        .N_DISPATCH(N_DISPATCH),
        .N_ISSUE(OOO_N_ISSUE),
        .N_CDB(N_CDB)
    ) u_issue_queue (
        .clk(clk),
        .rst_n(rst_n),
        .flush_i(flush_i),
        .dispatch_valid_i(dispatch_iq_fire_w),
        .dispatch_ready_o(iq_dispatch_ready_w),
        .dispatch_alu_ctrl_i(dispatch_alu_ctrl_i),
        .dispatch_alu_src_i(dispatch_alu_src_i),
        .dispatch_lui_i(dispatch_lui_i),
        .dispatch_auipc_i(dispatch_auipc_i),
        .dispatch_pc_i(dispatch_pc_i),
        .dispatch_imm_i(dispatch_imm_i),
        .dispatch_rob_tag_i(rob_dispatch_tag_w),
        .dispatch_dst_phys_i(alloc_phys_dispatch_w),
        .dispatch_dst_valid_i(needs_phys_w),
        .dispatch_src1_phys_i(src1_phys_w),
        .dispatch_src2_phys_i(src2_phys_w),
        .dispatch_src1_ready_i(src1_ready_w),
        .dispatch_src2_ready_i(src2_ready_w),
        .dispatch_src1_data_i(src_data_w),
        .dispatch_src2_data_i(src2_data_w),
        .cdb_valid_i(cdb_valid_w & cdb_phys_valid_w),
        .cdb_phys_i(cdb_phys_w),
        .cdb_data_i(cdb_data_w),
        .issue_valid_o(issue_valid_w),
        .issue_alu_ctrl_o(issue_alu_ctrl_w),
        .issue_alu_src_o(issue_alu_src_w),
        .issue_lui_o(issue_lui_w),
        .issue_auipc_o(issue_auipc_w),
        .issue_pc_o(issue_pc_w),
        .issue_imm_o(issue_imm_w),
        .issue_src1_data_o(issue_src1_data_w),
        .issue_src2_data_o(issue_src2_data_w),
        .issue_rob_tag_o(issue_rob_tag_w),
        .issue_dst_phys_o(issue_dst_phys_w),
        .issue_dst_valid_o(issue_dst_valid_w)
    );

    wire [OOO_N_ISSUE-1:0] fu_valid_w;
    wire [OOO_N_ISSUE*DATA_W-1:0] fu_data_w;
    wire [OOO_N_ISSUE*ROB_TAG_W-1:0] fu_rob_tag_w;
    wire [OOO_N_ISSUE*PHYS_AW-1:0] fu_dst_phys_w;
    wire [OOO_N_ISSUE-1:0] fu_dst_valid_w;
    wire [OOO_N_FU-1:0] all_fu_valid_w = {lsq_fu_valid_w, fu_valid_w};
    wire [OOO_N_FU*DATA_W-1:0] all_fu_data_w = {lsq_fu_data_w, fu_data_w};
    wire [OOO_N_FU*ROB_TAG_W-1:0] all_fu_rob_tag_w =
        {lsq_fu_rob_tag_w, fu_rob_tag_w};
    wire [OOO_N_FU*PHYS_AW-1:0] all_fu_dst_phys_w =
        {lsq_fu_dst_phys_w, fu_dst_phys_w};
    wire [OOO_N_FU-1:0] all_fu_dst_valid_w =
        {lsq_fu_dst_valid_w, fu_dst_valid_w};

    tomasulo_alu_execute_2way #(
        .DATA_W(DATA_W),
        .PHYS_REGS(PHYS_REGS),
        .ROB_DEPTH(ROB_DEPTH),
        .N_ISSUE(OOO_N_ISSUE)
    ) u_alu_execute (
        .clk(clk),
        .rst_n(rst_n),
        .flush_i(flush_i),
        .issue_valid_i(issue_valid_w),
        .issue_alu_ctrl_i(issue_alu_ctrl_w),
        .issue_alu_src_i(issue_alu_src_w),
        .issue_lui_i(issue_lui_w),
        .issue_auipc_i(issue_auipc_w),
        .issue_pc_i(issue_pc_w),
        .issue_imm_i(issue_imm_w),
        .issue_src1_data_i(issue_src1_data_w),
        .issue_src2_data_i(issue_src2_data_w),
        .issue_rob_tag_i(issue_rob_tag_w),
        .issue_dst_phys_i(issue_dst_phys_w),
        .issue_dst_valid_i(issue_dst_valid_w),
        .fu_valid_o(fu_valid_w),
        .fu_data_o(fu_data_w),
        .fu_rob_tag_o(fu_rob_tag_w),
        .fu_dst_phys_o(fu_dst_phys_w),
        .fu_dst_valid_o(fu_dst_valid_w)
    );

    tomasulo_cdb_arbiter #(
        .DATA_W(DATA_W),
        .PHYS_REGS(PHYS_REGS),
        .ROB_DEPTH(ROB_DEPTH),
        .N_FU(OOO_N_FU),
        .N_CDB(N_CDB)
    ) u_cdb (
        .fu_valid_i(all_fu_valid_w),
        .fu_data_i(all_fu_data_w),
        .fu_rob_tag_i(all_fu_rob_tag_w),
        .fu_dst_phys_i(all_fu_dst_phys_w),
        .fu_dst_valid_i(all_fu_dst_valid_w),
        .cdb_valid_o(cdb_valid_w),
        .cdb_data_o(cdb_data_w),
        .cdb_rob_tag_o(cdb_rob_tag_w),
        .cdb_phys_o(cdb_phys_w),
        .cdb_phys_valid_o(cdb_phys_valid_w)
    );
endmodule
