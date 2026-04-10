module arbiter_iwrr_1cycle
#(
    parameter P_REQUESTER_NUM     = 4,
    parameter [0:(P_REQUESTER_NUM*32)-1] P_REQUESTER_WEIGHT = {32'd5, 32'd3, 32'd2, 32'd1},
    parameter P_NUM_GRANT_REQ_W   = 1
)
(
    input                               clk,
    input                               rst_n,
    input  [P_REQUESTER_NUM-1:0]        req_i,
    input  [0:(P_REQUESTER_NUM*32)-1]   req_weight_i,       // dynamic weights (unused, use static)
    input  [P_NUM_GRANT_REQ_W-1:0]      num_grant_req_i,    // unused
    input                               grant_ready_i,
    output [P_REQUESTER_NUM-1:0]        grant_valid_o
);
    localparam REQ_W    = P_REQUESTER_NUM;
    localparam WEIGHT_W = 32;
    localparam IDX_W    = (REQ_W > 1) ? $clog2(REQ_W) : 1;

    // Weight lookup
    function [WEIGHT_W-1:0] get_weight;
        input integer idx;
        begin
            get_weight = P_REQUESTER_WEIGHT[idx * 32 +: 32];
        end
    endfunction

    // Internal signals
    reg  [IDX_W-1:0]      last_grant_idx;
    reg  [WEIGHT_W-1:0]   credit [0:REQ_W-1];
    wire                   grant_handshake;

    integer i;

    // Round-robin with credit-based weighting
    reg [REQ_W-1:0] grant_candidate;
    reg found;
    reg [IDX_W:0]   idx_sum;
    reg [IDX_W-1:0] idx_wrapped;

    always @(*) begin
        grant_candidate = {REQ_W{1'b0}};
        found = 1'b0;

        // Phase 1: Search from (last_grant_idx+1), prefer requester with remaining credit
        for (i = 0; i < REQ_W; i = i + 1) begin
            if (!found) begin
                idx_sum = {1'b0, last_grant_idx} + 1 + i;
                idx_wrapped = (idx_sum >= REQ_W) ? (idx_sum - REQ_W) : idx_sum[IDX_W-1:0];
                if (req_i[idx_wrapped] && (credit[idx_wrapped] > 0)) begin
                    grant_candidate[idx_wrapped] = 1'b1;
                    found = 1'b1;
                end
            end
        end

        // Phase 2: If no one with credit, pick any active requester (triggers refill)
        if (!found) begin
            for (i = 0; i < REQ_W; i = i + 1) begin
                if (!found) begin
                    idx_sum = {1'b0, last_grant_idx} + 1 + i;
                    idx_wrapped = (idx_sum >= REQ_W) ? (idx_sum - REQ_W) : idx_sum[IDX_W-1:0];
                    if (req_i[idx_wrapped]) begin
                        grant_candidate[idx_wrapped] = 1'b1;
                        found = 1'b1;
                    end
                end
            end
        end
    end

    assign grant_valid_o = (|req_i) ? grant_candidate : {REQ_W{1'b0}};
    assign grant_handshake = (|grant_valid_o) & grant_ready_i;

    // One-hot to binary for indexing
    reg [IDX_W-1:0] grant_idx;
    always @(*) begin
        grant_idx = {IDX_W{1'b0}};
        for (i = 0; i < REQ_W; i = i + 1) begin
            if (grant_candidate[i])
                grant_idx = i[IDX_W-1:0];
        end
    end

    // Sequential: update last_grant_idx and credits
    always @(posedge clk) begin
        if (!rst_n) begin
            last_grant_idx <= {IDX_W{1'b0}};
            for (i = 0; i < REQ_W; i = i + 1)
                credit[i] <= get_weight(i);
        end else if (grant_handshake) begin
            last_grant_idx <= grant_idx;
            // Decrement credit for granted requester
            if (credit[grant_idx] > 0)
                credit[grant_idx] <= credit[grant_idx] - 1'b1;
            // Refill when granted requester exhausts credit
            if (credit[grant_idx] <= 1) begin
                for (i = 0; i < REQ_W; i = i + 1)
                    credit[i] <= get_weight(i);
            end
        end
    end

endmodule

module sa_Ax_channel 
#(
    // Interconnect configuration
    parameter                       MST_AMT             = 3,
    parameter                       OUTSTANDING_AMT     = 8,
    parameter [0:(MST_AMT*32)-1]    MST_WEIGHT          = {32'd5, 32'd3, 32'd2},
    parameter                       MST_ID_W            = $clog2(MST_AMT),
    // Transaction configuration
    parameter                       DATA_WIDTH          = 32,
    parameter                       ADDR_WIDTH          = 32,
    parameter                       TRANS_MST_ID_W      = 5,                                // Width of master transaction ID 
    parameter                       TRANS_SLV_ID_W      = TRANS_MST_ID_W + $clog2(MST_AMT), // Width of slave transaction ID
    parameter                       TRANS_BURST_W       = 2,                                // Width of xBURST 
    parameter                       TRANS_DATA_LEN_W    = 8,                                // Width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter                       TRANS_DATA_SIZE_W   = 3,                                // Width of xSIZE
    // Slave info configuration
    parameter [31:0] SLV_BASE_ADDR = 32'h0,
    parameter [31:0] SLV_ADDR_MASK = 32'h0
)
(
    // Input declaration
    // -- Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    input                                   xDATA_stall_i,
    // -- To Dispatcher x3
    input   [TRANS_MST_ID_W*MST_AMT-1:0]    dsp_AxID_i,
    input   [ADDR_WIDTH*MST_AMT-1:0]        dsp_AxADDR_i,
    input   [TRANS_BURST_W*MST_AMT-1:0]     dsp_AxBURST_i,
    input   [TRANS_DATA_LEN_W*MST_AMT-1:0]  dsp_AxLEN_i,
    input   [TRANS_DATA_SIZE_W*MST_AMT-1:0] dsp_AxSIZE_i,
    input   [MST_AMT-1:0]                   dsp_AxLOCK_i,
    input   [4*MST_AMT-1:0]                 dsp_AxCACHE_i,
    input   [3*MST_AMT-1:0]                 dsp_AxPROT_i,
    input   [4*MST_AMT-1:0]                 dsp_AxQOS_i,
    input   [4*MST_AMT-1:0]                 dsp_AxREGION_i,
    input   [MST_AMT-1:0]                   dsp_AxVALID_i,
    input   [MST_AMT-1:0]                   dsp_dispatcher_full_i,
    // -- To slave (master interface of the interconnect)
    input                                   s_AxREADY_i,
    
    // Output declaration
    // -- To Dispatcher
    output  [MST_AMT-1:0]                   dsp_AxREADY_o,
    // -- To slave (master interface of the interconnect)
    output  [TRANS_SLV_ID_W-1:0]            s_AxID_o,
    output  [ADDR_WIDTH-1:0]                s_AxADDR_o,
    output  [TRANS_BURST_W-1:0]             s_AxBURST_o,
    output  [TRANS_DATA_LEN_W-1:0]          s_AxLEN_o,
    output  [TRANS_DATA_SIZE_W-1:0]         s_AxSIZE_o,
    output                                  s_AxLOCK_o,
    output  [3:0]                            s_AxCACHE_o,
    output  [2:0]                            s_AxPROT_o,
    output  [3:0]                            s_AxQOS_o,
    output  [3:0]                            s_AxREGION_o,
    output                                  s_AxVALID_o,
    // -- To xDATA channel
    output  [TRANS_SLV_ID_W-1:0]            xDATA_AxID_o,
    output  [TRANS_DATA_LEN_W-1:0]          xDATA_AxLEN_o,
    output  [MST_ID_W-1:0]                  xDATA_mst_id_o,
    output                                  xDATA_crossing_flag_o,
    output                                  xDATA_fifo_order_wr_en_o
);
    // Local parameters initialization
    localparam ADDR_INFO_W  = TRANS_MST_ID_W + ADDR_WIDTH + TRANS_BURST_W + TRANS_DATA_LEN_W + TRANS_DATA_SIZE_W + 1 + 4 + 3 + 4 + 4; // +LOCK+CACHE+PROT+QOS+REGION
    localparam AX_INFO_W    = TRANS_SLV_ID_W + ADDR_WIDTH + TRANS_BURST_W + TRANS_DATA_LEN_W + TRANS_DATA_SIZE_W + 1 + 4 + 3 + 4 + 4; // +LOCK+CACHE+PROT+QOS+REGION
    
    // Internal variable declaration
    genvar mst_idx;
    
    // Internal signal declaration
    // wire declaration
    // ---- Pre Arbitration
    wire    [ADDR_INFO_W-1:0]       ADDR_info               [MST_AMT-1:0];
    wire    [ADDR_INFO_W-1:0]       ADDR_info_valid         [MST_AMT-1:0];
    wire    [ADDR_WIDTH-1:0]        AxADDR_i                [MST_AMT-1:0];  // De-flatten wire 
    wire    [TRANS_MST_ID_W-1:0]    AxID_valid              [MST_AMT-1:0];
    wire    [ADDR_WIDTH-1:0]        AxADDR_valid            [MST_AMT-1:0];
    wire    [TRANS_BURST_W-1:0]     AxBURST_valid           [MST_AMT-1:0];
    wire    [TRANS_DATA_LEN_W-1:0]  AxLEN_valid             [MST_AMT-1:0];
    wire    [TRANS_DATA_SIZE_W-1:0] AxSIZE_valid            [MST_AMT-1:0];
    wire                            AxLOCK_valid            [MST_AMT-1:0];
    wire    [3:0]                   AxCACHE_valid           [MST_AMT-1:0];
    wire    [2:0]                   AxPROT_valid            [MST_AMT-1:0];
    wire    [3:0]                   AxQOS_valid             [MST_AMT-1:0];
    wire    [3:0]                   AxREGION_valid          [MST_AMT-1:0];
    wire    [ADDR_WIDTH-1:0]        AxADDR_valid_split      [MST_AMT-1:0];
    wire    [TRANS_DATA_LEN_W-1:0]  AxLEN_valid_split       [MST_AMT-1:0];
    wire                            fifo_addr_info_full     [MST_AMT-1:0];
    wire                            fifo_addr_info_empt     [MST_AMT-1:0];
    wire                            fifo_addr_info_wr_en    [MST_AMT-1:0];
    wire                            fifo_addr_info_rd_en    [MST_AMT-1:0];
    wire                            dsp_handshake_occur     [MST_AMT-1:0];  
    wire                            dsp_AxVALID_dec         [MST_AMT-1:0];  // decoded AxVALID
    wire                            slv_addr_decoder        [MST_AMT-1:0];
    wire                            msk_addr_crossing_flag  [MST_AMT-1:0];
    wire                            msk_addr_crossing_valid [MST_AMT-1:0];
    wire                            msk_split_addr_sel_nxt  [MST_AMT-1:0];
    wire                            msk_split_addr_sel_en   [MST_AMT-1:0];
    wire                            rd_addr_info            [MST_AMT-1:0];
    // ---- In Arbitration
    wire    [MST_AMT-1:0]           arb_req;
    wire    [MST_AMT-1:0]           arb_grant_valid;
    wire                            arb_grant_ready;
    wire    [TRANS_DATA_LEN_W-1:0]  arb_num_grant_req;
    wire    [MST_ID_W-1:0]          granted_mst_id;
    wire                            arb_req_remain;
    // ---- Post arbitration
    wire    [TRANS_SLV_ID_W-1:0]    AxID_o_nxt;
    wire    [ADDR_WIDTH-1:0]        AxADDR_o_nxt;
    wire    [TRANS_BURST_W-1:0]     AxBURST_o_nxt;
    wire    [TRANS_DATA_LEN_W-1:0]  AxLEN_o_nxt;
    wire    [TRANS_DATA_SIZE_W-1:0] AxSIZE_o_nxt;
    wire                            AxLOCK_o_nxt;
    wire    [3:0]                   AxCACHE_o_nxt;
    wire    [2:0]                   AxPROT_o_nxt;
    wire    [3:0]                   AxQOS_o_nxt;
    wire    [3:0]                   AxREGION_o_nxt;
    wire                            AxVALID_o_nxt;
    wire                            slv_handshake_occur;
    wire                            tbr_trans_boot;
    // 
    wire                            xADDR_channel_shift_en;
    wire                            x_channel_shift_en;
    // -- Slave skid buffer
    wire    [AX_INFO_W-1:0]         ssb_bwd_data;
    wire                            ssb_bwd_valid;
    wire                            ssb_bwd_ready;
    wire    [AX_INFO_W-1:0]         ssb_fwd_data;
    wire                            ssb_fwd_valid;
    wire                            ssb_fwd_ready;
    wire    [TRANS_SLV_ID_W-1:0]    ssb_fwd_AxID;
    wire    [ADDR_WIDTH-1:0]        ssb_fwd_AxADDR;
    wire    [TRANS_BURST_W-1:0]     ssb_fwd_AxBURST;
    wire    [TRANS_DATA_LEN_W-1:0]  ssb_fwd_AxLEN;
    wire    [TRANS_DATA_SIZE_W-1:0] ssb_fwd_AxSIZE;
    wire                            ssb_fwd_AxLOCK;
    wire    [3:0]                   ssb_fwd_AxCACHE;
    wire    [2:0]                   ssb_fwd_AxPROT;
    wire    [3:0]                   ssb_fwd_AxQOS;
    wire    [3:0]                   ssb_fwd_AxREGION;
    
    // reg declaration
    reg     [TRANS_SLV_ID_W-1:0]    AxID_o_r;
    reg     [ADDR_WIDTH-1:0]        AxADDR_o_r;
    reg     [TRANS_BURST_W-1:0]     AxBURST_o_r;
    reg     [TRANS_DATA_LEN_W-1:0]  AxLEN_o_r;
    reg     [TRANS_DATA_SIZE_W-1:0] AxSIZE_o_r;
    reg                             AxLOCK_o_r;
    reg     [3:0]                   AxCACHE_o_r;
    reg     [2:0]                   AxPROT_o_r;
    reg     [3:0]                   AxQOS_o_r;
    reg     [3:0]                   AxREGION_o_r;
    reg                             AxVALID_o_r;
    reg                             msk_split_addr_sel      [MST_AMT-1:0];
    reg                             trans_booter_flag;
    
    // Module initialization
    generate
        for(mst_idx = 0; mst_idx < MST_AMT; mst_idx = mst_idx + 1) begin : MST_FIFO
            // ADDR info FIFO
            sync_fifo #(
                .FIFO_TYPE(2),              // Full flop
                .DATA_WIDTH(ADDR_INFO_W),
                .FIFO_DEPTH(OUTSTANDING_AMT)
            ) fifo_Ax_channel (
                .clk(ACLK_i),
                .data_i(ADDR_info[mst_idx]),
                .data_o(ADDR_info_valid[mst_idx]),
                .rd_valid_i(fifo_addr_info_rd_en[mst_idx]),
                .wr_valid_i(fifo_addr_info_wr_en[mst_idx]),
                .empty_o(fifo_addr_info_empt[mst_idx]),
                .full_o(fifo_addr_info_full[mst_idx]),
                .wr_ready_o(),
                .rd_ready_o(),
                .almost_empty_o(),
                .almost_full_o(),
                .counter(),
                .rst_n(ARESETn_i)
            );
            // 4KB masker
            splitting_4kb_masker #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .LEN_WIDTH(TRANS_DATA_LEN_W),
                .SIZE_WIDTH(TRANS_DATA_SIZE_W)
            ) splitting_4kb_masker (
                .ADDR_i(AxADDR_valid[mst_idx]),      
                .LEN_i(AxLEN_valid[mst_idx]),
                .SIZE_i(AxSIZE_valid[mst_idx]),      
                .mask_sel_i(msk_split_addr_sel[mst_idx]),
                .ADDR_split_o(AxADDR_valid_split[mst_idx]),
                .LEN_split_o(AxLEN_valid_split[mst_idx]), 
                .crossing_flag(msk_addr_crossing_flag[mst_idx])
            );
        end
    endgenerate
    arbiter_iwrr_1cycle #(
        .P_REQUESTER_NUM(MST_AMT),
        .P_REQUESTER_WEIGHT(MST_WEIGHT),
        .P_NUM_GRANT_REQ_W(1)
    ) arbiter (
        .clk(ACLK_i),
        .rst_n(ARESETn_i),
        .req_i(arb_req),          
        .req_weight_i(),
        .num_grant_req_i(1'b1),
        .grant_ready_i(arb_grant_ready),  
        .grant_valid_o(arb_grant_valid)
    );
    onehot_encoder #(
        .INPUT_W(MST_AMT),
        .OUTPUT_W(MST_ID_W)
    ) master_id_encoder (
        .i(arb_grant_valid),
        .o(granted_mst_id)
    );
    
    // Slave skid buffer (pipelined in/out)
    skid_buffer #(
        .SBUF_TYPE(3),
        .DATA_WIDTH(AX_INFO_W)
    ) slv_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (ssb_bwd_data),
        .bwd_valid_i(ssb_bwd_valid),
        .fwd_ready_i(ssb_fwd_ready),
        .fwd_data_o (ssb_fwd_data),
        .bwd_ready_o(ssb_bwd_ready),
        .fwd_valid_o(ssb_fwd_valid)
    );
//    edgedet 
//    #(
//        .RISING_EDGE(1'b1)
//    )transaction_booter(
//        .clk(ACLK_i),
//        .i(AxVALID_o_nxt),
////        .en(1'b1),
//        .en(~AxVALID_o_r),
//        .o(tbr_trans_boot),
//        .rst_n(ARESETn_i)
//    );
    //////////////////////////////////////////
    assign tbr_trans_boot = AxVALID_o_nxt & ~AxVALID_o_r;
    //////////////////////////////////////////
    
    // Combinational logic
    generate
    for(mst_idx = 0; mst_idx < MST_AMT; mst_idx = mst_idx + 1) begin : MST_LOGIC
        // Dispatcher interface
        assign dsp_AxREADY_o[mst_idx] = ~(dsp_dispatcher_full_i[mst_idx] | fifo_addr_info_full[mst_idx]);
        // FIFO
        assign ADDR_info[mst_idx] = {dsp_AxID_i[TRANS_MST_ID_W*(mst_idx+1)-1-:TRANS_MST_ID_W], dsp_AxADDR_i[ADDR_WIDTH*(mst_idx+1)-1-:ADDR_WIDTH], dsp_AxBURST_i[TRANS_BURST_W*(mst_idx+1)-1-:TRANS_BURST_W], dsp_AxLEN_i[TRANS_DATA_LEN_W*(mst_idx+1)-1-:TRANS_DATA_LEN_W], dsp_AxSIZE_i[TRANS_DATA_SIZE_W*(mst_idx+1)-1-:TRANS_DATA_SIZE_W], dsp_AxLOCK_i[mst_idx], dsp_AxCACHE_i[4*(mst_idx+1)-1-:4], dsp_AxPROT_i[3*(mst_idx+1)-1-:3], dsp_AxQOS_i[4*(mst_idx+1)-1-:4], dsp_AxREGION_i[4*(mst_idx+1)-1-:4]};
        assign AxADDR_i[mst_idx] = dsp_AxADDR_i[ADDR_WIDTH*(mst_idx+1)-1-:ADDR_WIDTH];
        assign slv_addr_decoder[mst_idx] = (AxADDR_i[mst_idx] & SLV_ADDR_MASK) == SLV_BASE_ADDR;
        assign dsp_AxVALID_dec[mst_idx] = slv_addr_decoder[mst_idx] & dsp_AxVALID_i[mst_idx];
        assign dsp_handshake_occur[mst_idx] = dsp_AxVALID_dec[mst_idx] & dsp_AxREADY_o[mst_idx];
        assign fifo_addr_info_wr_en[mst_idx] = dsp_handshake_occur[mst_idx];
        assign {AxID_valid[mst_idx], AxADDR_valid[mst_idx], AxBURST_valid[mst_idx], AxLEN_valid[mst_idx], AxSIZE_valid[mst_idx], AxLOCK_valid[mst_idx], AxCACHE_valid[mst_idx], AxPROT_valid[mst_idx], AxQOS_valid[mst_idx], AxREGION_valid[mst_idx]} = ADDR_info_valid[mst_idx];
        // ADDR mask controller
        assign rd_addr_info[mst_idx] = arb_grant_valid[mst_idx] & xADDR_channel_shift_en & AxVALID_o_nxt;
        assign fifo_addr_info_rd_en[mst_idx] = rd_addr_info[mst_idx] & (~msk_addr_crossing_flag[mst_idx] | msk_split_addr_sel[mst_idx]);
        assign msk_split_addr_sel_nxt[mst_idx] = ~msk_split_addr_sel[mst_idx];
        assign msk_split_addr_sel_en[mst_idx] = rd_addr_info[mst_idx] & msk_addr_crossing_flag[mst_idx];
        assign msk_addr_crossing_valid[mst_idx] = (~msk_split_addr_sel[mst_idx]) & msk_addr_crossing_flag[mst_idx];
        // Arbiter        
        assign arb_req[mst_idx] = ~fifo_addr_info_empt[mst_idx];
    end
    endgenerate
    // Arbiter
    assign arb_req_remain = |arb_req;
    assign arb_grant_ready = xADDR_channel_shift_en;
    assign arb_num_grant_req = AxLEN_o_nxt + 1'b1;
    assign slv_handshake_occur = ssb_bwd_valid & ssb_bwd_ready;
    assign xADDR_channel_shift_en = slv_handshake_occur | tbr_trans_boot;
    assign x_channel_shift_en = xADDR_channel_shift_en & ~xDATA_stall_i;
    
    assign s_AxID_o = ssb_fwd_AxID;
    assign s_AxADDR_o = ssb_fwd_AxADDR;
    assign s_AxBURST_o = ssb_fwd_AxBURST;
    assign s_AxLEN_o = ssb_fwd_AxLEN;
    assign s_AxSIZE_o = ssb_fwd_AxSIZE;
    assign s_AxLOCK_o = ssb_fwd_AxLOCK;
    assign s_AxCACHE_o = ssb_fwd_AxCACHE;
    assign s_AxPROT_o = ssb_fwd_AxPROT;
    assign s_AxQOS_o = ssb_fwd_AxQOS;
    assign s_AxREGION_o = ssb_fwd_AxREGION;
    assign s_AxVALID_o = ssb_fwd_valid;
    assign AxID_o_nxt = {granted_mst_id, AxID_valid[granted_mst_id]};
    assign AxADDR_o_nxt = AxADDR_valid_split[granted_mst_id];
    assign AxBURST_o_nxt = AxBURST_valid[granted_mst_id];
    assign AxLEN_o_nxt = AxLEN_valid_split[granted_mst_id];
    assign AxSIZE_o_nxt = AxSIZE_valid[granted_mst_id];
    assign AxLOCK_o_nxt = AxLOCK_valid[granted_mst_id];
    assign AxCACHE_o_nxt = AxCACHE_valid[granted_mst_id];
    assign AxPROT_o_nxt = AxPROT_valid[granted_mst_id];
    assign AxQOS_o_nxt = AxQOS_valid[granted_mst_id];
    assign AxREGION_o_nxt = AxREGION_valid[granted_mst_id];
    assign AxVALID_o_nxt = arb_req_remain & ~xDATA_stall_i;
    // -- 
    assign xDATA_AxID_o = AxID_o_nxt;
    assign xDATA_mst_id_o = AxID_o_nxt[TRANS_SLV_ID_W-1-:MST_ID_W];
    assign xDATA_crossing_flag_o = msk_addr_crossing_valid[granted_mst_id];
    assign xDATA_AxLEN_o = AxLEN_o_nxt;
    assign xDATA_fifo_order_wr_en_o = AxVALID_o_nxt & x_channel_shift_en;
    // -- Slave skid buffer 
    assign ssb_bwd_data     = {AxID_o_r, AxADDR_o_r, AxBURST_o_r, AxLEN_o_r, AxSIZE_o_r, AxLOCK_o_r, AxCACHE_o_r, AxPROT_o_r, AxQOS_o_r, AxREGION_o_r};
    assign ssb_bwd_valid    = AxVALID_o_r;
    assign ssb_fwd_ready    = s_AxREADY_i;
    assign {ssb_fwd_AxID, ssb_fwd_AxADDR, ssb_fwd_AxBURST, ssb_fwd_AxLEN, ssb_fwd_AxSIZE, ssb_fwd_AxLOCK, ssb_fwd_AxCACHE, ssb_fwd_AxPROT, ssb_fwd_AxQOS, ssb_fwd_AxREGION} = ssb_fwd_data;
    // Flip-flop logic
    generate
    // -- ADDR mask controller
    for(mst_idx = 0; mst_idx < MST_AMT; mst_idx = mst_idx + 1) begin : MST_FLOP
        always @(posedge ACLK_i) begin
            if(~ARESETn_i) begin
                msk_split_addr_sel[mst_idx] <= 0;
            end
            else if (msk_split_addr_sel_en[mst_idx]) begin
                msk_split_addr_sel[mst_idx] <= msk_split_addr_sel_nxt[mst_idx];
            end
        end
    end 
    endgenerate
    // -- Output reg
    // -- -- AW info
    always @(posedge ACLK_i) begin
        if(~ARESETn_i) begin
            AxID_o_r <= 0;
            AxADDR_o_r <= 0;
            AxBURST_o_r <= 0;
            AxLEN_o_r <= 0;
            AxSIZE_o_r <= 0;
            AxLOCK_o_r <= 0;
            AxCACHE_o_r <= 0;
            AxPROT_o_r <= 0;
            AxQOS_o_r <= 0;
            AxREGION_o_r <= 0;
        end
        else if(x_channel_shift_en) begin
            AxID_o_r <= AxID_o_nxt;
            AxADDR_o_r <= AxADDR_o_nxt;
            AxBURST_o_r <= AxBURST_o_nxt;
            AxLEN_o_r <= AxLEN_o_nxt;
            AxSIZE_o_r <= AxSIZE_o_nxt;
            AxLOCK_o_r <= AxLOCK_o_nxt;
            AxCACHE_o_r <= AxCACHE_o_nxt;
            AxPROT_o_r <= AxPROT_o_nxt;
            AxQOS_o_r <= AxQOS_o_nxt;
            AxREGION_o_r <= AxREGION_o_nxt;
        end
    end
    // -- -- AW control
    always @(posedge ACLK_i) begin
        if(~ARESETn_i) begin
            AxVALID_o_r <= 0;
        end
        else if(xADDR_channel_shift_en) begin
            AxVALID_o_r <= AxVALID_o_nxt;
        end
    end
endmodule

module sa_R_channel 
#(
    // Interconnect configuration
    parameter                       MST_AMT             = 3,
    parameter                       OUTSTANDING_AMT     = 8,
    parameter                       MST_ID_W            = $clog2(MST_AMT),
    // Transaction configuration
    parameter                       DATA_WIDTH          = 32,
    parameter                       ADDR_WIDTH          = 32,
    parameter                       TRANS_MST_ID_W      = 5,                            // Bus width of master transaction ID 
    parameter                       TRANS_SLV_ID_W      = TRANS_MST_ID_W + MST_ID_W,    // Bus width of slave transaction ID
    parameter                       TRANS_WR_RESP_W     = 2
)
(
    // Input declaration
    // -- Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // -- To Dispatcher
    // ---- Read data channel
    input   [MST_AMT-1:0]                   dsp_RREADY_i,
    // -- To slave (master interface of the interconnect)
    // ---- Read data channel 
    input   [TRANS_SLV_ID_W-1:0]            s_RID_i,
    input   [DATA_WIDTH-1:0]                s_RDATA_i,
    input   [TRANS_WR_RESP_W-1:0]           s_RRESP_i,
    input                                   s_RLAST_i,
    input                                   s_RVALID_i,
    // -- To Read Address channel
    input   [TRANS_SLV_ID_W-1:0]            AR_AxID_i,
    input                                   AR_crossing_flag_i,
    input                                   AR_shift_en_i,
    
    // Output declaration
    // -- To Dispatcher
    // ---- Read data channel (master)
    output  [TRANS_MST_ID_W*MST_AMT-1:0]    dsp_RID_o,
    output  [DATA_WIDTH*MST_AMT-1:0]        dsp_RDATA_o,
    output  [TRANS_WR_RESP_W*MST_AMT-1:0]   dsp_RRESP_o,
    output  [MST_AMT-1:0]                   dsp_RLAST_o,
    output  [MST_AMT-1:0]                   dsp_RVALID_o,
    // -- To slave (master interface of the interconnect)
    // ---- Read data channel
    output                                  s_RREADY_o,
    // To Write Address channel
    output                                  AR_stall_o
);

    // Local parameters initialization
    localparam FILTER_INFO_W    = TRANS_SLV_ID_W + 1; // crossing flag + transaction ID
    localparam R_INFO_W         = TRANS_SLV_ID_W + DATA_WIDTH + TRANS_WR_RESP_W + 1;
    
    // Internal variable declaration 
    genvar mst_idx;
    // Internal signal declaration
    // -- wire declaration
    // ---- FIFO RLAST filter
    wire    [FILTER_INFO_W-1:0]     filter_info;
    wire    [FILTER_INFO_W-1:0]     filter_info_valid;
    wire                            fifo_filter_wr_en;
    wire                            fifo_filter_rd_en;
    wire                            fifo_filter_full;
    wire                            fifo_filter_empty;
    // ---- Write response filter
    wire    [TRANS_SLV_ID_W-1:0]    ARID_valid;
    wire                            crossing_flag_valid;
    wire                            filter_ARID_match;
    wire                            filter_condition;
    wire                            filter_RLAST;
    // -- Handshake detector
    wire                            slv_handshake_occur;
    // -- Master mapping
    wire    [MST_ID_W-1:0]          mst_id;
    wire                            dsp_RREADY_valid;
    // -- Slave skid buffer
    wire    [R_INFO_W-1:0]          ssb_bwd_data;
    wire                            ssb_bwd_valid;
    wire                            ssb_bwd_ready;
    wire    [R_INFO_W-1:0]          ssb_fwd_data;
    wire                            ssb_fwd_valid;
    wire                            ssb_fwd_ready;
    wire    [TRANS_SLV_ID_W-1:0]    ssb_fwd_RID;
    wire    [DATA_WIDTH-1:0]        ssb_fwd_RDATA;
    wire    [TRANS_WR_RESP_W-1:0]   ssb_fwd_RRESP;
    wire                            ssb_fwd_RLAST;
    
    // Module
    // -- FIFO WRESP ordering
    sync_fifo 
    #(
        .FIFO_TYPE(0),
        .DATA_WIDTH(FILTER_INFO_W),
        .FIFO_DEPTH(OUTSTANDING_AMT)
    ) fifo_wresp_filter (
        .clk(ACLK_i),
        .rst_n(ARESETn_i),
        .data_i(filter_info),
        .data_o(filter_info_valid),
        .wr_valid_i(fifo_filter_wr_en),
        .rd_valid_i(fifo_filter_rd_en),
        .empty_o(fifo_filter_empty),
        .full_o(fifo_filter_full),
        .wr_ready_o(),
        .rd_ready_o(),
        .almost_empty_o(),
        .almost_full_o(),
        .counter()
    );
    // Slave skid buffer (pipelined in/out)
    skid_buffer #(
        .SBUF_TYPE(1),
        .DATA_WIDTH(R_INFO_W)
    ) slv_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (ssb_bwd_data),
        .bwd_valid_i(ssb_bwd_valid),
        .fwd_ready_i(ssb_fwd_ready),
        .fwd_data_o (ssb_fwd_data),
        .bwd_ready_o(ssb_bwd_ready),
        .fwd_valid_o(ssb_fwd_valid)
    );
    // Combinational logic
    // -- FIFO WRESP filter
    assign filter_info = {AR_crossing_flag_i, AR_AxID_i};
    assign fifo_filter_wr_en = AR_shift_en_i;
    assign fifo_filter_rd_en = slv_handshake_occur & ssb_fwd_RLAST;
    // -- Write response filter
    assign {crossing_flag_valid, ARID_valid} = filter_info_valid;
    assign filter_ARID_match = ARID_valid == ssb_fwd_RID;
    assign filter_condition = filter_ARID_match & (~fifo_filter_empty) & crossing_flag_valid;
    assign filter_RLAST = ssb_fwd_RLAST & ~filter_condition;
    // -- Handshake detector
    assign slv_handshake_occur = ssb_fwd_valid & ssb_fwd_ready;
    // -- Master mapping
    assign mst_id = ssb_fwd_RID[(TRANS_SLV_ID_W-1)-:MST_ID_W];
    // -- Slave Output
    assign s_RREADY_o = ssb_bwd_ready;
    // -- Dispatcher Output
    generate
        for(mst_idx = 0; mst_idx < MST_AMT; mst_idx = mst_idx + 1) begin : MST_LOGIC
            assign dsp_RVALID_o[mst_idx] = (mst_id == mst_idx) & ssb_fwd_valid;
            assign dsp_RID_o[TRANS_MST_ID_W*(mst_idx+1)-1-:TRANS_MST_ID_W] = ssb_fwd_RID[TRANS_MST_ID_W-1:0];
            assign dsp_RDATA_o[DATA_WIDTH*(mst_idx+1)-1-:DATA_WIDTH] = ssb_fwd_RDATA;
            assign dsp_RRESP_o[TRANS_WR_RESP_W*(mst_idx+1)-1-:TRANS_WR_RESP_W] = ssb_fwd_RRESP;
            assign dsp_RLAST_o[mst_idx] = filter_RLAST;
        end
    endgenerate
    // -- Slave skid buffer
    assign ssb_bwd_data     = {s_RID_i, s_RDATA_i, s_RRESP_i, s_RLAST_i};
    assign ssb_bwd_valid    = s_RVALID_i;
    assign ssb_fwd_ready    = dsp_RREADY_i[mst_id];
    assign {ssb_fwd_RID, ssb_fwd_RDATA, ssb_fwd_RRESP, ssb_fwd_RLAST} = ssb_fwd_data;
    // -- Write Address channel Output
    assign AR_stall_o = fifo_filter_full;

endmodule

module sa_W_channel
#(
    // Interconnect configuration
    parameter                       MST_AMT             = 3,
    parameter                       OUTSTANDING_AMT     = 8,
    parameter                       MST_ID_W            = $clog2(MST_AMT),
    // Transaction configuration
    parameter                       DATA_WIDTH          = 32,
    parameter                       ADDR_WIDTH          = 32,
    parameter                       TRANS_DATA_LEN_W    = 8                                 // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
)
(
    // Input declaration
    // -- Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // -- To Dispatcher
    // ---- Write data channel
    input   [DATA_WIDTH*MST_AMT-1:0]        dsp_WDATA_i,
    input   [(DATA_WIDTH/8)*MST_AMT-1:0]    dsp_WSTRB_i,
    input   [MST_AMT-1:0]                   dsp_WLAST_i,
    input   [MST_AMT-1:0]                   dsp_WVALID_i,
    // ---- Control
    input   [MST_AMT-1:0]                   dsp_slv_sel_i,
    // -- To slave (master interface of the interconnect)
    // ---- Write data channel (master)
    input                                   s_WREADY_i,
    // -- To Write Address channel
    input   [MST_ID_W-1:0]                  AW_mst_id_i,
    input   [TRANS_DATA_LEN_W-1:0]          AW_AxLEN_i,
    input                                   AW_fifo_order_wr_en_i,
    
    // Output declaration
    // -- To Dispatcher
    // ---- Write data channel (master)
    output  [MST_AMT-1:0]                   dsp_WREADY_o,
    // -- To slave (master interface of the interconnect)
    // ---- Write data channel
    output  [DATA_WIDTH-1:0]                s_WDATA_o,
    output  [DATA_WIDTH/8-1:0]              s_WSTRB_o,
    output                                  s_WLAST_o,
    output                                  s_WVALID_o,
    // -- To Ax channel
    output                                  AW_stall_o      // stall shift_en of xADDR channel
);
    // Local parameters declaration
    localparam STRB_WIDTH   = DATA_WIDTH / 8;
    localparam WLAST_W      = 1;
    localparam DATA_INFO_W  = DATA_WIDTH + STRB_WIDTH;
    localparam ADDR_INFO_W  = MST_ID_W + TRANS_DATA_LEN_W;
    localparam W_INFO_W     = DATA_WIDTH + STRB_WIDTH + 1;
    // Internal variable declaration
    genvar mst_idx;
    
    // Internal signal declaration
    // Wire declaration
    // -- FIFO WDATA ordering
    wire    [ADDR_INFO_W-1:0]       ADDR_info;
    wire    [ADDR_INFO_W-1:0]       ADDR_info_valid;
    wire                            fifo_order_rd_en;
    wire                            fifo_order_full;
    wire                            fifo_order_empt;
    wire    [MST_ID_W-1:0]          Ax_mst_id_valid;       
    wire    [TRANS_DATA_LEN_W-1:0]  Ax_AxLEN_valid;
    wire                            mst_sel                 [MST_AMT-1:0];
    // -- FIFO WDATA in                                              
    wire    [DATA_INFO_W-1:0]       DATA_info               [MST_AMT-1:0];
    wire    [DATA_INFO_W-1:0]       DATA_info_valid         [MST_AMT-1:0];
    wire                            fifo_wdata_wr_en        [MST_AMT-1:0];
    wire                            fifo_wdata_rd_en        [MST_AMT-1:0];
    wire                            fifo_wdata_full         [MST_AMT-1:0];
    wire                            fifo_wdata_empt         [MST_AMT-1:0];
    wire                            dsp_WVALID_dec          [MST_AMT-1:0];
    wire    [DATA_WIDTH-1:0]        dsp_WDATA_valid         [MST_AMT-1:0];
    wire    [STRB_WIDTH-1:0]        dsp_WSTRB_valid         [MST_AMT-1:0];
    wire                            dsp_WLAST_valid         [MST_AMT-1:0];
    // -- Handshake detector
    wire                            dsp_handshake_occur     [MST_AMT-1:0];
    wire                            slv_handshake_occur;
    // -- Master MUX
    wire    [DATA_WIDTH-1:0]        s_WDATA_o_nxt;
    wire    [STRB_WIDTH-1:0]        s_WSTRB_o_nxt;
    wire                            s_WLAST_o_nxt;
    wire                            s_WVALID_o_nxt;
    // -- Booting condition
    wire                            transaction_en;
    // -- Transaction booter
    wire                            transaction_boot;
    // -- Transfer counter
    wire    [TRANS_DATA_LEN_W-1:0]  transfer_ctn_nxt;
    wire    [TRANS_DATA_LEN_W-1:0]  transfer_ctn_incr;
    wire                            shift_en_trans_ctn;
    // -- Output control 
    wire                            WDATA_channel_shift_en;
    // -- Slave skid buffer
    wire    [W_INFO_W-1:0]          ssb_bwd_data;
    wire                            ssb_bwd_valid;
    wire                            ssb_bwd_ready;
    wire    [W_INFO_W-1:0]          ssb_fwd_data;
    wire                            ssb_fwd_valid;
    wire                            ssb_fwd_ready;
    wire    [DATA_WIDTH-1:0]        ssb_fwd_WDATA;
    wire    [STRB_WIDTH-1:0]        ssb_fwd_WSTRB;
    wire                            ssb_fwd_WLAST;
    
    // Reg declaration
    // -- Output control
    reg     [DATA_WIDTH-1:0]        s_WDATA_o_r;
    reg     [STRB_WIDTH-1:0]        s_WSTRB_o_r;
    reg                             s_WLAST_o_r;
    reg                             s_WVALID_o_r;
    // -- Transfer counter
    reg     [TRANS_DATA_LEN_W-1:0]  transfer_ctn_r;
    sync_fifo 
    #(
        .FIFO_TYPE(0),
        .DATA_WIDTH(ADDR_INFO_W),
        .FIFO_DEPTH(OUTSTANDING_AMT)
    ) fifo_wdata_order (
        .clk(ACLK_i),
        .rst_n(ARESETn_i),
        .data_i(ADDR_info),
        .data_o(ADDR_info_valid),
        .wr_valid_i(AW_fifo_order_wr_en_i),
        .rd_valid_i(fifo_order_rd_en),
        .empty_o(fifo_order_empt),
        .full_o(fifo_order_full),
        .wr_ready_o(),
        .rd_ready_o(),
        .almost_empty_o(),
        .almost_full_o(),
        .counter()
    );
    // Slave skid buffer (pipelined in/out)
    skid_buffer #(
        .SBUF_TYPE(3),
        .DATA_WIDTH(W_INFO_W)
    ) slv_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (ssb_bwd_data),
        .bwd_valid_i(ssb_bwd_valid),
        .fwd_ready_i(ssb_fwd_ready),
        .fwd_data_o (ssb_fwd_data),
        .bwd_ready_o(ssb_bwd_ready),
        .fwd_valid_o(ssb_fwd_valid)
    );
//    onehot_encoder #(
//        .INPUT_W(MST_AMT),
//        .OUTPUT_W(MST_ID_W)
//    ) master_id_encoder (
//        .i(Ax_mst_id_valid),
//        .o(Ax_mst_id_valid)
//    );

//    edgedet #(
//        .RISING_EDGE(1'b1)
//    )transaction_booter(
//        .clk(ACLK_i),
//        .i(transaction_en),
//        .en(1'b1),
//        .o(transaction_boot),
//        .rst_n(ARESETn_i)
//    );    
    ////////////////////////////////////////////////////////////////////////////
    assign transaction_boot = (~s_WVALID_o_r) & s_WVALID_o_nxt;
    ////////////////////////////////////////////////////////////////////////////
    
    generate
        for(mst_idx = 0; mst_idx < MST_AMT; mst_idx = mst_idx + 1) begin : MST_W_FIFO
            sync_fifo 
            #(
                .FIFO_TYPE(0),
                .DATA_WIDTH(DATA_INFO_W),
                .FIFO_DEPTH(32)
            ) fifo_wdata (
                .clk(ACLK_i),
                .rst_n(ARESETn_i),
                .data_i(DATA_info[mst_idx]),
                .data_o(DATA_info_valid[mst_idx]),
                .wr_valid_i(fifo_wdata_wr_en[mst_idx]),
                .rd_valid_i(fifo_wdata_rd_en[mst_idx]),
                .empty_o(fifo_wdata_empt[mst_idx]),
                .full_o(fifo_wdata_full[mst_idx]),
                .wr_ready_o(),
                .rd_ready_o(),
                .almost_empty_o(),
                .almost_full_o(),
                .counter()
            );
        end
    endgenerate
    
    // Combinational logic
    // -- FIFO WDATA ordering
    assign ADDR_info                            = {AW_mst_id_i, AW_AxLEN_i};
    assign {Ax_mst_id_valid, Ax_AxLEN_valid}    = ADDR_info_valid;
    assign fifo_order_rd_en                     = s_WLAST_o_nxt & shift_en_trans_ctn;
    generate
        for(mst_idx = 0; mst_idx < MST_AMT; mst_idx = mst_idx + 1) begin : MST_LOGIC
            // Onehot decoder - Master ID
            assign mst_sel[mst_idx]             = Ax_mst_id_valid == mst_idx;
            // FIFO WDATA
            assign DATA_info[mst_idx]           = {dsp_WDATA_i[DATA_WIDTH*(mst_idx+1)-1-:DATA_WIDTH], dsp_WSTRB_i[STRB_WIDTH*(mst_idx+1)-1-:STRB_WIDTH]};
            assign {dsp_WDATA_valid[mst_idx], dsp_WSTRB_valid[mst_idx]} = DATA_info_valid[mst_idx];
            assign dsp_WVALID_dec[mst_idx]      = dsp_WVALID_i[mst_idx] & dsp_slv_sel_i[mst_idx];
            assign fifo_wdata_wr_en[mst_idx]    = dsp_handshake_occur[mst_idx];
            assign fifo_wdata_rd_en[mst_idx]    = mst_sel[mst_idx] & WDATA_channel_shift_en & transaction_en;
            // Handshake detector
            assign dsp_handshake_occur[mst_idx] = dsp_WVALID_dec[mst_idx] & dsp_WREADY_o[mst_idx];
            // Dispatcher Interface 
            assign dsp_WREADY_o[mst_idx]        = ~fifo_wdata_full[mst_idx];
        end
    endgenerate
    // Booting condition 
    assign transaction_en           = ~fifo_wdata_empt[Ax_mst_id_valid] & ~fifo_order_empt;
    // Transfer counter
    assign shift_en_trans_ctn       = transaction_en & WDATA_channel_shift_en;
    assign transfer_ctn_incr        = transfer_ctn_r + 1'b1;
    assign transfer_ctn_nxt         = (Ax_AxLEN_valid == transfer_ctn_r) ? {TRANS_DATA_LEN_W{1'b0}} : transfer_ctn_incr;
    // Handshake detector
    assign slv_handshake_occur      = ssb_bwd_valid & ssb_bwd_ready;
    // Output control 
    assign WDATA_channel_shift_en   = transaction_boot | slv_handshake_occur;
    assign AW_stall_o               = fifo_order_full;
    assign s_WVALID_o_nxt           = transaction_en;
    assign s_WLAST_o_nxt            = (Ax_AxLEN_valid == transfer_ctn_r) & transaction_en;
    assign s_WSTRB_o_nxt            = dsp_WSTRB_valid[Ax_mst_id_valid];
    assign s_WDATA_o_nxt            = dsp_WDATA_valid[Ax_mst_id_valid];
    assign s_WVALID_o               = ssb_fwd_valid;
    assign s_WLAST_o                = ssb_fwd_WLAST;
    assign s_WSTRB_o                = ssb_fwd_WSTRB;
    assign s_WDATA_o                = ssb_fwd_WDATA;
    // -- Slave skid buffer
    assign ssb_bwd_data             = {s_WDATA_o_r, s_WSTRB_o_r, s_WLAST_o_r};
    assign ssb_bwd_valid            = s_WVALID_o_r;
    assign ssb_fwd_ready            = s_WREADY_i;
    assign {ssb_fwd_WDATA, ssb_fwd_WSTRB, ssb_fwd_WLAST}   = ssb_fwd_data;
    
    // Flip-flop logic
    always @(posedge ACLK_i) begin
        if(~ARESETn_i) begin
            transfer_ctn_r <= {TRANS_DATA_LEN_W{1'b0}};
        end
        else if(shift_en_trans_ctn) begin
            transfer_ctn_r <= transfer_ctn_nxt;
        end
    end
    
    always @(posedge ACLK_i) begin
        if(~ARESETn_i) begin
            s_WVALID_o_r <= 1'b0;
            s_WLAST_o_r <= 1'b0;
            s_WSTRB_o_r <= {STRB_WIDTH{1'b0}};
            s_WDATA_o_r <= {DATA_WIDTH{1'b0}};
        end
        else if(WDATA_channel_shift_en) begin
            s_WVALID_o_r <= s_WVALID_o_nxt;
            s_WLAST_o_r <= s_WLAST_o_nxt;
            s_WSTRB_o_r <= s_WSTRB_o_nxt;
            s_WDATA_o_r <= s_WDATA_o_nxt;
        end
    end

endmodule

module sa_B_channel 
#(
    // Interconnect configuration
    parameter MST_AMT           = 3,
    parameter OUTSTANDING_AMT   = 8,
    parameter MST_ID_W          = $clog2(MST_AMT),
    // Transaction configuration
    parameter TRANS_MST_ID_W    = 5,                                // Width of master transaction ID 
    parameter TRANS_SLV_ID_W    = TRANS_MST_ID_W + $clog2(MST_AMT), // Width of slave transaction ID
    parameter TRANS_WR_RESP_W   = 2
)
(
    // Input declaration
    // -- Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // -- To Dispatcher
    // ---- Write response channel
    input   [MST_AMT-1:0]                   dsp_BREADY_i,
    // -- To slave (master interface of the interconnect)
    // ---- Write response channel (master)
    input   [TRANS_SLV_ID_W-1:0]            s_BID_i,
    input   [TRANS_WR_RESP_W-1:0]           s_BRESP_i,
    input                                   s_BVALID_i,
    // -- To Write Address channel
    input   [TRANS_SLV_ID_W-1:0]            AW_AxID_i,
    input                                   AW_crossing_flag_i,
    input                                   AW_shift_en_i,
    // Output declaration
    // -- To Dispatcher
    // ---- Write response channel (master)
    output  [TRANS_MST_ID_W*MST_AMT-1:0]    dsp_BID_o,
    output  [TRANS_WR_RESP_W*MST_AMT-1:0]   dsp_BRESP_o,
    output  [MST_AMT-1:0]                   dsp_BVALID_o,
    // -- To slave (master interface of the interconnect)
    // ---- Write response channel          
    output                                  s_BREADY_o,
    // To Write Address channel
    output                                  AW_stall_o
);
    // Local parameters initialization
    localparam FILTER_INFO_W    = TRANS_SLV_ID_W + 1;  // crossing flag + Transaction ID
    localparam B_INFO_W         = TRANS_SLV_ID_W + TRANS_WR_RESP_W;
    
    // Internal variable declaration 
    genvar mst_idx;
    // Internal signal declaration
    // -- wire declaration
    // ---- FIFO WRESP filter
    wire    [FILTER_INFO_W-1:0]     filter_info;
    wire    [FILTER_INFO_W-1:0]     filter_info_valid;
    wire                            fifo_filter_wr_en;
    wire                            fifo_filter_rd_en;
    wire                            fifo_filter_full;
    wire                            fifo_filter_empty;
    // ---- Write response filter
    wire    [TRANS_SLV_ID_W-1:0]    AWID_valid;
    wire                            crossing_flag_valid;
    wire                            filter_AWID_match;
    wire                            filter_condition;
    wire                            filter_BVALID;
    wire                            filter_BREADY_gen;
    // -- Handshake detector
    wire                            slv_handshake_occur;
    // -- Master mapping
    wire    [MST_ID_W-1:0]          mst_id;
    wire                            dsp_BREADY_valid;
    // -- Slave skid buffer
    wire    [B_INFO_W-1:0]          ssb_bwd_data;
    wire                            ssb_bwd_valid;
    wire                            ssb_bwd_ready;
    wire    [B_INFO_W-1:0]          ssb_fwd_data;
    wire                            ssb_fwd_valid;
    wire                            ssb_fwd_ready;
    wire    [TRANS_SLV_ID_W-1:0]    ssb_fwd_BID;
    wire    [TRANS_WR_RESP_W-1:0]   ssb_fwd_BRESP;
    // Module
    // -- FIFO WRESP ordering
    sync_fifo 
    #(
        .FIFO_TYPE(0),
        .DATA_WIDTH(FILTER_INFO_W),
        .FIFO_DEPTH(OUTSTANDING_AMT)
    ) fifo_wresp_filter (
        .clk(ACLK_i),
        .rst_n(ARESETn_i),
        .data_i(filter_info),
        .data_o(filter_info_valid),
        .wr_valid_i(fifo_filter_wr_en),
        .rd_valid_i(fifo_filter_rd_en),
        .empty_o(fifo_filter_empty),
        .full_o(fifo_filter_full),
        .wr_ready_o(),
        .rd_ready_o(),
        .almost_empty_o(),
        .almost_full_o(),
        .counter()
    );
    // Slave skid buffer (pipelined in/out)
    skid_buffer #(
        .SBUF_TYPE(1),
        .DATA_WIDTH(B_INFO_W)
    ) slv_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (ssb_bwd_data),
        .bwd_valid_i(ssb_bwd_valid),
        .fwd_ready_i(ssb_fwd_ready),
        .fwd_data_o (ssb_fwd_data),
        .bwd_ready_o(ssb_bwd_ready),
        .fwd_valid_o(ssb_fwd_valid)
    );
    // Combinational logic
    // -- FIFO WRESP filter
    assign filter_info = {AW_crossing_flag_i, AW_AxID_i};
    // assign fifo_filter_wr_en = AW_shift_en_i & AW_crossing_flag_i;
    // assign fifo_filter_rd_en = slv_handshake_occur & filter_condition;
    assign fifo_filter_wr_en = AW_shift_en_i;
    assign fifo_filter_rd_en = slv_handshake_occur; // Design constraint: slave must return BRESP in AW issue order
    // -- Write response filter
    assign {crossing_flag_valid, AWID_valid} = filter_info_valid;
    assign filter_AWID_match = (AWID_valid == ssb_fwd_BID) & crossing_flag_valid;
    assign filter_condition = filter_AWID_match & ~fifo_filter_empty;
    assign filter_BVALID = ssb_fwd_valid & ~filter_condition;
    assign filter_BREADY_gen = dsp_BREADY_valid | filter_condition;
    // -- Handshake detector
    assign slv_handshake_occur = ssb_fwd_valid & ssb_fwd_ready;
    // -- Master mapping
    assign mst_id = ssb_fwd_BID[(TRANS_SLV_ID_W-1)-:MST_ID_W];
    // -- Slave Output
    assign s_BREADY_o = ssb_bwd_ready;
    // -- Dispatcher Output
    assign dsp_BREADY_valid = dsp_BREADY_i[mst_id];
    generate
        for(mst_idx = 0; mst_idx < MST_AMT; mst_idx = mst_idx + 1) begin : MST_LOGIC
            assign dsp_BVALID_o[mst_idx] = (mst_id == mst_idx) & filter_BVALID;
            assign dsp_BID_o[TRANS_MST_ID_W*(mst_idx+1)-1-:TRANS_MST_ID_W] = ssb_fwd_BID[TRANS_MST_ID_W-1:0];
            assign dsp_BRESP_o[TRANS_WR_RESP_W*(mst_idx+1)-1-:TRANS_WR_RESP_W] = ssb_fwd_BRESP;
        end
    endgenerate
    // -- Slave skid buffer
    assign ssb_bwd_data     = {s_BID_i, s_BRESP_i};
    assign ssb_bwd_valid    = s_BVALID_i;
    assign ssb_fwd_ready    = filter_BREADY_gen;
    assign {ssb_fwd_BID, ssb_fwd_BRESP} = ssb_fwd_data;
    // -- Write Address channel Output
    assign AW_stall_o = fifo_filter_full;
    
endmodule

module ai_slave_arbitration
#(
    // Interconnect configuration
    parameter                       MST_AMT             = 4,
    parameter                       OUTSTANDING_AMT     = 8,
    parameter [0:(MST_AMT*32)-1]    MST_WEIGHT          = {32'd5, 32'd3, 32'd2, 32'd1},
    parameter                       MST_ID_W            = $clog2(MST_AMT),
    // Transaction configuration
    parameter                       DATA_WIDTH          = 32,
    parameter                       ADDR_WIDTH          = 32,
    parameter                       TRANS_MST_ID_W      = 5,                                // Bus width of master transaction ID 
    parameter                       TRANS_SLV_ID_W      = TRANS_MST_ID_W + $clog2(MST_AMT), // Bus width of slave transaction ID
    parameter                       TRANS_BURST_W       = 2,                                // Width of xBURST 
    parameter                       TRANS_DATA_LEN_W    = 8,                                // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter                       TRANS_DATA_SIZE_W   = 3,                                // Bus width of xSIZE
    parameter                       TRANS_WR_RESP_W     = 2,
    // Slave info configuration
    parameter [31:0] SLV_BASE_ADDR = 32'h0,
    parameter [31:0] SLV_ADDR_MASK = 32'h0
)
(
    // Input declaration
    // Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // To Dispatcher
    // Write address channel
    input   [TRANS_MST_ID_W*MST_AMT-1:0]    dsp_AWID_i,
    input   [ADDR_WIDTH*MST_AMT-1:0]        dsp_AWADDR_i,
    input   [TRANS_BURST_W*MST_AMT-1:0]     dsp_AWBURST_i,
    input   [TRANS_DATA_LEN_W*MST_AMT-1:0]  dsp_AWLEN_i,
    input   [TRANS_DATA_SIZE_W*MST_AMT-1:0] dsp_AWSIZE_i,
    input   [MST_AMT-1:0]                   dsp_AWLOCK_i,
    input   [4*MST_AMT-1:0]                 dsp_AWCACHE_i,
    input   [3*MST_AMT-1:0]                 dsp_AWPROT_i,
    input   [4*MST_AMT-1:0]                 dsp_AWQOS_i,
    input   [4*MST_AMT-1:0]                 dsp_AWREGION_i,
    input   [MST_AMT-1:0]                   dsp_AWVALID_i,
    input   [MST_AMT-1:0]                   dsp_AW_outst_full_i,
    // Write data channel
    input   [DATA_WIDTH*MST_AMT-1:0]        dsp_WDATA_i,
    input   [(DATA_WIDTH/8)*MST_AMT-1:0]    dsp_WSTRB_i,
    input   [MST_AMT-1:0]                   dsp_WLAST_i,
    input   [MST_AMT-1:0]                   dsp_WVALID_i,
    input   [MST_AMT-1:0]                   dsp_slv_sel_i,
    // Write response channel
    input   [MST_AMT-1:0]                   dsp_BREADY_i,
    // Read address channel
    input   [TRANS_MST_ID_W*MST_AMT-1:0]    dsp_ARID_i,
    input   [ADDR_WIDTH*MST_AMT-1:0]        dsp_ARADDR_i,
    input   [TRANS_BURST_W*MST_AMT-1:0]     dsp_ARBURST_i,
    input   [TRANS_DATA_LEN_W*MST_AMT-1:0]  dsp_ARLEN_i,
    input   [TRANS_DATA_SIZE_W*MST_AMT-1:0] dsp_ARSIZE_i,
    input   [MST_AMT-1:0]                   dsp_ARLOCK_i,
    input   [4*MST_AMT-1:0]                 dsp_ARCACHE_i,
    input   [3*MST_AMT-1:0]                 dsp_ARPROT_i,
    input   [4*MST_AMT-1:0]                 dsp_ARQOS_i,
    input   [4*MST_AMT-1:0]                 dsp_ARREGION_i,
    input   [MST_AMT-1:0]                   dsp_ARVALID_i,
    input   [MST_AMT-1:0]                   dsp_AR_outst_full_i,
    // Read data channel
    input   [MST_AMT-1:0]                   dsp_RREADY_i,
    // To slave (master interface of the interconnect)
    // Write address channel (master)
    input                                   s_AWREADY_i,
    // Write data channel (master)
    input                                   s_WREADY_i,
    // Write response channel (master)
    input  [TRANS_SLV_ID_W-1:0]             s_BID_i,
    input  [TRANS_WR_RESP_W-1:0]            s_BRESP_i,
    input                                   s_BVALID_i,
    // Read address channel (master)
    input                                   s_ARREADY_i,
    // Read data channel (master)
    input  [TRANS_SLV_ID_W-1:0]             s_RID_i,
    input  [DATA_WIDTH-1:0]                 s_RDATA_i,
    input  [TRANS_WR_RESP_W-1:0]            s_RRESP_i,
    input                                   s_RLAST_i,
    input                                   s_RVALID_i,
    
    // Output declaration
    // To Dispatcher
    // Write address channel (master)
    output  [MST_AMT-1:0]                   dsp_AWREADY_o,
    // Write data channel (master)
    output  [MST_AMT-1:0]                   dsp_WREADY_o,
    // Write response channel (master)
    output  [TRANS_MST_ID_W*MST_AMT-1:0]    dsp_BID_o,
    output  [TRANS_WR_RESP_W*MST_AMT-1:0]   dsp_BRESP_o,
    output  [MST_AMT-1:0]                   dsp_BVALID_o,
    // Read address channel (master)
    output  [MST_AMT-1:0]                   dsp_ARREADY_o,
    // Read data channel (master)
    output  [TRANS_MST_ID_W*MST_AMT-1:0]    dsp_RID_o,
    output  [DATA_WIDTH*MST_AMT-1:0]        dsp_RDATA_o,
    output  [TRANS_WR_RESP_W*MST_AMT-1:0]   dsp_RRESP_o,
    output  [MST_AMT-1:0]                   dsp_RLAST_o,
    output  [MST_AMT-1:0]                   dsp_RVALID_o,
    // To slave (master interface of the interconnect)
    // Write address channel
    output  [TRANS_SLV_ID_W-1:0]            s_AWID_o,
    output  [ADDR_WIDTH-1:0]                s_AWADDR_o,
    output  [TRANS_BURST_W-1:0]             s_AWBURST_o,
    output  [TRANS_DATA_LEN_W-1:0]          s_AWLEN_o,
    output  [TRANS_DATA_SIZE_W-1:0]         s_AWSIZE_o,
    output                                  s_AWLOCK_o,
    output  [3:0]                            s_AWCACHE_o,
    output  [2:0]                            s_AWPROT_o,
    output  [3:0]                            s_AWQOS_o,
    output  [3:0]                            s_AWREGION_o,
    output                                  s_AWVALID_o,
    // Write data channel
    output  [DATA_WIDTH-1:0]                s_WDATA_o,
    output  [DATA_WIDTH/8-1:0]              s_WSTRB_o,
    output                                  s_WLAST_o,
    output                                  s_WVALID_o,
    // Write response channel          
    output                                  s_BREADY_o,
    // Read address channel            
    output  [TRANS_SLV_ID_W-1:0]            s_ARID_o,
    output  [ADDR_WIDTH-1:0]                s_ARADDR_o,
    output  [TRANS_BURST_W-1:0]             s_ARBURST_o,
    output  [TRANS_DATA_LEN_W-1:0]          s_ARLEN_o,
    output  [TRANS_DATA_SIZE_W-1:0]         s_ARSIZE_o,
    output                                  s_ARLOCK_o,
    output  [3:0]                            s_ARCACHE_o,
    output  [2:0]                            s_ARPROT_o,
    output  [3:0]                            s_ARQOS_o,
    output  [3:0]                            s_ARREGION_o,
    output                                  s_ARVALID_o,
    // Read data channel
    output                                  s_RREADY_o
);
    // Internal signal declaration
    // Wire declaration
    // Write channel
    wire    [TRANS_SLV_ID_W-1:0]            AWID_valid_nxt;
    wire    [TRANS_DATA_LEN_W-1:0]          AWLEN_valid_nxt;
    wire    [MST_ID_W-1:0]                  AW_mst_valid_nxt;
    wire                                    AW_crossing_flag;
    wire                                    AW_shift_en;
    wire                                    AW_stall;
    wire                                    AW_stall_WDATA;
    wire                                    AW_stall_WRESP;
    //Read channel
    wire    [TRANS_SLV_ID_W-1:0]            ARID_valid_nxt;
    wire                                    AR_crossing_flag;
    wire                                    AR_shift_en;
    wire                                    AR_stall;
    wire                                    AR_stall_RDATA;
    
    // Combinational logic
    assign AW_stall = AW_stall_WDATA | AW_stall_WRESP;
    assign AR_stall = AR_stall_RDATA;
    
    // Module
    // Write channel
    // Write Address channel
    sa_Ax_channel #(
        .MST_AMT(MST_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .MST_WEIGHT(MST_WEIGHT),
        .MST_ID_W(MST_ID_W),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_SLV_ID_W(TRANS_SLV_ID_W),
        .TRANS_BURST_W(TRANS_BURST_W),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W(TRANS_DATA_SIZE_W),
        .SLV_BASE_ADDR(SLV_BASE_ADDR),
        .SLV_ADDR_MASK(SLV_ADDR_MASK)
    ) AW_channel (
        // Input
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .xDATA_stall_i(AW_stall),
        .dsp_AxID_i(dsp_AWID_i),
        .dsp_AxADDR_i(dsp_AWADDR_i),
        .dsp_AxBURST_i(dsp_AWBURST_i),
        .dsp_AxLEN_i(dsp_AWLEN_i),
        .dsp_AxSIZE_i(dsp_AWSIZE_i),
        .dsp_AxLOCK_i(dsp_AWLOCK_i),
        .dsp_AxCACHE_i(dsp_AWCACHE_i),
        .dsp_AxPROT_i(dsp_AWPROT_i),
        .dsp_AxQOS_i(dsp_AWQOS_i),
        .dsp_AxREGION_i(dsp_AWREGION_i),
        .dsp_AxVALID_i(dsp_AWVALID_i),
        .dsp_dispatcher_full_i(dsp_AW_outst_full_i),
        .s_AxREADY_i(s_AWREADY_i),
        // Output
        .dsp_AxREADY_o(dsp_AWREADY_o),
        .s_AxID_o(s_AWID_o),
        .s_AxADDR_o(s_AWADDR_o),
        .s_AxBURST_o(s_AWBURST_o),
        .s_AxLEN_o(s_AWLEN_o),
        .s_AxSIZE_o(s_AWSIZE_o),
        .s_AxLOCK_o(s_AWLOCK_o),
        .s_AxCACHE_o(s_AWCACHE_o),
        .s_AxPROT_o(s_AWPROT_o),
        .s_AxQOS_o(s_AWQOS_o),
        .s_AxREGION_o(s_AWREGION_o),
        .s_AxVALID_o(s_AWVALID_o),
        .xDATA_AxID_o(AWID_valid_nxt),
        .xDATA_mst_id_o(AW_mst_valid_nxt),
        .xDATA_crossing_flag_o(AW_crossing_flag),
        .xDATA_AxLEN_o(AWLEN_valid_nxt),
        .xDATA_fifo_order_wr_en_o(AW_shift_en)
    );
    // Write Data channel
    sa_W_channel #(
        .MST_AMT(MST_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .MST_ID_W(MST_ID_W),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W)
    ) W_channel (
        // Input
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .dsp_WDATA_i(dsp_WDATA_i),
        .dsp_WSTRB_i(dsp_WSTRB_i),
        .dsp_WLAST_i(dsp_WLAST_i),
        .dsp_WVALID_i(dsp_WVALID_i),
        .dsp_slv_sel_i(dsp_slv_sel_i),
        .s_WREADY_i(s_WREADY_i),
        .AW_mst_id_i(AW_mst_valid_nxt),
        .AW_AxLEN_i(AWLEN_valid_nxt),
        .AW_fifo_order_wr_en_i(AW_shift_en),
        // Output
        .dsp_WREADY_o(dsp_WREADY_o),
        .s_WDATA_o(s_WDATA_o),
        .s_WSTRB_o(s_WSTRB_o),
        .s_WLAST_o(s_WLAST_o),
        .s_WVALID_o(s_WVALID_o),
        .AW_stall_o(AW_stall_WDATA)
    );
    // Wire Response channel
    sa_B_channel #(
        .MST_AMT(MST_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .MST_ID_W(MST_ID_W),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_SLV_ID_W(TRANS_SLV_ID_W),
        .TRANS_WR_RESP_W(TRANS_WR_RESP_W)
    ) B_channel (
        // Input
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .dsp_BREADY_i(dsp_BREADY_i),
        .s_BID_i(s_BID_i),
        .s_BRESP_i(s_BRESP_i),
        .s_BVALID_i(s_BVALID_i),
        .AW_AxID_i(AWID_valid_nxt),
        .AW_crossing_flag_i(AW_crossing_flag),
        .AW_shift_en_i(AW_shift_en),
        // Output
        .dsp_BID_o(dsp_BID_o),
        .dsp_BRESP_o(dsp_BRESP_o),
        .dsp_BVALID_o(dsp_BVALID_o),
        .s_BREADY_o(s_BREADY_o),
        .AW_stall_o(AW_stall_WRESP)
    );
    // Read channel
    // Read Address channel
    sa_Ax_channel #(
        .MST_AMT(MST_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .MST_WEIGHT(MST_WEIGHT),
        .MST_ID_W(MST_ID_W),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_SLV_ID_W(TRANS_SLV_ID_W),
        .TRANS_BURST_W(TRANS_BURST_W),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W(TRANS_DATA_SIZE_W),
        .SLV_BASE_ADDR(SLV_BASE_ADDR),
        .SLV_ADDR_MASK(SLV_ADDR_MASK)
    ) AR_channel (
        // Input
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .xDATA_stall_i(AR_stall),
        .dsp_AxID_i(dsp_ARID_i),
        .dsp_AxADDR_i(dsp_ARADDR_i),
        .dsp_AxBURST_i(dsp_ARBURST_i),
        .dsp_AxLEN_i(dsp_ARLEN_i),
        .dsp_AxSIZE_i(dsp_ARSIZE_i),
        .dsp_AxLOCK_i(dsp_ARLOCK_i),
        .dsp_AxCACHE_i(dsp_ARCACHE_i),
        .dsp_AxPROT_i(dsp_ARPROT_i),
        .dsp_AxQOS_i(dsp_ARQOS_i),
        .dsp_AxREGION_i(dsp_ARREGION_i),
        .dsp_AxVALID_i(dsp_ARVALID_i),
        .dsp_dispatcher_full_i(dsp_AR_outst_full_i),
        .s_AxREADY_i(s_ARREADY_i),
        // Output
        .dsp_AxREADY_o(dsp_ARREADY_o),
        .s_AxID_o(s_ARID_o),
        .s_AxADDR_o(s_ARADDR_o),
        .s_AxBURST_o(s_ARBURST_o),
        .s_AxLEN_o(s_ARLEN_o),
        .s_AxSIZE_o(s_ARSIZE_o),
        .s_AxLOCK_o(s_ARLOCK_o),
        .s_AxCACHE_o(s_ARCACHE_o),
        .s_AxPROT_o(s_ARPROT_o),
        .s_AxQOS_o(s_ARQOS_o),
        .s_AxREGION_o(s_ARREGION_o),
        .s_AxVALID_o(s_ARVALID_o),
        .xDATA_AxID_o(ARID_valid_nxt),
        .xDATA_mst_id_o(),
        .xDATA_crossing_flag_o(AR_crossing_flag),
        .xDATA_AxLEN_o(),
        .xDATA_fifo_order_wr_en_o(AR_shift_en)
    );
    // Read Data channel 
    sa_R_channel #(
        .MST_AMT(MST_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .MST_ID_W(MST_ID_W),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_SLV_ID_W(TRANS_SLV_ID_W)
    ) R_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .dsp_RREADY_i(dsp_RREADY_i),
        .s_RID_i(s_RID_i),
        .s_RDATA_i(s_RDATA_i),
        .s_RRESP_i(s_RRESP_i),
        .s_RLAST_i(s_RLAST_i),
        .s_RVALID_i(s_RVALID_i),
        .AR_AxID_i(ARID_valid_nxt),
        .AR_crossing_flag_i(AR_crossing_flag),
        .AR_shift_en_i(AR_shift_en),
        .dsp_RID_o(dsp_RID_o),
        .dsp_RDATA_o(dsp_RDATA_o),
        .dsp_RRESP_o(dsp_RRESP_o),
        .dsp_RLAST_o(dsp_RLAST_o),
        .dsp_RVALID_o(dsp_RVALID_o),
        .s_RREADY_o(s_RREADY_o),
        .AR_stall_o(AR_stall_RDATA)
    );
    
endmodule
