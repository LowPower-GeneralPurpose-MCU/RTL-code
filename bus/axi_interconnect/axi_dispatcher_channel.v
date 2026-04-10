module dsp_Ax_channel
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    parameter OUTSTANDING_AMT   = 8,
    parameter OUTST_CTN_W       = $clog2(OUTSTANDING_AMT) + 1,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter TRANS_MST_ID_W    = 5,    // Bus width of master transaction ID 
    parameter TRANS_BURST_W     = 2,    // Width of xBURST 
    parameter TRANS_DATA_LEN_W  = 8,    // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter TRANS_DATA_SIZE_W = 3,    // Bus width of xSIZE
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT),
    parameter [SLV_AMT*32-1:0] SLV_BASE_ADDR = 0,
    parameter [SLV_AMT*32-1:0] SLV_ADDR_MASK = 0,
    // Timeout configuration
    parameter TIMEOUT_W         = 10    // Address-phase timeout width (2^W-1 cycles), 0 = disabled
)
(
    // Input declaration
    // Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // To Master (slave interface of the interconnect)
    // Write/Read address channel
    input   [TRANS_MST_ID_W-1:0]            m_AxID_i,
    input   [ADDR_WIDTH-1:0]                m_AxADDR_i,
    input   [TRANS_BURST_W-1:0]             m_AxBURST_i,
    input   [TRANS_DATA_LEN_W-1:0]          m_AxLEN_i,
    input   [TRANS_DATA_SIZE_W-1:0]         m_AxSIZE_i,
    input                                   m_AxLOCK_i,
    input   [3:0]                            m_AxCACHE_i,
    input   [2:0]                            m_AxPROT_i,
    input   [3:0]                            m_AxQOS_i,
    input   [3:0]                            m_AxREGION_i,
    input                                   m_AxVALID_i,
    // To xDATA channel Dispatcher
    input                                   m_xVALID_i,
    input                                   m_xREADY_i,
    // To Slave Arbitration
    // Write/Read address channel (master)
    input   [SLV_AMT-1:0]                   sa_AxREADY_i,
    // Output declaration
    // To Master (slave interface of interconnect)
    // Write/Read address channel (master)
    output                                  m_AxREADY_o,
    // To Slave Arbitration
    // Write/Read address channel
    output  [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_AxID_o,
    output  [ADDR_WIDTH*SLV_AMT-1:0]        sa_AxADDR_o,
    output  [TRANS_BURST_W*SLV_AMT-1:0]     sa_AxBURST_o,
    output  [TRANS_DATA_LEN_W*SLV_AMT-1:0]  sa_AxLEN_o,
    output  [TRANS_DATA_SIZE_W*SLV_AMT-1:0] sa_AxSIZE_o,
    output  [SLV_AMT-1:0]                   sa_AxLOCK_o,
    output  [4*SLV_AMT-1:0]                 sa_AxCACHE_o,
    output  [3*SLV_AMT-1:0]                 sa_AxPROT_o,
    output  [4*SLV_AMT-1:0]                 sa_AxQOS_o,
    output  [4*SLV_AMT-1:0]                 sa_AxREGION_o,
    output  [SLV_AMT-1:0]                   sa_AxVALID_o,
    output  [OUTST_CTN_W-1:0]               sa_Ax_outst_ctn_o,
    // To xDATA channel Dispatcher
    output  [SLV_ID_W-1:0]                  dsp_xDATA_slv_id_o,
    output                                  dsp_xDATA_disable_o,
    output                                  dsp_xDATA_err_o,
    output  [TRANS_MST_ID_W-1:0]            dsp_xDATA_axid_o,
    output                                  dsp_xDATA_last_o,
    // To WRESP channel Dispatcher
    output  [SLV_ID_W-1:0]                  dsp_WRESP_slv_id_o,
    output                                  dsp_WRESP_shift_en_o,
    output                                  dsp_WRESP_err_o,
    output  [TRANS_MST_ID_W-1:0]            dsp_WRESP_axid_o
);
    // Local parameters initialization
    localparam ADDR_INFO_W  = 1 + TRANS_MST_ID_W + SLV_ID_W + TRANS_DATA_LEN_W;
    localparam Ax_INFO_W    = TRANS_MST_ID_W + ADDR_WIDTH + TRANS_BURST_W + TRANS_DATA_LEN_W + TRANS_DATA_SIZE_W + 1 + 4 + 3 + 4 + 4; // +LOCK+CACHE+PROT+QOS+REGION
    // Internal variable declaration
    genvar slv_idx;
    
    // Internal signal declaration
    // xADDR order fifo
    wire    [ADDR_INFO_W-1:0]       addr_info;
    wire    [ADDR_INFO_W-1:0]       addr_info_valid;
    wire                            fifo_xa_order_wr_en;
    wire                            fifo_xa_order_rd_en;
    wire                            fifo_xa_order_empty;
    wire                            fifo_xa_order_full;
    
    // Handshake detector
    wire                            Ax_handshake_occur;
    wire                            xDATA_handshake_occur;
    
    // Misc
    // SỬA: Thay SLV_ID_MAP_W thành SLV_ID_W
    wire    [SLV_ID_W-1:0]          slv_id;
    wire    [SLV_AMT-1:0]           slv_sel;
    wire    [SLV_ID_W-1:0]          addr_slv_mapping;
    
    wire    [TRANS_DATA_LEN_W-1:0]  AxLEN_valid;
    wire    [TRANS_MST_ID_W-1:0]    AxID_valid;
    wire                            addr_err_valid;
    // Error / timeout
    wire                            addr_invalid;
    wire                            addr_err;
    wire                            normal_accept;
    wire                            timeout_expire;
    // Transfer counter
    wire    [TRANS_DATA_LEN_W-1:0]  transfer_ctn_nxt;
    wire    [TRANS_DATA_LEN_W-1:0]  transfer_ctn_incr;
    wire                            transfer_ctn_match;
    // Master skid buffer
    wire    [Ax_INFO_W-1:0]         msb_bwd_data;
    wire                            msb_bwd_valid;
    wire                            msb_bwd_ready;
    wire    [Ax_INFO_W-1:0]         msb_fwd_data;
    wire                            msb_fwd_valid;
    wire                            msb_fwd_ready;
    wire    [TRANS_MST_ID_W-1:0]    msb_fwd_AxID;
    wire    [ADDR_WIDTH-1:0]        msb_fwd_AxADDR;
    wire    [TRANS_BURST_W-1:0]     msb_fwd_AxBURST;
    wire    [TRANS_DATA_LEN_W-1:0]  msb_fwd_AxLEN;
    wire    [TRANS_DATA_SIZE_W-1:0] msb_fwd_AxSIZE;
    wire                            msb_fwd_AxLOCK;
    wire    [3:0]                   msb_fwd_AxCACHE;
    wire    [2:0]                   msb_fwd_AxPROT;
    wire    [3:0]                   msb_fwd_AxQOS;
    wire    [3:0]                   msb_fwd_AxREGION;
    
    // Reg declaration
    reg     [TRANS_DATA_LEN_W-1:0]  transfer_ctn_r;
    
    // Module
    // xADDR order FIFO 
    sync_fifo #(
        .FIFO_TYPE(0),
        .DATA_WIDTH(ADDR_INFO_W),
        .FIFO_DEPTH(OUTSTANDING_AMT)
    ) fifo_xaddr_order (
        .clk(ACLK_i),
        .rst_n(ARESETn_i),
        .data_i(addr_info),
        .data_o(addr_info_valid),
        .wr_valid_i(fifo_xa_order_wr_en),
        .rd_valid_i(fifo_xa_order_rd_en),
        .empty_o(fifo_xa_order_empty),
        .full_o(fifo_xa_order_full),
        .wr_ready_o(),
        .rd_ready_o(),
        .almost_empty_o(),
        .almost_full_o(),
        .counter(sa_Ax_outst_ctn_o)
    );
    // Master skid buffer
    skid_buffer #(
        .SBUF_TYPE(1),
        .DATA_WIDTH(Ax_INFO_W)
    ) mst_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (msb_bwd_data),
        .bwd_valid_i(msb_bwd_valid),
        .fwd_ready_i(msb_fwd_ready),
        .fwd_data_o (msb_fwd_data),
        .bwd_ready_o(msb_bwd_ready),
        .fwd_valid_o(msb_fwd_valid)
    );
    
    // Combinational logic
    // xADDR order FIFO
    reg [SLV_ID_W-1:0] decoded_slv_id;
    reg                decode_err;
    integer i;

    always @(*) begin
        decoded_slv_id = {SLV_ID_W{1'b0}};
        decode_err = 1'b1; // Mặc định là lỗi nếu địa chỉ không thuộc vùng nào
        for (i = 0; i < SLV_AMT; i = i + 1) begin
            if ((msb_fwd_AxADDR & SLV_ADDR_MASK[i*32 +: 32]) == SLV_BASE_ADDR[i*32 +: 32]) begin
                decoded_slv_id = i[SLV_ID_W-1:0];
                decode_err = 1'b0;
            end
        end
    end

    assign addr_slv_mapping = decoded_slv_id;
    
    assign addr_info = {addr_err, msb_fwd_AxID, addr_slv_mapping, msb_fwd_AxLEN};
    assign {addr_err_valid, AxID_valid, slv_id, AxLEN_valid} = addr_info_valid;
    assign fifo_xa_order_wr_en = Ax_handshake_occur;
    assign fifo_xa_order_rd_en = transfer_ctn_match & xDATA_handshake_occur;
    // Error / timeout detection
    assign addr_invalid = decode_err;
    assign normal_accept = (~addr_invalid) & sa_AxREADY_i[addr_slv_mapping[SLV_ID_W-1:0]] & (~fifo_xa_order_full);
    assign addr_err = addr_invalid | timeout_expire;
    // Handshake detector
    assign Ax_handshake_occur = msb_fwd_valid & msb_fwd_ready;
    assign xDATA_handshake_occur = m_xVALID_i & m_xREADY_i;
    // Transfer counter
    assign transfer_ctn_nxt = (transfer_ctn_match) ? {TRANS_DATA_LEN_W{1'b0}} : transfer_ctn_incr;
    assign transfer_ctn_incr = transfer_ctn_r + 1'b1;
    assign transfer_ctn_match = transfer_ctn_r == AxLEN_valid;
    // Output 
    // Output to Slave Arbitration
    generate 
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_LOGIC
            assign sa_AxID_o[TRANS_MST_ID_W*(slv_idx+1)-1-:TRANS_MST_ID_W]          = msb_fwd_AxID;
            assign sa_AxADDR_o[ADDR_WIDTH*(slv_idx+1)-1-:ADDR_WIDTH]                = msb_fwd_AxADDR;
            assign sa_AxBURST_o[TRANS_BURST_W*(slv_idx+1)-1-:TRANS_BURST_W]         = msb_fwd_AxBURST;
            assign sa_AxLEN_o[TRANS_DATA_LEN_W*(slv_idx+1)-1-:TRANS_DATA_LEN_W]     = msb_fwd_AxLEN;
            assign sa_AxSIZE_o[TRANS_DATA_SIZE_W*(slv_idx+1)-1-:TRANS_DATA_SIZE_W]  = msb_fwd_AxSIZE;
            assign sa_AxLOCK_o[slv_idx]                                             = msb_fwd_AxLOCK;
            assign sa_AxCACHE_o[4*(slv_idx+1)-1-:4]                                 = msb_fwd_AxCACHE;
            assign sa_AxPROT_o[3*(slv_idx+1)-1-:3]                                  = msb_fwd_AxPROT;
            assign sa_AxQOS_o[4*(slv_idx+1)-1-:4]                                   = msb_fwd_AxQOS;
            assign sa_AxREGION_o[4*(slv_idx+1)-1-:4]                                = msb_fwd_AxREGION;
            assign sa_AxVALID_o[slv_idx]                                            = msb_fwd_valid & (addr_slv_mapping == slv_idx) & (~fifo_xa_order_full) & (~addr_err);
        end
    endgenerate
    // Master skid buffer
    assign msb_bwd_data     = {m_AxID_i, m_AxADDR_i, m_AxBURST_i, m_AxLEN_i, m_AxSIZE_i, m_AxLOCK_i, m_AxCACHE_i, m_AxPROT_i, m_AxQOS_i, m_AxREGION_i};
    assign msb_bwd_valid    = m_AxVALID_i;
    assign msb_fwd_ready    = normal_accept | ((~fifo_xa_order_full) & addr_err);
    assign {msb_fwd_AxID, msb_fwd_AxADDR, msb_fwd_AxBURST, msb_fwd_AxLEN, msb_fwd_AxSIZE, msb_fwd_AxLOCK, msb_fwd_AxCACHE, msb_fwd_AxPROT, msb_fwd_AxQOS, msb_fwd_AxREGION} = msb_fwd_data;
    // Output to xDATA dispatcher
    assign dsp_xDATA_slv_id_o = slv_id;
    assign dsp_xDATA_disable_o = fifo_xa_order_empty;
    assign dsp_xDATA_err_o = addr_err_valid;
    assign dsp_xDATA_axid_o = AxID_valid;
    assign dsp_xDATA_last_o = transfer_ctn_match;
    // Output to WRESP dispatcher
    assign dsp_WRESP_slv_id_o = slv_id;
    assign dsp_WRESP_shift_en_o = fifo_xa_order_rd_en;
    assign dsp_WRESP_err_o = addr_err_valid;
    assign dsp_WRESP_axid_o = AxID_valid;
    // Output to Slave Arbitration
    assign m_AxREADY_o = msb_bwd_ready;
    
    // Flip-flop
    always @(posedge ACLK_i) begin
        if(~ARESETn_i) begin
            transfer_ctn_r <= {TRANS_DATA_LEN_W{1'b0}};
        end
        else if(xDATA_handshake_occur) begin
            transfer_ctn_r <= transfer_ctn_nxt;
        end
    end
    
    // Timeout counter
    generate
        if (TIMEOUT_W > 0) begin : TIMEOUT_LOGIC
            reg [TIMEOUT_W-1:0] timeout_ctn_r;
            wire timeout_counting = msb_fwd_valid & (~addr_invalid) & (~normal_accept) & (~fifo_xa_order_full);
            assign timeout_expire = timeout_counting & (&timeout_ctn_r);
            always @(posedge ACLK_i) begin
                if (~ARESETn_i || ~timeout_counting)
                    timeout_ctn_r <= {TIMEOUT_W{1'b0}};
                else if (~timeout_expire)
                    timeout_ctn_r <= timeout_ctn_r + 1'b1;
            end
        end else begin : NO_TIMEOUT
            assign timeout_expire = 1'b0;
        end
    endgenerate

endmodule

module dsp_R_channel
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    parameter TRANS_MST_ID_W    = 5,    // Bus width of master transaction ID 
    parameter TRANS_WR_RESP_W   = 2,
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT),
    // Dispatcher DATA depth configuration
    parameter DSP_RDATA_DEPTH   = 16
)
(
    // Input declaration
    // -- Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // -- To Master (slave interface of the interconnect)
    // ---- Read data channel
    input                                   m_RREADY_i,
    // -- To Slave Arbitration
    // ---- Read data channel (master)
    input   [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_RID_i,
    input   [DATA_WIDTH*SLV_AMT-1:0]        sa_RDATA_i,
    input   [TRANS_WR_RESP_W*SLV_AMT-1:0]   sa_RRESP_i,
    input   [SLV_AMT-1:0]                   sa_RLAST_i,
    input   [SLV_AMT-1:0]                   sa_RVALID_i,
    // -- To AR channel Dispatcher
    input   [SLV_ID_W-1:0]                  dsp_AR_slv_id_i,
    input                                   dsp_AR_disable_i,
    input                                   dsp_AR_err_i,
    input   [TRANS_MST_ID_W-1:0]            dsp_AR_axid_i,
    input                                   dsp_AR_last_i,
    // Output declaration
    // -- To Master (slave interface of interconnect)
    // ---- Read data channel (master)
    output  [TRANS_MST_ID_W-1:0]            m_RID_o,
    output  [DATA_WIDTH-1:0]                m_RDATA_o,
    output  [TRANS_WR_RESP_W-1:0]           m_RRESP_o,
    output                                  m_RLAST_o,
    output                                  m_RVALID_o,
    // -- To Slave Arbitration
    // ---- Read data channel
    output  [SLV_AMT-1:0]                   sa_RREADY_o,
    // -- To DSP AR chanenl
    output                                  dsp_RVALID_q1_o,
    output                                  dsp_RREADY_q1_o
);
    // Local parameter 
    localparam DATA_INFO_W = TRANS_MST_ID_W + DATA_WIDTH + TRANS_WR_RESP_W + 1;   // RID_W + DATA_W + RRESP + RLAST_W

    // Internal variable declaration
    genvar slv_idx;
    
    // Internal signal declaration
    // -- RDATA FIFO
    wire    [DATA_INFO_W-1:0]   data_info           [SLV_AMT-1:0];
    wire    [DATA_INFO_W-1:0]   data_info_valid     [SLV_AMT-1:0];
    wire                        fifo_rdata_wr_en    [SLV_AMT-1:0];
    wire                        fifo_rdata_rd_en    [SLV_AMT-1:0];
    wire                        fifo_rdata_empty    [SLV_AMT-1:0];
    wire                        fifo_rdata_full     [SLV_AMT-1:0];
    // -- Handshake detector
    wire                        sa_handshake_occur  [SLV_AMT-1:0];
    wire                        m_handshake_occur;
    // -- Misc
    wire   [TRANS_MST_ID_W-1:0] sa_RID_valid        [SLV_AMT-1:0];
    wire   [DATA_WIDTH-1:0]     sa_RDATA_valid      [SLV_AMT-1:0];
    wire   [TRANS_WR_RESP_W-1:0]sa_RRESP_valid      [SLV_AMT-1:0];
    wire                        sa_RLAST_valid      [SLV_AMT-1:0];
    // -- Master skid buffer 
    wire    [DATA_INFO_W-1:0]       msb_bwd_data;
    wire                            msb_bwd_valid;
    wire                            msb_bwd_ready;
    wire    [DATA_INFO_W-1:0]       msb_fwd_data;
    wire                            msb_fwd_valid;
    wire                            msb_fwd_ready;
    wire    [TRANS_MST_ID_W-1:0]    msb_fwd_RID;
    wire    [DATA_WIDTH-1:0]        msb_fwd_RDATA;
    wire    [TRANS_WR_RESP_W-1:0]   msb_fwd_RRESP;
    wire                            msb_fwd_RLAST;
    
    // Module
    // -- RDATA FIFO
    generate
    for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_FIFO
        sync_fifo 
            #(
            .FIFO_TYPE(0),
            .DATA_WIDTH(DATA_INFO_W),
            .FIFO_DEPTH(DSP_RDATA_DEPTH)
        ) fifo_rdata (
            .clk(ACLK_i),
            .rst_n(ARESETn_i),
            .data_i(data_info[slv_idx]),
            .data_o(data_info_valid[slv_idx]),
            .wr_valid_i(fifo_rdata_wr_en[slv_idx]),
            .rd_valid_i(fifo_rdata_rd_en[slv_idx]),
            .empty_o(fifo_rdata_empty[slv_idx]),
            .full_o(fifo_rdata_full[slv_idx]),
            .wr_ready_o(),
            .rd_ready_o(),
            .almost_empty_o(),
            .almost_full_o(),
            .counter()
        );
    end
    endgenerate
    // -- Master skid buffer
    skid_buffer #(
        .SBUF_TYPE(3),
        .DATA_WIDTH(DATA_INFO_W)
    ) mst_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (msb_bwd_data),
        .bwd_valid_i(msb_bwd_valid),
        .fwd_ready_i(msb_fwd_ready),
        .fwd_data_o (msb_fwd_data),
        .bwd_ready_o(msb_bwd_ready),
        .fwd_valid_o(msb_fwd_valid)
    );
    
    // Combinational logic
    // -- RDATA FIFO
    generate
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_LOGIC
            assign data_info[slv_idx] = {sa_RID_i[TRANS_MST_ID_W*(slv_idx+1)-1-:TRANS_MST_ID_W], sa_RDATA_i[DATA_WIDTH*(slv_idx+1)-1-:DATA_WIDTH], sa_RRESP_i[TRANS_WR_RESP_W*(slv_idx+1)-1-:TRANS_WR_RESP_W], sa_RLAST_i[slv_idx]};
            assign {sa_RID_valid[slv_idx], sa_RDATA_valid[slv_idx], sa_RRESP_valid[slv_idx], sa_RLAST_valid[slv_idx]} = data_info_valid[slv_idx];
            assign fifo_rdata_wr_en[slv_idx] = sa_handshake_occur[slv_idx];
            assign fifo_rdata_rd_en[slv_idx] = m_handshake_occur & (dsp_AR_slv_id_i == slv_idx) & ~dsp_AR_err_i;
        end
    endgenerate
    // -- Handshake detector
    generate
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_HSK
            assign sa_handshake_occur[slv_idx] = sa_RVALID_i[slv_idx] & sa_RREADY_o[slv_idx];
        end
    endgenerate
    assign m_handshake_occur = msb_bwd_valid & msb_bwd_ready;
    // -- Output
    // -- -- Output to Master
    assign m_RID_o = msb_fwd_RID;
    assign m_RDATA_o = msb_fwd_RDATA;
    assign m_RRESP_o = msb_fwd_RRESP;
    assign m_RLAST_o = msb_fwd_RLAST;
    assign m_RVALID_o = msb_fwd_valid;
    // -- -- Output to Slave arbitration
    generate
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_OUT
            assign sa_RREADY_o[slv_idx]= ~fifo_rdata_full[slv_idx];
        end
    endgenerate
    // -- -- Output to DSP AR
    assign dsp_RVALID_q1_o  = msb_bwd_valid;
    assign dsp_RREADY_q1_o  = msb_bwd_ready;
    // -- Master skid buffer 
    assign msb_bwd_data     = dsp_AR_err_i ?
        {dsp_AR_axid_i, {DATA_WIDTH{1'b0}}, {TRANS_WR_RESP_W{1'b1}}, dsp_AR_last_i} :  // DECERR + zero data + RLAST from Ax counter
        {sa_RID_valid[dsp_AR_slv_id_i], sa_RDATA_valid[dsp_AR_slv_id_i], sa_RRESP_valid[dsp_AR_slv_id_i], sa_RLAST_valid[dsp_AR_slv_id_i]};
    assign msb_bwd_valid    = dsp_AR_err_i ?
        ~dsp_AR_disable_i :                                          // Error: always valid when not disabled
        ~(fifo_rdata_empty[dsp_AR_slv_id_i] | dsp_AR_disable_i);
    assign msb_fwd_ready    = m_RREADY_i;
    assign {msb_fwd_RID, msb_fwd_RDATA, msb_fwd_RRESP, msb_fwd_RLAST} = msb_fwd_data;
endmodule

module dsp_W_channel
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT)
)
(
    // Input declaration
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // -- To Master (slave interface of the interconnect)
    // ---- Write data channel
    input   [DATA_WIDTH-1:0]                m_WDATA_i,
    input   [DATA_WIDTH/8-1:0]              m_WSTRB_i,
    input                                   m_WLAST_i,
    input                                   m_WVALID_i,
    // -- To Slave Arbitration
    // ---- Write data channel (master)
    input   [SLV_AMT-1:0]                   sa_WREADY_i,
    // -- To AW channel Dispatcher
    input   [SLV_ID_W-1:0]                  dsp_AW_slv_id_i,
    input                                   dsp_AW_disable_i,
    input                                   dsp_AW_err_i,
    // Output declaration
    // -- To Master (slave interface of interconnect)
    // ---- Write data channel (master)
    output                                  m_WREADY_o,
    // -- To Slave Arbitration
    // ---- Write data channel
    output  [DATA_WIDTH*SLV_AMT-1:0]        sa_WDATA_o,
    output  [(DATA_WIDTH/8)*SLV_AMT-1:0]    sa_WSTRB_o,
    output  [SLV_AMT-1:0]                   sa_WLAST_o,
    output  [SLV_AMT-1:0]                   sa_WVALID_o,
    // -- To DSP AW channel
    output                                  dsp_AW_WVALID_o,
    output                                  dsp_AW_WREADY_o
);
    // Local parameters
    localparam SLV_ID_VALID_W   = SLV_ID_W + 1;   // SLV_ID_W + 1bit (~valid)
    localparam STRB_WIDTH        = DATA_WIDTH / 8;
    localparam W_INFO_W         = DATA_WIDTH + STRB_WIDTH + 1;
    
    // Internal variable declaration
    genvar slv_idx;
    
    // Internal signal declaration
    // -- Slave ID decoder
    wire    [SLV_ID_VALID_W-1:0]    slv_id_valid;
    wire    [SLV_AMT-1:0]           slv_sel;
    // -- Master skid buffer
    wire    [W_INFO_W-1:0]          msb_bwd_data;
    wire                            msb_bwd_valid;
    wire                            msb_bwd_ready;
    wire    [W_INFO_W-1:0]          msb_fwd_data;
    wire                            msb_fwd_valid;
    wire                            msb_fwd_ready;
    wire    [DATA_WIDTH-1:0]        msb_fwd_WDATA;
    wire    [STRB_WIDTH-1:0]        msb_fwd_WSTRB;
    wire                            msb_fwd_WLAST;
    
    // Internal module
    onehot_decoder #(
        .INPUT_W(SLV_ID_VALID_W),
        .OUTPUT_W(SLV_AMT)
    ) slave_id_decoder (
        .i(slv_id_valid),
        .o(slv_sel)
    );
    // -- Master skid buffer
    skid_buffer #(
        .SBUF_TYPE(1),
        .DATA_WIDTH(W_INFO_W)
    ) mst_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (msb_bwd_data),
        .bwd_valid_i(msb_bwd_valid),
        .fwd_ready_i(msb_fwd_ready),
        .fwd_data_o (msb_fwd_data),
        .bwd_ready_o(msb_bwd_ready),
        .fwd_valid_o(msb_fwd_valid)
    );
    
    // Combinational logic
    // -- Slave ID decoder
    assign slv_id_valid = {dsp_AW_disable_i | dsp_AW_err_i, dsp_AW_slv_id_i};
    // -- Output
    // -- -- Output to Master
    assign m_WREADY_o = msb_bwd_ready;
    // -- -- Output to Slave arbitration
    generate
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_LOGIC
            assign sa_WDATA_o[DATA_WIDTH*(slv_idx+1)-1-:DATA_WIDTH] = msb_fwd_WDATA;
            assign sa_WSTRB_o[STRB_WIDTH*(slv_idx+1)-1-:STRB_WIDTH] = msb_fwd_WSTRB;
            assign sa_WLAST_o[slv_idx] = msb_fwd_WLAST;
            assign sa_WVALID_o[slv_idx] = msb_fwd_valid & slv_sel[slv_idx];
        end
    endgenerate
    // -- To DSP AW channel 
    assign dsp_AW_WVALID_o  = msb_fwd_valid;
    assign dsp_AW_WREADY_o  = msb_fwd_ready;
    // -- Master skid buffer 
    assign msb_bwd_data     = {m_WDATA_i, m_WSTRB_i, m_WLAST_i};
    assign msb_bwd_valid    = m_WVALID_i;
    assign msb_fwd_ready    = (~dsp_AW_disable_i) & (dsp_AW_err_i | sa_WREADY_i[dsp_AW_slv_id_i]);
    assign {msb_fwd_WDATA, msb_fwd_WSTRB, msb_fwd_WLAST} = msb_fwd_data;
endmodule

module dsp_B_channel
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    parameter OUTSTANDING_AMT   = 8,
    parameter OUTST_CTN_W       = $clog2(OUTSTANDING_AMT) + 1,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter TRANS_MST_ID_W    = 5,    // Bus width of master transaction ID 
    parameter TRANS_BURST_W     = 2,    // Width of xBURST 
    parameter TRANS_DATA_LEN_W  = 8,    // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter TRANS_DATA_SIZE_W = 3,    // Bus width of xSIZE
    parameter TRANS_WR_RESP_W   = 2,
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT)
)
(
    // Input declaration
    // Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // To Master (slave interface of the interconnect)
    // Write response channel
    input                                   m_BREADY_i,
    // To Slave Arbitration
    // Write response channel
    input   [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_BID_i,
    input   [TRANS_WR_RESP_W*SLV_AMT-1:0]   sa_BRESP_i,
    input   [SLV_AMT-1:0]                   sa_BVALID_i,
    // To AW channel Dispatcher
    input   [SLV_ID_W-1:0]                  dsp_AW_slv_id_i,
    input                                   dsp_AW_shift_en_i,
    input                                   dsp_AW_err_i,
    input   [TRANS_MST_ID_W-1:0]            dsp_AW_axid_i,
    // Output declaration
    // To Master (slave interface of interconnect)
    // Write response channel (master)
    output  [TRANS_MST_ID_W-1:0]            m_BID_o,
    output  [TRANS_WR_RESP_W-1:0]           m_BRESP_o,
    output                                  m_BVALID_o,
    // To Slave Arbitration
    // Write address channel
    output  [OUTST_CTN_W-1:0]               sa_B_outst_ctn_o,
    // Write response channel
    output  [SLV_AMT-1:0]                   sa_BREADY_o
);
    // Local parameter
    localparam SLV_INFO_W = 1 + TRANS_MST_ID_W + SLV_ID_W;  // err + axid + slv_id
    localparam RESP_INFO_W = TRANS_MST_ID_W + TRANS_WR_RESP_W;
    // Internal variable declaration
    genvar slv_idx;
    
    // Internal signal declaration
    // Slave order FIFO
    wire    [SLV_INFO_W-1:0]        slv_info;
    wire    [SLV_INFO_W-1:0]        slv_info_valid;
    wire                            fifo_slv_ord_wr_en;
    wire                            fifo_slv_ord_rd_en;
    wire                            fifo_slv_ord_empty;
    // Order FIFO unpacked fields
    wire                            ord_err_valid;
    wire    [TRANS_MST_ID_W-1:0]    ord_axid_valid;
    wire    [SLV_ID_W-1:0]          ord_slv_id_valid;
    // Slave resp FIFO
    wire    [RESP_INFO_W-1:0]       resp_info           [SLV_AMT-1:0];
    wire    [RESP_INFO_W-1:0]       resp_info_valid     [SLV_AMT-1:0];
    wire                            fifo_wresp_wr_en    [SLV_AMT-1:0];
    wire                            fifo_wresp_rd_en    [SLV_AMT-1:0];
    wire                            fifo_wresp_empty    [SLV_AMT-1:0];
    wire                            fifo_wresp_full     [SLV_AMT-1:0];
    // Handshake detector
    wire                            sa_handshake_occur  [SLV_AMT-1:0];
    wire                            m_handshake_occur;
    // Misc
    wire    [TRANS_MST_ID_W-1:0]    sa_BID_valid        [SLV_AMT-1:0];
    wire    [TRANS_WR_RESP_W-1:0]   sa_BRESP_valid      [SLV_AMT-1:0];
    // Master skid buffer 
    wire    [RESP_INFO_W-1:0]       msb_bwd_data;
    wire                            msb_bwd_valid;
    wire                            msb_bwd_ready;
    wire    [RESP_INFO_W-1:0]       msb_fwd_data;
    wire                            msb_fwd_valid;
    wire                            msb_fwd_ready;
    wire    [TRANS_MST_ID_W-1:0]    msb_fwd_BID;
    wire    [TRANS_WR_RESP_W-1:0]   msb_fwd_BRESP;
    // Module
    // Slave order FIFO
    sync_fifo #(
        .FIFO_TYPE(0),
        .DATA_WIDTH(SLV_INFO_W),
        .FIFO_DEPTH(OUTSTANDING_AMT)
    ) fifo_slv_order (
        .clk(ACLK_i),
        .rst_n(ARESETn_i),
        .data_i(slv_info),
        .data_o(slv_info_valid),
        .wr_valid_i(fifo_slv_ord_wr_en),
        .rd_valid_i(fifo_slv_ord_rd_en),
        .empty_o(fifo_slv_ord_empty),
        .full_o(),
        .wr_ready_o(),
        .rd_ready_o(),
        .almost_empty_o(),
        .almost_full_o(),
        .counter(sa_B_outst_ctn_o)
    );
    // Slave Response FIFO
    generate 
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_FIFO
            sync_fifo #(
                .FIFO_TYPE(0),
                .DATA_WIDTH(RESP_INFO_W),
                .FIFO_DEPTH(OUTSTANDING_AMT)
            ) fifo_WRESP_slv (
                .clk(ACLK_i),
                .rst_n(ARESETn_i),
                .data_i(resp_info[slv_idx]),
                .data_o(resp_info_valid[slv_idx]),
                .wr_valid_i(fifo_wresp_wr_en[slv_idx]),
                .rd_valid_i(fifo_wresp_rd_en[slv_idx]),
                .empty_o(fifo_wresp_empty[slv_idx]),
                .full_o(fifo_wresp_full[slv_idx]),
                .wr_ready_o(),
                .rd_ready_o(),
                .almost_empty_o(),
                .almost_full_o(),
                .counter()
            );
        end
    endgenerate
    // Master skid buffer
    skid_buffer #(
        .SBUF_TYPE(3),
        .DATA_WIDTH(RESP_INFO_W)
    ) mst_skid_buffer (
        .clk        (ACLK_i),
        .rst_n      (ARESETn_i),
        .bwd_data_i (msb_bwd_data),
        .bwd_valid_i(msb_bwd_valid),
        .fwd_ready_i(msb_fwd_ready),
        .fwd_data_o (msb_fwd_data),
        .bwd_ready_o(msb_bwd_ready),
        .fwd_valid_o(msb_fwd_valid)
    );
    
    // Combinational logic
    // Slave order FIFO
    assign slv_info = {dsp_AW_err_i, dsp_AW_axid_i, dsp_AW_slv_id_i};
    assign {ord_err_valid, ord_axid_valid, ord_slv_id_valid} = slv_info_valid;
    assign fifo_slv_ord_wr_en = dsp_AW_shift_en_i;
    assign fifo_slv_ord_rd_en = m_handshake_occur;
    // Slave resp FIFO
    generate
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_LOGIC
            assign resp_info[slv_idx] = {sa_BID_i[TRANS_MST_ID_W*(slv_idx+1)-1-:TRANS_MST_ID_W], sa_BRESP_i[TRANS_WR_RESP_W*(slv_idx+1)-1-:TRANS_WR_RESP_W]};
            assign {sa_BID_valid[slv_idx], sa_BRESP_valid[slv_idx]} = resp_info_valid[slv_idx];
            assign fifo_wresp_wr_en[slv_idx] = sa_handshake_occur[slv_idx];
            assign fifo_wresp_rd_en[slv_idx] = m_handshake_occur & (ord_slv_id_valid == slv_idx) & ~ord_err_valid;
        end
    endgenerate
    // Handshake detector
    assign m_handshake_occur = msb_bwd_valid & msb_bwd_ready;
    generate
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_HSK
            assign sa_handshake_occur[slv_idx] = sa_BVALID_i[slv_idx] & sa_BREADY_o[slv_idx];
        end
    endgenerate
    // Output to Master
    assign m_BID_o = msb_fwd_BID;
    assign m_BRESP_o = msb_fwd_BRESP;
    assign m_BVALID_o = msb_fwd_valid;
    // Output to Slave arbitration
    generate
        for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_OUT
            assign sa_BREADY_o[slv_idx] = ~fifo_wresp_full[slv_idx];
        end
    endgenerate
    // Master skid buffer
    assign msb_bwd_data     = ord_err_valid ?
        {ord_axid_valid, {TRANS_WR_RESP_W{1'b1}}} :                   // DECERR = 2'b11
        {sa_BID_valid[ord_slv_id_valid], sa_BRESP_valid[ord_slv_id_valid]};
    assign msb_bwd_valid    = ord_err_valid ?
        ~fifo_slv_ord_empty :                                          // Error: don't wait for slave
        ~(fifo_slv_ord_empty | fifo_wresp_empty[ord_slv_id_valid]);
    assign msb_fwd_ready    = m_BREADY_i;
    assign {msb_fwd_BID, msb_fwd_BRESP} = msb_fwd_data;
endmodule

module dsp_read_channel
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    parameter OUTSTANDING_AMT   = 8,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter TRANS_MST_ID_W    = 5,    // Bus width of master transaction ID 
    parameter TRANS_BURST_W     = 2,    // Width of xBURST 
    parameter TRANS_DATA_LEN_W  = 8,    // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter TRANS_DATA_SIZE_W = 3,    // Bus width of xSIZE
    parameter TRANS_WR_RESP_W   = 2,
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT),
    parameter [SLV_AMT*32-1:0] SLV_BASE_ADDR = 0,
    parameter [SLV_AMT*32-1:0] SLV_ADDR_MASK = 0,
    // Dispatcher DATA depth configuration
    parameter DSP_RDATA_DEPTH   = 16
)
(
    // Input declaration
    // Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // To Master (slave interface of the interconnect)
    // --Read address channel
    input   [TRANS_MST_ID_W-1:0]            m_ARID_i,
    input   [ADDR_WIDTH-1:0]                m_ARADDR_i,
    input   [TRANS_BURST_W-1:0]             m_ARBURST_i,
    input   [TRANS_DATA_LEN_W-1:0]          m_ARLEN_i,
    input   [TRANS_DATA_SIZE_W-1:0]         m_ARSIZE_i,
    input                                   m_ARLOCK_i,
    input   [3:0]                            m_ARCACHE_i,
    input   [2:0]                            m_ARPROT_i,
    input   [3:0]                            m_ARQOS_i,
    input   [3:0]                            m_ARREGION_i,
    input                                   m_ARVALID_i,
    // --Read data channel
    input                                   m_RREADY_i,
    // To Slave Arbitration
    // --Read address channel (master)
    input   [SLV_AMT-1:0]                   sa_ARREADY_i,
    // --Read data channel (master)
    input   [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_RID_i,
    input   [DATA_WIDTH*SLV_AMT-1:0]        sa_RDATA_i,
    input   [TRANS_WR_RESP_W*SLV_AMT-1:0]   sa_RRESP_i,
    input   [SLV_AMT-1:0]                   sa_RLAST_i,
    input   [SLV_AMT-1:0]                   sa_RVALID_i,
    // Output declaration
    // To Master (slave interface of interconnect)
    // --Read address channel (master)
    output                                  m_ARREADY_o,
    // --Read data channel (master)
    output  [TRANS_MST_ID_W-1:0]            m_RID_o,
    output  [DATA_WIDTH-1:0]                m_RDATA_o,
    output  [TRANS_WR_RESP_W-1:0]           m_RRESP_o,
    output                                  m_RLAST_o,
    output                                  m_RVALID_o,
    // To Slave Arbitration
    // --Read address channel            
    output  [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_ARID_o,
    output  [ADDR_WIDTH*SLV_AMT-1:0]        sa_ARADDR_o,
    output  [TRANS_BURST_W*SLV_AMT-1:0]     sa_ARBURST_o,
    output  [TRANS_DATA_LEN_W*SLV_AMT-1:0]  sa_ARLEN_o,
    output  [TRANS_DATA_SIZE_W*SLV_AMT-1:0] sa_ARSIZE_o,
    output  [SLV_AMT-1:0]                   sa_ARLOCK_o,
    output  [4*SLV_AMT-1:0]                 sa_ARCACHE_o,
    output  [3*SLV_AMT-1:0]                 sa_ARPROT_o,
    output  [4*SLV_AMT-1:0]                 sa_ARQOS_o,
    output  [4*SLV_AMT-1:0]                 sa_ARREGION_o,
    output  [SLV_AMT-1:0]                   sa_ARVALID_o,
    output  [SLV_AMT-1:0]                   sa_AR_outst_full_o,  // The Dispatcher is full
    // --Read data channel
    output  [SLV_AMT-1:0]                   sa_RREADY_o
);
    // Localparam initialization
    localparam OUTST_CTN_W = $clog2(OUTSTANDING_AMT) + 1;
    
    // Internal variable
    genvar slv_idx;
    
    // Internal signal declaration
    // AR channel to R channel 
    wire [SLV_ID_W-1:0]     AR_R_slv_id;
    wire                    AR_R_disable;
    wire                    AR_R_err;
    wire [TRANS_MST_ID_W-1:0] AR_R_axid;
    wire                    AR_R_last;
    // W channel to AW channel
    wire                    R_AR_RVALID_q1;
    wire                    R_AR_RREADY_q1;
    // AW channel Slave arbitration
    wire [OUTST_CTN_W-1:0]  Ax_outst_ctn;
    
    // Combinational logic
    generate
    for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_LOGIC
        assign sa_AR_outst_full_o[slv_idx] = Ax_outst_ctn == OUTSTANDING_AMT;
    end
    endgenerate
    // Module
    dsp_Ax_channel #(
        .SLV_AMT(SLV_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_BURST_W(TRANS_BURST_W),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W(TRANS_DATA_SIZE_W),
        .SLV_ID_W(SLV_ID_W),
        .SLV_BASE_ADDR(SLV_BASE_ADDR),  // THÊM VÀO
        .SLV_ADDR_MASK(SLV_ADDR_MASK)   // THÊM VÀO
    ) AR_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .m_AxID_i(m_ARID_i),
        .m_AxADDR_i(m_ARADDR_i),
        .m_AxBURST_i(m_ARBURST_i),
        .m_AxLEN_i(m_ARLEN_i),
        .m_AxSIZE_i(m_ARSIZE_i),
        .m_AxLOCK_i(m_ARLOCK_i),
        .m_AxCACHE_i(m_ARCACHE_i),
        .m_AxPROT_i(m_ARPROT_i),
        .m_AxQOS_i(m_ARQOS_i),
        .m_AxREGION_i(m_ARREGION_i),
        .m_AxVALID_i(m_ARVALID_i),
        .m_xVALID_i(R_AR_RVALID_q1),
        .m_xREADY_i(R_AR_RREADY_q1),
        .sa_AxREADY_i(sa_ARREADY_i),
        .m_AxREADY_o(m_ARREADY_o),
        .sa_AxID_o(sa_ARID_o),
        .sa_AxADDR_o(sa_ARADDR_o),
        .sa_AxBURST_o(sa_ARBURST_o),
        .sa_AxLEN_o(sa_ARLEN_o),
        .sa_AxSIZE_o(sa_ARSIZE_o),
        .sa_AxLOCK_o(sa_ARLOCK_o),
        .sa_AxCACHE_o(sa_ARCACHE_o),
        .sa_AxPROT_o(sa_ARPROT_o),
        .sa_AxQOS_o(sa_ARQOS_o),
        .sa_AxREGION_o(sa_ARREGION_o),
        .sa_AxVALID_o(sa_ARVALID_o),
        .sa_Ax_outst_ctn_o(Ax_outst_ctn),
        .dsp_xDATA_slv_id_o(AR_R_slv_id),
        .dsp_xDATA_disable_o(AR_R_disable),
        .dsp_xDATA_err_o(AR_R_err),
        .dsp_xDATA_axid_o(AR_R_axid),
        .dsp_xDATA_last_o(AR_R_last),
        .dsp_WRESP_slv_id_o(),  // N/C
        .dsp_WRESP_shift_en_o(), // N/C
        .dsp_WRESP_err_o(),     // N/C
        .dsp_WRESP_axid_o()     // N/C
    );

    dsp_R_channel #(
        .SLV_AMT(SLV_AMT),
        .DATA_WIDTH(DATA_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_WR_RESP_W(TRANS_WR_RESP_W),
        .SLV_ID_W(SLV_ID_W),
        .DSP_RDATA_DEPTH(DSP_RDATA_DEPTH)
    ) R_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .m_RREADY_i(m_RREADY_i),
        .sa_RID_i(sa_RID_i),
        .sa_RDATA_i(sa_RDATA_i),
        .sa_RRESP_i(sa_RRESP_i),
        .sa_RLAST_i(sa_RLAST_i),
        .sa_RVALID_i(sa_RVALID_i),
        .dsp_AR_slv_id_i(AR_R_slv_id),
        .dsp_AR_disable_i(AR_R_disable),
        .dsp_AR_err_i(AR_R_err),
        .dsp_AR_axid_i(AR_R_axid),
        .dsp_AR_last_i(AR_R_last),
        .m_RID_o(m_RID_o),
        .m_RDATA_o(m_RDATA_o),
        .m_RRESP_o(m_RRESP_o),
        .m_RLAST_o(m_RLAST_o),
        .m_RVALID_o(m_RVALID_o),
        .sa_RREADY_o(sa_RREADY_o),
        .dsp_RVALID_q1_o(R_AR_RVALID_q1),
        .dsp_RREADY_q1_o(R_AR_RREADY_q1)
    );

endmodule

module dsp_write_channel
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    parameter OUTSTANDING_AMT   = 8,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter TRANS_MST_ID_W    = 5,    // Bus width of master transaction ID 
    parameter TRANS_BURST_W     = 2,    // Width of xBURST 
    parameter TRANS_DATA_LEN_W  = 8,    // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter TRANS_DATA_SIZE_W = 3,    // Bus width of xSIZE
    parameter TRANS_WR_RESP_W   = 2,
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT),
    parameter [SLV_AMT*32-1:0] SLV_BASE_ADDR = 0,
    parameter [SLV_AMT*32-1:0] SLV_ADDR_MASK = 0
)
(
    // Input declaration
    // -- Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // -- To Master (slave interface of the interconnect)
    // ---- Write address channel
    input   [TRANS_MST_ID_W-1:0]            m_AWID_i,
    input   [ADDR_WIDTH-1:0]                m_AWADDR_i,
    input   [TRANS_BURST_W-1:0]             m_AWBURST_i,
    input   [TRANS_DATA_LEN_W-1:0]          m_AWLEN_i,
    input   [TRANS_DATA_SIZE_W-1:0]         m_AWSIZE_i,
    input                                   m_AWLOCK_i,
    input   [3:0]                            m_AWCACHE_i,
    input   [2:0]                            m_AWPROT_i,
    input   [3:0]                            m_AWQOS_i,
    input   [3:0]                            m_AWREGION_i,
    input                                   m_AWVALID_i,
    // ---- Write data channel
    input   [DATA_WIDTH-1:0]                m_WDATA_i,
    input   [DATA_WIDTH/8-1:0]              m_WSTRB_i,
    input                                   m_WLAST_i,
    input                                   m_WVALID_i,
    // ---- Write response channel
    input                                   m_BREADY_i,
    // -- To Slave Arbitration
    // ---- Write address channel (master)
    input   [SLV_AMT-1:0]                   sa_AWREADY_i,
    // ---- Write data channel (master)
    input   [SLV_AMT-1:0]                   sa_WREADY_i,
    // ---- Write response channel (master)
    input   [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_BID_i,
    input   [TRANS_WR_RESP_W*SLV_AMT-1:0]   sa_BRESP_i,
    input   [SLV_AMT-1:0]                   sa_BVALID_i,
    // Output declaration
    // -- To Master (slave interface of interconnect)
    // ---- Write address channel (master)
    output                                  m_AWREADY_o,
    // ---- Write data channel (master)
    output                                  m_WREADY_o,
    // ---- Write response channel (master)
    output  [TRANS_MST_ID_W-1:0]            m_BID_o,
    output  [TRANS_WR_RESP_W-1:0]           m_BRESP_o,
    output                                  m_BVALID_o,
    // -- To Slave Arbitration
    // ---- Write address channel
    output  [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_AWID_o,
    output  [ADDR_WIDTH*SLV_AMT-1:0]        sa_AWADDR_o,
    output  [TRANS_BURST_W*SLV_AMT-1:0]     sa_AWBURST_o,
    output  [TRANS_DATA_LEN_W*SLV_AMT-1:0]  sa_AWLEN_o,
    output  [TRANS_DATA_SIZE_W*SLV_AMT-1:0] sa_AWSIZE_o,
    output  [SLV_AMT-1:0]                   sa_AWLOCK_o,
    output  [4*SLV_AMT-1:0]                 sa_AWCACHE_o,
    output  [3*SLV_AMT-1:0]                 sa_AWPROT_o,
    output  [4*SLV_AMT-1:0]                 sa_AWQOS_o,
    output  [4*SLV_AMT-1:0]                 sa_AWREGION_o,
    output  [SLV_AMT-1:0]                   sa_AWVALID_o,
    output  [SLV_AMT-1:0]                   sa_AW_outst_full_o,  // The Dispatcher is full
    // ---- Write data channel
    output  [DATA_WIDTH*SLV_AMT-1:0]        sa_WDATA_o,
    output  [(DATA_WIDTH/8)*SLV_AMT-1:0]    sa_WSTRB_o,
    output  [SLV_AMT-1:0]                   sa_WLAST_o,
    output  [SLV_AMT-1:0]                   sa_WVALID_o,
    // ---- Write response channel          
    output  [SLV_AMT-1:0]                   sa_BREADY_o
);
    // Localparam initialization
    localparam OUTST_CTN_W = $clog2(OUTSTANDING_AMT) + 1;
    // Internal variable declaration
    genvar slv_idx;
    
    // Internal signal declaration
    // -- AW channel to W channel 
    wire [SLV_ID_W-1:0]     AW_W_slv_id;
    wire                    AW_W_disable;
    wire                    AW_W_err;
    // -- AW channel to B channel
    wire [SLV_ID_W-1:0]     AW_B_slv_id;
    wire                    AW_B_shift_en;
    wire                    AW_B_err;
    wire [TRANS_MST_ID_W-1:0] AW_B_axid;
    // -- To AW channel Slave arbitration
    wire [OUTST_CTN_W-1:0]  AW_outst_ctn;
    wire [OUTST_CTN_W-1:0]  B_outst_ctn;
    // -- Interconnect
    wire                    W_AW_WVALID;
    wire                    W_AW_WREADY;
        
    // Combinational logic
    generate
    for(slv_idx = 0; slv_idx < SLV_AMT; slv_idx = slv_idx + 1) begin : SLV_LOGIC
        assign sa_AW_outst_full_o[slv_idx] = (AW_outst_ctn + B_outst_ctn) == OUTSTANDING_AMT;
    end
    endgenerate
    
    // Module
    dsp_Ax_channel #(
        .SLV_AMT(SLV_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_BURST_W(TRANS_BURST_W),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W(TRANS_DATA_SIZE_W),
        .SLV_ID_W(SLV_ID_W),
        .SLV_BASE_ADDR(SLV_BASE_ADDR),  // THÊM VÀO
        .SLV_ADDR_MASK(SLV_ADDR_MASK)   // THÊM VÀO
    ) AW_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .m_AxID_i(m_AWID_i),
        .m_AxADDR_i(m_AWADDR_i),
        .m_AxBURST_i(m_AWBURST_i),
        .m_AxLEN_i(m_AWLEN_i),
        .m_AxSIZE_i(m_AWSIZE_i),
        .m_AxLOCK_i(m_AWLOCK_i),
        .m_AxCACHE_i(m_AWCACHE_i),
        .m_AxPROT_i(m_AWPROT_i),
        .m_AxQOS_i(m_AWQOS_i),
        .m_AxREGION_i(m_AWREGION_i),
        .m_AxVALID_i(m_AWVALID_i),
        .m_xVALID_i(W_AW_WVALID),
        .m_xREADY_i(W_AW_WREADY),
        .sa_AxREADY_i(sa_AWREADY_i),
        .m_AxREADY_o(m_AWREADY_o),
        .sa_AxID_o(sa_AWID_o),
        .sa_AxADDR_o(sa_AWADDR_o),
        .sa_AxBURST_o(sa_AWBURST_o),
        .sa_AxLEN_o(sa_AWLEN_o),
        .sa_AxSIZE_o(sa_AWSIZE_o),
        .sa_AxLOCK_o(sa_AWLOCK_o),
        .sa_AxCACHE_o(sa_AWCACHE_o),
        .sa_AxPROT_o(sa_AWPROT_o),
        .sa_AxQOS_o(sa_AWQOS_o),
        .sa_AxREGION_o(sa_AWREGION_o),
        .sa_AxVALID_o(sa_AWVALID_o),
        .sa_Ax_outst_ctn_o(AW_outst_ctn),
        .dsp_xDATA_slv_id_o(AW_W_slv_id),
        .dsp_xDATA_disable_o(AW_W_disable),
        .dsp_xDATA_err_o(AW_W_err),
        .dsp_xDATA_axid_o(),
        .dsp_xDATA_last_o(),
        .dsp_WRESP_slv_id_o(AW_B_slv_id),
        .dsp_WRESP_shift_en_o(AW_B_shift_en),
        .dsp_WRESP_err_o(AW_B_err),
        .dsp_WRESP_axid_o(AW_B_axid)
    );
    
    dsp_W_channel #(
        .SLV_AMT(SLV_AMT),
        .DATA_WIDTH(DATA_WIDTH),
        .SLV_ID_W(SLV_ID_W)
    ) W_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .m_WDATA_i(m_WDATA_i),
        .m_WSTRB_i(m_WSTRB_i),
        .m_WLAST_i(m_WLAST_i),
        .m_WVALID_i(m_WVALID_i),
        .sa_WREADY_i(sa_WREADY_i),
        .dsp_AW_slv_id_i(AW_W_slv_id),
        .dsp_AW_disable_i(AW_W_disable),
        .dsp_AW_err_i(AW_W_err),
        .m_WREADY_o(m_WREADY_o),
        .sa_WDATA_o(sa_WDATA_o),
        .sa_WSTRB_o(sa_WSTRB_o),
        .sa_WLAST_o(sa_WLAST_o),
        .sa_WVALID_o(sa_WVALID_o),
        .dsp_AW_WVALID_o(W_AW_WVALID),
        .dsp_AW_WREADY_o(W_AW_WREADY)
    );
    
    dsp_B_channel #(
        .SLV_AMT(SLV_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_BURST_W(TRANS_BURST_W),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W(TRANS_DATA_SIZE_W),
        .TRANS_WR_RESP_W(TRANS_WR_RESP_W),
        .SLV_ID_W(SLV_ID_W)
    ) B_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .m_BREADY_i(m_BREADY_i),
        .sa_BID_i(sa_BID_i),
        .sa_BRESP_i(sa_BRESP_i),
        .sa_BVALID_i(sa_BVALID_i),
        .dsp_AW_slv_id_i(AW_B_slv_id),
        .dsp_AW_shift_en_i(AW_B_shift_en),
        .dsp_AW_err_i(AW_B_err),
        .dsp_AW_axid_i(AW_B_axid),
        .m_BID_o(m_BID_o),
        .m_BRESP_o(m_BRESP_o),
        .m_BVALID_o(m_BVALID_o),
        .sa_B_outst_ctn_o(B_outst_ctn),
        .sa_BREADY_o(sa_BREADY_o)
    );

endmodule

module ai_dispatcher
#(
    // Dispatcher configuration
    parameter SLV_AMT           = 2,
    parameter OUTSTANDING_AMT   = 8,
    // Transaction configuration
    parameter DATA_WIDTH        = 32,
    parameter ADDR_WIDTH        = 32,
    parameter TRANS_MST_ID_W    = 5,    // Bus width of master transaction ID 
    parameter TRANS_BURST_W     = 2,    // Width of xBURST 
    parameter TRANS_DATA_LEN_W  = 8,    // Bus width of xLEN (AXI4: 8-bit, burst 1-256)
    parameter TRANS_DATA_SIZE_W = 3,    // Bus width of xSIZE
    parameter TRANS_WR_RESP_W   = 2,
    // Slave configuration
    parameter SLV_ID_W          = $clog2(SLV_AMT),
    parameter [SLV_AMT*32-1:0] SLV_BASE_ADDR = 0,
    parameter [SLV_AMT*32-1:0] SLV_ADDR_MASK = 0,
    // Dispatcher DATA depth configuration
    parameter DSP_RDATA_DEPTH   = 16
)
(
    // Input declaration
    // Global signals
    input                                   ACLK_i,
    input                                   ARESETn_i,
    // To Master (slave interface of the interconnect)
    // Write address channel
    input   [TRANS_MST_ID_W-1:0]            m_AWID_i,
    input   [ADDR_WIDTH-1:0]                m_AWADDR_i,
    input   [TRANS_BURST_W-1:0]             m_AWBURST_i,
    input   [TRANS_DATA_LEN_W-1:0]          m_AWLEN_i,
    input   [TRANS_DATA_SIZE_W-1:0]         m_AWSIZE_i,
    input                                   m_AWLOCK_i,
    input   [3:0]                            m_AWCACHE_i,
    input   [2:0]                            m_AWPROT_i,
    input   [3:0]                            m_AWQOS_i,
    input   [3:0]                            m_AWREGION_i,
    input                                   m_AWVALID_i,
    // Write data channel
    input   [DATA_WIDTH-1:0]                m_WDATA_i,
    input   [DATA_WIDTH/8-1:0]              m_WSTRB_i,
    input                                   m_WLAST_i,
    input                                   m_WVALID_i,
    // Write response channel
    input                                   m_BREADY_i,
    // Read address channel
    input   [TRANS_MST_ID_W-1:0]            m_ARID_i,
    input   [ADDR_WIDTH-1:0]                m_ARADDR_i,
    input   [TRANS_BURST_W-1:0]             m_ARBURST_i,
    input   [TRANS_DATA_LEN_W-1:0]          m_ARLEN_i,
    input   [TRANS_DATA_SIZE_W-1:0]         m_ARSIZE_i,
    input                                   m_ARLOCK_i,
    input   [3:0]                            m_ARCACHE_i,
    input   [2:0]                            m_ARPROT_i,
    input   [3:0]                            m_ARQOS_i,
    input   [3:0]                            m_ARREGION_i,
    input                                   m_ARVALID_i,
    // Read data channel
    input                                   m_RREADY_i,
    // To Slave Arbitration
    // Write address channel (master)
    input   [SLV_AMT-1:0]                   sa_AWREADY_i,
    // Write data channel (master)
    input   [SLV_AMT-1:0]                   sa_WREADY_i,
    // Write response channel (master)
    input   [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_BID_i,
    input   [TRANS_WR_RESP_W*SLV_AMT-1:0]   sa_BRESP_i,
    input   [SLV_AMT-1:0]                   sa_BVALID_i,
    // Read address channel (master)
    input   [SLV_AMT-1:0]                   sa_ARREADY_i,
    // Read data channel (master)
    input   [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_RID_i,
    input   [DATA_WIDTH*SLV_AMT-1:0]        sa_RDATA_i,
    input   [TRANS_WR_RESP_W*SLV_AMT-1:0]   sa_RRESP_i,
    input   [SLV_AMT-1:0]                   sa_RLAST_i,
    input   [SLV_AMT-1:0]                   sa_RVALID_i,
    // Output declaration
    // To Master (slave interface of interconnect)
    // Write address channel (master)
    output                                  m_AWREADY_o,
    // Write data channel (master)
    output                                  m_WREADY_o,
    // Write response channel (master)
    output  [TRANS_MST_ID_W-1:0]            m_BID_o,
    output  [TRANS_WR_RESP_W-1:0]           m_BRESP_o,
    output                                  m_BVALID_o,
    // Read address channel (master)
    output                                  m_ARREADY_o,
    // Read data channel (master)
    output  [TRANS_MST_ID_W-1:0]            m_RID_o,
    output  [DATA_WIDTH-1:0]                m_RDATA_o,
    output  [TRANS_WR_RESP_W-1:0]           m_RRESP_o,
    output                                  m_RLAST_o,
    output                                  m_RVALID_o,
    // To Slave Arbitration
    // Write address channel
    output  [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_AWID_o,
    output  [ADDR_WIDTH*SLV_AMT-1:0]        sa_AWADDR_o,
    output  [TRANS_BURST_W*SLV_AMT-1:0]     sa_AWBURST_o,
    output  [TRANS_DATA_LEN_W*SLV_AMT-1:0]  sa_AWLEN_o,
    output  [TRANS_DATA_SIZE_W*SLV_AMT-1:0] sa_AWSIZE_o,
    output  [SLV_AMT-1:0]                   sa_AWLOCK_o,
    output  [4*SLV_AMT-1:0]                 sa_AWCACHE_o,
    output  [3*SLV_AMT-1:0]                 sa_AWPROT_o,
    output  [4*SLV_AMT-1:0]                 sa_AWQOS_o,
    output  [4*SLV_AMT-1:0]                 sa_AWREGION_o,
    output  [SLV_AMT-1:0]                   sa_AWVALID_o,
    output  [SLV_AMT-1:0]                   sa_AW_outst_full_o,  // The Dispatcher is full
    // Write data channel
    output  [DATA_WIDTH*SLV_AMT-1:0]        sa_WDATA_o,
    output  [(DATA_WIDTH/8)*SLV_AMT-1:0]    sa_WSTRB_o,
    output  [SLV_AMT-1:0]                   sa_WLAST_o,
    output  [SLV_AMT-1:0]                   sa_WVALID_o,
    // Write response channel          
    output  [SLV_AMT-1:0]                   sa_BREADY_o,
    // Read address channel            
    output  [TRANS_MST_ID_W*SLV_AMT-1:0]    sa_ARID_o,
    output  [ADDR_WIDTH*SLV_AMT-1:0]        sa_ARADDR_o,
    output  [TRANS_BURST_W*SLV_AMT-1:0]     sa_ARBURST_o,
    output  [TRANS_DATA_LEN_W*SLV_AMT-1:0]  sa_ARLEN_o,
    output  [TRANS_DATA_SIZE_W*SLV_AMT-1:0] sa_ARSIZE_o,
    output  [SLV_AMT-1:0]                   sa_ARLOCK_o,
    output  [4*SLV_AMT-1:0]                 sa_ARCACHE_o,
    output  [3*SLV_AMT-1:0]                 sa_ARPROT_o,
    output  [4*SLV_AMT-1:0]                 sa_ARQOS_o,
    output  [4*SLV_AMT-1:0]                 sa_ARREGION_o,
    output  [SLV_AMT-1:0]                   sa_ARVALID_o,
    output  [SLV_AMT-1:0]                   sa_AR_outst_full_o,  // The Dispatcher is full
    // Read data channel
    output  [SLV_AMT-1:0]                   sa_RREADY_o
);
    dsp_write_channel #(
        .SLV_AMT(SLV_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_BURST_W(TRANS_BURST_W),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W(TRANS_DATA_SIZE_W),
        .TRANS_WR_RESP_W(TRANS_WR_RESP_W),
        .SLV_ID_W(SLV_ID_W),
        .SLV_BASE_ADDR(SLV_BASE_ADDR),   // THÊM VÀO
        .SLV_ADDR_MASK(SLV_ADDR_MASK)    // THÊM VÀO      
    ) write_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .m_AWID_i(m_AWID_i),
        .m_AWADDR_i(m_AWADDR_i),
        .m_AWBURST_i(m_AWBURST_i),
        .m_AWLEN_i(m_AWLEN_i),
        .m_AWSIZE_i(m_AWSIZE_i),
        .m_AWLOCK_i(m_AWLOCK_i),
        .m_AWCACHE_i(m_AWCACHE_i),
        .m_AWPROT_i(m_AWPROT_i),
        .m_AWQOS_i(m_AWQOS_i),
        .m_AWREGION_i(m_AWREGION_i),
        .m_AWVALID_i(m_AWVALID_i),
        .m_WDATA_i(m_WDATA_i),
        .m_WSTRB_i(m_WSTRB_i),
        .m_WLAST_i(m_WLAST_i),
        .m_WVALID_i(m_WVALID_i),
        .m_BREADY_i(m_BREADY_i),
        .sa_AWREADY_i(sa_AWREADY_i),
        .sa_WREADY_i(sa_WREADY_i),
        .sa_BID_i(sa_BID_i),
        .sa_BRESP_i(sa_BRESP_i),
        .sa_BVALID_i(sa_BVALID_i),
        .m_AWREADY_o(m_AWREADY_o),
        .m_WREADY_o(m_WREADY_o),
        .m_BID_o(m_BID_o),
        .m_BRESP_o(m_BRESP_o),
        .m_BVALID_o(m_BVALID_o),
        .sa_AWID_o(sa_AWID_o),
        .sa_AWADDR_o(sa_AWADDR_o),
        .sa_AWBURST_o(sa_AWBURST_o),
        .sa_AWLEN_o(sa_AWLEN_o),
        .sa_AWSIZE_o(sa_AWSIZE_o),
        .sa_AWLOCK_o(sa_AWLOCK_o),
        .sa_AWCACHE_o(sa_AWCACHE_o),
        .sa_AWPROT_o(sa_AWPROT_o),
        .sa_AWQOS_o(sa_AWQOS_o),
        .sa_AWREGION_o(sa_AWREGION_o),
        .sa_AWVALID_o(sa_AWVALID_o),
        .sa_AW_outst_full_o(sa_AW_outst_full_o),
        .sa_WDATA_o(sa_WDATA_o),
        .sa_WSTRB_o(sa_WSTRB_o),
        .sa_WLAST_o(sa_WLAST_o),
        .sa_WVALID_o(sa_WVALID_o),
        .sa_BREADY_o(sa_BREADY_o)       
    );
    
    dsp_read_channel #(
        .SLV_AMT(SLV_AMT),
        .OUTSTANDING_AMT(OUTSTANDING_AMT),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TRANS_MST_ID_W(TRANS_MST_ID_W),
        .TRANS_BURST_W(TRANS_BURST_W),
        .TRANS_DATA_LEN_W(TRANS_DATA_LEN_W),
        .TRANS_DATA_SIZE_W(TRANS_DATA_SIZE_W),
        .TRANS_WR_RESP_W(TRANS_WR_RESP_W),
        .SLV_ID_W(SLV_ID_W),
        .SLV_BASE_ADDR(SLV_BASE_ADDR),   // THÊM VÀO
        .SLV_ADDR_MASK(SLV_ADDR_MASK),    // THÊM VÀO
        .DSP_RDATA_DEPTH(DSP_RDATA_DEPTH)
    ) read_channel (
        .ACLK_i(ACLK_i),
        .ARESETn_i(ARESETn_i),
        .m_ARID_i(m_ARID_i),
        .m_ARADDR_i(m_ARADDR_i),
        .m_ARBURST_i(m_ARBURST_i),
        .m_ARLEN_i(m_ARLEN_i),
        .m_ARSIZE_i(m_ARSIZE_i),
        .m_ARLOCK_i(m_ARLOCK_i),
        .m_ARCACHE_i(m_ARCACHE_i),
        .m_ARPROT_i(m_ARPROT_i),
        .m_ARQOS_i(m_ARQOS_i),
        .m_ARREGION_i(m_ARREGION_i),
        .m_ARVALID_i(m_ARVALID_i),
        .m_RREADY_i(m_RREADY_i),
        .sa_ARREADY_i(sa_ARREADY_i),
        .sa_RID_i(sa_RID_i),
        .sa_RDATA_i(sa_RDATA_i),
        .sa_RRESP_i(sa_RRESP_i),
        .sa_RLAST_i(sa_RLAST_i),
        .sa_RVALID_i(sa_RVALID_i),
        .m_ARREADY_o(m_ARREADY_o),
        .m_RID_o(m_RID_o),
        .m_RDATA_o(m_RDATA_o),
        .m_RRESP_o(m_RRESP_o),
        .m_RLAST_o(m_RLAST_o),
        .m_RVALID_o(m_RVALID_o),
        .sa_ARID_o(sa_ARID_o),
        .sa_ARADDR_o(sa_ARADDR_o),
        .sa_ARBURST_o(sa_ARBURST_o),
        .sa_ARLEN_o(sa_ARLEN_o),
        .sa_ARSIZE_o(sa_ARSIZE_o),
        .sa_ARLOCK_o(sa_ARLOCK_o),
        .sa_ARCACHE_o(sa_ARCACHE_o),
        .sa_ARPROT_o(sa_ARPROT_o),
        .sa_ARQOS_o(sa_ARQOS_o),
        .sa_ARREGION_o(sa_ARREGION_o),
        .sa_ARVALID_o(sa_ARVALID_o),
        .sa_AR_outst_full_o(sa_AR_outst_full_o),
        .sa_RREADY_o(sa_RREADY_o)
    );
endmodule
