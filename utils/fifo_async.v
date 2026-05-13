`timescale 1ns / 1ps

module async_fifo
#(
    parameter   ASFIFO_TYPE     = 0,    // Async FIFO type: 0-Normal 1-Full-registered
    parameter   DATA_WIDTH      = 8,
    parameter   FIFO_DEPTH      = 32,
    // Synchronizer configuration
    parameter   NUM_SYNC_FF     = 2,    // Number of synchronizing flip-flop (rcmd: 2 FFs)
    // Do not configure
    parameter   ADDR_WIDTH      = $clog2(FIFO_DEPTH)
)
(
    input                           clk_wr_domain,
    input                           clk_rd_domain,

    input       [DATA_WIDTH - 1:0]  data_i,
    output      [DATA_WIDTH - 1:0]  data_o,

    input                           wr_valid_i,
    input                           rd_valid_i,

    output                          empty_o,
    output                          full_o,
    output                          wr_ready_o,
    output                          rd_ready_o,
    output                          almost_empty_o,
    output                          almost_full_o,

    input                           wrst_n,  // Write-domain reset (async, active-low)
    input                           rrst_n   // Read-domain reset  (async, active-low)
);

    // Localparameter initialization
    localparam ADDR_OVF_WIDTH = ADDR_WIDTH + 1;

    // Internal variable declaration
    genvar sync_ff_idx;

    generate
    if(ASFIFO_TYPE == 0) begin          // Normal FIFO type
        // Internal signal declaration
        // wire declaration
        // -- Write domain
        wire                       wr_handshake;
        wire[ADDR_OVF_WIDTH-1:0]   wr_addr_inc;
        wire[ADDR_OVF_WIDTH-1:0]   wr_addr_gray_nxt;
        wire[ADDR_WIDTH-1:0]       wr_addr_map;
        wire[ADDR_OVF_WIDTH-1:0]   rd_addr_stable;
        // -- Read domain
        wire[DATA_WIDTH-1:0]       rd_data;
        wire                       rd_handshake;
        wire[ADDR_OVF_WIDTH-1:0]   rd_addr_inc;
        wire[ADDR_OVF_WIDTH-1:0]   rd_addr_gray_nxt;
        wire[ADDR_WIDTH-1:0]       rd_addr_map;
        wire[ADDR_OVF_WIDTH-1:0]   wr_addr_stable;
        // reg declaration
        // -- Global
        reg [DATA_WIDTH - 1:0]     buffer          [0:FIFO_DEPTH-1];
        // -- Write domain
        reg [ADDR_OVF_WIDTH-1:0]   wr_addr;
        reg [ADDR_OVF_WIDTH-1:0]   wr_addr_gray;
        reg [ADDR_OVF_WIDTH-1:0]   rd_addr_meta    [0:NUM_SYNC_FF-1];
        // -- Read domain
        reg [ADDR_OVF_WIDTH-1:0]   rd_addr;
        reg [ADDR_OVF_WIDTH-1:0]   rd_addr_gray;
        reg [ADDR_OVF_WIDTH-1:0]   wr_addr_meta    [0:NUM_SYNC_FF-1];

        // Internal module initialization
        // -- Write domain
        bin2gray_converter #(
            .DATA_WIDTH(ADDR_OVF_WIDTH)
        ) b2g_wr_domain (
            .bin_i(wr_addr),
            .gray_o(wr_addr_gray_nxt)
        );
        gray2bin_converter #(
            .DATA_WIDTH (ADDR_OVF_WIDTH)
        ) g2b_wr_domain (
            .gray_i     (rd_addr_meta[NUM_SYNC_FF-1]),
            .bin_o      (rd_addr_stable)
        );
        // -- Read domain
        bin2gray_converter #(
            .DATA_WIDTH(ADDR_OVF_WIDTH)
        ) b2g_rd_domain (
            .bin_i(rd_addr),
            .gray_o(rd_addr_gray_nxt)
        );
        gray2bin_converter #(
            .DATA_WIDTH (ADDR_OVF_WIDTH)
        ) g2b_rd_domain (
            .gray_i     (wr_addr_meta[NUM_SYNC_FF-1]),
            .bin_o      (wr_addr_stable)
        );

        // combinational logic
        assign wr_addr_inc      = wr_addr + 1'b1;
        assign rd_addr_inc      = rd_addr + 1'b1;
        assign wr_addr_map      = wr_addr[ADDR_WIDTH - 1:0];
        assign rd_addr_map      = rd_addr[ADDR_WIDTH - 1:0];
        // -- Write domain
        assign wr_handshake     = wr_valid_i & wr_ready_o;
        assign full_o           = (wr_addr_map == rd_addr_stable[ADDR_WIDTH-1:0]) & (wr_addr[ADDR_WIDTH] ^ rd_addr_stable[ADDR_WIDTH]);
        assign wr_ready_o       = ~full_o;
        assign almost_full_o    = wr_addr_map + 1'b1 == rd_addr_stable[ADDR_WIDTH-1:0];
        // -- Read domain
        assign data_o           = buffer[rd_addr_map];
        assign rd_handshake     = rd_valid_i & rd_ready_o;
        assign empty_o          = (wr_addr_stable == rd_addr);
        assign rd_ready_o       = ~empty_o;
        assign almost_empty_o   = rd_addr_inc ==  wr_addr_stable;

        // flip-flop logic
        // -- Write domain: buffer write (no reset needed for BRAM-style)
        always @(posedge clk_wr_domain) begin
            if(wr_handshake) begin
                buffer[wr_addr_map] <= data_i;
            end
        end

        // -- Write domain: write pointer
        always @(posedge clk_wr_domain or negedge wrst_n) begin
            if(!wrst_n) begin
                wr_addr <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else if(wr_handshake) begin
                wr_addr <= wr_addr_inc;
            end
        end

        // -- Write domain: write pointer (gray encoding)
        always @(posedge clk_wr_domain or negedge wrst_n) begin
            if(!wrst_n) begin
                wr_addr_gray <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else begin
                wr_addr_gray <= wr_addr_gray_nxt;
            end
        end

        // -- Write domain: read pointer synchronizer (captures rd_addr_gray into wr domain)
        for(sync_ff_idx = 0; sync_ff_idx < NUM_SYNC_FF; sync_ff_idx = sync_ff_idx + 1) begin : MULT_FF_RD_SYNC
            if(sync_ff_idx == 0) begin
                always @(posedge clk_wr_domain or negedge wrst_n) begin
                    if(!wrst_n) begin
                        rd_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        rd_addr_meta[sync_ff_idx] <= rd_addr_gray;
                    end
                end
            end
            else begin
                always @(posedge clk_wr_domain or negedge wrst_n) begin
                    if(!wrst_n) begin
                        rd_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        rd_addr_meta[sync_ff_idx] <= rd_addr_meta[sync_ff_idx-1];
                    end
                end
            end
        end

        // -- Read domain: read pointer
        always @(posedge clk_rd_domain or negedge rrst_n) begin
            if(!rrst_n) begin
                rd_addr <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else if(rd_handshake) begin
                rd_addr <= rd_addr_inc;
            end
        end

        // -- Read domain: read pointer (gray encoding)
        always @(posedge clk_rd_domain or negedge rrst_n) begin
            if(!rrst_n) begin
                rd_addr_gray <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else begin
                rd_addr_gray <= rd_addr_gray_nxt;
            end
        end

        // -- Read domain: write pointer synchronizer (captures wr_addr_gray into rd domain)
        for(sync_ff_idx = 0; sync_ff_idx < NUM_SYNC_FF; sync_ff_idx = sync_ff_idx + 1) begin : MULT_FF_WR_SYNC
            if(sync_ff_idx == 0) begin
                always @(posedge clk_rd_domain or negedge rrst_n) begin
                    if(!rrst_n) begin
                        wr_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        wr_addr_meta[sync_ff_idx] <= wr_addr_gray;
                    end
                end
            end
            else begin
                always @(posedge clk_rd_domain or negedge rrst_n) begin
                    if(!rrst_n) begin
                        wr_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        wr_addr_meta[sync_ff_idx] <= wr_addr_meta[sync_ff_idx-1];
                    end
                end
            end
        end
    end
    else if(ASFIFO_TYPE == 1) begin     // Full-registered FIFO type
        // Internal signal declaration
        // wire declaration
        // -- Write domain
        wire                       wr_handshake;
        wire[ADDR_OVF_WIDTH-1:0]   wr_addr_inc;
        wire[ADDR_OVF_WIDTH-1:0]   wr_addr_gray_nxt;
        wire[ADDR_WIDTH-1:0]       wr_addr_map;
        wire[ADDR_OVF_WIDTH-1:0]   rd_addr_stable;
        wire                       full;
        wire                       full_o_d;
        // -- Read domain
        wire[DATA_WIDTH-1:0]       rd_data;
        wire                       rd_handshake;
        wire[ADDR_OVF_WIDTH-1:0]   rd_addr_inc;
        wire[ADDR_OVF_WIDTH-1:0]   rd_addr_gray_nxt;
        wire[ADDR_WIDTH-1:0]       rd_addr_map;
        wire[ADDR_OVF_WIDTH-1:0]   wr_addr_stable;
        wire                       empty;
        wire                       empty_d;
        // reg declaration
        // -- Global
        reg [DATA_WIDTH - 1:0]     buffer          [0:FIFO_DEPTH-1];
        // -- Write domain
        reg [ADDR_OVF_WIDTH-1:0]   wr_addr;
        reg [ADDR_OVF_WIDTH-1:0]   wr_addr_gray;
        reg                        wr_ready_o_q;
        reg                        full_o_q;
        reg [ADDR_OVF_WIDTH-1:0]   rd_addr_meta    [0:NUM_SYNC_FF-1];
        // -- Read domain
        reg [DATA_WIDTH-1:0]       data_o_q;
        reg [ADDR_OVF_WIDTH-1:0]   rd_addr;
        reg [ADDR_OVF_WIDTH-1:0]   rd_addr_gray;
        reg                        rd_ready_o_q;
        reg                        empty_o_q;
        reg [ADDR_OVF_WIDTH-1:0]   wr_addr_meta    [0:NUM_SYNC_FF-1];

        // Internal module initialization
        // -- Write domain
        bin2gray_converter #(
            .DATA_WIDTH(ADDR_OVF_WIDTH)
        ) b2g_wr_domain (
            .bin_i(wr_addr),
            .gray_o(wr_addr_gray_nxt)
        );
        gray2bin_converter #(
            .DATA_WIDTH (ADDR_OVF_WIDTH)
        ) g2b_wr_domain (
            .gray_i     (rd_addr_meta[NUM_SYNC_FF-1]),
            .bin_o      (rd_addr_stable)
        );
        // -- Read domain
        bin2gray_converter #(
            .DATA_WIDTH(ADDR_OVF_WIDTH)
        ) b2g_rd_domain (
            .bin_i(rd_addr),
            .gray_o(rd_addr_gray_nxt)
        );
        gray2bin_converter #(
            .DATA_WIDTH (ADDR_OVF_WIDTH)
        ) g2b_rd_domain (
            .gray_i     (wr_addr_meta[NUM_SYNC_FF-1]),
            .bin_o      (wr_addr_stable)
        );

        // combinational logic
        assign wr_addr_inc      = wr_addr + 1'b1;
        assign rd_addr_inc      = rd_addr + 1'b1;
        assign wr_addr_map      = wr_addr[ADDR_WIDTH - 1:0];
        assign rd_addr_map      = (empty_o & (~empty_d)) ? rd_addr[ADDR_WIDTH - 1:0] : rd_addr[ADDR_WIDTH - 1:0] + 1'b1;
        // -- Write domain
        assign wr_handshake     = wr_valid_i & wr_ready_o;
        assign full_o           = full_o_q;
        assign full_o_d         = almost_full_o | full;
        assign full             = (wr_addr_map == rd_addr_stable[ADDR_WIDTH-1:0]) & (wr_addr[ADDR_WIDTH] ^ rd_addr_stable[ADDR_WIDTH]);
        assign wr_ready_o       = wr_ready_o_q;
        assign almost_full_o    = wr_addr_map + 1'b1 == rd_addr_stable[ADDR_WIDTH-1:0];
        // -- Read domain
        assign data_o           = data_o_q;
        assign rd_data          = buffer[rd_addr_map];
        assign rd_handshake     = rd_valid_i & rd_ready_o;
        assign empty_o          = empty_o_q;
        assign empty            = wr_addr_stable == rd_addr;
        assign empty_d          = (rd_handshake & almost_empty_o) | empty;
        assign rd_ready_o       = rd_ready_o_q;
        assign almost_empty_o   = rd_addr_inc ==  wr_addr_stable;

        // flip-flop logic
        // -- Write domain: buffer write (no reset needed for BRAM-style)
        always @(posedge clk_wr_domain) begin
            if(wr_handshake) begin
                buffer[wr_addr_map] <= data_i;
            end
        end

        // -- Write domain: write pointer
        always @(posedge clk_wr_domain or negedge wrst_n) begin
            if(!wrst_n) begin
                wr_addr <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else if(wr_handshake) begin
                wr_addr <= wr_addr_inc;
            end
        end

        // -- Write domain: full status
        always @(posedge clk_wr_domain or negedge wrst_n) begin
            if(!wrst_n) begin
                full_o_q <= 1'b0;
            end
            else begin
                full_o_q <= full_o_d;
            end
        end

        always @(posedge clk_wr_domain or negedge wrst_n) begin
            if(!wrst_n) begin
                wr_ready_o_q <= 1'b1;
            end
            else begin
                wr_ready_o_q <= ~full_o_d;
            end
        end

        // -- Write domain: write pointer (gray encoding)
        always @(posedge clk_wr_domain or negedge wrst_n) begin
            if(!wrst_n) begin
                wr_addr_gray <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else begin
                wr_addr_gray <= wr_addr_gray_nxt;
            end
        end

        // -- Write domain: read pointer synchronizer
        for(sync_ff_idx = 0; sync_ff_idx < NUM_SYNC_FF; sync_ff_idx = sync_ff_idx + 1) begin : MULT_FF_RD_SYNC
            if(sync_ff_idx == 0) begin
                always @(posedge clk_wr_domain or negedge wrst_n) begin
                    if(!wrst_n) begin
                        rd_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        rd_addr_meta[sync_ff_idx] <= rd_addr_gray;
                    end
                end
            end
            else begin
                always @(posedge clk_wr_domain or negedge wrst_n) begin
                    if(!wrst_n) begin
                        rd_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        rd_addr_meta[sync_ff_idx] <= rd_addr_meta[sync_ff_idx-1];
                    end
                end
            end
        end

        // -- Read domain: data output register
        always @(posedge clk_rd_domain or negedge rrst_n) begin
            if(!rrst_n) begin
                data_o_q <= {DATA_WIDTH{1'b0}};
            end
            else if ((empty_o & (~empty_d)) | rd_handshake) begin
                data_o_q <= rd_data;
            end
        end

        // -- Read domain: read pointer
        always @(posedge clk_rd_domain or negedge rrst_n) begin
            if(!rrst_n) begin
                rd_addr <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else if(rd_handshake) begin
                rd_addr <= rd_addr_inc;
            end
        end

        // -- Read domain: empty status
        always @(posedge clk_rd_domain or negedge rrst_n) begin
            if(!rrst_n) begin
                empty_o_q <= 1'b1;
            end
            else begin
                empty_o_q <= empty_d;
            end
        end

        always @(posedge clk_rd_domain or negedge rrst_n) begin
            if(!rrst_n) begin
                rd_ready_o_q <= 1'b0;
            end
            else begin
                rd_ready_o_q <= ~empty_d;
            end
        end

        // -- Read domain: read pointer (gray encoding)
        always @(posedge clk_rd_domain or negedge rrst_n) begin
            if(!rrst_n) begin
                rd_addr_gray <= {ADDR_OVF_WIDTH{1'b0}};
            end
            else begin
                rd_addr_gray <= rd_addr_gray_nxt;
            end
        end

        // -- Read domain: write pointer synchronizer
        for(sync_ff_idx = 0; sync_ff_idx < NUM_SYNC_FF; sync_ff_idx = sync_ff_idx + 1) begin : MULT_FF_WR_SYNC
            if(sync_ff_idx == 0) begin
                always @(posedge clk_rd_domain or negedge rrst_n) begin
                    if(!rrst_n) begin
                        wr_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        wr_addr_meta[sync_ff_idx] <= wr_addr_gray;
                    end
                end
            end
            else begin
                always @(posedge clk_rd_domain or negedge rrst_n) begin
                    if(!rrst_n) begin
                        wr_addr_meta[sync_ff_idx] <= {ADDR_OVF_WIDTH{1'b0}};
                    end
                    else begin
                        wr_addr_meta[sync_ff_idx] <= wr_addr_meta[sync_ff_idx-1];
                    end
                end
            end
        end
    end
    endgenerate
endmodule
