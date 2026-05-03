`timescale 1ns/1ps

`timescale 1ns/1ps

module rob_banked_array #(
    parameter ROB_DEPTH       = 64,
    parameter DATA_W          = 64,
    parameter N_BANKS         = 4,    // phải chia đều ROB_DEPTH
    parameter N_READ          = 6,    // 2 fwd + N_COMMIT
    parameter N_WRITE         = 4,    // = N_CDB
    parameter PIPELINE_OUTPUT = 1,    // 1 = registered output (1-cycle latency)

    parameter ROB_TAG_W  = $clog2(ROB_DEPTH),
    parameter BANK_DEPTH = ROB_DEPTH / N_BANKS,
    parameter BANK_AW    = $clog2(BANK_DEPTH),
    parameter BANK_SEL_W = $clog2(N_BANKS)
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          flush_i,

    // Write ports (from CDB)
    input  wire [N_WRITE-1:0]            wr_en_i,
    input  wire [N_WRITE*ROB_TAG_W-1:0]  wr_addr_i,
    input  wire [N_WRITE*DATA_W-1:0]     wr_data_i,

    // Read ports (forwarding + commit)
    input  wire [N_READ-1:0]             rd_en_i,
    input  wire [N_READ*ROB_TAG_W-1:0]   rd_addr_i,
    output wire [N_READ*DATA_W-1:0]      rd_data_o,
    output wire [N_READ-1:0]             rd_valid_o,

    // Conflict indicator: 1 bit per bank (for performance counters)
    output wire [N_BANKS-1:0]            wr_conflict_o
);

    function [BANK_SEL_W-1:0] get_bank;
        input [ROB_TAG_W-1:0] addr;
        get_bank = addr[BANK_SEL_W-1:0];
    endfunction

    function [BANK_AW-1:0] get_bank_addr;
        input [ROB_TAG_W-1:0] addr;
        get_bank_addr = addr[ROB_TAG_W-1:BANK_SEL_W];
    endfunction

    reg [DATA_W-1:0]  pend_data0_r [0:N_BANKS-1]; // head of FIFO (drains first)
    reg [BANK_AW-1:0] pend_addr0_r [0:N_BANKS-1];
    reg [DATA_W-1:0]  pend_data1_r [0:N_BANKS-1]; // tail of FIFO
    reg [BANK_AW-1:0] pend_addr1_r [0:N_BANKS-1];
    reg [1:0]         pend_count_r [0:N_BANKS-1]; // 0, 1, or 2

    // Next-cycle pending state (combinational, computed in arb block)
    reg [DATA_W-1:0]  nxt_data0  [0:N_BANKS-1];
    reg [BANK_AW-1:0] nxt_addr0  [0:N_BANKS-1];
    reg [DATA_W-1:0]  nxt_data1  [0:N_BANKS-1];
    reg [BANK_AW-1:0] nxt_addr1  [0:N_BANKS-1];
    reg [1:0]         nxt_count  [0:N_BANKS-1];

    // Immediate write slot (goes to SRAM this cycle)
    reg [N_BANKS-1:0] imm_en;
    reg [BANK_AW-1:0] imm_addr [0:N_BANKS-1];
    reg [DATA_W-1:0]  imm_data [0:N_BANKS-1];

    // [Issue 3] was conflict_r -> now conflict_w (combinational, driven by always @(*))
    reg [N_BANKS-1:0] conflict_w;

    integer arb_b_v;    // bank loop in arbitration always @(*)
    integer arb_w_v;    // write-port loop in arbitration
    integer arb_tb_v;   // target bank (temp, replaces 'automatic integer tb')
    integer arb_ta_v;   // target addr (temp, replaces 'automatic integer ta')
    integer pend_b_v;   // bank loop in pending register always @(posedge clk)
    integer rv_v;       // read-valid/register loop
    integer out_r_v;    // output mux read-port loop
    integer out_b_v;    // output mux bank loop

    // =========================================================
    // Write Arbitration (Combinational)
    //
    // Algorithm:
    //   Step 1. Drain head of each bank's pending FIFO -> imm slot.
    //           Shift slot1 -> slot0, decrement count.
    //   Step 2. Walk write ports 0..N_WRITE-1:
    //     a. If imm[bank] free         -> write immediately.
    //     b. Else if nxt_count[bank]==0 -> pending slot 0.
    //     c. Else if nxt_count[bank]==1 -> pending slot 1.
    //     d. Else (both slots full)    -> overflow: $error + last-write-wins.
    //
    // Precondition (upstream must ensure):
    //   ≤ 2 new writes per bank per cycle (1 imm + 1 pending absorbed).
    //   Violation triggers simulation $error but does not deadlock hardware.
    // =========================================================
    always @(*) begin
        // --- Initialise: drain pending slot 0 to imm ---
        for (arb_b_v = 0; arb_b_v < N_BANKS; arb_b_v = arb_b_v + 1) begin
            conflict_w[arb_b_v]   = 1'b0;
            nxt_addr1[arb_b_v]    = {BANK_AW{1'b0}};
            nxt_data1[arb_b_v]    = {DATA_W{1'b0}};
            if (pend_count_r[arb_b_v] != 2'b00) begin
                // Drain slot 0 -> imm
                imm_en  [arb_b_v] = 1'b1;
                imm_addr[arb_b_v] = pend_addr0_r[arb_b_v];
                imm_data[arb_b_v] = pend_data0_r[arb_b_v];
                // Shift slot 1 -> slot 0
                nxt_addr0[arb_b_v]  = pend_addr1_r[arb_b_v];
                nxt_data0[arb_b_v]  = pend_data1_r[arb_b_v];
                nxt_count[arb_b_v]  = pend_count_r[arb_b_v] - 2'b01;
            end else begin
                imm_en  [arb_b_v] = 1'b0;
                imm_addr[arb_b_v] = {BANK_AW{1'b0}};
                imm_data[arb_b_v] = {DATA_W{1'b0}};
                nxt_addr0[arb_b_v]  = {BANK_AW{1'b0}};
                nxt_data0[arb_b_v]  = {DATA_W{1'b0}};
                nxt_count[arb_b_v]  = 2'b00;
            end
        end

        // --- Process new write ports in order 0..N_WRITE-1 ---
        for (arb_w_v = 0; arb_w_v < N_WRITE; arb_w_v = arb_w_v + 1) begin
            if (wr_en_i[arb_w_v]) begin
                // [Issue 1] compute into named module-level integers, not 'automatic'
                arb_tb_v = get_bank    (wr_addr_i[arb_w_v*ROB_TAG_W +: ROB_TAG_W]);
                arb_ta_v = get_bank_addr(wr_addr_i[arb_w_v*ROB_TAG_W +: ROB_TAG_W]);

                if (!imm_en[arb_tb_v]) begin
                    // Case a: imm slot free
                    imm_en  [arb_tb_v] = 1'b1;
                    imm_addr[arb_tb_v] = arb_ta_v[BANK_AW-1:0];
                    imm_data[arb_tb_v] = wr_data_i[arb_w_v*DATA_W +: DATA_W];

                end else if (nxt_count[arb_tb_v] == 2'b00) begin
                    // Case b: pending empty -> slot 0
                    nxt_addr0[arb_tb_v] = arb_ta_v[BANK_AW-1:0];
                    nxt_data0[arb_tb_v] = wr_data_i[arb_w_v*DATA_W +: DATA_W];
                    nxt_count[arb_tb_v] = 2'b01;
                    conflict_w[arb_tb_v] = 1'b1;

                end else if (nxt_count[arb_tb_v] == 2'b01) begin
                    // Case c: slot 0 taken -> slot 1
                    nxt_addr1[arb_tb_v] = arb_ta_v[BANK_AW-1:0];
                    nxt_data1[arb_tb_v] = wr_data_i[arb_w_v*DATA_W +: DATA_W];
                    nxt_count[arb_tb_v] = 2'b10;
                    conflict_w[arb_tb_v] = 1'b1;

                end else begin
                    // Case d: both slots full – overflow
                    // Last-write-wins on slot 1 (documented behavior).
                    // Upstream CDB arbitration should prevent this.
                    nxt_addr1[arb_tb_v] = arb_ta_v[BANK_AW-1:0];
                    nxt_data1[arb_tb_v] = wr_data_i[arb_w_v*DATA_W +: DATA_W];
                    conflict_w[arb_tb_v] = 1'b1;

                end
            end
        end
    end

    assign wr_conflict_o = conflict_w;

    // Pending FIFO Register Update
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            for (pend_b_v = 0; pend_b_v < N_BANKS; pend_b_v = pend_b_v + 1) begin
                pend_count_r[pend_b_v] <= 2'b00;
                pend_addr0_r[pend_b_v] <= {BANK_AW{1'b0}};
                pend_data0_r[pend_b_v] <= {DATA_W{1'b0}};
                pend_addr1_r[pend_b_v] <= {BANK_AW{1'b0}};
                pend_data1_r[pend_b_v] <= {DATA_W{1'b0}};
            end
        end else begin
            for (pend_b_v = 0; pend_b_v < N_BANKS; pend_b_v = pend_b_v + 1) begin
                pend_count_r[pend_b_v] <= nxt_count[pend_b_v];
                pend_addr0_r[pend_b_v] <= nxt_addr0[pend_b_v];
                pend_data0_r[pend_b_v] <= nxt_data0[pend_b_v];
                pend_addr1_r[pend_b_v] <= nxt_addr1[pend_b_v];
                pend_data1_r[pend_b_v] <= nxt_data1[pend_b_v];
            end
        end
    end

    // BRAM read replicas.
    //
    // One physical BRAM can only provide a small number of synchronous read
    // ports.  Replicating each bank per read port preserves the existing
    // multi-read interface while keeping every individual memory as a simple
    // one-write/one-read RAM.  The final output mux now selects only between
    // N_BANKS banks instead of a full ROB_DEPTH-wide data mux.
    wire [N_READ*BANK_SEL_W-1:0] rd_bank_w;
    wire [N_READ*BANK_AW-1:0]    rd_bank_addr_w;

    genvar ra;
    generate
        for (ra = 0; ra < N_READ; ra = ra + 1) begin : rd_addr_decode_gen
            wire [ROB_TAG_W-1:0] raddr =
                rd_addr_i[ra*ROB_TAG_W +: ROB_TAG_W];
            assign rd_bank_w     [ra*BANK_SEL_W +: BANK_SEL_W] = get_bank(raddr);
            assign rd_bank_addr_w[ra*BANK_AW    +: BANK_AW]    = get_bank_addr(raddr);
        end
    endgenerate

    wire [N_READ*N_BANKS*DATA_W-1:0] rd_bank_data_w;

    genvar rr, rb;
    generate
        for (rr = 0; rr < N_READ; rr = rr + 1) begin : rd_replica_gen
            for (rb = 0; rb < N_BANKS; rb = rb + 1) begin : bank_mem_gen
                localparam [BANK_SEL_W-1:0] BANK_ID = rb[BANK_SEL_W-1:0];
                (* ram_style = "block", rw_addr_collision = "yes" *)
                reg [DATA_W-1:0] bank_mem [0:BANK_DEPTH-1];
                reg [DATA_W-1:0] rd_data_r;

                wire rd_bank_hit_w =
                    rd_en_i[rr] &&
                    (rd_bank_w[rr*BANK_SEL_W +: BANK_SEL_W] == BANK_ID);

                always @(posedge clk) begin
                    if (!flush_i && imm_en[rb])
                        bank_mem[imm_addr[rb]] <= imm_data[rb];
                    if (rd_bank_hit_w)
                        rd_data_r <= bank_mem[rd_bank_addr_w[rr*BANK_AW +: BANK_AW]];
                end

                assign rd_bank_data_w[(rr*N_BANKS + rb)*DATA_W +: DATA_W] =
                    rd_data_r;
            end
        end
    endgenerate

    reg [N_READ*BANK_SEL_W-1:0] rd_bank_q;
    reg [N_READ-1:0]            rd_valid_q;
    reg [DATA_W-1:0]            rd_data_mux [0:N_READ-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_i) begin
            rd_bank_q  <= {N_READ*BANK_SEL_W{1'b0}};
            rd_valid_q <= {N_READ{1'b0}};
        end else begin
            rd_valid_q <= rd_en_i;
            for (rv_v = 0; rv_v < N_READ; rv_v = rv_v + 1)
                rd_bank_q[rv_v*BANK_SEL_W +: BANK_SEL_W] <=
                    rd_bank_w[rv_v*BANK_SEL_W +: BANK_SEL_W];
        end
    end

    always @(*) begin
        for (out_r_v = 0; out_r_v < N_READ; out_r_v = out_r_v + 1) begin
            rd_data_mux[out_r_v] = {DATA_W{1'b0}};
            if (rd_valid_q[out_r_v]) begin
                for (out_b_v = 0; out_b_v < N_BANKS; out_b_v = out_b_v + 1) begin
                    if (rd_bank_q[out_r_v*BANK_SEL_W +: BANK_SEL_W] ==
                        out_b_v[BANK_SEL_W-1:0]) begin
                        rd_data_mux[out_r_v] =
                            rd_bank_data_w[(out_r_v*N_BANKS + out_b_v)*DATA_W +: DATA_W];
                    end
                end
            end
        end
    end

    // Output (registered by the synchronous BRAM read)
    generate
        if (PIPELINE_OUTPUT) begin : pipe_out
            genvar gr;
            for (gr = 0; gr < N_READ; gr = gr + 1) begin : out_pipe
                assign rd_data_o [gr*DATA_W +: DATA_W] = rd_data_mux[gr];
                assign rd_valid_o[gr]                   = rd_valid_q[gr];
            end

        end else begin : comb_out
            genvar gr;
            for (gr = 0; gr < N_READ; gr = gr + 1) begin : out_comb
                assign rd_data_o [gr*DATA_W +: DATA_W] = rd_data_mux[gr];
                assign rd_valid_o[gr]                   = rd_valid_q[gr];
            end
        end
    endgenerate

endmodule
