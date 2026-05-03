`timescale 1ns/1ps

module cdb_bypass_mux #(
    parameter DATA_W            = 64,
    parameter TAG_W             = 6,
    parameter N_CDB             = 4,
    parameter REGISTERED_OUTPUT = 0   // 1 = register output for timing closure
)(
    input  wire                      clk,    // FIX Bug 3: added port
    input  wire                      rst_n,  // FIX Bug 3: added port

    input  wire [TAG_W-1:0]          src_tag_i,
    input  wire                      src_valid_i,

    input  wire [N_CDB*TAG_W-1:0]    cdb_tag_i,
    input  wire [N_CDB*DATA_W-1:0]   cdb_data_i,
    input  wire [N_CDB-1:0]          cdb_valid_i,

    output wire [DATA_W-1:0]         fwd_data_o,
    output wire                      fwd_hit_o
);

    wire [N_CDB-1:0] match;
    genvar k;
    generate
        for (k = 0; k < N_CDB; k = k + 1) begin : cmp_gen
            assign match[k] = cdb_valid_i[k] &&
                               src_valid_i   &&
                               (cdb_tag_i[k*TAG_W +: TAG_W] == src_tag_i);
        end
    endgenerate

    reg [DATA_W-1:0] sel_data;
    integer m;
    always @(*) begin
        sel_data = {DATA_W{1'b0}};
        for (m = N_CDB-1; m >= 0; m = m - 1)
            if (match[m])
                sel_data = cdb_data_i[m*DATA_W +: DATA_W];
    end

    // Combinational hit signal (used by both output paths)
    wire hit_comb = |match;
    generate
        if (REGISTERED_OUTPUT) begin : reg_out
            reg [DATA_W-1:0] fwd_data_r;
            reg              fwd_hit_r;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    fwd_data_r <= {DATA_W{1'b0}};
                    fwd_hit_r  <= 1'b0;
                end else begin
                    fwd_data_r <= sel_data;
                    fwd_hit_r  <= hit_comb;  // FIX: registered, aligned with fwd_data_r
                end
            end

            assign fwd_data_o = fwd_data_r;
            assign fwd_hit_o  = fwd_hit_r;   // FIX: also registered now

        end else begin : comb_out
            assign fwd_data_o = sel_data;
            assign fwd_hit_o  = hit_comb;
        end
    endgenerate

endmodule