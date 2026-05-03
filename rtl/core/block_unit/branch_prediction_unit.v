module branch_prediction_unit (
    input clk, reset_n,
    input [31:0] pc_in, ex_mem_pc_in,
    input ex_mem_branch, ex_mem_branch_taken, ex_mem_predict_taken, ex_mem_btb_hit,
    input [31:0] ex_mem_branch_target,
    output bpu_correct, predict_taken, btb_hit, actual_taken,
    output [31:0] predict_target
);
    assign actual_taken = ex_mem_branch && ex_mem_branch_taken;
    assign bpu_correct = (ex_mem_predict_taken == actual_taken);
    wire [1:0] update_btb = ((!ex_mem_btb_hit && ex_mem_branch && actual_taken) || 
                            (ex_mem_btb_hit && ex_mem_branch && !ex_mem_predict_taken && actual_taken)) ? 2'b01 :
                            ((ex_mem_btb_hit && !ex_mem_branch) ? 2'b10 : 2'b00);
    wire update_bht = ex_mem_branch;

    branch_target_buffer BTB (
        .clk(clk),
        .reset_n(reset_n),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .update_btb(update_btb), 
        .actual_target(ex_mem_branch_target),
        .predict_target(predict_target),
        .btb_hit(btb_hit)
    );

    branch_history_table BHT (
        .clk(clk), 
        .reset_n(reset_n),
        .pc_in(pc_in),
        .ex_mem_pc_in(ex_mem_pc_in),
        .update_bht(update_bht),
        .btb_hit(btb_hit),
        .actual_taken(actual_taken),
        .predict_taken(predict_taken)
    );
endmodule

module branch_target_buffer (
    input clk, reset_n,
    input [31:0] pc_in, ex_mem_pc_in,
    input [1:0] update_btb,
    input [31:0] actual_target,
    output [31:0] predict_target,
    output btb_hit
);
    parameter ENTRY = 256;
    parameter INDEX = 8;
    parameter TAG = 22;
    parameter TARGET_ADDR = 30;

    wire [29:0] lookup_addr   = pc_in[31:2];
    wire [TAG-1:0] lookup_tag = lookup_addr[29:8];
    wire [INDEX-1:0] lookup_index = lookup_addr[7:0];

    wire [29:0] update_addr   = ex_mem_pc_in[31:2];
    wire [TAG-1:0] update_tag = update_addr[29:8];
    wire [INDEX-1:0] update_index = update_addr[7:0];

    reg [TAG-1:0] tags [0:ENTRY-1];
    reg [TARGET_ADDR-1:0] targets [0:ENTRY-1];
    reg valids [0:ENTRY-1];

    assign btb_hit = valids[lookup_index] && (tags[lookup_index] == lookup_tag);
    assign predict_target = btb_hit ? {targets[lookup_index], 2'b00} : (pc_in + 32'd4);

    integer i;
    always @(posedge clk) begin
        if (!reset_n) begin
            for (i = 0; i < ENTRY; i = i + 1) begin
                valids[i]  <= 1'b0;
                tags[i]    <= {TAG{1'b0}};
                targets[i] <= {TARGET_ADDR{1'b0}};
            end
        end else begin
            case (update_btb)
                2'b01: begin
                    tags[update_index]    <= update_tag;
                    targets[update_index] <= actual_target[31:2];
                    valids[update_index]  <= 1'b1;
                end

                2'b10: begin
                    tags[update_index]    <= {TAG{1'b0}};
                    targets[update_index] <= {TARGET_ADDR{1'b0}};
                    valids[update_index]  <= 1'b0;
                end
            endcase
        end
    end
endmodule

module branch_history_table (
    input clk, reset_n,
    input [31:0] pc_in, ex_mem_pc_in,
    input update_bht,
    input btb_hit,
    input actual_taken,
    output predict_taken
);
    parameter ENTRY = 64;
    parameter INDEX = 6;
    parameter PREDICT = 2;

    wire [INDEX-1:0] lookup_index = pc_in[7:2];
    wire [INDEX-1:0] update_index = ex_mem_pc_in[7:2];

    reg [PREDICT-1:0] predicts [0:ENTRY-1];

    // 00: strongly NT
    // 01: weakly NT
    // 10: weakly T
    // 11: strongly T
    assign predict_taken = btb_hit && predicts[lookup_index][1];

    integer i;
    always @(posedge clk) begin
        if (!reset_n) begin
            for (i = 0; i < ENTRY; i = i + 1) begin
                predicts[i] <= 2'b10;   // giữ như bản cũ: weakly taken
            end
        end else if (update_bht) begin
            if (actual_taken) begin
                if (predicts[update_index] != 2'b11)
                    predicts[update_index] <= predicts[update_index] + 2'b01;
            end else begin
                if (predicts[update_index] != 2'b00)
                    predicts[update_index] <= predicts[update_index] - 2'b01;
            end
        end
    end
endmodule
