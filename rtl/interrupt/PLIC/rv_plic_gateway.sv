// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// RISC-V Platform-Level Interrupt Gateways module

module rv_plic_gateway #(
  parameter int N_SOURCE   = 32,
  parameter int EDGE_CNT_W = 2   // Per-source queued edge counter width
) (
  input clk_i,
  input rst_ni,

  input  logic [N_SOURCE-1:0] src,
  input  logic [N_SOURCE-1:0] le,      // 0:level  1:edge

  input  logic [N_SOURCE-1:0] claim,    // $onehot0(claim)
  input  logic [N_SOURCE-1:0] complete, // $onehot0(complete)

  output logic [N_SOURCE-1:0] ip
);

  logic [N_SOURCE-1:0] ia;          // Interrupt Active

  logic [N_SOURCE-1:0] set_raw;     // (le) ? rising-edge : level
  logic [N_SOURCE-1:0] replay_set;  // replay one queued edge when slot free
  logic [N_SOURCE-1:0] set_eff;
  logic [N_SOURCE-1:0] src_d;

  logic [N_SOURCE-1:0][EDGE_CNT_W-1:0] edge_cnt;

  localparam logic [EDGE_CNT_W-1:0] EdgeCntMax = {EDGE_CNT_W{1'b1}};

  // Delay src by one cycle for edge detection
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) src_d <= '0;
    else         src_d <= src;
  end

  // Combinational set / replay signals
  always_comb begin
    for (int i = 0; i < N_SOURCE; i++) begin
      set_raw[i]    = le[i] ? (src[i] & ~src_d[i]) : src[i];
      replay_set[i] = le[i] & ~ia[i] & ~ip[i] & (|edge_cnt[i]);
      set_eff[i]    = set_raw[i] | replay_set[i];
    end
  end

  // ip: set by source (gated by ia/ip), cleared by claim
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) ip <= '0;
    else         ip <= (ip | (set_eff & ~ia & ~ip)) & ~claim;
  end

  // ia: tracks active interrupt, cleared by complete
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) ia <= '0;
    else         ia <= (ia | (set_eff & ~ia)) & ~complete;
  end

  // edge_cnt: saturating counter queuing edges while gateway busy
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      edge_cnt <= '0;
    end else begin
      for (int i = 0; i < N_SOURCE; i++) begin
        if (!le[i]) begin
          edge_cnt[i] <= '0;
        end else begin
          unique case ({(set_raw[i] & (ia[i] | ip[i] | replay_set[i])), replay_set[i]})
            2'b10:   edge_cnt[i] <= (edge_cnt[i] == EdgeCntMax) ? edge_cnt[i]
                                                                 : edge_cnt[i] + 1'b1;
            2'b01:   edge_cnt[i] <= (edge_cnt[i] == '0)         ? edge_cnt[i]
                                                                 : edge_cnt[i] - 1'b1;
            default: edge_cnt[i] <= edge_cnt[i];
          endcase
        end
      end
    end
  end

endmodule
