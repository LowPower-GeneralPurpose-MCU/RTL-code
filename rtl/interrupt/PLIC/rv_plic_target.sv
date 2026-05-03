// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// RISC-V Platform-Level Interrupt Generator for Target
//
// Finds the highest-priority pending+enabled interrupt that exceeds the
// per-target threshold and drives irq / irq_id to the CPU hart.
//
// ALGORITHM selects the implementation:
//   SEQUENTIAL  – simple for-loop, O(N) depth, smallest area
//   BINARY_TREE – balanced compare tree, O(logN) depth, O(N) comparators
//   MATRIX      – N×N comparator matrix, depth=1, largest area

module rv_plic_target #(
  parameter int    N_SOURCE  = 32,
  parameter int    MAX_PRIO  = 7,
  parameter string ALGORITHM = "SEQUENTIAL", // SEQUENTIAL | MATRIX | BINARY_TREE

  // Derived – do not override
  parameter int unsigned SRCW  = $clog2(N_SOURCE+1),
  parameter int unsigned PRIOW = $clog2(MAX_PRIO+1)
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  logic [N_SOURCE-1:0]            ip,
  input  logic [N_SOURCE-1:0]            ie,
  input  logic [N_SOURCE-1:0][PRIOW-1:0] prio,
  input  logic              [PRIOW-1:0]  threshold,

  output logic             irq,
  output logic [SRCW-1:0]  irq_id
);

  // -----------------------------------------------------------------------
  // SEQUENTIAL – iterate sources high-to-low index; last write wins so
  // lower index wins ties (spec: lower ID takes priority when equal prio).
  // -----------------------------------------------------------------------
  if (ALGORITHM == "SEQUENTIAL") begin : gen_sequential

    logic [PRIOW-1:0] max_prio;
    logic             irq_next;
    logic [SRCW-1:0]  irq_id_next;

    always_comb begin
      max_prio    = threshold + 1'b1;
      irq_id_next = '0;
      irq_next    = 1'b0;
      // Iterate high-to-low so lower index overwrites, winning ties
      for (int i = N_SOURCE-1; i >= 0; i--) begin
        if (ip[i] & ie[i] & (prio[i] >= max_prio)) begin
          max_prio    = prio[i];
          irq_id_next = SRCW'(i + 1);
          irq_next    = 1'b1;
        end
      end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        irq    <= 1'b0;
        irq_id <= '0;
      end else begin
        irq    <= irq_next;
        irq_id <= irq_id_next;
      end
    end

  // -----------------------------------------------------------------------
  // BINARY_TREE – O(logN) depth balanced tournament tree.
  // Tie-break: lower ID (smaller index) wins.
  // -----------------------------------------------------------------------
  end else if (ALGORITHM == "BINARY_TREE") begin : gen_tree

    localparam int unsigned LEVELS = (N_SOURCE <= 1) ? 0 : $clog2(N_SOURCE);
    localparam int unsigned LEAFS  = (N_SOURCE <= 1) ? 1 : (1 << LEVELS);

    logic [LEVELS:0][LEAFS-1:0]            node_vld;
    logic [LEVELS:0][LEAFS-1:0][PRIOW-1:0] node_prio;
    logic [LEVELS:0][LEAFS-1:0][SRCW-1:0]  node_id;

    logic             irq_next;
    logic [SRCW-1:0]  irq_id_next;

    always_comb begin
      // Leaf initialisation
      for (int i = 0; i < LEAFS; i++) begin
        if ((i < N_SOURCE) && ip[i] && ie[i] && (prio[i] > threshold)) begin
          node_vld [0][i] = 1'b1;
          node_prio[0][i] = prio[i];
          node_id  [0][i] = SRCW'(i + 1);
        end else begin
          node_vld [0][i] = 1'b0;
          node_prio[0][i] = '0;
          node_id  [0][i] = '0;
        end
      end

      // Tournament levels
      for (int lvl = 0; lvl < LEVELS; lvl++) begin
        for (int j = 0; j < (LEAFS >> (lvl + 1)); j++) begin
          automatic logic             l_v  = node_vld [lvl][2*j];
          automatic logic             r_v  = node_vld [lvl][2*j+1];
          automatic logic [PRIOW-1:0] l_p  = node_prio[lvl][2*j];
          automatic logic [PRIOW-1:0] r_p  = node_prio[lvl][2*j+1];
          automatic logic [SRCW-1:0]  l_id = node_id  [lvl][2*j];
          automatic logic [SRCW-1:0]  r_id = node_id  [lvl][2*j+1];

          if (l_v && !r_v) begin
            node_vld [lvl+1][j] = 1'b1;
            node_prio[lvl+1][j] = l_p;
            node_id  [lvl+1][j] = l_id;
          end else if (!l_v && r_v) begin
            node_vld [lvl+1][j] = 1'b1;
            node_prio[lvl+1][j] = r_p;
            node_id  [lvl+1][j] = r_id;
          end else if (!l_v && !r_v) begin
            node_vld [lvl+1][j] = 1'b0;
            node_prio[lvl+1][j] = '0;
            node_id  [lvl+1][j] = '0;
          end else if (l_p > r_p) begin
            node_vld [lvl+1][j] = 1'b1;
            node_prio[lvl+1][j] = l_p;
            node_id  [lvl+1][j] = l_id;
          end else if (l_p < r_p) begin
            node_vld [lvl+1][j] = 1'b1;
            node_prio[lvl+1][j] = r_p;
            node_id  [lvl+1][j] = r_id;
          end else begin
            // Equal priority: lower ID wins
            node_vld [lvl+1][j] = 1'b1;
            node_prio[lvl+1][j] = l_p;
            node_id  [lvl+1][j] = (l_id <= r_id) ? l_id : r_id;
          end
        end
      end

      irq_next    = node_vld [LEVELS][0];
      irq_id_next = node_id  [LEVELS][0];
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        irq    <= 1'b0;
        irq_id <= '0;
      end else begin
        irq    <= irq_next;
        irq_id <= irq_id_next;
      end
    end

  // -----------------------------------------------------------------------
  // MATRIX – N×N comparator matrix; depth = 1 comparator + logN AND tree.
  // Highest-priority source with all mat[i][j]=1 is selected via LOD.
  // Tie-break: lowest index (LOD picks lowest set bit).
  // -----------------------------------------------------------------------
  end else if (ALGORITHM == "MATRIX") begin : gen_mat

    logic [N_SOURCE-1:0]               is;          // ip & ie
    logic [N_SOURCE-1:0][N_SOURCE-1:0] mat;
    logic [N_SOURCE-1:0]               merged_row;
    logic [N_SOURCE-1:0]               lod;         // leading-one detector

    assign is = ip & ie;

    always_comb begin
      // Last source: just check threshold
      merged_row[N_SOURCE-1] = is[N_SOURCE-1] & (prio[N_SOURCE-1] > threshold);

      for (int i = 0; i < N_SOURCE-1; i++) begin
        merged_row[i] = 1'b1;
        for (int j = i+1; j < N_SOURCE; j++) begin
          // mat[i][j] = 1 when source i beats or ties source j
          mat[i][j] = (prio[i] <= threshold) ? 1'b0 :
                      (is[i] & is[j])         ? (prio[i] >= prio[j]) :
                      is[i]                   ? 1'b1 : 1'b0;
          merged_row[i] = merged_row[i] & mat[i][j];
        end
      end
    end

    // Isolate lowest set bit → lowest-index winner (tie-break)
    assign lod = merged_row & (~merged_row + 1'b1);

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        irq    <= 1'b0;
        irq_id <= '0;
      end else if (|lod) begin
        irq <= 1'b1;
        for (int i = N_SOURCE-1; i >= 0; i--) begin
          if (lod[i]) irq_id <= SRCW'(i + 1);
        end
      end else begin
        irq    <= 1'b0;
        irq_id <= '0;
      end
    end

  end // ALGORITHM

endmodule
