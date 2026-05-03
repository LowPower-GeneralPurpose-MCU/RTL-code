// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// RISC-V Platform-Level Interrupt Controller (PLIC) – top level
//
// Parameters
//   N_SOURCE   – number of interrupt sources  (default 32)
//   N_TARGET   – number of interrupt targets / CPU harts (default 1)
//   MAX_PRIO   – maximum priority value
//   FIND_MAX   – "SEQUENTIAL" | "BINARY_TREE" | "MATRIX"
//   EDGE_CNT_W – width of per-source edge counter (default 2)
//
// Interrupt ID 0 is reserved ("no interrupt"); valid IDs are 1..N_SOURCE.

module rv_plic #(
  parameter int    N_SOURCE   = 32,
  parameter int    N_TARGET   = 1,
  parameter int    MAX_PRIO   = 7,
  parameter string FIND_MAX   = "SEQUENTIAL",
  parameter int    EDGE_CNT_W = 2,
  // Derived – do not override
  parameter int    SRCW       = $clog2(N_SOURCE+1)
) (
  input  logic clk_i,
  input  logic rst_ni,

  // TL-UL register bus
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Interrupt sources
  input  logic [N_SOURCE-1:0] intr_src_i,

  // Interrupt notification to targets
  output logic            irq_o    [N_TARGET],
  output logic [SRCW-1:0] irq_id_o [N_TARGET]
);

  localparam int unsigned PRIOW = $clog2(MAX_PRIO+1);

  // -------------------------------------------------------------------------
  // Internal signals
  // -------------------------------------------------------------------------
  logic [N_SOURCE-1:0]               le;
  logic [N_SOURCE-1:0]               ip;
  logic [N_SOURCE-1:0]               gateway_src;
  logic [N_SOURCE-1:0]               msi_set;

  logic [N_TARGET-1:0][N_SOURCE-1:0] ie;
  logic [N_SOURCE-1:0][PRIOW-1:0]    prio;
  logic [N_TARGET-1:0][PRIOW-1:0]    threshold;

  logic [N_TARGET-1:0]               claim_re;
  logic [N_TARGET-1:0][SRCW-1:0]     claim_id;
  logic [N_SOURCE-1:0]               claim;

  logic [N_TARGET-1:0]               complete_we;
  logic [N_TARGET-1:0][SRCW-1:0]     complete_id;
  logic [N_SOURCE-1:0]               complete;

  // -------------------------------------------------------------------------
  // claim_id comes from irq_id_o (target output)
  // -------------------------------------------------------------------------
  always_comb begin
    for (int i = 0; i < N_TARGET; i++)
      claim_id[i] = irq_id_o[i];
  end

  // -------------------------------------------------------------------------
  // Decode claim_re + claim_id -> one-hot claim
  // -------------------------------------------------------------------------
  always_comb begin
    claim = '0;
    for (int i = 0; i < N_TARGET; i++) begin
      if (claim_re[i] && (claim_id[i] != '0) &&
          (int'(claim_id[i]) <= N_SOURCE)) begin
        claim[claim_id[i]-1] = 1'b1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Decode complete_we + complete_id -> one-hot complete
  // -------------------------------------------------------------------------
  always_comb begin
    complete = '0;
    for (int i = 0; i < N_TARGET; i++) begin
      if (complete_we[i] && (complete_id[i] != '0) &&
          (int'(complete_id[i]) <= N_SOURCE)) begin
        complete[complete_id[i]-1] = 1'b1;
      end
    end
  end

  // MSI doorbell merged with hw interrupt sources
  assign gateway_src = intr_src_i | msi_set;

  // Sub-module: gateway
  rv_plic_gateway #(
    .N_SOURCE   (N_SOURCE),
    .EDGE_CNT_W (EDGE_CNT_W)
  ) u_gateway (
    .clk_i,
    .rst_ni,
    .src      (gateway_src),
    .le       (le),
    .claim    (claim),
    .complete (complete),
    .ip       (ip)
  );

  // Sub-module: one target per hart
  for (genvar i = 0; i < N_TARGET; i++) begin : gen_target
    rv_plic_target #(
      .N_SOURCE  (N_SOURCE),
      .MAX_PRIO  (MAX_PRIO),
      .ALGORITHM (FIND_MAX)
    ) u_target (
      .clk_i,
      .rst_ni,
      .ip        (ip),
      .ie        (ie[i]),
      .prio      (prio),
      .threshold (threshold[i]),
      .irq       (irq_o[i]),
      .irq_id    (irq_id_o[i])
    );
  end
  // Sub-module: register map
  rv_plic_regmap #(
    .N_SOURCE (N_SOURCE),
    .N_TARGET (N_TARGET),
    .MAX_PRIO (MAX_PRIO),
    .SRCW     (SRCW),
    .PRIOW    (PRIOW)
  ) u_regmap (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .ip_i          (ip),
    .claim_id_i    (claim_id),
    .le_o          (le),
    .prio_o        (prio),
    .ie_o          (ie),
    .threshold_o   (threshold),
    .claim_re_o    (claim_re),
    .complete_we_o (complete_we),
    .complete_id_o (complete_id),
    .msi_set_o     (msi_set)
  );

endmodule