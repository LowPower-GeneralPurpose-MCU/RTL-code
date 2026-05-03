// Copyright lowRISC contributors (adapted).
// SPDX-License-Identifier: Apache-2.0
//
// rv_plic_regmap — RISC-V PLIC spec-compliant register map
//
// Address layout (FIXED per RISC-V PLIC spec v1.0):
//
//   0x000000 + 4*ID   : Priority[ID]         ID = 0..N_SOURCE  (ID=0 hardwired=0)
//   0x001000 + 4*w    : Pending[w]            w = 0..ceil(N_SOURCE/32)-1  (read-only)
//   0x002000 + 0x80*t + 4*w : IE[t][w]        t = 0..N_TARGET-1
//   0x200000 + 0x1000*t     : Threshold[t]
//   0x200004 + 0x1000*t     : Claim/Complete[t]
//
// Extra (non-spec, vendor extension):
//   0x1F0000          : LE bitmap (level/edge select) — NOT at spec-breaking position
//   0x1F0004          : MSI doorbell write-only
//
// Compatible string: "riscv,plic0" (SiFive-style, supported by Linux/OpenSBI/xv6)

module rv_plic_regmap #(
  parameter int N_SOURCE = 32,
  parameter int N_TARGET = 1,
  parameter int MAX_PRIO = 7,
  parameter int SRCW     = $clog2(N_SOURCE+1),
  parameter int PRIOW    = $clog2(MAX_PRIO+1)
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  input  logic [N_SOURCE-1:0]            ip_i,
  input  logic [N_TARGET-1:0][SRCW-1:0]  claim_id_i,

  output logic [N_SOURCE-1:0]                   le_o,
  output logic [N_SOURCE-1:0][PRIOW-1:0]        prio_o,
  output logic [N_TARGET-1:0][N_SOURCE-1:0]     ie_o,
  output logic [N_TARGET-1:0][PRIOW-1:0]        threshold_o,

  output logic [N_TARGET-1:0]            claim_re_o,
  output logic [N_TARGET-1:0]            complete_we_o,
  output logic [N_TARGET-1:0][SRCW-1:0]  complete_id_o,

  output logic [N_SOURCE-1:0]            msi_set_o
);

  localparam int unsigned DW  = top_pkg::TL_DW;
  localparam int unsigned DBW = top_pkg::TL_DBW;
  localparam logic [top_pkg::TL_SZW-1:0] FSZ = top_pkg::TL_SZW'($clog2(DBW));

  // -----------------------------------------------------------------------
  // Spec-fixed base offsets
  // -----------------------------------------------------------------------
  localparam logic [31:0] PRIO_BASE  = 32'h000000;  // 0x4*ID per source
  localparam logic [31:0] PEND_BASE  = 32'h001000;  // read-only pending
  localparam logic [31:0] IE_BASE    = 32'h002000;  // 0x80 per context
  localparam logic [31:0] CTX_BASE   = 32'h200000;  // 0x1000 per context
  // Vendor extensions — placed far from spec region, no conflict
  localparam logic [31:0] LE_ADDR    = 32'h1F0000;
  localparam logic [31:0] MSI_ADDR   = 32'h1F0004;

  localparam int unsigned PEND_WORDS = (N_SOURCE + DW - 1) / DW;
  localparam int unsigned IE_WORDS   = (N_SOURCE + DW - 1) / DW;  // per target

  // -----------------------------------------------------------------------
  // Registered state
  // -----------------------------------------------------------------------
  logic [N_SOURCE-1:0][PRIOW-1:0]    prio_q;
  logic [N_SOURCE-1:0]               le_q;
  logic [N_TARGET-1:0][N_SOURCE-1:0] ie_q;
  logic [N_TARGET-1:0][PRIOW-1:0]    threshold_q;

  logic [N_TARGET-1:0]               claim_re_q;
  logic [N_TARGET-1:0]               complete_we_q;
  logic [N_TARGET-1:0][SRCW-1:0]     complete_id_q;
  logic [N_SOURCE-1:0]               msi_set_q;

  logic                              outstanding_q;
  logic [DW-1:0]                     rsp_data_q;
  tlul_pkg::tl_d_op_e                rsp_opcode_q;
  logic [top_pkg::TL_AIW-1:0]        rsp_source_q;
  logic                              rsp_error_q;

  // -----------------------------------------------------------------------
  // Combinational decode
  // -----------------------------------------------------------------------
  logic req_fire, req_is_write, req_is_read, req_malformed;
  logic [DW-1:0] rsp_data_d;
  logic addr_error_d;
  logic wr_prio_d, wr_ie_d, wr_threshold_d, wr_cc_d, wr_le_d, wr_msi_d;
  logic rd_cc_d;
  logic [$clog2(N_SOURCE+1)-1:0] prio_idx_d;
  logic [$clog2(N_TARGET > 1 ? N_TARGET : 2)-1:0] ctx_idx_d;
  logic [$clog2(IE_WORDS > 1 ? IE_WORDS*N_TARGET : 2)-1:0] ie_word_d;

  assign req_fire     = tl_i.a_valid & tl_o.a_ready;
  assign req_is_write = (tl_i.a_opcode == tlul_pkg::PutFullData) |
                        (tl_i.a_opcode == tlul_pkg::PutPartialData);
  assign req_is_read  = (tl_i.a_opcode == tlul_pkg::Get);
  assign req_malformed = (!req_is_write & !req_is_read) |
                         (tl_i.a_size != FSZ) |
                         (tl_i.a_mask != {DBW{1'b1}}) |
                         (tl_i.a_user[8] == 1'b1) |
                         (tl_i.a_address[1:0] != 2'b00);

  always_comb begin
    int unsigned addr;
    int unsigned idx, t, w, bit_base;

    rsp_data_d     = '0;
    addr_error_d   = 1'b0;
    wr_prio_d      = 1'b0; wr_ie_d = 1'b0; wr_threshold_d = 1'b0;
    wr_cc_d        = 1'b0; wr_le_d = 1'b0; wr_msi_d = 1'b0;
    rd_cc_d        = 1'b0;
    prio_idx_d     = '0;
    ctx_idx_d      = '0;
    ie_word_d      = '0;

    addr = {tl_i.a_address[31:2], 2'b00};

    // ---- PRIORITY  0x000000 + 4*ID, ID=0..N_SOURCE ----
    if (addr >= PRIO_BASE && addr < PRIO_BASE + (N_SOURCE+1)*4) begin
      idx = (addr - PRIO_BASE) >> 2;            // idx 0 = source0 (reserved)
      if (idx == 0) begin
        // Source 0 priority hardwired to 0
        if (req_is_write) addr_error_d = 1'b1;  // writes ignored / error
      end else if (idx <= N_SOURCE) begin
        prio_idx_d = idx - 1;                   // map to 0-indexed array
        if (req_is_write) wr_prio_d = 1'b1;
        else rsp_data_d[PRIOW-1:0] = prio_q[idx-1];
      end else addr_error_d = 1'b1;

    // ---- PENDING  0x001000 (read-only) ----
    end else if (addr >= PEND_BASE && addr < PEND_BASE + PEND_WORDS*4) begin
      if (req_is_write) begin addr_error_d = 1'b1; end
      else begin
        w = (addr - PEND_BASE) >> 2;
        bit_base = w * DW;
        for (int b = 0; b < DW; b++)
          if ((bit_base + b) < N_SOURCE) rsp_data_d[b] = ip_i[bit_base + b];
      end

    // ---- IE  0x002000, stride 0x80 per context ----
    end else if (addr >= IE_BASE && addr < IE_BASE + N_TARGET*32'h80) begin
      t        = (addr - IE_BASE) >> 7;           // context index
      w        = ((addr - IE_BASE) & 32'h7F) >> 2; // word within context
      ie_word_d = w;
      ctx_idx_d = t;
      bit_base  = w * DW;
      if (t < N_TARGET) begin
        if (req_is_write) wr_ie_d = 1'b1;
        else for (int b = 0; b < DW; b++)
          if ((bit_base + b) < N_SOURCE) rsp_data_d[b] = ie_q[t][bit_base+b];
      end else addr_error_d = 1'b1;

    // ---- THRESHOLD / CLAIM  0x200000, stride 0x1000 per context ----
    end else if (addr >= CTX_BASE && addr < CTX_BASE + N_TARGET*32'h1000) begin
      t = (addr - CTX_BASE) >> 12;               // context index
      w = (addr - CTX_BASE) & 32'hFFF;           // offset within context
      ctx_idx_d = t;
      if (t < N_TARGET) begin
        if (w == 0) begin                         // Threshold at +0x000
          if (req_is_write) wr_threshold_d = 1'b1;
          else rsp_data_d[PRIOW-1:0] = threshold_q[t];
        end else if (w == 4) begin                // Claim/Complete at +0x004
          if (req_is_write) wr_cc_d = 1'b1;
          else begin rd_cc_d = 1'b1; rsp_data_d[SRCW-1:0] = claim_id_i[t]; end
        end else addr_error_d = 1'b1;
      end else addr_error_d = 1'b1;

    // ---- LE (vendor extension) ----
    end else if (addr == LE_ADDR) begin
      if (req_is_write) wr_le_d = 1'b1;
      else for (int b = 0; b < DW; b++)
        if (b < N_SOURCE) rsp_data_d[b] = le_q[b];

    // ---- MSI doorbell (vendor extension) ----
    end else if (addr == MSI_ADDR) begin
      if (req_is_write) wr_msi_d = 1'b1;

    end else addr_error_d = 1'b1;
  end

  // Sequential: registers + TL-UL response pipeline
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      prio_q        <= '0; le_q   <= '0;
      ie_q          <= '0; threshold_q <= '0;
      claim_re_q    <= '0; complete_we_q <= '0;
      complete_id_q <= '0; msi_set_q <= '0;
      outstanding_q <= 1'b0;
      rsp_data_q    <= '0; rsp_opcode_q <= tlul_pkg::AccessAck;
      rsp_source_q  <= '0; rsp_error_q  <= 1'b0;
    end else begin
      claim_re_q    <= '0;
      complete_we_q <= '0;
      msi_set_q     <= '0;

      if (req_fire) begin
        rsp_opcode_q  <= req_is_read ? tlul_pkg::AccessAckData : tlul_pkg::AccessAck;
        rsp_source_q  <= tl_i.a_source;
        rsp_error_q   <= req_malformed | addr_error_d;
        rsp_data_q    <= rsp_data_d;
        outstanding_q <= 1'b1;

        if (!(req_malformed | addr_error_d)) begin
          if (wr_prio_d)
            prio_q[prio_idx_d] <= tl_i.a_data[PRIOW-1:0];

          if (wr_ie_d) begin
            for (int b = 0; b < DW; b++)
              if (((int'(ie_word_d) * DW) + b) < N_SOURCE)
                ie_q[int'(ctx_idx_d)][(int'(ie_word_d)*DW)+b] <= tl_i.a_data[b];
          end

          if (wr_threshold_d)
            threshold_q[ctx_idx_d] <= tl_i.a_data[PRIOW-1:0];

          if (rd_cc_d)  claim_re_q[ctx_idx_d]   <= 1'b1;
          if (wr_cc_d) begin
            complete_we_q[ctx_idx_d]  <= 1'b1;
            complete_id_q[ctx_idx_d]  <= tl_i.a_data[SRCW-1:0];
          end

          if (wr_le_d)
            for (int b = 0; b < DW; b++)
              if (b < N_SOURCE) le_q[b] <= tl_i.a_data[b];

          if (wr_msi_d && (tl_i.a_data[SRCW-1:0] != '0) &&
              (int'(tl_i.a_data[SRCW-1:0]) <= N_SOURCE))
            msi_set_q[tl_i.a_data[SRCW-1:0]-1] <= 1'b1;
        end

      end else if (outstanding_q && tl_i.d_ready) begin
        outstanding_q <= 1'b0;
      end
    end
  end

  assign prio_o        = prio_q;
  assign le_o          = le_q;
  assign ie_o          = ie_q;
  assign threshold_o   = threshold_q;
  assign claim_re_o    = claim_re_q;
  assign complete_we_o = complete_we_q;
  assign complete_id_o = complete_id_q;
  assign msi_set_o     = msi_set_q;

  assign tl_o.a_ready  = ~outstanding_q;
  assign tl_o.d_valid  = outstanding_q;
  assign tl_o.d_opcode = rsp_opcode_q;
  assign tl_o.d_param  = '0;
  assign tl_o.d_size   = FSZ;
  assign tl_o.d_source = rsp_source_q;
  assign tl_o.d_sink   = '0;
  assign tl_o.d_data   = rsp_data_q;
  assign tl_o.d_user   = '0;
  assign tl_o.d_error  = rsp_error_q;

endmodule