`timescale 1ns / 1ps
`include "dma_defines.vh"

// ============================================================
// dma_channel.v — 1 kênh DMA (src -> FIFO -> dst)
// Layer : Reusable IP
// `include: dma_defines.vh
//
// Parameters — MỌI giá trị đều có thể override
//   ADDR_W       : AXI address bits              [default 32]
//   LEN_W        : AXLEN width                   [default 4]
//   ID_W         : AXI ID width                  [default 4]
//   FIFO_DEPTH   : Internal data FIFO depth      [default 16]
//   MAX_BURST    : Byte tối đa mỗi burst         [default 64]
//   BURST_W      : Bits cho burst_max field      [default 7]
//   TOKEN_W      : Bits cho token counter        [default 4]
//   OUT_W        : Bits cho outstanding counter  [default 4]
//   TIMEOUT_W    : Bits cho watchdog counter     [default 10]
//   PERIPH_NUM_W : Bits cho peripheral index     [default 5]
//   LEN_FIELD_W  : Bits cho transfer length reg  [default 16]
//
// Giao diện với dma_engine
//   cfg_*   : cấu hình (ổn định trong suốt transfer)
//   start   : 1-cycle pulse kích hoạt
//   done    : 1-cycle pulse khi hoàn thành
//   active  : 1 khi đang chạy
//   err     : mã lỗi khi done
//
// Giao diện với axi4_master_rd (qua dma_engine mux)
//   rd_cmd_*  : lệnh đọc
//   rd_dat_*  : data nhận về
//   rd_rsp_*  : phản hồi cuối burst
//
// Giao diện với axi4_master_wr
//   wr_cmd_*  : lệnh ghi
//   wr_dat_*  : data gửi đi
//   wr_rsp_*  : phản hồi cuối burst
// ============================================================

module dma_channel #(
    parameter ADDR_W       = 32,
    parameter LEN_W        = 4,
    parameter ID_W         = 4,
    parameter N_CH_W       = 2,    // bits for channel index = clog2(N_CH)
    parameter CH_IDX       = 0,    // this channel's index — embedded in AXI ID top bits
    parameter FIFO_DEPTH   = 16,
    parameter MAX_BURST    = 64,
    parameter BURST_W      = 7,
    parameter TOKEN_W      = 4,
    parameter OUT_W        = 4,
    parameter TIMEOUT_W    = 10,
    parameter PERIPH_NUM_W = 5,
    parameter LEN_FIELD_W  = 16
) (
    input  wire              clk,
    input  wire              rst_n,

    //  Cấu hình (stable khi active=1) 
    input  wire [ADDR_W-1:0]       cfg_src_addr,
    input  wire [ADDR_W-1:0]       cfg_dst_addr,
    input  wire [LEN_FIELD_W-1:0]  cfg_len,          // tổng bytes
    input  wire [BURST_W-1:0]      cfg_burst_max,    // bytes/burst ≤ MAX_BURST
    input  wire [TOKEN_W-1:0]      cfg_tokens,       // max inflight rd bursts
    input  wire [OUT_W-1:0]        cfg_rd_out_max,   // max outstanding rd cmds
    input  wire [OUT_W-1:0]        cfg_wr_out_max,   // max outstanding wr cmds
    input  wire                    cfg_src_incr,     // 1 = increment src addr
    input  wire                    cfg_dst_incr,     // 1 = increment dst addr
    input  wire [PERIPH_NUM_W-1:0] cfg_periph_num,   // 0 = memory (always ready)

    //  Control 
    input  wire              start,
    output wire              done,
    output wire              active,
    output reg  [1:0]        err,

    //  Peripheral trigger 
    input  wire [31:1]       periph_req,
    output reg  [31:1]       periph_clr,

    //   RD command (-> axi4_master_rd) 
    output wire              rd_cmd_valid,
    input  wire              rd_cmd_ready,
    output wire [ADDR_W-1:0] rd_cmd_addr,
    output wire [LEN_W-1:0]  rd_cmd_len,
    output wire [2:0]        rd_cmd_size,
    output wire [ID_W-1:0]   rd_cmd_id,

    //  RD data (<- axi4_master_rd) 
    input  wire                    rd_dat_valid,
    output wire                    rd_dat_ready,
    input  wire [`AXI_DATA_W-1:0]  rd_dat_data,
    input  wire                    rd_dat_last,

    //  RD response 
    input  wire              rd_rsp_valid,
    input  wire [ID_W-1:0]   rd_rsp_id,
    input  wire [1:0]        rd_rsp_err,

    //  WR command (<- axi4_master_wr) ─
    output wire              wr_cmd_valid,
    input  wire              wr_cmd_ready,
    output wire [ADDR_W-1:0] wr_cmd_addr,
    output wire [LEN_W-1:0]  wr_cmd_len,
    output wire [2:0]        wr_cmd_size,
    output wire [ID_W-1:0]   wr_cmd_id,

    //  WR data (<- axi4_master_wr) 
    output wire                    wr_dat_valid,
    input  wire                    wr_dat_ready,
    output wire [`AXI_DATA_W-1:0]  wr_dat_data,

    //  WR response 
    input  wire              wr_rsp_valid,
    input  wire [ID_W-1:0]   wr_rsp_id,
    input  wire [1:0]        wr_rsp_err,

    //  Timeout flags (từ AXI masters) 
    input  wire              timeout_rd,
    input  wire              timeout_wr_aw,
    input  wire              timeout_wr_w
);

   // localparam
   localparam BYTES_PER_BEAT = `AXI_BYTES;     // = 4 cho 32-bit
    function integer clog2_fn;
        input integer v;
        integer i;
        begin
            clog2_fn = 0;
            for (i = v-1; i > 0; i = i >> 1)
                clog2_fn = clog2_fn + 1;
        end
    endfunction
    localparam BEAT_BITS  = clog2_fn(BYTES_PER_BEAT);  // = 2

    // FIFO_CNT_W: phải bằng PTR_W+1 của sync_fifo.
    // sync_fifo.count là [PTR_W:0] = PTR_W+1 bits,
    // với PTR_W = clog2(FIFO_DEPTH).
    // Vì vậy FIFO_CNT_W = clog2_fn(FIFO_DEPTH) + 1.
    localparam FIFO_PTR_W = clog2_fn(FIFO_DEPTH);      // = PTR_W trong sync_fifo
    localparam FIFO_CNT_W = FIFO_PTR_W + 1;            // = width cua count port
    localparam CMP_W = (FIFO_CNT_W > LEN_W) ? FIFO_CNT_W : (LEN_W + 1);
    localparam MAX_AXI_BURST_BYTES = (1 << LEN_W) * BYTES_PER_BEAT;
    localparam MAX_BURST_LIMIT_INT = (MAX_BURST < MAX_AXI_BURST_BYTES) ?
                                     MAX_BURST : MAX_AXI_BURST_BYTES;
    localparam [BURST_W-1:0] MAX_BURST_LIMIT = MAX_BURST_LIMIT_INT;
    localparam [CMP_W-1:0] FIFO_DEPTH_EXT = FIFO_DEPTH;
    localparam [31:0] AXI_4K_BYTES = 32'd4096;

   // State machine
   localparam [2:0]
        ST_IDLE  = 3'd0,
        ST_RUN   = 3'd1,
        ST_DRAIN = 3'd2,   // rd xong, đợi FIFO drain + wr xong
        ST_DONE  = 3'd3;   // 1-cycle done pulse

    reg [2:0] state;

   // Address và remaining counters
   reg [ADDR_W-1:0]      rd_addr, wr_addr;
    reg [LEN_FIELD_W-1:0] rd_remain, wr_remain;

    // Tính burst size thực tế (min của remain và cfg_burst_max)
    // Dùng combo logic để luôn reflect giá trị hiện tại
    wire [BURST_W-1:0] cfg_burst_defaulted =
        (cfg_burst_max == {BURST_W{1'b0}}) ? MAX_BURST_LIMIT : cfg_burst_max;
    wire [BURST_W-1:0] cfg_burst_clamped =
        (cfg_burst_defaulted > MAX_BURST_LIMIT) ? MAX_BURST_LIMIT : cfg_burst_defaulted;
    wire [BURST_W-1:0] burst_cap_bytes =
        {cfg_burst_clamped[BURST_W-1:BEAT_BITS], {BEAT_BITS{1'b0}}};

    wire cfg_len_unaligned = |cfg_len[BEAT_BITS-1:0];
    wire cfg_src_unaligned = |cfg_src_addr[BEAT_BITS-1:0];
    wire cfg_dst_unaligned = |cfg_dst_addr[BEAT_BITS-1:0];
    wire cfg_burst_invalid = (cfg_burst_max != {BURST_W{1'b0}}) &
                             (burst_cap_bytes == {BURST_W{1'b0}});
    wire cfg_bad = (cfg_len != {LEN_FIELD_W{1'b0}}) &
                   (cfg_len_unaligned | cfg_src_unaligned |
                    cfg_dst_unaligned | cfg_burst_invalid);

    wire [31:0] rd_remain_ext = {{(32-LEN_FIELD_W){1'b0}}, rd_remain};
    wire [31:0] wr_remain_ext = {{(32-LEN_FIELD_W){1'b0}}, wr_remain};
    wire [31:0] burst_cap_ext = {{(32-BURST_W){1'b0}}, burst_cap_bytes};
    wire [31:0] rd_4k_room_ext = AXI_4K_BYTES - {20'h0, rd_addr[11:0]};
    wire [31:0] wr_4k_room_ext = AXI_4K_BYTES - {20'h0, wr_addr[11:0]};

    wire [31:0] rd_burst_rem_cap =
        (rd_remain_ext < burst_cap_ext) ? rd_remain_ext : burst_cap_ext;
    wire [31:0] wr_burst_rem_cap =
        (wr_remain_ext < burst_cap_ext) ? wr_remain_ext : burst_cap_ext;
    wire [31:0] rd_burst_4k_cap =
        (rd_4k_room_ext < rd_burst_rem_cap) ? rd_4k_room_ext : rd_burst_rem_cap;
    wire [31:0] wr_burst_4k_cap =
        (wr_4k_room_ext < wr_burst_rem_cap) ? wr_4k_room_ext : wr_burst_rem_cap;

    wire [BURST_W-1:0] rd_burst_bytes = rd_burst_4k_cap[BURST_W-1:0];
    wire [BURST_W-1:0] wr_burst_bytes = wr_burst_4k_cap[BURST_W-1:0];

    // ARLEN = (burst_bytes / BYTES_PER_BEAT) - 1
    // Dùng BEAT_BITS (localparam = clog2(BYTES_PER_BEAT)) thay vì hardcode 2
    wire [LEN_W-1:0] rd_burst_len =
        (rd_burst_bytes[BURST_W-1:BEAT_BITS] == {(BURST_W-BEAT_BITS){1'b0}})
        ? {LEN_W{1'b0}}
        : rd_burst_bytes[BURST_W-1:BEAT_BITS] - 1'b1;

    wire [LEN_W-1:0] wr_burst_len =
        (wr_burst_bytes[BURST_W-1:BEAT_BITS] == {(BURST_W-BEAT_BITS){1'b0}})
        ? {LEN_W{1'b0}}
        : wr_burst_bytes[BURST_W-1:BEAT_BITS] - 1'b1;

   // Outstanding command counters
   reg [OUT_W-1:0] rd_outs, wr_outs;

    wire rd_cmd_fire = rd_cmd_valid & rd_cmd_ready;
    wire wr_cmd_fire = wr_cmd_valid & wr_cmd_ready;

    wire rd_stall = (cfg_rd_out_max != {OUT_W{1'b0}}) &
                    (rd_outs >= cfg_rd_out_max);
    wire wr_stall = (cfg_wr_out_max != {OUT_W{1'b0}}) &
                    (wr_outs >= cfg_wr_out_max);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || (state == ST_IDLE)) begin
            rd_outs <= {OUT_W{1'b0}};
            wr_outs <= {OUT_W{1'b0}};
        end else begin
            case ({rd_cmd_fire, rd_rsp_valid})
                2'b10: rd_outs <= rd_outs + 1'b1;
                2'b01: rd_outs <= (rd_outs == {OUT_W{1'b0}}) ?
                                   {OUT_W{1'b0}} : rd_outs - 1'b1;
                default: rd_outs <= rd_outs;
            endcase

            case ({wr_cmd_fire, wr_rsp_valid})
                2'b10: wr_outs <= wr_outs + 1'b1;
                2'b01: wr_outs <= (wr_outs == {OUT_W{1'b0}}) ?
                                   {OUT_W{1'b0}} : wr_outs - 1'b1;
                default: wr_outs <= wr_outs;
            endcase
        end
    end

    // Token counter — rate-match RD vs WR để tránh FIFO overflow
    // Mỗi token = 1 phép RD burst chưa được WR xử lý
    reg [TOKEN_W-1:0] tokens;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tokens <= {TOKEN_W{1'b0}};
        else if ((state == ST_IDLE) || (state == ST_DONE))
            tokens <= cfg_tokens;
        else if (rd_cmd_fire & ~wr_rsp_valid)
            tokens <= (tokens == {TOKEN_W{1'b0}}) ? {TOKEN_W{1'b0}} : tokens - 1'b1;
        else if (~rd_cmd_fire & wr_rsp_valid)
            tokens <= tokens + 1'b1;
    end

    wire token_ok = (cfg_tokens == {TOKEN_W{1'b0}}) |
                    (tokens != {TOKEN_W{1'b0}}) |
                    (wr_remain == {LEN_FIELD_W{1'b0}});

   // Peripheral ready signal
   wire [31:0] periph_req_ext = {periph_req, 1'b1};
    wire        periph_rdy     = periph_req_ext[cfg_periph_num];

    // periph_clr: pulse 1 cycle sau mỗi rd burst
    // Bit trong periph_clr[31:1] tương ứng periph_num 1..31
    // periph_clr[periph_num-1] = 1 khi clr periph thứ periph_num
    // Khi periph_num=0 (memory) <- không clr gì cả
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            periph_clr <= {31{1'b0}};
        end else begin
            periph_clr <= {31{1'b0}};
            if (rd_cmd_fire && (cfg_periph_num != {PERIPH_NUM_W{1'b0}})) begin
                // Dịch an toàn: cfg_periph_num >= 1 ở đây
                // periph_clr là [31:1], bit i tương ứng periph_num = i
                // periph_clr[periph_num - 1] = 1
                periph_clr <= (31'b1 << (cfg_periph_num - {{(PERIPH_NUM_W-1){1'b0}}, 1'b1}));
            end
        end
    end

   // Internal data FIFO
   wire              fifo_wr_en  = rd_dat_valid & rd_dat_ready;
    wire              fifo_rd_en  = wr_dat_valid & wr_dat_ready;
    wire              fifo_full, fifo_empty;
    wire [`AXI_DATA_W-1:0] fifo_rdata;
    // count port của sync_fifo là [PTR_W:0] = [FIFO_PTR_W:0] = FIFO_CNT_W bits
    wire [FIFO_CNT_W-1:0]  fifo_count;

    assign rd_dat_ready = ~fifo_full;

    sync_ff #(
        .DATA_W    (`AXI_DATA_W),
        .DEPTH     (FIFO_DEPTH),
        .PTR_W     (FIFO_PTR_W),  // phải khớp với clog2(FIFO_DEPTH)
        .AFULL_TH  (4),            // giữ 4 slot dự phòng
        .AEMPTY_TH (1),
        .OUTREG    (0)             // 0-latency cho WR data path
    ) u_data_fifo (
        .clk         (clk),
        .rst_n       (rst_n),
        .wr_en       (fifo_wr_en),
        .wr_data     (rd_dat_data),
        .rd_en       (fifo_rd_en),
        .rd_data     (fifo_rdata),
        .full        (fifo_full),
        .empty       (fifo_empty),
        .almost_full (),
        .almost_empty(),
        .count       (fifo_count)
    );

    wire [CMP_W-1:0] fifo_count_ext = fifo_count;
    wire [CMP_W-1:0] fifo_free_ext = FIFO_DEPTH_EXT - fifo_count_ext;
    wire [CMP_W-1:0] rd_burst_len_ext = rd_burst_len;
    wire [CMP_W-1:0] wr_burst_len_ext = wr_burst_len;
    wire [CMP_W-1:0] rd_beats_needed =
        rd_burst_len_ext + {{(CMP_W-1){1'b0}}, 1'b1};
    wire [CMP_W-1:0] wr_beats_needed =
        wr_burst_len_ext + {{(CMP_W-1){1'b0}}, 1'b1};
    wire fifo_has_room   = (fifo_free_ext >= rd_beats_needed);
    wire fifo_has_enough = (fifo_count_ext >= wr_beats_needed);
    wire rd_rsp_error = rd_rsp_valid & (rd_rsp_err != `AXI_RESP_OKAY);
    wire wr_rsp_error = wr_rsp_valid & (wr_rsp_err != `AXI_RESP_OKAY);
    wire timeout_seen = timeout_rd | timeout_wr_aw | timeout_wr_w;

   // State machine
   always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            rd_addr   <= {ADDR_W{1'b0}};
            wr_addr   <= {ADDR_W{1'b0}};
            rd_remain <= {LEN_FIELD_W{1'b0}};
            wr_remain <= {LEN_FIELD_W{1'b0}};
            err       <= 2'b00;
        end else begin
            case (state)

                ST_IDLE: begin
                    if (start) begin
                        state     <= (cfg_bad || (cfg_len == {LEN_FIELD_W{1'b0}})) ?
                                     ST_DONE : ST_RUN;
                        rd_addr   <= cfg_src_addr;
                        wr_addr   <= cfg_dst_addr;
                        rd_remain <= cfg_len;
                        wr_remain <= cfg_len;
                        err       <= cfg_bad ? `DMA_ERR_DECERR : `DMA_ERR_NONE;
                    end
                end

                ST_RUN: begin
                    // Cập nhật address / remaining
                    if (rd_cmd_fire) begin
                        if (cfg_src_incr)
                            rd_addr <= rd_addr + {{(ADDR_W-BURST_W){1'b0}}, rd_burst_bytes};
                        rd_remain <= rd_remain - {{(LEN_FIELD_W-BURST_W){1'b0}}, rd_burst_bytes};
                    end
                    if (wr_cmd_fire) begin
                        if (cfg_dst_incr)
                            wr_addr <= wr_addr + {{(ADDR_W-BURST_W){1'b0}}, wr_burst_bytes};
                        wr_remain <= wr_remain - {{(LEN_FIELD_W-BURST_W){1'b0}}, wr_burst_bytes};
                    end

                    // Thu thập lỗi
                    if (rd_rsp_error)
                        err <= (rd_rsp_err == `AXI_RESP_SLVERR)
                               ? `DMA_ERR_SLVERR : `DMA_ERR_DECERR;
                    if (wr_rsp_error)
                        err <= (wr_rsp_err == `AXI_RESP_SLVERR)
                               ? `DMA_ERR_SLVERR : `DMA_ERR_DECERR;
                    if (timeout_seen)
                        err <= `DMA_ERR_TIMEOUT;
                    if (timeout_seen)
                        state <= ST_DONE;
                    else if (rd_rsp_error | wr_rsp_error | (err != `DMA_ERR_NONE))
                        state <= ST_DRAIN;
                    else if ((rd_remain == {LEN_FIELD_W{1'b0}} && !rd_cmd_fire) ||
                             (rd_cmd_fire &&
                              (rd_remain == {{(LEN_FIELD_W-BURST_W){1'b0}}, rd_burst_bytes})))
                        state <= ST_DRAIN;
                end

                ST_DRAIN: begin
                    if (wr_cmd_fire) begin
                        if (cfg_dst_incr)
                            wr_addr <= wr_addr + {{(ADDR_W-BURST_W){1'b0}}, wr_burst_bytes};
                        wr_remain <= wr_remain - {{(LEN_FIELD_W-BURST_W){1'b0}}, wr_burst_bytes};
                    end
                    if (rd_rsp_error)
                        err <= (rd_rsp_err == `AXI_RESP_SLVERR)
                               ? `DMA_ERR_SLVERR : `DMA_ERR_DECERR;
                    if (wr_rsp_error)
                        err <= (wr_rsp_err == `AXI_RESP_SLVERR)
                               ? `DMA_ERR_SLVERR : `DMA_ERR_DECERR;
                    if (timeout_seen)
                        err <= `DMA_ERR_TIMEOUT;
                    // Điều kiện hoàn thành bình thường: FIFO empty, WR xong
                    if (fifo_empty &&
                        rd_outs == {OUT_W{1'b0}} &&
                        wr_outs == {OUT_W{1'b0}} &&
                        wr_remain == {LEN_FIELD_W{1'b0}})
                        state <= ST_DONE;
                    if (timeout_seen)
                        state <= ST_DONE;
                end

                ST_DONE: begin
                    state <= ST_IDLE;   // 1-cycle pulse
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

   // RD command generation
   // SEQ_W: bits available for burst sequence within this channel
    // ID = {CH_IDX[N_CH_W-1:0], seq[SEQ_W-1:0]}
    // This ensures IDs from different channels never collide.
    localparam SEQ_W = ID_W - N_CH_W;

    reg [SEQ_W-1:0] rd_id_cnt;   // burst sequence counter (SEQ_W bits only)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || state == ST_IDLE) rd_id_cnt <= {SEQ_W{1'b0}};
        else if (rd_cmd_fire) rd_id_cnt <= rd_id_cnt + 1'b1;
    end

    assign rd_cmd_valid = (state == ST_RUN) &
                          (rd_remain != {LEN_FIELD_W{1'b0}}) &
                          ~cfg_bad      &
                          ~rd_stall     &
                          fifo_has_room &
                          token_ok      &
                          periph_rdy;

    assign rd_cmd_addr  = rd_addr;
    assign rd_cmd_len   = rd_burst_len;
    assign rd_cmd_size  = `AXI_SIZE_4B;
    assign rd_cmd_id    = {CH_IDX[N_CH_W-1:0], rd_id_cnt};   // top bits = channel, low bits = seq

   // WR command generation
    // Chờ FIFO đủ data cho 1 burst trước khi phát AW
   reg [SEQ_W-1:0] wr_id_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || state == ST_IDLE) wr_id_cnt <= {SEQ_W{1'b0}};
        else if (wr_cmd_fire) wr_id_cnt <= wr_id_cnt + 1'b1;
    end

    // Số beats cần thiết cho burst hiện tại
    // wr_burst_len là [LEN_W-1:0], +1 <- tối đa LEN_W bits cần +1 = LEN_W+1 bits
    // fifo_count là [FIFO_CNT_W-1:0]
    // Để so sánh an toàn, mở rộng cả hai về max(FIFO_CNT_W, LEN_W+1) bits
    assign wr_cmd_valid = ((state == ST_RUN) | (state == ST_DRAIN)) &
                          (wr_remain != {LEN_FIELD_W{1'b0}}) &
                          ~cfg_bad &
                          ~wr_stall &
                          fifo_has_enough;

    assign wr_cmd_addr  = wr_addr;
    assign wr_cmd_len   = wr_burst_len;
    assign wr_cmd_size  = `AXI_SIZE_4B;
    assign wr_cmd_id    = {CH_IDX[N_CH_W-1:0], wr_id_cnt};   // top bits = channel, low bits = seq

   // WR data: feed trực tiếp từ FIFO
    //
    // wr_dat_active set khi AW cmd được accepted (wr_cmd_fire),
    // giữ cho đến khi toàn bộ beats đã gửi (track bằng beat counter).
    // wr_dat_valid = wr_dat_active & ~fifo_empty
   reg [LEN_W-1:0] wr_beat_cnt;
    reg             wr_dat_active;

    wire wr_dat_beat = wr_dat_valid & wr_dat_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_dat_active <= 1'b0;
            wr_beat_cnt   <= {LEN_W{1'b0}};
        end else begin
            if (~wr_dat_active & wr_cmd_fire) begin
                // AW accepted <- bắt đầu gửi data
                wr_dat_active <= 1'b1;
                wr_beat_cnt   <= wr_burst_len;
            end else if (wr_dat_active & wr_dat_beat) begin
                if (wr_beat_cnt == {LEN_W{1'b0}}) begin
                    // Beat cuối — pipeline nếu có lệnh tiếp theo cùng cycle
                    if (wr_cmd_fire) begin
                        wr_dat_active <= 1'b1;
                        wr_beat_cnt   <= wr_burst_len;
                    end else begin
                        wr_dat_active <= 1'b0;
                    end
                end else begin
                    wr_beat_cnt <= wr_beat_cnt - 1'b1;
                end
            end
        end
    end

    assign wr_dat_valid = wr_dat_active & ~fifo_empty;
    assign wr_dat_data  = fifo_rdata;

    // Status outputs
    assign active = (state != ST_IDLE);
    assign done   = (state == ST_DONE);

endmodule

// ============================================================
// dma_engine.v — Multi-channel DMA engine
// Layer : Reusable IP (top của thư viện)
// `include: dma_defines.vh
//
// Đây là module bạn instantiate trong project.
// Layer 3 wrapper chỉ map port names và chọn parameters.
//
// Parameters — TẤT CẢ đều override được
//   N_CH         : Số kênh DMA (1..8)            [default 4]
//   N_CH_W       : $clog2(N_CH) — tính tự động   [KHÔNG override]
//   ADDR_W       : AXI địa chỉ                   [default 32]
//   LEN_W        : AXLEN bits                    [default 4]
//   ID_W         : AXI ID width (≥ N_CH_W+1)     [default 4]
//   FIFO_DEPTH   : FIFO depth mỗi kênh            [default 16]
//   MAX_BURST    : Max bytes/burst                [default 64]
//   BURST_W      : Bits field burst_max           [default 7]
//   TOKEN_W      : Token counter width            [default 4]
//   OUT_W        : Outstanding counter width      [default 4]
//   TIMEOUT_W    : Watchdog width                 [default 10]
//   PERIPH_NUM_W : Peripheral index width         [default 5]
//   LEN_FIELD_W  : Transfer length register       [default 16]
//   APB_ADDR_W   : APB paddr width                [default 13]
//
// APB register map
//   paddr[APB_ADDR_W-1 : `DMA_CH_STRIDE] = channel index
//   paddr[`DMA_CH_STRIDE-1 : 0]          = register offset (xem dma_defines.vh)
//   paddr = `REG_GLOBAL_CTRL             = 0xF00 (global control)
//
// AXI interface
//   1 shared AXI master port — round-robin giữa N_CH kênh
// ============================================================

module dma_engine #(
    parameter N_CH         = 4,
    parameter N_CH_W       = 2,    // PHẢI = $clog2(N_CH), tính thủ công vì Verilog-2001
    parameter ADDR_W       = 32,
    parameter LEN_W        = 4,
    parameter ID_W         = 4,
    parameter FIFO_DEPTH   = 16,
    parameter MAX_BURST    = 64,
    parameter BURST_W      = 7,
    parameter TOKEN_W      = 4,
    parameter OUT_W        = 4,
    parameter TIMEOUT_W    = 10,
    parameter PERIPH_NUM_W = 5,
    parameter LEN_FIELD_W  = 16,
    parameter APB_ADDR_W   = 13,
    // Giá trị default cho cfg_tokens và cfg_out_max của các kênh.
    // Phải là hằng số (không thể dùng expression trong Verilog-2001 port connection).
    // DEF_TOKENS ≤ 2^TOKEN_W - 1, DEF_OUTS ≤ 2^OUT_W - 1
    parameter DEF_TOKENS   = 4,    // default outstanding rd burst tokens/kênh
    parameter DEF_OUTS     = 4     // default max outstanding commands/kênh
) (
    input  wire              clk,
    input  wire              rst_n,

    //  APB Slave
    input  wire                    apb_psel,
    input  wire                    apb_penable,
    input  wire                    apb_pwrite,
    input  wire [APB_ADDR_W-1:0]   apb_paddr,
    input  wire [31:0]             apb_pwdata,
    output reg  [31:0]             apb_prdata,
    output wire                    apb_pslverr,
    output reg                     apb_pready,

    //  IRQ (1 bit per channel) 
    output wire [N_CH-1:0]  irq,

    //  Peripheral triggers 
    input  wire [31:1]       periph_req,
    output wire [31:1]       periph_clr,

    //  AXI4 Master — AR channel 
    output wire [ADDR_W-1:0]      ARADDR,
    output wire [LEN_W-1:0]       ARLEN,
    output wire [2:0]             ARSIZE,
    output wire [ID_W-1:0]        ARID,
    output wire                   ARVALID,
    input  wire                   ARREADY,

    //  AXI4 Master — R channel 
    input  wire [`AXI_DATA_W-1:0] RDATA,
    input  wire [ID_W-1:0]        RID,
    input  wire [1:0]             RRESP,
    input  wire                   RLAST,
    input  wire                   RVALID,
    output wire                   RREADY,

    //  AXI4 Master — AW channel 
    output wire [ADDR_W-1:0]      AWADDR,
    output wire [LEN_W-1:0]       AWLEN,
    output wire [2:0]             AWSIZE,
    output wire [ID_W-1:0]        AWID,
    output wire                   AWVALID,
    input  wire                   AWREADY,

    //  AXI4 Master — W channel 
    output wire [`AXI_DATA_W-1:0]  WDATA,
    output wire [`AXI_STRB_W-1:0]  WSTRB,
    output wire                    WLAST,
    output wire                    WVALID,
    input  wire                    WREADY,

    //  AXI4 Master — B channel 
    input  wire [ID_W-1:0]        BID,
    input  wire [1:0]             BRESP,
    input  wire                   BVALID,
    output wire                   BREADY
);

    // localparam & function
    // function phải khai báo TRƯỚC localparam dùng nó (Verilog-2001)
    function integer clog2_fn;
        input integer v;
        integer i;
        begin
            clog2_fn = 0;
            for (i = v-1; i > 0; i = i >> 1)
                clog2_fn = clog2_fn + 1;
        end
    endfunction

    localparam REG_PER_CH = (1 << `DMA_CH_STRIDE); // bytes/channel

    // CTRL register bit positions (dùng localparam thay vì hardcode số)
    // Layout: [0]=start [BURST_W:1]=burst_max [8]=src_incr [9]=dst_incr
    //         [9+PERIPH_NUM_W:10]=periph_num
    localparam CTRL_START_BIT  = 0;
    localparam CTRL_BMAX_LSB   = 1;
    localparam CTRL_BMAX_MSB   = BURST_W;           // = CTRL_BMAX_LSB + BURST_W - 1
    localparam CTRL_SI_BIT     = BURST_W + 1;        // = 8 khi BURST_W=7
    localparam CTRL_DI_BIT     = BURST_W + 2;        // = 9 khi BURST_W=7
    localparam CTRL_PNUM_LSB   = BURST_W + 3;        // = 10 khi BURST_W=7
    localparam CTRL_PNUM_MSB   = BURST_W + 2 + PERIPH_NUM_W; // = 14 khi BURST_W=7, PERIPH_NUM_W=5

    // STATUS register bit positions
    localparam STAT_ACTIVE_BIT = 0;
    localparam STAT_DONE_BIT   = 1;
    localparam STAT_ERR_LSB    = 2;
    localparam STAT_ERR_MSB    = 3;  // 2-bit err code
    // Per-channel registers
    reg [ADDR_W-1:0]      ch_src    [0:N_CH-1];
    reg [ADDR_W-1:0]      ch_dst    [0:N_CH-1];
    reg [LEN_FIELD_W-1:0] ch_len    [0:N_CH-1];
    reg [BURST_W-1:0]     ch_bmax   [0:N_CH-1];
    reg                   ch_si     [0:N_CH-1];   // src_incr
    reg                   ch_di     [0:N_CH-1];   // dst_incr
    reg [PERIPH_NUM_W-1:0]ch_pnum   [0:N_CH-1];   // periph_num
    reg                   ch_start_r[0:N_CH-1];   // 1-cycle pulse
    reg                   ch_int_en [0:N_CH-1];
    reg [1:0]             ch_int_st [0:N_CH-1];   // W1C: [0]=done [1]=err

    wire                  ch_active [0:N_CH-1];
    wire                  ch_done   [0:N_CH-1];
    wire [1:0]            ch_err    [0:N_CH-1];

    // APB decode
    wire apb_wr  = apb_psel & ~apb_penable & apb_pwrite;
    wire apb_rd  = apb_psel & ~apb_penable & ~apb_pwrite;

    // channel index = paddr[APB_ADDR_W-1:`DMA_CH_STRIDE]
    wire [N_CH_W-1:0] ch_idx = apb_paddr[N_CH_W+`DMA_CH_STRIDE-1:`DMA_CH_STRIDE];
    wire [7:0]        reg_off = apb_paddr[7:0];
    wire is_global = (apb_paddr[11:0] == `REG_GLOBAL_CTRL);
    localparam [N_CH_W:0] N_CH_CMP = N_CH;   // N_CH_W+1 bits, holds 0..N_CH safely
    wire is_ch_reg = ~is_global & (ch_idx < N_CH_CMP);
    reg soft_rst_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) soft_rst_r <= 1'b0;
        else if (apb_wr & is_global & apb_pwdata[0]) soft_rst_r <= 1'b1;
        else soft_rst_r <= 1'b0;   // auto-clear after 1 cycle
    end
    // Combine với hard rst_n: nếu một trong hai active → reset kênh
    wire ch_rst_n = rst_n & ~soft_rst_r;
    always @(posedge clk or negedge rst_n) begin
        apb_pready <= !rst_n ? 1'b0 : (apb_psel & apb_penable);
    end
    assign apb_pslverr = 1'b0;

    // APB write
    integer n;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || soft_rst_r) begin
            // Hard reset OR soft reset: clear all channel control registers
            for (n = 0; n < N_CH; n = n + 1) begin
                ch_start_r[n] <= 1'b0;
                ch_int_st[n]  <= 2'b00;
            end
            // On hard reset only: also clear config registers
            if (!rst_n) begin
                for (n = 0; n < N_CH; n = n + 1) begin
                    ch_src[n]    <= {ADDR_W{1'b0}};
                    ch_dst[n]    <= {ADDR_W{1'b0}};
                    ch_len[n]    <= {LEN_FIELD_W{1'b0}};
                    ch_bmax[n]   <= MAX_BURST[BURST_W-1:0];
                    ch_si[n]     <= 1'b1;
                    ch_di[n]     <= 1'b1;
                    ch_pnum[n]   <= {PERIPH_NUM_W{1'b0}};
                    ch_int_en[n] <= 1'b1;
                end
            end
        end else begin
            // Clear start pulses mỗi cycle
            for (n = 0; n < N_CH; n = n + 1)
                ch_start_r[n] <= 1'b0;

            // Cập nhật interrupt status
            for (n = 0; n < N_CH; n = n + 1) begin
                if (ch_done[n] & ch_int_en[n])
                    ch_int_st[n][0] <= 1'b1;
                if (ch_done[n] & (ch_err[n] != `DMA_ERR_NONE) & ch_int_en[n])
                    ch_int_st[n][1] <= 1'b1;
            end

            // APB channel register writes
            if (apb_wr & is_ch_reg) begin
                case (reg_off)
                    `REG_SRC_ADDR: if (!ch_active[ch_idx])
                                       ch_src[ch_idx]  <= apb_pwdata[ADDR_W-1:0];
                    `REG_DST_ADDR: if (!ch_active[ch_idx])
                                       ch_dst[ch_idx]  <= apb_pwdata[ADDR_W-1:0];
                    `REG_LEN:      if (!ch_active[ch_idx])
                                       ch_len[ch_idx]  <= apb_pwdata[LEN_FIELD_W-1:0];
                    `REG_CTRL: begin
                        if (!ch_active[ch_idx]) begin
                            ch_start_r[ch_idx] <= apb_pwdata[CTRL_START_BIT];
                            ch_bmax[ch_idx]    <= apb_pwdata[CTRL_BMAX_MSB:CTRL_BMAX_LSB];
                            ch_si[ch_idx]      <= apb_pwdata[CTRL_SI_BIT];
                            ch_di[ch_idx]      <= apb_pwdata[CTRL_DI_BIT];
                            ch_pnum[ch_idx]    <= apb_pwdata[CTRL_PNUM_MSB:CTRL_PNUM_LSB];
                        end
                    end
                    `REG_INT_EN:   ch_int_en[ch_idx]  <= apb_pwdata[0];
                    `REG_INT_STAT: ch_int_st[ch_idx]  <= ch_int_st[ch_idx] & ~apb_pwdata[1:0];
                    default: ;
                endcase
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apb_prdata <= 32'h0;
        end else if (apb_rd) begin
            if (is_ch_reg) begin
                case (reg_off)
                    `REG_SRC_ADDR: apb_prdata <= {{(32-ADDR_W){1'b0}}, ch_src[ch_idx]};
                    `REG_DST_ADDR: apb_prdata <= {{(32-ADDR_W){1'b0}}, ch_dst[ch_idx]};
                    `REG_LEN:      apb_prdata <= {{(32-LEN_FIELD_W){1'b0}}, ch_len[ch_idx]};
                    `REG_CTRL:     apb_prdata <= {
                                    {(32-PERIPH_NUM_W-CTRL_PNUM_LSB){1'b0}},
                                    ch_pnum[ch_idx],
                                    ch_di[ch_idx],
                                    ch_si[ch_idx],
                                    ch_bmax[ch_idx],
                                    1'b0 };       // [0]=write-only start
                    // STATUS: [0]=active, [1]=done (latched = ch_int_st[0]),
                    //         [3:2]=err
                    // Dùng ch_int_st[n][0] thay vì ch_done (1-cycle pulse)
                    // để STATUS.done=1 bền vững cho đến khi SW W1C.
                    `REG_STATUS:   apb_prdata <= {
                                    {(32-STAT_ERR_MSB-1){1'b0}},
                                    ch_err[ch_idx],
                                    ch_int_st[ch_idx][0],
                                    ch_active[ch_idx] };
                    `REG_INT_EN:   apb_prdata <= {31'h0, ch_int_en[ch_idx]};
                    `REG_INT_STAT: apb_prdata <= {30'h0, ch_int_st[ch_idx]};
                    default:       apb_prdata <= 32'h0;
                endcase
            end else begin
                apb_prdata <= 32'h0;
            end
        end
    end

    // IRQ output
    genvar gi;
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_irq
            assign irq[gi] = |ch_int_st[gi];
        end
    endgenerate

    // RD cmd bus
    wire [N_CH-1:0]         ch_rd_cmd_valid;
    wire [N_CH-1:0]         ch_rd_cmd_ready;
    wire [N_CH*ADDR_W-1:0]  ch_rd_cmd_addr;
    wire [N_CH*LEN_W-1:0]   ch_rd_cmd_len;
    wire [N_CH*3-1:0]       ch_rd_cmd_size;
    wire [N_CH*ID_W-1:0]    ch_rd_cmd_id;

    // RD data bus
    wire [N_CH-1:0]               ch_rd_dat_valid;
    wire [N_CH-1:0]               ch_rd_dat_ready;
    wire [N_CH*`AXI_DATA_W-1:0]   ch_rd_dat_data;
    wire [N_CH-1:0]               ch_rd_dat_last;

    // RD response bus
    wire [N_CH-1:0]         ch_rd_rsp_valid;
    wire [N_CH*ID_W-1:0]    ch_rd_rsp_id;
    wire [N_CH*2-1:0]       ch_rd_rsp_err;

    // WR cmd bus
    wire [N_CH-1:0]         ch_wr_cmd_valid;
    wire [N_CH-1:0]         ch_wr_cmd_ready;
    wire [N_CH*ADDR_W-1:0]  ch_wr_cmd_addr;
    wire [N_CH*LEN_W-1:0]   ch_wr_cmd_len;
    wire [N_CH*3-1:0]       ch_wr_cmd_size;
    wire [N_CH*ID_W-1:0]    ch_wr_cmd_id;

    // WR data bus
    wire [N_CH-1:0]               ch_wr_dat_valid;
    wire [N_CH-1:0]               ch_wr_dat_ready;
    wire [N_CH*`AXI_DATA_W-1:0]   ch_wr_dat_data;

    // WR response bus
    wire [N_CH-1:0]         ch_wr_rsp_valid;
    wire [N_CH*ID_W-1:0]    ch_wr_rsp_id;
    wire [N_CH*2-1:0]       ch_wr_rsp_err;

    // Periph clr per channel
    wire [N_CH*31-1:0]      ch_periph_clr_bus;

    // Timeout per channel
    wire [N_CH-1:0]         ch_timeout_rd;
    wire [N_CH-1:0]         ch_timeout_aw;
    wire [N_CH-1:0]         ch_timeout_w;

    // Channel instances
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_ch
            dma_channel #(
                .ADDR_W       (ADDR_W),
                .LEN_W        (LEN_W),
                .ID_W         (ID_W),
                .N_CH_W       (N_CH_W),   // FIX ID: channel index bits
                .CH_IDX       (gi),        // FIX ID: this channel's index
                .FIFO_DEPTH   (FIFO_DEPTH),
                .MAX_BURST    (MAX_BURST),
                .BURST_W      (BURST_W),
                .TOKEN_W      (TOKEN_W),
                .OUT_W        (OUT_W),
                .TIMEOUT_W    (TIMEOUT_W),
                .PERIPH_NUM_W (PERIPH_NUM_W),
                .LEN_FIELD_W  (LEN_FIELD_W)
            ) u_ch (
                .clk            (clk),
                .rst_n          (ch_rst_n),   // soft reset support
                .cfg_src_addr   (ch_src[gi]),
                .cfg_dst_addr   (ch_dst[gi]),
                .cfg_len        (ch_len[gi]),
                .cfg_burst_max  (ch_bmax[gi]),
                .cfg_tokens     (DEF_TOKENS[TOKEN_W-1:0]),
                .cfg_rd_out_max (DEF_OUTS[OUT_W-1:0]),
                .cfg_wr_out_max (DEF_OUTS[OUT_W-1:0]),
                .cfg_src_incr   (ch_si[gi]),
                .cfg_dst_incr   (ch_di[gi]),
                .cfg_periph_num (ch_pnum[gi]),
                .start          (ch_start_r[gi]),
                .done           (ch_done[gi]),
                .active         (ch_active[gi]),
                .err            (ch_err[gi]),
                .periph_req     (periph_req),
                .periph_clr     (ch_periph_clr_bus[gi*31+30 : gi*31]),
                // RD cmd
                .rd_cmd_valid   (ch_rd_cmd_valid[gi]),
                .rd_cmd_ready   (ch_rd_cmd_ready[gi]),
                .rd_cmd_addr    (ch_rd_cmd_addr[gi*ADDR_W+:ADDR_W]),
                .rd_cmd_len     (ch_rd_cmd_len[gi*LEN_W+:LEN_W]),
                .rd_cmd_size    (ch_rd_cmd_size[gi*3+:3]),
                .rd_cmd_id      (ch_rd_cmd_id[gi*ID_W+:ID_W]),
                // RD data
                .rd_dat_valid   (ch_rd_dat_valid[gi]),
                .rd_dat_ready   (ch_rd_dat_ready[gi]),
                .rd_dat_data    (ch_rd_dat_data[gi*`AXI_DATA_W+:`AXI_DATA_W]),
                .rd_dat_last    (ch_rd_dat_last[gi]),
                // RD rsp
                .rd_rsp_valid   (ch_rd_rsp_valid[gi]),
                .rd_rsp_id      (ch_rd_rsp_id[gi*ID_W+:ID_W]),
                .rd_rsp_err     (ch_rd_rsp_err[gi*2+:2]),
                // WR cmd
                .wr_cmd_valid   (ch_wr_cmd_valid[gi]),
                .wr_cmd_ready   (ch_wr_cmd_ready[gi]),
                .wr_cmd_addr    (ch_wr_cmd_addr[gi*ADDR_W+:ADDR_W]),
                .wr_cmd_len     (ch_wr_cmd_len[gi*LEN_W+:LEN_W]),
                .wr_cmd_size    (ch_wr_cmd_size[gi*3+:3]),
                .wr_cmd_id      (ch_wr_cmd_id[gi*ID_W+:ID_W]),
                // WR data
                .wr_dat_valid   (ch_wr_dat_valid[gi]),
                .wr_dat_ready   (ch_wr_dat_ready[gi]),
                .wr_dat_data    (ch_wr_dat_data[gi*`AXI_DATA_W+:`AXI_DATA_W]),
                // WR rsp
                .wr_rsp_valid   (ch_wr_rsp_valid[gi]),
                .wr_rsp_id      (ch_wr_rsp_id[gi*ID_W+:ID_W]),
                .wr_rsp_err     (ch_wr_rsp_err[gi*2+:2]),
                // Timeouts
                .timeout_rd     (ch_timeout_rd[gi]),
                .timeout_wr_aw  (ch_timeout_aw[gi]),
                .timeout_wr_w   (ch_timeout_w[gi])
            );
        end
    endgenerate

    // periph_clr: OR của tất cả kênh
    // ch_periph_clr_bus là [N_CH*31-1:0], mỗi kênh chiếm 31 bits
    // periph_clr output là [31:1] (31 bits)
    integer pi;
    reg [30:0] pclr_or;   // 31 bits nội bộ (index 0..30 = periph 1..31)
    always @(*) begin
        pclr_or = 31'h0;
        for (pi = 0; pi < N_CH; pi = pi + 1)
            pclr_or = pclr_or | ch_periph_clr_bus[pi*31 +: 31];
    end
    assign periph_clr = pclr_or;

    // Round-robin arbiter — RD commands -> axi4_master_rd
    wire [N_CH-1:0] rd_grant;
    wire rd_cmd_ready_top;
    wire rd_rsp_valid_top;
    wire [ID_W-1:0] rd_rsp_id_top;
    wire [1:0]      rd_rsp_err_top;
    wire            rd_timeout_top;

    arbiter_iwrr_1cycle #(
        .P_REQUESTER_NUM(N_CH),
        .P_REQUESTER_WEIGHT({N_CH{32'd1}})
    ) u_rd_arb (
        .clk             (clk),
        .rst_n           (ch_rst_n),
        .req_i           (ch_rd_cmd_valid),
        .req_weight_i    ({(N_CH*32){1'b0}}), // unused, set to 0 or any value since we're using static weights
        .num_grant_req_i (1'b0),  // unused, set to 0 or any value
        .grant_ready_i   (rd_cmd_ready_top),  
        .grant_valid_o   (rd_grant)
    );

    // Assign rd_cmd_ready dựa vào grant
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_rd_ready
            assign ch_rd_cmd_ready[gi] = rd_grant[gi] & rd_cmd_ready_top;
        end
    endgenerate

    // Mux AR signals từ kênh được grant
    wire [ADDR_W-1:0] mux_rd_addr;
    wire [LEN_W-1:0]  mux_rd_len;
    wire [2:0]        mux_rd_size;
    wire [ID_W-1:0]   mux_rd_id;

    onehot_mux #(.DATA_W(ADDR_W), .N(N_CH)) mux_araddr (
        .din(ch_rd_cmd_addr), .sel(rd_grant), .dout(mux_rd_addr));
    onehot_mux #(.DATA_W(LEN_W),  .N(N_CH)) mux_arlen (
        .din(ch_rd_cmd_len),  .sel(rd_grant), .dout(mux_rd_len));
    onehot_mux #(.DATA_W(3),      .N(N_CH)) mux_arsize (
        .din(ch_rd_cmd_size), .sel(rd_grant), .dout(mux_rd_size));
    onehot_mux #(.DATA_W(ID_W),   .N(N_CH)) mux_arid (
        .din(ch_rd_cmd_id),   .sel(rd_grant), .dout(mux_rd_id));

    wire rd_cmd_valid_top = |(rd_grant & ch_rd_cmd_valid);

    // RD data: broadcast đến kênh phù hợp theo RID[N_CH_W-1:0]
    wire                   rd_dout_valid;
    wire                   rd_dout_ready;
    wire [`AXI_DATA_W-1:0] rd_dout_data;
    wire                   rd_dout_last;
    wire [ID_W-1:0]        rd_dout_id;

    axi4_master_rd #(
        .ADDR_W    (ADDR_W),
        .ID_W      (ID_W),
        .LEN_W     (LEN_W),
        .CMD_DEPTH (N_CH),     // 1 slot/channel
        .TIMEOUT_W (TIMEOUT_W)
    ) u_rd_master (
        .clk         (clk),
        .rst_n       (ch_rst_n),   // soft reset clears cmd_fifo & AXI state
        .cmd_valid   (rd_cmd_valid_top),
        .cmd_ready   (rd_cmd_ready_top),
        .cmd_addr    (mux_rd_addr),
        .cmd_len     (mux_rd_len),
        .cmd_size    (mux_rd_size),
        .cmd_id      (mux_rd_id),
        .dout_valid  (rd_dout_valid),
        .dout_ready  (rd_dout_ready),
        .dout_data   (rd_dout_data),
        .dout_last   (rd_dout_last),
        .dout_id     (rd_dout_id),
        .rsp_valid   (rd_rsp_valid_top),
        .rsp_id      (rd_rsp_id_top),
        .rsp_err     (rd_rsp_err_top),
        .timeout_out (rd_timeout_top),
        .ARADDR      (ARADDR),
        .ARLEN       (ARLEN),
        .ARSIZE      (ARSIZE),
        .ARID        (ARID),
        .ARVALID     (ARVALID),
        .ARREADY     (ARREADY),
        .RDATA       (RDATA),
        .RID         (RID),
        .RRESP       (RRESP),
        .RLAST       (RLAST),
        .RVALID      (RVALID),
        .RREADY      (RREADY)
    );

    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_rd_route
            assign ch_rd_dat_valid[gi] =
                rd_dout_valid & (rd_dout_id[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_rd_dat_data[gi*`AXI_DATA_W+:`AXI_DATA_W] = rd_dout_data;
            assign ch_rd_dat_last[gi]  = rd_dout_last;

            assign ch_rd_rsp_valid[gi] =
                rd_rsp_valid_top & (rd_rsp_id_top[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_rd_rsp_id[gi*ID_W+:ID_W]   = rd_rsp_id_top;
            assign ch_rd_rsp_err[gi*2+:2]         = rd_rsp_err_top;
            assign ch_timeout_rd[gi] = rd_timeout_top &
                (ARVALID ? (ARID[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0])
                        : rd_grant[gi]);
        end
    endgenerate

    // rd_dout_ready: chỉ assert khi kênh ĐÍCH (khớp dout_id) có FIFO ready.

    assign rd_dout_ready = |(ch_rd_dat_valid & ch_rd_dat_ready);
    wire [N_CH-1:0] wr_grant_pulse;  // registered output từ rr_arbiter
    reg  [N_CH-1:0] wr_grant_r;
    reg  [N_CH-1:0] wr_dat_grant_r;
    wire [N_CH-1:0] wr_grant = wr_grant_r;
    wire            wr_cmd_fire_top;
    wire            wr_cmd_ready_top;
    wire            wr_cmd_valid_top;
    wire            wr_cmd_valid_gated;
    wire            wr_wlast_done;
    wire wr_arb_accept = wr_cmd_fire_top | (wr_cmd_ready_top & ~|wr_dat_grant_r & ~wr_cmd_valid_top);
    arbiter_iwrr_1cycle #(
        .P_REQUESTER_NUM(N_CH),
        .P_REQUESTER_WEIGHT({N_CH{32'd1}})
    ) u_wr_arb (
        .clk             (clk),
        .rst_n           (ch_rst_n),
        .req_i           (ch_wr_cmd_valid),
        .req_weight_i    ({(N_CH*32){1'b0}}), // unused, set to 0 or any value since we're using static weights
        .num_grant_req_i (1'b0),  // unused, set to 0 or any value
        .grant_ready_i   (wr_arb_accept),  
        .grant_valid_o   (wr_grant_pulse)
    );

    always @(posedge clk or negedge ch_rst_n) begin
        if (!ch_rst_n) begin
            wr_grant_r <= {N_CH{1'b0}};
        end else begin
            if (wr_cmd_fire_top) begin
                // Handshake xong: advance đến kênh tiếp theo
                wr_grant_r <= wr_grant_pulse;
            end else if (wr_cmd_ready_top & ~|wr_dat_grant_r & ~wr_cmd_valid_top) begin
                wr_grant_r <= wr_grant_pulse;
            end
            // else: giữ nguyên (đang chờ AWREADY, hoặc kênh đang phát W data)
        end
    end

    // wr_burst_gate: cho phép AW mới chỉ khi không có W burst đang chạy,
    // HOẶC đúng cycle WLAST (pipeline). Dùng chung cho CẢ ch_wr_cmd_ready
    // VÀ wr_cmd_valid_gated để dma_channel và u_wr_master luôn đồng thuận.
    wire wr_burst_gate = ~|wr_dat_grant_r | wr_wlast_done;

    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_wr_ready
            assign ch_wr_cmd_ready[gi] = wr_grant[gi] & wr_cmd_ready_top & wr_burst_gate;
        end
    endgenerate

    wire [ADDR_W-1:0] mux_wr_addr;
    wire [LEN_W-1:0]  mux_wr_len;
    wire [2:0]        mux_wr_size;
    wire [ID_W-1:0]   mux_wr_id;

    onehot_mux #(.DATA_W(ADDR_W), .N(N_CH)) mux_awaddr (
        .din(ch_wr_cmd_addr), .sel(wr_grant), .dout(mux_wr_addr));
    onehot_mux #(.DATA_W(LEN_W),  .N(N_CH)) mux_awlen (
        .din(ch_wr_cmd_len),  .sel(wr_grant), .dout(mux_wr_len));
    onehot_mux #(.DATA_W(3),      .N(N_CH)) mux_awsize (
        .din(ch_wr_cmd_size), .sel(wr_grant), .dout(mux_wr_size));
    onehot_mux #(.DATA_W(ID_W),   .N(N_CH)) mux_awid (
        .din(ch_wr_cmd_id),   .sel(wr_grant), .dout(mux_wr_id));
    assign wr_wlast_done = WLAST & WVALID & WREADY;

    always @(posedge clk or negedge ch_rst_n) begin
        if (!ch_rst_n) begin
            wr_dat_grant_r <= {N_CH{1'b0}};
        end else begin
            if (wr_wlast_done & wr_cmd_fire_top) begin
                // Pipeline: burst kết thúc, AW mới accepted cùng cycle
                wr_dat_grant_r <= wr_grant;
            end else if (wr_wlast_done) begin
                wr_dat_grant_r <= {N_CH{1'b0}};
            end else if (~|wr_dat_grant_r & wr_cmd_fire_top) begin
                wr_dat_grant_r <= wr_grant;
            end
            // else: giữ nguyên cho đến WLAST
        end
    end

    // Sel: W phase dùng wr_dat_grant_r; giữa các burst dùng wr_grant
    wire [N_CH-1:0] wr_dat_sel = (|wr_dat_grant_r) ? wr_dat_grant_r : wr_grant;
    // WR data mux: từ kênh đang active trong W phase
    wire [`AXI_DATA_W-1:0] mux_wr_data;
    onehot_mux #(.DATA_W(`AXI_DATA_W), .N(N_CH)) mux_wdata (
        .din(ch_wr_dat_data), .sel(wr_dat_sel), .dout(mux_wr_data));

    // Same fix as rd_cmd_valid_top: gate with actual channel valid.
    assign wr_cmd_valid_top = |(wr_grant & ch_wr_cmd_valid);
    assign wr_cmd_valid_gated = wr_cmd_valid_top & wr_burst_gate;

    // wr_cmd_fire_top: forward-declared above, driven here
    // Dùng wr_cmd_valid_gated thay vì wr_cmd_valid_top
    assign wr_cmd_fire_top = wr_cmd_valid_gated & wr_cmd_ready_top;
    wire wr_din_valid = |(ch_wr_dat_valid & wr_dat_sel);
    wire wr_din_ready;
    wire wr_rsp_valid_top;
    wire [ID_W-1:0] wr_rsp_id_top;
    wire [1:0]      wr_rsp_err_top;
    wire            wr_timeout_aw_top, wr_timeout_w_top;

    // wr_dat_ready: chỉ kênh được sticky grant nhận ready
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_wr_dat_ready
            assign ch_wr_dat_ready[gi] = wr_dat_sel[gi] & wr_din_ready;
        end
    endgenerate

    axi4_master_wr #(
        .ADDR_W    (ADDR_W),
        .ID_W      (ID_W),
        .LEN_W     (LEN_W),
        .CMD_DEPTH (N_CH),
        .TIMEOUT_W (TIMEOUT_W)
    ) u_wr_master (
        .clk         (clk),
        .rst_n       (ch_rst_n),   // soft reset clears internal state
        .cmd_valid   (wr_cmd_valid_gated),   // FIX: dùng gated version
        .cmd_ready   (wr_cmd_ready_top),
        .cmd_addr    (mux_wr_addr),
        .cmd_len     (mux_wr_len),
        .cmd_size    (mux_wr_size),
        .cmd_id      (mux_wr_id),
        .din_valid   (wr_din_valid),
        .din_ready   (wr_din_ready),
        .din_data    (mux_wr_data),
        .rsp_valid   (wr_rsp_valid_top),
        .rsp_id      (wr_rsp_id_top),
        .rsp_err     (wr_rsp_err_top),
        .timeout_aw  (wr_timeout_aw_top),
        .timeout_w   (wr_timeout_w_top),
        .AWADDR      (AWADDR),
        .AWLEN       (AWLEN),
        .AWSIZE      (AWSIZE),
        .AWID        (AWID),
        .AWVALID     (AWVALID),
        .AWREADY     (AWREADY),
        .WDATA       (WDATA),
        .WSTRB       (WSTRB),
        .WLAST       (WLAST),
        .WVALID      (WVALID),
        .WREADY      (WREADY),
        .BID         (BID),
        .BRESP       (BRESP),
        .BVALID      (BVALID),
        .BREADY      (BREADY)
    );

    // Route WR rsp về đúng kênh
    // FIX ID routing: dùng N_CH_W bit CAO của BID
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : gen_wr_rsp_route
            assign ch_wr_rsp_valid[gi] =
                wr_rsp_valid_top & (wr_rsp_id_top[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_wr_rsp_id[gi*ID_W+:ID_W] = wr_rsp_id_top;
            assign ch_wr_rsp_err[gi*2+:2]       = wr_rsp_err_top;
            assign ch_timeout_aw[gi] = wr_timeout_aw_top &
                                    (AWID[ID_W-1:ID_W-N_CH_W] == gi[N_CH_W-1:0]);
            assign ch_timeout_w[gi]  = wr_timeout_w_top & wr_dat_grant_r[gi];
        end
    endgenerate

endmodule
