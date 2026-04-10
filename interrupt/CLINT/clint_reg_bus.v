`timescale 1ns / 1ps
`include "clint_defines.vh"

// ============================================================
//  clint_axi4_slave.v  —  AXI4 Full Slave controller
//
//  ═
//  THIẾT KẾ HIỆU SUẤT CAO:
//
//  Write path (zero-bubble):
//     AW pre-buffer 1-deep: awready = 1 KHI pre-buffer rỗng
//      -> luôn có thể nhận AW kế tiếp dù đang xử lý transaction
//     AW pre-buffer -> active regs khi IDLE (0-cycle transition)
//     wready = 1 khi state == WR_BURST -> không bao giờ stall W
//     B response: 
//      -> transaction hoàn thành trong 1 cycle thêm sau WLAST
//     Back-to-back: AW[1] lưu trong pre-buffer trong khi
//      AW[0] đang xử lý -> 0 dead cycle giữa transactions
//
//  Read path (pipelined):
//     arready = 1 khi IDLE -> nhận AR ngay lập tức
//     Cycle N:   AR nhận, địa chỉ latch, gửi req đến regfile
//     Cycle N+1: regfile comb read xong, latch vào r_pipe
//                rvalid = 1, rdata = r_pipe
//     Burst: địa chỉ tăng tự động, 1 beat/cycle với backpressure
//     RLAST đúng vị trí beat cuối
//
//  Error handling:
//     Địa chỉ OOB (ngoài 0x0000 – 0xBFFF) -> SLVERR
//     Địa chỉ không word-aligned (bit[1:0] != 0) -> SLVERR
//     AWSIZE/ARSIZE != 3'b010 (4 bytes) -> SLVERR
//     WRAP burst -> SLVERR (không hỗ trợ, không cần cho reg map)
//     Hart index OOB -> SLVERR (phát hiện bởi regfile)
//     Error tích lũy trong burst -> SLVERR trên B/R response
//
//  ID routing:
//     AWID latch -> BID (khớp chính xác)
//     ARID latch -> RID (giữ suốt burst)
//
//  Giới hạn:
//     1 outstanding write transaction
//     1 outstanding read transaction
//     Data width cố định 32-bit (CLINT spec)
//  ═
// ============================================================

module clint_axi4_slave #(
    parameter ADDR_W   = 26,
    parameter ID_W     = 4
) (
    input  wire clk_i,
    input  wire rst_ni,

    // 
    //  AXI4 Full Slave Interface
    // 

    // Write Address (AW)
    input  wire  [ID_W-1:0]   s_axi_awid,
    input  wire  [ADDR_W-1:0] s_axi_awaddr,
    input  wire  [7:0]        s_axi_awlen,
    input  wire  [2:0]        s_axi_awsize,
    input  wire  [1:0]        s_axi_awburst,
    input  wire               s_axi_awlock,
    input  wire  [3:0]        s_axi_awcache,
    input  wire  [2:0]        s_axi_awprot,
    input  wire  [3:0]        s_axi_awqos,
    input  wire  [3:0]        s_axi_awregion,
    input  wire               s_axi_awvalid,
    output wire               s_axi_awready,

    // Write Data (W)
    input  wire  [31:0]       s_axi_wdata,
    input  wire   [3:0]       s_axi_wstrb,
    input  wire               s_axi_wlast,
    input  wire               s_axi_wvalid,
    output wire               s_axi_wready,

    // Write Response (B)
    output reg   [ID_W-1:0]  s_axi_bid,
    output reg   [1:0]       s_axi_bresp,
    output reg               s_axi_bvalid,
    input  wire              s_axi_bready,

    // Read Address (AR)
    input  wire  [ID_W-1:0]   s_axi_arid,
    input  wire  [ADDR_W-1:0] s_axi_araddr,
    input  wire  [7:0]        s_axi_arlen,
    input  wire  [2:0]        s_axi_arsize,
    input  wire  [1:0]        s_axi_arburst,
    input  wire               s_axi_arlock,
    input  wire  [3:0]        s_axi_arcache,
    input  wire  [2:0]        s_axi_arprot,
    input  wire  [3:0]        s_axi_arqos,
    input  wire  [3:0]        s_axi_arregion,
    input  wire               s_axi_arvalid,
    output wire               s_axi_arready,

    // Read Data (R)
    output reg   [ID_W-1:0]  s_axi_rid,
    output reg   [31:0]      s_axi_rdata,
    output reg   [1:0]       s_axi_rresp,
    output reg               s_axi_rlast,
    output reg               s_axi_rvalid,
    input  wire              s_axi_rready,

    //  Internal Register Bus (-> clint_regfile)
    output reg               reg_req_valid_o,
    output reg               reg_req_write_o,
    output reg  [ADDR_W-1:0] reg_req_addr_o,
    output reg  [31:0]       reg_req_wdata_o,
    output reg   [3:0]       reg_req_wstrb_o,
    input  wire [31:0]       reg_rsp_rdata_i,
    input  wire              reg_rsp_error_i
);
    //  WRITE PATH
    //  Write FSM states 
    localparam WR_IDLE  = 2'b00; // chờ AW (từ pre-buffer)
    localparam WR_BURST = 2'b01; // đang nhận W beats
    localparam WR_RESP  = 2'b10; // gửi B response

    reg [1:0] wr_state;
    //  AW pre-buffer 1-deep 
    // Cho phép nhận AW tiếp theo trong khi đang xử lý transaction.
    // awready = 1 khi pre-buffer RỖNG -> không bao giờ stall AW!
    reg               aw_buf_valid;
    reg  [ID_W-1:0]   aw_buf_id;
    reg  [ADDR_W-1:0] aw_buf_addr;
    reg  [7:0]        aw_buf_len;
    reg  [2:0]        aw_buf_size;
    reg  [1:0]        aw_buf_burst;
    reg               aw_buf_err; // lỗi từ kiểm tra AW sớm

    // Kiểm tra AW ngay khi nhận (tổ hợp)
    wire aw_check_err = (s_axi_awburst == `AXI_BURST_WRAP) ||
                        (s_axi_awsize  != 3'b010)          ||
                        (s_axi_awaddr[1:0] != 2'b00);
                        
    // awready = 1 khi buffer rỗng
    assign s_axi_awready = !aw_buf_valid;
    
    // Cờ consume: pop AW từ buffer vào active regs
    wire aw_consume = aw_buf_valid && (wr_state == WR_IDLE);
    
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            aw_buf_valid <= 1'b0;
            aw_buf_id    <= {ID_W{1'b0}};
            aw_buf_addr  <= {ADDR_W{1'b0}};
            aw_buf_len   <= 8'h0;
            aw_buf_size  <= 3'h2;
            aw_buf_burst <= `AXI_BURST_INCR;
            aw_buf_err   <= 1'b0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin
                // Nhận AW vào buffer
                aw_buf_valid <= 1'b1;
                aw_buf_id    <= s_axi_awid;
                aw_buf_addr  <= s_axi_awaddr;
                aw_buf_len   <= s_axi_awlen;
                aw_buf_size  <= s_axi_awsize;
                aw_buf_burst <= s_axi_awburst;
                aw_buf_err   <= aw_check_err;
            end else if (aw_consume) begin
                // Pop buffer khi bắt đầu IDLE -> BURST
                aw_buf_valid <= 1'b0;
            end
        end
    end

    //  Active transaction registers 
    reg  [ID_W-1:0]   wr_id;
    reg  [ADDR_W-1:0] wr_addr;       // địa chỉ beat hiện tại
    reg  [7:0]        wr_len_cnt;    // beats còn lại
    reg  [1:0]        wr_burst;
    reg               wr_err;        // lỗi tích lũy trong transaction

    // wready: chấp nhận W khi đang ở BURST
    assign s_axi_wready = (wr_state == WR_BURST); 
    // W beat handshake
     wire w_fire = s_axi_wvalid && s_axi_wready;
    //  Write FSM 
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            wr_state    <= WR_IDLE;
            wr_id       <= {ID_W{1'b0}};
            wr_addr     <= {ADDR_W{1'b0}};
            wr_len_cnt  <= 8'h0;
            wr_burst    <= `AXI_BURST_INCR;
            wr_err      <= 1'b0;
        end else begin
            case (wr_state)

                //  IDLE: chờ AW từ pre-buffer 
                WR_IDLE: begin
                    if (aw_consume) begin
                        // Nạp active regs từ buffer
                        wr_id      <= aw_buf_id;
                        wr_addr    <= aw_buf_addr;
                        wr_len_cnt <= aw_buf_len;
                        wr_burst   <= aw_buf_burst;
                        wr_err     <= aw_buf_err;
                        wr_state   <= WR_BURST;
                    end
                end
                //  BURST: nhận W beats 
                WR_BURST: begin
                    if (w_fire) begin
                        // Tích lũy lỗi từ regfile (OOB address)
                        if (reg_rsp_error_i) wr_err <= 1'b1;
                        
                        if (s_axi_wlast) begin
                            // Chuyển sang trạng thái gửi B response một cách chuẩn mực
                            wr_state <= WR_RESP;
                        end else begin
                            // Beat giữa burst: tăng địa chỉ (INCR)
                            if (wr_burst == `AXI_BURST_INCR)
                                wr_addr <= wr_addr + 32'h4; // 4 bytes/beat
                            wr_len_cnt <= wr_len_cnt - 8'h1;
                        end
                    end
                end
                //  RESP: chờ master nhận B 
                WR_RESP: begin
                    if (s_axi_bvalid && s_axi_bready) begin
                        wr_state <= WR_IDLE;
                        wr_err   <= 1'b0;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    //  B channel output 
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            s_axi_bvalid <= 1'b0;
            s_axi_bid    <= {ID_W{1'b0}};
            s_axi_bresp  <= `AXI_OKAY;
        end else begin
            // Set bvalid khi WLAST được nhận
            if (w_fire && s_axi_wlast) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bid    <= wr_id;
                s_axi_bresp  <= (wr_err || reg_rsp_error_i) ? `AXI_SLVERR : `AXI_OKAY;
            end else if (s_axi_bvalid && s_axi_bready) begin
                // Xóa bvalid sau khi handshake thành công
                s_axi_bvalid <= 1'b0;
            end
        end
    end
    //  READ PATH
    //  Read FSM states 
    localparam RD_IDLE  = 2'b00; // chờ AR
    localparam RD_FETCH = 2'b01; // 1 cycle: gửi read req đến regfile
    localparam RD_BURST = 2'b10; // stream R beats

    reg [1:0] rd_state;
    //  Active read transaction 
    reg  [ID_W-1:0]   rd_id;
    reg  [ADDR_W-1:0] rd_addr;       // địa chỉ beat hiện tại
    reg  [7:0]        rd_len_cnt;    // beats còn lại cần gửi
    reg  [1:0]        rd_burst;
    reg               rd_err;
    reg               rd_last_pending; // beat cuối đang chờ gửi

    // arready: 1 khi IDLE (chỉ 1 outstanding read)
    assign s_axi_arready = (rd_state == RD_IDLE);
    
    // AR check
    wire ar_check_err = (s_axi_arburst == `AXI_BURST_WRAP) ||
                        (s_axi_arsize  != 3'b010)          ||
                        (s_axi_araddr[1:0] != 2'b00);
                        
    // R pipeline register 
    // Regfile đọc là combinational, kết quả sẵn trong cùng cycle ta gửi req.
    // Ta latch vào r_pipe_* ở cycle tiếp theo.
    reg [31:0] r_pipe_data;
    reg [1:0]  r_pipe_resp;
    reg        r_pipe_last;
    reg        r_pipe_valid;
    
    // Cho phép fetch beat tiếp theo: khi pipeline trống hoặc
    // master đang nhận beat hiện tại (space available)
    wire rd_can_fetch = !r_pipe_valid || (s_axi_rvalid && s_axi_rready);

    //  Read FSM 
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rd_state       <= RD_IDLE;
            rd_id          <= {ID_W{1'b0}};
            rd_addr        <= {ADDR_W{1'b0}};
            rd_len_cnt     <= 8'h0;
            rd_burst       <= `AXI_BURST_INCR;
            rd_err         <= 1'b0;
            rd_last_pending <= 1'b0;
            r_pipe_valid   <= 1'b0;
            r_pipe_data    <= 32'h0;
            r_pipe_resp    <= `AXI_OKAY;
            r_pipe_last    <= 1'b0;
        end else begin

            // Xóa pipeline khi master nhận beat (trước khi set lại)
            if (s_axi_rvalid && s_axi_rready) begin
                r_pipe_valid <= 1'b0;
            end
            case (rd_state)
                //  IDLE: chờ AR 
                RD_IDLE: begin
                    if (s_axi_arvalid && s_axi_arready) begin
                        rd_id          <= s_axi_arid;
                        rd_addr        <= s_axi_araddr;
                        rd_len_cnt     <= s_axi_arlen;
                        rd_burst       <= s_axi_arburst;
                        rd_err         <= ar_check_err;
                        rd_last_pending <= (s_axi_arlen == 8'h0);
                        rd_state       <= RD_FETCH;
                    end
                end
                RD_FETCH: begin
                    if (rd_can_fetch) begin
                        r_pipe_data  <= reg_rsp_rdata_i;
                        r_pipe_resp  <= (rd_err || reg_rsp_error_i) ? `AXI_SLVERR : `AXI_OKAY;
                        r_pipe_last  <= rd_last_pending;
                        r_pipe_valid <= 1'b1;
                        
                        if (rd_last_pending) begin
                            // Beat duy nhất -> về IDLE
                            rd_state <= RD_IDLE;
                            rd_err   <= 1'b0;
                        end else begin
                            // Có nhiều beat -> sang BURST
                            if (rd_burst == `AXI_BURST_INCR)
                                rd_addr <= rd_addr + 32'h4;
                            rd_len_cnt     <= rd_len_cnt - 8'h1;
                            rd_last_pending <= (rd_len_cnt == 8'h1);
                            rd_state       <= RD_BURST;
                        end
                    end
                end
                //  BURST: stream beats còn lại 
                RD_BURST: begin
                    if (rd_can_fetch) begin
                        r_pipe_data  <= reg_rsp_rdata_i;
                        r_pipe_resp  <= (rd_err || reg_rsp_error_i) ? `AXI_SLVERR : `AXI_OKAY;
                        r_pipe_last  <= rd_last_pending;
                        r_pipe_valid <= 1'b1;
                        
                        if (rd_last_pending) begin
                            rd_state <= RD_IDLE;
                            rd_err   <= 1'b0;
                        end else begin
                            if (rd_burst == `AXI_BURST_INCR)
                                rd_addr <= rd_addr + 32'h4;
                            rd_len_cnt     <= rd_len_cnt - 8'h1;
                            rd_last_pending <= (rd_len_cnt == 8'h1);
                        end
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    //  R channel output (từ pipeline register) 
    always @(*) begin
        s_axi_rid    = rd_id;
        s_axi_rdata  = r_pipe_data;
        s_axi_rresp  = r_pipe_resp;
        s_axi_rlast  = r_pipe_last;
        s_axi_rvalid = r_pipe_valid;
    end
    always @(*) begin
        reg_req_valid_o = 1'b0;
        reg_req_write_o = 1'b0;
        reg_req_addr_o  = {ADDR_W{1'b0}};
        reg_req_wdata_o = 32'h0;
        reg_req_wstrb_o = 4'h0;
        
        if (wr_state == WR_BURST && s_axi_wvalid) begin
            //  Write beat -> regfile 
            reg_req_valid_o = 1'b1;
            reg_req_write_o = 1'b1;
            reg_req_addr_o  = wr_addr;
            reg_req_wdata_o = s_axi_wdata;
            reg_req_wstrb_o = s_axi_wstrb;
        end else if ((rd_state == RD_FETCH) ||
                     (rd_state == RD_BURST && rd_can_fetch)) begin
            //  Read request -> regfile 
            reg_req_valid_o = 1'b1;
            reg_req_write_o = 1'b0;
            reg_req_addr_o  = rd_addr;
            reg_req_wdata_o = 32'h0;
            reg_req_wstrb_o = 4'h0;
        end
    end

endmodule

// ============================================================
//  clint_regfile.v  —  Thanh ghi msip[] và mtimecmp[]
//
//  Tối ưu:
//    1. Read path: THUẦN TỔ HỢP (combinational) -> 0 cycle latency
//       trong regfile, pipeline được xử lý ở AXI slave
//    2. Write path: 1 clock cycle (registered)
//    3. Address decode: priority casez thuần tổ hợp
//       -> tool synthesis tối ưu thành tree of muxes
//    4. Flat output: mtimecmp_flat_o = [NUM_HARTS*64-1:0]
//       Verilog-2001 không cho phép unpacked
//       array ở port
//    5. Kiểm tra hart index OOB -> error signal
//    6. msip chỉ bit[0] hợp lệ, bit[31:1] luôn đọc 0
//
//  Read latency: 0 cycles (combinational)
//  Write latency: 1 cycle (registered at posedge)
//
//  Ghi chú về mtime:
//    Ghi mtime: regfile KHÔNG lưu, chỉ route tín hiệu mtime_wr_*
//    ra clint_timer.v để timer xử lý.
//    Đọc mtime: lấy trực tiếp từ timer outputs (mtime_lo_i, mtime_hi_snap_i).
// ============================================================

module clint_regfile #(
    parameter NUM_HARTS   = 1,
    parameter HART_IDX_W  = 1,   // = max(1, $clog2(NUM_HARTS)), truyền từ top
    parameter ADDR_W      = 26
) (
    input  wire              clk_i,
    input  wire              rst_ni,

    //  Internal register bus (từ AXI4 slave) 
    // Request
    input  wire              req_valid_i,
    input  wire              req_write_i,    // 1=write, 0=read
    input  wire [ADDR_W-1:0] req_addr_i,
    input  wire       [31:0] req_wdata_i,
    input  wire        [3:0] req_wstrb_i,
    // Response (combinational, valid ngay khi req_valid_i)
    output reg        [31:0] rsp_rdata_o,
    output reg               rsp_error_o,

    //  Timer interface 
    input  wire [31:0] mtime_lo_i,         // live mtime lo
    input  wire [31:0] mtime_hi_snap_i,    // snapshot mtime hi
    // Route mtime write đến timer
    output reg         mtime_wr_valid_o,
    output reg         mtime_wr_hi_o,
    output reg  [31:0] mtime_wr_data_o,
    output reg   [3:0] mtime_wr_strb_o,
    // Snapshot trigger
    output wire        snapshot_latch_o,   // 1 khi đọc mtime_lo

    //  Register outputs (-> clint_irq_gen) 
    output reg  [NUM_HARTS-1:0]     msip_o,
    output reg  [NUM_HARTS*64-1:0]  mtimecmp_flat_o  // flattened
);
    //  Register storage
    reg              msip_r     [0:NUM_HARTS-1]; // chỉ 1 bit/hart
    reg [63:0]       mtimecmp_r [0:NUM_HARTS-1];

    //  Address decode (combinational)
    //  Phân vùng:
    //    in_msip      : addr trong [0x0000, 0x4000)
    //    in_mtimecmp  : addr trong [0x4000, 0xBFF8)
    //    in_mtime_lo  : addr == 0xBFF8
    //    in_mtime_hi  : addr == 0xBFFC
    //    addr_oob     : ngoài map -> error
    // Các cờ vùng (wire, tổ hợp)
    wire in_msip     = (req_addr_i < `CLINT_MTIMECMP_BASE);
    wire in_mtimecmp = (req_addr_i >= `CLINT_MTIMECMP_BASE) &&
                       (req_addr_i < `CLINT_MTIME_LO);
    wire in_mtime_lo = (req_addr_i == `CLINT_MTIME_LO);
    wire in_mtime_hi = (req_addr_i == `CLINT_MTIME_HI);
    wire in_any      = in_msip | in_mtimecmp | in_mtime_lo | in_mtime_hi;

    // Hart index tính từ địa chỉ
    // msip:     word index = addr[25:2], lấy HART_IDX_W bit thấp
    // mtimecmp: dword index = (addr - 0x4000) >> 3
    wire [HART_IDX_W-1:0] msip_hart_idx;
    wire [HART_IDX_W-1:0] mtcmp_hart_idx;
    wire                  mtcmp_hi_sel; // 0=lo word, 1=hi word

    assign msip_hart_idx  = req_addr_i[2 + HART_IDX_W - 1 : 2];
    assign mtcmp_hi_sel   = req_addr_i[2];   // bit 2 phân biệt lo/hi
    
    // (addr - 0x4000) >> 3 -> lấy HART_IDX_W bit bắt đầu từ bit 3
    wire [ADDR_W-1:0] mtcmp_offset = req_addr_i - `CLINT_MTIMECMP_BASE;
    assign mtcmp_hart_idx = mtcmp_offset[3 + HART_IDX_W - 1 : 3];

    // Hart index OOB check (chuyển sang integer để compare)
    wire msip_hart_ok  = ({1'b0, msip_hart_idx}  < NUM_HARTS);
    wire mtcmp_hart_ok = ({1'b0, mtcmp_hart_idx} < NUM_HARTS);

    // Final address error
    wire addr_error = req_valid_i && (
        !in_any ||
        (in_msip     && !msip_hart_ok)  ||
        (in_mtimecmp && !mtcmp_hart_ok)
    );

    // Snapshot trigger: pulse khi đọc mtime_lo
    assign snapshot_latch_o = req_valid_i && !req_write_i && in_mtime_lo;

    // Read mux (priority casez style trong always @(*))
    always @(*) begin
        rsp_rdata_o = 32'h0; // default
        rsp_error_o = addr_error;

        if (req_valid_i && !req_write_i) begin
            if (in_mtime_lo) begin
                rsp_rdata_o = mtime_lo_i;
            end else if (in_mtime_hi) begin
                // Trả shadow register (nhất quán với lo vừa đọc)
                rsp_rdata_o = mtime_hi_snap_i;
            end else if (in_msip && msip_hart_ok) begin
                // Chỉ bit[0] hợp lệ, bit[31:1] luôn = 0
                rsp_rdata_o = {31'h0, msip_r[msip_hart_idx]};
            end else if (in_mtimecmp && mtcmp_hart_ok) begin
                rsp_rdata_o = mtcmp_hi_sel
                    ? mtimecmp_r[mtcmp_hart_idx][63:32]
                    : mtimecmp_r[mtcmp_hart_idx][31:0];
            end
            // in_any nhưng OOB -> rdata=0, error=1 (đã set ở trên)
        end
    end

    integer wi; // loop variable cho reset
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (wi = 0; wi < NUM_HARTS; wi = wi + 1) begin
                msip_r[wi]     <= 1'b0;
                mtimecmp_r[wi] <= `CLINT_MTIMECMP_RST;
            end
        end else if (req_valid_i && req_write_i) begin
            if (in_msip && msip_hart_ok) begin
                // Ghi bit[0]. Bit[31:1] bị bỏ qua hoàn toàn.
                if (req_wstrb_i[0])
                    msip_r[msip_hart_idx] <= req_wdata_i[0];
            end else if (in_mtimecmp && mtcmp_hart_ok) begin
                if (!mtcmp_hi_sel) begin
                    // Ghi mtimecmp LO với byte strobe
                    if (req_wstrb_i[0])
                        mtimecmp_r[mtcmp_hart_idx][7:0]   <= req_wdata_i[7:0];
                    if (req_wstrb_i[1])
                        mtimecmp_r[mtcmp_hart_idx][15:8]  <= req_wdata_i[15:8];
                    if (req_wstrb_i[2])
                        mtimecmp_r[mtcmp_hart_idx][23:16] <= req_wdata_i[23:16];
                    if (req_wstrb_i[3])
                        mtimecmp_r[mtcmp_hart_idx][31:24] <= req_wdata_i[31:24];
                end else begin
                    // Ghi mtimecmp HI với byte strobe
                    if (req_wstrb_i[0])
                        mtimecmp_r[mtcmp_hart_idx][39:32] <= req_wdata_i[7:0];
                    if (req_wstrb_i[1])
                        mtimecmp_r[mtcmp_hart_idx][47:40] <= req_wdata_i[15:8];
                    if (req_wstrb_i[2])
                        mtimecmp_r[mtcmp_hart_idx][55:48] <= req_wdata_i[23:16];
                    if (req_wstrb_i[3])
                        mtimecmp_r[mtcmp_hart_idx][63:56] <= req_wdata_i[31:24];
                end
                // mtime write: route ra ngoài (xử lý bằng always @ dưới)
            end
        end
    end

    always @(*) begin
        mtime_wr_valid_o = req_valid_i && req_write_i &&
                           (in_mtime_lo || in_mtime_hi);
        mtime_wr_hi_o    = in_mtime_hi;
        mtime_wr_data_o  = req_wdata_i;
        mtime_wr_strb_o  = req_wstrb_i;
    end

    //  Flatten outputs (-> irq_gen)
    //  nên flatten thành bus phẳng.
    //  irq_gen sẽ dùng [i*64+:64].
    integer gi;
    always @(*) begin
        for (gi = 0; gi < NUM_HARTS; gi = gi + 1) begin
            msip_o[gi]                   = msip_r[gi];
            mtimecmp_flat_o[gi*64 +: 64] = mtimecmp_r[gi];
        end
    end

endmodule