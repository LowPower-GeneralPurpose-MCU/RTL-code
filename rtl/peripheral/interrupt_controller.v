`timescale 1ns / 1ps

module apb_interrupt_controller #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter NUM_IRQ    = 6
)(
    // --- APB4 Slave Interface ---
    input  wire                   pclk,
    input  wire                   presetn,
    input  wire [ADDR_WIDTH-1:0]  paddr,
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [DATA_WIDTH-1:0]  pwdata,
    input  wire [3:0]             pstrb,
    output wire                   pready,
    output reg  [DATA_WIDTH-1:0]  prdata,
    output wire                   pslverr,

    // --- Interrupt Sources ---
    input  wire                   irq_timer, // ID = 1
    input  wire                   irq_uart,  // ID = 2
    input  wire                   irq_spi,   // ID = 3
    input  wire                   irq_i2c,   // ID = 4
    input  wire                   irq_gpio,  // ID = 5
    input  wire                   irq_accel, // ID = 6

    // --- Signal to CPU ---
    output wire                   cpu_ext_irq
);

    localparam [31:0] VERSION_ID = 32'h0001_0000;

    // 1. Đồng bộ hóa & phát hiện cạnh lên
    wire [NUM_IRQ:1] raw_irq = {irq_accel, irq_gpio, irq_i2c, irq_spi, irq_uart, irq_timer};

    reg [NUM_IRQ:1] irq_sync1, irq_sync2, irq_sync3;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) {irq_sync3, irq_sync2, irq_sync1} <= 0;
        else          {irq_sync3, irq_sync2, irq_sync1} <= {irq_sync2, irq_sync1, raw_irq};
    end
    wire [NUM_IRQ:1] irq_edge = irq_sync2 & ~irq_sync3;

    // 2. Nội tại: thanh ghi, bộ đếm, overflow, priority, threshold
    reg [31:0]       ie_reg;                    // interrupt enable (32-bit, chỉ dùng NUM_IRQ bit thấp)
    reg [3:0]        counters [1:NUM_IRQ];      // bộ đếm pending (0..15)
    reg [NUM_IRQ:1]  overflow_reg;               // cờ tràn
    reg [2:0]        priority [1:NUM_IRQ];       // 3-bit priority
    reg [2:0]        threshold;                   // ngưỡng ưu tiên toàn cục

    // active_irq = có pending và được enable
    wire [NUM_IRQ:1] active_irq;
    genvar g;
    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_ACTIVE
            assign active_irq[g] = (counters[g] > 0) && ie_reg[g];
        end
    endgenerate

    assign cpu_ext_irq = (|active_above_threshold);

    // 3. Tín hiệu APB cơ bản
    wire apb_cycle   = psel && penable;
    wire apb_write   = apb_cycle && pwrite;
    wire apb_read    = apb_cycle && !pwrite;

    // Giải mã địa chỉ (12 bit offset)
    wire addr_ie       = (paddr[11:0] == 12'h000);
    wire addr_claim    = (paddr[11:0] == 12'h004);  // claim khi đọc, complete khi ghi
    wire addr_pending  = (paddr[11:0] == 12'h008);
    wire addr_overflow = (paddr[11:0] == 12'h00C);
    wire addr_version  = (paddr[11:0] == 12'h010);
    wire addr_force    = (paddr[11:0] == 12'h014);  // ghi force
    wire addr_threshold= (paddr[11:0] == 12'h018);
    wire priority_range= (paddr[11:0] >= 12'h100) && (paddr[11:0] < (12'h100 + 4*NUM_IRQ));

    wire valid_addr = addr_ie || addr_claim || addr_pending || addr_overflow ||
                      addr_version || addr_force || addr_threshold || priority_range;

    // 4. pready và pslverr (tổ hợp)
    assign pready  = apb_cycle;
    assign pslverr = apb_cycle && !valid_addr;

    // 5. Các tín hiệu điều khiển nội bộ
    // Complete từ ghi vào addr_claim
    wire is_complete = apb_write && addr_claim;
    wire [2:0] complete_id = pwdata[2:0];

    // Force từ software
    reg        force_wr;
    reg [31:0] force_val;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            force_wr <= 1'b0;
            force_val <= 32'b0;
        end else begin
            force_wr <= apb_write && addr_force;
            if (apb_write && addr_force)
                force_val <= pwdata;
        end
    end
    wire [NUM_IRQ:1] force_set;
    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_FORCE
            assign force_set[g] = force_wr && force_val[g];
        end
    endgenerate

    // Precompute increment/decrement events (tổ hợp)
    wire [NUM_IRQ:1] inc_events;
    wire [NUM_IRQ:1] dec_events;
    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_EVENTS
            assign inc_events[g] = irq_edge[g] || force_set[g];
            assign dec_events[g] = is_complete && (complete_id == g) && (counters[g] > 0);
        end
    endgenerate

    // 6. Priority-based Arbiter (tổ hợp)
    wire [NUM_IRQ:1] active_above_threshold;
    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_ABOVE
            assign active_above_threshold[g] = active_irq[g] && (priority[g] > threshold);
        end
    endgenerate

    reg [31:0] current_claim_id;
    integer j;
    always @(*) begin
        current_claim_id = 32'd0;
        for (j = 1; j <= NUM_IRQ; j = j + 1) begin
            if (active_above_threshold[j]) begin
                if (current_claim_id == 32'd0) begin
                    current_claim_id = j;
                end else begin
                    if (priority[j] > priority[current_claim_id])
                        current_claim_id = j;
                    // nếu bằng nhau, giữ ID nhỏ hơn
                end
            end
        end
    end
    integer i, idx_wr;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            ie_reg <= 32'd0;
            for (i = 1; i <= NUM_IRQ; i = i + 1) begin
                counters[i] <= 4'd0;
                priority[i] <= 3'd0;
            end
            overflow_reg <= 0;
            threshold <= 3'd0;
        end else begin
            // --- Cập nhật counters và overflow ---
            for (i = 1; i <= NUM_IRQ; i = i + 1) begin
                if (inc_events[i] && !dec_events[i]) begin
                    if (counters[i] < 4'd15)
                        counters[i] <= counters[i] + 1;
                    else
                        overflow_reg[i] <= 1'b1;
                end
                else if (!inc_events[i] && dec_events[i]) begin
                    counters[i] <= counters[i] - 1;
                end
                // inc && dec đồng thời: counters[i] giữ nguyên
            end
            // --- Xóa overflow (ghi 1 để xóa) ---
            if (apb_write && addr_overflow) begin
                for (i = 1; i <= NUM_IRQ; i = i + 1) begin
                    if (pstrb[0] && pwdata[i])
                        overflow_reg[i] <= 1'b0;
                end
            end
            // --- Ghi IE (dùng pstrb đầy đủ) ---
            if (apb_write && addr_ie) begin
                if (pstrb[0]) ie_reg[7:0] <= pwdata[7:0];
                if (pstrb[1]) ie_reg[15:8] <= pwdata[15:8];
                if (pstrb[2]) ie_reg[23:16] <= pwdata[23:16];
                if (pstrb[3]) ie_reg[31:24] <= pwdata[31:24];
            end
            // --- Ghi threshold ---
            if (apb_write && addr_threshold) begin
                if (pstrb[0]) threshold <= pwdata[2:0];
            end
            // --- Ghi priority ---
            if (apb_write && priority_range) begin
                idx_wr = (paddr[11:0] - 12'h100) >> 2;
                if (idx_wr >= 0 && idx_wr < NUM_IRQ) begin
                    if (pstrb[0]) priority[idx_wr+1] <= pwdata[2:0];
                end
            end
        end
    end

    // 8. APB đọc thanh ghi
    integer idx_rd;
    wire [NUM_IRQ:1] pending_status;
    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_PENDING
            assign pending_status[g] = (counters[g] > 0);
        end
    endgenerate

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            prdata <= 0;
        end else begin
            prdata <= 32'b0;   // mặc định

            if (apb_read) begin
                if (addr_ie) begin
                    prdata <= ie_reg;
                end else if (addr_claim) begin
                    prdata <= current_claim_id;   // claim: trả về ID ưu tiên cao nhất (không thay đổi counters)
                end else if (addr_pending) begin
                    prdata <= { {(32-NUM_IRQ-1){1'b0}}, pending_status, 1'b0 };
                end else if (addr_overflow) begin
                    prdata <= { {(32-NUM_IRQ-1){1'b0}}, overflow_reg, 1'b0 };
                end else if (addr_version) begin
                    prdata <= VERSION_ID;
                end else if (addr_threshold) begin
                    prdata <= {29'd0, threshold};
                end else if (priority_range) begin
                    idx_rd = (paddr[11:0] - 12'h100) >> 2;
                    if (idx_rd >= 0 && idx_rd < NUM_IRQ)
                        prdata <= {29'd0, priority[idx_rd+1]};
                end
                // Các địa chỉ khác trả về 0
            end
        end
    end

endmodule