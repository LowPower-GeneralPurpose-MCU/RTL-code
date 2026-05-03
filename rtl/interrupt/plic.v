`timescale 1ns / 1ps

module axi_plic #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter NUM_IRQ    = 31
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // AXI4 Slave Interface
    input  wire [ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  wire                   s_axi_awvalid,
    output reg                    s_axi_awready,
    input  wire [DATA_WIDTH-1:0]  s_axi_wdata,
    input  wire [3:0]             s_axi_wstrb,
    input  wire                   s_axi_wvalid,
    output reg                    s_axi_wready,
    output reg  [1:0]             s_axi_bresp,
    output reg                    s_axi_bvalid,
    input  wire                   s_axi_bready,
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    output reg  [DATA_WIDTH-1:0]  s_axi_rdata,
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready,

    // Interrupt Sources
    input  wire [NUM_IRQ:1]       irq_sources,
    // External Interrupt to Core
    output wire                   ext_irq
);

    // PLIC Registers
    reg [2:0]  priority [1:NUM_IRQ];  // 3-bit priority (0-7)
    reg [NUM_IRQ:1] pending;
    reg [NUM_IRQ:1] enable;
    reg [2:0]  threshold;             // Mức ưu tiên thấp nhất để được gọi ngắt
    reg [31:0] claim_id;

    wire [NUM_IRQ:1] pending_next = irq_sources | pending;
    
    // Tìm kiếm ngắt có mức độ ưu tiên cao nhất
    integer i;
    reg [2:0] max_priority;
    reg [31:0] best_id;
    
    always @(*) begin
        max_priority = 3'd0;
        best_id = 32'd0;
        for (i = 1; i <= NUM_IRQ; i = i + 1) begin
            if (pending[i] && enable[i] && (priority[i] > max_priority)) begin
                max_priority = priority[i];
                best_id = i;
            end
        end
        claim_id = best_id;
    end

    assign ext_irq = (max_priority > threshold);

    // Bắt tín hiệu Pending
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending <= 0;
        end else begin
            // Xóa Pending khi CPU xác nhận hoàn thành (Complete) hoặc tự động cập nhật
            pending <= pending_next;
        end
    end

    // --- AXI FSM: Read/Write Logic ---
    reg [ADDR_WIDTH-1:0] awaddr_reg;
    reg [ADDR_WIDTH-1:0] araddr_reg;
    wire [15:0] local_awaddr = awaddr_reg[15:0];
    wire [15:0] local_araddr = araddr_reg[15:0];

    // Write Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            enable        <= 0;
            threshold     <= 3'd0;
            for (i = 1; i <= NUM_IRQ; i = i + 1) priority[i] <= 3'd0;
        end else begin
            if (s_axi_awvalid && !s_axi_awready) begin
                s_axi_awready <= 1'b1;
                awaddr_reg    <= s_axi_awaddr;
            end else begin
                s_axi_awready <= 1'b0;
            end

            if (s_axi_wvalid && s_axi_awready && !s_axi_wready) begin
                s_axi_wready <= 1'b1;
                
                // Decode AXI Address (Mô phỏng bộ nhớ PLIC tiêu chuẩn)
                if (local_awaddr >= 16'h0000 && local_awaddr <= 16'h007C) begin // Priorities
                    priority[local_awaddr[6:2]] <= s_axi_wdata[2:0];
                end else if (local_awaddr == 16'h2000) begin // Enable block
                    enable <= s_axi_wdata[NUM_IRQ:1];
                end else if (local_awaddr == 16'h4000) begin // Context 0 Threshold
                    threshold <= s_axi_wdata[2:0];
                end else if (local_awaddr == 16'h4004) begin // Context 0 Claim/Complete
                    pending[s_axi_wdata[4:0]] <= 1'b0; // Clear pending khi ghi Complete ID
                end
            end else begin
                s_axi_wready <= 1'b0;
            end

            if (s_axi_wready && s_axi_wvalid) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // Read Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (s_axi_arvalid && !s_axi_arready) begin
                s_axi_arready <= 1'b1;
                araddr_reg    <= s_axi_araddr;
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_arready && s_axi_arvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
                
                if (local_araddr >= 16'h0000 && local_araddr <= 16'h007C) begin
                    s_axi_rdata <= {29'd0, priority[local_araddr[6:2]]};
                end else if (local_araddr == 16'h1000) begin
                    s_axi_rdata <= { {(32-NUM_IRQ){1'b0}}, pending };
                end else if (local_araddr == 16'h2000) begin
                    s_axi_rdata <= { {(32-NUM_IRQ){1'b0}}, enable };
                end else if (local_araddr == 16'h4000) begin
                    s_axi_rdata <= {29'd0, threshold};
                end else if (local_araddr == 16'h4004) begin
                    s_axi_rdata <= claim_id;
                end else begin
                    s_axi_rdata <= 32'd0;
                end
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
endmodule