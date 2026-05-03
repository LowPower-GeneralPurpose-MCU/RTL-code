`timescale 1ns / 1ps

module axi_clint #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
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

    // Output Interrupts to CPU
    output wire                   timer_irq,
    output wire                   software_irq
);

    // Memory Map CLINT chuẩn RISC-V:
    // 0x0000: msip
    // 0x4000: mtimecmp (thấp)
    // 0x4004: mtimecmp (cao)
    // 0xBFF8: mtime (thấp)
    // 0xBFFC: mtime (cao)

    reg [31:0] msip;
    reg [63:0] mtimecmp;
    reg [63:0] mtime;

    assign software_irq = msip[0];
    assign timer_irq    = (mtime >= mtimecmp);

    // Tăng giá trị mtime mỗi chu kỳ clock (hoặc có thể chia tần số tùy ứng dụng)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mtime <= 64'd0;
        end else begin
            mtime <= mtime + 1'b1;
        end
    end

    // --- AXI FSM: Read/Write Logic ---
    reg [ADDR_WIDTH-1:0] awaddr_reg;
    reg [ADDR_WIDTH-1:0] araddr_reg;

    // Write Address
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
        end else if (s_axi_awvalid && !s_axi_awready) begin
            s_axi_awready <= 1'b1;
            awaddr_reg    <= s_axi_awaddr;
        end else begin
            s_axi_awready <= 1'b0;
        end
    end

    // Write Data & Response
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
            msip         <= 32'd0;
            mtimecmp     <= 64'hFFFF_FFFF_FFFF_FFFF; // Giá trị khởi tạo chuẩn
        end else begin
            if (s_axi_wvalid && !s_axi_wready && s_axi_awready) begin
                s_axi_wready <= 1'b1;
                
                // Giải mã địa chỉ ghi
                case (awaddr_reg[15:0])
                    16'h0000: if (s_axi_wstrb[0]) msip[0] <= s_axi_wdata[0];
                    16'h4000: begin
                        if (s_axi_wstrb[0]) mtimecmp[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) mtimecmp[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) mtimecmp[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) mtimecmp[31:24] <= s_axi_wdata[31:24];
                    end
                    16'h4004: begin
                        if (s_axi_wstrb[0]) mtimecmp[39:32] <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) mtimecmp[47:40] <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) mtimecmp[55:48] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) mtimecmp[63:56] <= s_axi_wdata[31:24];
                    end
                    // Chú ý: Việc ghi vào mtime thường không được khuyến khích, nhưng vẫn cho phép ở chế độ debug
                endcase
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

    // Read Address
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
        end else if (s_axi_arvalid && !s_axi_arready) begin
            s_axi_arready <= 1'b1;
            araddr_reg    <= s_axi_araddr;
        end else begin
            s_axi_arready <= 1'b0;
        end
    end

    // Read Data
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'd0;
            s_axi_rresp  <= 2'b00;
        end else begin
            if (s_axi_arready && s_axi_arvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
                
                // Giải mã địa chỉ đọc
                case (araddr_reg[15:0])
                    16'h0000: s_axi_rdata <= msip;
                    16'h4000: s_axi_rdata <= mtimecmp[31:0];
                    16'h4004: s_axi_rdata <= mtimecmp[63:32];
                    16'hBFF8: s_axi_rdata <= mtime[31:0];
                    16'hBFFC: s_axi_rdata <= mtime[63:32];
                    default:  s_axi_rdata <= 32'h0;
                endcase
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
endmodule