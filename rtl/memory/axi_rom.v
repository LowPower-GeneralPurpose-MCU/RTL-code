`timescale 1ns / 1ps

module axi_rom #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter BASE_ADDR  = 32'h0000_1000, // Địa chỉ bắt đầu của ROM
    parameter MEM_DEPTH  = 4096           // Số lượng Word (16KB)
)(
    input wire clk,
    input wire rst_n,

    // --- Kênh Ghi (AXI chuẩn yêu cầu nhưng ROM sẽ báo lỗi nếu Master cố ghi) ---
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

    // --- Kênh Đọc (Chức năng chính của ROM) ---
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    output reg  [DATA_WIDTH-1:0]  s_axi_rdata,
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready
);

    // Giải mã địa chỉ Word (Bỏ 2 bit thấp, offset so với BASE_ADDR)
    // Tối ưu ASIC: Dùng phép cắt bit thay cho phép chia
    wire [11:0] read_index = (s_axi_araddr[13:2] - BASE_ADDR[13:2]);

    reg [31:0] rom_data_out;

    // =========================================================================
    // BOOTLOADER LOGIC (MÃ MÁY RISC-V)
    // =========================================================================
    always @(*) begin
        case (read_index)
            // 1. Thiết lập địa chỉ nguồn (Flash: 0x2000_0000) và đích (RAM: 0x8000_0000)
            12'h000: rom_data_out = 32'h200002b7; // lui t0, 0x20000      (Source Flash)
            12'h001: rom_data_out = 32'h80000337; // lui t1, 0x80000      (Dest RAM)
            
            // 2. Thiết lập kích thước copy (Ví dụ: 16KB = 0x4000)
            12'h002: rom_data_out = 32'h000043b7; // lui t2, 0x00004
            12'h003: rom_data_out = 32'h007303b3; // add t2, t1, t2       (End Address)
            
            // 3. Vòng lặp Copy (Loop)
            12'h004: rom_data_out = 32'h0002ae03; // lw  t3, 0(t0)        (Đọc từ Flash)
            12'h005: rom_data_out = 32'h01c32023; // sw  t3, 0(t1)        (Ghi vào RAM)
            12'h006: rom_data_out = 32'h00428293; // addi t0, t0, 4       (Tăng Source)
            12'h007: rom_data_out = 32'h00430313; // addi t1, t1, 4       (Tăng Dest)
            12'h008: rom_data_out = 32'hfe7348e3; // blt  t1, t2, -16     (Lặp lại nếu chưa xong)
            
            // 4. Nhảy vào RAM để thực thi chương trình chính
            12'h009: rom_data_out = 32'h80000337; // lui  t1, 0x80000
            12'h00A: rom_data_out = 32'h00030067; // jalr x0, t1, 0       (Jump to RAM)
            
            // Mặc định trả về lệnh NOP (No Operation)
            default: rom_data_out = 32'h00000013; // addi x0, x0, 0 (NOP)
        endcase
    end

    // =========================================================================
    // GIAO THỨC BẮT TAY AXI4-LITE (HANDSHAKE)
    // =========================================================================
    
    // Logic Kênh Ghi (Báo lỗi nếu Master cố ghi vào ROM)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
        end else begin
            if (s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                s_axi_bvalid  <= 1'b1;
                s_axi_bresp   <= 2'b10; // SLVERR: ROM không cho phép ghi
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                s_axi_bvalid  <= 1'b0;
            end
        end
    end

    // Logic Kênh Đọc
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'h0;
            s_axi_rresp   <= 2'b00;
        end else begin
            // Nhận yêu cầu đọc
            if (!s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
            end 
            // Trả dữ liệu
            else if (s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b0;
                s_axi_rvalid  <= 1'b1;
                s_axi_rdata   <= rom_data_out;
                s_axi_rresp   <= 2'b00; // OKAY
            end 
            // Kết thúc lượt đọc
            else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid  <= 1'b0;
            end
        end
    end

endmodule