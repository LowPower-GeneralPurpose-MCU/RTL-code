`timescale 1ns / 1ps

module apb_watchdog #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    input  wire                   pclk,
    input  wire                   presetn,
    input  wire [ADDR_WIDTH-1:0]  paddr,
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [DATA_WIDTH-1:0]  pwdata,
    input  wire [3:0]             pstrb,
    output reg                    pready,
    output reg  [DATA_WIDTH-1:0]  prdata,
    output reg                    pslverr,

    output wire                   wdt_irq,
    output wire                   wdt_rst
);

    // Registers
    reg [31:0] wdt_val;     // Đếm tăng dần
    reg [31:0] wdt_cmp;     // Ngưỡng báo ngắt
    reg [31:0] wdt_rst_cmp; // Ngưỡng báo Reset
    reg wdt_en;

    assign wdt_irq = wdt_en && (wdt_val >= wdt_cmp);
    assign wdt_rst = wdt_en && (wdt_val >= wdt_rst_cmp);

    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            wdt_val     <= 32'd0;
            wdt_cmp     <= 32'hFFFF_FFFF;
            wdt_rst_cmp <= 32'hFFFF_FFFF;
            wdt_en      <= 1'b0;
            pready      <= 1'b0;
            pslverr     <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;
            
            // Bộ đếm hoạt động độc lập
            if (wdt_en) wdt_val <= wdt_val + 1'b1;

            if (apb_write) begin
                case (paddr[7:0])
                    8'h00: wdt_en <= pwdata[0];
                    8'h04: wdt_cmp <= pwdata;
                    8'h08: wdt_rst_cmp <= pwdata;
                    8'h0C: if (pwdata == 32'hFEED_D0G1) wdt_val <= 32'd0; // Khóa Feed (Clear)
                    default: pslverr <= 1'b1;
                endcase
            end

            if (apb_read) begin
                case (paddr[7:0])
                    8'h00: prdata <= {31'd0, wdt_en};
                    8'h04: prdata <= wdt_cmp;
                    8'h08: prdata <= wdt_rst_cmp;
                    8'h0C: prdata <= wdt_val;
                    default: begin prdata <= 32'b0; pslverr <= 1'b1; end
                endcase
            end
        end
    end
endmodule