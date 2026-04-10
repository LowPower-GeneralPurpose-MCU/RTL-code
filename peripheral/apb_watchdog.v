`timescale 1ns / 1ps

module apb_watchdog #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    // Miền APB
    input  wire pclk, presetn, psel, penable, pwrite,
    input  wire [ADDR_WIDTH-1:0] paddr,
    input  wire [DATA_WIDTH-1:0] pwdata,
    input  wire [3:0] pstrb,
    output reg  [DATA_WIDTH-1:0] prdata,
    output reg  pready, pslverr,

    // Miền Thời gian thực (RTC)
    input  wire rtc_clk,
    input  wire rtc_rst_n,
    output wire wdt_irq,
    output wire wdt_rst
);
    reg [31:0] wdt_load;
    reg wdt_en, wdt_ie, wdt_re;
    reg wdt_feed_pclk;

    // Truyền lệnh Feed chó từ APB sang RTC
    wire feed_rtc;
    cdc_pulse u_feed_sync (
        .clk_src(pclk), .rst_src_n(presetn), .pulse_src(wdt_feed_pclk),
        .clk_dst(rtc_clk), .rst_dst_n(rtc_rst_n), .pulse_dst(feed_rtc)
    );

    // Đồng bộ cấu hình sang miền RTC
    reg [31:0] wdt_load_rtc;
    reg wdt_en_rtc, wdt_ie_rtc, wdt_re_rtc;
    always @(posedge rtc_clk or negedge rtc_rst_n) begin
        if (!rtc_rst_n) begin
            wdt_load_rtc <= 32'b0; wdt_en_rtc <= 1'b0; 
            wdt_ie_rtc <= 1'b0; wdt_re_rtc <= 1'b0;
        end else begin
            wdt_load_rtc <= wdt_load; wdt_en_rtc <= wdt_en; 
            wdt_ie_rtc <= wdt_ie; wdt_re_rtc <= wdt_re;
        end
    end

    // Bộ đếm đếm theo rtc_clk (Luôn luôn sống kể cả khi tắt pclk)
    reg [31:0] counter;
    reg irq_out_rtc, rst_out_rtc;
    always @(posedge rtc_clk or negedge rtc_rst_n) begin
        if (!rtc_rst_n) begin
            counter <= 32'b0; irq_out_rtc <= 1'b0; rst_out_rtc <= 1'b0;
        end else begin
            if (feed_rtc) begin
                counter <= wdt_load_rtc;
                irq_out_rtc <= 1'b0;
                rst_out_rtc <= 1'b0;
            end else if (wdt_en_rtc && counter > 0) begin
                counter <= counter - 1;
            end

            if (wdt_en_rtc && counter == 0) begin
                if (wdt_ie_rtc) irq_out_rtc <= 1'b1;
                if (wdt_re_rtc) rst_out_rtc <= 1'b1;
            end
        end
    end

    // Gửi tín hiệu Ngắt về miền APB
    cdc_sync_bit u_irq_sync (.clk_dst(pclk), .rst_dst_n(presetn), .d_in(irq_out_rtc), .q_out(wdt_irq));
    assign wdt_rst = rst_out_rtc; 

    // Logic thanh ghi APB
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            wdt_load <= 32'hFFFF_FFFF; 
            wdt_en <= 1'b0; wdt_ie <= 1'b0; wdt_re <= 1'b0; wdt_feed_pclk <= 1'b0;
            pready <= 1'b0; prdata <= 32'b0; pslverr <= 1'b0;
        end else begin
            pready <= psel && penable; 
            pslverr <= 1'b0; 
            wdt_feed_pclk <= 1'b0;
            
            if (psel && penable && pwrite) begin
                case (paddr[11:0])
                    12'h000: wdt_load <= pwdata;
                    12'h004: begin wdt_en <= pwdata[0]; wdt_ie <= pwdata[1]; wdt_re <= pwdata[2]; end
                    12'h008: if (pwdata == 32'h5A5A5A5A) wdt_feed_pclk <= 1'b1;
                    default: pslverr <= 1'b1;
                endcase
            end
            
            if (psel && !penable && !pwrite) begin
                case (paddr[11:0])
                    12'h000: prdata <= wdt_load;
                    12'h004: prdata <= {29'b0, wdt_re, wdt_ie, wdt_en};
                    default: prdata <= 32'b0;
                endcase
            end
        end
    end
endmodule