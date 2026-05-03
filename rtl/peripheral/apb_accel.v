`timescale 1ns / 1ps

module apb_accelerator #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    input wire pclk, presetn, psel, penable, pwrite,
    input wire [ADDR_WIDTH-1:0] paddr,
    input wire [DATA_WIDTH-1:0] pwdata,
    input wire [3:0] pstrb,
    output reg pready, prdata, pslverr,

    output wire accel_irq
);

    // =========================================
    // 0x00: CTRL    [0] START, [1] IRQ_EN
    // 0x04: STATUS  [0] BUSY, [1] DONE
    // 0x08: OPA     Toán hạng A
    // 0x0C: OPB     Toán hạng B
    // 0x10: RESULT  Kết quả (Đọc)
    // =========================================

    reg [1:0]  reg_ctrl;
    reg [31:0] reg_opa, reg_opb, reg_result;
    reg        busy, done;
    
    assign accel_irq = done & reg_ctrl[1];

    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite;

    // FSM Xử lý thuật toán nội bộ
    reg [7:0] compute_delay;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl <= 0; reg_opa <= 0; reg_opb <= 0; reg_result <= 0;
            busy <= 0; done <= 0; compute_delay <= 0;
            pready <= 0; prdata <= 0; pslverr <= 0;
        end else begin
            pready <= psel && penable;
            pslverr <= 0;

            // Xử lý Ghi APB
            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: begin
                        if (pwdata[0] && !busy) begin
                            busy <= 1'b1;
                            done <= 1'b0;
                            compute_delay <= 8'd10; // Giả lập tốn 10 chu kỳ clock
                        end
                        reg_ctrl[1] <= pwdata[1]; // IRQ Enable
                    end
                    12'h004: if (pwdata[1]) done <= 1'b0; // Viết 1 vào bit DONE để xóa ngắt
                    12'h008: reg_opa <= pwdata;
                    12'h00C: reg_opb <= pwdata;
                    default: pslverr <= 1'b1;
                endcase
            end
            
            // Xử lý Đọc APB
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= {30'b0, reg_ctrl};
                    12'h004: prdata <= {30'b0, done, busy};
                    12'h008: prdata <= reg_opa;
                    12'h00C: prdata <= reg_opb;
                    12'h010: prdata <= reg_result;
                    default: pslverr <= 1'b1;
                endcase
            end

            // Core Logic Accelerator
            if (busy) begin
                if (compute_delay == 0) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    reg_result <= reg_opa * reg_opb; // Ví dụ: Phép nhân
                end else begin
                    compute_delay <= compute_delay - 1;
                end
            end
        end
    end
endmodule