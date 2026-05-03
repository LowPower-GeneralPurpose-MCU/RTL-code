`timescale 1ns / 1ps

module rv_debug_module_sba (
    input  wire clk_sys,
    input  wire rst_sys_n,

    // Giao tiếp với DTM (đã qua CDC, giả định DTM đưa tín hiệu qua module CDC trước)
    input  wire        dmi_req_valid,
    input  wire [6:0]  dmi_req_addr,
    input  wire [31:0] dmi_req_data,
    input  wire [1:0]  dmi_req_op,
    output reg         dmi_resp_ready,
    output reg         dmi_resp_valid,
    output reg  [31:0] dmi_resp_data,
    output reg  [1:0]  dmi_resp_op,

    // Giao tiếp với dtm_axi_master.v hiện tại của bạn
    output reg         axi_req,
    output reg  [1:0]  axi_op,      // 1: Read, 2: Write
    output reg  [31:0] axi_addr,
    output reg  [31:0] axi_wdata,
    input  wire        axi_ack,
    input  wire [31:0] axi_rdata,
    input  wire [1:0]  axi_resp
);

    // Standard RISC-V DM Registers
    localparam DMCONTROL = 7'h10;
    localparam DMSTATUS  = 7'h11;
    localparam SBCS      = 7'h38;
    localparam SBADDRESS0= 7'h39;
    localparam SBDATA0   = 7'h3C;

    reg [31:0] sbaddress0;
    reg [31:0] sbdata0;
    
    // State machine siêu đơn giản
    reg [1:0] state;
    localparam IDLE=0, WAIT_AXI=1, RESP=2;

    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            state <= IDLE;
            dmi_resp_ready <= 1'b1;
            dmi_resp_valid <= 1'b0;
            axi_req <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    dmi_resp_valid <= 1'b0;
                    if (dmi_req_valid && dmi_resp_ready) begin
                        dmi_resp_ready <= 1'b0;
                        
                        if (dmi_req_op == 2'd2) begin // WRITE DMI
                            case (dmi_req_addr)
                                SBADDRESS0: begin 
                                    sbaddress0 <= dmi_req_data;
                                    state <= RESP; // Không tạo AXI req
                                end
                                SBDATA0: begin
                                    sbdata0 <= dmi_req_data;
                                    // Bắt đầu đẩy dữ liệu ra AXI
                                    axi_addr <= sbaddress0;
                                    axi_wdata <= dmi_req_data;
                                    axi_op <= 2'd2; // Write
                                    axi_req <= 1'b1;
                                    state <= WAIT_AXI;
                                    sbaddress0 <= sbaddress0 + 4; // Auto increment
                                end
                                default: state <= RESP; // Phớt lờ các thanh ghi khác
                            endcase
                        end else if (dmi_req_op == 2'd1) begin // READ DMI
                            case (dmi_req_addr)
                                DMSTATUS: begin
                                    dmi_resp_data <= 32'h00000000; // Fake DMSTATUS
                                    state <= RESP;
                                end
                                SBDATA0: begin
                                    axi_addr <= sbaddress0;
                                    axi_op <= 2'd1; // Read
                                    axi_req <= 1'b1;
                                    state <= WAIT_AXI;
                                    sbaddress0 <= sbaddress0 + 4; // Auto increment
                                end
                                default: begin
                                    dmi_resp_data <= 32'h0;
                                    state <= RESP;
                                end
                            endcase
                        end
                    end
                end

                WAIT_AXI: begin
                    if (axi_ack) begin
                        axi_req <= 1'b0;
                        if (axi_op == 2'd1) dmi_resp_data <= axi_rdata; // Read data from AXI
                        dmi_resp_op <= (axi_resp == 0) ? 2'b00 : 2'b10; // Chuyển đổi AXI lỗi sang DMI lỗi
                        state <= RESP;
                    end
                end

                RESP: begin
                    dmi_resp_valid <= 1'b1;
                    dmi_resp_ready <= 1'b1; // Sẵn sàng nhận lệnh mới
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule