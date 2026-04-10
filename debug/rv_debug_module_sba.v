`timescale 1ns / 1ps

module rv_debug_module_sba (
    input  wire clk_sys,
    input  wire rst_sys_n,

    input  wire        dmi_req_valid,
    input  wire [6:0]  dmi_req_addr,
    input  wire [31:0] dmi_req_data,
    input  wire [1:0]  dmi_req_op,
    output reg         dmi_resp_ready, 
    output reg         dmi_resp_valid,
    output reg  [31:0] dmi_resp_data,
    output reg  [1:0]  dmi_resp_op,

    output reg         axi_req,
    output reg  [1:0]  axi_op, 
    output wire [1:0]  axi_size,     
    output reg  [31:0] axi_addr,
    output reg  [31:0] axi_wdata,
    input  wire        axi_ack,
    input  wire [31:0] axi_rdata,
    input  wire [1:0]  axi_resp,

    output reg         cpu_halt_req,
    output reg         cpu_resume_req,      
    input  wire        cpu_halted,
    
    output reg [15:0]  cpu_reg_read_addr,  
    input  wire [31:0] cpu_reg_read_data,
    output reg         cpu_reg_write_en,
    output reg [15:0]  cpu_reg_write_addr,
    output reg [31:0]  cpu_reg_write_data,

    output wire        ndmreset_req
);
    localparam DATA0      = 7'h04;
    localparam DMCONTROL  = 7'h10;
    localparam DMSTATUS   = 7'h11;
    localparam ABSTRACTCS = 7'h16;
    localparam COMMAND    = 7'h17;
    localparam SBCS       = 7'h38;
    localparam SBADDRESS0 = 7'h39;
    localparam SBDATA0    = 7'h3C;

    reg [31:0] sbaddress0;
    reg [31:0] sbdata0;
    reg [2:0]  sbaccess;
    assign axi_size = sbaccess[1:0];
    reg        sbautoincrement;
    reg        sbreadondata;
    reg        sbreadonaddr;
    reg [2:0]  sberror;
    
    reg [31:0] data0_reg;
    reg [31:0] dmcontrol_reg;
    reg        dmactive;
    reg        havereset;
    
    reg trigger_axi_read;
    reg trigger_axi_write;
    reg is_background_axi; 

    wire [31:0] sbcs_val = {
        3'b001, 7'b0, 1'b0, sbreadonaddr, sbaccess, sbautoincrement, sbreadondata, sberror, 
        7'd32, 2'b0, 1'b1, 1'b1, 1'b1
    };
    
    reg [2:0] req_valid_sync;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) req_valid_sync <= 3'b0;
        else            req_valid_sync <= {req_valid_sync[1:0], dmi_req_valid};
    end
    wire req_valid_sys = req_valid_sync[2];

    assign ndmreset_req = dmcontrol_reg[1];

    reg [1:0] state;
    localparam IDLE=0, RESP=1, WAIT_AXI=2;
    
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            state <= IDLE;
            havereset <= 1'b1;
            dmi_resp_ready <= 1'b1; dmi_resp_valid <= 1'b0; dmi_resp_data <= 32'b0; dmi_resp_op <= 2'b0;
            axi_req <= 1'b0;
            cpu_halt_req <= 1'b0; cpu_resume_req <= 1'b0;
            cpu_reg_write_en <= 1'b0; dmcontrol_reg <= 32'b0; dmactive <= 1'b0;
            sbaccess <= 3'd2; sbautoincrement <= 1'b0; sbreadondata <= 1'b0; sbreadonaddr <= 1'b0;
            sberror <= 3'd0;
            data0_reg <= 32'b0;
            trigger_axi_read <= 1'b0; trigger_axi_write <= 1'b0; is_background_axi <= 1'b0;
        end else begin
            cpu_reg_write_en <= 1'b0; 
            
            case (state)
                IDLE: begin
                    dmi_resp_valid <= 1'b0;
                    if (req_valid_sys) begin
                        dmi_resp_op <= 2'b00;
                        if (dmi_req_op == 2'd2) begin // WRITE
                            case (dmi_req_addr)
                                DMCONTROL: begin
                                    dmcontrol_reg <= dmi_req_data; dmactive <= dmi_req_data[0];
                                    cpu_halt_req <= dmi_req_data[31];
                                    if (dmi_req_data[1]) havereset <= 1'b1;
                                    if (dmi_req_data[28]) havereset <= 1'b0;
                                    if (dmi_req_data[30]) cpu_resume_req <= 1'b1; // Chỉ kích hoạt Resume
                                    state <= RESP;
                                end
                                DATA0: begin data0_reg <= dmi_req_data; state <= RESP; end
                                COMMAND: begin
                                    if (dmi_req_data[31:24] == 8'h00) begin
                                        if (dmi_req_data[16]) begin 
                                            cpu_reg_write_addr <= dmi_req_data[15:0];
                                            cpu_reg_write_data <= data0_reg;
                                            cpu_reg_write_en   <= 1'b1;
                                        end else cpu_reg_read_addr <= dmi_req_data[15:0];
                                    end
                                    state <= RESP;
                                end
                                SBCS: begin
                                    sbreadonaddr <= dmi_req_data[20];
                                    sbaccess <= dmi_req_data[19:17];
                                    sbautoincrement <= dmi_req_data[16]; sbreadondata <= dmi_req_data[15];
                                    if (dmi_req_data[14]) sberror[2] <= 1'b0;
                                    if (dmi_req_data[13]) sberror[1] <= 1'b0;
                                    if (dmi_req_data[12]) sberror[0] <= 1'b0;
                                    state <= RESP;
                                end
                                SBADDRESS0: begin 
                                    sbaddress0 <= dmi_req_data;
                                    if (sberror == 0 && sbreadonaddr) trigger_axi_read <= 1'b1; 
                                    state <= RESP;
                                end
                                SBDATA0: begin
                                    sbdata0 <= dmi_req_data;
                                    if (sberror == 0) begin
                                        if (sbreadonaddr || sbreadondata) begin
                                            trigger_axi_write <= 1'b1;
                                            state <= RESP;
                                        end else begin
                                            axi_addr <= sbaddress0;
                                            axi_wdata <= dmi_req_data;
                                            axi_op <= 2'd2; axi_req <= 1'b1;
                                            is_background_axi <= 1'b0; state <= WAIT_AXI;
                                        end
                                    end else state <= RESP;
                                end
                                default: state <= RESP;
                            endcase
                        end 
                        else if (dmi_req_op == 2'd1) begin // READ
                            case (dmi_req_addr)
                                DMCONTROL:  begin dmi_resp_data <= dmcontrol_reg; state <= RESP; end
                                DMSTATUS: begin 
                                    dmi_resp_data <= (dmactive) ? 
                                        (32'h000000A2 | 
                                        (cpu_halted ? 32'h00000300 : 32'h00000C00) | 
                                        (havereset  ? 32'h00030000 : 32'h00000000)) 
                                        : 32'h00000002; 
                                    state <= RESP; 
                                end
                                ABSTRACTCS: begin dmi_resp_data <= 32'h00000001; state <= RESP; end
                                DATA0:      begin dmi_resp_data <= cpu_reg_read_data; state <= RESP; end
                                SBCS:       begin dmi_resp_data <= sbcs_val; state <= RESP; end
                                SBADDRESS0: begin dmi_resp_data <= sbaddress0; state <= RESP; end
                                SBDATA0: begin
                                    if (sberror == 0) begin
                                        if (sbreadonaddr || sbreadondata) begin
                                            dmi_resp_data <= sbdata0;
                                            if (sbreadondata) trigger_axi_read <= 1'b1;
                                            state <= RESP;
                                        end else begin
                                            axi_addr <= sbaddress0;
                                            axi_op <= 2'd1; axi_req <= 1'b1;
                                            is_background_axi <= 1'b0; state <= WAIT_AXI;
                                        end
                                    end else begin dmi_resp_data <= 32'h0; state <= RESP; end
                                end
                                default: begin dmi_resp_data <= 32'h0; state <= RESP; end
                            endcase
                        end
                    end
                end

                RESP: begin
                    dmi_resp_valid <= 1'b1;
                    cpu_resume_req <= 1'b0; // AUTO-CLEAR để tạo xung đúng 1 nhịp
                    
                    if (!req_valid_sys) begin 
                        dmi_resp_valid <= 1'b0;
                        if (trigger_axi_read) begin
                            axi_addr <= sbaddress0;
                            axi_op <= 2'd1; axi_req <= 1'b1;
                            trigger_axi_read <= 1'b0; is_background_axi <= 1'b1; state <= WAIT_AXI;
                        end else if (trigger_axi_write) begin
                            axi_addr <= sbaddress0;
                            axi_wdata <= sbdata0; axi_op <= 2'd2; axi_req <= 1'b1;
                            trigger_axi_write <= 1'b0; is_background_axi <= 1'b1; state <= WAIT_AXI;
                        end else state <= IDLE;
                    end
                end

                WAIT_AXI: begin
                    if (axi_ack) begin
                        axi_req <= 1'b0;
                        if (axi_op == 2'd1) begin
                            if (is_background_axi) sbdata0 <= axi_rdata;
                            else dmi_resp_data <= axi_rdata;
                        end
                        
                        if (axi_resp != 0) sberror <= 3'd2;
                        else if (sbautoincrement) sbaddress0 <= sbaddress0 + (1 << sbaccess);
                        
                        if (is_background_axi) begin
                            is_background_axi <= 1'b0; state <= IDLE;
                        end else state <= RESP;
                    end
                end
            endcase
            
            if (state == RESP && dmi_req_addr == COMMAND && !dmi_req_data[16]) data0_reg <= cpu_reg_read_data;
        end
    end
endmodule