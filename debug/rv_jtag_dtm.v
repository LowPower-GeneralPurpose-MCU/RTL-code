`timescale 1ns / 1ps

module rv_jtag_dtm #(
    parameter ABITS = 7
)(
    input  wire             tck,
    input  wire             trst_n,
    input  wire             tms,
    input  wire             tdi,
    output reg              tdo,

    output reg              dmi_req_valid,
    output reg  [ABITS-1:0] dmi_req_addr,
    output reg  [31:0]      dmi_req_data,
    output reg  [1:0]       dmi_req_op,
    input  wire             dmi_resp_ready, 
    input  wire             dmi_resp_valid,
    input  wire [31:0]      dmi_resp_data,
    input  wire [1:0]       dmi_resp_op
);

    localparam TLR=4'h0, RTI=4'h1, SDS=4'h2, CDR=4'h3, SDR=4'h4, E1D=4'h5, PDR=4'h6, E2D=4'h7,
               UDR=4'h8, SIS=4'h9, CIR=4'hA, SIR=4'hB, E1I=4'hC, PIR=4'hD, E2I=4'hE, UIR=4'hF;
    localparam IR_IDCODE = 5'h01, IR_DTMCS = 5'h10, IR_DMI = 5'h11;
    localparam [31:0] IDCODE_VAL = 32'h10e31913;

    reg [3:0] state, next_state;
    reg [4:0] ir, ir_shift;
    reg [ABITS+33:0] dr_shift;

    reg sticky_busy;
    reg busy_captured; // CỜ QUAN TRỌNG: Chống Ghost Execution

    reg [2:0] resp_valid_sync;
    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) resp_valid_sync <= 3'b0;
        else resp_valid_sync <= {resp_valid_sync[1:0], dmi_resp_valid};
    end
    wire resp_valid_tck = resp_valid_sync[2];

    reg [31:0] dmi_read_data;
    reg [1:0]  dmi_read_op;

    wire [1:0] dmistat = (dmi_req_valid || sticky_busy) ? 2'b11 : dmi_read_op;
    wire [31:0] dtmcs_val = {14'b0, 1'b0, 1'b0, 1'b0, 3'b000, dmistat, 6'd7, 4'h1};

    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) state <= TLR; else state <= next_state;
    end

    always @(*) begin
        case (state)
            TLR: next_state = tms ? TLR : RTI;
            RTI: next_state = tms ? SDS : RTI;
            SDS: next_state = tms ? SIS : CDR;
            CDR: next_state = tms ? E1D : SDR;
            SDR: next_state = tms ? E1D : SDR;
            E1D: next_state = tms ? UDR : PDR;
            PDR: next_state = tms ? E2D : PDR;
            E2D: next_state = tms ? UDR : SDR;
            UDR: next_state = tms ? SDS : RTI;
            SIS: next_state = tms ? TLR : CIR;
            CIR: next_state = tms ? E1I : SIR;
            SIR: next_state = tms ? E1I : SIR;
            E1I: next_state = tms ? UIR : PIR;
            PIR: next_state = tms ? E2I : PIR;
            E2I: next_state = tms ? UIR : SIR;
            UIR: next_state = tms ? SDS : RTI;
            default: next_state = TLR;
        endcase
    end

    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            ir <= IR_IDCODE; ir_shift <= 5'h01; dr_shift <= 0;
            dmi_req_valid <= 0; dmi_read_data <= 0; dmi_read_op <= 0;
            sticky_busy <= 0; busy_captured <= 0;
        end else begin
            if (state == TLR) begin
                ir <= IR_IDCODE;
                sticky_busy <= 1'b0; // Xóa cờ lỗi để phiên mới chạy mượt mà
            end
            case (state)
                CIR: ir_shift <= 5'b00001;
                SIR: ir_shift <= {tdi, ir_shift[4:1]};
                UIR: ir       <= ir_shift;
                CDR: begin
                    if (ir == IR_IDCODE)      dr_shift[31:0] <= IDCODE_VAL;
                    else if (ir == IR_DTMCS)  dr_shift[31:0] <= dtmcs_val;
                    else if (ir == IR_DMI) begin
                        // NẾU BẬN LÚC CAPTURE, BẬT CỜ KHÓA UPDATE LẠI!
                        if (dmi_req_valid || sticky_busy) begin
                            dr_shift <= { {(ABITS){1'b0}}, dmi_read_data, 2'b11};
                            busy_captured <= 1'b1; 
                        end else begin
                            dr_shift <= {dmi_req_addr, dmi_read_data, dmi_read_op};
                            busy_captured <= 1'b0;
                        end
                    end
                end
                SDR: begin
                    if (ir == IR_DMI) dr_shift <= {tdi, dr_shift[ABITS+33:1]};
                    else              dr_shift[31:0] <= {tdi, dr_shift[31:1]};
                end
            endcase

            if (state == UDR) begin
                if (ir == IR_DMI) begin
                    if (dr_shift[1:0] != 2'b00) begin 
                        // TỪ CHỐI LỆNH NẾU LÚC CAPTURE ĐÃ BÁO BUSY
                        if (busy_captured || sticky_busy) begin
                            sticky_busy <= 1'b1; 
                        end else if (!dmi_req_valid) begin
                            dmi_req_valid <= 1'b1;
                            dmi_req_addr  <= dr_shift[ABITS+33:34];
                            dmi_req_data  <= dr_shift[33:2];
                            dmi_req_op    <= dr_shift[1:0];
                            dmi_read_op   <= 2'b11; 
                        end
                    end
                end
                else if (ir == IR_DTMCS) begin
                    if (dr_shift[16]) begin dmi_read_op <= 2'b00; sticky_busy <= 1'b0; end
                end
            end 
            else if (resp_valid_tck && dmi_req_valid) begin
                dmi_req_valid <= 1'b0;          
                dmi_read_data <= dmi_resp_data; 
                dmi_read_op   <= dmi_resp_op;   
            end
        end
    end

    always @(negedge tck or negedge trst_n) begin
        if (!trst_n) tdo <= 1'b0;
        else begin
            case (state)
                SIR: tdo <= ir_shift[0];
                SDR: tdo <= dr_shift[0];
                CIR: tdo <= 1'b1; 
                CDR: begin
                    if (ir == IR_IDCODE)      tdo <= IDCODE_VAL[0];
                    else if (ir == IR_DTMCS)  tdo <= dtmcs_val[0];
                    else if (ir == IR_DMI)    tdo <= (dmi_req_valid || sticky_busy) ? 1'b1 : dmi_read_op[0]; 
                    else tdo <= 1'b0;
                end
                default: tdo <= 1'b0;
            endcase
        end
    end
endmodule