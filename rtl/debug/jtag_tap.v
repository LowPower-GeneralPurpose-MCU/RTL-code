`timescale 1ns / 1ps

module jtag_tap (
    // JTAG Physical Pins
    input  wire       tck,
    input  wire       trst_n,
    input  wire       tms,
    input  wire       tdi,
    output wire       tdo,
    
    // Interface to Debug Module (DMI)
    output wire [4:0] o_ir,    
    output wire       o_shift_dr,
    output wire       o_capture_dr,
    output wire       o_update_dr,
    output wire       o_sel_dmi,  // 1 if DMI register is selected
    input  wire       i_tdo_dmi   // TDO from DMI register
);

    // IEEE 1149.1 Standard JTAG TAP States (16 states)
    localparam [3:0] 
        TLR = 4'h0, RTI = 4'h1, 
        SDS = 4'h2, CDR = 4'h3, SDR = 4'h4, E1D = 4'h5, PDR = 4'h6, E2D = 4'h7, UDR = 4'h8,
        SIS = 4'h9, CIR = 4'hA, SIR = 4'hB, E1I = 4'hC, PIR = 4'hD, E2I = 4'hE, UIR = 4'hF;

    // Standard JTAG Instructions
    localparam [4:0]
        IR_IDCODE = 5'b00001,
        IR_DMI    = 5'b10001, // Custom Instruction for Debug Module Interface
        IR_BYPASS = 5'b11111;

    reg [3:0] state, next_state;
    reg [4:0] ir_reg;
    reg [4:0] ir_shift_reg;
    reg       bypass_reg;

    // -----------------------------------------------------
    // TAP State Machine (Transitions based on TMS)
    // -----------------------------------------------------
    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) state <= TLR;
        else         state <= next_state;
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

    // -----------------------------------------------------
    // Instruction Register (IR) Logic
    // -----------------------------------------------------
    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            ir_reg <= IR_IDCODE;
            ir_shift_reg <= 5'b0;
        end else begin
            if (state == TLR) begin
                ir_reg <= IR_IDCODE;
            end else if (state == CIR) begin
                ir_shift_reg <= 5'b00001;
            end else if (state == SIR) begin
                ir_shift_reg <= {tdi, ir_shift_reg[4:1]};
            end else if (state == UIR) begin
                ir_reg <= ir_shift_reg;
            end
        end
    end

    // -----------------------------------------------------
    // Bypass Register Logic
    // -----------------------------------------------------
    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) bypass_reg <= 1'b0;
        else if (state == SDR && ir_reg == IR_BYPASS) bypass_reg <= tdi;
    end

    // -----------------------------------------------------
    // Outputs to Debug Module
    // -----------------------------------------------------
    assign o_ir         = ir_reg;
    assign o_shift_dr   = (state == SDR);
    assign o_capture_dr = (state == CDR);
    assign o_update_dr  = (state == UDR);
    assign o_sel_dmi    = (ir_reg == IR_DMI);

    // -----------------------------------------------------
    // TDO Muxing
    // -----------------------------------------------------
    reg tdo_reg;
    always @(*) begin
        if (state == SIR) begin
            tdo_reg = ir_shift_reg[0];
        end else if (state == SDR) begin
            case (ir_reg)
                IR_DMI:  tdo_reg = i_tdo_dmi;
                default: tdo_reg = bypass_reg;
            endcase
        end else begin
            tdo_reg = 1'b0;
        end
    end
    
    // TDO is typically output on falling edge of TCK in JTAG spec
    reg tdo_falling;
    always @(negedge tck or negedge trst_n) begin
        if (!trst_n) tdo_falling <= 1'b0;
        else         tdo_falling <= tdo_reg;
    end
    assign tdo = tdo_falling;

endmodule