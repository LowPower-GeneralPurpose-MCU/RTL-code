//==================================================================================================
// File: register_file.v
//==================================================================================================

module register_file (
    input clk,
    input reset_n,
    input [4:0] read_reg1,
    input [4:0] read_reg2,
    input mem_wb_reg_write,
    input [4:0] mem_wb_rd,
    input [31:0] mem_wb_write_data,
    output [31:0] read_data1,
    output [31:0] read_data2,
    
    // --- DEBUG CHUYÊN DỤNG ---
    input  wire        dbg_mode,           // Báo hiệu CPU đang Halt
    input  wire [4:0]  dbg_read_addr,      // DM muốn đọc thanh ghi nào
    output wire [31:0] dbg_read_data,      // Dữ liệu trả về cho DM
    input  wire        dbg_write_en,       // DM muốn ghi đè
    input  wire [4:0]  dbg_write_addr,     // Địa chỉ DM muốn ghi
    input  wire [31:0] dbg_write_data      // Dữ liệu DM ghi
);

    (* ram_style = "distributed" *) reg [31:0] rf_main [0:31];
    reg [31:0] x2_sp;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            x2_sp <= 32'h8001_0000;
        end else begin
            // Ưu tiên cao nhất cho Debug Mode ghi đè
            if (dbg_mode && dbg_write_en) begin
                if (dbg_write_addr == 5'd2) begin
                    x2_sp <= dbg_write_data;
                end else if (dbg_write_addr != 5'd0) begin
                    rf_main[dbg_write_addr] <= dbg_write_data;
                end
            end 
            // Nếu không Halt, pipeline hoạt động bình thường
            else if (mem_wb_reg_write) begin
                if (mem_wb_rd == 5'd2) begin
                    x2_sp <= mem_wb_write_data;
                end else if (mem_wb_rd != 5'd0) begin
                    rf_main[mem_wb_rd] <= mem_wb_write_data;
                end
            end
        end
    end

    // Các cổng đọc của CPU
    assign read_data1 = (read_reg1 == 5'd0) ? 32'd0 :
                        (read_reg1 == 5'd2) ? x2_sp : rf_main[read_reg1];
                        
    assign read_data2 = (read_reg2 == 5'd0) ? 32'd0 :
                        (read_reg2 == 5'd2) ? x2_sp : rf_main[read_reg2];

    // Cổng đọc riêng biệt của Debug (Tổ hợp, trả về ngay lập tức)
    assign dbg_read_data = (dbg_read_addr == 5'd0) ? 32'd0 :
                           (dbg_read_addr == 5'd2) ? x2_sp : rf_main[dbg_read_addr];
                        
endmodule


module f_register_file (
    input clk, 
    input reset_n,
    input [4:0] read_reg1, 
    input [4:0] read_reg2,
    output [31:0] read_data1, 
    output [31:0] read_data2,
    input reg_write_en,   
    input [4:0] write_reg,    
    input [31:0] write_data,
    
    // --- BỔ SUNG CỔNG DEBUG CHUYÊN DỤNG ---
    input  wire        dbg_mode,           // Trạng thái CPU đang Halt
    input  wire [4:0]  dbg_read_addr,      // Địa chỉ FPR cần đọc
    output wire [31:0] dbg_read_data,      // Dữ liệu trả về
    input  wire        dbg_write_en,       // Cho phép DM ghi đè
    input  wire [4:0]  dbg_write_addr,     // Địa chỉ FPR cần ghi
    input  wire [31:0] dbg_write_data      // Dữ liệu DM ghi
);
    (* ram_style = "distributed" *) reg [31:0] f_regfile [0:31];
    
    // Đọc cho Pipeline
    assign read_data1 = f_regfile[read_reg1];
    assign read_data2 = f_regfile[read_reg2];
    
    // Đọc cho Debug (Tổ hợp, trả về ngay lập tức)
    assign dbg_read_data = f_regfile[dbg_read_addr];
    
    integer i;
    always @(posedge clk) begin
        if (!reset_n) begin

        end else begin
            // Ưu tiên cao nhất cho Debug Mode ghi đè
            if (dbg_mode && dbg_write_en) begin
                f_regfile[dbg_write_addr] <= dbg_write_data;
            end 
            // Nếu không Halt, pipeline hoạt động bình thường
            else if (reg_write_en) begin
                f_regfile[write_reg] <= write_data;
            end
        end
    end
endmodule


module csr_register_file (
    input clk,
    input reset_n,
    input meip_i, // External Interrupt Pending
    input msip_i, // Software Interrupt Pending
    input mtip_i, // Timer Interrupt Pending
    input [11:0] csr_addr,
    output reg [31:0] csr_read_data,
    input [11:0] csr_write_addr,
    input [31:0] csr_write_data,
    input [1:0] csr_op,
    input csr_write_en,
    input count_en,
    input instret_en,
    input trap_enter,
    input mret_exec,
    input [31:0] trap_cause,
    input [31:0] trap_pc,
    input [31:0] trap_val,
    output [31:0] mtvec_out,
    output [31:0] mepc_out,
    output [31:0] mie_out,
    output mstatus_mie,

    // --- CỔNG GIAO TIẾP DEBUG ---
    input         dbg_halt_req,
    input         dbg_halted,
    input  [31:0] debug_pc_in,
    output [31:0] dpc_out,
    output [31:0] dcsr_out,
    
    input  [11:0] dbg_reg_read_addr,  // Trỏ thẳng vào địa chỉ CSR 12-bit
    output reg [31:0] dbg_read_data,  // Đọc mọi CSR
    input         dbg_reg_write_en,
    input  [11:0] dbg_reg_write_addr, // Trỏ thẳng vào địa chỉ CSR 12-bit
    input  [31:0] dbg_reg_write_data
);

    localparam [31:0] MVENDORID  = 32'h0;
    localparam [31:0] MARCHID    = 32'h0;
    localparam [31:0] MIMPID     = 32'h01000000;
    localparam [31:0] MHARTID    = 32'h0;
    localparam [31:0] MISA       = 32'h40001120;

    reg [31:0] mstatus;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mscratch;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mtval;
    
    // Debug CSRs
    reg [31:0] dcsr;
    reg [31:0] dpc;
    reg [31:0] dscratch0;

    wire [31:0] mip_val = {20'd0, meip_i, 3'd0, mtip_i, 3'd0, msip_i, 3'd0};

    reg [63:0] mcycle;
    reg [63:0] minstret;
    reg        dbg_halted_q;

    assign mtvec_out = mtvec;
    assign mepc_out = mepc;
    assign mstatus_mie = mstatus[3];
    assign mie_out = mie;
    
    // Bổ sung: Gán giá trị dpc ra cổng dpc_out
    assign dpc_out = dpc;
    assign dcsr_out = dcsr;

    always @(*) begin
        case (csr_addr)
            12'hF11: csr_read_data = MVENDORID;
            12'hF12: csr_read_data = MARCHID;
            12'hF13: csr_read_data = MIMPID;
            12'hF14: csr_read_data = MHARTID;
            12'h301: csr_read_data = MISA;
            12'h300: csr_read_data = mstatus;
            12'h304: csr_read_data = mie;
            12'h305: csr_read_data = mtvec;
            12'h340: csr_read_data = mscratch;
            12'h341: csr_read_data = mepc;
            12'h342: csr_read_data = mcause;
            12'h343: csr_read_data = mtval;
            12'h344: csr_read_data = mip_val;
            12'hB00, 12'hC00, 12'hC01: csr_read_data = mcycle[31:0];
            12'hB80, 12'hC80, 12'hC81: csr_read_data = mcycle[63:32];
            12'hB02, 12'hC02:          csr_read_data = minstret[31:0];
            12'hB82, 12'hC82:          csr_read_data = minstret[63:32];
            // Read Debug CSRs
            12'h7b0: csr_read_data = dcsr;
            12'h7b1: csr_read_data = dpc;
            12'h7b2: csr_read_data = dscratch0;
            default: csr_read_data = 32'b0;
        endcase
    end

    always @(*) begin
        case (dbg_reg_read_addr)
            12'hF11: dbg_read_data = MVENDORID;
            12'hF12: dbg_read_data = MARCHID;
            12'h301: dbg_read_data = MISA;
            12'h300: dbg_read_data = mstatus;
            12'h304: dbg_read_data = mie;
            12'h305: dbg_read_data = mtvec;
            12'h341: dbg_read_data = mepc;
            12'hB00, 12'hC00, 12'hC01: dbg_read_data = mcycle[31:0];
            // Debug CSRs
            12'h7b0: dbg_read_data = dcsr;
            12'h7b1: dbg_read_data = dpc;
            12'h7b2: dbg_read_data = dscratch0;
            default: dbg_read_data = 32'b0;
        endcase
    end
    
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            mstatus   <= 32'h00003800;
            mie       <= 32'b0;
            mtvec     <= 32'b0;
            mscratch  <= 32'b0;
            mepc      <= 32'b0;
            mcause    <= 32'b0;
            mtval     <= 32'b0;
            mcycle    <= 64'b0;
            minstret  <= 64'b0;
            dcsr      <= 32'h00000003; // Privilege mode M
            dpc       <= 32'b0;
            dscratch0 <= 32'b0;
            dbg_halted_q <= 1'b0;
        end else begin
            dbg_halted_q <= dbg_halted;

            // 1. Logic tự động lưu PC & Cause khi CPU VỪA BỊ HALT (Cạnh lên)
            if (dbg_halted && !dbg_halted_q) begin
                dpc <= debug_pc_in;
                
                // Cập nhật nguyên nhân Halt vào dcsr[8:6]
                if (dbg_halt_req) begin
                    dcsr[8:6] <= 3'd3; // Cause 3: Dừng do bị yêu cầu Halt
                end else begin
                    dcsr[8:6] <= 3'd4; // Cause 4: Dừng do hoàn tất lệnh Step
                end
            end
            // 2. OpenOCD chủ động ghi đè DPC / DCSR qua lệnh Debug
            else if (dbg_reg_write_en && dbg_reg_write_addr == 12'h7b1) begin
                dpc <= dbg_reg_write_data;
            end
            else if (dbg_reg_write_en && dbg_reg_write_addr == 12'h7b0) begin
                dcsr <= dbg_reg_write_data;
            end
            // Nếu phần mềm cố tình ghi đè DPC bằng lệnh CSR write nội bộ...
            else if (csr_write_en && (csr_op != 2'b00) && (csr_write_addr == 12'h7b1)) begin
                dpc <= csr_write_data;
            end

            if (count_en) begin
                mcycle <= mcycle + 64'd1;
            end
            
            if (instret_en) begin
                minstret <= minstret + 64'd1;
            end
            
            if (trap_enter) begin
                mepc <= trap_pc;
                mcause <= trap_cause;
                mtval <= trap_val;
                mstatus[7] <= mstatus[3];
                mstatus[3] <= 1'b0;
                mstatus[12:11] <= 2'b11;
            end else if (mret_exec) begin
                mstatus[3] <= mstatus[7];
                mstatus[7] <= 1'b1;
                mstatus[12:11] <= 2'b11;
            end else if (csr_write_en && (csr_op != 2'b00)) begin
                case (csr_write_addr)
                    12'h300: begin
                        mstatus[3] <= csr_write_data[3];
                        mstatus[7] <= csr_write_data[7];
                        mstatus[12:11] <= csr_write_data[12:11];
                    end
                    12'h304: mie <= csr_write_data & 32'h00000888;
                    12'h305: mtvec <= csr_write_data;
                    12'h340: mscratch <= csr_write_data;
                    12'h341: mepc <= csr_write_data;
                    12'h342: mcause <= csr_write_data;
                    12'h343: mtval <= csr_write_data;
                    12'hB00: mcycle[31:0] <= csr_write_data;
                    12'hB80: mcycle[63:32] <= csr_write_data;
                    12'hB02: minstret[31:0] <= csr_write_data;
                    12'hB82: minstret[63:32] <= csr_write_data;
                    // Write Debug CSRs
                    12'h7b0: dcsr <= csr_write_data;
                    // 12'h7b1: dpc <= csr_write_data; // Đã xử lý ở khối ưu tiên phía trên
                    12'h7b2: dscratch0 <= csr_write_data;
                endcase
            end
        end
    end
    
endmodule