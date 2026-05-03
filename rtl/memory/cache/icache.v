`timescale 1ns / 1ps

// =============================================================================
// SUB-MODULE: ICache Data RAM & Tag RAM 
// =============================================================================
module icache_data_ram (
    input  wire        clk,
    input  wire [3:0]  index, // 4 bit nên được 16 set
    input  wire [1:0]  write_en,  // chọn way để ghi 
    input  wire [63:0] write_data,
    output wire [63:0] data1_out,
    output wire [63:0] data2_out
);
    (* ram_style = "distributed" *) reg [63:0] data1 [0:15]; // 16 set, mỗi set là 1 mảng data 64 bit 
    (* ram_style = "distributed" *) reg [63:0] data2 [0:15]; // như trên nhma cho set2
    assign data1_out = data1[index];
    assign data2_out = data2[index];
    always @(posedge clk) begin
        if (write_en[0]) data1[index] <= write_data;
        if (write_en[1]) data2[index] <= write_data;
    end
endmodule

module icache_tag_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [1:0]  write_en,
    input  wire [24:0] write_tag,
    output wire [24:0] tag1_out,
    output wire [24:0] tag2_out
);
    (* ram_style = "distributed" *) reg [24:0] tag1 [0:15];
    (* ram_style = "distributed" *) reg [24:0] tag2 [0:15];
    assign tag1_out = tag1[index];
    assign tag2_out = tag2[index];
    always @(posedge clk) begin
        if (write_en[0]) tag1[index] <= write_tag;
        if (write_en[1]) tag2[index] <= write_tag;
    end
endmodule

// =============================================================================
// MAIN MODULE: Instruction Cache Controller 
// =============================================================================
module instruction_cache (
    input  wire        clk,
    input  wire        rst_n,         
    input  wire        flush, // xóa valid cache ( nào branch hay trap thì dùng để nạp cái mới vào)
    input  wire        dcache_busy, // này để báo dcache đang dùng bus 
    input  wire predict_taken,
    input  wire btb_hit,
    input  wire [31:0] predict_target,
    
    // Interface CPU
    input  wire        cpu_read_req, // gửi yêu cầu để đọc lệnh từ bộ nhớ
    input  wire [31:0] cpu_addr,       // địa chỉ cần đọc
    output reg  [31:0] cpu_read_data,   // lệnh đọc được
    output reg         icache_hit,  // báo hit
    output reg         icache_stall,    // miss
    
    // Phần giao tiếp với bus AXI
    output reg  [31:0] m_axi_araddr, // địa chỉ block lệnh bị miss
    output wire [7:0]  m_axi_arlen, // số gói 
    output wire [2:0]  m_axi_arsize,    // kích thước mỗi gói
    output wire [1:0]  m_axi_arburst,   // cách tính địa chỉ gói tiếp 
    output reg         m_axi_arvalid,   // được nhận 
    input  wire        m_axi_arready,   // 
    
    input  wire [31:0] m_axi_rdata, // dữ liệu nhận được
    input  wire [1:0]  m_axi_rresp, // này để check xem đọc có lỗi ko
    input  wire        m_axi_rlast, //  báo gói cuối
    // handshake
    input  wire        m_axi_rvalid, // bus báo hợp lệ
    output reg         m_axi_rready // cache sẵn sàng nhận 
);

    wire [63:0] data1_out, data2_out;
    wire [24:0] tag1_out, tag2_out;
    
    reg valid1 [0:15];
    reg valid2 [0:15];
    reg plru   [0:15];

    assign m_axi_arlen   = 8'd1;  
    assign m_axi_arsize  = 3'b010; 
    assign m_axi_arburst = 2'b01; // ( N' = N+4)

    // Các trạng thái FSM 
    localparam IDLE        = 4'd0;
    localparam AR_REQ      = 4'd1; // gửi địa chỉ muốn lấy
    localparam R_WAIT_1    = 4'd2;
    localparam R_WAIT_2    = 4'd3;
    localparam UPDATE_RAM  = 4'd4; // ghi vào cache
    localparam PF_AR_REQ   = 4'd5;  // cũng là địa chỉ muốn lấy
    localparam PF_R_WAIT_1 = 4'd6;  
    localparam PF_R_WAIT_2 = 4'd7;  
    localparam PF_DONE     = 4'd8;  // ghi vào prefetch buffer
    
    reg [3:0]  state, next_state;
    reg [1:0]  way_update;
    reg [63:0] fetch_buffer;
    reg [31:0] miss_addr;
    reg fetch_error;
    reg [28:0] pf_req_addr;
    
    reg pb_write_en;   // enable ghi vào prefetch buffer
    reg pb_promote_en; // lấy trong buffer ra bỏ vào cache

    // Prefetch buffer 
    reg        pb_valid; 
    reg [28:0] pb_addr;     
    reg [63:0] pb_data;

    function is_prefetch_addr_allowed; // trả 1 nếu addr nằm trong vùng có thể fetch instruction, check kẻo địa chỉ nó nhảy ra vùng ko hợp lệ
        input [31:0] addr;
        begin
            if ((addr >= 32'h0000_1000 && addr <= 32'h0000_4FFF) ||
                (addr >= 32'h2000_0000 && addr <= 32'h2FFF_FFFF) ||
                (addr >= 32'h8000_0000 && addr <= 32'h8000_7FFF))    // này mấy vùng hợp lệ
                is_prefetch_addr_allowed = 1'b1;
            else
                is_prefetch_addr_allowed = 1'b0;
        end
    endfunction

    wire [31:0] current_addr = (state == AR_REQ || state == R_WAIT_1 || state == R_WAIT_2 || state == UPDATE_RAM) ? miss_addr : cpu_addr;
    wire [31:0] prefetch_base_addr = (state == UPDATE_RAM) ? miss_addr :(pb_promote_en)? cpu_addr:miss_addr; // lấy địa chỉ base, nếu miss thì base là lệnh mis, hit thì lấy lệnh sau
    
    // địa chỉ chọn nếu tuần tự    
    wire [31:0] seq_prefetch_addr = { (prefetch_base_addr[31:3] + 29'd1), 3'b000 };

    // đỉa chỉ chọn nếu rẽ nhánh
    wire [31:0] br_prefetch_addr  = { predict_target[31:3], 3'b000 };   

    wire branch_prefetch_en = predict_taken && btb_hit && is_prefetch_addr_allowed(br_prefetch_addr) && (br_prefetch_addr[31:3] != current_addr[31:3]) && !(pb_valid && pb_addr == br_prefetch_addr[31:3]);
    wire seq_prefetch_en =is_prefetch_addr_allowed(seq_prefetch_addr) && (seq_prefetch_addr[31:3] != current_addr[31:3]) && !(pb_valid && pb_addr == seq_prefetch_addr[31:3]);

    wire [31:0] prefetch_addr =branch_prefetch_en ? br_prefetch_addr : seq_prefetch_addr;
    wire prefetch_allowed = branch_prefetch_en ? 1'b1 : seq_prefetch_en;


    wire [24:0] tag         = current_addr[31:7];
    wire [3:0]  index       = current_addr[6:3];
    wire        word_offset = current_addr[2];
    
    wire [63:0] sram_write_data = pb_promote_en ? pb_data : fetch_buffer; // nếu có trong buffer thì  lấy ra ko thì lấy từ fetch buffer

    icache_data_ram DATA_RAM (
        .clk(clk), .index(index), .write_en(way_update), 
        .write_data(sram_write_data), .data1_out(data1_out), .data2_out(data2_out)
    );

    icache_tag_ram TAG_RAM (
        .clk(clk), .index(index), .write_en(way_update), 
        .write_tag(tag), .tag1_out(tag1_out), .tag2_out(tag2_out)
    );

    // ====================================================================
    // KHỐI 1: Quản lý State và SRAM Metadata
    // ====================================================================
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            for (i=0; i<16; i=i+1) begin 
                valid1[i] <= 1'b0; 
                valid2[i] <= 1'b0;
                plru[i] <= 1'b0; 
            end
        end else begin
            state <= next_state;

            // Flush: xóa toàn bộ valid bits và PLRU
            if (flush) begin
                for (i=0; i<16; i=i+1) begin 
                    valid1[i] <= 1'b0; 
                    valid2[i] <= 1'b0; 
                    plru[i] <= 1'b0; 
                end
            end else if (cpu_read_req && state == IDLE) begin
                if (valid1[index] && tag1_out == tag) plru[index] <= 1'b1; //hit way 1
                else if (valid2[index] && tag2_out == tag) plru[index] <= 1'b0;
                else if (pb_promote_en) begin // nếu trong buffer
                    if (way_update[0]) begin valid1[index] <= 1'b1; plru[index] <= 1'b1; end
                    else if (way_update[1]) begin valid2[index] <= 1'b1; plru[index] <= 1'b0; end
                end
            end else if (state == UPDATE_RAM) begin // miss rồi nên h nó ghi vào sram của cache
                if (!fetch_error) begin
                    if (way_update[0]) begin valid1[index] <= 1'b1; plru[index] <= 1'b1; end 
                    else if (way_update[1]) begin valid2[index] <= 1'b1; plru[index] <= 1'b0; end
                end
            end
        end
    end

    // ====================================================================
    // KHỐI 2: Quản lý Trạm Trung Chuyển (Fetch Buffer) & Miss Address
    // ====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            miss_addr <= 32'b0;
            fetch_buffer <= 64'b0;
            fetch_error <= 1'b0;
            pf_req_addr <= 29'b0;
        end else begin
            if (state == AR_REQ && m_axi_arready)
                fetch_error <= 1'b0;
            else if (state == PF_AR_REQ && m_axi_arready) begin
                fetch_error <= 1'b0;
                pf_req_addr <= prefetch_addr[31:3];
            end else if ((state == R_WAIT_1 || state == R_WAIT_2 || state == PF_R_WAIT_1 || state == PF_R_WAIT_2) && m_axi_rvalid && m_axi_rready && (m_axi_rresp != 2'b00))
                fetch_error <= 1'b1;
            if (state == IDLE && cpu_read_req && !icache_hit) begin // miss
                miss_addr <= cpu_addr;
            end else if (pb_promote_en) begin
                miss_addr <= cpu_addr; 
            end

            if (state == R_WAIT_1 && m_axi_rvalid && m_axi_rready) // nhận được dữ liệu từ axi
                fetch_buffer[31:0]  <= m_axi_rresp[1] ? 32'b0 : m_axi_rdata;
            else if (state == R_WAIT_2 && m_axi_rvalid && m_axi_rready) 
                fetch_buffer[63:32] <= m_axi_rresp[1] ? 32'b0 : m_axi_rdata;
            else if (state == PF_R_WAIT_1 && m_axi_rvalid && m_axi_rready) 
                fetch_buffer[31:0]  <= m_axi_rresp[1] ? 32'b0 : m_axi_rdata;
            else if (state == PF_R_WAIT_2 && m_axi_rvalid && m_axi_rready) 
                fetch_buffer[63:32] <= m_axi_rresp[1] ? 32'b0 : m_axi_rdata;
        end
    end

    // ====================================================================
    // KHỐI 3: Quản lý Prefetch Buffer Độc lập 
    // ====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pb_valid <= 1'b0; 
            pb_addr  <= 29'b0;
            pb_data  <= 64'b0;
        end else if (flush) begin
            pb_valid <= 1'b0;
        end else if (pb_promote_en) begin
            pb_valid <= 1'b0; // ms lấy ra cache nên xóa ở buffer
        end else if (pb_write_en) begin  // ghi cái mới vào
            pb_data  <= fetch_buffer;
            pb_addr  <= pf_req_addr;  
            pb_valid <= 1'b1;
        end
    end

    // ====================================================================
    // KHỐI TỔ HỢP: Điều khiển FSM & Logic Bắt Hit Toàn Cầu
    // ====================================================================
    always @(*) begin
        next_state    = state;
        icache_hit    = 1'b0;
        icache_stall  = 1'b0; 
        cpu_read_data = 32'b0;
        way_update    = 2'b00;
        m_axi_arvalid = 1'b0;
        m_axi_araddr  = {tag, index, 3'b000}; 
        m_axi_rready  = 1'b0;
        pb_write_en   = 1'b0;
        pb_promote_en = 1'b0;

        // --- LOGIC HIT TOÀN CẦU ---
        if (cpu_read_req) begin
            if (valid1[index] && tag1_out == tag) begin //hit
                icache_hit    = 1'b1;
                cpu_read_data = (word_offset == 1'b0) ? data1_out[31:0] : data1_out[63:32]; // lấy nửa nào để mang lên cpu
            end else if (valid2[index] && tag2_out == tag) begin
                icache_hit    = 1'b1;
                cpu_read_data = (word_offset == 1'b0) ? data2_out[31:0] : data2_out[63:32];
            end else if (pb_valid && pb_addr == current_addr[31:3]) begin   // hit nhưng ở buffer
                icache_hit    = 1'b1;
                cpu_read_data = (word_offset == 1'b0) ? pb_data[31:0] : pb_data[63:32];
            end else begin
                icache_stall  = 1'b1; // Ép Stall nếu Miss cả 2 nơi
            end
        end

        // --- ĐIỀU KHIỂN CHUYỂN TRẠNG THÁI FSM ---
        case (state)
            IDLE: begin
                if (cpu_read_req) begin
                    if (icache_hit) begin
                        // Nếu Hit là do Prefetch Buffer -> mang lên cache
                        if (pb_valid && pb_addr == current_addr[31:3]) begin
                            pb_promote_en = 1'b1;
                            if (!valid1[index])      way_update[0] = 1'b1;
                            else if (!valid2[index]) way_update[1] = 1'b1;
                            else if (plru[index] == 1'b0) way_update[0] = 1'b1;
                            else                     way_update[1] = 1'b1;
                            
                            if (!dcache_busy && !flush && prefetch_allowed) next_state = PF_AR_REQ;
                        end
                    end else begin
                        next_state = AR_REQ;
                    end
                end
            end

            AR_REQ: begin
                m_axi_arvalid = 1'b1;
                if (m_axi_arready) next_state = R_WAIT_1;
            end

            R_WAIT_1: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = R_WAIT_2;
            end

            R_WAIT_2: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid && m_axi_rlast) next_state = UPDATE_RAM;
            end
            
            UPDATE_RAM: begin
                if (!fetch_error) begin
                    if (!valid1[index])      way_update[0] = 1'b1;
                    else if (!valid2[index]) way_update[1] = 1'b1;
                    else if (plru[index] == 1'b0) way_update[0] = 1'b1;
                    else                     way_update[1] = 1'b1;
                end
                
                next_state = (!fetch_error && !dcache_busy && !flush && !pb_valid && prefetch_allowed) ? PF_AR_REQ : IDLE;
            end

            PF_AR_REQ: begin
                m_axi_arvalid = 1'b1;
                m_axi_araddr  = prefetch_addr;
                if (m_axi_arready) next_state = PF_R_WAIT_1;
            end

            PF_R_WAIT_1: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = PF_R_WAIT_2;
            end

            PF_R_WAIT_2: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid && m_axi_rlast) next_state = PF_DONE;
            end

            PF_DONE: begin
                if (!flush && !fetch_error) pb_write_en = 1'b1;
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
endmodule