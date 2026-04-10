`timescale 1ns / 1ps

// ... (Giả định bạn đã nhúng generic_tag_ram và generic_data_ram tương tự I-Cache) ...

// =============================================================================
// MAIN MODULE: Data Cache 
// =============================================================================
module data_cache #(
    parameter C_CACHE_SIZE       = 1024,
    parameter C_BLOCK_SIZE       = 16,
    parameter C_WAYS             = 4,
    parameter C_M_AXI_ID_W       = 5,
    parameter C_M_AXI_ADDR_W     = 32,
    parameter C_M_AXI_DATA_W     = 32
)(
    input  wire                          clk, 
    input  wire                          rst_n,          
    
    input  wire                          cpu_read_req, 
    input  wire                          cpu_write_req,
    input  wire [C_M_AXI_ADDR_W-1:0]     cpu_addr,       
    input  wire [C_M_AXI_DATA_W-1:0]     cpu_write_data,
    input  wire                          mem_unsigned, 
    input  wire [1:0]                    mem_size,
    input  wire                          uncache_en,
    output reg  [C_M_AXI_DATA_W-1:0]     cpu_read_data,
    output reg                           dcache_hit, 
    output reg                           dcache_stall,
    
    output wire [C_M_AXI_ID_W-1:0]       m_axi_awid,
    output reg  [C_M_AXI_ADDR_W-1:0]     m_axi_awaddr,
    output wire [7:0]                    m_axi_awlen,
    output wire [2:0]                    m_axi_awsize,
    output wire [1:0]                    m_axi_awburst,
    output wire                          m_axi_awlock,
    output wire [3:0]                    m_axi_awcache,
    output wire [2:0]                    m_axi_awprot,
    output wire [3:0]                    m_axi_awqos,
    output wire [3:0]                    m_axi_awregion,
    output reg                           m_axi_awvalid,
    input  wire                          m_axi_awready,
    output reg  [C_M_AXI_DATA_W-1:0]     m_axi_wdata,
    output reg  [(C_M_AXI_DATA_W/8)-1:0] m_axi_wstrb,
    output reg                           m_axi_wlast,
    output reg                           m_axi_wvalid,
    input  wire                          m_axi_wready,
    input  wire [C_M_AXI_ID_W-1:0]       m_axi_bid,
    input  wire [1:0]                    m_axi_bresp,
    input  wire                          m_axi_bvalid,
    output reg                           m_axi_bready,
    output wire [C_M_AXI_ID_W-1:0]       m_axi_arid,
    output reg  [C_M_AXI_ADDR_W-1:0]     m_axi_araddr,
    output reg  [7:0]                    m_axi_arlen,
    output wire [2:0]                    m_axi_arsize,
    output wire [1:0]                    m_axi_arburst,
    output wire                          m_axi_arlock,
    output wire [3:0]                    m_axi_arcache,
    output wire [2:0]                    m_axi_arprot,
    output wire [3:0]                    m_axi_arqos,
    output wire [3:0]                    m_axi_arregion,
    output reg                           m_axi_arvalid,
    input  wire                          m_axi_arready,
    input  wire [C_M_AXI_ID_W-1:0]       m_axi_rid,
    input  wire [C_M_AXI_DATA_W-1:0]     m_axi_rdata,
    input  wire [1:0]                    m_axi_rresp,
    input  wire                          m_axi_rlast,
    input  wire                          m_axi_rvalid,
    output reg                           m_axi_rready
);

    localparam BLOCK_W      = C_BLOCK_SIZE * 8;
    localparam OFFSET_W     = $clog2(C_BLOCK_SIZE);
    localparam NUM_SETS     = C_CACHE_SIZE / (C_BLOCK_SIZE * C_WAYS);
    localparam INDEX_W      = $clog2(NUM_SETS);
    localparam TAG_W        = C_M_AXI_ADDR_W - INDEX_W - OFFSET_W;
    localparam BURST_LEN    = (BLOCK_W / C_M_AXI_DATA_W) - 1;

    function [31:0] read_data_with_size;
        input [31:0] data; input [1:0] size; input [1:0] offset; input unsigned_flag;
        reg [31:0] res;
        begin
            case (size)
                2'b10: res = data;
                2'b01: res = (offset[1] == 0) ? (unsigned_flag ? {16'b0, data[15:0]} : {{16{data[15]}}, data[15:0]}) : (unsigned_flag ? {16'b0, data[31:16]} : {{16{data[31]}}, data[31:16]});
                2'b00: begin
                    case (offset)
                        2'b00: res = unsigned_flag ? {24'b0, data[7:0]}   : {{24{data[7]}}, data[7:0]};
                        2'b01: res = unsigned_flag ? {24'b0, data[15:8]}  : {{24{data[15]}}, data[15:8]};
                        2'b10: res = unsigned_flag ? {24'b0, data[23:16]} : {{24{data[23]}}, data[23:16]};
                        2'b11: res = unsigned_flag ? {24'b0, data[31:24]} : {{24{data[31]}}, data[31:24]};
                    endcase
                end
                default: res = data;
            endcase
            read_data_with_size = res;
        end
    endfunction

    function [31:0] write_data_with_size;
        input [31:0] orig_data; input [31:0] w_data; input [1:0] size; input [1:0] offset;
        reg [31:0] res;
        begin
            res = orig_data;
            case (size)
                2'b10: res = w_data;
                2'b01: if (offset[1] == 0) res[15:0] = w_data[15:0]; else res[31:16] = w_data[15:0];
                2'b00: case (offset)
                        2'b00: res[7:0]   = w_data[7:0];
                        2'b01: res[15:8]  = w_data[7:0];
                        2'b10: res[23:16] = w_data[7:0];
                        2'b11: res[31:24] = w_data[7:0];
                    endcase
            endcase
            write_data_with_size = res;
        end
    endfunction

    function [3:0] gen_wstrb;
        input [1:0] size; input [1:0] offset;
        reg [3:0] strb;
        begin
            case (size)
                2'b10: strb = 4'b1111;
                2'b01: strb = (offset[1] == 0) ? 4'b0011 : 4'b1100;
                2'b00: strb = (4'b0001 << offset);
                default: strb = 4'b1111;
            endcase
            gen_wstrb = strb;
        end
    endfunction

    assign m_axi_awid = 0; assign m_axi_awsize = mem_size; assign m_axi_awburst = 2'b01; 
    assign m_axi_awlock = 0; assign m_axi_awcache = uncache_en ? 4'b0000 : 4'b0011;
    assign m_axi_awprot = 3'b000; assign m_axi_awqos = 0; assign m_axi_awregion = 0; assign m_axi_awlen = 0; 
    assign m_axi_arid = 0; assign m_axi_arsize = $clog2(C_M_AXI_DATA_W/8); assign m_axi_arburst = 2'b01;
    assign m_axi_arlock = 0; assign m_axi_arcache = uncache_en ? 4'b0000 : 4'b0011;
    assign m_axi_arprot = 3'b000; assign m_axi_arqos = 0; assign m_axi_arregion = 0;

    localparam IDLE   = 3'd0,
               AR_REQ = 3'd1,
               R_WAIT = 3'd2,
               AW_REQ = 3'd3,
               W_REQ  = 3'd4,
               B_WAIT = 3'd5,
               DONE   = 3'd6;
               
    reg [2:0] state, next_state;
    reg [C_WAYS-1:0] valid_arr [0:NUM_SETS-1];
    reg [$clog2(C_WAYS)-1:0] rr_ptr [0:NUM_SETS-1];

    wire [(C_WAYS*BLOCK_W)-1:0] data_out_bus;
    wire [(C_WAYS*TAG_W)-1:0]   tag_out_bus;
    reg [C_WAYS-1:0]  way_write_en;
    reg [BLOCK_W-1:0] write_cache_block;
    reg [BLOCK_W-1:0] fetch_buffer;
    reg [C_M_AXI_ADDR_W-1:0] req_addr;
    
    wire [C_M_AXI_ADDR_W-1:0] current_addr = (state == IDLE) ? cpu_addr : req_addr;
    wire [TAG_W-1:0]          tag          = current_addr[C_M_AXI_ADDR_W-1 : C_M_AXI_ADDR_W-TAG_W];
    wire [INDEX_W-1:0]        index        = current_addr[OFFSET_W+INDEX_W-1 : OFFSET_W];
    wire [OFFSET_W-1:0]       offset       = current_addr[OFFSET_W-1 : 0];
    wire [1:0]                byte_offset  = offset[1:0];
    wire [$clog2(BLOCK_W/C_M_AXI_DATA_W)-1:0] word_idx = offset[OFFSET_W-1 : 2];

    generic_data_ram #(.WAYS(C_WAYS), .INDEX_W(INDEX_W), .BLOCK_W(BLOCK_W)) DATA_RAM (
        .clk(clk), .index(index), .write_en(way_write_en), .write_data(write_cache_block), .data_out(data_out_bus)
    );
    generic_tag_ram #(.WAYS(C_WAYS), .INDEX_W(INDEX_W), .TAG_W(TAG_W)) TAG_RAM (
        .clk(clk), .index(index), .write_en(way_write_en), .write_tag(tag), .tag_out(tag_out_bus)
    );

    integer i, w;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; req_addr <= 0;
            for (i=0; i<NUM_SETS; i=i+1) begin valid_arr[i]<=0; rr_ptr[i]<=0; end
        end else begin
            state <= next_state;
            if (state == IDLE && (cpu_read_req || cpu_write_req)) req_addr <= cpu_addr;
            
            if (state == R_WAIT && m_axi_rvalid && m_axi_rready) begin
                if (uncache_en) fetch_buffer[C_M_AXI_DATA_W-1:0] <= m_axi_rdata;
                else fetch_buffer <= {m_axi_rdata, fetch_buffer[BLOCK_W-1:C_M_AXI_DATA_W]};
            end

            if (state == DONE && !uncache_en && cpu_read_req) begin
                for (w=0; w<C_WAYS; w=w+1) if (way_write_en[w]) valid_arr[index][w] <= 1'b1;
                rr_ptr[index] <= rr_ptr[index] + 1;
            end
        end
    end

    reg hit_flag; reg [$clog2(C_WAYS)-1:0] hit_way; reg [C_M_AXI_DATA_W-1:0] read_word; reg [BLOCK_W-1:0] hit_block;

    always @(*) begin
        next_state    = state; dcache_hit    = 1'b0; dcache_stall  = 1'b0;
        cpu_read_data = 0; way_write_en  = 0; write_cache_block = 0;
        
        m_axi_awvalid = 0; m_axi_wvalid = 0; m_axi_bready = 0; m_axi_arvalid = 0; m_axi_rready = 0;
        m_axi_awaddr  = current_addr; m_axi_wdata = cpu_write_data;
        m_axi_wstrb   = gen_wstrb(mem_size, byte_offset); m_axi_wlast = 1'b1;

        if (uncache_en) begin
            m_axi_arlen = 8'd0; m_axi_araddr = current_addr;
        end else begin
            m_axi_arlen = BURST_LEN; m_axi_araddr = {tag, index, {OFFSET_W{1'b0}}};
        end

        hit_flag = 1'b0; hit_way = 0; read_word = 0; hit_block = 0;
        for (w=0; w<C_WAYS; w=w+1) begin
            if (!uncache_en && valid_arr[index][w] && tag_out_bus[w*TAG_W +: TAG_W] == tag) begin
                hit_flag = 1'b1; hit_way = w; hit_block = data_out_bus[w*BLOCK_W +: BLOCK_W];
                read_word = hit_block[word_idx*C_M_AXI_DATA_W +: C_M_AXI_DATA_W];
            end
        end

        case (state)
            IDLE: begin
                if (cpu_read_req) begin
                    if (uncache_en) begin
                        dcache_stall = 1'b1; next_state = AR_REQ;
                    end else if (hit_flag) begin
                        dcache_hit = 1'b1; cpu_read_data = read_data_with_size(read_word, mem_size, byte_offset, mem_unsigned);
                    end else begin
                        dcache_stall = 1'b1; next_state = AR_REQ;
                    end
                end 
                else if (cpu_write_req) begin
                    dcache_stall = 1'b1;
                    if (uncache_en) begin
                        next_state = AW_REQ;
                    end else if (hit_flag) begin
                        // Cập nhật RAM ngay trong chu kỳ Hit IDLE
                        way_write_en[hit_way] = 1'b1;
                        write_cache_block = hit_block;
                        write_cache_block[word_idx*C_M_AXI_DATA_W +: C_M_AXI_DATA_W] = write_data_with_size(read_word, cpu_write_data, mem_size, byte_offset);
                        next_state = AW_REQ;
                    end else begin
                        next_state = AW_REQ;
                    end
                end
            end
            AR_REQ: begin
                dcache_stall = 1'b1; m_axi_arvalid = 1'b1;
                if (m_axi_arready) next_state = R_WAIT;
            end
            R_WAIT: begin
                dcache_stall = 1'b1; m_axi_rready = 1'b1;
                if (m_axi_rvalid) begin
                    if (uncache_en) next_state = DONE;
                    else if (m_axi_rlast) next_state = DONE;
                end
            end
            AW_REQ: begin
                dcache_stall = 1'b1; m_axi_awvalid = 1'b1;
                if (m_axi_awready) next_state = W_REQ;
            end
            W_REQ: begin
                dcache_stall = 1'b1; m_axi_wvalid = 1'b1;
                if (m_axi_wready) next_state = B_WAIT;
            end
            B_WAIT: begin
                dcache_stall = 1'b1; m_axi_bready = 1'b1;
                if (m_axi_bvalid) next_state = DONE;
            end
            DONE: begin
                // 1. LUÔN CẬP NHẬT CACHE RAM NẾU LÀ READ MISS (Không Uncache)
                // Kể cả khi CPU đã Flush đổi địa chỉ, ta vẫn lưu data này lại để dùng sau.
                if (cpu_read_req && !uncache_en) begin
                    way_write_en[rr_ptr[index]] = 1'b1;
                    write_cache_block = fetch_buffer;
                end

                // 2. RÀNG BUỘC TÍN HIỆU TRẢ VỀ CPU BẰNG ĐỊA CHỈ
                if (cpu_addr == req_addr) begin
                    // Nhả stall và báo Hit vì lệnh vẫn còn nguyên (không bị Flush)
                    dcache_stall = 1'b0;
                    dcache_hit   = 1'b1;
                    
                    if (cpu_read_req) begin
                        if (uncache_en) begin
                            cpu_read_data = read_data_with_size(fetch_buffer[C_M_AXI_DATA_W-1:0], mem_size, byte_offset, mem_unsigned);
                        end else begin
                            cpu_read_data = read_data_with_size(fetch_buffer[word_idx*C_M_AXI_DATA_W +: C_M_AXI_DATA_W], mem_size, byte_offset, mem_unsigned);
                        end
                    end
                end else begin
                    // Lệnh đã bị Flush sang địa chỉ khác. Giữ stall để FSM quay về IDLE xử lý an toàn.
                    dcache_stall  = 1'b1;
                    dcache_hit    = 1'b0;
                    cpu_read_data = {C_M_AXI_DATA_W{1'b0}};
                end

                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end
endmodule