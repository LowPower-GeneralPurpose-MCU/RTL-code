`timescale 1ns / 1ps

// =============================================================================
// SUB-MODULES (Tag RAM & Data RAM)
// =============================================================================
module generic_tag_ram #(
    parameter WAYS       = 2,
    parameter INDEX_W    = 4,
    parameter TAG_W      = 25
)(
    input  wire                       clk,
    input  wire [INDEX_W-1:0]         index,
    input  wire [WAYS-1:0]            write_en,
    input  wire [TAG_W-1:0]           write_tag,
    output wire [(WAYS*TAG_W)-1:0]    tag_out
);
    genvar i;
    generate
        for (i = 0; i < WAYS; i = i + 1) begin : tag_ways
            (* ram_style = "distributed" *) reg [TAG_W-1:0] ram [0:(1<<INDEX_W)-1];
            assign tag_out[(i+1)*TAG_W-1 : i*TAG_W] = ram[index];
            always @(posedge clk) begin
                if (write_en[i]) ram[index] <= write_tag;
            end
        end
    endgenerate
endmodule

module generic_data_ram #(
    parameter WAYS       = 2,
    parameter INDEX_W    = 4,
    parameter BLOCK_W    = 64
)(
    input  wire                       clk,
    input  wire [INDEX_W-1:0]         index,
    input  wire [WAYS-1:0]            write_en,
    input  wire [BLOCK_W-1:0]         write_data,
    output wire [(WAYS*BLOCK_W)-1:0]  data_out
);
    genvar i;
    generate
        for (i = 0; i < WAYS; i = i + 1) begin : data_ways
            (* ram_style = "distributed" *) reg [BLOCK_W-1:0] ram [0:(1<<INDEX_W)-1];
            assign data_out[(i+1)*BLOCK_W-1 : i*BLOCK_W] = ram[index];
            always @(posedge clk) begin
                if (write_en[i]) ram[index] <= write_data;
            end
        end
    endgenerate
endmodule

// =============================================================================
// MAIN MODULE: Instruction Cache
// =============================================================================
module instruction_cache #(
    parameter C_CACHE_SIZE       = 1024,
    parameter C_BLOCK_SIZE       = 16,
    parameter C_WAYS             = 2,
    parameter C_M_AXI_ID_W       = 5,
    parameter C_M_AXI_ADDR_W     = 32,
    parameter C_M_AXI_DATA_W     = 32
)(
    input  wire                          clk,
    input  wire                          rst_n,          
    
    input  wire                          cpu_read_req,
    input  wire [C_M_AXI_ADDR_W-1:0]     cpu_addr,
    input  wire                          uncache_en,
    output reg  [C_M_AXI_DATA_W-1:0]     cpu_read_data,
    output reg                           icache_hit,
    output reg                           icache_stall,
    
    output wire [C_M_AXI_ID_W-1:0]       m_axi_awid,
    output wire [C_M_AXI_ADDR_W-1:0]     m_axi_awaddr,
    output wire [7:0]                    m_axi_awlen,
    output wire [2:0]                    m_axi_awsize,
    output wire [1:0]                    m_axi_awburst,
    output wire                          m_axi_awlock,
    output wire [3:0]                    m_axi_awcache,
    output wire [2:0]                    m_axi_awprot,
    output wire [3:0]                    m_axi_awqos,
    output wire [3:0]                    m_axi_awregion,
    output wire                          m_axi_awvalid,
    input  wire                          m_axi_awready,
    output wire [C_M_AXI_DATA_W-1:0]     m_axi_wdata,
    output wire [(C_M_AXI_DATA_W/8)-1:0] m_axi_wstrb,
    output wire                          m_axi_wlast,
    output wire                          m_axi_wvalid,
    input  wire                          m_axi_wready,
    input  wire [C_M_AXI_ID_W-1:0]       m_axi_bid,
    input  wire [1:0]                    m_axi_bresp,
    input  wire                          m_axi_bvalid,
    output wire                          m_axi_bready,
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

    assign m_axi_awid = 0; assign m_axi_awaddr = 0; assign m_axi_awlen = 0;
    assign m_axi_awsize = 0; assign m_axi_awburst = 0; assign m_axi_awlock = 0;
    assign m_axi_awcache = 0; assign m_axi_awprot = 0; assign m_axi_awqos = 0;
    assign m_axi_awregion = 0; assign m_axi_awvalid = 0;
    assign m_axi_wdata = 0; assign m_axi_wstrb = 0; assign m_axi_wlast = 0; 
    assign m_axi_wvalid = 0; assign m_axi_bready = 1'b1;
    
    assign m_axi_arid = 0; assign m_axi_arsize = $clog2(C_M_AXI_DATA_W/8); 
    assign m_axi_arburst = 2'b01; assign m_axi_arlock = 1'b0;
    assign m_axi_arcache = uncache_en ? 4'b0000 : 4'b0011;
    assign m_axi_arprot = 3'b100; assign m_axi_arqos = 4'b0000; assign m_axi_arregion = 4'b0000;

    wire [(C_WAYS*BLOCK_W)-1:0] data_out_bus;
    wire [(C_WAYS*TAG_W)-1:0]   tag_out_bus;
    reg  [C_WAYS-1:0]           valid_arr [0:NUM_SETS-1];
    reg  [$clog2(C_WAYS)-1:0]   rr_ptr    [0:NUM_SETS-1];
    
    localparam IDLE   = 3'd0,
               AR_REQ = 3'd1,
               R_WAIT = 3'd2,
               DONE   = 3'd3;
               
    reg [2:0]  state, next_state;
    reg [C_WAYS-1:0] way_update;
    reg [BLOCK_W-1:0] fetch_buffer;
    reg [C_M_AXI_ADDR_W-1:0] miss_addr;
    
    wire [C_M_AXI_ADDR_W-1:0] current_addr = (state == IDLE) ? cpu_addr : miss_addr;
    wire [TAG_W-1:0]          tag          = current_addr[C_M_AXI_ADDR_W-1 : C_M_AXI_ADDR_W-TAG_W];
    wire [INDEX_W-1:0]        index        = current_addr[OFFSET_W+INDEX_W-1 : OFFSET_W];
    wire [OFFSET_W-1:0]       offset       = current_addr[OFFSET_W-1 : 0];
    wire [$clog2(BLOCK_W/C_M_AXI_DATA_W)-1:0] word_idx = offset[OFFSET_W-1 : $clog2(C_M_AXI_DATA_W/8)];

    generic_data_ram #(.WAYS(C_WAYS), .INDEX_W(INDEX_W), .BLOCK_W(BLOCK_W)) DATA_RAM (
        .clk(clk), .index(index), .write_en(way_update), .write_data(fetch_buffer), .data_out(data_out_bus)
    );
    generic_tag_ram #(.WAYS(C_WAYS), .INDEX_W(INDEX_W), .TAG_W(TAG_W)) TAG_RAM (
        .clk(clk), .index(index), .write_en(way_update), .write_tag(tag), .tag_out(tag_out_bus)
    );

    integer i, w;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; fetch_buffer <= 0; miss_addr <= 0;
            for (i=0; i<NUM_SETS; i=i+1) begin valid_arr[i]<=0; rr_ptr[i]<=0; end
        end else begin
            state <= next_state;
            if (state == IDLE && cpu_read_req && !icache_hit) miss_addr <= cpu_addr;

            if (state == R_WAIT && m_axi_rvalid && m_axi_rready) begin
                if (uncache_en) fetch_buffer[C_M_AXI_DATA_W-1:0] <= m_axi_rdata;
                else fetch_buffer <= {m_axi_rdata, fetch_buffer[BLOCK_W-1:C_M_AXI_DATA_W]};
            end

            if (state == DONE && !uncache_en) begin
                for (w=0; w<C_WAYS; w=w+1) if (way_update[w]) valid_arr[index][w] <= 1'b1;
                rr_ptr[index] <= rr_ptr[index] + 1;
            end
        end
    end

    reg hit_flag;
    reg [C_M_AXI_DATA_W-1:0] read_word;

    always @(*) begin
        next_state    = state;
        icache_hit    = 1'b0;
        icache_stall  = 1'b0; 
        cpu_read_data = 0;
        way_update    = 0;
        m_axi_arvalid = 1'b0;
        m_axi_rready  = 1'b0;
        
        if (uncache_en) begin
            m_axi_arlen  = 8'd0; 
            m_axi_araddr = current_addr;
        end else begin
            m_axi_arlen  = BURST_LEN; 
            m_axi_araddr = {tag, index, {OFFSET_W{1'b0}}};
        end

        hit_flag = 1'b0; read_word = 0;
        for (w=0; w<C_WAYS; w=w+1) begin
            if (!uncache_en && valid_arr[index][w] && tag_out_bus[w*TAG_W +: TAG_W] == tag) begin
                hit_flag = 1'b1;
                read_word = data_out_bus[(w*BLOCK_W) + (word_idx*C_M_AXI_DATA_W) +: C_M_AXI_DATA_W];
            end
        end

        case (state)
            IDLE: begin
                if (cpu_read_req) begin
                    if (uncache_en) begin
                        icache_stall = 1'b1; next_state = AR_REQ;
                    end else if (hit_flag) begin
                        icache_hit = 1'b1; cpu_read_data = read_word;
                    end else begin
                        icache_stall = 1'b1; next_state = AR_REQ;
                    end
                end
            end
            AR_REQ: begin
                icache_stall = 1'b1; m_axi_arvalid = 1'b1;
                if (m_axi_arready) next_state = R_WAIT;
            end
            R_WAIT: begin
                icache_stall = 1'b1; m_axi_rready = 1'b1;
                if (m_axi_rvalid && m_axi_rlast) next_state = DONE;
            end
            DONE: begin
                // Kiểm tra xem CPU có vừa bị flush nhảy PC đi nơi khác không
                if (cpu_addr == miss_addr) begin
                    icache_stall = 1'b0;
                    icache_hit   = 1'b1;
                    cpu_read_data = uncache_en ?
                        fetch_buffer[C_M_AXI_DATA_W-1:0] : fetch_buffer[word_idx*C_M_AXI_DATA_W +: C_M_AXI_DATA_W];
                end else begin
                    // Nếu PC đã thay đổi, HỦY việc trả lệnh này cho CPU (giữ stall để cycle sau IDLE xử lý PC mới)
                    icache_stall = 1'b1;
                    icache_hit   = 1'b0;
                    cpu_read_data = {C_M_AXI_DATA_W{1'b0}}; 
                end
                
                // Vẫn ghi dữ liệu vừa fetch vào SRAM để tận dụng (vì đằng nào cũng tốn công fetch)
                if (!uncache_en) way_update[rr_ptr[index]] = 1'b1; 
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end
endmodule