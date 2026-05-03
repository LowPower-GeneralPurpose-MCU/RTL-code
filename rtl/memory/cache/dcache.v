`timescale 1ns / 1ps

// =============================================================================
// SUB-MODULE: DCache Tag RAM (Lưu trữ 4-Way Tags) - 26 bit Tag cho địa chỉ 32 bit
// =============================================================================
module dcache_tag_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [3:0]  write_en, // One-hot: [0]=Way1 ... [3]=Way4 ( chọn way để ghi)
    input  wire [25:0] write_tag,
    output wire [25:0] t1, 
    output wire [25:0] t2, 
    output wire [25:0] t3, 
    output wire [25:0] t4
);
    (* ram_style = "distributed" *) reg [25:0] tag1 [0:15];
    (* ram_style = "distributed" *) reg [25:0] tag2 [0:15];
    (* ram_style = "distributed" *) reg [25:0] tag3 [0:15];
    (* ram_style = "distributed" *) reg [25:0] tag4 [0:15];

    assign t1 = tag1[index]; 
    assign t2 = tag2[index];
    assign t3 = tag3[index]; 
    assign t4 = tag4[index];

    always @(posedge clk) begin
        if (write_en[0]) tag1[index] <= write_tag;
        if (write_en[1]) tag2[index] <= write_tag;
        if (write_en[2]) tag3[index] <= write_tag;
        if (write_en[3]) tag4[index] <= write_tag;
    end
endmodule

// =============================================================================
// SUB-MODULE: DCache Data RAM (Lưu trữ 4-Way Data - 32 bit / Block)
// =============================================================================
module dcache_data_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [3:0]  write_en,
    input  wire [31:0] write_data,
    output wire [31:0] d1, 
    output wire [31:0] d2, 
    output wire [31:0] d3, 
    output wire [31:0] d4
);
    (* ram_style = "distributed" *) reg [31:0] data1 [0:15];
    (* ram_style = "distributed" *) reg [31:0] data2 [0:15];
    (* ram_style = "distributed" *) reg [31:0] data3 [0:15];
    (* ram_style = "distributed" *) reg [31:0] data4 [0:15];

    assign d1 = data1[index]; 
    assign d2 = data2[index];
    assign d3 = data3[index]; 
    assign d4 = data4[index];

    always @(posedge clk) begin
        if (write_en[0]) data1[index] <= write_data;
        if (write_en[1]) data2[index] <= write_data;
        if (write_en[2]) data3[index] <= write_data;
        if (write_en[3]) data4[index] <= write_data;
    end
endmodule

// =============================================================================
// MAIN MODULE: Data Cache Controller
// =============================================================================
module data_cache (
    input  wire        clk, 
    input  wire        rst_n,          
    input  wire        cpu_read_req, // cpu load 
    input  wire        cpu_write_req, // cpu store 
    input  wire [31:0] cpu_addr,       // địa chỉ load/store
    input  wire [31:0] mem_read_data,  // dữ liệu đọc được
    input  wire [31:0] cpu_write_data,  // dữ liệu store
    input  wire        mem_unsigned, 
    input  wire [1:0]  mem_size,
    input  wire        mem_read_ready,       
    input  wire        mem_read_valid, 
    input  wire        mem_write_ready,      
    input  wire        mem_write_back_valid,
    output reg         mem_read_req, // yêu cầu đọc ram
    output reg         mem_write_req, // yêu cầu ghi vào ram
    output reg  [31:0] mem_addr,       // địa chỉ truy cập ram
    output reg  [31:0] cpu_read_data, // dữ liệu trả cho cpu
    output reg  [31:0] mem_write_data, // dữ liệu muốn ghi xuống ram
    output reg         dcache_hit,  // hit trong cache hoặc write buffer
    output reg         dcache_stall,
    output wire        dcache_bus_busy // dcache đang chiếm ram
);

    // --- CÁC HÀM XỬ LÝ SIZE  ---
    function [31:0] read_data_with_size;
        input [31:0] data; input [1:0] size; input [1:0] offset; input unsigned_flag;
        reg   [31:0] result;
        begin
            case (size)
                2'b10: result = data;
                2'b01: begin
                    if (offset[1] == 1'b0) result = unsigned_flag ? {16'b0, data[15:0]} : {{16{data[15]}}, data[15:0]};
                    else                   result = unsigned_flag ? {16'b0, data[31:16]} : {{16{data[31]}}, data[31:16]};
                end
                2'b00: begin
                    case (offset)
                        2'b00: result = unsigned_flag ? {24'b0, data[7:0]}   : {{24{data[7]}}, data[7:0]};
                        2'b01: result = unsigned_flag ? {24'b0, data[15:8]}  : {{24{data[15]}}, data[15:8]};
                        2'b10: result = unsigned_flag ? {24'b0, data[23:16]} : {{24{data[23]}}, data[23:16]};
                        2'b11: result = unsigned_flag ? {24'b0, data[31:24]} : {{24{data[31]}}, data[31:24]};
                    endcase
                end
                default: result = data;
            endcase
            read_data_with_size = result;
        end
    endfunction

    function [31:0] write_data_with_size;
        input [31:0] original_data; input [31:0] write_data; input [1:0] size; input [1:0] offset;
        reg   [31:0] result;
        begin
            result = original_data;
            case (size)
                2'b10: result = write_data;
                2'b01: begin
                    if (offset[1] == 1'b0) result[15:0]  = write_data[15:0];
                    else                   result[31:16] = write_data[15:0];
                end
                2'b00: begin
                    case (offset)
                        2'b00: result[7:0]   = write_data[7:0];
                        2'b01: result[15:8]  = write_data[7:0];
                        2'b10: result[23:16] = write_data[7:0];
                        2'b11: result[31:24] = write_data[7:0];
                    endcase
                end
            endcase
            write_data_with_size = result;
        end
    endfunction

    function [1:0] select_replacement_way;
        input [2:0] plru_bits; reg [1:0] way;
        begin
            if (plru_bits[0] == 1'b0) way = (plru_bits[1] == 1'b0) ? 2'b00 : 2'b01;
            else                      way = (plru_bits[2] == 1'b0) ? 2'b10 : 2'b11;
            select_replacement_way = way;
        end
    endfunction

    function [2:0] update_plru;
        input [2:0] old_plru; input [1:0] accessed_way; reg [2:0] new_plru;
        begin
            new_plru = old_plru;
            case (accessed_way)
                2'b00: begin new_plru[0] = 1'b1; new_plru[1] = 1'b1; end
                2'b01: begin new_plru[0] = 1'b1; new_plru[1] = 1'b0; end
                2'b10: begin new_plru[0] = 1'b0; new_plru[2] = 1'b1; end
                2'b11: begin new_plru[0] = 1'b0; new_plru[2] = 1'b0; end
            endcase
            update_plru = new_plru;
        end
    endfunction

    // --- PHÂN TÍCH ĐỊA CHỈ & BIẾN NỘI BỘ ---
    wire [25:0] tag         = cpu_addr[31:6];
    wire [3:0]  index       = cpu_addr[5:2];
    wire [1:0]  byte_offset = cpu_addr[1:0];

    reg valid1[0:15], valid2[0:15], valid3[0:15], valid4[0:15];
    reg [2:0] plru[0:15];

    wire [25:0] t1, t2, t3, t4;
    wire [31:0] d1, d2, d3, d4;

    parameter IDLE = 2'b00, MEM_READ = 2'b01;
    reg [1:0] state, next_state;

    wire cache_lookup_en = (state == IDLE); 

    wire cache_hit_way0 = cache_lookup_en && valid1[index] && (t1 == tag);
    wire cache_hit_way1 = cache_lookup_en && valid2[index] && (t2 == tag);
    wire cache_hit_way2 = cache_lookup_en && valid3[index] && (t3 == tag);
    wire cache_hit_way3 = cache_lookup_en && valid4[index] && (t4 == tag);

    wire cache_array_hit = cache_hit_way0 | cache_hit_way1 | cache_hit_way2 | cache_hit_way3;

    // các tín hiệu khi miss ( các tín hiệu cho hàm read/write data with size)
    reg        miss_is_write; // miss của store hay load
    reg [31:0] miss_addr; //địa chỉ miss
    reg [31:0] miss_wdata;  // dữ liệu cần ghi ( nếu là miss store )
    reg [1:0]  miss_size; 
    reg        miss_unsigned;
    reg [1:0]  miss_target_way; // way để thay sau khi lấy

    wire [25:0] miss_tag         = miss_addr[31:6];
    wire [3:0]  miss_index       = miss_addr[5:2];
    wire [1:0]  miss_byte_offset = miss_addr[1:0];

    wire [3:0]  cache_index     = (state == IDLE) ? index : miss_index;
    wire [25:0] cache_write_tag = (state == IDLE) ? tag : miss_tag;

    reg [1:0]  target_way;
    reg [3:0]  way_write_en;
    reg [31:0] final_write_data;
    reg        req_sent; // chống gửi lặp yêu cầu đọc trong state MEM_READ.

    // =========================================================================
    // CONSUMER FSM — Đẩy dữ liệu  từ buffer xuống ram
    // =========================================================================
    parameter CONS_IDLE = 1'b0, CONS_WRITE = 1'b1;
    reg        cons_state, cons_next_state;
    reg        cons_req_sent;   // AXI handshake tracker riêng cho Consumer
    
    wire main_using_bus = (state == MEM_READ);
    wire cons_using_bus = (cons_state == CONS_WRITE);
    assign dcache_bus_busy = main_using_bus || cons_using_bus;

    // khởi tạo
    dcache_tag_ram TAGS (.clk(clk), .index(cache_index), .write_en(way_write_en), .write_tag(cache_write_tag), .t1(t1), .t2(t2), .t3(t3), .t4(t4));
    dcache_data_ram DATA (.clk(clk), .index(cache_index), .write_en(way_write_en), .write_data(final_write_data), .d1(d1), .d2(d2), .d3(d3), .d4(d4));

    // =========================================================================
    // WRITE BUFFER — dữ liệu cpu muốn ghi vào ram thì ghi vào đây để nó tự ghi xuống, cpu ko cần chờ
    // =========================================================================
    reg [63:0] wbuf_mem [0:3]; // 4 entry 64 bit
    reg [2:0]  wbuf_wptr, wbuf_rptr; // 3 bit, bit đầu là vòng, 2 bit sau là entry
    reg [3:0]  wbuf_valid;       // này để check valid lúc CPU miss mà nó đọc ở trong cái buffer này
    
    wire wbuf_full  = (wbuf_wptr[2] != wbuf_rptr[2]) && 
                      (wbuf_wptr[1:0] == wbuf_rptr[1:0]);
    wire wbuf_empty = (wbuf_wptr == wbuf_rptr);
    
    reg         wbuf_push; // enable ghi xuống
    reg         wbuf_pop; // flag để báo đã nạp vào ram xong
    reg  [63:0] wbuf_wdata;
    wire [63:0] wbuf_rdata = wbuf_mem[wbuf_rptr[1:0]];
    wire [31:0] wbuf_out_addr = wbuf_rdata[63:32];
    wire [31:0] wbuf_out_data = wbuf_rdata[31:0];
    
    wire cpu_read_miss  = (state == IDLE) && cpu_read_req  && !wbuf_fwd_hit && !cache_array_hit;
    wire cpu_write_miss = (state == IDLE) && cpu_write_req && !cache_array_hit && !wbuf_fwd_hit && (mem_size != 2'b10) && !wbuf_full;
    wire pending_cpu_miss = cpu_read_miss || cpu_write_miss;

    // --- Tính next_wbuf_valid - khi nào push thì ghi 1 pop ra thì 0 ---
    reg [3:0] next_wbuf_valid;
    always @(*) begin
        next_wbuf_valid = wbuf_valid;
        if (wbuf_push && !wbuf_full)
            next_wbuf_valid[wbuf_wptr[1:0]] = 1'b1;
        if (wbuf_pop && !wbuf_empty)
            next_wbuf_valid[wbuf_rptr[1:0]] = 1'b0;
    end

    // --- quét qua buffer xem dữ liệu cần có trong đây ko---
    wire [29:0] cpu_word_addr = cpu_addr[31:2];
    reg        wbuf_fwd_hit;
    reg [31:0] wbuf_fwd_data;
    integer k;
    reg [2:0] scan_ptr;
    reg [1:0] scan_idx;
    reg [29:0] scan_addr;
    always @(*) begin
        wbuf_fwd_hit  = 1'b0;
        wbuf_fwd_data = 32'b0;

        for (k = 0; k < 4; k = k + 1) begin
            scan_ptr  = wbuf_wptr - 3'd1 - k;
            scan_idx  = scan_ptr[1:0];
            scan_addr = wbuf_mem[scan_idx][63:34];

            if (!wbuf_fwd_hit && wbuf_valid[scan_idx] && (scan_addr == cpu_word_addr)) begin
                wbuf_fwd_hit  = 1'b1;
                wbuf_fwd_data = wbuf_mem[scan_idx][31:0];
            end
        end
    end

    // Ghi vào buffer / xóa khỏi buffer
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wbuf_wptr  <= 3'b000;
            wbuf_rptr  <= 3'b000;
            wbuf_valid <= 4'b0000;
        end else begin
            wbuf_valid <= next_wbuf_valid;                
            if (wbuf_push && !wbuf_full) begin
                wbuf_mem[wbuf_wptr[1:0]] <= wbuf_wdata;
                wbuf_wptr <= wbuf_wptr + 1;
            end
            if (wbuf_pop && !wbuf_empty) begin
                wbuf_rptr <= wbuf_rptr + 1;
            end
        end
    end

    // =========================================================================
    // CONSUMER FSM — Sequential Logic
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cons_state    <= CONS_IDLE;
            cons_req_sent <= 1'b0;
        end else begin
            cons_state <= cons_next_state;   
            if (cons_state == CONS_IDLE) begin
                cons_req_sent <= 1'b0;
            end else if (cons_state == CONS_WRITE) begin
                if (mem_write_back_valid) // ghi xong
                    cons_req_sent <= 1'b0;
                else if (mem_write_ready && mem_write_req)
                    cons_req_sent <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // LOGIC CHUYỂN TRẠNG THÁI
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            req_sent <= 1'b0;
            miss_is_write <= 1'b0;
            miss_addr     <= 32'b0;
            miss_wdata    <= 32'b0;
            miss_size     <= 2'b10;
            miss_unsigned <= 1'b0;
            miss_target_way <= 2'b00;
            for(i=0; i<16; i=i+1) begin 
                valid1[i]<=1'b0; 
                valid2[i]<=1'b0; 
                valid3[i]<=1'b0; 
                valid4[i]<=1'b0; 
                plru[i]<=3'b000; 
            end
            
        end else begin
            state <= next_state;

            if (state == IDLE) begin
                if (cpu_read_req && !cache_array_hit && !wbuf_fwd_hit && (cons_state == CONS_IDLE)) begin // miss
                    miss_is_write  <= 1'b0;
                    miss_addr      <= cpu_addr;
                    miss_wdata     <= 32'b0;
                    miss_size      <= mem_size;
                    miss_unsigned  <= mem_unsigned;
                    miss_target_way <= target_way;
                end else if (cpu_write_req && !cache_array_hit && !wbuf_fwd_hit && (mem_size != 2'b10) && !wbuf_full && (cons_state == CONS_IDLE)) begin // miss
                    miss_is_write  <= 1'b1;
                    miss_addr      <= cpu_addr;
                    miss_wdata     <= cpu_write_data;
                    miss_size      <= mem_size;
                    miss_unsigned  <= mem_unsigned;
                    miss_target_way <= target_way;
                end
            end

            if (state == IDLE) begin
                req_sent <= 1'b0;      // IDLE không dùng AXI bus
            end else if (state == MEM_READ) begin
                if (mem_read_valid) req_sent <= 1'b0;
                else if (mem_read_ready && mem_read_req) req_sent <= 1'b1;
            end

            // Cập nhật Metadata khi Ghi Cache thành công
            if (way_write_en != 4'b0000) begin
                plru[cache_index] <= update_plru(plru[cache_index], (state == IDLE) ? target_way : miss_target_way);
                if (state == MEM_READ || state == IDLE) begin 
                    case((state == IDLE) ? target_way : miss_target_way)
                        2'd0: valid1[cache_index] <= 1'b1;
                        2'd1: valid2[cache_index] <= 1'b1;
                        2'd2: valid3[cache_index] <= 1'b1;
                        2'd3: valid4[cache_index] <= 1'b1;
                    endcase
                end
            end else if (state == IDLE && cpu_read_req && cache_array_hit) begin
                plru[index] <= update_plru(plru[index], target_way);
            end
        end
    end

    // -------------------------------------------------------------------------
    // LOGIC TỔ HỢP ĐIỀU KHIỂN FSM
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state       = state;
        cons_next_state  = cons_state;      
        target_way       = 2'd0;
        dcache_hit       = 1'b0; 
        dcache_stall     = 1'b0;
        mem_read_req     = 1'b0; 
        mem_write_req    = 1'b0;            
        mem_addr         = 32'b0;          
        cpu_read_data    = 32'b0;
        mem_write_data   = 32'b0;           
        way_write_en     = 4'b0000; 
        final_write_data = 32'b0;
        wbuf_push        = 1'b0;
        wbuf_pop         = 1'b0;            
        wbuf_wdata       = 64'b0;
        
        if      (cache_hit_way0) begin dcache_hit=1'b1; target_way=2'd0; end
        else if (cache_hit_way1) begin dcache_hit=1'b1; target_way=2'd1; end
        else if (cache_hit_way2) begin dcache_hit=1'b1; target_way=2'd2; end
        else if (cache_hit_way3) begin dcache_hit=1'b1; target_way=2'd3; end
        else begin
            dcache_hit = 1'b0;
            if      (!valid1[index]) target_way = 2'd0;
            else if (!valid2[index]) target_way = 2'd1;
            else if (!valid3[index]) target_way = 2'd2;
            else if (!valid4[index]) target_way = 2'd3;
            else                     target_way = select_replacement_way(plru[index]);
        end

        case (state)
            IDLE: begin
                // // BỘ LỌC CDC CHỐNG TREO: Chờ tín hiệu valid cũ hạ hẳn xuống mới làm việc tiếp
                // if ((cpu_read_req || cpu_write_req) && (mem_read_valid || mem_write_back_valid)) begin
                //     dcache_stall = 1'b1;
                //     next_state   = IDLE;
                
                // end else 
                if (cpu_read_req) begin // cpu yêu cầu đọc
                    if (wbuf_fwd_hit) begin // hit ở buffer
                        dcache_hit    = 1'b1;
                        //$display("xac nhan buffer hit");
                        cpu_read_data = read_data_with_size(wbuf_fwd_data, mem_size, byte_offset, mem_unsigned);
                    end else if (cache_array_hit) begin // hit ở trong cache
                        //$display("xac nhan cache hit");
                        dcache_hit = 1'b1;
                        case (target_way)
                            2'd0: cpu_read_data = read_data_with_size(d1, mem_size, byte_offset, mem_unsigned);
                            2'd1: cpu_read_data = read_data_with_size(d2, mem_size, byte_offset, mem_unsigned);
                            2'd2: cpu_read_data = read_data_with_size(d3, mem_size, byte_offset, mem_unsigned);
                            2'd3: cpu_read_data = read_data_with_size(d4, mem_size, byte_offset, mem_unsigned);
                        endcase
                    end else begin
                        dcache_stall = 1'b1;
                        if (cons_state == CONS_IDLE) begin
                            next_state = MEM_READ;
                        end
                    end               
                end else if (cpu_write_req) begin
                    if (cache_array_hit) begin
                        dcache_hit = 1'b1;

                        if (!wbuf_full) begin
                            way_write_en[target_way] = 1'b1;
                            case (target_way)
                                2'd0: final_write_data = write_data_with_size(d1, cpu_write_data, mem_size, byte_offset);
                                2'd1: final_write_data = write_data_with_size(d2, cpu_write_data, mem_size, byte_offset);
                                2'd2: final_write_data = write_data_with_size(d3, cpu_write_data, mem_size, byte_offset);
                                2'd3: final_write_data = write_data_with_size(d4, cpu_write_data, mem_size, byte_offset);
                            endcase

                            // Push merged word vào WBUF để consumer ghi xuống RAM
                            wbuf_push  = 1'b1;
                            wbuf_wdata = {{tag, index, 2'b00}, final_write_data};
                        end else begin
                            dcache_stall = 1'b1;
                        end
                    end else if (wbuf_fwd_hit) begin
                        dcache_hit = 1'b1;

                        if (!wbuf_full) begin
                            way_write_en[target_way] = 1'b1;
                            final_write_data         = write_data_with_size(wbuf_fwd_data, cpu_write_data, mem_size, byte_offset);

                            wbuf_push  = 1'b1;
                            wbuf_wdata = {{tag, index, 2'b00}, final_write_data};
                        end else begin
                            dcache_stall = 1'b1;
                        end
                    end else begin
                        if (mem_size != 2'b10) begin
                            dcache_stall = 1'b1;
                            if (cons_state == CONS_IDLE && !wbuf_full) begin
                                next_state = MEM_READ;
                            end
                        end else begin
                            if (!wbuf_full) begin
                                way_write_en[target_way] = 1'b1;
                                final_write_data = cpu_write_data;
                                wbuf_push = 1'b1;
                                wbuf_wdata = {{tag, index, 2'b00}, cpu_write_data};
                            end else begin
                                dcache_stall = 1'b1;
                            end
                        end
                    end
                end
            end

            MEM_READ: begin
                dcache_stall = 1'b1;
                mem_read_req = !req_sent; 
                mem_addr     = {miss_tag, miss_index, 2'b00};

                if (mem_read_valid) begin
                    mem_read_req = 1'b0;
                    way_write_en = 4'b0000;
                    way_write_en[miss_target_way] = 1'b1;
                    
                    if (miss_is_write) begin
                        final_write_data = write_data_with_size(mem_read_data, miss_wdata, miss_size, miss_byte_offset);
                        wbuf_push  = 1'b1;
                        wbuf_wdata = {{miss_tag, miss_index, 2'b00}, final_write_data};
                        dcache_stall = 1'b0;
                        next_state   = IDLE;
                    end else begin // miss do lệnh read
                        final_write_data = mem_read_data;
                        cpu_read_data    = read_data_with_size(mem_read_data, miss_size, miss_byte_offset, miss_unsigned);
                        dcache_stall     = 1'b0;
                        next_state       = IDLE;
                    end
                end
            end
            
            default: next_state = IDLE;
        endcase 

        // =================================================================
        // CONSUMER FSM — Combinational Logic 
        // =================================================================
        cons_next_state = cons_state;    // default: giữ nguyên state
        
        case (cons_state)
            CONS_IDLE: begin // ko làm gì thì ghi xuống
                if (!wbuf_empty && !main_using_bus && !pending_cpu_miss)
                    cons_next_state = CONS_WRITE;
            end
            
            CONS_WRITE: begin
                mem_write_req  = !cons_req_sent;
                mem_addr       = wbuf_out_addr;
                mem_write_data = wbuf_out_data;
                
                if (mem_write_back_valid) begin
                    mem_write_req   = 1'b0;
                    wbuf_pop        = 1'b1;
                    cons_next_state = CONS_IDLE;
                end
            end
        endcase   
    end
endmodule