`timescale 1ns / 1ps

module tb_instruction_cache;

    // =========================================================================
    // SIGNALS
    // =========================================================================
    reg         clk;
    reg         rst_n;
    reg         flush;
    
    // CPU Interface
    reg         cpu_read_req;
    reg  [31:0] cpu_addr;
    wire [31:0] cpu_read_data;
    wire        icache_hit;
    wire        icache_stall;
    
    // AXI4 Interface
    wire [31:0] m_axi_araddr;
    wire [7:0]  m_axi_arlen;
    wire [2:0]  m_axi_arsize;
    wire [1:0]  m_axi_arburst;
    wire        m_axi_arvalid;
    reg         m_axi_arready;
    
    reg  [31:0] m_axi_rdata;
    reg  [1:0]  m_axi_rresp;
    reg         m_axi_rlast;
    reg         m_axi_rvalid;
    wire        m_axi_rready;

    // =========================================================================
    // DUT INSTANTIATION
    // =========================================================================
    instruction_cache dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .flush          (flush),
        .cpu_read_req   (cpu_read_req),
        .cpu_addr       (cpu_addr),
        .cpu_read_data  (cpu_read_data),
        .icache_hit     (icache_hit),
        .icache_stall   (icache_stall),
        
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready)
    );

    // =========================================================================
    // CLOCK GENERATION
    // =========================================================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end

    // =========================================================================
    // MOCK AXI4 SLAVE MEMORY (1MB, word addressable)
    // =========================================================================
    reg [31:0] mock_mem [0:262143]; 
    integer i;
    
    initial begin
        // Initialize memory with recognizable patterns: Data = Address ^ 0xDEADBEEF
        for (i = 0; i < 262144; i = i + 1) begin
            mock_mem[i] = (i * 4) ^ 32'hDEADBEEF;
        end
    end

    // =========================================================================
    // MOCK AXI4 SLAVE MEMORY FSM (Đã sửa lỗi Timing RLAST)
    // =========================================================================
    reg [2:0] axi_state;
    reg [31:0] read_base_addr;
    integer axi_config_delay; 
    integer axi_delay_cnt;

    initial begin
        m_axi_arready = 0;
        m_axi_rvalid  = 0;
        m_axi_rlast   = 0;
        m_axi_rdata   = 0;
        m_axi_rresp   = 0;
        axi_state     = 0;
        axi_config_delay = 0;
        axi_delay_cnt = 0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axi_arready <= 0;
            m_axi_rvalid  <= 0;
            m_axi_rlast   <= 0;
            axi_state     <= 0;
        end else begin
            case (axi_state)
                0: begin // Đợi Master gửi địa chỉ (ARVALID)
                    m_axi_rvalid <= 0;
                    m_axi_rlast  <= 0;
                    if (m_axi_arvalid) begin
                        if (axi_delay_cnt < axi_config_delay) begin
                            axi_delay_cnt <= axi_delay_cnt + 1;
                        end else begin
                            m_axi_arready <= 1;
                            read_base_addr <= m_axi_araddr;
                            axi_state <= 1;
                            axi_delay_cnt <= 0;
                        end
                    end
                end
                1: begin // ARREADY giữ 1 nhịp, sau đó chuẩn bị nhịp Data đầu tiên
                    m_axi_arready <= 0;
                    if (axi_config_delay > 0) begin
                        axi_state <= 4;
                    end else begin
                        m_axi_rvalid <= 1; // Đẩy Data nhịp 1
                        m_axi_rdata  <= mock_mem[read_base_addr >> 2];
                        m_axi_rlast  <= 0;
                        axi_state <= 2;
                    end
                end
                4: begin // Mô phỏng trễ (Backpressure)
                    if (axi_delay_cnt < axi_config_delay) begin
                        axi_delay_cnt <= axi_delay_cnt + 1;
                    end else begin
                        axi_delay_cnt <= 0;
                        m_axi_rvalid <= 1;
                        m_axi_rdata  <= mock_mem[read_base_addr >> 2];
                        m_axi_rlast  <= 0;
                        axi_state <= 2;
                    end
                end
                2: begin // R-Channel BEAT 1 (Lower 32 bits)
                    // Nếu Master báo đã nhận (RREADY)
                    if (m_axi_rvalid && m_axi_rready) begin
                        // Chuẩn bị nhịp 2 ngay lập tức
                        m_axi_rdata <= mock_mem[(read_base_addr >> 2) + 1];
                        m_axi_rlast <= 1; // Bật cờ Last cho nhịp cuối
                        axi_state <= 3;
                    end
                end
                3: begin // R-Channel BEAT 2 (Upper 32 bits)
                    // Đợi Master nhận nhịp cuối
                    if (m_axi_rvalid && m_axi_rready) begin
                        m_axi_rvalid <= 0;
                        m_axi_rlast  <= 0;
                        axi_state <= 0; // Xong Burst, quay về IDLE
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // TEST FRAMEWORK & TASKS
    // =========================================================================
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    // Task to perform a CPU read
    task do_cpu_read;
        input  [31:0] addr;
        output [31:0] data;
        output        hit; // Trạng thái Hit đầu tiên khi yêu cầu
        begin
            @(posedge clk);
            cpu_read_req = 1;
            cpu_addr     = addr;
            
            // Chờ 1 nhịp để cache phản hồi Hit/Miss ban đầu
            @(posedge clk);
            #1; // Delay nhẹ để đọc tín hiệu combinational ổn định
            
            hit = icache_hit; // LƯU LẠI KẾT QUẢ HIT/MISS BAN ĐẦU VÀ KHÔNG GHI ĐÈ
            
            // Nếu cache bị miss và stall, tiếp tục chờ đến khi lấy xong dữ liệu
            while (icache_stall) begin
                @(posedge clk);
                #1;
                // XÓA DÒNG GHI ĐÈ: "hit = icache_hit;" Ở ĐÂY!
            end
            
            // Khi stall xuống 0, dữ liệu đã sẵn sàng
            data = cpu_read_data;
            
            // Giải phóng tín hiệu yêu cầu
            @(posedge clk);
            cpu_read_req = 0;
        end
    endtask

    // Task to check results
    task check_result;
        input [800:1] test_name;
        input [31:0]  expected_data;
        input         expected_hit;
        input [31:0]  actual_data;
        input         actual_hit;
        begin
            if ((expected_data !== actual_data) || (expected_hit !== actual_hit)) begin
                $display("[FAIL] %s", test_name);
                $display("       Expected: Data = 0x%08X, Hit = %b", expected_data, expected_hit);
                $display("       Actual  : Data = 0x%08X, Hit = %b", actual_data, actual_hit);
                fail_cnt = fail_cnt + 1;
            end else begin
                $display("[PASS] %s", test_name);
                pass_cnt = pass_cnt + 1;
            end
        end
    endtask

    function [31:0] get_expected_data(input [31:0] addr);
        begin
            get_expected_data = addr ^ 32'hDEADBEEF;
        end
    endfunction

    // =========================================================================
    // MAIN TEST SEQUENCE
    // =========================================================================
    reg [31:0] actual_data;
    reg        actual_hit;
    reg [31:0] test_addr;

    initial begin
        // Reset sequence
        $display("\n============================================================");
        $display("= INSTRUCTION CACHE TORTURE TESTBENCH STARTED              =");
        $display("============================================================");
        
        cpu_read_req = 0;
        cpu_addr = 0;
        flush = 0;
        rst_n = 0;
        #25;
        rst_n = 1;
        #15;

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 1: Basic Cache Miss & Fill ===");
        // ---------------------------------------------------------------------
        test_addr = 32'h0000_0000; // Index 0, Tag 0, Word 0
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P1: Initial read should MISS and fetch data", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 2: Basic Cache Hit ===");
        // ---------------------------------------------------------------------
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P2: Second read to same address should HIT", get_expected_data(test_addr), 1'b1, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 3: Word Offset Check (Same Block) ===");
        // ---------------------------------------------------------------------
        test_addr = 32'h0000_0004; // Index 0, Tag 0, Word 1 (offset 1)
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P3: Read adjacent word in the same block should HIT", get_expected_data(test_addr), 1'b1, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 4: Set Associativity (Fill Way 2) ===");
        // ---------------------------------------------------------------------
        test_addr = 32'h0000_0080; // Index 0, Tag 1 (Same index, different tag)
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P4: Read new tag at same index should MISS (Fill Way 2)", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 5: Hit on Way 2 ===");
        // ---------------------------------------------------------------------
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P5: Read second tag again should HIT (Way 2 verified)", get_expected_data(test_addr), 1'b1, actual_data, actual_hit);
        
        // Also verify Way 1 is still alive
        test_addr = 32'h0000_0000; 
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P5: Way 1 should still be a HIT", get_expected_data(test_addr), 1'b1, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 6: PLRU Eviction Test ===");
        // ---------------------------------------------------------------------
        // Since we just read Tag 0 (Way 1), PLRU should protect Way 1 and target Way 2 for eviction
        test_addr = 32'h0000_0100; // Index 0, Tag 2
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P6: Read third tag should MISS (Evict Way 2 via PLRU)", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 7: Verify Eviction Correctness ===");
        // ---------------------------------------------------------------------
        test_addr = 32'h0000_0080; // Tag 1 (đã bị xóa ở P6)
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P7: Read evicted Tag 1 should MISS", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);
        
        // LƯU Ý: Lệnh đọc Tag 1 ở trên đã kích hoạt Fetch và ghi đè mất Tag 0. 
        // Do đó lần đọc Tag 0 này CHẮC CHẮN PHẢI MISS!
        test_addr = 32'h0000_0000; 
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P7: Read Tag 0 should MISS (Evicted by P7a fetch)", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 8: Cache Flush Mechanism ===");
        // ---------------------------------------------------------------------
        @(posedge clk);
        flush = 1;
        @(posedge clk);
        flush = 0;
        
        test_addr = 32'h0000_0000; 
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P8: After FLUSH, old data should MISS", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 9: AXI Backpressure & Delay Torture ===");
        // ---------------------------------------------------------------------
        axi_config_delay = 5; // Introduce 5 cycle delay for ARREADY and RVALID
        test_addr = 32'h0000_0200; 
        do_cpu_read(test_addr, actual_data, actual_hit);
        check_result("P9: FSM must handle AXI delay without data corruption", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);
        axi_config_delay = 0; // Restore speed

        // ---------------------------------------------------------------------
        $display("\n=== PHASE 10: Randomized Sequential Burst (Full Cache Fill) ===");
        // ---------------------------------------------------------------------
        @(posedge clk);
        flush = 1;
        @(posedge clk);
        flush = 0;

        // Loop through all 16 sets, filling them up
        for (i = 0; i < 16; i = i + 1) begin
            test_addr = (i * 8); // Index i, offset 0
            do_cpu_read(test_addr, actual_data, actual_hit);
            check_result("P10.a: Fill sequence MISS", get_expected_data(test_addr), 1'b0, actual_data, actual_hit);
            
            test_addr = (i * 8) + 4; // Index i, offset 1
            do_cpu_read(test_addr, actual_data, actual_hit);
            check_result("P10.b: Sequential word HIT", get_expected_data(test_addr), 1'b1, actual_data, actual_hit);
        end

        // ---------------------------------------------------------------------
        // FINAL SUMMARY
        // ---------------------------------------------------------------------
        $display("\n============================================================");
        $display("= TORTURE TEST SUMMARY                                     =");
        $display("============================================================");
        $display("  Total Tests Passed: %0d", pass_cnt);
        $display("  Total Tests Failed: %0d", fail_cnt);
        if (fail_cnt == 0) begin
            $display("\n  >>> RESULT: ALL TESTS PASSED! <<< \n");
        end else begin
            $display("\n  >>> RESULT: FAILED! <<< \n");
        end
        $display("============================================================\n");
        
        $finish;
    end

endmodule