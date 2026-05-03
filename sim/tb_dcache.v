`timescale 1ns / 1ps

module tb_data_cache;

    // -------------------------------------------------------------------------
    // CLOCK & RESET
    // -------------------------------------------------------------------------
    reg clk;
    reg rst_n;
    
    always #5 clk = ~clk; // 100MHz Clock

    // -------------------------------------------------------------------------
    // DUT INTERFACE SIGNALS
    // -------------------------------------------------------------------------
    // CPU Interface
    reg         cpu_read_req;
    reg         cpu_write_req;
    reg  [31:0] cpu_addr;
    reg  [31:0] cpu_write_data;
    reg         mem_unsigned;
    reg  [1:0]  mem_size;
    wire [31:0] cpu_read_data;
    wire        dcache_hit;
    wire        dcache_stall;
    
    // Memory Interface
    reg  [31:0] mem_read_data;
    reg         mem_read_ready;
    reg         mem_read_valid;
    reg         mem_write_ready;
    reg         mem_write_back_valid;
    wire        mem_read_req;
    wire        mem_write_req;
    wire [31:0] mem_addr;
    wire [31:0] mem_write_data;

    // -------------------------------------------------------------------------
    // DUT INSTANTIATION
    // -------------------------------------------------------------------------
    data_cache dut (
        .clk                  (clk),
        .rst_n                (rst_n),
        .cpu_read_req         (cpu_read_req),
        .cpu_write_req        (cpu_write_req),
        .cpu_addr             (cpu_addr),
        .mem_read_data        (mem_read_data),
        .cpu_write_data       (cpu_write_data),
        .mem_unsigned         (mem_unsigned),
        .mem_size             (mem_size),
        .mem_read_ready       (mem_read_ready),
        .mem_read_valid       (mem_read_valid),
        .mem_write_ready      (mem_write_ready),
        .mem_write_back_valid (mem_write_back_valid),
        .mem_read_req         (mem_read_req),
        .mem_write_req        (mem_write_req),
        .mem_addr             (mem_addr),
        .cpu_read_data        (cpu_read_data),
        .mem_write_data       (mem_write_data),
        .dcache_hit           (dcache_hit),
        .dcache_stall         (dcache_stall)
    );

    // -------------------------------------------------------------------------
    // SIMULATED MAIN MEMORY MODEL (1024 Words) & LATENCY CONTROLS
    // -------------------------------------------------------------------------
    reg [31:0] main_memory [0:1023];
    integer mem_read_latency  = 2;
    integer mem_write_latency = 2;
    
    // FSM for Memory Responder
    reg [2:0] mem_state;
    parameter M_IDLE = 0, M_READ_WAIT = 1, M_WRITE_WAIT = 2;
    integer mem_delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_state            <= M_IDLE;
            mem_read_ready       <= 1'b0;
            mem_read_valid       <= 1'b0;
            mem_write_ready      <= 1'b0;
            mem_write_back_valid <= 1'b0;
            mem_read_data        <= 32'b0;
            mem_delay_cnt        <= 0;
        end else begin
            // Default pulldowns
            mem_read_ready       <= 1'b0;
            mem_read_valid       <= 1'b0;
            mem_write_ready      <= 1'b0;
            mem_write_back_valid <= 1'b0;
            
            case (mem_state)
                M_IDLE: begin
                    if (mem_read_req) begin
                        mem_read_ready <= 1'b1; // Accept request
                        mem_delay_cnt  <= mem_read_latency;
                        mem_state      <= M_READ_WAIT;
                    end else if (mem_write_req) begin
                        mem_write_ready <= 1'b1; // Accept request
                        mem_delay_cnt   <= mem_write_latency;
                        mem_state       <= M_WRITE_WAIT;
                    end
                end
                M_READ_WAIT: begin
                    if (mem_delay_cnt > 0) begin
                        mem_delay_cnt <= mem_delay_cnt - 1;
                    end else begin
                        mem_read_valid <= 1'b1;
                        // Fetch from simulated memory (ignoring 2 LSBs)
                        mem_read_data  <= main_memory[mem_addr[11:2]];
                        mem_state      <= M_IDLE;
                    end
                end
                M_WRITE_WAIT: begin
                    if (mem_delay_cnt > 0) begin
                        mem_delay_cnt <= mem_delay_cnt - 1;
                    end else begin
                        mem_write_back_valid <= 1'b1;
                        // Write to simulated memory
                        main_memory[mem_addr[11:2]] <= mem_write_data;
                        mem_state <= M_IDLE;
                    end
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // TEST FRAMEWORK & STATISTICS
    // -------------------------------------------------------------------------
    integer total_tests = 0;
    integer passed_tests = 0;
    integer failed_tests = 0;

    task print_phase;
        input [80*8:1] phase_name;
        begin
            $display("\n=========================================================================");
            $display("[PHASE START] %s", phase_name);
            $display("=========================================================================");
        end
    endtask

    task assert_eq;
        input [31:0] expected;
        input [31:0] actual;
        input [80*8:1] test_name;
        begin
            total_tests = total_tests + 1;
            if (expected === actual) begin
                $display("  [PASS] %s | Expected: %08h, Got: %08h", test_name, expected, actual);
                passed_tests = passed_tests + 1;
            end else begin
                $display("  [FAIL] %s | Expected: %08h, Got: %08h !!!", test_name, expected, actual);
                failed_tests = failed_tests + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // CPU BUS DRIVER TASKS
    // -------------------------------------------------------------------------
    task cpu_read;
        input  [31:0] addr;
        input  [1:0]  size;
        input         unsig;
        input  [31:0] expected_data;
        input  [80*8:1] msg;
        reg    [31:0] actual_data;
        begin
            @(negedge clk);
            cpu_read_req = 1'b1;
            cpu_addr     = addr;
            mem_size     = size;
            mem_unsigned = unsig;
            
            @(posedge clk); // Sample DUT outputs
            while (dcache_stall) @(posedge clk);
            
            actual_data = cpu_read_data;
            cpu_read_req = 1'b0;
            assert_eq(expected_data, actual_data, msg);
        end
    endtask

    task cpu_write;
        input [31:0] addr;
        input [31:0] data;
        input [1:0]  size;
        begin
            @(negedge clk);
            cpu_write_req  = 1'b1;
            cpu_addr       = addr;
            cpu_write_data = data;
            mem_size       = size;
            
            @(posedge clk);
            while (dcache_stall) @(posedge clk);
            
            cpu_write_req = 1'b0;
        end
    endtask

    // -------------------------------------------------------------------------
    // MAIN TEST SEQUENCE (16 PHASES)
    // -------------------------------------------------------------------------
    integer i;
    
    initial begin
        // Initialize Signals
        clk = 0;
        rst_n = 0;
        cpu_read_req = 0;
        cpu_write_req = 0;
        cpu_addr = 0;
        cpu_write_data = 0;
        mem_unsigned = 0;
        mem_size = 0;
        
        // Initialize Memory Map with known pattern
        for (i = 0; i < 1024; i = i + 1) begin
            main_memory[i] = i * 32'h00010001; // e.g., Addr 4 -> 0x00010001
        end

        // Assert Reset
        $display("Applying Reset...");
        #20 rst_n = 1;
        #10;
        
        // =====================================================================
        print_phase("Phase 1: Reset & Init Check (Cold Cache)");
        // =====================================================================
        cpu_read(32'h00000004, 2'b00, 1'b0, 32'h00010001, "Cold Read Miss (Word)");
        
        // =====================================================================
        print_phase("Phase 2: Basic Hit (Word Write & Read)");
        // =====================================================================
        cpu_write(32'h00000008, 32'hDEADBEEF, 2'b00); // Word write
        cpu_read(32'h00000008, 2'b00, 1'b0, 32'hDEADBEEF, "Read Hit Verification");

        // =====================================================================
        print_phase("Phase 3: Byte Access & Modification");
        // =====================================================================
        cpu_write(32'h0000000C, 32'h11223344, 2'b00); // Init Word
        cpu_write(32'h0000000D, 32'h000000FF, 2'b10); // Overwrite byte at offset 1
        cpu_read(32'h0000000C, 2'b00, 1'b0, 32'h1122FF44, "Byte Modified Word Read");

        // =====================================================================
        print_phase("Phase 4: Half-word Access & Modification");
        // =====================================================================
        cpu_write(32'h00000010, 32'hAABBCCDD, 2'b00);
        cpu_write(32'h00000012, 32'h00009999, 2'b01); // Overwrite upper half (offset 2)
        cpu_read(32'h00000010, 2'b00, 1'b0, 32'h9999CCDD, "Halfword Modified Word Read");

        // =====================================================================
        print_phase("Phase 5: Sign Extension vs Unsigned Support");
        // =====================================================================
        cpu_write(32'h00000014, 32'h88000000, 2'b00); // MSB is 1
        cpu_read(32'h00000017, 2'b10, 1'b0, 32'hFFFFFF88, "Signed Byte Read");
        cpu_read(32'h00000017, 2'b10, 1'b1, 32'h00000088, "Unsigned Byte Read");
        cpu_read(32'h00000016, 2'b01, 1'b0, 32'hFFFF8800, "Signed Halfword Read");
        cpu_read(32'h00000016, 2'b01, 1'b1, 32'h00008800, "Unsigned Halfword Read");

        // =====================================================================
        print_phase("Phase 6: Compulsory Misses (Fill 4-Way Set 0)");
        // =====================================================================
        // Addresses mapping to index 0:
        // Way 1: 0x00000000 (Tag 0)
        // Way 2: 0x00000040 (Tag 1)
        // Way 3: 0x00000080 (Tag 2)
        // Way 4: 0x000000C0 (Tag 3)
        cpu_write(32'h00000000, 32'hAAAA0000, 2'b00);
        cpu_write(32'h00000040, 32'hBBBB0000, 2'b00);
        cpu_write(32'h00000080, 32'hCCCC0000, 2'b00);
        cpu_write(32'h000000C0, 32'hDDDD0000, 2'b00);
        cpu_read(32'h00000000, 2'b00, 1'b0, 32'hAAAA0000, "Check Set 0 - Way 1");
        cpu_read(32'h000000C0, 2'b00, 1'b0, 32'hDDDD0000, "Check Set 0 - Way 4");

        // =====================================================================
        print_phase("Phase 7: Tree-PLRU Eviction Trigger");
        // =====================================================================
        // Write 5th element to Index 0 -> Tag 4 (0x00000100). Evicts Way 2 (PLRU logic due to access order)
        cpu_write(32'h00000100, 32'hEEEE0000, 2'b00); 
        cpu_read(32'h00000100, 2'b00, 1'b0, 32'hEEEE0000, "5th Element Read Hit");
        // Reading evicted element should cause a miss & fetch
        cpu_read(32'h00000040, 2'b00, 1'b0, 32'hBBBB0000, "Fetch Evicted Element");

        // =====================================================================
        print_phase("Phase 8: Write-Back Policy Verification");
        // =====================================================================
        // The previous evictions were dirty lines. Let's verify Main Memory holds the data.
        // Cache line 0x00000040 was dirty and evicted. Main Memory should have BBBB0000.
        assert_eq(32'hBBBB0000, main_memory[16], "Memory Verification (Dirty Evict)");

        // =====================================================================
        print_phase("Phase 9: Full Word Write Miss Optimization");
        // =====================================================================
        // Full word write shouldn't fetch from memory, it just overwrites.
        cpu_write(32'h00000200, 32'hCAFEF00D, 2'b00);
        cpu_read(32'h00000200, 2'b00, 1'b0, 32'hCAFEF00D, "Full Word Write-Allocate Read");

        // =====================================================================
        print_phase("Phase 10: Sub-Word Write Miss (Fetch-Before-Write)");
        // =====================================================================
        // Byte write to un-cached address. Cache must fetch the word first, modify, then store.
        main_memory[32'h00000204 >> 2] = 32'h12345678; // Pre-seed memory
        cpu_write(32'h00000204, 32'h000000FF, 2'b10); // Write Byte to offset 0
        cpu_read(32'h00000204, 2'b00, 1'b0, 32'h123456FF, "Fetch-and-Modify Sub-word Result");

        // =====================================================================
        print_phase("Phase 11: Alternating Read/Write Rapid Fire");
        // =====================================================================
        cpu_write(32'h00000300, 32'h11111111, 2'b00);
        cpu_read(32'h00000300, 2'b00, 1'b0, 32'h11111111, "Rapid RW 1");
        cpu_write(32'h00000300, 32'h22222222, 2'b00);
        cpu_read(32'h00000300, 2'b00, 1'b0, 32'h22222222, "Rapid RW 2");

        // =====================================================================
        print_phase("Phase 12: Memory Read Backpressure Handling");
        // =====================================================================
        mem_read_latency = 10; // Increase latency drastically
        cpu_read(32'h00000400, 2'b00, 1'b0, main_memory[32'h00000400>>2], "Long Latency Read Miss");
        mem_read_latency = 2;  // Restore

        // =====================================================================
        print_phase("Phase 13: Memory Write Backpressure Handling");
        // =====================================================================
        // Force a dirty line eviction with long write latency
        cpu_write(32'h00000500, 32'hD1111111, 2'b00); // Sửa 32'hDIRTY111
        cpu_write(32'h00000540, 32'hD2222222, 2'b00); // Sửa 32'hDIRTY222
        cpu_write(32'h00000580, 32'hD3333333, 2'b00); // Sửa 32'hDIRTY333
        cpu_write(32'h000005C0, 32'hD4444444, 2'b00); // Sửa 32'hDIRTY444
        mem_write_latency = 15; 
        cpu_write(32'h00000600, 32'hEEEE1111, 2'b00); // Sửa 32'hNEWWRITE
        mem_write_latency = 2;  
        assert_eq(32'hD1111111, main_memory[32'h00000500>>2], "Delayed Eviction Write-Back Status");

        // =====================================================================
        print_phase("Phase 14: Cache Index Thrashing (Index 0xF)");
        // =====================================================================
        cpu_write(32'h0000003C, 32'hA1A1A1A1, 2'b00); // Index 0xF (Tag 0)
        cpu_write(32'h0000007C, 32'hB2B2B2B2, 2'b00); // Index 0xF (Tag 1)
        cpu_write(32'h000000BC, 32'hC3C3C3C3, 2'b00); // Index 0xF (Tag 2)
        cpu_write(32'h000000FC, 32'hD4D4D4D4, 2'b00); // Index 0xF (Tag 3)
        cpu_write(32'h0000013C, 32'hE5E5E5E5, 2'b00); // Index 0xF (Tag 4 - Evicts Tag 0)
        cpu_read(32'h0000007C, 2'b00, 1'b0, 32'hB2B2B2B2, "Thrash Check Surviving Way 2");
        cpu_read(32'h0000003C, 2'b00, 1'b0, 32'hA1A1A1A1, "Thrash Re-fetch Evicted Way");

        // =====================================================================
        print_phase("Phase 15: Read-After-Write Hazard (Immediate sequence)");
        // =====================================================================
        // Test combinational hit paths
        @(negedge clk);
        cpu_write_req = 1; cpu_addr = 32'h00000700; cpu_write_data = 32'hA5A5A599; mem_size = 0; // Sửa 32'hHAZARD99
        @(posedge clk);
        while(dcache_stall) @(posedge clk);
        cpu_write_req = 0;
        
        // Immediate Read in the next cycle
        cpu_read_req = 1; cpu_addr = 32'h00000700; mem_size = 0; mem_unsigned = 0;
        @(posedge clk);
        assert_eq(32'hA5A5A599, cpu_read_data, "0-Cycle RAW Forwarding Check"); // Sửa 32'hHAZARD99
        cpu_read_req = 0;

        // =====================================================================
        print_phase("Phase 16: Cache Line Overwrite & Validation Wrap-up");
        // =====================================================================
        cpu_write(32'h00000040, 32'hF11A1000, 2'b00); // Sửa 32'hFINAL000
        cpu_read(32'h00000040, 2'b00, 1'b0, 32'hF11A1000, "Overwrite Validation"); // Sửa 32'hFINAL000

        // -------------------------------------------------------------------------
        // FINAL SUMMARY
        // -------------------------------------------------------------------------
        $display("\n=========================================================================");
        $display("                          TEST BENCH COMPLETE                            ");
        $display("=========================================================================");
        $display("  Total Tests Run : %0d", total_tests);
        $display("  Passed Tests    : %0d", passed_tests);
        $display("  Failed Tests    : %0d", failed_tests);
        if (failed_tests == 0) begin
            $display("  Status          : [ALL TEST PASSED]");
        end else begin
            $display("  Status          : [FAILED].");
        end
        $display("=========================================================================\n");

        #100 $finish;
    end

endmodule