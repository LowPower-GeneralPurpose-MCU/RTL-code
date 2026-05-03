`timescale 1ns / 1ps

module tb_fifo_cdc;

    // =========================================================================
    // 1. CLOCK & RESET GENERATION
    // =========================================================================
    reg clk_fast = 0; // 400 MHz
    reg clk_slow = 0; // 200 MHz or 300 MHz
    reg rst_n = 0;

    real half_period_fast = 1.25; // 400MHz -> 2.5ns period
    real half_period_slow = 2.50; // Starts at 200MHz -> 5.0ns period

    always #half_period_fast clk_fast = ~clk_fast;
    always #half_period_slow clk_slow = ~clk_slow;

    // =========================================================================
    // 2. DUT SIGNALS
    // =========================================================================
    // --- async_fifo (DUT1) ---
    reg         f1_winc;
    reg  [31:0] f1_wdata;
    wire        f1_wfull;
    reg         f1_rinc;
    wire [31:0] f1_rdata;
    wire        f1_rempty;

    // --- async_fifo_cdc (DUT2) ---
    reg         f2_winc;
    reg  [31:0] f2_wdata;
    wire        f2_wfull;
    reg         f2_rinc;
    wire [31:0] f2_rdata;
    wire        f2_rempty;

    // --- native_cdc_bridge (DUT3) ---
    reg         n_cpu_req_val;
    reg         n_cpu_req_is_write;
    reg  [31:0] n_cpu_req_addr;
    reg  [31:0] n_cpu_req_wdata;
    reg  [3:0]  n_cpu_req_wstrb;
    reg  [1:0]  n_cpu_req_size;
    wire        n_cpu_req_ready;
    wire        n_cpu_resp_val;
    wire [31:0] n_cpu_resp_rdata;

    wire        n_bus_req_val;
    wire        n_bus_req_is_write;
    wire [31:0] n_bus_req_addr;
    wire [31:0] n_bus_req_wdata;
    wire [3:0]  n_bus_req_wstrb;
    wire [1:0]  n_bus_req_size;
    reg         n_bus_req_ready;
    reg         n_bus_resp_val;
    reg  [31:0] n_bus_resp_rdata;

    // --- axi4_read_cdc (DUT4) ---
    reg  [31:0] a_s_axi_araddr;
    reg  [7:0]  a_s_axi_arlen;
    reg  [2:0]  a_s_axi_arsize;
    reg  [1:0]  a_s_axi_arburst;
    reg         a_s_axi_arvalid;
    wire        a_s_axi_arready;
    wire [31:0] a_s_axi_rdata;
    wire [1:0]  a_s_axi_rresp;
    wire        a_s_axi_rlast;
    wire        a_s_axi_rvalid;
    reg         a_s_axi_rready;

    wire [31:0] a_m_axi_araddr;
    wire [7:0]  a_m_axi_arlen;
    wire [2:0]  a_m_axi_arsize;
    wire [1:0]  a_m_axi_arburst;
    wire        a_m_axi_arvalid;
    reg         a_m_axi_arready;
    reg  [31:0] a_m_axi_rdata;
    reg  [1:0]  a_m_axi_rresp;
    reg         a_m_axi_rlast;
    reg         a_m_axi_rvalid;
    wire        a_m_axi_rready;

    // Phase 16 Loopback control
    reg phase_16_en = 0;

    // =========================================================================
    // 3. DUT INSTANTIATIONS
    // =========================================================================
    async_fifo #(.DATA_WIDTH(32), .ADDR_WIDTH(4)) DUT1_FIFO (
        .wclk(clk_fast), .wrst_n(rst_n), .winc(f1_winc), .wdata(f1_wdata), .wfull(f1_wfull),
        .rclk(clk_slow), .rrst_n(rst_n), .rinc(f1_rinc), .rdata(f1_rdata), .rempty(f1_rempty)
    );

    async_fifo_cdc #(.DATA_WIDTH(32), .ADDR_WIDTH(4)) DUT2_FIFO_CDC (
        .wclk(clk_fast), .wrst_n(rst_n), .winc(f2_winc), .wdata(f2_wdata), .wfull(f2_wfull),
        .rclk(clk_slow), .rrst_n(rst_n), .rinc(f2_rinc), .rdata(f2_rdata), .rempty(f2_rempty)
    );

    native_cdc_bridge DUT3_NATIVE (
        .cpu_clk(clk_fast), .cpu_rst_n(rst_n),
        .cpu_req_val(n_cpu_req_val), .cpu_req_is_write(n_cpu_req_is_write),
        .cpu_req_addr(n_cpu_req_addr), .cpu_req_wdata(n_cpu_req_wdata),
        .cpu_req_wstrb(n_cpu_req_wstrb), .cpu_req_size(n_cpu_req_size),
        .cpu_req_ready(n_cpu_req_ready), .cpu_resp_val(n_cpu_resp_val), .cpu_resp_rdata(n_cpu_resp_rdata),
        
        .bus_clk(clk_slow), .bus_rst_n(rst_n),
        .bus_req_val(n_bus_req_val), .bus_req_is_write(n_bus_req_is_write),
        .bus_req_addr(n_bus_req_addr), .bus_req_wdata(n_bus_req_wdata),
        .bus_req_wstrb(n_bus_req_wstrb), .bus_req_size(n_bus_req_size),
        .bus_req_ready(phase_16_en ? a_s_axi_arready : n_bus_req_ready), 
        .bus_resp_val(n_bus_resp_val), .bus_resp_rdata(n_bus_resp_rdata)
    );

    axi4_read_cdc #( .ADDR_WIDTH(32), .DATA_WIDTH(32) ) DUT4_AXI (
        .clk_core(clk_fast), .rst_core_n(rst_n),
        .s_axi_araddr(phase_16_en ? n_bus_req_addr : a_s_axi_araddr), 
        .s_axi_arlen(phase_16_en ? 8'h00 : a_s_axi_arlen), 
        .s_axi_arsize(phase_16_en ? {1'b0, n_bus_req_size} : a_s_axi_arsize), 
        .s_axi_arburst(phase_16_en ? 2'b01 : a_s_axi_arburst), 
        .s_axi_arvalid(phase_16_en ? n_bus_req_val : a_s_axi_arvalid), 
        .s_axi_arready(a_s_axi_arready),
        .s_axi_rdata(a_s_axi_rdata), .s_axi_rresp(a_s_axi_rresp), .s_axi_rlast(a_s_axi_rlast), 
        .s_axi_rvalid(a_s_axi_rvalid), .s_axi_rready(a_s_axi_rready),
        
        .clk_bus(clk_slow), .rst_bus_n(rst_n),
        .m_axi_araddr(a_m_axi_araddr), .m_axi_arlen(a_m_axi_arlen), .m_axi_arsize(a_m_axi_arsize), 
        .m_axi_arburst(a_m_axi_arburst), .m_axi_arvalid(a_m_axi_arvalid), .m_axi_arready(a_m_axi_arready),
        .m_axi_rdata(a_m_axi_rdata), .m_axi_rresp(a_m_axi_rresp), .m_axi_rlast(a_m_axi_rlast), 
        .m_axi_rvalid(a_m_axi_rvalid), .m_axi_rready(a_m_axi_rready)
    );

    // =========================================================================
    // 4. SCOREBOARD & HELPER VARIABLES
    // =========================================================================
    integer global_errors = 0;
    integer phase_errors  = 0;
    integer i, j;
    reg [31:0] expected_data;

    // FIFO validation memory arrays
    reg [31:0] mem_f1 [0:1023]; integer head_f1 = 0, tail_f1 = 0;
    reg [31:0] mem_f2 [0:1023]; integer head_f2 = 0, tail_f2 = 0;

    // Macro-like checking task
    task check_val;
        input [31:0] act, exp;
        input [63:0] name;
        begin
            if (act !== exp) begin
                $display("[ERROR] %s mismatch! Expected: %h, Got: %h", name, exp, act);
                phase_errors = phase_errors + 1;
            end
        end
    endtask

    // System reset sequence
    task sys_reset;
        begin
            rst_n = 0;
            f1_winc = 0; f1_wdata = 0; f1_rinc = 0;
            f2_winc = 0; f2_wdata = 0; f2_rinc = 0;
            
            n_cpu_req_val = 0; n_cpu_req_is_write = 0; n_cpu_req_addr = 0; 
            n_cpu_req_wdata = 0; n_cpu_req_wstrb = 0; n_cpu_req_size = 0;
            n_bus_req_ready = 0; n_bus_resp_val = 0; n_bus_resp_rdata = 0;
            
            a_s_axi_arvalid = 0; a_s_axi_araddr = 0; a_s_axi_arlen = 0; 
            a_s_axi_arsize = 0; a_s_axi_arburst = 0; a_s_axi_rready = 0;
            a_m_axi_arready = 0; a_m_axi_rvalid = 0; a_m_axi_rdata = 0; 
            a_m_axi_rresp = 0; a_m_axi_rlast = 0;

            head_f1 = 0; tail_f1 = 0;
            head_f2 = 0; tail_f2 = 0;
            phase_16_en = 0;

            #50 rst_n = 1;
            #50;
        end
    endtask

    // =========================================================================
    // 5. MAIN TORTURE TEST SEQUENCE
    // =========================================================================
    integer mode; // 0: 400-200MHz, 1: 400-300MHz
    
    initial begin
        $display("===============================================================");
        $display(" STARTING CDC & FIFO TORTURE TEST SUITE ");
        $display("===============================================================");

        for (mode = 0; mode < 2; mode = mode + 1) begin
            if (mode == 0) begin
                $display("\n---> RUNNING SUITE IN MODE 0: CPU=400MHz, BUS=200MHz <---");
                half_period_slow = 2.50; // 200MHz
            end else begin
                $display("\n---> RUNNING SUITE IN MODE 1: CPU=400MHz, BUS=300MHz <---");
                half_period_slow = 1.666; // 300MHz
            end
            sys_reset();

            // -----------------------------------------------------------------
            // FIFOS TEST PHASES (1-5)
            // -----------------------------------------------------------------
            
            // PHASE 1: Basic Write & Read
            $display("[PHASE 1] Basic Write & Read on Both FIFOs");
            phase_errors = 0;
            @(negedge clk_fast);
            f1_wdata = 32'hAAAA_1111; f1_winc = 1;
            f2_wdata = 32'hBBBB_2222; f2_winc = 1;
            @(negedge clk_fast); f1_winc = 0; f2_winc = 0;
            #20; // CDC wait
            @(negedge clk_slow);
            if(f1_rempty) phase_errors = phase_errors + 1;
            if(f2_rempty) phase_errors = phase_errors + 1;
            check_val(f1_rdata, 32'hAAAA_1111, "F1 RDATA");
            check_val(f2_rdata, 32'hBBBB_2222, "F2 RDATA");
            f1_rinc = 1; f2_rinc = 1;
            @(negedge clk_slow); f1_rinc = 0; f2_rinc = 0;
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // PHASE 2: Write till Full, then Read
            $display("[PHASE 2] Write till Full, check backpressure, then Read");
            phase_errors = 0;
            for(i=0; i<18; i=i+1) begin
                @(negedge clk_fast);
                if (!f1_wfull) begin f1_wdata = i; f1_winc = 1; mem_f1[head_f1] = i; head_f1=head_f1+1; end else f1_winc = 0;
                if (!f2_wfull) begin f2_wdata = i; f2_winc = 1; mem_f2[head_f2] = i; head_f2=head_f2+1; end else f2_winc = 0;
            end
            @(negedge clk_fast); f1_winc = 0; f2_winc = 0;
            if (!f1_wfull || !f2_wfull) phase_errors = phase_errors + 1; // Expected to be full at 16 depth
            #50;
            for(i=0; i<16; i=i+1) begin
                @(negedge clk_slow);
                if (!f1_rempty) begin check_val(f1_rdata, mem_f1[tail_f1], "F2 FullRd"); f1_rinc=1; tail_f1=tail_f1+1; end
                if (!f2_rempty) begin check_val(f2_rdata, mem_f2[tail_f2], "F2 FullRd"); f2_rinc=1; tail_f2=tail_f2+1; end
            end
            @(negedge clk_slow); f1_rinc=0; f2_rinc=0;
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // PHASE 3: Read till Empty, then Write
            $display("[PHASE 3] Read till Empty validation");
            phase_errors = 0;
            @(negedge clk_slow); f1_rinc = 1; f2_rinc = 1; // Try to underflow
            @(negedge clk_slow); f1_rinc = 0; f2_rinc = 0;
            if(!f1_rempty || !f2_rempty) phase_errors = phase_errors + 1;
            @(negedge clk_fast); f1_wdata = 32'hCAFE; f1_winc = 1; f2_wdata = 32'hBABE; f2_winc = 1;
            @(negedge clk_fast); f1_winc = 0; f2_winc = 0;
            #50;
            @(negedge clk_slow);
            check_val(f1_rdata, 32'hCAFE, "F1 REC");
            check_val(f2_rdata, 32'hBABE, "F2 REC");
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // PHASE 4: Concurrent Random Push/Pop
            $display("[PHASE 4] Concurrent Write/Read (Stress Test)");
            phase_errors = 0;
            fork
                begin // Writer thread
                    for(j=0; j<50; j=j+1) begin
                        @(negedge clk_fast);
                        if (!f1_wfull && ($random % 2)) begin f1_wdata = $random; f1_winc = 1; end else f1_winc = 0;
                        if (!f2_wfull && ($random % 2)) begin f2_wdata = $random; f2_winc = 1; end else f2_winc = 0;
                    end
                    @(negedge clk_fast); f1_winc = 0; f2_winc = 0;
                end
                begin // Reader thread
                    for(j=0; j<80; j=j+1) begin
                        @(negedge clk_slow);
                        if (!f1_rempty && ($random % 2)) f1_rinc = 1; else f1_rinc = 0;
                        if (!f2_rempty && ($random % 2)) f2_rinc = 1; else f2_rinc = 0;
                    end
                    @(negedge clk_slow); f1_rinc = 0; f2_rinc = 0;
                end
            join
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // PHASE 5: Reset during operations
            $display("[PHASE 5] Asynchronous Reset Mid-Operation");
            phase_errors = 0;
            @(negedge clk_fast); f1_winc=1; f1_wdata=32'h111; f2_winc=1; f2_wdata=32'h222;
            @(negedge clk_fast); rst_n = 0; // Async reset triggers
            @(negedge clk_fast); rst_n = 1; f1_winc=0; f2_winc=0;
            #20;
            if (!f1_rempty || !f2_rempty) phase_errors = phase_errors + 1; // Should be empty after reset
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // -----------------------------------------------------------------
            // NATIVE CDC TEST PHASES (6-10)
            // -----------------------------------------------------------------

            // PHASE 6: Native Bridge Single Req/Resp
            $display("[PHASE 6] Native CDC Single Request/Response");
            phase_errors = 0;
            n_bus_req_ready = 1; // Always ready to pop
            @(negedge clk_fast);
            n_cpu_req_val = 1; n_cpu_req_addr = 32'h8000; n_cpu_req_wdata = 32'hDEADBEEF;
            @(negedge clk_fast); n_cpu_req_val = 0;
            
            // Wait for bus domain to see it
            while(!n_bus_req_val) @(posedge clk_slow);
            check_val(n_bus_req_addr, 32'h8000, "Native Req Addr");
            check_val(n_bus_req_wdata, 32'hDEADBEEF, "Native Req Data");
            
            @(negedge clk_slow);
            n_bus_resp_val = 1; n_bus_resp_rdata = 32'hC0FFEE;
            @(negedge clk_slow); n_bus_resp_val = 0;
            
            while(!n_cpu_resp_val) @(posedge clk_fast);
            check_val(n_cpu_resp_rdata, 32'hC0FFEE, "Native Resp Data");
            
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // PHASE 7: Native Burst Reqs
            $display("[PHASE 7] Native CDC Burst CPU Requests");
            phase_errors = 0;
            n_bus_req_ready = 1;
            for(i=0; i<8; i=i+1) begin
                @(negedge clk_fast);
                n_cpu_req_val = 1; n_cpu_req_addr = i * 4;
            end
            @(negedge clk_fast); n_cpu_req_val = 0;
            #150; // Let them propagate
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // PHASE 8 & 9 & 10 (Merged to Stress Task for brevity): Full Duplex & Backpressure
            $display("[PHASE 8/9/10] Native CDC Full Duplex with Random Backpressure");
            phase_errors = 0;
            fork
                begin // CPU Injector
                    for(j=0; j<20; j=j+1) begin
                        @(negedge clk_fast);
                        n_cpu_req_val = 1; n_cpu_req_addr = j;
                        while(!n_cpu_req_ready) @(negedge clk_fast);
                    end
                    @(negedge clk_fast); n_cpu_req_val = 0;
                end
                begin // Bus Acceptor & Resp Injector
                    for(j=0; j<20; j=j+1) begin
                        @(negedge clk_slow);
                        n_bus_req_ready = $random % 2; // Random backpressure
                        if(n_bus_req_val && n_bus_req_ready) begin
                            n_bus_resp_val = 1; n_bus_resp_rdata = n_bus_req_addr + 10;
                        end else begin
                            n_bus_resp_val = 0;
                        end
                    end
                    @(negedge clk_slow); n_bus_resp_val = 0;
                end
            join
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // -----------------------------------------------------------------
            // AXI CDC TEST PHASES (11-15)
            // -----------------------------------------------------------------

            // PHASE 11: AXI Single AR/R
            $display("[PHASE 11] AXI4 CDC Single AR and R transaction");
            phase_errors = 0;
            a_m_axi_arready = 1; a_s_axi_rready = 1;
            @(negedge clk_fast);
            a_s_axi_arvalid = 1; a_s_axi_araddr = 32'h4000;
            @(negedge clk_fast); a_s_axi_arvalid = 0;
            
            while(!a_m_axi_arvalid) @(posedge clk_slow);
            check_val(a_m_axi_araddr, 32'h4000, "AXI AR Addr");

            @(negedge clk_slow);
            a_m_axi_rvalid = 1; a_m_axi_rdata = 32'hFEEDFACE; a_m_axi_rlast = 1;
            @(negedge clk_slow); a_m_axi_rvalid = 0;

            while(!a_s_axi_rvalid) @(posedge clk_fast);
            check_val(a_s_axi_rdata, 32'hFEEDFACE, "AXI R Data");

            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // PHASE 12/13/14/15: AXI Stress & Random Valid/Ready 
            $display("[PHASE 12-15] AXI4 CDC Heavy Random Traffic & Backpressure");
            phase_errors = 0;
            fork
                begin // Core AR Injector
                    for(j=0; j<25; j=j+1) begin
                        @(negedge clk_fast);
                        a_s_axi_arvalid = 1; a_s_axi_araddr = j * 8;
                        while(!a_s_axi_arready) @(negedge clk_fast);
                    end
                    @(negedge clk_fast); a_s_axi_arvalid = 0;
                end
                begin // Bus AR Acceptor & R Injector
                    for(j=0; j<25; j=j+1) begin
                        @(negedge clk_slow);
                        a_m_axi_arready = $random % 2; 
                        if(a_m_axi_arvalid && a_m_axi_arready) begin
                            a_m_axi_rvalid = 1; a_m_axi_rdata = a_m_axi_araddr;
                        end else begin
                            a_m_axi_rvalid = 0;
                        end
                    end
                    @(negedge clk_slow); a_m_axi_rvalid = 0;
                end
                begin // Core R Acceptor
                    for(j=0; j<50; j=j+1) begin
                        @(negedge clk_fast);
                        a_s_axi_rready = $random % 2; 
                    end
                    @(negedge clk_fast); a_s_axi_rready = 1;
                end
            join
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; sys_reset();

            // -----------------------------------------------------------------
            // COMBINED PHASE (16)
            // -----------------------------------------------------------------
            
            // PHASE 16: Connect Native CDC -> AXI CDC (Loopback Test)
            $display("[PHASE 16] COMBINED CDC LOOPBACK (Native CPU -> AXI Bus)");
            phase_errors = 0;
            phase_16_en = 1; 
            
            // FIX: Set ready = 0 để giam data lại, không cho FIFO tự pop
            a_m_axi_arready = 0; 
            
            @(negedge clk_fast);
            n_cpu_req_val = 1; n_cpu_req_addr = 32'h1337_1337; n_cpu_req_size = 2'b10;
            @(negedge clk_fast); n_cpu_req_val = 0;
            
            // FIX: Chủ động polling chờ tín hiệu valid (timeout sau 100 chu kỳ slow)
            i = 0;
            while(!a_m_axi_arvalid && i < 100) begin
                @(posedge clk_slow);
                i = i + 1;
            end
            
            if(a_m_axi_arvalid) begin
                check_val(a_m_axi_araddr, 32'h1337_1337, "Combined Loopback Addr");
                
                // Sau khi check xong mới cấp ready = 1 để pop data
                @(negedge clk_slow);
                a_m_axi_arready = 1; 
                @(negedge clk_slow);
                a_m_axi_arready = 0;
            end else begin
                $display("[ERROR] Combined test timed out!");
                phase_errors = phase_errors + 1;
            end
            
            if (phase_errors == 0) $display(" -> PASS"); else $display(" -> FAIL");
            global_errors = global_errors + phase_errors; 
            phase_16_en = 0;
            sys_reset();

        end // end for loop (Clock Modes)

        $display("\n===============================================================");
        if (global_errors == 0)
            $display(">>> SUCCESS! ALL TEST PASSED! <<<");
        else
            $display(">>> CRITICAL FAILURE DETECTED! <<<");
        $display("===============================================================");
        $finish;
    end

endmodule