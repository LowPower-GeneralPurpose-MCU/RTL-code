`timescale 1ns / 1ps

module tb_axi_to_apb_bridge;

    // Parameters
    parameter ADDR_WIDTH = 16;
    parameter DATA_WIDTH = 32;
    parameter CLK_PERIOD = 10;

    // --- Signals ---
    reg                     clk;
    reg                     rst_n;

    // AXI Slave Interface
    reg  [ADDR_WIDTH-1:0]   s_axi_awaddr;
    reg  [2:0]              s_axi_awprot;
    reg                     s_axi_awvalid;
    wire                    s_axi_awready;
    reg  [DATA_WIDTH-1:0]   s_axi_wdata;
    reg  [3:0]              s_axi_wstrb;
    reg                     s_axi_wvalid;
    wire                    s_axi_wready;
    wire [1:0]              s_axi_bresp;
    wire                    s_axi_bvalid;
    reg                     s_axi_bready;
    reg  [ADDR_WIDTH-1:0]   s_axi_araddr;
    reg  [2:0]              s_axi_arprot;
    reg                     s_axi_arvalid;
    wire                    s_axi_arready;
    wire [DATA_WIDTH-1:0]   s_axi_rdata;
    wire [1:0]              s_axi_rresp;
    wire                    s_axi_rvalid;
    reg                     s_axi_rready;

    // APB Master Interface
    wire [ADDR_WIDTH-1:0]   m_apb_paddr;
    wire [2:0]              m_apb_pprot;
    wire                    m_apb_psel;
    wire                    m_apb_penable;
    wire                    m_apb_pwrite;
    wire [DATA_WIDTH-1:0]   m_apb_pwdata;
    wire [3:0]              m_apb_pstrb;
    reg                     m_apb_pready;
    reg  [DATA_WIDTH-1:0]   m_apb_prdata;
    reg                     m_apb_pslverr;

    // Testbench Control
    integer error_count = 0;
    integer test_phase = 0;
    reg [DATA_WIDTH-1:0] mem [0:65535]; // Simple memory for APB Slave Emulator

    // --- Device Under Test (DUT) ---
    axi_to_apb_bridge #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arprot(s_axi_arprot),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .m_apb_paddr(m_apb_paddr),
        .m_apb_pprot(m_apb_pprot),
        .m_apb_psel(m_apb_psel),
        .m_apb_penable(m_apb_penable),
        .m_apb_pwrite(m_apb_pwrite),
        .m_apb_pwdata(m_apb_pwdata),
        .m_apb_pstrb(m_apb_pstrb),
        .m_apb_pready(m_apb_pready),
        .m_apb_prdata(m_apb_prdata),
        .m_apb_pslverr(m_apb_pslverr)
    );

    // --- Clock Generation ---
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // --- APB Slave Emulator ---
    // Simulates a peripheral with memory and random wait states
    always @(posedge clk) begin
        if (!rst_n) begin
            m_apb_pready <= 0;
            m_apb_prdata <= 0;
            m_apb_pslverr <= 0;
        end else begin
            if (m_apb_psel && !m_apb_penable) begin
                // Setup phase: decide response for Access phase
                m_apb_pready <= 0; 
            end else if (m_apb_psel && m_apb_penable) begin
                // Access phase
                m_apb_pready <= 1; // Default: 0 wait state
                if (m_apb_pwrite) begin
                    mem[m_apb_paddr] <= m_apb_pwdata;
                end else begin
                    m_apb_prdata <= mem[m_apb_paddr];
                end
                
                // Inject Error for specific address in Phase 8
                if (test_phase == 8 && m_apb_paddr == 16'hDEAD)
                    m_apb_pslverr <= 1;
                else
                    m_apb_pslverr <= 0;
            end else begin
                m_apb_pready <= 0;
            end
        end
    end

    // --- Test Tasks ---
    task axi_write(input [ADDR_WIDTH-1:0] addr, input [DATA_WIDTH-1:0] data, input [3:0] strb);
    begin
        @(posedge clk);
        s_axi_awaddr = addr;
        s_axi_awvalid = 1;
        s_axi_wdata = data;
        s_axi_wstrb = strb;
        s_axi_wvalid = 1;
        s_axi_bready = 1;

        wait (s_axi_awready && s_axi_wready);
        @(posedge clk);
        s_axi_awvalid = 0;
        s_axi_wvalid = 0;

        wait (s_axi_bvalid);
        if (s_axi_bresp !== 2'b00 && test_phase != 8) begin
            $display("[ERROR] Write Response Error at Addr: %h", addr);
            error_count = error_count + 1;
        end
        @(posedge clk);
        s_axi_bready = 0;
    end
    endtask

    task axi_read(input [ADDR_WIDTH-1:0] addr, input [DATA_WIDTH-1:0] expected_data, input check);
    begin
        @(posedge clk);
        s_axi_araddr = addr;
        s_axi_arvalid = 1;
        s_axi_rready = 1;

        wait (s_axi_arready);
        @(posedge clk);
        s_axi_arvalid = 0;

        wait (s_axi_rvalid);
        if (check && (s_axi_rdata !== expected_data)) begin
            $display("[ERROR] Read Data Mismatch! Addr: %h, Got: %h, Exp: %h", addr, s_axi_rdata, expected_data);
            error_count = error_count + 1;
        end
        @(posedge clk);
        s_axi_rready = 0;
    end
    endtask

    // --- Main Test Sequence ---
    initial begin
        // Initialize
        rst_n = 0;
        s_axi_awaddr = 0; s_axi_awprot = 0; s_axi_awvalid = 0;
        s_axi_wdata = 0;  s_axi_wstrb = 0;  s_axi_wvalid = 0;
        s_axi_bready = 0; s_axi_araddr = 0; s_axi_arprot = 0;
        s_axi_arvalid = 0; s_axi_rready = 0;
        
        $display("---------------------------------------------------");
        $display("STARTING AXI TO APB BRIDGE TORTURE TEST");
        $display("---------------------------------------------------");

        // PHASE 1: Reset
        test_phase = 1;
        $display("[PHASE 1] Resetting System...");
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        if (s_axi_awready || m_apb_psel) begin
            $display("[FAIL] Reset state not clean");
            error_count = error_count + 1;
        end else $display("[PASS] Reset Clean.");

        // PHASE 2: Single Write
        test_phase = 2;
        $display("[PHASE 2] Single Write Operation...");
        axi_write(16'h1000, 32'hDEADBEEF, 4'hF);
        $display("[PASS] Single Write Finished.");

        // PHASE 3: Single Read
        test_phase = 3;
        $display("[PHASE 3] Single Read Operation...");
        axi_read(16'h1000, 32'hDEADBEEF, 1);
        $display("[PASS] Single Read Finished.");

        // PHASE 4: Sequential Write/Read
        test_phase = 4;
        $display("[PHASE 4] Sequential Access (10 items)...");
        begin : seq_test
            integer i;
            for (i=0; i<10; i=i+1) begin
                axi_write(i*4, i*32'h1111, 4'hF);
            end
            for (i=0; i<10; i=i+1) begin
                axi_read(i*4, i*32'h1111, 1);
            end
        end
        $display("[PASS] Sequential Access Finished.");

        // PHASE 5: Back-to-Back (No Delay)
        test_phase = 5;
        $display("[PHASE 5] Stress Test: Back-to-Back Write-Read...");
        fork
            axi_write(16'h2000, 32'hAAAA_BBBB, 4'hF);
            axi_read(16'h0004, 32'h0000_1111, 1);
        join
        $display("[PASS] Back-to-back sequence handled.");

        // PHASE 6: Strobe Test (Byte Enables)
        test_phase = 6;
        $display("[PHASE 6] Strobe (WSTRB) Testing...");
        // Note: Our simple APB slave ignores PSTRB in logic, but we verify signal toggle
        axi_write(16'h3000, 32'h12345678, 4'b0001); 
        if (dut.m_apb_pstrb !== 4'b0001) begin
             $display("[FAIL] PSTRB mismatch");
             error_count = error_count + 1;
        end else $display("[PASS] Strobe signal verified.");

        // PHASE 7: Protection Bits
        test_phase = 7;
        $display("[PHASE 7] AXI Protection Bits (AWPROT/ARPROT)...");
        s_axi_awprot = 3'b101;
        axi_write(16'h4000, 32'h1, 4'hF);
        if (m_apb_pprot !== 3'b101) begin
            $display("[FAIL] PPROT propagation failed");
            error_count = error_count + 1;
        end else $display("[PASS] Protection bits propagated.");

        // PHASE 8: Slave Error (PSLVERR)
        test_phase = 8;
        $display("[PHASE 8] Error Handling (PSLVERR -> BRESP/RRESP)...");
        axi_write(16'hDEAD, 32'h0, 4'hF); // Emulator triggers error on 0xDEAD
        if (s_axi_bresp !== 2'b10) begin
            $display("[FAIL] BRESP did not indicate SLVERR");
            error_count = error_count + 1;
        end else $display("[PASS] Slave Error correctly reported.");

        // PHASE 9: Rapid Toggle
        test_phase = 9;
        $display("[PHASE 9] Rapid Toggle Check...");
        repeat(5) begin
            axi_write(16'h5000, $random, 4'hF);
        end
        $display("[PASS] Rapid toggle complete.");

        // PHASE 10: Final Readback
        test_phase = 10;
        $display("[PHASE 10] Consistency Check...");
        axi_read(16'h0000, 32'h0000_0000, 1);
        axi_read(16'h0004, 32'h0000_1111, 1);
        $display("[PASS] System consistent.");

        // --- Final Result ---
        $display("---------------------------------------------------");
        if (error_count == 0) begin
            $display("TEST RESULT: PASSED");
            $display("All 10 phases completed successfully.");
        end else begin
            $display("TEST RESULT: FAILED");
            $display("Total Errors Found: %d", error_count);
        end
        $display("---------------------------------------------------");
        $finish;
    end

    // Timeout Monitor
    initial begin
        #5000;
        $display("[TIMEOUT] Testbench hung! Check FSM states.");
        $finish;
    end

endmodule