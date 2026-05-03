`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Testbench for apb_interrupt_controller
// Tests: reset, APB R/W, edge detect, priority arbiter, threshold,
//        claim/complete, overflow, force, pslverr
//////////////////////////////////////////////////////////////////////////////////

module test_bench;

    // ── Parameters ──
    localparam CLK_PERIOD = 10;  // 100 MHz
    localparam NUM_IRQ    = 6;

    // ── Register address map ──
    localparam ADDR_IE        = 12'h000;
    localparam ADDR_CLAIM     = 12'h004;
    localparam ADDR_PENDING   = 12'h008;
    localparam ADDR_OVERFLOW  = 12'h00C;
    localparam ADDR_VERSION   = 12'h010;
    localparam ADDR_FORCE     = 12'h014;
    localparam ADDR_THRESHOLD = 12'h018;
    // Priority regs: 0x100 + 4*(id-1)  for id = 1..6

    // ── DUT signals ──
    reg         pclk;
    reg         presetn;
    reg  [11:0] paddr;
    reg         psel;
    reg         penable;
    reg         pwrite;
    reg  [31:0] pwdata;
    reg  [3:0]  pstrb;
    wire        pready;
    wire [31:0] prdata;
    wire        pslverr;

    reg         irq_timer;
    reg         irq_uart;
    reg         irq_spi;
    reg         irq_i2c;
    reg         irq_gpio;
    reg         irq_accel;
    wire        cpu_ext_irq;

    // ── For checking ──
    reg  [31:0] rd_data;
    integer     err_count;
    integer     test_num;

    // ── DUT instantiation ──
    apb_interrupt_controller #(
        .ADDR_WIDTH(12),
        .DATA_WIDTH(32),
        .NUM_IRQ(NUM_IRQ)
    ) uut (
        .pclk       (pclk),
        .presetn    (presetn),
        .paddr      (paddr),
        .psel       (psel),
        .penable    (penable),
        .pwrite     (pwrite),
        .pwdata     (pwdata),
        .pstrb      (pstrb),
        .pready     (pready),
        .prdata     (prdata),
        .pslverr    (pslverr),
        .irq_timer  (irq_timer),
        .irq_uart   (irq_uart),
        .irq_spi    (irq_spi),
        .irq_i2c    (irq_i2c),
        .irq_gpio   (irq_gpio),
        .irq_accel  (irq_accel),
        .cpu_ext_irq(cpu_ext_irq)
    );

    // ── Clock generation ──
    initial pclk = 0;
    always #(CLK_PERIOD/2) pclk = ~pclk;

    // APB helper tasks

    // APB Write (full word, all strobes)
    task apb_write;
        input [11:0] addr;
        input [31:0] data;
        begin
            @(posedge pclk);
            // Setup phase
            paddr   <= addr;
            psel    <= 1'b1;
            penable <= 1'b0;
            pwrite  <= 1'b1;
            pwdata  <= data;
            pstrb   <= 4'hF;
            @(posedge pclk);
            // Access phase
            penable <= 1'b1;
            @(posedge pclk);
            // Deassert
            psel    <= 1'b0;
            penable <= 1'b0;
            pwrite  <= 1'b0;
        end
    endtask

    // APB Read — returns data via rd_data, also captures pslverr
    task apb_read;
        input [11:0] addr;
        begin
            @(posedge pclk);
            // Setup phase
            paddr   <= addr;
            psel    <= 1'b1;
            penable <= 1'b0;
            pwrite  <= 1'b0;
            pwdata  <= 32'd0;
            pstrb   <= 4'h0;
            @(posedge pclk);
            // Access phase
            penable <= 1'b1;
            @(posedge pclk);
            // prdata is registered — sample one cycle after access
            @(posedge pclk);
            rd_data = prdata;
            // Deassert
            psel    <= 1'b0;
            penable <= 1'b0;
        end
    endtask

    // Generate a single-cycle pulse on an IRQ line
    task pulse_irq;
        input integer which;  // 1..6
        begin
            case (which)
                1: begin irq_timer <= 1'b1; @(posedge pclk); irq_timer <= 1'b0; end
                2: begin irq_uart  <= 1'b1; @(posedge pclk); irq_uart  <= 1'b0; end
                3: begin irq_spi   <= 1'b1; @(posedge pclk); irq_spi   <= 1'b0; end
                4: begin irq_i2c   <= 1'b1; @(posedge pclk); irq_i2c   <= 1'b0; end
                5: begin irq_gpio  <= 1'b1; @(posedge pclk); irq_gpio  <= 1'b0; end
                6: begin irq_accel <= 1'b1; @(posedge pclk); irq_accel <= 1'b0; end
            endcase
        end
    endtask

    // Wait N clock cycles
    task wait_clk;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1)
                @(posedge pclk);
        end
    endtask

    // Check helper
    task check;
        input [255:0] msg;
        input [31:0]  actual;
        input [31:0]  expected;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s : got 0x%08X, expected 0x%08X (time=%0t)",
                         msg, actual, expected, $time);
                err_count = err_count + 1;
            end else begin
                $display("[PASS] %0s : 0x%08X (time=%0t)", msg, actual, $time);
            end
        end
    endtask

    // Main test sequence
    initial begin
        // Init
        err_count  = 0;
        test_num   = 0;
        presetn    = 1'b0;
        psel       = 1'b0;
        penable    = 1'b0;
        pwrite     = 1'b0;
        paddr      = 12'd0;
        pwdata     = 32'd0;
        pstrb      = 4'h0;
        irq_timer  = 1'b0;
        irq_uart   = 1'b0;
        irq_spi    = 1'b0;
        irq_i2c    = 1'b0;
        irq_gpio   = 1'b0;
        irq_accel  = 1'b0;

        // TEST 1: Reset
        test_num = 1;
        $display("\n===== TEST %0d: Reset =====", test_num);
        #(CLK_PERIOD * 5);
        presetn = 1'b1;
        wait_clk(2);

        apb_read(ADDR_IE);
        check("IE after reset", rd_data, 32'd0);

        apb_read(ADDR_PENDING);
        check("PENDING after reset", rd_data, 32'd0);

        apb_read(ADDR_OVERFLOW);
        check("OVERFLOW after reset", rd_data, 32'd0);

        apb_read(ADDR_THRESHOLD);
        check("THRESHOLD after reset", rd_data, 32'd0);

        apb_read(ADDR_CLAIM);
        check("CLAIM after reset", rd_data, 32'd0);

        check("cpu_ext_irq after reset", {31'd0, cpu_ext_irq}, 32'd0);

        // ──────────────────────────────────────────────────
        // TEST 2: Version register
        // ──────────────────────────────────────────────────
        test_num = 2;
        $display("\n===== TEST %0d: Version Register =====", test_num);
        apb_read(ADDR_VERSION);
        check("VERSION_ID", rd_data, 32'h0001_0000);

        // ──────────────────────────────────────────────────
        // TEST 3: IE register write/read
        // ──────────────────────────────────────────────────
        test_num = 3;
        $display("\n===== TEST %0d: IE Register R/W =====", test_num);
        apb_write(ADDR_IE, 32'h0000_003E);  // enable IRQ 1..5
        apb_read(ADDR_IE);
        check("IE readback", rd_data, 32'h0000_003E);

        // ──────────────────────────────────────────────────
        // TEST 4: Priority register write/read
        // ──────────────────────────────────────────────────
        test_num = 4;
        $display("\n===== TEST %0d: Priority Registers R/W =====", test_num);
        // Set priority: timer=3, uart=5, spi=2, i2c=7, gpio=1, accel=4
        apb_write(12'h100, 32'd3);  // priority[1] = 3
        apb_write(12'h104, 32'd5);  // priority[2] = 5
        apb_write(12'h108, 32'd2);  // priority[3] = 2
        apb_write(12'h10C, 32'd7);  // priority[4] = 7
        apb_write(12'h110, 32'd1);  // priority[5] = 1
        apb_write(12'h114, 32'd4);  // priority[6] = 4

        apb_read(12'h100); check("PRI[1] timer", rd_data, 32'd3);
        apb_read(12'h104); check("PRI[2] uart",  rd_data, 32'd5);
        apb_read(12'h108); check("PRI[3] spi",   rd_data, 32'd2);
        apb_read(12'h10C); check("PRI[4] i2c",   rd_data, 32'd7);
        apb_read(12'h110); check("PRI[5] gpio",  rd_data, 32'd1);
        apb_read(12'h114); check("PRI[6] accel", rd_data, 32'd4);

        // ──────────────────────────────────────────────────
        // TEST 5: Threshold register
        // ──────────────────────────────────────────────────
        test_num = 5;
        $display("\n===== TEST %0d: Threshold Register =====", test_num);
        apb_write(ADDR_THRESHOLD, 32'd0);   // threshold = 0  (all pass)
        apb_read(ADDR_THRESHOLD);
        check("THRESHOLD readback", rd_data, 32'd0);

        // ──────────────────────────────────────────────────
        // TEST 6: Single IRQ — edge detect, pending, claim, complete
        // ──────────────────────────────────────────────────
        test_num = 6;
        $display("\n===== TEST %0d: Single IRQ (timer) Edge-Pending-Claim-Complete =====", test_num);

        // Enable all 6 IRQs
        apb_write(ADDR_IE, 32'h0000_007E);  // bits [6:1] = 1
        apb_write(ADDR_THRESHOLD, 32'd0);    // threshold = 0

        // Pulse timer IRQ (ID=1, priority=3)
        pulse_irq(1);
        // Wait for 3-stage sync + edge detection pipeline
        wait_clk(5);

        // Check pending
        apb_read(ADDR_PENDING);
        check("PENDING after timer pulse", rd_data[1], 1'b1);

        // cpu_ext_irq should be high (priority 3 > threshold 0)
        check("cpu_ext_irq asserted", {31'd0, cpu_ext_irq}, 32'd1);

        // Claim — should return ID=1
        apb_read(ADDR_CLAIM);
        check("CLAIM returns timer ID=1", rd_data, 32'd1);

        // Complete — write ID=1 to claim register
        apb_write(ADDR_CLAIM, 32'd1);
        wait_clk(2);

        // Pending should clear
        apb_read(ADDR_PENDING);
        check("PENDING after complete", rd_data[1], 1'b0);

        // cpu_ext_irq should deassert
        wait_clk(1);
        check("cpu_ext_irq deasserted", {31'd0, cpu_ext_irq}, 32'd0);

        // ──────────────────────────────────────────────────
        // TEST 7: Priority arbitration — multiple IRQs
        // ──────────────────────────────────────────────────
        test_num = 7;
        $display("\n===== TEST %0d: Priority Arbitration =====", test_num);

        // Pulse timer(pri=3) and uart(pri=5) simultaneously
        irq_timer <= 1'b1;
        irq_uart  <= 1'b1;
        @(posedge pclk);
        irq_timer <= 1'b0;
        irq_uart  <= 1'b0;
        wait_clk(5);

        // Claim should return uart (ID=2) because priority 5 > 3
        apb_read(ADDR_CLAIM);
        check("CLAIM returns uart ID=2 (higher pri)", rd_data, 32'd2);

        // Complete uart
        apb_write(ADDR_CLAIM, 32'd2);
        wait_clk(2);

        // Now claim should return timer (ID=1)
        apb_read(ADDR_CLAIM);
        check("CLAIM returns timer ID=1 after uart complete", rd_data, 32'd1);

        // Complete timer
        apb_write(ADDR_CLAIM, 32'd1);
        wait_clk(2);

        apb_read(ADDR_PENDING);
        check("PENDING all clear", rd_data, 32'd0);

        // ──────────────────────────────────────────────────
        // TEST 8: Threshold filtering
        // ──────────────────────────────────────────────────
        test_num = 8;
        $display("\n===== TEST %0d: Threshold Filtering =====", test_num);

        // Set threshold = 4, so only priority > 4 passes => uart(5), i2c(7)
        apb_write(ADDR_THRESHOLD, 32'd4);

        // Pulse timer (pri=3) — should NOT raise cpu_ext_irq
        pulse_irq(1);
        wait_clk(5);

        check("cpu_ext_irq LOW (timer pri=3 <= threshold=4)", {31'd0, cpu_ext_irq}, 32'd0);

        // Pending should still show timer
        apb_read(ADDR_PENDING);
        check("PENDING shows timer even below threshold", rd_data[1], 1'b1);

        // Claim should return 0 (nothing above threshold)
        apb_read(ADDR_CLAIM);
        check("CLAIM=0 when below threshold", rd_data, 32'd0);

        // Now pulse uart (pri=5 > threshold=4)
        pulse_irq(2);
        wait_clk(5);

        check("cpu_ext_irq HIGH (uart pri=5 > threshold=4)", {31'd0, cpu_ext_irq}, 32'd1);

        apb_read(ADDR_CLAIM);
        check("CLAIM returns uart ID=2", rd_data, 32'd2);

        // Cleanup: complete both
        apb_write(ADDR_CLAIM, 32'd2);
        wait_clk(1);
        apb_write(ADDR_CLAIM, 32'd1);
        wait_clk(1);
        apb_write(ADDR_THRESHOLD, 32'd0);

        // ──────────────────────────────────────────────────
        // TEST 9: Pending counter — multiple edges accumulate
        // ──────────────────────────────────────────────────
        test_num = 9;
        $display("\n===== TEST %0d: Pending Counter Accumulation =====", test_num);

        // Pulse SPI 3 times
        pulse_irq(3); wait_clk(5);
        pulse_irq(3); wait_clk(5);
        pulse_irq(3); wait_clk(5);

        // Pending should show SPI
        apb_read(ADDR_PENDING);
        check("SPI pending after 3 pulses", rd_data[3], 1'b1);

        // Complete once — counter should go from 3 to 2 (still pending)
        apb_write(ADDR_CLAIM, 32'd3);
        wait_clk(2);
        apb_read(ADDR_PENDING);
        check("SPI still pending after 1 complete", rd_data[3], 1'b1);

        // Complete 2 more times
        apb_write(ADDR_CLAIM, 32'd3);
        wait_clk(2);
        apb_write(ADDR_CLAIM, 32'd3);
        wait_clk(2);

        apb_read(ADDR_PENDING);
        check("SPI cleared after 3 completes", rd_data[3], 1'b0);

        // ──────────────────────────────────────────────────
        // TEST 10: Overflow detection
        // ──────────────────────────────────────────────────
        test_num = 10;
        $display("\n===== TEST %0d: Overflow Detection =====", test_num);

        // Pulse GPIO 16 times (counter max = 15)
        begin : OVERFLOW_PULSE
            integer p;
            for (p = 0; p < 16; p = p + 1) begin
                pulse_irq(5);
                wait_clk(5);
            end
        end

        apb_read(ADDR_OVERFLOW);
        check("GPIO overflow flag set", rd_data[5], 1'b1);

        // Write-1-to-clear overflow for GPIO (bit 5)
        apb_write(ADDR_OVERFLOW, 32'h0000_0020);
        wait_clk(2);

        apb_read(ADDR_OVERFLOW);
        check("GPIO overflow cleared (W1C)", rd_data[5], 1'b0);

        // Cleanup: complete GPIO 15 times
        begin : OVERFLOW_COMPLETE
            integer p;
            for (p = 0; p < 15; p = p + 1) begin
                apb_write(ADDR_CLAIM, 32'd5);
                wait_clk(1);
            end
        end
        wait_clk(2);

        // ──────────────────────────────────────────────────
        // TEST 11: Software force interrupt
        // ──────────────────────────────────────────────────
        test_num = 11;
        $display("\n===== TEST %0d: Software Force Interrupt =====", test_num);

        // Force I2C (bit 4)
        apb_write(ADDR_FORCE, 32'h0000_0010);
        wait_clk(3);

        apb_read(ADDR_PENDING);
        check("I2C pending after force", rd_data[4], 1'b1);

        apb_read(ADDR_CLAIM);
        check("CLAIM returns I2C ID=4 after force", rd_data, 32'd4);

        // Complete
        apb_write(ADDR_CLAIM, 32'd4);
        wait_clk(2);

        apb_read(ADDR_PENDING);
        check("I2C cleared after complete", rd_data[4], 1'b0);

        // ──────────────────────────────────────────────────
        // TEST 12: IE masking
        // ──────────────────────────────────────────────────
        test_num = 12;
        $display("\n===== TEST %0d: IE Masking =====", test_num);

        // Disable all except uart (bit 2)
        apb_write(ADDR_IE, 32'h0000_0004);

        // Pulse timer — disabled, should not raise cpu_ext_irq
        pulse_irq(1);
        wait_clk(5);
        check("cpu_ext_irq LOW (timer disabled)", {31'd0, cpu_ext_irq}, 32'd0);

        // Pending still shows timer
        apb_read(ADDR_PENDING);
        check("Timer pending even when disabled", rd_data[1], 1'b1);

        // Pulse uart — enabled
        pulse_irq(2);
        wait_clk(5);
        check("cpu_ext_irq HIGH (uart enabled)", {31'd0, cpu_ext_irq}, 32'd1);

        // Cleanup
        apb_write(ADDR_IE, 32'h0000_007E);
        apb_write(ADDR_CLAIM, 32'd1);
        wait_clk(1);
        apb_write(ADDR_CLAIM, 32'd2);
        wait_clk(2);

        // ──────────────────────────────────────────────────
        // TEST 13: pslverr on invalid address
        // ──────────────────────────────────────────────────
        test_num = 13;
        $display("\n===== TEST %0d: pslverr on Invalid Address =====", test_num);
        @(posedge pclk);
        paddr   <= 12'hFFF;  // invalid
        psel    <= 1'b1;
        penable <= 1'b0;
        pwrite  <= 1'b0;
        @(posedge pclk);
        penable <= 1'b1;
        @(posedge pclk);
        check("pslverr on invalid addr", {31'd0, pslverr}, 32'd1);
        psel    <= 1'b0;
        penable <= 1'b0;
        wait_clk(1);

        // ──────────────────────────────────────────────────
        // TEST 14: Tie-breaking — same priority, lower ID wins
        // ──────────────────────────────────────────────────
        test_num = 14;
        $display("\n===== TEST %0d: Priority Tie-Breaking =====", test_num);

        // Set timer(1) and spi(3) to same priority = 5
        apb_write(12'h100, 32'd5);  // priority[1] = 5
        apb_write(12'h108, 32'd5);  // priority[3] = 5

        // Pulse both
        irq_timer <= 1'b1;
        irq_spi   <= 1'b1;
        @(posedge pclk);
        irq_timer <= 1'b0;
        irq_spi   <= 1'b0;
        wait_clk(5);

        // Claim should return ID=1 (lower ID wins on tie)
        apb_read(ADDR_CLAIM);
        check("Tie-break: lower ID=1 wins", rd_data, 32'd1);

        // Cleanup
        apb_write(ADDR_CLAIM, 32'd1);
        wait_clk(1);
        apb_write(ADDR_CLAIM, 32'd3);
        wait_clk(1);
        // Restore original priorities
        apb_write(12'h100, 32'd3);
        apb_write(12'h108, 32'd2);

        // ──────────────────────────────────────────────────
        // TEST 15: Full ISR flow — pulse, claim, service, complete
        // ──────────────────────────────────────────────────
        test_num = 15;
        $display("\n===== TEST %0d: Full ISR Flow =====", test_num);
        apb_write(ADDR_THRESHOLD, 32'd0);

        // Trigger 3 interrupts: uart(pri=5), i2c(pri=7), accel(pri=4)
        irq_uart  <= 1'b1;
        irq_i2c   <= 1'b1;
        irq_accel <= 1'b1;
        @(posedge pclk);
        irq_uart  <= 1'b0;
        irq_i2c   <= 1'b0;
        irq_accel <= 1'b0;
        wait_clk(5);

        // ISR loop: claim highest, complete, until no more
        // 1st: i2c (pri 7)
        apb_read(ADDR_CLAIM);
        check("ISR 1st claim: i2c ID=4", rd_data, 32'd4);
        apb_write(ADDR_CLAIM, rd_data[2:0]);
        wait_clk(2);

        // 2nd: uart (pri 5)
        apb_read(ADDR_CLAIM);
        check("ISR 2nd claim: uart ID=2", rd_data, 32'd2);
        apb_write(ADDR_CLAIM, rd_data[2:0]);
        wait_clk(2);

        // 3rd: accel (pri 4)
        apb_read(ADDR_CLAIM);
        check("ISR 3rd claim: accel ID=6", rd_data, 32'd6);
        apb_write(ADDR_CLAIM, rd_data[2:0]);
        wait_clk(2);

        // No more
        apb_read(ADDR_CLAIM);
        check("ISR no more: claim=0", rd_data, 32'd0);
        check("cpu_ext_irq LOW after all serviced", {31'd0, cpu_ext_irq}, 32'd0);

        // ──────────────────────────────────────────────────
        // Summary
        // ──────────────────────────────────────────────────
        #(CLK_PERIOD * 10);
        $display("\n========================================");
        if (err_count == 0)
            $display("  ALL TESTS PASSED (%0d tests)", test_num);
        else
            $display("  %0d FAILURES detected", err_count);
        $display("========================================\n");
        $finish;
    end

    // ── Timeout watchdog ──
    initial begin
        #(CLK_PERIOD * 50000);
        $display("[TIMEOUT] Simulation exceeded maximum time.");
        $finish;
    end

endmodule
