`timescale 1ns / 1ps

module tb_apb_interrupt_controller();

    // Parameters
    parameter ADDR_WIDTH = 12;
    parameter DATA_WIDTH = 32;
    parameter NUM_IRQ    = 6;
    parameter CLK_PERIOD = 10;

    // Signals
    reg                     pclk;
    reg                     presetn;
    reg  [ADDR_WIDTH-1:0]   paddr;
    reg                     psel;
    reg                     penable;
    reg                     pwrite;
    reg  [DATA_WIDTH-1:0]   pwdata;
    reg  [3:0]              pstrb;
    wire                    pready;
    wire [DATA_WIDTH-1:0]   prdata;
    wire                    pslverr;

    reg                     irq_timer;
    reg                     irq_uart;
    reg                     irq_spi;
    reg                     irq_i2c;
    reg                     irq_gpio;
    reg                     irq_accel;
    wire                    cpu_ext_irq;

    // Testbench monitoring
    integer errors = 0;
    integer tests_passed = 0;
    reg [127:0] phase_name;

    // Instantiate DUT
    apb_interrupt_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .NUM_IRQ(NUM_IRQ)
    ) dut (
        .pclk        (pclk),
        .presetn     (presetn),
        .paddr       (paddr),
        .psel        (psel),
        .penable     (penable),
        .pwrite      (pwrite),
        .pwdata      (pwdata),
        .pstrb       (pstrb),
        .pready      (pready),
        .prdata      (prdata),
        .pslverr     (pslverr),
        .irq_timer   (irq_timer),
        .irq_uart    (irq_uart),
        .irq_spi     (irq_spi),
        .irq_i2c     (irq_i2c),
        .irq_gpio    (irq_gpio),
        .irq_accel   (irq_accel),
        .cpu_ext_irq (cpu_ext_irq)
    );

    // Clock Generation
    initial pclk = 0;
    always #(CLK_PERIOD/2) pclk = ~pclk;

    // --- APB Tasks ---
    task apb_write(input [ADDR_WIDTH-1:0] addr, input [DATA_WIDTH-1:0] data);
        begin
            @(posedge pclk);
            paddr   <= addr;
            pwdata  <= data;
            pwrite  <= 1'b1;
            psel    <= 1'b1;
            pstrb   <= 4'hF;
            @(posedge pclk);
            penable <= 1'b1;
            while (!pready) @(posedge pclk);
            @(posedge pclk);
            psel    <= 1'b0;
            penable <= 1'b0;
        end
    endtask

    task apb_read(input [ADDR_WIDTH-1:0] addr, output [DATA_WIDTH-1:0] data);
        begin
            @(posedge pclk);
            paddr   <= addr;
            pwrite  <= 1'b0;
            psel    <= 1'b1;
            @(posedge pclk);
            penable <= 1'b1;
            while (!pready) @(posedge pclk);
            data = prdata;
            @(posedge pclk);
            psel    <= 1'b0;
            penable <= 1'b0;
        end
    endtask

    // --- IRQ Tasks ---
    task trigger_irq(input integer id);
        begin
            case(id)
                1: irq_timer <= 1; 2: irq_uart <= 1; 3: irq_spi <= 1;
                4: irq_i2c   <= 1; 5: irq_gpio <= 1; 6: irq_accel <= 1;
            endcase
            #(CLK_PERIOD * 2);
            case(id)
                1: irq_timer <= 0; 2: irq_uart <= 0; 3: irq_spi <= 0;
                4: irq_i2c   <= 0; 5: irq_gpio <= 0; 6: irq_accel <= 0;
            endcase
            #(CLK_PERIOD * 2);
        end
    endtask

    // --- Check Macro-like Task ---
    task check_value(input [DATA_WIDTH-1:0] actual, input [DATA_WIDTH-1:0] expected, input [127:0] msg);
        begin
            if (actual !== expected) begin
                $display("[FAIL] %s | Expected: %h, Actual: %h", msg, expected, actual);
                errors = errors + 1;
            end else begin
                $display("[PASS] %s", msg);
                tests_passed = tests_passed + 1;
            end
        end
    endtask

    // Main Test Sequence
    reg [DATA_WIDTH-1:0] rdata;
    integer k;

    initial begin
        // Initialize
        presetn     = 0;
        paddr       = 0;
        psel        = 0;
        penable     = 0;
        pwrite      = 0;
        pwdata      = 0;
        pstrb       = 0;
        irq_timer   = 0; irq_uart = 0; irq_spi = 0;
        irq_i2c     = 0; irq_gpio = 0; irq_accel = 0;

        $display("---------------------------------------------------------");
        $display("STARTING COMPREHENSIVE INTERRUPT CONTROLLER TEST");
        $display("---------------------------------------------------------");

        // PHASE 1: RESET & INITIAL STATE
        phase_name = "Phase 1: Reset";
        #50 presetn = 1;
        apb_read(12'h000, rdata); check_value(rdata, 32'h0, "IE initial state");
        apb_read(12'h00C, rdata); check_value(rdata, 32'h0, "Pending initial state");
        check_value(cpu_ext_irq, 1'b0, "CPU IRQ line initial state");

        // PHASE 2: IE REGISTER ACCESS
        phase_name = "Phase 2: IE Access";
        apb_write(12'h000, 32'h0000_007E); // Enable all 6 IRQs
        apb_read(12'h000, rdata); 
        check_value(rdata, 32'h0000_007E, "IE read back (ID 1-6 shifted)"); // Note: user logic shifts ie output by 1 bit: {ie, 1'b0}

        // PHASE 3: SINGLE INTERRUPT TRIGGER
        phase_name = "Phase 3: Single IRQ";
        trigger_irq(1); // Timer
        #20;
        check_value(cpu_ext_irq, 1'b1, "CPU IRQ after Timer trigger");
        apb_read(12'h00C, rdata); check_value(rdata, 32'h0000_0002, "Pending status bit 1");

        // PHASE 4: CLAIM & COMPLETE
        phase_name = "Phase 4: Claim/Complete";
        apb_read(12'h004, rdata); check_value(rdata, 32'd1, "Claim ID check");
        apb_write(12'h004, 32'd1); // Complete IRQ 1
        #20;
        check_value(cpu_ext_irq, 1'b0, "CPU IRQ after completion");

        // PHASE 5: MULTI-INTERRUPT ROUND ROBIN
        phase_name = "Phase 5: Round Robin";
        trigger_irq(2); // UART
        trigger_irq(5); // GPIO
        #20;
        apb_read(12'h004, rdata); check_value(rdata, 32'd2, "Claim first (ID 2)");
        apb_write(12'h004, 32'd2);
        #20;
        apb_read(12'h004, rdata); check_value(rdata, 32'd5, "Claim second (ID 5)");
        apb_write(12'h004, 32'd5);

        // PHASE 6: SIMULTANEOUS TRIGGER PRIORITY
        phase_name = "Phase 6: Simultaneous";
        {irq_accel, irq_timer} = 2'b11; #(CLK_PERIOD*2); {irq_accel, irq_timer} = 2'b00;
        #20;
        apb_read(12'h004, rdata); // Since last_served was 5, ID 6 should be next in RR
        check_value(rdata, 32'd6, "RR Priority after ID 5");
        apb_write(12'h004, 32'd6);
        apb_read(12'h004, rdata);
        check_value(rdata, 32'd1, "RR Wrap around to ID 1");
        apb_write(12'h004, 32'd1);

        // PHASE 7: QUEUE COUNTER STRESS (Multiple triggers same ID)
        phase_name = "Phase 7: Queue Stress";
        for(k=0; k<5; k=k+1) trigger_irq(3); // Trigger SPI 5 times
        #20;
        apb_read(12'h00C, rdata); check_value(rdata, 32'h0000_0008, "Pending status for ID 3");
        for(k=0; k<5; k=k+1) begin
            apb_read(12'h004, rdata);
            apb_write(12'h004, 32'd3);
        end
        #20;
        check_value(cpu_ext_irq, 1'b0, "IRQ clear after draining queue");

        // PHASE 8: OVERFLOW TEST
        phase_name = "Phase 8: Overflow";
        for(k=0; k<17; k=k+1) trigger_irq(4); // ID 4 (Max depth 15)
        #20;
        apb_read(12'h008, rdata); 
        check_value(rdata, 32'h0000_0010, "Overflow bit 4 set");

        // PHASE 9: OVERFLOW CLEARING
        phase_name = "Phase 9: Overflow Clear";
        apb_read(12'h008, rdata); // Read clears overflow
        #20;
        apb_read(12'h008, rdata); check_value(rdata, 32'h0, "Overflow cleared after read");
        // Drain the 15 items in queue to clean up
        for(k=0; k<15; k=k+1) apb_write(12'h004, 32'd4);

        // PHASE 10: DISABLED INTERRUPT TEST
        phase_name = "Phase 10: Masking";
        apb_write(12'h000, 32'h0); // Disable all
        trigger_irq(1);
        #20;
        check_value(cpu_ext_irq, 1'b0, "CPU IRQ masked when IE=0");
        apb_read(12'h00C, rdata); check_value(rdata, 32'h0000_0002, "Pending bit still set even if masked");

        // PHASE 11: SLAVE ERROR ACCESS
        phase_name = "Phase 11: Slave Error";
        apb_write(12'hF00, 32'hDEADBEEF); // Invalid address
        check_value(pslverr, 1'b1, "PSLVERR on invalid write");
        apb_read(12'h008, rdata); // Read valid to check pslverr recovery
        check_value(pslverr, 1'b0, "PSLVERR recovery on valid read");

        // PHASE 12: RACE CONDITION STRESS
        phase_name = "Phase 12: Race Stress";
        // Trigger IRQ and Complete IRQ in same cycle (handled by internal logic)
        apb_write(12'h000, 32'hFF); // Enable all
        fork
            trigger_irq(2);
            apb_write(12'h004, 32'd2);
        join
        #50;
        $display("---------------------------------------------------------");
        $display("TEST SUMMARY");
        $display("Tests Passed: %0d", tests_passed);
        $display("Tests Failed: %0d", errors);
        $display("---------------------------------------------------------");
        if (errors == 0) $display("RESULT: ALL TESTS PASSED");
        else             $display("RESULT: TEST FAILED");
        $display("---------------------------------------------------------");
        $finish;
    end

endmodule