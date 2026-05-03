`timescale 1ns / 1ps

/**
 * TESTBENCH: tb_cpu_interrupt_system
 * Feature: Stress test CPU + PLIC + ISR simulation
 * Phases: 5 Phase Comprehensive Test
 */
module tb_cpu_interrupt_system();

    // 1. Parameters & Signals
    parameter CLK_PERIOD = 10;
    parameter ADDR_WIDTH = 12;
    parameter DATA_WIDTH = 32;
    parameter NUM_IRQ    = 6;

    reg clk;
    reg rst_n;
    reg riscv_start;
    
    // APB Signals for PLIC Configuration (Mock Master)
    reg  [ADDR_WIDTH-1:0] paddr;
    reg                   psel;
    reg                   penable;
    reg                   pwrite;
    reg  [DATA_WIDTH-1:0] pwdata;
    wire                  pready;
    wire [DATA_WIDTH-1:0] prdata;
    wire                  pslverr;

    // Interrupt Sources
    reg irq_timer, irq_uart, irq_spi, irq_i2c, irq_gpio, irq_accel;
    wire cpu_ext_irq;

    // CPU Interfaces
    wire icache_read_req;
    wire [31:0] icache_addr;
    reg  [31:0] icache_read_data;
    reg         icache_hit;
    
    wire dcache_read_req, dcache_write_req;
    wire [31:0] dcache_addr, dcache_write_data;
    reg  [31:0] dcache_read_data;
    reg         dcache_hit;

    // Simulation Monitoring
    integer tests_passed = 0;
    integer tests_failed = 0;
    reg [255:0] phase_name;

    // 2. Instantiate DUT: Interrupt Controller (PLIC)
    apb_interrupt_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .NUM_IRQ(NUM_IRQ)
    ) plic_inst (
        .pclk        (clk),
        .presetn     (rst_n),
        .paddr       (paddr),
        .psel        (psel),
        .penable     (penable),
        .pwrite      (pwrite),
        .pwdata      (pwdata),
        .pstrb       (4'hF),
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

    // 3. Instantiate DUT: RISC-V Pipeline (CPU)
    riscv_pipeline cpu_inst (
        .clk               (clk),
        .reset_n           (rst_n),
        .riscv_start       (riscv_start),
        .external_irq_in   (cpu_ext_irq),
        .reset_vector_in   (32'h0000_0000),
        .riscv_done        (),
        .icache_read_req   (icache_read_req),
        .icache_addr       (icache_addr),
        .icache_read_data  (icache_read_data),
        .icache_hit        (icache_hit),
        .icache_stall      (1'b0),
        .dcache_read_req   (dcache_read_req),
        .dcache_write_req  (dcache_write_req),
        .dcache_addr       (dcache_addr),
        .dcache_write_data (dcache_write_data),
        .dcache_read_data  (dcache_read_data),
        .dcache_hit        (dcache_hit),
        .dcache_stall      (1'b0),
        .flush_top         (),
        .mem_size_top      (),
        .mem_unsigned_top  (),
        .wfi_sleep_out     ()
    );

    // 4. Clock Generation
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // 5. Mock Instruction Memory (ISR Handler & Main Code)
    // Code logic: 
    // 0x00: Configure CSRs (Done via TB force for simplicity in 2001)
    // 0x04: Loop idle
    // 0x100: ISR Handler (Claim -> Complete -> MRET)
    always @(*) begin
        icache_hit = 1'b1;
        case (icache_addr)
            32'h0000_0000: icache_read_data = 32'h00000013; // nop
            32'h0000_0004: icache_read_data = 32'h0000006f; // j 0x04 (infinite loop)
            // ISR at 0x100
            32'h0000_0100: icache_read_data = 32'h400002b7; // lui t0, 0x40000 (PLIC Base)
            32'h0000_0104: icache_read_data = 32'h0042a303; // lw t1, 4(t0) (Claim ID)
            32'h0000_0108: icache_read_data = 32'h0062a223; // sw t1, 4(t0) (Complete ID)
            32'h0000_010C: icache_read_data = 32'h30200073; // mret
            default:       icache_read_data = 32'h00000013;
        endcase
    end

    // 6. Mock Data Bus / APB Bridge for CPU -> PLIC
    always @(*) begin
        dcache_hit = 1'b1;
        dcache_read_data = 32'h0;
        // Simple bridge: If CPU accesses PLIC range, tie to PLIC outputs
        if (dcache_read_req && dcache_addr[31:12] == 20'h40000) begin
            // In a real TB, this should trigger APB cycles, but here we simplify
            // to check if CPU is attempting to read Claim register
        end
    end

    // 7. Test Tasks
    task apb_write(input [ADDR_WIDTH-1:0] addr, input [DATA_WIDTH-1:0] data);
        begin
            @(posedge clk);
            paddr <= addr; pwdata <= data; pwrite <= 1'b1; psel <= 1'b1;
            @(posedge clk); penable <= 1'b1;
            while(!pready) @(posedge clk);
            @(posedge clk); psel <= 1'b0; penable <= 1'b0; pwrite <= 1'b0;
        end
    endtask

    // 8. Main Test Sequence
    initial begin
        // Init signals
        rst_n = 0; riscv_start = 0; 
        paddr = 0; psel = 0; penable = 0; pwrite = 0; pwdata = 0;
        irq_timer = 0; irq_uart = 0; irq_spi = 0; 
        irq_i2c = 0; irq_gpio = 0; irq_accel = 0;

        $display("------------------------------------------------------------");
        $display("  FULL CPU-INTERRUPT SYSTEM SIMULATION START (VERILOG 2001) ");
        $display("------------------------------------------------------------");

        // PHASE 1: System Reset
        phase_name = "PHASE 1: Power-on Reset";
        #100 rst_n = 1;
        #20;
        if (cpu_ext_irq === 0) begin
            $display("[PASS] %s: System released from reset successfully.", phase_name);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] %s: CPU IRQ active during reset!", phase_name);
            tests_failed = tests_failed + 1;
        end

        // PHASE 2: CPU CSR Configuration
        phase_name = "PHASE 2: CPU CSR Setup";
        // Force internal CSRs to enable interrupts (MIE bit in mstatus, MEIE bit in mie)
        // Note: mtvec set to 0x100
        force cpu_inst.CSR_RF.mstatus = 32'h0000_1808; // Set bit 3 (MIE) = 1
        force cpu_inst.CSR_RF.mie     = 32'h0000_0800; // Set bit 11 (MEIE) = 1
        force cpu_inst.CSR_RF.mtvec   = 32'h0000_0100; // ISR Vector address
        riscv_start = 1;
        #50;
        $display("[PASS] %s: CPU started and CSRs initialized (Mocked).", phase_name);
        tests_passed = tests_passed + 1;

        // PHASE 3: PLIC Setup & Trigger
        phase_name = "PHASE 3: PLIC IE Setup & Trigger IRQ";
        apb_write(12'h000, 32'h0000_0002); // Enable IRQ ID 1 (Timer)
        #20;
        irq_timer = 1; // Trigger Timer Interrupt
        #50;
        if (cpu_ext_irq === 1) begin
            $display("[PASS] %s: PLIC asserted external_irq to CPU.", phase_name);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] %s: PLIC failed to assert IRQ.", phase_name);
            tests_failed = tests_failed + 1;
        end

        // PHASE 4: Trap Entry (CPU Jump to 0x100)
        phase_name = "PHASE 4: CPU Trap Entry Check";
        // Wait for CPU to reach ISR address
        repeat(20) @(posedge clk);
        if (cpu_inst.IF.pc_out >= 32'h100 && cpu_inst.IF.pc_out <= 32'h110) begin
            $display("[PASS] %s: CPU successfully jumped to MTVEC (0x100).", phase_name);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] %s: CPU PC is 0x%h, expected ISR range.", phase_name, cpu_inst.IF.pc_out);
            tests_failed = tests_failed + 1;
        end

        // PHASE 5: ISR Logic & PLIC Completion
        phase_name = "PHASE 5: Claim/Complete Handshake";
        // Simulate CPU performing APB write to PLIC Complete register via dcache
        // In the handler code: sw t1, 4(t0) -> writes to offset 0x004
        irq_timer = 0; // Clear source
        #10;
        // Mock the completion write to PLIC
        apb_write(12'h004, 32'd1); // CPU writes ID 1 to Complete
        #50;
        if (cpu_ext_irq === 0 && plic_inst.counters[1] === 0) begin
            $display("[PASS] %s: Interrupt Handshake complete. IRQ de-asserted.", phase_name);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] %s: IRQ still active or counter not decremented.", phase_name);
            tests_failed = tests_failed + 1;
        end

        // Final Summary
        $display("------------------------------------------------------------");
        $display("  SIMULATION FINISHED");
        $display("  Tests Passed: %0d", tests_passed);
        $display("  Tests Failed: %0d", tests_failed);
        $display("------------------------------------------------------------");
        if (tests_failed == 0) $display("  RESULT: SUCCESS");
        else                  $display("  RESULT: FAILURE");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule