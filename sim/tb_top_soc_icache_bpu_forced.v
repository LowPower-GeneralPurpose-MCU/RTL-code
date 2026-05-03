`timescale 1ns / 1ps

module tb_top_soc_icache_bpu_forced;
    reg clk_core = 0;
    reg clk_bus  = 0;
    reg rst_n_pad = 0;

    // External IOs tied off
    reg  jtag_tck = 0;
    reg  jtag_trst_n = 0;
    reg  jtag_tms = 0;
    reg  jtag_tdi = 0;
    wire jtag_tdo;

    reg  uart_rx = 1'b1;
    wire uart_tx;

    wire spi_sclk, spi_mosi, spi_cs_n;
    reg  spi_miso = 1'b0;

    wire i2c_scl_o, i2c_scl_oen, i2c_sda_o, i2c_sda_oen;
    reg  i2c_scl_i = 1'b1;
    reg  i2c_sda_i = 1'b1;

    reg  [31:0] gpio_in = 32'b0;
    wire [31:0] gpio_out, gpio_dir;

    wire spi_flash_sck, spi_flash_cs_n, spi_flash_mosi;
    reg  spi_flash_miso = 1'b0;

    wire sdram_clk, sdram_cke, sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n;
    wire [1:0] sdram_ba;
    wire [12:0] sdram_addr;
    wire [3:0] sdram_dqm;
    wire [31:0] sdram_dq;

    top_soc dut (
        .clk_core(clk_core), .clk_bus(clk_bus), .rst_n_pad(rst_n_pad),
        .jtag_tck(jtag_tck), .jtag_trst_n(jtag_trst_n), .jtag_tms(jtag_tms), .jtag_tdi(jtag_tdi), .jtag_tdo(jtag_tdo),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso), .spi_cs_n(spi_cs_n),
        .i2c_scl_o(i2c_scl_o), .i2c_scl_oen(i2c_scl_oen), .i2c_scl_i(i2c_scl_i),
        .i2c_sda_o(i2c_sda_o), .i2c_sda_oen(i2c_sda_oen), .i2c_sda_i(i2c_sda_i),
        .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_dir(gpio_dir),
        .spi_flash_sck(spi_flash_sck), .spi_flash_cs_n(spi_flash_cs_n), .spi_flash_mosi(spi_flash_mosi), .spi_flash_miso(spi_flash_miso),
        .sdram_clk(sdram_clk), .sdram_cke(sdram_cke), .sdram_cs_n(sdram_cs_n), .sdram_ras_n(sdram_ras_n), .sdram_cas_n(sdram_cas_n), .sdram_we_n(sdram_we_n),
        .sdram_ba(sdram_ba), .sdram_addr(sdram_addr), .sdram_dqm(sdram_dqm), .sdram_dq(sdram_dq)
    );

    // clocks
    always #5  clk_core = ~clk_core;
    always #10 clk_bus  = ~clk_bus;

    // dump
    initial begin
        $dumpfile("tb_top_soc_icache_bpu_forced.vcd");
        $dumpvars(0, tb_top_soc_icache_bpu_forced);
    end

    // ---------------------------------------------------------------------
    // Helpers: drive I$ path and BPU path by force (keeps full top instantiated,
    // but isolates the specific integration we want to verify).
    // ---------------------------------------------------------------------
    task force_idle_defaults;
        begin
            force dut.ic_arready     = 1'b1;
            force dut.ic_rvalid      = 1'b0;
            force dut.ic_rdata       = 32'b0;
            force dut.ic_rresp       = 2'b00;
            force dut.ic_rlast       = 1'b0;

            force dut.cpu_ic_req     = 1'b0;
            force dut.cpu_ic_addr    = 32'b0;
            force dut.cpu_flush      = 1'b0;
            force dut.dcache_bus_busy = 1'b0;

            // Make pipeline/BPU inputs quiet by default
            force dut.CPU_CORE.pc_in                = 32'b0;
            force dut.CPU_CORE.ex_mem_pc_in         = 32'b0;
            force dut.CPU_CORE.ex_mem_branch        = 1'b0;
            force dut.CPU_CORE.ex_mem_branch_taken  = 1'b0;
            force dut.CPU_CORE.ex_mem_predict_taken = 1'b0;
            force dut.CPU_CORE.ex_mem_btb_hit       = 1'b0;
            force dut.CPU_CORE.ex_mem_branch_target = 32'b0;
        end
    endtask

    task issue_fetch;
        input [31:0] addr;
        begin
            force dut.cpu_ic_req  = 1'b1;
            force dut.cpu_ic_addr = addr;
            force dut.CPU_CORE.pc_in = addr;
        end
    endtask

    task clear_fetch;
        begin
            force dut.cpu_ic_req  = 1'b0;
            force dut.cpu_ic_addr = 32'b0;
            force dut.CPU_CORE.pc_in = 32'b0;
        end
    endtask

    task train_bpu_taken;
        input [31:0] branch_pc;
        input [31:0] target_pc;
        begin
            @(posedge clk_core);
            force dut.CPU_CORE.ex_mem_pc_in         = branch_pc;
            force dut.CPU_CORE.ex_mem_branch        = 1'b1;
            force dut.CPU_CORE.ex_mem_branch_taken  = 1'b1;
            force dut.CPU_CORE.ex_mem_predict_taken = 1'b0;
            force dut.CPU_CORE.ex_mem_btb_hit       = 1'b0;
            force dut.CPU_CORE.ex_mem_branch_target = target_pc;
            @(posedge clk_core);
            force dut.CPU_CORE.ex_mem_branch        = 1'b0;
            force dut.CPU_CORE.ex_mem_branch_taken  = 1'b0;
            force dut.CPU_CORE.ex_mem_predict_taken = 1'b0;
            force dut.CPU_CORE.ex_mem_btb_hit       = 1'b0;
            force dut.CPU_CORE.ex_mem_branch_target = 32'b0;
        end
    endtask

    task service_ic_line;
        input [31:0] line_addr;
        input [63:0] line_data;
        begin
            wait (dut.ic_arvalid === 1'b1 && dut.ic_araddr === line_addr);
            @(posedge clk_core);

            // beat 0
            force dut.ic_rvalid = 1'b1;
            force dut.ic_rresp  = 2'b00;
            force dut.ic_rlast  = 1'b0;
            force dut.ic_rdata  = line_data[31:0];
            wait (dut.ic_rready === 1'b1);
            @(posedge clk_core);

            // beat 1
            force dut.ic_rvalid = 1'b1;
            force dut.ic_rresp  = 2'b00;
            force dut.ic_rlast  = 1'b1;
            force dut.ic_rdata  = line_data[63:32];
            wait (dut.ic_rready === 1'b1);
            @(posedge clk_core);

            force dut.ic_rvalid = 1'b0;
            force dut.ic_rresp  = 2'b00;
            force dut.ic_rlast  = 1'b0;
            force dut.ic_rdata  = 32'b0;
        end
    endtask

    task expect_icache_hit;
        input [31:0] addr;
        begin
            issue_fetch(addr);
            @(posedge clk_core);
            #1;
            if (dut.cpu_ic_hit !== 1'b1 || dut.cpu_ic_stall !== 1'b0) begin
                $display("[FAIL] Expected I$ hit at %h, got hit=%b stall=%b time=%0t", addr, dut.cpu_ic_hit, dut.cpu_ic_stall, $time);
                $finish;
            end
            clear_fetch();
        end
    endtask

    task expect_icache_miss;
        input [31:0] addr;
        begin
            issue_fetch(addr);
            @(posedge clk_core);
            #1;
            if (dut.cpu_ic_hit !== 1'b0 || dut.cpu_ic_stall !== 1'b1) begin
                $display("[FAIL] Expected I$ miss at %h, got hit=%b stall=%b time=%0t", addr, dut.cpu_ic_hit, dut.cpu_ic_stall, $time);
                $finish;
            end
            clear_fetch();
        end
    endtask

    integer pass_count = 0;

    initial begin
        force_idle_defaults();

        repeat (4) @(posedge clk_core);
        rst_n_pad   = 1'b1;
        jtag_trst_n = 1'b1;
        repeat (8) @(posedge clk_core);

        // ================================================================
        // TEST 1: baseline sequential prefetch still works when BPU says NT
        // ================================================================
        $display("[TEST1] sequential fallback prefetch");
        issue_fetch(32'h0000_1000);
        fork
            service_ic_line(32'h0000_1000, 64'h11111111_22222222);
            service_ic_line(32'h0000_1008, 64'h33333333_44444444);
        join
        clear_fetch();
        repeat (3) @(posedge clk_core);
        expect_icache_hit(32'h0000_1008);
        pass_count = pass_count + 1;

        // ================================================================
        // TEST 2: train real BPU, then icache should prefetch branch target
        // ================================================================
        $display("[TEST2] branch-target prefetch via real BPU outputs");
        train_bpu_taken(32'h0000_1010, 32'h2000_0020);
        // now fetch the same branch PC so BPU lookup should hit/taken
        issue_fetch(32'h0000_1010);
        fork
            service_ic_line(32'h0000_1010, 64'hAAAA0001_BBBB0002);
            service_ic_line(32'h2000_0020, 64'hCCCC0003_DDDD0004);
        join
        clear_fetch();
        repeat (3) @(posedge clk_core);
        expect_icache_hit(32'h2000_0020);
        pass_count = pass_count + 1;

        // ================================================================
        // TEST 3: dcache busy should block prefetch
        // ================================================================
        $display("[TEST3] dcache bus busy blocks prefetch");
        force dut.dcache_bus_busy = 1'b1;
        issue_fetch(32'h0000_1020);
        fork
            service_ic_line(32'h0000_1020, 64'h11112222_33334444);
        join
        clear_fetch();
        repeat (3) @(posedge clk_core);
        force dut.dcache_bus_busy = 1'b0;
        expect_icache_miss(32'h0000_1028);
        pass_count = pass_count + 1;

        // ================================================================
        // TEST 4: flush clears prefetched line
        // ================================================================
        $display("[TEST4] flush invalidates prefetched buffer entry");
        issue_fetch(32'h0000_1030);
        fork
            service_ic_line(32'h0000_1030, 64'h55556666_77778888);
            service_ic_line(32'h0000_1038, 64'h9999AAAA_BBBBCCCC);
        join
        clear_fetch();
        repeat (2) @(posedge clk_core);
        force dut.cpu_flush = 1'b1;
        @(posedge clk_core);
        force dut.cpu_flush = 1'b0;
        repeat (2) @(posedge clk_core);
        expect_icache_miss(32'h0000_1038);
        pass_count = pass_count + 1;

        $display("[PASS] %0d tests passed.", pass_count);
        $finish;
    end
endmodule
