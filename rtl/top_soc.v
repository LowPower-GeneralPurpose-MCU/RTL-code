`timescale 1ns / 1ps

module top_soc (
    // --- Hệ thống Clock và Reset (2 Miền Độc Lập) ---
    input  wire        clk_core,     // Ví dụ: 400MHz cho CPU, Cache
    input  wire        clk_bus,      // Ví dụ: 200MHz cho RAM, Bus, Ngoại vi
    input  wire        rst_n_pad,    // Reset cứng từ Pad (Active Low)

    // --- Giao tiếp JTAG ---
    input  wire        jtag_tck,
    input  wire        jtag_trst_n,
    input  wire        jtag_tms,
    input  wire        jtag_tdi,
    output wire        jtag_tdo,

    // --- UART Interface ---
    input  wire        uart_rx,
    output wire        uart_tx,

    // --- SPI Interface ---
    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs_n,

    // --- I2C Interface ---
    output wire        i2c_scl_o,
    output wire        i2c_scl_oen,
    input  wire        i2c_scl_i,
    output wire        i2c_sda_o,
    output wire        i2c_sda_oen,
    input  wire        i2c_sda_i,

    // --- GPIO Interface ---
    input  wire [31:0] gpio_in,
    output wire [31:0] gpio_out,
    output wire [31:0] gpio_dir,

    // --- AXI SPI Flash Ports ---
    output wire        spi_flash_sck,
    output wire        spi_flash_cs_n,
    output wire        spi_flash_mosi,
    input  wire        spi_flash_miso,

    // --- Off-Chip SDRAM Physical Ports (Mới thêm) ---
    output wire        sdram_clk,
    output wire        sdram_cke,
    output wire        sdram_cs_n,
    output wire        sdram_ras_n,
    output wire        sdram_cas_n,
    output wire        sdram_we_n,
    output wire [1:0]  sdram_ba,
    output wire [12:0] sdram_addr,
    output wire [3:0]  sdram_dqm,
    inout  wire [31:0] sdram_dq    // Inout bus hai chiều đặc thù của RAM
);

    // =========================================================================
    // 1. SDRAM TRI-STATE BUFFER LOGIC
    // =========================================================================
    wire [31:0] sdram_dq_in;
    wire [31:0] sdram_dq_out;
    wire        sdram_dq_oe;

    assign sdram_dq = sdram_dq_oe ? sdram_dq_out : 32'bz;
    assign sdram_dq_in = sdram_dq;

    // =========================================================================
    // 2. WATCHDOG RESET VÀ RESET SYNCHRONIZERS
    // =========================================================================
    wire wdt_rst_req; 
    wire global_rst_n = rst_n_pad & ~wdt_rst_req; // Watchdog có quyền kéo reset toàn bộ hệ thống

    reg [2:0] core_rst_sync;
    always @(posedge clk_core or negedge global_rst_n) begin
        if (!global_rst_n) core_rst_sync <= 3'b000;
        else               core_rst_sync <= {core_rst_sync[1:0], 1'b1};
    end
    wire core_rst_n = core_rst_sync[2];

    reg [2:0] bus_rst_sync;
    always @(posedge clk_bus or negedge global_rst_n) begin
        if (!global_rst_n) bus_rst_sync <= 3'b000;
        else               bus_rst_sync <= {bus_rst_sync[1:0], 1'b1};
    end
    wire bus_rst_n = bus_rst_sync[2];

    // =========================================================================
    // 3. KHAI BÁO TÍN HIỆU KẾT NỐI NỘI BỘ
    // =========================================================================
    wire [31:0] sys_reset_vector;
    wire        cpu_wfi_sleep;
    wire        cpu_flush;
    wire        riscv_done;

    // Caches <-> CPU
    wire        cpu_ic_req, cpu_ic_hit, cpu_ic_stall;
    wire [31:0] cpu_ic_addr, cpu_ic_rdata;
    wire        cpu_dc_rd, cpu_dc_wr, cpu_dc_hit, cpu_dc_stall, cpu_mem_unsigned;
    wire [31:0] cpu_dc_addr, cpu_dc_wdata, cpu_dc_rdata;
    wire [1:0]  cpu_dc_size;
    wire dcache_bus_busy;

    // Caches <-> CDC Bridges <-> Bus
    wire        dc_cdc_rd, dc_cdc_wr, dc_cdc_ready, dc_cdc_resp_val;
    wire [31:0] dc_cdc_addr, dc_cdc_wdata, dc_cdc_rdata;
    wire [1:0]  dc_cdc_size;
    wire        dc_bus_req, dc_bus_is_write, dc_bus_ready, dc_bus_resp_val;
    wire [31:0] dc_bus_addr, dc_bus_wdata, dc_bus_rdata;
    wire [1:0]  dc_bus_size;

    wire [31:0] ic_araddr;
    wire [7:0]  ic_arlen;
    wire [2:0]  ic_arsize;
    wire [1:0]  ic_arburst;
    wire        ic_arvalid, ic_arready, ic_rvalid, ic_rready, ic_rlast;
    wire [31:0] ic_rdata;
    wire [1:0]  ic_rresp;

    // Tín hiệu Debug SBA/JTAG
    wire        dmi_req_valid_tck, dmi_resp_ready_tck, dmi_resp_valid_tck;
    wire [6:0]  dmi_req_addr_tck;
    wire [31:0] dmi_req_data_tck, dmi_resp_data_tck;
    wire [1:0]  dmi_req_op_tck, dmi_resp_op_tck;
    wire        dmi_req_valid_sys, dmi_resp_ready_sys, dmi_resp_valid_sys;
    wire [6:0]  dmi_req_addr_sys;
    wire [31:0] dmi_req_data_sys, dmi_resp_data_sys;
    wire [1:0]  dmi_req_op_sys, dmi_resp_op_sys;
    wire        debug_axi_req, debug_axi_ack;
    wire [1:0]  debug_axi_op, debug_axi_resp;
    wire [31:0] debug_axi_addr, debug_axi_wdata, debug_axi_rdata;

    // Clock Gating
    wire clk_en_cpu, clk_en_dbg, clk_en_tmr, clk_en_urt, clk_en_spi, clk_en_i2c, clk_en_gpo, clk_en_acc;
    wire clk_cpu_gated, clk_dbg_gated, clk_tmr_gated, clk_urt_gated, clk_spi_gated, clk_i2c_gated, clk_gpo_gated, clk_acc_gated;

    // =========================================================================
    // 4. INTERRUPT SYNCHRONIZERS (Từ clk_bus sang clk_core)
    // =========================================================================
    wire plic_ext_irq, clint_software_irq, clint_timer_irq;
    reg [1:0] sync_meip, sync_msip, sync_mtip;
    always @(posedge clk_core or negedge core_rst_n) begin
        if (!core_rst_n) begin
            sync_meip <= 2'b00;
            sync_msip <= 2'b00;
            sync_mtip <= 2'b00;
        end else begin
            sync_meip <= {sync_meip[0], plic_ext_irq};
            sync_msip <= {sync_msip[0], clint_software_irq};
            sync_mtip <= {sync_mtip[0], clint_timer_irq};
        end
    end
    wire core_meip = sync_meip[1];
    wire core_msip = sync_msip[1];
    wire core_mtip = sync_mtip[1];

    // =========================================================================
    // 5. CLOCK GATING & JTAG DEBUG SYSTEM
    // =========================================================================
    clock_gate CG_CPU (.clk_in(clk_core), .en(clk_en_cpu), .test_en(1'b0), .clk_out(clk_cpu_gated));
    clock_gate CG_DBG (.clk_in(clk_core), .en(clk_en_dbg), .test_en(1'b0), .clk_out(clk_dbg_gated));
    clock_gate CG_TMR (.clk_in(clk_bus),  .en(clk_en_tmr), .test_en(1'b0), .clk_out(clk_tmr_gated));
    clock_gate CG_URT (.clk_in(clk_bus),  .en(clk_en_urt), .test_en(1'b0), .clk_out(clk_urt_gated));
    clock_gate CG_SPI (.clk_in(clk_bus),  .en(clk_en_spi), .test_en(1'b0), .clk_out(clk_spi_gated));
    clock_gate CG_I2C (.clk_in(clk_bus),  .en(clk_en_i2c), .test_en(1'b0), .clk_out(clk_i2c_gated));
    clock_gate CG_GPO (.clk_in(clk_bus),  .en(clk_en_gpo), .test_en(1'b0), .clk_out(clk_gpo_gated));
    clock_gate CG_ACC (.clk_in(clk_bus),  .en(clk_en_acc), .test_en(1'b0), .clk_out(clk_acc_gated));

    rv_jtag_dtm #( .ABITS(7) ) JTAG_DTM_INST (
        .tck(jtag_tck), .trst_n(jtag_trst_n), .tms(jtag_tms), .tdi(jtag_tdi), .tdo(jtag_tdo),
        .dmi_req_valid(dmi_req_valid_tck), .dmi_req_addr(dmi_req_addr_tck), .dmi_req_data(dmi_req_data_tck), .dmi_req_op(dmi_req_op_tck),
        .dmi_resp_ready(dmi_resp_ready_tck), .dmi_resp_valid(dmi_resp_valid_tck), .dmi_resp_data(dmi_resp_data_tck), .dmi_resp_op(dmi_resp_op_tck)
    );

    // CDC DMI Handshake (TCK <-> SYS)
    reg [1:0] req_valid_sync, resp_ready_sync, resp_valid_sync, req_ready_sync;
    always @(posedge clk_dbg_gated or negedge core_rst_n) begin
        if (!core_rst_n) begin req_valid_sync <= 2'b00; resp_ready_sync <= 2'b00; end 
        else             begin req_valid_sync <= {req_valid_sync[0], dmi_req_valid_tck}; resp_ready_sync <= {resp_ready_sync[0], dmi_resp_ready_tck}; end
    end
    always @(posedge jtag_tck or negedge jtag_trst_n) begin
        if (!jtag_trst_n) begin resp_valid_sync <= 2'b00; req_ready_sync <= 2'b00; end 
        else              begin resp_valid_sync <= {resp_valid_sync[0], dmi_resp_valid_sys}; req_ready_sync <= {req_ready_sync[0], dmi_resp_ready_sys}; end
    end
    assign dmi_req_valid_sys  = req_valid_sync[1];
    assign dmi_req_addr_sys   = dmi_req_addr_tck;
    assign dmi_req_data_sys   = dmi_req_data_tck;
    assign dmi_req_op_sys     = dmi_req_op_tck;
    assign dmi_resp_ready_sys = resp_ready_sync[1];
    assign dmi_resp_valid_tck = resp_valid_sync[1];
    assign dmi_resp_data_tck  = dmi_resp_data_sys;
    assign dmi_resp_op_tck    = dmi_resp_op_sys;

    rv_debug_module_sba DBG_MODULE_SBA_INST (
        .clk_sys(clk_dbg_gated), .rst_sys_n(core_rst_n),
        .dmi_req_valid(dmi_req_valid_sys), .dmi_req_addr(dmi_req_addr_sys), .dmi_req_data(dmi_req_data_sys), .dmi_req_op(dmi_req_op_sys),
        .dmi_resp_ready(dmi_resp_ready_sys), .dmi_resp_valid(dmi_resp_valid_sys), .dmi_resp_data(dmi_resp_data_sys), .dmi_resp_op(dmi_resp_op_sys),
        .axi_req(debug_axi_req), .axi_op(debug_axi_op), .axi_addr(debug_axi_addr), .axi_wdata(debug_axi_wdata), .axi_ack(debug_axi_ack), .axi_rdata(debug_axi_rdata), .axi_resp(debug_axi_resp)
    );

    // =========================================================================
    // 6. KHAI BÁO TÍN HIỆU AXI4 (Miền Bus) - 4 Masters, 8 Slaves
    // =========================================================================
    // MASTERS
    wire [31:0] m0_araddr;
    wire [7:0]  m0_arlen;
    wire [2:0]  m0_arsize;
    wire [1:0]  m0_arburst;
    wire        m0_arvalid, m0_arready, m0_rvalid, m0_rready, m0_rlast;
    wire [31:0] m0_rdata;
    wire [1:0]  m0_rresp;

    wire [31:0] m1_araddr, m1_awaddr, m1_wdata, m1_rdata;
    wire [3:0]  m1_wstrb;
    wire [1:0]  m1_bresp, m1_rresp;
    wire        m1_awvalid, m1_awready, m1_wvalid, m1_wready, m1_bvalid, m1_bready, m1_arvalid, m1_arready, m1_rvalid, m1_rready;

    wire [31:0] m2_araddr, m2_awaddr, m2_wdata, m2_rdata;
    wire [3:0]  m2_wstrb;
    wire [1:0]  m2_bresp, m2_rresp;
    wire        m2_awvalid, m2_awready, m2_wvalid, m2_wready, m2_bvalid, m2_bready, m2_arvalid, m2_arready, m2_rvalid, m2_rready;

    wire [31:0] m3_araddr, m3_awaddr, m3_wdata, m3_rdata;
    wire [3:0]  m3_wstrb;
    wire [1:0]  m3_bresp, m3_rresp;
    wire        m3_awvalid, m3_awready, m3_wvalid, m3_wready, m3_bvalid, m3_bready, m3_arvalid, m3_arready, m3_rvalid, m3_rready;

    // SLAVES
    wire [31:0] s0_araddr, s0_rdata; wire [1:0] s0_rresp; wire s0_arvalid, s0_arready, s0_rvalid, s0_rready;
    
    wire [31:0] s1_awaddr, s1_araddr, s1_wdata, s1_rdata; wire [3:0] s1_wstrb; wire [1:0] s1_bresp, s1_rresp; wire [7:0] s1_arlen; wire [2:0] s1_arsize; wire [1:0] s1_arburst;
    wire s1_awvalid, s1_awready, s1_wvalid, s1_wready, s1_bvalid, s1_bready, s1_arvalid, s1_arready, s1_rvalid, s1_rready, s1_rlast;

    wire [31:0] s2_awaddr, s2_araddr, s2_wdata, s2_rdata; wire [3:0] s2_wstrb; wire [1:0] s2_bresp, s2_rresp;
    wire s2_awvalid, s2_awready, s2_wvalid, s2_wready, s2_bvalid, s2_bready, s2_arvalid, s2_arready, s2_rvalid, s2_rready;

    wire [31:0] s3_araddr, s3_rdata; wire [1:0] s3_rresp; wire s3_arvalid, s3_arready, s3_rvalid, s3_rready;

    wire [31:0] s4_awaddr, s4_araddr, s4_wdata, s4_rdata; wire [3:0] s4_wstrb; wire [1:0] s4_bresp, s4_rresp;
    wire s4_awvalid, s4_awready, s4_wvalid, s4_wready, s4_bvalid, s4_bready, s4_arvalid, s4_arready, s4_rvalid, s4_rready;

    wire [31:0] s5_awaddr, s5_araddr, s5_wdata, s5_rdata; wire [3:0] s5_wstrb; wire [1:0] s5_bresp, s5_rresp;
    wire s5_awvalid, s5_awready, s5_wvalid, s5_wready, s5_bvalid, s5_bready, s5_arvalid, s5_arready, s5_rvalid, s5_rready;

    wire [31:0] s6_awaddr, s6_araddr, s6_wdata, s6_rdata; wire [3:0] s6_wstrb; wire [1:0] s6_bresp, s6_rresp; wire [7:0] s6_arlen; wire [2:0] s6_arsize; wire [1:0] s6_arburst;
    wire s6_awvalid, s6_awready, s6_wvalid, s6_wready, s6_bvalid, s6_bready, s6_arvalid, s6_arready, s6_rvalid, s6_rready, s6_rlast;

    wire [31:0] s7_awaddr, s7_araddr, s7_wdata, s7_rdata; wire [3:0] s7_wstrb; wire [1:0] s7_bresp, s7_rresp;
    wire s7_awvalid, s7_awready, s7_wvalid, s7_wready, s7_bvalid, s7_bready, s7_arvalid, s7_arready, s7_rvalid, s7_rready;

    // =========================================================================
    // 7. CPU CORE VÀ CACHES
    // =========================================================================
    
    wire        bpu_predict_taken;
    wire        bpu_btb_hit;
    wire [31:0] bpu_predict_target;

    riscv_pipeline CPU_CORE (
        .clk(clk_cpu_gated), .reset_n(core_rst_n), .riscv_start(1'b1),
        .meip_i(core_meip), .msip_i(core_msip), .mtip_i(core_mtip), // Cập nhật cổng ngắt chuẩn RISC-V
        .reset_vector_in(sys_reset_vector), .wfi_sleep_out(cpu_wfi_sleep), .riscv_done(riscv_done),
        .icache_read_req(cpu_ic_req), .icache_addr(cpu_ic_addr), .icache_read_data(cpu_ic_rdata), .icache_hit(cpu_ic_hit), .icache_stall(cpu_ic_stall),.bpu_predict_taken_out(bpu_predict_taken), .bpu_btb_hit_out(bpu_btb_hit), .bpu_predict_target_out(bpu_predict_target),
        .dcache_read_req(cpu_dc_rd), .dcache_write_req(cpu_dc_wr), .dcache_addr(cpu_dc_addr), .dcache_write_data(cpu_dc_wdata), .dcache_read_data(cpu_dc_rdata), .dcache_hit(cpu_dc_hit), .dcache_stall(cpu_dc_stall),
        .flush_top(cpu_flush), .mem_size_top(cpu_dc_size), .mem_unsigned_top(cpu_mem_unsigned)
    );

    instruction_cache I_CACHE (
        .clk(clk_core), .rst_n(core_rst_n), .flush(cpu_flush),.dcache_busy(dcache_bus_busy),
        .predict_taken(bpu_predict_taken), .btb_hit(bpu_btb_hit), .predict_target(bpu_predict_target),
        .cpu_read_req(cpu_ic_req), .cpu_addr(cpu_ic_addr), .cpu_read_data(cpu_ic_rdata), .icache_hit(cpu_ic_hit), .icache_stall(cpu_ic_stall),
        .m_axi_araddr(ic_araddr), .m_axi_arlen(ic_arlen), .m_axi_arsize(ic_arsize), .m_axi_arburst(ic_arburst), .m_axi_arvalid(ic_arvalid), .m_axi_arready(ic_arready),
        .m_axi_rdata(ic_rdata), .m_axi_rresp(ic_rresp), .m_axi_rlast(ic_rlast), .m_axi_rvalid(ic_rvalid), .m_axi_rready(ic_rready)
    );

    data_cache D_CACHE (
        .clk(clk_core), .rst_n(core_rst_n),
        .cpu_read_req(cpu_dc_rd), .cpu_write_req(cpu_dc_wr), .cpu_addr(cpu_dc_addr), .cpu_write_data(cpu_dc_wdata), .cpu_read_data(cpu_dc_rdata),
        .mem_unsigned(cpu_mem_unsigned), .mem_size(cpu_dc_size), .dcache_hit(cpu_dc_hit), .dcache_stall(cpu_dc_stall), .dcache_bus_busy(dcache_bus_busy),
        .mem_read_req(dc_cdc_rd), .mem_write_req(dc_cdc_wr), .mem_addr(dc_cdc_addr), .mem_write_data(dc_cdc_wdata), .mem_read_data(dc_cdc_rdata),
        .mem_read_ready(dc_cdc_ready), .mem_read_valid(dc_cdc_resp_val), .mem_write_ready(dc_cdc_ready), .mem_write_back_valid(dc_cdc_resp_val)
    );

    // =========================================================================
    // 8. CDC BRIDGES VÀ AXI ADAPTERS (M0, M1, M2)
    // =========================================================================
    axi4_read_cdc ICACHE_CDC (
        .clk_core(clk_core), .rst_core_n(core_rst_n),
        .s_axi_araddr(ic_araddr), .s_axi_arlen(ic_arlen), .s_axi_arsize(ic_arsize), .s_axi_arburst(ic_arburst), .s_axi_arvalid(ic_arvalid), .s_axi_arready(ic_arready),
        .s_axi_rdata(ic_rdata), .s_axi_rresp(ic_rresp), .s_axi_rlast(ic_rlast), .s_axi_rvalid(ic_rvalid), .s_axi_rready(ic_rready),
        .clk_bus(clk_bus), .rst_bus_n(bus_rst_n),
        .m_axi_araddr(m0_araddr), .m_axi_arlen(m0_arlen), .m_axi_arsize(m0_arsize), .m_axi_arburst(m0_arburst), .m_axi_arvalid(m0_arvalid), .m_axi_arready(m0_arready),
        .m_axi_rdata(m0_rdata), .m_axi_rresp(m0_rresp), .m_axi_rlast(m0_rlast), .m_axi_rvalid(m0_rvalid), .m_axi_rready(m0_rready)
    );

    native_cdc_bridge D_CDC_BRG (
        .cpu_clk(clk_core), .cpu_rst_n(core_rst_n),
        .cpu_req_val(dc_cdc_rd || dc_cdc_wr), .cpu_req_is_write(dc_cdc_wr), .cpu_req_addr(dc_cdc_addr), .cpu_req_wdata(dc_cdc_wdata), .cpu_req_wstrb(4'b1111), .cpu_req_size(dc_cdc_size), .cpu_req_ready(dc_cdc_ready),
        .cpu_resp_val(dc_cdc_resp_val), .cpu_resp_rdata(dc_cdc_rdata),
        .bus_clk(clk_bus), .bus_rst_n(bus_rst_n),
        .bus_req_val(dc_bus_req), .bus_req_is_write(dc_bus_is_write), .bus_req_addr(dc_bus_addr), .bus_req_wdata(dc_bus_wdata), .bus_req_wstrb(), .bus_req_size(dc_bus_size), .bus_req_ready(dc_bus_ready),
        .bus_resp_val(dc_bus_resp_val), .bus_resp_rdata(dc_bus_rdata)
    );

    axi_master_adapter M1_DCACHE_ADAPT (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .cpu_read_req(dc_bus_req && !dc_bus_is_write), .cpu_write_req(dc_bus_req && dc_bus_is_write), .cpu_addr(dc_bus_addr), .cpu_wdata(dc_bus_wdata), .cpu_mem_size(dc_bus_size), .cpu_rdata(dc_bus_rdata), .cpu_ready(dc_bus_ready),
        .m_axi_awaddr(m1_awaddr), .m_axi_awvalid(m1_awvalid), .m_axi_awready(m1_awready), .m_axi_wdata(m1_wdata), .m_axi_wstrb(m1_wstrb), .m_axi_wvalid(m1_wvalid), .m_axi_wready(m1_wready), .m_axi_bresp(m1_bresp), .m_axi_bvalid(m1_bvalid), .m_axi_bready(m1_bready),
        .m_axi_araddr(m1_araddr), .m_axi_arvalid(m1_arvalid), .m_axi_arready(m1_arready), .m_axi_rdata(m1_rdata), .m_axi_rresp(m1_rresp), .m_axi_rvalid(m1_rvalid), .m_axi_rready(m1_rready)
    );

    dtm_axi_master #(.ADDR_WIDTH(32)) M2_DTM_AXI_INST (
        .clk_sys(clk_bus), .rst_sys_n(bus_rst_n),
        .i_req(debug_axi_req), .i_op(debug_axi_op), .i_addr(debug_axi_addr), .i_wdata(debug_axi_wdata), .o_ack(debug_axi_ack), .o_resp(debug_axi_resp), .o_rdata(debug_axi_rdata),
        .m_axi_awaddr(m2_awaddr), .m_axi_awvalid(m2_awvalid), .m_axi_awready(m2_awready), .m_axi_wdata(m2_wdata), .m_axi_wstrb(m2_wstrb), .m_axi_wvalid(m2_wvalid), .m_axi_wready(m2_wready), .m_axi_bresp(m2_bresp), .m_axi_bvalid(m2_bvalid), .m_axi_bready(m2_bready),
        .m_axi_araddr(m2_araddr), .m_axi_arvalid(m2_arvalid), .m_axi_arready(m2_arready), .m_axi_rdata(m2_rdata), .m_axi_rresp(m2_rresp), .m_axi_rvalid(m2_rvalid), .m_axi_rready(m2_rready)
    );

    // =========================================================================
    // 9. AXI INTERCONNECT VÀ SLAVES CHÍNH (ROM, RAM, FLASH, CLINT, PLIC, SDRAM, DMA)
    // =========================================================================
    axi_interconnect MAIN_BUS_MATRIX (
        .clk(clk_bus), .rst_n(bus_rst_n),
        
        .m0_araddr(m0_araddr), .m0_arlen(m0_arlen), .m0_arsize(m0_arsize), .m0_arburst(m0_arburst), .m0_arvalid(m0_arvalid), .m0_arready(m0_arready), .m0_rdata(m0_rdata), .m0_rresp(m0_rresp), .m0_rlast(m0_rlast), .m0_rvalid(m0_rvalid), .m0_rready(m0_rready),
        .m1_awaddr(m1_awaddr), .m1_awvalid(m1_awvalid), .m1_awready(m1_awready), .m1_wdata(m1_wdata), .m1_wstrb(m1_wstrb), .m1_wvalid(m1_wvalid), .m1_wready(m1_wready), .m1_bresp(m1_bresp), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready), .m1_araddr(m1_araddr), .m1_arvalid(m1_arvalid), .m1_arready(m1_arready), .m1_rdata(m1_rdata), .m1_rresp(m1_rresp), .m1_rvalid(m1_rvalid), .m1_rready(m1_rready),
        .m2_awaddr(m2_awaddr), .m2_awvalid(m2_awvalid), .m2_awready(m2_awready), .m2_wdata(m2_wdata), .m2_wstrb(m2_wstrb), .m2_wvalid(m2_wvalid), .m2_wready(m2_wready), .m2_bresp(m2_bresp), .m2_bvalid(m2_bvalid), .m2_bready(m2_bready), .m2_araddr(m2_araddr), .m2_arvalid(m2_arvalid), .m2_arready(m2_arready), .m2_rdata(m2_rdata), .m2_rresp(m2_rresp), .m2_rvalid(m2_rvalid), .m2_rready(m2_rready),
        .m3_awaddr(m3_awaddr), .m3_awvalid(m3_awvalid), .m3_awready(m3_awready), .m3_wdata(m3_wdata), .m3_wstrb(m3_wstrb), .m3_wvalid(m3_wvalid), .m3_wready(m3_wready), .m3_bresp(m3_bresp), .m3_bvalid(m3_bvalid), .m3_bready(m3_bready), .m3_araddr(m3_araddr), .m3_arvalid(m3_arvalid), .m3_arready(m3_arready), .m3_rdata(m3_rdata), .m3_rresp(m3_rresp), .m3_rvalid(m3_rvalid), .m3_rready(m3_rready),

        .s0_araddr(s0_araddr), .s0_arvalid(s0_arvalid), .s0_arready(s0_arready), .s0_rdata(s0_rdata), .s0_rresp(s0_rresp), .s0_rvalid(s0_rvalid), .s0_rready(s0_rready),
        .s1_awaddr(s1_awaddr), .s1_awvalid(s1_awvalid), .s1_awready(s1_awready), .s1_wdata(s1_wdata), .s1_wstrb(s1_wstrb), .s1_wvalid(s1_wvalid), .s1_wready(s1_wready), .s1_bresp(s1_bresp), .s1_bvalid(s1_bvalid), .s1_bready(s1_bready), .s1_araddr(s1_araddr), .s1_arlen(s1_arlen), .s1_arsize(s1_arsize), .s1_arburst(s1_arburst), .s1_arvalid(s1_arvalid), .s1_arready(s1_arready), .s1_rdata(s1_rdata), .s1_rresp(s1_rresp), .s1_rlast(s1_rlast), .s1_rvalid(s1_rvalid), .s1_rready(s1_rready),
        .s2_awaddr(s2_awaddr), .s2_awvalid(s2_awvalid), .s2_awready(s2_awready), .s2_wdata(s2_wdata), .s2_wstrb(s2_wstrb), .s2_wvalid(s2_wvalid), .s2_wready(s2_wready), .s2_bresp(s2_bresp), .s2_bvalid(s2_bvalid), .s2_bready(s2_bready), .s2_araddr(s2_araddr), .s2_arvalid(s2_arvalid), .s2_arready(s2_arready), .s2_rdata(s2_rdata), .s2_rresp(s2_rresp), .s2_rvalid(s2_rvalid), .s2_rready(s2_rready),
        .s3_araddr(s3_araddr), .s3_arvalid(s3_arvalid), .s3_arready(s3_arready), .s3_rdata(s3_rdata), .s3_rresp(s3_rresp), .s3_rvalid(s3_rvalid), .s3_rready(s3_rready),
        .s4_awaddr(s4_awaddr), .s4_awvalid(s4_awvalid), .s4_awready(s4_awready), .s4_wdata(s4_wdata), .s4_wstrb(s4_wstrb), .s4_wvalid(s4_wvalid), .s4_wready(s4_wready), .s4_bresp(s4_bresp), .s4_bvalid(s4_bvalid), .s4_bready(s4_bready), .s4_araddr(s4_araddr), .s4_arvalid(s4_arvalid), .s4_arready(s4_arready), .s4_rdata(s4_rdata), .s4_rresp(s4_rresp), .s4_rvalid(s4_rvalid), .s4_rready(s4_rready),
        .s5_awaddr(s5_awaddr), .s5_awvalid(s5_awvalid), .s5_awready(s5_awready), .s5_wdata(s5_wdata), .s5_wstrb(s5_wstrb), .s5_wvalid(s5_wvalid), .s5_wready(s5_wready), .s5_bresp(s5_bresp), .s5_bvalid(s5_bvalid), .s5_bready(s5_bready), .s5_araddr(s5_araddr), .s5_arvalid(s5_arvalid), .s5_arready(s5_arready), .s5_rdata(s5_rdata), .s5_rresp(s5_rresp), .s5_rvalid(s5_rvalid), .s5_rready(s5_rready),
        .s6_awaddr(s6_awaddr), .s6_awvalid(s6_awvalid), .s6_awready(s6_awready), .s6_wdata(s6_wdata), .s6_wstrb(s6_wstrb), .s6_wvalid(s6_wvalid), .s6_wready(s6_wready), .s6_bresp(s6_bresp), .s6_bvalid(s6_bvalid), .s6_bready(s6_bready), .s6_araddr(s6_araddr), .s6_arlen(s6_arlen), .s6_arsize(s6_arsize), .s6_arburst(s6_arburst), .s6_arvalid(s6_arvalid), .s6_arready(s6_arready), .s6_rdata(s6_rdata), .s6_rresp(s6_rresp), .s6_rlast(s6_rlast), .s6_rvalid(s6_rvalid), .s6_rready(s6_rready),
        .s7_awaddr(s7_awaddr), .s7_awvalid(s7_awvalid), .s7_awready(s7_awready), .s7_wdata(s7_wdata), .s7_wstrb(s7_wstrb), .s7_wvalid(s7_wvalid), .s7_wready(s7_wready), .s7_bresp(s7_bresp), .s7_bvalid(s7_bvalid), .s7_bready(s7_bready), .s7_araddr(s7_araddr), .s7_arvalid(s7_arvalid), .s7_arready(s7_arready), .s7_rdata(s7_rdata), .s7_rresp(s7_rresp), .s7_rvalid(s7_rvalid), .s7_rready(s7_rready)
    );

    axi_rom BOOT_ROM_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(32'b0), .s_axi_awvalid(1'b0), .s_axi_awready(), .s_axi_wdata(32'b0), .s_axi_wstrb(4'b0), .s_axi_wvalid(1'b0), .s_axi_wready(), .s_axi_bresp(), .s_axi_bvalid(), .s_axi_bready(1'b0),
        .s_axi_araddr(s0_araddr), .s_axi_arvalid(s0_arvalid), .s_axi_arready(s0_arready), .s_axi_rdata(s0_rdata), .s_axi_rresp(s0_rresp), .s_axi_rvalid(s0_rvalid), .s_axi_rready(s0_rready)
    );

    axi_ram SYSTEM_RAM_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(s1_awaddr), .s_axi_awvalid(s1_awvalid), .s_axi_awready(s1_awready), .s_axi_wdata(s1_wdata), .s_axi_wstrb(s1_wstrb), .s_axi_wvalid(s1_wvalid), .s_axi_wready(s1_wready), .s_axi_bresp(s1_bresp), .s_axi_bvalid(s1_bvalid), .s_axi_bready(s1_bready),
        .s_axi_araddr(s1_araddr), .s_axi_arlen(s1_arlen), .s_axi_arsize(s1_arsize), .s_axi_arburst(s1_arburst), .s_axi_arvalid(s1_arvalid), .s_axi_arready(s1_arready), .s_axi_rdata(s1_rdata), .s_axi_rresp(s1_rresp), .s_axi_rlast(s1_rlast), .s_axi_rvalid(s1_rvalid), .s_axi_rready(s1_rready)
    );

    axi_spi_flash SPI_FLASH_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(32'b0), .s_axi_awvalid(1'b0), .s_axi_awready(), .s_axi_wdata(32'b0), .s_axi_wstrb(4'b0), .s_axi_wvalid(1'b0), .s_axi_wready(), .s_axi_bresp(), .s_axi_bvalid(), .s_axi_bready(1'b0),
        .s_axi_araddr(s3_araddr), .s_axi_arvalid(s3_arvalid), .s_axi_arready(s3_arready), .s_axi_rdata(s3_rdata), .s_axi_rresp(s3_rresp), .s_axi_rvalid(s3_rvalid), .s_axi_rready(s3_rready),
        .spi_cs_n(spi_flash_cs_n), .spi_sck(spi_flash_sck), .spi_mosi(spi_flash_mosi), .spi_miso(spi_flash_miso)
    );

    axi_clint CLINT_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(s4_awaddr), .s_axi_awvalid(s4_awvalid), .s_axi_awready(s4_awready), .s_axi_wdata(s4_wdata), .s_axi_wstrb(s4_wstrb), .s_axi_wvalid(s4_wvalid), .s_axi_wready(s4_wready), .s_axi_bresp(s4_bresp), .s_axi_bvalid(s4_bvalid), .s_axi_bready(s4_bready),
        .s_axi_araddr(s4_araddr), .s_axi_arvalid(s4_arvalid), .s_axi_arready(s4_arready), .s_axi_rdata(s4_rdata), .s_axi_rresp(s4_rresp), .s_axi_rvalid(s4_rvalid), .s_axi_rready(s4_rready),
        .timer_irq(clint_timer_irq), .software_irq(clint_software_irq)
    );

    wire dma_irq_out, irq_timer, irq_uart, irq_spi, irq_i2c, irq_gpio, irq_accel, wdt_irq_out;
    wire [31:1] plic_irq_sources = {23'd0, dma_irq_out, wdt_irq_out, irq_accel, irq_gpio, irq_i2c, irq_spi, irq_uart, irq_timer};
    
    axi_plic #(.NUM_IRQ(31)) PLIC_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(s5_awaddr), .s_axi_awvalid(s5_awvalid), .s_axi_awready(s5_awready), .s_axi_wdata(s5_wdata), .s_axi_wstrb(s5_wstrb), .s_axi_wvalid(s5_wvalid), .s_axi_wready(s5_wready), .s_axi_bresp(s5_bresp), .s_axi_bvalid(s5_bvalid), .s_axi_bready(s5_bready),
        .s_axi_araddr(s5_araddr), .s_axi_arvalid(s5_arvalid), .s_axi_arready(s5_arready), .s_axi_rdata(s5_rdata), .s_axi_rresp(s5_rresp), .s_axi_rvalid(s5_rvalid), .s_axi_rready(s5_rready),
        .irq_sources(plic_irq_sources), .ext_irq(plic_ext_irq)
    );

    axi_sdram_controller SDRAM_CTRL_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(s6_awaddr), .s_axi_awvalid(s6_awvalid), .s_axi_awready(s6_awready), .s_axi_wdata(s6_wdata), .s_axi_wstrb(s6_wstrb), .s_axi_wvalid(s6_wvalid), .s_axi_wready(s6_wready), .s_axi_bresp(s6_bresp), .s_axi_bvalid(s6_bvalid), .s_axi_bready(s6_bready),
        .s_axi_araddr(s6_araddr), .s_axi_arvalid(s6_arvalid), .s_axi_arready(s6_arready), .s_axi_rdata(s6_rdata), .s_axi_rresp(s6_rresp), .s_axi_rvalid(s6_rvalid), .s_axi_rready(s6_rready),
        .sdram_clk(sdram_clk), .sdram_cke(sdram_cke), .sdram_cs_n(sdram_cs_n), .sdram_ras_n(sdram_ras_n), .sdram_cas_n(sdram_cas_n), .sdram_we_n(sdram_we_n), .sdram_ba(sdram_ba), .sdram_addr(sdram_addr), .sdram_dqm(sdram_dqm), .sdram_dq_in(sdram_dq_in), .sdram_dq_out(sdram_dq_out), .sdram_dq_oe(sdram_dq_oe)
    );

    axi_dma DMA_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(s7_awaddr), .s_axi_awvalid(s7_awvalid), .s_axi_awready(s7_awready), .s_axi_wdata(s7_wdata), .s_axi_wstrb(s7_wstrb), .s_axi_wvalid(s7_wvalid), .s_axi_wready(s7_wready), .s_axi_bresp(s7_bresp), .s_axi_bvalid(s7_bvalid), .s_axi_bready(s7_bready),
        .s_axi_araddr(s7_araddr), .s_axi_arvalid(s7_arvalid), .s_axi_arready(s7_arready), .s_axi_rdata(s7_rdata), .s_axi_rresp(s7_rresp), .s_axi_rvalid(s7_rvalid), .s_axi_rready(s7_rready),
        .m_axi_awaddr(m3_awaddr), .m_axi_awvalid(m3_awvalid), .m_axi_awready(m3_awready), .m_axi_wdata(m3_wdata), .m_axi_wstrb(m3_wstrb), .m_axi_wvalid(m3_wvalid), .m_axi_wready(m3_wready), .m_axi_bresp(m3_bresp), .m_axi_bvalid(m3_bvalid), .m_axi_bready(m3_bready),
        .m_axi_araddr(m3_araddr), .m_axi_arvalid(m3_arvalid), .m_axi_arready(m3_arready), .m_axi_rdata(m3_rdata), .m_axi_rresp(m3_rresp), .m_axi_rvalid(m3_rvalid), .m_axi_rready(m3_rready),
        .dma_irq(dma_irq_out)
    );

    // =========================================================================
    // 10. APB SUBSYSTEM (Miền Bus)
    // =========================================================================
    wire [31:0] apb_paddr, apb_pwdata, apb_prdata;
    wire [3:0]  apb_pstrb;
    wire        apb_psel, apb_penable, apb_pwrite, apb_pready, apb_pslverr;

    axi_to_apb_bridge BRIDGE_INST (
        .clk(clk_bus), .rst_n(bus_rst_n),
        .s_axi_awaddr(s2_awaddr), .s_axi_awvalid(s2_awvalid), .s_axi_awready(s2_awready), .s_axi_wdata(s2_wdata), .s_axi_wstrb(s2_wstrb), .s_axi_wvalid(s2_wvalid), .s_axi_wready(s2_wready), .s_axi_bresp(s2_bresp), .s_axi_bvalid(s2_bvalid), .s_axi_bready(s2_bready),
        .s_axi_araddr(s2_araddr), .s_axi_arvalid(s2_arvalid), .s_axi_arready(s2_arready), .s_axi_rdata(s2_rdata), .s_axi_rresp(s2_rresp), .s_axi_rvalid(s2_rvalid), .s_axi_rready(s2_rready),
        .m_apb_paddr(apb_paddr), .m_apb_psel(apb_psel), .m_apb_penable(apb_penable), .m_apb_pwrite(apb_pwrite), .m_apb_pwdata(apb_pwdata), .m_apb_pstrb(apb_pstrb), .m_apb_pready(apb_pready), .m_apb_prdata(apb_prdata), .m_apb_pslverr(apb_pslverr)
    );

    wire sel_syscon, sel_wdt, sel_timer, sel_uart, sel_spi, sel_i2c, sel_gpio, sel_accel;
    wire [31:0] rdata_syscon, rdata_wdt, rdata_timer, rdata_uart, rdata_spi, rdata_i2c, rdata_gpio, rdata_accel;
    wire ready_syscon, ready_wdt, ready_timer, ready_uart, ready_spi, ready_i2c, ready_gpio, ready_accel;
    wire err_syscon, err_wdt, err_timer, err_uart, err_spi, err_i2c, err_gpio, err_accel;

    apb_interconnect APB_BUS_MATRIX (
        .m_paddr(apb_paddr), .m_psel(apb_psel), .m_penable(apb_penable), .m_pwrite(apb_pwrite), .m_pwdata(apb_pwdata), .m_pstrb(apb_pstrb), .m_prdata(apb_prdata), .m_pready(apb_pready), .m_pslverr(apb_pslverr),
        .s0_psel(sel_syscon), .s0_paddr(), .s0_penable(), .s0_pwrite(), .s0_pwdata(), .s0_pstrb(), .s0_prdata(rdata_syscon), .s0_pready(ready_syscon), .s0_pslverr(err_syscon),
        .s1_psel(sel_wdt),    .s1_paddr(), .s1_penable(), .s1_pwrite(), .s1_pwdata(), .s1_pstrb(), .s1_prdata(rdata_wdt),    .s1_pready(ready_wdt),    .s1_pslverr(err_wdt),
        .s2_psel(sel_timer),  .s2_paddr(), .s2_penable(), .s2_pwrite(), .s2_pwdata(), .s2_pstrb(), .s2_prdata(rdata_timer),  .s2_pready(ready_timer),  .s2_pslverr(err_timer),
        .s3_psel(sel_uart),   .s3_paddr(), .s3_penable(), .s3_pwrite(), .s3_pwdata(), .s3_pstrb(), .s3_prdata(rdata_uart),   .s3_pready(ready_uart),   .s3_pslverr(err_uart),
        .s4_psel(sel_spi),    .s4_paddr(), .s4_penable(), .s4_pwrite(), .s4_pwdata(), .s4_pstrb(), .s4_prdata(rdata_spi),    .s4_pready(ready_spi),    .s4_pslverr(err_spi),
        .s5_psel(sel_i2c),    .s5_paddr(), .s5_penable(), .s5_pwrite(), .s5_pwdata(), .s5_pstrb(), .s5_prdata(rdata_i2c),    .s5_pready(ready_i2c),    .s5_pslverr(err_i2c),
        .s6_psel(sel_gpio),   .s6_paddr(), .s6_penable(), .s6_pwrite(), .s6_pwdata(), .s6_pstrb(), .s6_prdata(rdata_gpio),   .s6_pready(ready_gpio),   .s6_pslverr(err_gpio),
        .s7_psel(sel_accel),  .s7_paddr(), .s7_penable(), .s7_pwrite(), .s7_pwdata(), .s7_pstrb(), .s7_prdata(rdata_accel),  .s7_pready(ready_accel),  .s7_pslverr(err_accel)
    );

    // =========================================================================
    // 11. NGOẠI VI APB
    // =========================================================================
    apb_syscon SYSCON_INST (.pclk(clk_bus), .presetn(bus_rst_n), .paddr(apb_paddr[15:0]), .psel(sel_syscon), .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pready(ready_syscon), .prdata(rdata_syscon), .pslverr(err_syscon), .o_reset_vector(sys_reset_vector), .i_wfi_sleep(cpu_wfi_sleep), .i_ext_irq(plic_ext_irq), .o_cpu_clk_en(clk_en_cpu), .o_dbg_clk_en(clk_en_dbg), .o_tmr_clk_en(clk_en_tmr), .o_urt_clk_en(clk_en_urt), .o_spi_clk_en(clk_en_spi), .o_i2c_clk_en(clk_en_i2c), .o_gpo_clk_en(clk_en_gpo), .o_acc_clk_en(clk_en_acc));
    apb_watchdog WDT_INST  (.pclk(clk_bus), .presetn(bus_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_wdt),    .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_wdt),    .prdata(rdata_wdt),    .pslverr(err_wdt),    .wdt_irq(wdt_irq_out), .wdt_rst(wdt_rst_req));
    apb_timer  TIMER_INST  (.pclk(clk_tmr_gated), .presetn(bus_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_timer),  .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_timer),  .prdata(rdata_timer),  .pslverr(err_timer),  .timer_irq(irq_timer));
    apb_uart   UART_INST   (.pclk(clk_urt_gated), .presetn(bus_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_uart),   .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_uart),   .prdata(rdata_uart),   .pslverr(err_uart),   .rx(uart_rx), .tx(uart_tx), .uart_irq(irq_uart));
    apb_spi    SPI_INST    (.pclk(clk_spi_gated), .presetn(bus_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_spi),    .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_spi),    .prdata(rdata_spi),    .pslverr(err_spi),    .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso), .cs_n(spi_cs_n), .spi_irq(irq_spi));
    apb_i2c    I2C_INST    (.pclk(clk_i2c_gated), .presetn(bus_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_i2c),    .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_i2c),    .prdata(rdata_i2c),    .pslverr(err_i2c),    .scl_o(i2c_scl_o), .scl_oen(i2c_scl_oen), .scl_i(i2c_scl_i), .sda_o(i2c_sda_o), .sda_oen(i2c_sda_oen), .sda_i(i2c_sda_i), .i2c_irq(irq_i2c));
    apb_gpio   GPIO_INST   (.pclk(clk_gpo_gated), .presetn(bus_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_gpio),   .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pready(ready_gpio), .prdata(rdata_gpio), .pslverr(err_gpio), .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_dir(gpio_dir), .gpio_irq(irq_gpio));
    apb_accelerator ACCEL_INST (.pclk(clk_acc_gated), .presetn(bus_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_accel), .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_accel), .prdata(rdata_accel), .pslverr(err_accel), .accel_irq(irq_accel));

endmodule