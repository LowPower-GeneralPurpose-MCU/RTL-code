`timescale 1ns / 1ps

module top_soc (
    // --- Các Nguồn Xung Nhịp ---
    input  wire        clk_core,   // 400 MHz
    input  wire        clk_axi,    // 200 MHz
    input  wire        clk_apb,    // 100 MHz
    input  wire        clk_sdram_ctrl, // 100 MHz (0 độ lệch pha)
    input  wire        clk_sdram_ext,  // 100 MHz (LỆCH PHA - cấp riêng cho chip RAM ngoài)
    input  wire        uart_clk,   
    input  wire        spi_clk,    
    input  wire        i2c_clk,    
    input  wire        rtc_clk,    // 32.768 kHz

    input  wire        rst_n,

    // JTAG
    input  wire        tck,
    input  wire        trst_n,
    input  wire        tms,
    input  wire        tdi,
    output wire        tdo,

    // Ngoại vi
    input  wire        uart_rx,
    output wire        uart_tx,

    input  wire [31:0] gpio_in,
    output wire [31:0] gpio_out,
    output wire [31:0] gpio_oe,
    output wire        pwm_out,

    output wire        spi_sck,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_ss,

    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    output wire        flash_sck,
    output wire        flash_cs_n,
    inout  wire [3:0]  flash_io,

    output wire        sdram_clk,
    output wire        sdram_cke,
    output wire        sdram_cs_n,
    output wire        sdram_ras_n,
    output wire        sdram_cas_n,
    output wire        sdram_we_n,
    output wire [1:0]  sdram_ba,
    output wire [12:0] sdram_addr,
    inout  wire [15:0] sdram_dq,
    output wire [1:0]  sdram_dqm
);
    assign sdram_clk = clk_sdram_ext;

    // =========================================================================
    // 1. RESET SYNCHRONIZERS
    // =========================================================================
    wire wdt_rst;
    wire ndmreset_req;
    wire reset_sys_n_raw = rst_n & ~ndmreset_req & ~wdt_rst;

    wire reset_core_n_sync;
    wire reset_axi_n_sync;
    wire reset_apb_n_sync;
    wire reset_sdram_n_sync;
    
    reset_sync u_core_rst_sync (.clk(clk_core), .rst_in_n(reset_sys_n_raw), .rst_out_n(reset_core_n_sync));
    reset_sync u_axi_rst_sync  (.clk(clk_axi),  .rst_in_n(reset_sys_n_raw), .rst_out_n(reset_axi_n_sync));
    reset_sync u_apb_rst_sync  (.clk(clk_apb),  .rst_in_n(reset_sys_n_raw), .rst_out_n(reset_apb_n_sync));
    reset_sync u_sdram_rst_sync(.clk(clk_sdram_ctrl), .rst_in_n(reset_sys_n_raw), .rst_out_n(reset_sdram_n_sync));

    // =========================================================================
    // 2. CLOCK GATING NETWORK (ĐÃ PHỤC HỒI 100%)
    // =========================================================================
    wire clk_en_cpu, clk_en_dbg, clk_en_pwm, clk_en_uart;
    wire clk_en_spi, clk_en_i2c, clk_en_gpio, clk_en_acc;

    wire clk_cpu, clk_dbg, clk_pwm, clk_gpio, clk_cordic;
    wire clk_uart_gated, clk_spi_gated, clk_i2c_gated;

    // Gating cho Core (từ clk_core)
    clock_gate cg_cpu   (.clk_in(clk_core), .en(clk_en_cpu),  .test_en(1'b0), .clk_out(clk_cpu));
    // Gating cho Debug (từ clk_axi)
    clock_gate cg_dbg   (.clk_in(clk_axi),  .en(clk_en_dbg),  .test_en(1'b0), .clk_out(clk_dbg));
    // Gating cho APB Peripherals (từ clk_apb)
    clock_gate cg_pwm   (.clk_in(clk_apb),  .en(clk_en_pwm),  .test_en(1'b0), .clk_out(clk_pwm));
    clock_gate cg_gpio  (.clk_in(clk_apb),  .en(clk_en_gpio), .test_en(1'b0), .clk_out(clk_gpio));
    clock_gate cg_cordic(.clk_in(clk_apb),  .en(clk_en_acc),  .test_en(1'b0), .clk_out(clk_cordic));
    // Gating cho Lõi ngoại vi độc lập (Dual-Clock Cores)
    clock_gate cg_uart  (.clk_in(uart_clk), .en(clk_en_uart), .test_en(1'b0), .clk_out(clk_uart_gated));
    clock_gate cg_spi   (.clk_in(spi_clk),  .en(clk_en_spi),  .test_en(1'b0), .clk_out(clk_spi_gated));
    clock_gate cg_i2c   (.clk_in(i2c_clk),  .en(clk_en_i2c),  .test_en(1'b0), .clk_out(clk_i2c_gated));

    // =========================================================================
    // 3. TÍN HIỆU NGẮT VÀ CDC
    // =========================================================================
    wire [0:0] cpu_msip_raw;
    wire [0:0] cpu_mtip_raw;
    wire       cpu_meip_raw;

    // Đồng bộ Ngắt vào CPU (vào clk_cpu)
    wire cpu_meip_sync, cpu_mtip_sync, cpu_msip_sync;
    cdc_sync_bit u_sync_meip (.clk_dst(clk_cpu), .rst_dst_n(reset_core_n_sync), .d_in(cpu_meip_raw),   .q_out(cpu_meip_sync));
    cdc_sync_bit u_sync_mtip (.clk_dst(clk_cpu), .rst_dst_n(reset_core_n_sync), .d_in(cpu_mtip_raw[0]),.q_out(cpu_mtip_sync));
    cdc_sync_bit u_sync_msip (.clk_dst(clk_cpu), .rst_dst_n(reset_core_n_sync), .d_in(cpu_msip_raw[0]),.q_out(cpu_msip_sync));

    wire uart_irq_raw, gpio_irq_raw, spi_irq_raw, i2c_irq_raw, wdt_irq_raw;
    wire uart_dma_tx_raw, uart_dma_rx_raw, spi_dma_tx_raw, spi_dma_rx_raw, i2c_dma_tx_raw, i2c_dma_rx_raw;

    // Đồng bộ Ngắt ngoại vi về clk_apb (cho PLIC)
    wire uart_irq, gpio_irq, spi_irq, i2c_irq, wdt_irq;
    cdc_sync_bit u_sync_uart_irq (.clk_dst(clk_apb), .rst_dst_n(reset_apb_n_sync), .d_in(uart_irq_raw), .q_out(uart_irq));
    cdc_sync_bit u_sync_gpio_irq (.clk_dst(clk_apb), .rst_dst_n(reset_apb_n_sync), .d_in(gpio_irq_raw), .q_out(gpio_irq));
    cdc_sync_bit u_sync_spi_irq  (.clk_dst(clk_apb), .rst_dst_n(reset_apb_n_sync), .d_in(spi_irq_raw),  .q_out(spi_irq));
    cdc_sync_bit u_sync_i2c_irq  (.clk_dst(clk_apb), .rst_dst_n(reset_apb_n_sync), .d_in(i2c_irq_raw),  .q_out(i2c_irq));
    cdc_sync_bit u_sync_wdt_irq  (.clk_dst(clk_apb), .rst_dst_n(reset_apb_n_sync), .d_in(wdt_irq_raw),  .q_out(wdt_irq));

    // Đồng bộ DMA Req về clk_axi (cho DMA Controller)
    wire uart_dma_tx, uart_dma_rx, spi_dma_tx, spi_dma_rx, i2c_dma_tx, i2c_dma_rx;
    cdc_sync_bit u_sync_utx (.clk_dst(clk_axi), .rst_dst_n(reset_axi_n_sync), .d_in(uart_dma_tx_raw), .q_out(uart_dma_tx));
    cdc_sync_bit u_sync_urx (.clk_dst(clk_axi), .rst_dst_n(reset_axi_n_sync), .d_in(uart_dma_rx_raw), .q_out(uart_dma_rx));
    cdc_sync_bit u_sync_stx (.clk_dst(clk_axi), .rst_dst_n(reset_axi_n_sync), .d_in(spi_dma_tx_raw),  .q_out(spi_dma_tx));
    cdc_sync_bit u_sync_srx (.clk_dst(clk_axi), .rst_dst_n(reset_axi_n_sync), .d_in(spi_dma_rx_raw),  .q_out(spi_dma_rx));
    cdc_sync_bit u_sync_itx (.clk_dst(clk_axi), .rst_dst_n(reset_axi_n_sync), .d_in(i2c_dma_tx_raw),  .q_out(i2c_dma_tx));
    cdc_sync_bit u_sync_irx (.clk_dst(clk_axi), .rst_dst_n(reset_axi_n_sync), .d_in(i2c_dma_rx_raw),  .q_out(i2c_dma_rx));

    wire [3:0] dma_irq; 
    wire dma_irq_sync;
    cdc_sync_bit u_sync_dma_irq (.clk_dst(clk_apb), .rst_dst_n(reset_apb_n_sync), .d_in(|dma_irq), .q_out(dma_irq_sync));

    wire [31:1] periph_dma_req = { 25'd0, i2c_dma_rx, i2c_dma_tx, spi_dma_rx, spi_dma_tx, uart_dma_rx, uart_dma_tx };
    wire [31:1] periph_dma_clr;
    wire [31:0] plic_irq_src = { 25'd0, dma_irq_sync, wdt_irq, i2c_irq, spi_irq, gpio_irq, uart_irq, 1'b0 };

    wire [31:0] syscon_reset_vector;
    wire        wfi_sleep_state;

    // =========================================================================
    // 4. LÕI CPU VÀ CACHES (Chạy bằng clk_cpu đã qua Gating)
    // =========================================================================
    wire [31:0] cpu_inst_addr, cpu_inst_data, cpu_data_addr, cpu_data_wdata, cpu_data_rdata;
    wire cpu_inst_req, cpu_inst_hit, cpu_inst_stall, cpu_data_rd_req, cpu_data_wr_req, cpu_data_hit, cpu_data_stall, cpu_data_unsigned;
    wire [1:0] cpu_data_size;
    wire dbg_halt_req, dbg_resume_req, dbg_halted, dbg_reg_write_en;
    wire [15:0] dbg_reg_read_addr, dbg_reg_write_addr;
    wire [31:0] dbg_reg_read_data, dbg_reg_write_data;
    // --- Kênh từ Debug Module (200MHz) sang CPU (400MHz) ---
    wire dbg_halt_req_raw, dbg_resume_req_raw;
    wire dbg_halt_req_sync, dbg_resume_req_sync;

    cdc_sync_bit u_sync_halt (
        .clk_dst    (clk_cpu),           // Miền đích CPU
        .rst_dst_n  (reset_core_n_sync),
        .d_in       (dbg_halt_req_raw),  // Xuất phát từ DM
        .q_out      (dbg_halt_req_sync)  // Đã sync để đưa vào CPU
    );

    cdc_sync_bit u_sync_resume (
        .clk_dst    (clk_cpu),
        .rst_dst_n  (reset_core_n_sync),
        .d_in       (dbg_resume_req_raw),
        .q_out      (dbg_resume_req_sync)
    );

    // --- Kênh báo trạng thái từ CPU (400MHz) ngược về DM (200MHz) ---
    wire dbg_halted_raw;
    wire dbg_halted_sync;

    cdc_sync_bit u_sync_halted (
        .clk_dst    (clk_dbg),           // Miền đích DM
        .rst_dst_n  (reset_axi_n_sync),
        .d_in       (dbg_halted_raw),    // Xuất phát từ CPU
        .q_out      (dbg_halted_sync)    // Đã sync để đưa về DM
    );

    riscv_pipeline u_core (
        .clk                (clk_cpu),
        .reset_n            (reset_core_n_sync),
        .riscv_start        (1'b1),
        .meip_i             (cpu_meip_sync),
        .msip_i             (cpu_msip_sync),
        .mtip_i             (cpu_mtip_sync),
        .reset_vector_in    (syscon_reset_vector),
        .riscv_done         (),
        .icache_read_req    (cpu_inst_req),
        .icache_addr        (cpu_inst_addr),
        .icache_read_data   (cpu_inst_data),
        .icache_hit         (cpu_inst_hit),
        .icache_stall       (cpu_inst_stall),
        .dcache_read_req    (cpu_data_rd_req),
        .dcache_write_req   (cpu_data_wr_req),
        .dcache_addr        (cpu_data_addr),
        .dcache_write_data  (cpu_data_wdata),
        .dcache_read_data   (cpu_data_rdata),
        .dcache_hit         (cpu_data_hit),
        .dcache_stall       (cpu_data_stall),
        .mem_size_top       (cpu_data_size),
        .mem_unsigned_top   (cpu_data_unsigned),
        .wfi_sleep_out      (wfi_sleep_state),
        .dbg_halt_req       (dbg_halt_req_sync), // Dùng tín hiệu đã qua CDC
        .dbg_resume_req     (dbg_resume_req_sync),
        .dbg_halted         (dbg_halted_raw),    // Đưa tín hiệu thô ra để vào CDC
        .dbg_reg_read_addr  (dbg_reg_read_addr),
        .dbg_reg_read_data  (dbg_reg_read_data),
        .dbg_reg_write_en   (dbg_reg_write_en),
        .dbg_reg_write_addr (dbg_reg_write_addr),
        .dbg_reg_write_data (dbg_reg_write_data)
    );

    // =========================================================================
    // 5. KHAI BÁO CÁC KÊNH AXI CHI TIẾT (Không viết tắt)
    // =========================================================================
    localparam MST_AMT = 4;
    localparam SLV_AMT = 6;

    wire [MST_AMT*5-1:0]  m_axi_awid;   wire [MST_AMT*32-1:0] m_axi_awaddr; wire [MST_AMT*8-1:0]  m_axi_awlen;
    wire [MST_AMT*3-1:0]  m_axi_awsize; wire [MST_AMT*2-1:0]  m_axi_awburst; wire [MST_AMT*3-1:0]  m_axi_awprot;
    wire [MST_AMT-1:0]    m_axi_awvalid; wire [MST_AMT-1:0]    m_axi_awready; wire [MST_AMT*32-1:0] m_axi_wdata;
    wire [MST_AMT*4-1:0]  m_axi_wstrb;  wire [MST_AMT-1:0]    m_axi_wlast;  wire [MST_AMT-1:0]    m_axi_wvalid;
    wire [MST_AMT-1:0]    m_axi_wready; wire [MST_AMT*5-1:0]  m_axi_bid;    wire [MST_AMT*2-1:0]  m_axi_bresp;
    wire [MST_AMT-1:0]    m_axi_bvalid; wire [MST_AMT-1:0]    m_axi_bready; wire [MST_AMT*5-1:0]  m_axi_arid;
    wire [MST_AMT*32-1:0] m_axi_araddr; wire [MST_AMT*8-1:0]  m_axi_arlen;  wire [MST_AMT*3-1:0]  m_axi_arsize;
    wire [MST_AMT*2-1:0]  m_axi_arburst; wire [MST_AMT*3-1:0]  m_axi_arprot; wire [MST_AMT-1:0]    m_axi_arvalid;
    wire [MST_AMT-1:0]    m_axi_arready; wire [MST_AMT*5-1:0]  m_axi_rid;    wire [MST_AMT*32-1:0] m_axi_rdata;
    wire [MST_AMT*2-1:0]  m_axi_rresp;  wire [MST_AMT-1:0]    m_axi_rlast;  wire [MST_AMT-1:0]    m_axi_rvalid;
    wire [MST_AMT-1:0]    m_axi_rready;

    // Master 0: ICache
    wire [4:0]  m0_awid;    wire [31:0] m0_awaddr;  wire [7:0]  m0_awlen;   wire [2:0]  m0_awsize;  wire [1:0]  m0_awburst; wire [2:0]  m0_awprot;  wire m0_awvalid; wire m0_awready;
    wire [31:0] m0_wdata;   wire [3:0]  m0_wstrb;   wire        m0_wlast;   wire        m0_wvalid;  wire m0_wready;
    wire [4:0]  m0_bid;     wire [1:0]  m0_bresp;   wire        m0_bvalid;  wire        m0_bready;
    wire [4:0]  m0_arid;    wire [31:0] m0_araddr;  wire [7:0]  m0_arlen;   wire [2:0]  m0_arsize;  wire [1:0]  m0_arburst; wire [2:0]  m0_arprot;  wire m0_arvalid; wire m0_arready;
    wire [4:0]  m0_rid;     wire [31:0] m0_rdata;   wire [1:0]  m0_rresp;   wire        m0_rlast;   wire m0_rvalid; wire m0_rready;

    // Master 1: DCache
    wire [4:0]  m1_awid;    wire [31:0] m1_awaddr;  wire [7:0]  m1_awlen;   wire [2:0]  m1_awsize;  wire [1:0]  m1_awburst; wire [2:0]  m1_awprot;  wire m1_awvalid; wire m1_awready;
    wire [31:0] m1_wdata;   wire [3:0]  m1_wstrb;   wire        m1_wlast;   wire        m1_wvalid;  wire m1_wready;
    wire [4:0]  m1_bid;     wire [1:0]  m1_bresp;   wire        m1_bvalid;  wire        m1_bready;
    wire [4:0]  m1_arid;    wire [31:0] m1_araddr;  wire [7:0]  m1_arlen;   wire [2:0]  m1_arsize;  wire [1:0]  m1_arburst; wire [2:0]  m1_arprot;  wire m1_arvalid; wire m1_arready;
    wire [4:0]  m1_rid;     wire [31:0] m1_rdata;   wire [1:0]  m1_rresp;   wire        m1_rlast;   wire m1_rvalid; wire m1_rready;

    // Master 2: DTM (Debug)
    wire [4:0]  m2_awid;    wire [31:0] m2_awaddr;  wire [7:0]  m2_awlen;   wire [2:0]  m2_awsize;  wire [1:0]  m2_awburst; wire [2:0]  m2_awprot;  wire m2_awvalid; wire m2_awready;
    wire [31:0] m2_wdata;   wire [3:0]  m2_wstrb;   wire        m2_wlast;   wire        m2_wvalid;  wire m2_wready;
    wire [4:0]  m2_bid;     wire [1:0]  m2_bresp;   wire        m2_bvalid;  wire        m2_bready;
    wire [4:0]  m2_arid;    wire [31:0] m2_araddr;  wire [7:0]  m2_arlen;   wire [2:0]  m2_arsize;  wire [1:0]  m2_arburst; wire [2:0]  m2_arprot;  wire m2_arvalid; wire m2_arready;
    wire [4:0]  m2_rid;     wire [31:0] m2_rdata;   wire [1:0]  m2_rresp;   wire        m2_rlast;   wire m2_rvalid; wire m2_rready;

    // Master 3: DMA
    wire [4:0]  m3_awid;    wire [31:0] m3_awaddr;  wire [7:0]  m3_awlen;   wire [2:0]  m3_awsize;  wire [1:0]  m3_awburst; wire [2:0]  m3_awprot;  wire m3_awvalid; wire m3_awready;
    wire [31:0] m3_wdata;   wire [3:0]  m3_wstrb;   wire        m3_wlast;   wire        m3_wvalid;  wire m3_wready;
    wire [4:0]  m3_bid;     wire [1:0]  m3_bresp;   wire        m3_bvalid;  wire        m3_bready;
    wire [4:0]  m3_arid;    wire [31:0] m3_araddr;  wire [7:0]  m3_arlen;   wire [2:0]  m3_arsize;  wire [1:0]  m3_arburst; wire [2:0]  m3_arprot;  wire m3_arvalid; wire m3_arready;
    wire [4:0]  m3_rid;     wire [31:0] m3_rdata;   wire [1:0]  m3_rresp;   wire        m3_rlast;   wire m3_rvalid; wire m3_rready;

    assign m_axi_awid    = {m3_awid, m2_awid, m1_awid, m0_awid};
    assign m_axi_awaddr  = {m3_awaddr, m2_awaddr, m1_awaddr, m0_awaddr};
    assign m_axi_awlen   = {m3_awlen, m2_awlen, m1_awlen, m0_awlen};
    assign m_axi_awsize  = {m3_awsize, m2_awsize, m1_awsize, m0_awsize};
    assign m_axi_awburst = {m3_awburst, m2_awburst, m1_awburst, m0_awburst};
    assign m_axi_awprot  = {m3_awprot, m2_awprot, m1_awprot, m0_awprot};
    assign m_axi_awvalid = {m3_awvalid, m2_awvalid, m1_awvalid, m0_awvalid};
    assign {m3_awready, m2_awready, m1_awready, m0_awready} = m_axi_awready;

    assign m_axi_wdata   = {m3_wdata, m2_wdata, m1_wdata, m0_wdata};
    assign m_axi_wstrb   = {m3_wstrb, m2_wstrb, m1_wstrb, m0_wstrb};
    assign m_axi_wlast   = {m3_wlast, m2_wlast, m1_wlast, m0_wlast};
    assign m_axi_wvalid  = {m3_wvalid, m2_wvalid, m1_wvalid, m0_wvalid};
    assign {m3_wready, m2_wready, m1_wready, m0_wready} = m_axi_wready;

    assign m_axi_bready  = {m3_bready, m2_bready, m1_bready, m0_bready};
    assign {m3_bid, m2_bid, m1_bid, m0_bid}             = m_axi_bid;
    assign {m3_bresp, m2_bresp, m1_bresp, m0_bresp}       = m_axi_bresp;
    assign {m3_bvalid, m2_bvalid, m1_bvalid, m0_bvalid}    = m_axi_bvalid;

    assign m_axi_arid    = {m3_arid, m2_arid, m1_arid, m0_arid};
    assign m_axi_araddr  = {m3_araddr, m2_araddr, m1_araddr, m0_araddr};
    assign m_axi_arlen   = {m3_arlen, m2_arlen, m1_arlen, m0_arlen};
    assign m_axi_arsize  = {m3_arsize, m2_arsize, m1_arsize, m0_arsize};
    assign m_axi_arburst = {m3_arburst, m2_arburst, m1_arburst, m0_arburst};
    assign m_axi_arprot  = {m3_arprot, m2_arprot, m1_arprot, m0_arprot};
    assign m_axi_arvalid = {m3_arvalid, m2_arvalid, m1_arvalid, m0_arvalid};
    assign {m3_arready, m2_arready, m1_arready, m0_arready} = m_axi_arready;

    assign m_axi_rready  = {m3_rready, m2_rready, m1_rready, m0_rready};
    assign {m3_rid, m2_rid, m1_rid, m0_rid}             = m_axi_rid;
    assign {m3_rdata, m2_rdata, m1_rdata, m0_rdata}       = m_axi_rdata;
    assign {m3_rresp, m2_rresp, m1_rresp, m0_rresp}       = m_axi_rresp;
    assign {m3_rlast, m2_rlast, m1_rlast, m0_rlast}       = m_axi_rlast;
    assign {m3_rvalid, m2_rvalid, m1_rvalid, m0_rvalid}    = m_axi_rvalid;

    // Slaves Arrays
    wire [SLV_AMT*7-1:0]  s_axi_awid;   wire [SLV_AMT*32-1:0] s_axi_awaddr; wire [SLV_AMT*8-1:0]  s_axi_awlen;
    wire [SLV_AMT*3-1:0]  s_axi_awsize; wire [SLV_AMT*2-1:0]  s_axi_awburst; wire [SLV_AMT*3-1:0]  s_axi_awprot;
    wire [SLV_AMT-1:0]    s_axi_awvalid; wire [SLV_AMT-1:0]    s_axi_awready; wire [SLV_AMT*32-1:0] s_axi_wdata;
    wire [SLV_AMT*4-1:0]  s_axi_wstrb;  wire [SLV_AMT-1:0]    s_axi_wlast;  wire [SLV_AMT-1:0]    s_axi_wvalid;
    wire [SLV_AMT-1:0]    s_axi_wready; wire [SLV_AMT*7-1:0]  s_axi_bid;    wire [SLV_AMT*2-1:0]  s_axi_bresp;
    wire [SLV_AMT-1:0]    s_axi_bvalid; wire [SLV_AMT-1:0]    s_axi_bready; wire [SLV_AMT*7-1:0]  s_axi_arid;
    wire [SLV_AMT*32-1:0] s_axi_araddr; wire [SLV_AMT*8-1:0]  s_axi_arlen;  wire [SLV_AMT*3-1:0]  s_axi_arsize;
    wire [SLV_AMT*2-1:0]  s_axi_arburst; wire [SLV_AMT*3-1:0]  s_axi_arprot; wire [SLV_AMT-1:0]    s_axi_arvalid;
    wire [SLV_AMT-1:0]    s_axi_arready; wire [SLV_AMT*7-1:0]  s_axi_rid;    wire [SLV_AMT*32-1:0] s_axi_rdata;
    wire [SLV_AMT*2-1:0]  s_axi_rresp;  wire [SLV_AMT-1:0]    s_axi_rlast;  wire [SLV_AMT-1:0]    s_axi_rvalid;
    wire [SLV_AMT-1:0]    s_axi_rready;

    // Từng Slave đơn lẻ
    wire [6:0] s0_awid; wire [31:0] s0_awaddr; wire [7:0] s0_awlen; wire [2:0] s0_awsize; wire [1:0] s0_awburst; wire [2:0] s0_awprot; wire s0_awvalid; wire s0_awready;
    wire [31:0] s0_wdata; wire [3:0] s0_wstrb; wire s0_wlast; wire s0_wvalid; wire s0_wready;
    wire [6:0] s0_bid; wire [1:0] s0_bresp; wire s0_bvalid; wire s0_bready;
    wire [6:0] s0_arid; wire [31:0] s0_araddr; wire [7:0] s0_arlen; wire [2:0] s0_arsize; wire [1:0] s0_arburst; wire [2:0] s0_arprot; wire s0_arvalid; wire s0_arready;
    wire [6:0] s0_rid; wire [31:0] s0_rdata; wire [1:0] s0_rresp; wire s0_rlast; wire s0_rvalid; wire s0_rready;

    wire [6:0] s1_awid; wire [31:0] s1_awaddr; wire [7:0] s1_awlen; wire [2:0] s1_awsize; wire [1:0] s1_awburst; wire [2:0] s1_awprot; wire s1_awvalid; wire s1_awready;
    wire [31:0] s1_wdata; wire [3:0] s1_wstrb; wire s1_wlast; wire s1_wvalid; wire s1_wready;
    wire [6:0] s1_bid; wire [1:0] s1_bresp; wire s1_bvalid; wire s1_bready;
    wire [6:0] s1_arid; wire [31:0] s1_araddr; wire [7:0] s1_arlen; wire [2:0] s1_arsize; wire [1:0] s1_arburst; wire [2:0] s1_arprot; wire s1_arvalid; wire s1_arready;
    wire [6:0] s1_rid; wire [31:0] s1_rdata; wire [1:0] s1_rresp; wire s1_rlast; wire s1_rvalid; wire s1_rready;

    wire [6:0] s2_awid; wire [31:0] s2_awaddr; wire [7:0] s2_awlen; wire [2:0] s2_awsize; wire [1:0] s2_awburst; wire [2:0] s2_awprot; wire s2_awvalid; wire s2_awready;
    wire [31:0] s2_wdata; wire [3:0] s2_wstrb; wire s2_wlast; wire s2_wvalid; wire s2_wready;
    wire [6:0] s2_bid; wire [1:0] s2_bresp; wire s2_bvalid; wire s2_bready;
    wire [6:0] s2_arid; wire [31:0] s2_araddr; wire [7:0] s2_arlen; wire [2:0] s2_arsize; wire [1:0] s2_arburst; wire [2:0] s2_arprot; wire s2_arvalid; wire s2_arready;
    wire [6:0] s2_rid; wire [31:0] s2_rdata; wire [1:0] s2_rresp; wire s2_rlast; wire s2_rvalid; wire s2_rready;

    wire [6:0] s3_awid; wire [31:0] s3_awaddr; wire [7:0] s3_awlen; wire [2:0] s3_awsize; wire [1:0] s3_awburst; wire [2:0] s3_awprot; wire s3_awvalid; wire s3_awready;
    wire [31:0] s3_wdata; wire [3:0] s3_wstrb; wire s3_wlast; wire s3_wvalid; wire s3_wready;
    wire [6:0] s3_bid; wire [1:0] s3_bresp; wire s3_bvalid; wire s3_bready;
    wire [6:0] s3_arid; wire [31:0] s3_araddr; wire [7:0] s3_arlen; wire [2:0] s3_arsize; wire [1:0] s3_arburst; wire [2:0] s3_arprot; wire s3_arvalid; wire s3_arready;
    wire [6:0] s3_rid; wire [31:0] s3_rdata; wire [1:0] s3_rresp; wire s3_rlast; wire s3_rvalid; wire s3_rready;

    wire [6:0] s4_awid; wire [31:0] s4_awaddr; wire [7:0] s4_awlen; wire [2:0] s4_awsize; wire [1:0] s4_awburst; wire [2:0] s4_awprot; wire s4_awvalid; wire s4_awready;
    wire [31:0] s4_wdata; wire [3:0] s4_wstrb; wire s4_wlast; wire s4_wvalid; wire s4_wready;
    wire [6:0] s4_bid; wire [1:0] s4_bresp; wire s4_bvalid; wire s4_bready;
    wire [6:0] s4_arid; wire [31:0] s4_araddr; wire [7:0] s4_arlen; wire [2:0] s4_arsize; wire [1:0] s4_arburst; wire [2:0] s4_arprot; wire s4_arvalid; wire s4_arready;
    wire [6:0] s4_rid; wire [31:0] s4_rdata; wire [1:0] s4_rresp; wire s4_rlast; wire s4_rvalid; wire s4_rready;

    wire [6:0] s5_awid; wire [31:0] s5_awaddr; wire [7:0] s5_awlen; wire [2:0] s5_awsize; wire [1:0] s5_awburst; wire [2:0] s5_awprot; wire s5_awvalid; wire s5_awready;
    wire [31:0] s5_wdata; wire [3:0] s5_wstrb; wire s5_wlast; wire s5_wvalid; wire s5_wready;
    wire [6:0] s5_bid; wire [1:0] s5_bresp; wire s5_bvalid; wire s5_bready;
    wire [6:0] s5_arid; wire [31:0] s5_araddr; wire [7:0] s5_arlen; wire [2:0] s5_arsize; wire [1:0] s5_arburst; wire [2:0] s5_arprot; wire s5_arvalid; wire s5_arready;
    wire [6:0] s5_rid; wire [31:0] s5_rdata; wire [1:0] s5_rresp; wire s5_rlast; wire s5_rvalid; wire s5_rready;

    assign {s5_awready, s4_awready, s3_awready, s2_awready, s1_awready, s0_awready} = s_axi_awready;
    assign s_axi_awid    = {s5_awid, s4_awid, s3_awid, s2_awid, s1_awid, s0_awid};
    assign s_axi_awaddr  = {s5_awaddr, s4_awaddr, s3_awaddr, s2_awaddr, s1_awaddr, s0_awaddr};
    assign s_axi_awlen   = {s5_awlen, s4_awlen, s3_awlen, s2_awlen, s1_awlen, s0_awlen};
    assign s_axi_awsize  = {s5_awsize, s4_awsize, s3_awsize, s2_awsize, s1_awsize, s0_awsize};
    assign s_axi_awburst = {s5_awburst, s4_awburst, s3_awburst, s2_awburst, s1_awburst, s0_awburst};
    assign s_axi_awprot  = {s5_awprot, s4_awprot, s3_awprot, s2_awprot, s1_awprot, s0_awprot};
    assign s_axi_awvalid = {s5_awvalid, s4_awvalid, s3_awvalid, s2_awvalid, s1_awvalid, s0_awvalid};

    assign {s5_wready, s4_wready, s3_wready, s2_wready, s1_wready, s0_wready} = s_axi_wready;
    assign s_axi_wdata   = {s5_wdata, s4_wdata, s3_wdata, s2_wdata, s1_wdata, s0_wdata};
    assign s_axi_wstrb   = {s5_wstrb, s4_wstrb, s3_wstrb, s2_wstrb, s1_wstrb, s0_wstrb};
    assign s_axi_wlast   = {s5_wlast, s4_wlast, s3_wlast, s2_wlast, s1_wlast, s0_wlast};
    assign s_axi_wvalid  = {s5_wvalid, s4_wvalid, s3_wvalid, s2_wvalid, s1_wvalid, s0_wvalid};

    assign s_axi_bready  = {s5_bready, s4_bready, s3_bready, s2_bready, s1_bready, s0_bready};
    assign {s5_bid, s4_bid, s3_bid, s2_bid, s1_bid, s0_bid}             = s_axi_bid;
    assign {s5_bresp, s4_bresp, s3_bresp, s2_bresp, s1_bresp, s0_bresp}   = s_axi_bresp;
    assign {s5_bvalid, s4_bvalid, s3_bvalid, s2_bvalid, s1_bvalid, s0_bvalid} = s_axi_bvalid;

    assign {s5_arready, s4_arready, s3_arready, s2_arready, s1_arready, s0_arready} = s_axi_arready;
    assign s_axi_arid    = {s5_arid, s4_arid, s3_arid, s2_arid, s1_arid, s0_arid};
    assign s_axi_araddr  = {s5_araddr, s4_araddr, s3_araddr, s2_araddr, s1_araddr, s0_araddr};
    assign s_axi_arlen   = {s5_arlen, s4_arlen, s3_arlen, s2_arlen, s1_arlen, s0_arlen};
    assign s_axi_arsize  = {s5_arsize, s4_arsize, s3_arsize, s2_arsize, s1_arsize, s0_arsize};
    assign s_axi_arburst = {s5_arburst, s4_arburst, s3_arburst, s2_arburst, s1_arburst, s0_arburst};
    assign s_axi_arprot  = {s5_arprot, s4_arprot, s3_arprot, s2_arprot, s1_arprot, s0_arprot};
    assign s_axi_arvalid = {s5_arvalid, s4_arvalid, s3_arvalid, s2_arvalid, s1_arvalid, s0_arvalid};

    assign s_axi_rready  = {s5_rready, s4_rready, s3_rready, s2_rready, s1_rready, s0_rready};
    assign {s5_rid, s4_rid, s3_rid, s2_rid, s1_rid, s0_rid}             = s_axi_rid;
    assign {s5_rdata, s4_rdata, s3_rdata, s2_rdata, s1_rdata, s0_rdata}   = s_axi_rdata;
    assign {s5_rresp, s4_rresp, s3_rresp, s2_rresp, s1_rresp, s0_rresp}   = s_axi_rresp;
    assign {s5_rlast, s4_rlast, s3_rlast, s2_rlast, s1_rlast, s0_rlast}   = s_axi_rlast;
    assign {s5_rvalid, s4_rvalid, s3_rvalid, s2_rvalid, s1_rvalid, s0_rvalid} = s_axi_rvalid;

    // =========================================================================
    // 6. INSTANTIATE CÁC MASTER MODULES
    // =========================================================================    
// --- KHAI BÁO DÂY LÕI (CHẠY clk_cpu 400MHz) ---
    wire [4:0]  ic_arid;    wire [31:0] ic_araddr;  wire [7:0]  ic_arlen;   
    wire [2:0]  ic_arsize;  wire [1:0]  ic_arburst; wire [2:0]  ic_arprot;  
    wire        ic_arvalid; wire        ic_arready; wire [4:0]  ic_rid;     
    wire [31:0] ic_rdata;   wire [1:0]  ic_rresp;   wire        ic_rlast;   
    wire        ic_rvalid;  wire        ic_rready;

    // --- M0. ICACHE ---
    wire ic_uncache_en = (cpu_inst_addr >= 32'h4000_0000 && cpu_inst_addr <= 32'h47FF_FFFF);
    instruction_cache u_icache (
        .clk             (clk_cpu),              // clk_cpu (400MHz)
        .rst_n           (reset_core_n_sync),
        .cpu_read_req    (cpu_inst_req),
        .cpu_addr        (cpu_inst_addr),
        .uncache_en      (ic_uncache_en),
        .cpu_read_data   (cpu_inst_data),
        .icache_hit      (cpu_inst_hit),
        .icache_stall    (cpu_inst_stall),
        
        // Nối vào dây lõi ICache (400MHz)
        .m_axi_arid      (ic_arid),   .m_axi_araddr  (ic_araddr), .m_axi_arlen   (ic_arlen),
        .m_axi_arsize    (ic_arsize), .m_axi_arburst (ic_arburst),.m_axi_arvalid (ic_arvalid),
        .m_axi_arready   (ic_arready),.m_axi_rdata   (ic_rdata),  .m_axi_rresp   (ic_rresp),
        .m_axi_rlast     (ic_rlast),  .m_axi_rvalid  (ic_rvalid), .m_axi_rready  (ic_rready)
    );

    // --- CẦU NỐI ICACHE (400MHz) -> AXI INTERCONNECT M0 (200MHz) ---
    axi_async_bridge u_ic_axi_bridge (
        .s_clk(clk_cpu), .s_rst_n(reset_core_n_sync),
        
        // Kênh Write Slave: Ép cứng bằng 0 vì ICache không bao giờ ghi
        .s_axi_awid(5'b0), .s_axi_awaddr(32'b0), .s_axi_awlen(8'b0), .s_axi_awsize(3'b0), .s_axi_awburst(2'b0), .s_axi_awprot(3'b0), .s_axi_awvalid(1'b0), .s_axi_awready(),
        .s_axi_wdata(32'b0), .s_axi_wstrb(4'b0), .s_axi_wlast(1'b0), .s_axi_wvalid(1'b0), .s_axi_wready(),
        .s_axi_bid(), .s_axi_bresp(), .s_axi_bvalid(), .s_axi_bready(1'b1),
        
        // Kênh Read Slave: Nhận từ ICache
        .s_axi_arid(ic_arid), .s_axi_araddr(ic_araddr), .s_axi_arlen(ic_arlen), .s_axi_arsize(ic_arsize), .s_axi_arburst(ic_arburst), .s_axi_arprot(ic_arprot), .s_axi_arvalid(ic_arvalid), .s_axi_arready(ic_arready),
        .s_axi_rid(ic_rid), .s_axi_rdata(ic_rdata), .s_axi_rresp(ic_rresp), .s_axi_rlast(ic_rlast), .s_axi_rvalid(ic_rvalid), .s_axi_rready(ic_rready),
        
        .m_clk(clk_axi), .m_rst_n(reset_axi_n_sync),
        // Nối ra dây m0* (200MHz) đi vào AXI Interconnect
        .m_axi_awid(m0_awid), .m_axi_awaddr(m0_awaddr), .m_axi_awlen(m0_awlen), .m_axi_awsize(m0_awsize), .m_axi_awburst(m0_awburst), .m_axi_awprot(m0_awprot), .m_axi_awvalid(m0_awvalid), .m_axi_awready(m0_awready),
        .m_axi_wdata(m0_wdata), .m_axi_wstrb(m0_wstrb), .m_axi_wlast(m0_wlast), .m_axi_wvalid(m0_wvalid), .m_axi_wready(m0_wready),
        .m_axi_bid(m0_bid), .m_axi_bresp(m0_bresp), .m_axi_bvalid(m0_bvalid), .m_axi_bready(m0_bready),
        .m_axi_arid(m0_arid), .m_axi_araddr(m0_araddr), .m_axi_arlen(m0_arlen), .m_axi_arsize(m0_arsize), .m_axi_arburst(m0_arburst), .m_axi_arprot(m0_arprot), .m_axi_arvalid(m0_arvalid), .m_axi_arready(m0_arready),
        .m_axi_rid(m0_rid), .m_axi_rdata(m0_rdata), .m_axi_rresp(m0_rresp), .m_axi_rlast(m0_rlast), .m_axi_rvalid(m0_rvalid), .m_axi_rready(m0_rready)
    );

    // --- KHAI BÁO DÂY LÕI DCACHE (CHẠY clk_cpu 400MHz) ---
    wire [4:0]  dc_awid;    wire [31:0] dc_awaddr;  wire [7:0]  dc_awlen;   wire [2:0]  dc_awsize;  wire [1:0]  dc_awburst; wire [2:0]  dc_awprot;  wire dc_awvalid; wire dc_awready;
    wire [31:0] dc_wdata;   wire [3:0]  dc_wstrb;   wire        dc_wlast;   wire        dc_wvalid;  wire dc_wready;
    wire [4:0]  dc_bid;     wire [1:0]  dc_bresp;   wire        dc_bvalid;  wire        dc_bready;
    wire [4:0]  dc_arid;    wire [31:0] dc_araddr;  wire [7:0]  dc_arlen;   wire [2:0]  dc_arsize;  wire [1:0]  dc_arburst; wire [2:0]  dc_arprot;  wire dc_arvalid; wire dc_arready;
    wire [4:0]  dc_rid;     wire [31:0] dc_rdata;   wire [1:0]  dc_rresp;   wire        dc_rlast;   wire dc_rvalid; wire dc_rready;

    // --- M1. DCACHE ---
    wire dc_uncache_en = (cpu_data_addr >= 32'h4000_0000 && cpu_data_addr <= 32'h47FF_FFFF);
    data_cache u_dcache (
        .clk             (clk_cpu),              // clk_cpu (400MHz)
        .rst_n           (reset_core_n_sync),
        .cpu_read_req    (cpu_data_rd_req),
        .cpu_write_req   (cpu_data_wr_req),
        .cpu_addr        (cpu_data_addr),
        .cpu_write_data  (cpu_data_wdata),
        .mem_unsigned    (cpu_data_unsigned),
        .mem_size        (cpu_data_size),
        .uncache_en      (dc_uncache_en),
        .cpu_read_data   (cpu_data_rdata),
        .dcache_hit      (cpu_data_hit),
        .dcache_stall    (cpu_data_stall),
        
        // Nối vào dây lõi DCache (400MHz)
        .m_axi_awid      (dc_awid),   .m_axi_awaddr  (dc_awaddr), .m_axi_awlen   (dc_awlen),
        .m_axi_awsize    (dc_awsize), .m_axi_awburst (dc_awburst),.m_axi_awvalid (dc_awvalid), .m_axi_awready (dc_awready),
        .m_axi_wdata     (dc_wdata),  .m_axi_wstrb   (dc_wstrb),  .m_axi_wlast   (dc_wlast),  .m_axi_wvalid  (dc_wvalid), .m_axi_wready (dc_wready),
        .m_axi_bresp     (dc_bresp),  .m_axi_bvalid  (dc_bvalid), .m_axi_bready  (dc_bready),
        .m_axi_arid      (dc_arid),   .m_axi_araddr  (dc_araddr), .m_axi_arlen   (dc_arlen),
        .m_axi_arsize    (dc_arsize), .m_axi_arburst (dc_arburst),.m_axi_arvalid (dc_arvalid), .m_axi_arready (dc_arready),
        .m_axi_rdata     (dc_rdata),  .m_axi_rresp   (dc_rresp),  .m_axi_rlast   (dc_rlast),  .m_axi_rvalid  (dc_rvalid), .m_axi_rready (dc_rready)
    );

    // --- CẦU NỐI DCACHE (400MHz) -> AXI INTERCONNECT M1 (200MHz) ---
    axi_async_bridge u_dc_axi_bridge (
        .s_clk(clk_cpu), .s_rst_n(reset_core_n_sync),
        .s_axi_awid(dc_awid), .s_axi_awaddr(dc_awaddr), .s_axi_awlen(dc_awlen), .s_axi_awsize(dc_awsize), .s_axi_awburst(dc_awburst), .s_axi_awprot(dc_awprot), .s_axi_awvalid(dc_awvalid), .s_axi_awready(dc_awready),
        .s_axi_wdata(dc_wdata), .s_axi_wstrb(dc_wstrb), .s_axi_wlast(dc_wlast), .s_axi_wvalid(dc_wvalid), .s_axi_wready(dc_wready),
        .s_axi_bid(dc_bid), .s_axi_bresp(dc_bresp), .s_axi_bvalid(dc_bvalid), .s_axi_bready(dc_bready),
        .s_axi_arid(dc_arid), .s_axi_araddr(dc_araddr), .s_axi_arlen(dc_arlen), .s_axi_arsize(dc_arsize), .s_axi_arburst(dc_arburst), .s_axi_arprot(dc_arprot), .s_axi_arvalid(dc_arvalid), .s_axi_arready(dc_arready),
        .s_axi_rid(dc_rid), .s_axi_rdata(dc_rdata), .s_axi_rresp(dc_rresp), .s_axi_rlast(dc_rlast), .s_axi_rvalid(dc_rvalid), .s_axi_rready(dc_rready),
        
        .m_clk(clk_axi), .m_rst_n(reset_axi_n_sync),
        // Nối ra dây m1_* (200MHz) đi vào AXI Interconnect
        .m_axi_awid(m1_awid), .m_axi_awaddr(m1_awaddr), .m_axi_awlen(m1_awlen), .m_axi_awsize(m1_awsize), .m_axi_awburst(m1_awburst), .m_axi_awprot(m1_awprot), .m_axi_awvalid(m1_awvalid), .m_axi_awready(m1_awready),
        .m_axi_wdata(m1_wdata), .m_axi_wstrb(m1_wstrb), .m_axi_wlast(m1_wlast), .m_axi_wvalid(m1_wvalid), .m_axi_wready(m1_wready),
        .m_axi_bid(m1_bid), .m_axi_bresp(m1_bresp), .m_axi_bvalid(m1_bvalid), .m_axi_bready(m1_bready),
        .m_axi_arid(m1_arid), .m_axi_araddr(m1_araddr), .m_axi_arlen(m1_arlen), .m_axi_arsize(m1_arsize), .m_axi_arburst(m1_arburst), .m_axi_arprot(m1_arprot), .m_axi_arvalid(m1_arvalid), .m_axi_arready(m1_arready),
        .m_axi_rid(m1_rid), .m_axi_rdata(m1_rdata), .m_axi_rresp(m1_rresp), .m_axi_rlast(m1_rlast), .m_axi_rvalid(m1_rvalid), .m_axi_rready(m1_rready)
    );

    // M2: Debug Module (JTAG + DTM AXI Master)
    wire dmi_req_valid, dmi_resp_valid, dmi_resp_ready;
    wire [6:0] dmi_req_addr; wire [31:0] dmi_req_data, dmi_resp_data; wire [1:0] dmi_req_op, dmi_resp_op;

    rv_jtag_dtm u_jtag_dtm (
        .tck(tck), .trst_n(trst_n), .tms(tms), .tdi(tdi), .tdo(tdo),
        .dmi_req_valid(dmi_req_valid), .dmi_req_addr(dmi_req_addr), .dmi_req_data(dmi_req_data), .dmi_req_op(dmi_req_op),
        .dmi_resp_ready(dmi_resp_ready), .dmi_resp_valid(dmi_resp_valid), .dmi_resp_data(dmi_resp_data), .dmi_resp_op(dmi_resp_op)
    );

    wire sba_req, sba_ack;
    wire [1:0] sba_op, sba_size, sba_resp;
    wire [31:0] sba_addr, sba_wdata, sba_rdata;

    rv_debug_module_sba u_debug_module (
        .clk_sys            (clk_dbg), // Dùng clk_dbg (gated clk_axi)
        .rst_sys_n          (reset_axi_n_sync),
        .dmi_req_valid      (dmi_req_valid), .dmi_req_addr(dmi_req_addr), .dmi_req_data(dmi_req_data), .dmi_req_op(dmi_req_op),
        .dmi_resp_ready     (dmi_resp_ready), .dmi_resp_valid(dmi_resp_valid), .dmi_resp_data(dmi_resp_data), .dmi_resp_op(dmi_resp_op),
        .axi_req            (sba_req), .axi_op(sba_op), .axi_size(sba_size), .axi_addr(sba_addr), .axi_wdata(sba_wdata), .axi_ack(sba_ack), .axi_rdata(sba_rdata), .axi_resp(sba_resp),
        .cpu_halt_req       (dbg_halt_req_raw), .cpu_resume_req (dbg_resume_req_raw), .cpu_halted (dbg_halted_sync),
        .cpu_reg_read_addr  (dbg_reg_read_addr), .cpu_reg_read_data(dbg_reg_read_data), .cpu_reg_write_en(dbg_reg_write_en), .cpu_reg_write_addr(dbg_reg_write_addr), .cpu_reg_write_data(dbg_reg_write_data),
        .ndmreset_req       (ndmreset_req)
    );

    dtm_axi_master u_dtm_axi (
        .clk_sys         (clk_dbg), // Dùng clk_dbg
        .rst_sys_n       (reset_axi_n_sync),
        .i_req           (sba_req), .i_op(sba_op), .i_size(sba_size), .i_addr(sba_addr), .i_wdata(sba_wdata), .o_ack(sba_ack), .o_resp(sba_resp), .o_rdata(sba_rdata),
        .m_axi_awid      (m2_awid), .m_axi_awaddr(m2_awaddr), .m_axi_awlen(m2_awlen), .m_axi_awsize(m2_awsize), .m_axi_awburst(m2_awburst), .m_axi_awvalid(m2_awvalid), .m_axi_awready(m2_awready),
        .m_axi_wdata     (m2_wdata), .m_axi_wstrb(m2_wstrb), .m_axi_wlast(m2_wlast), .m_axi_wvalid(m2_wvalid), .m_axi_wready(m2_wready),
        .m_axi_bresp     (m2_bresp), .m_axi_bvalid(m2_bvalid), .m_axi_bready(m2_bready),
        .m_axi_arid      (m2_arid), .m_axi_araddr(m2_araddr), .m_axi_arlen(m2_arlen), .m_axi_arsize(m2_arsize), .m_axi_arburst(m2_arburst), .m_axi_arvalid(m2_arvalid), .m_axi_arready(m2_arready),
        .m_axi_rdata     (m2_rdata), .m_axi_rresp(m2_rresp), .m_axi_rlast(m2_rlast), .m_axi_rvalid(m2_rvalid), .m_axi_rready(m2_rready)
    );

    // =========================================================================
    // 7. AXI INTERCONNECT
    // =========================================================================
    axi_interconnect #(
        .MST_AMT(MST_AMT), .SLV_AMT(SLV_AMT),
        .MST_WEIGHT (128'h00000005_00000004_00000003_00000001),
        .SLV_BASE_ADDR (192'h0200_0000_4000_0000_8000_0000_3000_0000_2000_0000_0001_0000),
        .SLV_ADDR_MASK (192'hFFFF_0000_F800_0000_FC00_0000_FF00_0000_FFFE_0000_FFFF_0000)
    ) u_axi_interconnect (
        .ACLK_i          (clk_axi), // AXI Bus chạy clk_axi (luôn sống)
        .ARESETn_i       (reset_axi_n_sync),
        .m_AWID_i(m_axi_awid), .m_AWADDR_i(m_axi_awaddr), .m_AWBURST_i(m_axi_awburst), .m_AWLEN_i(m_axi_awlen), .m_AWSIZE_i(m_axi_awsize), .m_AWVALID_i(m_axi_awvalid), .m_AWREADY_o(m_axi_awready),
        .m_WDATA_i(m_axi_wdata), .m_WSTRB_i(m_axi_wstrb), .m_WLAST_i(m_axi_wlast), .m_WVALID_i(m_axi_wvalid), .m_WREADY_o(m_axi_wready),
        .m_BID_o(m_axi_bid), .m_BRESP_o(m_axi_bresp), .m_BVALID_o(m_axi_bvalid), .m_BREADY_i(m_axi_bready),
        .m_ARID_i(m_axi_arid), .m_ARADDR_i(m_axi_araddr), .m_ARBURST_i(m_axi_arburst), .m_ARLEN_i(m_axi_arlen), .m_ARSIZE_i(m_axi_arsize), .m_ARVALID_i(m_axi_arvalid), .m_ARREADY_o(m_axi_arready),
        .m_RID_o(m_axi_rid), .m_RDATA_o(m_axi_rdata), .m_RRESP_o(m_axi_rresp), .m_RLAST_o(m_axi_rlast), .m_RVALID_o(m_axi_rvalid), .m_RREADY_i(m_axi_rready),
        .s_AWID_o(s_axi_awid), .s_AWADDR_o(s_axi_awaddr), .s_AWBURST_o(s_axi_awburst), .s_AWLEN_o(s_axi_awlen), .s_AWSIZE_o(s_axi_awsize), .s_AWVALID_o(s_axi_awvalid), .s_AWREADY_i(s_axi_awready),
        .s_WDATA_o(s_axi_wdata), .s_WSTRB_o(s_axi_wstrb), .s_WLAST_o(s_axi_wlast), .s_WVALID_o(s_axi_wvalid), .s_WREADY_i(s_axi_wready),
        .s_BID_i(s_axi_bid), .s_BRESP_i(s_axi_bresp), .s_BVALID_i(s_axi_bvalid), .s_BREADY_o(s_axi_bready),
        .s_ARID_o(s_axi_arid), .s_ARADDR_o(s_axi_araddr), .s_ARBURST_o(s_axi_arburst), .s_ARLEN_o(s_axi_arlen), .s_ARSIZE_o(s_axi_arsize), .s_ARVALID_o(s_axi_arvalid), .s_ARREADY_i(s_axi_arready),
        .s_RID_i(s_axi_rid), .s_RDATA_i(s_axi_rdata), .s_RRESP_i(s_axi_rresp), .s_RLAST_i(s_axi_rlast), .s_RVALID_i(s_axi_rvalid), .s_RREADY_o(s_axi_rready)
    );

    // =========================================================================
    // 8. KHỞI TẠO CÁC AXI SLAVES
    // =========================================================================
    axi_rom #(.ID_WIDTH (7)) u_axi_rom (
        .clk(clk_axi), .rst_n(reset_axi_n_sync),
        .s_axi_arid(s0_arid), .s_axi_araddr(s0_araddr), .s_axi_arlen(s0_arlen), .s_axi_arsize(s0_arsize), .s_axi_arburst(s0_arburst), .s_axi_arvalid(s0_arvalid), .s_axi_arready(s0_arready),
        .s_axi_rid(s0_rid), .s_axi_rdata(s0_rdata), .s_axi_rresp(s0_rresp), .s_axi_rlast(s0_rlast), .s_axi_rvalid(s0_rvalid), .s_axi_rready(s0_rready)
    );
    assign s0_awready = 0; assign s0_wready = 0; assign s0_bvalid = 0;

    axi_ram #(.ID_WIDTH (7)) u_axi_ram (
        .clk(clk_axi), .rst_n(reset_axi_n_sync),
        .s_axi_awid(s1_awid), .s_axi_awaddr(s1_awaddr), .s_axi_awlen(s1_awlen), .s_axi_awsize(s1_awsize), .s_axi_awburst(s1_awburst), .s_axi_awvalid(s1_awvalid), .s_axi_awready(s1_awready),
        .s_axi_wdata(s1_wdata), .s_axi_wstrb(s1_wstrb), .s_axi_wlast(s1_wlast), .s_axi_wvalid(s1_wvalid), .s_axi_wready(s1_wready),
        .s_axi_bid(s1_bid), .s_axi_bresp(s1_bresp), .s_axi_bvalid(s1_bvalid), .s_axi_bready(s1_bready),
        .s_axi_arid(s1_arid), .s_axi_araddr(s1_araddr), .s_axi_arlen(s1_arlen), .s_axi_arsize(s1_arsize), .s_axi_arburst(s1_arburst), .s_axi_arvalid(s1_arvalid), .s_axi_arready(s1_arready),
        .s_axi_rid(s1_rid), .s_axi_rdata(s1_rdata), .s_axi_rresp(s1_rresp), .s_axi_rlast(s1_rlast), .s_axi_rvalid(s1_rvalid), .s_axi_rready(s1_rready)
    );

    wire flash_io0_o, flash_io0_oe, flash_io0_i;
    wire flash_io1_o, flash_io1_oe, flash_io1_i;
    wire flash_io2_o, flash_io2_oe, flash_io2_i;
    wire flash_io3_o, flash_io3_oe, flash_io3_i;

    // Chiều OUT: Nếu Output Enable (OE) = 1 thì đẩy dữ liệu ra, nếu bằng 0 thì nhả ra (High-Z)
    assign flash_io[0] = flash_io0_oe ? flash_io0_o : 1'bz;
    assign flash_io[1] = flash_io1_oe ? flash_io1_o : 1'bz;
    assign flash_io[2] = flash_io2_oe ? flash_io2_o : 1'bz;
    assign flash_io[3] = flash_io3_oe ? flash_io3_o : 1'bz;

    // Chiều IN: Luôn luôn đọc dữ liệu từ chân vật lý vào trong ruột chip
    assign flash_io0_i = flash_io[0];
    assign flash_io1_i = flash_io[1];
    assign flash_io2_i = flash_io[2];
    assign flash_io3_i = flash_io[3];

    axi_spi_flash #(.ID_WIDTH (7)) u_axi_flash (
        .clk(clk_axi), .rst_n(reset_axi_n_sync),
        
        // Giao diện Quad SPI vật lý
        .spi_clk_o  (flash_sck), 
        .spi_cs_n_o (flash_cs_n), 
        .spi_io0_o  (flash_io0_o), .spi_io0_i  (flash_io0_i), .spi_io0_oe (flash_io0_oe),
        .spi_io1_o  (flash_io1_o), .spi_io1_i  (flash_io1_i), .spi_io1_oe (flash_io1_oe),
        .spi_io2_o  (flash_io2_o), .spi_io2_i  (flash_io2_i), .spi_io2_oe (flash_io2_oe),
        .spi_io3_o  (flash_io3_o), .spi_io3_i  (flash_io3_i), .spi_io3_oe (flash_io3_oe),

        // Các kênh AXI giữ nguyên
        .s_axi_arid   (s2_arid),    .s_axi_araddr (s2_araddr), 
        .s_axi_arlen  (s2_arlen),   .s_axi_arsize (s2_arsize), 
        .s_axi_arburst(s2_arburst), .s_axi_arvalid(s2_arvalid), 
        .s_axi_arready(s2_arready),
        .s_axi_rid    (s2_rid),     .s_axi_rdata  (s2_rdata), 
        .s_axi_rresp  (s2_rresp),   .s_axi_rlast  (s2_rlast), 
        .s_axi_rvalid (s2_rvalid),  .s_axi_rready (s2_rready),
        
        // Cắm cứng kênh Ghi bằng 0 vì module bạn chặn kênh Ghi rồi (Hoặc nối vào s2_*)
        .s_axi_awid(s2_awid), .s_axi_awaddr(s2_awaddr), .s_axi_awlen(s2_awlen), .s_axi_awsize(s2_awsize), .s_axi_awburst(s2_awburst), .s_axi_awvalid(s2_awvalid), .s_axi_awready(s2_awready),
        .s_axi_wdata(s2_wdata), .s_axi_wstrb(s2_wstrb), .s_axi_wlast(s2_wlast), .s_axi_wvalid(s2_wvalid), .s_axi_wready(s2_wready),
        .s_axi_bid(s2_bid), .s_axi_bresp(s2_bresp), .s_axi_bvalid(s2_bvalid), .s_axi_bready(s2_bready)
    );

    wire [15:0] sdram_dq_i, sdram_dq_o; wire sdram_dq_oe;
    assign sdram_dq   = sdram_dq_oe ? sdram_dq_o : 16'bz;
    assign sdram_dq_i = sdram_dq;

    wire [15:0] sdram_dq_i_sync;
    
    // Đồng bộ tín hiệu DQ về clk_sdram_ctrl thay vì clk_axi
    cdc_sync_vector #(.WIDTH(16), .STAGES(2)) u_sync_sdram_dq (
        .clk_dst   (clk_sdram_ctrl),     
        .rst_dst_n (reset_sdram_n_sync),
        .d_in      (sdram_dq_i),         
        .q_out     (sdram_dq_i_sync)     
    );

    // Dây tín hiệu AXI sau khi qua cầu CDC sang miền 100MHz
    wire [6:0]  sdram_axi_awid;   wire [31:0] sdram_axi_awaddr; wire [7:0] sdram_axi_awlen;
    wire [2:0]  sdram_axi_awsize; wire [1:0]  sdram_axi_awburst;wire [2:0] sdram_axi_awprot;
    wire        sdram_axi_awvalid;wire        sdram_axi_awready;
    wire [31:0] sdram_axi_wdata;  wire [3:0]  sdram_axi_wstrb;  wire       sdram_axi_wlast;
    wire        sdram_axi_wvalid; wire        sdram_axi_wready;
    wire [6:0]  sdram_axi_bid;    wire [1:0]  sdram_axi_bresp;  wire       sdram_axi_bvalid; wire sdram_axi_bready;
    wire [6:0]  sdram_axi_arid;   wire [31:0] sdram_axi_araddr; wire [7:0] sdram_axi_arlen;
    wire [2:0]  sdram_axi_arsize; wire [1:0]  sdram_axi_arburst;wire [2:0] sdram_axi_arprot;
    wire        sdram_axi_arvalid;wire        sdram_axi_arready;
    wire [6:0]  sdram_axi_rid;    wire [31:0] sdram_axi_rdata;  wire [1:0] sdram_axi_rresp;
    wire        sdram_axi_rlast;  wire        sdram_axi_rvalid; wire       sdram_axi_rready;

    // CẦU CDC CHO SDRAM: Chuyển s3_* (200MHz) sang sdram_axi_* (100MHz)
    axi_async_bridge #(
        .ADDR_WIDTH(32),
        .DATA_WIDTH(32),
        .ID_WIDTH(7) // Kênh Slave trong Interconnect của bạn dùng ID 7 bit
    ) u_axi_cdc_sdram (
        .s_clk(clk_axi), .s_rst_n(reset_axi_n_sync),
        .s_axi_awid(s3_awid), .s_axi_awaddr(s3_awaddr), .s_axi_awlen(s3_awlen), .s_axi_awsize(s3_awsize), .s_axi_awburst(s3_awburst), .s_axi_awprot(s3_awprot), .s_axi_awvalid(s3_awvalid), .s_axi_awready(s3_awready),
        .s_axi_wdata(s3_wdata), .s_axi_wstrb(s3_wstrb), .s_axi_wlast(s3_wlast), .s_axi_wvalid(s3_wvalid), .s_axi_wready(s3_wready),
        .s_axi_bid(s3_bid), .s_axi_bresp(s3_bresp), .s_axi_bvalid(s3_bvalid), .s_axi_bready(s3_bready),
        .s_axi_arid(s3_arid), .s_axi_araddr(s3_araddr), .s_axi_arlen(s3_arlen), .s_axi_arsize(s3_arsize), .s_axi_arburst(s3_arburst), .s_axi_arprot(s3_arprot), .s_axi_arvalid(s3_arvalid), .s_axi_arready(s3_arready),
        .s_axi_rid(s3_rid), .s_axi_rdata(s3_rdata), .s_axi_rresp(s3_rresp), .s_axi_rlast(s3_rlast), .s_axi_rvalid(s3_rvalid), .s_axi_rready(s3_rready),

        .m_clk(clk_sdram_ctrl), .m_rst_n(reset_sdram_n_sync),
        .m_axi_awid(sdram_axi_awid), .m_axi_awaddr(sdram_axi_awaddr), .m_axi_awlen(sdram_axi_awlen), .m_axi_awsize(sdram_axi_awsize), .m_axi_awburst(sdram_axi_awburst), .m_axi_awprot(sdram_axi_awprot), .m_axi_awvalid(sdram_axi_awvalid), .m_axi_awready(sdram_axi_awready),
        .m_axi_wdata(sdram_axi_wdata), .m_axi_wstrb(sdram_axi_wstrb), .m_axi_wlast(sdram_axi_wlast), .m_axi_wvalid(sdram_axi_wvalid), .m_axi_wready(sdram_axi_wready),
        .m_axi_bid(sdram_axi_bid), .m_axi_bresp(sdram_axi_bresp), .m_axi_bvalid(sdram_axi_bvalid), .m_axi_bready(sdram_axi_bready),
        .m_axi_arid(sdram_axi_arid), .m_axi_araddr(sdram_axi_araddr), .m_axi_arlen(sdram_axi_arlen), .m_axi_arsize(sdram_axi_arsize), .m_axi_arburst(sdram_axi_arburst), .m_axi_arprot(sdram_axi_arprot), .m_axi_arvalid(sdram_axi_arvalid), .m_axi_arready(sdram_axi_arready),
        .m_axi_rid(sdram_axi_rid), .m_axi_rdata(sdram_axi_rdata), .m_axi_rresp(sdram_axi_rresp), .m_axi_rlast(sdram_axi_rlast), .m_axi_rvalid(sdram_axi_rvalid), .m_axi_rready(sdram_axi_rready)
    );

    // SDRAM CONTROLLER: Ép tham số 100MHz và nhận dây từ CDC
    axi_sdram_controller #(
        .ID_WIDTH (7)
    ) u_axi_sdram (
        .clk           (clk_sdram_ctrl),     // 100MHz nội bộ
        .clk_phy       (clk_sdram_phy),      // 100MHz lệch pha 180 độ
        .rst_n         (reset_sdram_n_sync),
        
        // Giao tiếp chip RAM (sdram_clk xuất trực tiếp ra port ngoài ở đây)
        .sdram_clk     (), 
        .sdram_cke     (sdram_cke), 
        .sdram_cs_n    (sdram_cs_n), 
        .sdram_ras_n   (sdram_ras_n),
        .sdram_cas_n   (sdram_cas_n), 
        .sdram_we_n    (sdram_we_n), 
        .sdram_ba      (sdram_ba), 
        .sdram_addr    (sdram_addr),
        .sdram_dqm     (sdram_dqm), 
        .sdram_dq_i    (sdram_dq_i_sync), 
        .sdram_dq_o    (sdram_dq_o), 
        .sdram_dq_oe   (sdram_dq_oe),
        
        // Nhận tín hiệu AXI 100MHz từ cầu CDC
        .s_axi_awid    (sdram_axi_awid), .s_axi_awaddr(sdram_axi_awaddr), .s_axi_awlen(sdram_axi_awlen), .s_axi_awsize(sdram_axi_awsize), .s_axi_awburst(sdram_axi_awburst), .s_axi_awvalid(sdram_axi_awvalid), .s_axi_awready(sdram_axi_awready),
        .s_axi_wdata   (sdram_axi_wdata), .s_axi_wstrb(sdram_axi_wstrb), .s_axi_wlast(sdram_axi_wlast), .s_axi_wvalid(sdram_axi_wvalid), .s_axi_wready(sdram_axi_wready),
        .s_axi_bid     (sdram_axi_bid), .s_axi_bresp(sdram_axi_bresp), .s_axi_bvalid(sdram_axi_bvalid), .s_axi_bready(sdram_axi_bready),
        .s_axi_arid    (sdram_axi_arid), .s_axi_araddr(sdram_axi_araddr), .s_axi_arlen(sdram_axi_arlen), .s_axi_arsize(sdram_axi_arsize), .s_axi_arburst(sdram_axi_arburst), .s_axi_arvalid(sdram_axi_arvalid), .s_axi_arready(sdram_axi_arready),
        .s_axi_rid     (sdram_axi_rid), .s_axi_rdata(sdram_axi_rdata), .s_axi_rresp(sdram_axi_rresp), .s_axi_rlast(sdram_axi_rlast), .s_axi_rvalid(sdram_axi_rvalid), .s_axi_rready(sdram_axi_rready)
    );

    wire [31:0] apb_paddr, apb_pwdata, apb_prdata; wire [3:0] apb_pstrb; wire [2:0] apb_pprot;
    wire apb_psel, apb_penable, apb_pwrite, apb_pready, apb_pslverr;
    axi_to_apb_bridge u_axi_to_apb (
        .clk_axi(clk_axi), .clk_apb(clk_apb), .rst_n(reset_axi_n_sync),
        .s_axi_awid(s4_awid), .s_axi_awaddr(s4_awaddr), .s_axi_awlen(s4_awlen), .s_axi_awsize(s4_awsize), .s_axi_awburst(s4_awburst), .s_axi_awprot(s4_awprot), .s_axi_awvalid(s4_awvalid), .s_axi_awready(s4_awready),
        .s_axi_wdata(s4_wdata), .s_axi_wstrb(s4_wstrb), .s_axi_wlast(s4_wlast), .s_axi_wvalid(s4_wvalid), .s_axi_wready(s4_wready),
        .s_axi_bid(s4_bid), .s_axi_bresp(s4_bresp), .s_axi_bvalid(s4_bvalid), .s_axi_bready(s4_bready),
        .s_axi_arid(s4_arid), .s_axi_araddr(s4_araddr), .s_axi_arlen(s4_arlen), .s_axi_arsize(s4_arsize), .s_axi_arburst(s4_arburst), .s_axi_arprot(s4_arprot), .s_axi_arvalid(s4_arvalid), .s_axi_arready(s4_arready),
        .s_axi_rid(s4_rid), .s_axi_rdata(s4_rdata), .s_axi_rresp(s4_rresp), .s_axi_rlast(s4_rlast), .s_axi_rvalid(s4_rvalid), .s_axi_rready(s4_rready),
        .m_apb_paddr(apb_paddr), .m_apb_psel(apb_psel), .m_apb_penable(apb_penable), .m_apb_pwrite(apb_pwrite), .m_apb_pwdata(apb_pwdata), .m_apb_pstrb(apb_pstrb), .m_apb_pprot(apb_pprot),
        .m_apb_pready(apb_pready), .m_apb_prdata(apb_prdata), .m_apb_pslverr(apb_pslverr)
    );

    // S5: CLINT
    axi_clint #(
        .NUM_HARTS      (1),
        .HART_IDX_W     (1),
        .AXI_ADDR_WIDTH (32),
        .AXI_ID_WIDTH   (7),     // AXI Interconnect sử dụng 7-bit ID cho Slave
        .PIPELINE_IRQ   (1)
    ) u_clint (
        // Clocks & Reset
        .clk_i          (clk_axi),
        .rst_ni         (reset_axi_n_sync),
        .rtc_clk_i      (rtc_clk),
        
        // AXI Write Address Channel
        .s_axi_awid     (s5_awid), 
        .s_axi_awaddr   (s5_awaddr), 
        .s_axi_awlen    (s5_awlen), 
        .s_axi_awsize   (s5_awsize), 
        .s_axi_awburst  (s5_awburst), 
        .s_axi_awprot   (s5_awprot), 
        .s_axi_awvalid  (s5_awvalid), 
        .s_axi_awready  (s5_awready),    
        
        // AXI Write Data Channel
        .s_axi_wdata    (s5_wdata), 
        .s_axi_wstrb    (s5_wstrb), 
        .s_axi_wlast    (s5_wlast), 
        .s_axi_wvalid   (s5_wvalid), 
        .s_axi_wready   (s5_wready),
        
        // AXI Write Response Channel
        .s_axi_bid      (s5_bid), 
        .s_axi_bresp    (s5_bresp), 
        .s_axi_bvalid   (s5_bvalid), 
        .s_axi_bready   (s5_bready),
        
        // AXI Read Address Channel
        .s_axi_arid     (s5_arid), 
        .s_axi_araddr   (s5_araddr), 
        .s_axi_arlen    (s5_arlen), 
        .s_axi_arsize   (s5_arsize), 
        .s_axi_arburst  (s5_arburst), 
        .s_axi_arprot   (s5_arprot), 
        .s_axi_arvalid  (s5_arvalid), 
        .s_axi_arready  (s5_arready),     
        
        // AXI Read Data Channel
        .s_axi_rid      (s5_rid), 
        .s_axi_rdata    (s5_rdata), 
        .s_axi_rresp    (s5_rresp), 
        .s_axi_rlast    (s5_rlast), 
        .s_axi_rvalid   (s5_rvalid), 
        .s_axi_rready   (s5_rready),
        
        // Interrupt Outputs (Nối vào tín hiệu raw đã được tạo, sẽ qua Sync bọc lại sau)
        .msip_o         (cpu_msip_raw),
        .mtip_o         (cpu_mtip_raw)
    );

    // =========================================================================
    // 9. APB INTERCONNECT VÀ CÁC NGOẠI VI
    // =========================================================================
    wire [31:0] paddr_0, paddr_1, paddr_2, paddr_3, paddr_4, paddr_5, paddr_6, paddr_7, paddr_8, paddr_9;
    wire [31:0] pwdata_0, pwdata_1, pwdata_2, pwdata_3, pwdata_4, pwdata_5, pwdata_6, pwdata_7, pwdata_8, pwdata_9;
    wire [31:0] prdata_0, prdata_1, prdata_2, prdata_3, prdata_4, prdata_5, prdata_6, prdata_7, prdata_8, prdata_9;
    wire psel_0, psel_1, psel_2, psel_3, psel_4, psel_5, psel_6, psel_7, psel_8, psel_9;
    wire penable_0, penable_1, penable_2, penable_3, penable_4, penable_5, penable_6, penable_7, penable_8, penable_9;
    wire pwrite_0, pwrite_1, pwrite_2, pwrite_3, pwrite_4, pwrite_5, pwrite_6, pwrite_7, pwrite_8, pwrite_9;
    wire pready_0, pready_1, pready_2, pready_3, pready_4, pready_5, pready_6, pready_7, pready_8, pready_9;
    wire pslverr_0, pslverr_1, pslverr_2, pslverr_3, pslverr_4, pslverr_5, pslverr_6, pslverr_7, pslverr_8, pslverr_9;

    apb_interconnect u_apb_interconnect (
        .clk(clk_apb), .rst_n(reset_apb_n_sync),
        .m_paddr(apb_paddr), .m_psel(apb_psel), .m_penable(apb_penable), .m_pwrite(apb_pwrite), .m_pwdata(apb_pwdata), .m_pstrb(apb_pstrb), .m_pprot(apb_pprot), .m_pready(apb_pready), .m_prdata(apb_prdata), .m_pslverr(apb_pslverr),
        .s0_paddr(paddr_0), .s0_psel(psel_0), .s0_penable(penable_0), .s0_pwrite(pwrite_0), .s0_pwdata(pwdata_0), .s0_pready(pready_0), .s0_prdata(prdata_0), .s0_pslverr(pslverr_0),
        .s1_paddr(paddr_1), .s1_psel(psel_1), .s1_penable(penable_1), .s1_pwrite(pwrite_1), .s1_pwdata(pwdata_1), .s1_pready(pready_1), .s1_prdata(prdata_1), .s1_pslverr(pslverr_1),
        .s2_paddr(paddr_2), .s2_psel(psel_2), .s2_penable(penable_2), .s2_pwrite(pwrite_2), .s2_pwdata(pwdata_2), .s2_pready(pready_2), .s2_prdata(prdata_2), .s2_pslverr(pslverr_2),
        .s3_paddr(paddr_3), .s3_psel(psel_3), .s3_penable(penable_3), .s3_pwrite(pwrite_3), .s3_pwdata(pwdata_3), .s3_pready(pready_3), .s3_prdata(prdata_3), .s3_pslverr(pslverr_3),
        .s4_paddr(paddr_4), .s4_psel(psel_4), .s4_penable(penable_4), .s4_pwrite(pwrite_4), .s4_pwdata(pwdata_4), .s4_pready(pready_4), .s4_prdata(prdata_4), .s4_pslverr(pslverr_4),
        .s5_paddr(paddr_5), .s5_psel(psel_5), .s5_penable(penable_5), .s5_pwrite(pwrite_5), .s5_pwdata(pwdata_5), .s5_pready(pready_5), .s5_prdata(prdata_5), .s5_pslverr(pslverr_5),
        .s6_paddr(paddr_6), .s6_psel(psel_6), .s6_penable(penable_6), .s6_pwrite(pwrite_6), .s6_pwdata(pwdata_6), .s6_pready(pready_6), .s6_prdata(prdata_6), .s6_pslverr(pslverr_6),
        .s7_paddr(paddr_7), .s7_psel(psel_7), .s7_penable(penable_7), .s7_pwrite(pwrite_7), .s7_pwdata(pwdata_7), .s7_pready(pready_7), .s7_prdata(prdata_7), .s7_pslverr(pslverr_7),
        .s8_paddr(paddr_8), .s8_psel(psel_8), .s8_penable(penable_8), .s8_pwrite(pwrite_8), .s8_pwdata(pwdata_8), .s8_pready(pready_8), .s8_prdata(prdata_8), .s8_pslverr(pslverr_8),
        .s9_paddr(paddr_9), .s9_psel(psel_9), .s9_penable(penable_9), .s9_pwrite(pwrite_9), .s9_pwdata(pwdata_9), .s9_pready(pready_9), .s9_prdata(prdata_9), .s9_pslverr(pslverr_9)
    );

    // S0: UART
    apb_uart u_apb_uart (
        .pclk(clk_apb), .presetn(reset_apb_n_sync),
        .psel(psel_0), .penable(penable_0), .pwrite(pwrite_0), .paddr(paddr_0[11:0]), .pwdata(pwdata_0), .prdata(prdata_0), .pready(pready_0), .pslverr(pslverr_0),
        .uart_clk(clk_uart_gated), .uart_rst_n(reset_apb_n_sync), // Clock qua Gating
        .rxd(uart_rx), .txd(uart_tx),
        .uart_irq(uart_irq_raw), .dma_tx_req(uart_dma_tx_raw), .dma_rx_req(uart_dma_rx_raw)
    );

    // S1: GPIO (Dùng nguyên bản gốc)
    apb_gpio u_apb_gpio (
        .pclk(clk_gpio), .presetn(reset_apb_n_sync), // Clock qua Gating
        .psel(psel_1), .penable(penable_1), .pwrite(pwrite_1), .paddr(paddr_1[11:0]), .pwdata(pwdata_1), .prdata(prdata_1), .pready(pready_1), .pslverr(pslverr_1),
        .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_dir(gpio_oe), 
        .gpio_irq(gpio_irq_raw)
    );

    // S2: PWM
    apb_pwm u_apb_pwm (
        .pclk(clk_pwm), .presetn(reset_apb_n_sync), // Clock qua Gating
        .psel(psel_2), .penable(penable_2), .pwrite(pwrite_2), .paddr(paddr_2[11:0]), .pwdata(pwdata_2), .pstrb(apb_pstrb), .prdata(prdata_2), .pready(pready_2), .pslverr(pslverr_2),
        .pwm_out(pwm_out)
    );

    // S3: SPI
    apb_spi u_apb_spi (
        .pclk(clk_apb), .presetn(reset_apb_n_sync),
        .psel(psel_3), .penable(penable_3), .pwrite(pwrite_3), .paddr(paddr_3[11:0]), .pwdata(pwdata_3), .pstrb(apb_pstrb), .prdata(prdata_3), .pready(pready_3), .pslverr(pslverr_3),
        .spi_clk(clk_spi_gated), .spi_rst_n(reset_apb_n_sync), // Clock qua Gating
        .sclk(spi_sck), .mosi(spi_mosi), .miso(spi_miso), .cs_n(spi_ss),
        .spi_irq(spi_irq_raw), .dma_tx_req(spi_dma_tx_raw), .dma_rx_req(spi_dma_rx_raw)
    );

    // S4: I2C
    wire i2c_scl_o, i2c_scl_oe, i2c_sda_o, i2c_sda_oe;
    assign i2c_scl = i2c_scl_oe ? i2c_scl_o : 1'bz;
    assign i2c_sda = i2c_sda_oe ? i2c_sda_o : 1'bz;
    apb_i2c u_apb_i2c (
        .pclk(clk_apb), .presetn(reset_apb_n_sync),
        .psel(psel_4), .penable(penable_4), .pwrite(pwrite_4), .paddr(paddr_4[11:0]), .pwdata(pwdata_4), .pstrb(apb_pstrb), .prdata(prdata_4), .pready(pready_4), .pslverr(pslverr_4),
        .i2c_clk(clk_i2c_gated), .i2c_rst_n(reset_apb_n_sync), // Clock qua Gating
        .scl_o(i2c_scl_o), .scl_oen(i2c_scl_oe), .scl_i(i2c_scl), .sda_o(i2c_sda_o), .sda_oen(i2c_sda_oe), .sda_i(i2c_sda),
        .i2c_irq(i2c_irq_raw), .dma_tx_req(i2c_dma_tx_raw), .dma_rx_req(i2c_dma_rx_raw)
    );

    // S5: Watchdog (Đã hỗ trợ rtc_clk)
    apb_watchdog u_apb_watchdog (
        .pclk(clk_apb), .presetn(reset_apb_n_sync),
        .psel(psel_5), .penable(penable_5), .pwrite(pwrite_5), .paddr(paddr_5[11:0]), .pwdata(pwdata_5), .pstrb(apb_pstrb), .prdata(prdata_5), .pready(pready_5), .pslverr(pslverr_5),
        .rtc_clk(rtc_clk), .rtc_rst_n(reset_apb_n_sync),
        .wdt_irq(wdt_irq_raw), .wdt_rst(wdt_rst)
    );

    // S6: CORDIC
    apb_cordic u_apb_cordic (
        .pclk(clk_cordic), .presetn(reset_apb_n_sync), // Clock qua Gating
        .psel(psel_6), .penable(penable_6), .pwrite(pwrite_6), .paddr(paddr_6[11:0]), .pwdata(pwdata_6), .prdata(prdata_6), .pready(pready_6), .pslverr(pslverr_6)
    );

    // S7: Syscon
    apb_syscon u_apb_syscon (
        .pclk(clk_apb), .presetn(rst_n), // Dùng Reset gốc
        .psel(psel_7), .penable(penable_7), .pwrite(pwrite_7), .paddr(paddr_7[11:0]), .pwdata(pwdata_7), .prdata(prdata_7), .pready(pready_7), .pslverr(pslverr_7),
        .o_reset_vector(syscon_reset_vector), .i_wfi_sleep(wfi_sleep_state), .i_ext_irq(cpu_meip_sync),
        .o_cpu_clk_en  (clk_en_cpu), 
        .o_dbg_clk_en  (clk_en_dbg),
        .o_pwm_clk_en  (clk_en_pwm),
        .o_urt_clk_en  (clk_en_uart),
        .o_spi_clk_en  (clk_en_spi),
        .o_i2c_clk_en  (clk_en_i2c),
        .o_gpo_clk_en  (clk_en_gpio),
        .o_acc_clk_en  (clk_en_acc)
    );

    // S8: PLIC
    apb_plic #(.ALGORITHM("BINARY_TREE")) u_apb_plic (
        .clk_i(clk_apb), .rst_ni(reset_apb_n_sync),
        .paddr(paddr_8), .psel(psel_8), .penable(penable_8), .pwrite(pwrite_8), .pwdata(pwdata_8), .pready(pready_8), .prdata(prdata_8), .pslverr(pslverr_8),
        .irq_src_i(plic_irq_src), .irq_o(cpu_meip_raw)
    );

    // =========================================================================
    // 10. DMA CONTROLLER VÀ APB ASYNC BRIDGE
    // =========================================================================
    // Các dây nối chạy ở tần số clk_axi (200MHz)
    wire [12:0] dma_paddr;
    wire [31:0] dma_pwdata, dma_prdata; 
    wire        dma_psel, dma_penable, dma_pwrite, dma_pready, dma_pslverr;

    // Cầu CDC: Chuyển lệnh cấu hình từ clk_apb (100MHz) sang clk_axi (200MHz)
    apb_async_bridge #(
        .ADDR_WIDTH(13),
        .DATA_WIDTH(32)
    ) u_apb_cdc_dma (
        .s_clk          (clk_apb),            // Nguồn 100MHz
        .s_rst_n        (reset_apb_n_sync),
        .s_apb_psel     (psel_9), 
        .s_apb_penable  (penable_9), 
        .s_apb_pwrite   (pwrite_9), 
        .s_apb_paddr    (paddr_9[12:0]), 
        .s_apb_pwdata   (pwdata_9), 
        .s_apb_prdata   (prdata_9), 
        .s_apb_pslverr  (pslverr_9), 
        .s_apb_pready   (pready_9),

        .m_clk          (clk_axi),            // Đích 200MHz
        .m_rst_n        (reset_axi_n_sync),
        .m_apb_psel     (dma_psel), 
        .m_apb_penable  (dma_penable), 
        .m_apb_pwrite   (dma_pwrite), 
        .m_apb_paddr    (dma_paddr), 
        .m_apb_pwdata   (dma_pwdata), 
        .m_apb_prdata   (dma_prdata), 
        .m_apb_pslverr  (dma_pslverr), 
        .m_apb_pready   (dma_pready)
    );

    // Module DMA giữ nguyên ruột, chạy hoàn toàn bằng clk_axi
    axi_apb_dma u_axi_apb_dma (
        .clk_bus        (clk_axi),          // Toàn bộ logic DMA chạy 200MHz
        .rst_bus_n      (reset_axi_n_sync),
        
        // Giao tiếp APB Slave (Nối vào đầu ra của cầu CDC)
        .s_apb_psel     (dma_psel), 
        .s_apb_penable  (dma_penable), 
        .s_apb_pwrite   (dma_pwrite), 
        .s_apb_paddr    (dma_paddr), 
        .s_apb_pwdata   (dma_pwdata), 
        .s_apb_prdata   (dma_prdata), 
        .s_apb_pslverr  (dma_pslverr), 
        .s_apb_pready   (dma_pready),

        // Ngắt và DMA Request từ ngoại vi (đã được đồng bộ clk_axi ở các bước trước)
        .dma_irq        (dma_irq), 
        .periph_dma_req (periph_dma_req), 
        .periph_dma_clr (periph_dma_clr),

        // M-AXI Interface
        // Write Address Channel
        .m_axi_awid     (m3_awid),
        .m_axi_awaddr   (m3_awaddr),
        .m_axi_awlen    (m3_awlen),
        .m_axi_awsize   (m3_awsize),
        .m_axi_awburst  (m3_awburst),
        .m_axi_awvalid  (m3_awvalid),
        .m_axi_awready  (m3_awready),

        // Write Data Channel
        .m_axi_wdata    (m3_wdata),
        .m_axi_wstrb    (m3_wstrb),
        .m_axi_wlast    (m3_wlast),
        .m_axi_wvalid   (m3_wvalid),
        .m_axi_wready   (m3_wready),

        // Write Response Channel
        .m_axi_bid      (m3_bid),
        .m_axi_bresp    (m3_bresp),
        .m_axi_bvalid   (m3_bvalid),
        .m_axi_bready   (m3_bready),

        // Read Address Channel
        .m_axi_arid     (m3_arid),
        .m_axi_araddr   (m3_araddr),
        .m_axi_arlen    (m3_arlen),
        .m_axi_arsize   (m3_arsize),
        .m_axi_arburst  (m3_arburst),
        .m_axi_arvalid  (m3_arvalid),
        .m_axi_arready  (m3_arready),

        // Read Data Channel
        .m_axi_rid      (m3_rid),
        .m_axi_rdata    (m3_rdata),
        .m_axi_rresp    (m3_rresp),
        .m_axi_rlast    (m3_rlast),
        .m_axi_rvalid   (m3_rvalid),
        .m_axi_rready   (m3_rready)
    );

endmodule