`timescale 1ns / 1ps

/**
 * PROJECT: AXI4 INTERCONNECT VERIFICATION
 * SCALE: 16-PHASE GRANDMASTER TORTURE SUITE
 * COMPATIBILITY: VERILOG-2001 / SYSTEMVERILOG
 */

module tb_axi_interconnect();

    parameter ADDR_WIDTH = 32;
    parameter DATA_WIDTH = 32;

    reg clk;
    reg rst_n;

    // =========================================================================
    // MASTER INTERFACE SIGNALS
    // =========================================================================
    reg  [ADDR_WIDTH-1:0] m0_araddr; reg [7:0] m0_arlen; reg [2:0] m0_arsize; reg [1:0] m0_arburst; reg m0_arvalid; wire m0_arready;
    wire [DATA_WIDTH-1:0] m0_rdata; wire [1:0] m0_rresp; wire m0_rlast; wire m0_rvalid; reg m0_rready;

    reg  [ADDR_WIDTH-1:0] m1_awaddr; reg m1_awvalid; wire m1_awready; reg [DATA_WIDTH-1:0] m1_wdata; reg [3:0] m1_wstrb; reg m1_wvalid; wire m1_wready;
    wire [1:0] m1_bresp; wire m1_bvalid; reg m1_bready;
    reg  [ADDR_WIDTH-1:0] m1_araddr; reg m1_arvalid; wire m1_arready; wire [DATA_WIDTH-1:0] m1_rdata; wire [1:0] m1_rresp; wire m1_rvalid; reg m1_rready;

    reg  [ADDR_WIDTH-1:0] m2_awaddr; reg m2_awvalid; wire m2_awready; reg [DATA_WIDTH-1:0] m2_wdata; reg [3:0] m2_wstrb; reg m2_wvalid; wire m2_wready;
    wire [1:0] m2_bresp; wire m2_bvalid; reg m2_bready;
    reg  [ADDR_WIDTH-1:0] m2_araddr; reg m2_arvalid; wire m2_arready; wire [DATA_WIDTH-1:0] m2_rdata; wire [1:0] m2_rresp; wire m2_rvalid; reg m2_rready;

    // SLAVE INTERFACE SIGNALS
    wire [ADDR_WIDTH-1:0] s0_araddr, s1_awaddr, s1_araddr, s2_awaddr, s2_araddr, s3_araddr;
    wire [DATA_WIDTH-1:0] s0_rdata, s1_wdata, s1_rdata, s2_wdata, s2_rdata, s3_rdata;
    wire [7:0] s1_arlen; wire [3:0] s1_wstrb, s2_wstrb; wire [2:0] s1_arsize; wire [1:0] s0_rresp, s1_bresp, s1_arburst, s1_rresp, s2_bresp, s2_rresp, s3_rresp;
    wire s0_arvalid, s0_arready, s0_rvalid, s0_rready;
    wire s1_awvalid, s1_awready, s1_wvalid, s1_wready, s1_bvalid, s1_bready;
    wire s1_arvalid, s1_arready, s1_rvalid, s1_rready, s1_rlast;
    wire s2_awvalid, s2_awready, s2_wvalid, s2_wready, s2_bvalid, s2_bready;
    wire s2_arvalid, s2_arready, s2_rvalid, s2_rready;
    wire s3_arvalid, s3_arready, s3_rvalid, s3_rready;

    // =========================================================================
    // DUT INSTANTIATION
    // =========================================================================
    axi_interconnect #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) dut (
        .clk(clk), .rst_n(rst_n),
        .m0_araddr(m0_araddr), .m0_arlen(m0_arlen), .m0_arsize(m0_arsize), .m0_arburst(m0_arburst), .m0_arvalid(m0_arvalid), .m0_arready(m0_arready), .m0_rdata(m0_rdata), .m0_rresp(m0_rresp), .m0_rlast(m0_rlast), .m0_rvalid(m0_rvalid), .m0_rready(m0_rready),
        .m1_awaddr(m1_awaddr), .m1_awvalid(m1_awvalid), .m1_awready(m1_awready), .m1_wdata(m1_wdata), .m1_wstrb(m1_wstrb), .m1_wvalid(m1_wvalid), .m1_wready(m1_wready), .m1_bresp(m1_bresp), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready), .m1_araddr(m1_araddr), .m1_arvalid(m1_arvalid), .m1_arready(m1_arready), .m1_rdata(m1_rdata), .m1_rresp(m1_rresp), .m1_rvalid(m1_rvalid), .m1_rready(m1_rready),
        .m2_awaddr(m2_awaddr), .m2_awvalid(m2_awvalid), .m2_awready(m2_awready), .m2_wdata(m2_wdata), .m2_wstrb(m2_wstrb), .m2_wvalid(m2_wvalid), .m2_wready(m2_wready), .m2_bresp(m2_bresp), .m2_bvalid(m2_bvalid), .m2_bready(m2_bready), .m2_araddr(m2_araddr), .m2_arvalid(m2_arvalid), .m2_arready(m2_arready), .m2_rdata(m2_rdata), .m2_rresp(m2_rresp), .m2_rvalid(m2_rvalid), .m2_rready(m2_rready),
        .s0_araddr(s0_araddr), .s0_arvalid(s0_arvalid), .s0_arready(s0_arready), .s0_rdata(s0_rdata), .s0_rresp(s0_rresp), .s0_rvalid(s0_rvalid), .s0_rready(s0_rready),
        .s1_awaddr(s1_awaddr), .s1_awvalid(s1_awvalid), .s1_awready(s1_awready), .s1_wdata(s1_wdata), .s1_wstrb(s1_wstrb), .s1_wvalid(s1_wvalid), .s1_wready(s1_wready), .s1_bresp(s1_bresp), .s1_bvalid(s1_bvalid), .s1_bready(s1_bready), .s1_araddr(s1_araddr), .s1_arlen(s1_arlen), .s1_arsize(s1_arsize), .s1_arburst(s1_arburst), .s1_arvalid(s1_arvalid), .s1_arready(s1_arready), .s1_rdata(s1_rdata), .s1_rresp(s1_rresp), .s1_rlast(s1_rlast), .s1_rvalid(s1_rvalid), .s1_rready(s1_rready),
        .s2_awaddr(s2_awaddr), .s2_awvalid(s2_awvalid), .s2_awready(s2_awready), .s2_wdata(s2_wdata), .s2_wstrb(s2_wstrb), .s2_wvalid(s2_wvalid), .s2_wready(s2_wready), .s2_bresp(s2_bresp), .s2_bvalid(s2_bvalid), .s2_bready(s2_bready), .s2_araddr(s2_araddr), .s2_arvalid(s2_arvalid), .s2_arready(s2_arready), .s2_rdata(s2_rdata), .s2_rresp(s2_rresp), .s2_rvalid(s2_rvalid), .s2_rready(s2_rready),
        .s3_araddr(s3_araddr), .s3_arvalid(s3_arvalid), .s3_arready(s3_arready), .s3_rdata(s3_rdata), .s3_rresp(s3_rresp), .s3_rvalid(s3_rvalid), .s3_rready(s3_rready)
    );

    // Clock Gen
    initial begin clk = 0; forever #5 clk = ~clk; end

    // =========================================================================
    // AUTOMATED SCOREBOARD
    // =========================================================================
    integer test_passed = 0;
    integer test_failed = 0;
    
    task print_result;
        input [8*60:1] test_msg;
        input pass;
        begin
            if (pass) begin
                $display("  [PASS] %0s", test_msg);
                test_passed = test_passed + 1;
            end else begin
                $display("  [FAIL] %0s <--- ERROR DETECTED!", test_msg);
                test_failed = test_failed + 1;
            end
        end
    endtask

    // =========================================================================
    // SMART SLAVE MODELS (WITH AGGRESSIVE BACKPRESSURE)
    // =========================================================================
    reg random_ready;
    always @(posedge clk) random_ready <= ($random % 100) > 30;

    // S0, S3 (Static Slaves)
    reg s0_rv_reg, s3_rv_reg;
    assign s0_arready = random_ready; assign s0_rdata = 32'hAAAA_0000; assign s0_rresp = 2'b00; assign s0_rvalid = s0_rv_reg;
    assign s3_arready = random_ready; assign s3_rdata = 32'hCCCC_3333; assign s3_rresp = 2'b00; assign s3_rvalid = s3_rv_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s0_rv_reg <= 0; s3_rv_reg <= 0; end
        else begin
            if (s0_arvalid && s0_arready) s0_rv_reg <= 1; else if (s0_rready && s0_rv_reg) s0_rv_reg <= 0;
            if (s3_arvalid && s3_arready) s3_rv_reg <= 1; else if (s3_rready && s3_rv_reg) s3_rv_reg <= 0;
        end
    end

    // S1 (RAM - Upgraded to 64KB for Stress Tests)
    reg [31:0] ram_mem [0:16383]; integer r_idx;
    initial begin for(r_idx=0; r_idx<16384; r_idx=r_idx+1) ram_mem[r_idx] = 32'h0; end

    assign s1_awready = random_ready; assign s1_wready = random_ready; assign s1_bresp = 2'b00;
    reg s1_bv_reg; assign s1_bvalid = s1_bv_reg;
    reg [31:0] s1_w_addr; reg s1_aw_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s1_bv_reg <= 0; s1_aw_done <= 0; end
        else begin
            if (s1_bv_reg && s1_bready) s1_bv_reg <= 0;
            if (s1_awvalid && s1_awready) begin s1_w_addr <= s1_awaddr; s1_aw_done <= 1; end
            if (s1_wvalid && s1_wready) begin
                if (s1_awvalid && s1_awready) begin ram_mem[(s1_awaddr & 32'hFFFF) >> 2] <= s1_wdata; s1_bv_reg <= 1; end
                else if (s1_aw_done) begin ram_mem[(s1_w_addr & 32'hFFFF) >> 2] <= s1_wdata; s1_aw_done <= 0; s1_bv_reg <= 1; end
            end
        end
    end

    reg [7:0] s1_b_cnt; reg s1_rd; reg [31:0] s1_ptr;
    assign s1_arready = !s1_rd && random_ready;
    assign s1_rvalid  = s1_rd && random_ready; 
    assign s1_rdata   = ram_mem[(s1_ptr & 32'hFFFF) >> 2];
    assign s1_rresp   = 2'b00;
    assign s1_rlast   = s1_rd && (s1_b_cnt == 0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) s1_rd <= 0;
        else begin
            if (s1_arvalid && s1_arready) begin s1_rd <= 1; s1_b_cnt <= s1_arlen; s1_ptr <= s1_araddr; end
            else if (s1_rvalid && s1_rready) begin
                if (s1_rlast) s1_rd <= 0;
                else begin s1_b_cnt <= s1_b_cnt - 1; s1_ptr <= s1_ptr + 4; end
            end
        end
    end

    // S2 (Peripheral - 64B Buffer)
    reg [31:0] p_mem [0:15]; initial begin for(r_idx=0; r_idx<16; r_idx=r_idx+1) p_mem[r_idx] = 32'hB0B0B0B0; end
    assign s2_awready = random_ready; assign s2_wready = random_ready; assign s2_bresp = 2'b00;
    reg s2_bv_reg, s2_rv_reg; assign s2_bvalid = s2_bv_reg; assign s2_rvalid = s2_rv_reg;
    reg [31:0] s2_w_addr, s2_r_dat; reg s2_aw_done;
    assign s2_arready = random_ready; assign s2_rdata = s2_r_dat; assign s2_rresp = 2'b00;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s2_bv_reg <= 0; s2_rv_reg <= 0; s2_aw_done <= 0; end
        else begin
            if (s2_bv_reg && s2_bready) s2_bv_reg <= 0;
            if (s2_rv_reg && s2_rready) s2_rv_reg <= 0;
            if (s2_awvalid && s2_awready) begin s2_w_addr <= s2_awaddr; s2_aw_done <= 1; end
            if (s2_wvalid && s2_wready) begin
                if (s2_awvalid && s2_awready) begin p_mem[(s2_awaddr & 32'h3F) >> 2] <= s2_wdata; s2_bv_reg <= 1; end
                else if (s2_aw_done) begin p_mem[(s2_w_addr & 32'h3F) >> 2] <= s2_wdata; s2_aw_done <= 0; s2_bv_reg <= 1; end
            end
            if (s2_arvalid && s2_arready) begin s2_rv_reg <= 1; s2_r_dat <= p_mem[(s2_araddr & 32'h3F) >> 2]; end
        end
    end

    // =========================================================================
    // MASTER TASKS
    // =========================================================================
    task m1_write(input [31:0] addr, input [31:0] data, input [1:0] exp_r);
        begin : blk_m1_w
            @(posedge clk); m1_awaddr <= addr; m1_awvalid <= 1; m1_wdata <= data; m1_wstrb <= 4'b1111; m1_wvalid <= 1; m1_bready <= 1;
            fork
                begin : w_aw forever begin @(posedge clk); if(m1_awready && m1_awvalid) begin m1_awvalid <= 0; disable w_aw; end end end
                begin : w_w  forever begin @(posedge clk); if(m1_wready && m1_wvalid)   begin m1_wvalid <= 0;  disable w_w;  end end end
            join
            begin : w_b forever begin @(posedge clk); if(m1_bvalid && m1_bready) begin m1_bready <= 0; print_result("M1 Write Transaction", (m1_bresp == exp_r)); disable w_b; end end end
        end
    endtask

    task m1_read_check(input [31:0] addr, input [31:0] exp_d, input [1:0] exp_r);
        begin : blk_m1_r
            @(posedge clk); m1_araddr <= addr; m1_arvalid <= 1; m1_rready <= 1;
            begin : w_ar forever begin @(posedge clk); if(m1_arready && m1_arvalid) begin m1_arvalid <= 0; disable w_ar; end end end
            begin : w_r  forever begin @(posedge clk); if(m1_rvalid && m1_rready)   begin print_result("M1 Read Check", (m1_rdata == exp_d && m1_rresp == exp_r)); m1_rready <= 0; disable w_r; end end end
        end
    endtask

    task m2_write(input [31:0] addr, input [31:0] data, input [1:0] exp_r);
        begin : blk_m2_w
            @(posedge clk); m2_awaddr <= addr; m2_awvalid <= 1; m2_wdata <= data; m2_wstrb <= 4'b1111; m2_wvalid <= 1; m2_bready <= 1;
            fork
                begin : w_aw2 forever begin @(posedge clk); if(m2_awready && m2_awvalid) begin m2_awvalid <= 0; disable w_aw2; end end end
                begin : w_w2  forever begin @(posedge clk); if(m2_wready && m2_wvalid)   begin m2_wvalid <= 0;  disable w_w2;  end end end
            join
            begin : w_b2 forever begin @(posedge clk); if(m2_bvalid && m2_bready) begin m2_bready <= 0; print_result("M2 Write Transaction", (m2_bresp == exp_r)); disable w_b2; end end end
        end
    endtask

    task m2_read_check(input [31:0] addr, input [31:0] exp_d, input [1:0] exp_r);
        begin : blk_m2_r
            @(posedge clk); m2_araddr <= addr; m2_arvalid <= 1; m2_rready <= 1;
            begin : w_ar2 forever begin @(posedge clk); if(m2_arready && m2_arvalid) begin m2_arvalid <= 0; disable w_ar2; end end end
            begin : w_r2  forever begin @(posedge clk); if(m2_rvalid && m2_rready)   begin print_result("M2 Read Check", (m2_rdata == exp_d && m2_rresp == exp_r)); m2_rready <= 0; disable w_r2; end end end
        end
    endtask

    task m0_burst_check(input [31:0] addr, input [7:0] len, input [31:0] expected_first_data);
        reg pass_flag; reg first_beat_checked;
        begin : blk_m0_r
            pass_flag = 1; first_beat_checked = 0;
            @(posedge clk); m0_araddr <= addr; m0_arlen <= len; m0_arsize <= 3'd2; m0_arburst <= 2'd1; m0_arvalid <= 1; m0_rready <= 1;
            begin : w_ar0 forever begin @(posedge clk); if(m0_arready && m0_arvalid) begin m0_arvalid <= 0; disable w_ar0; end end end
            begin : w_r0 
                forever begin 
                    @(posedge clk); 
                    if(m0_rvalid && m0_rready) begin 
                        if(!first_beat_checked) begin
                            if(m0_rdata !== expected_first_data) pass_flag = 0;
                            first_beat_checked = 1;
                        end
                        if(m0_rlast) begin print_result("M0 Burst Read Completion", pass_flag); m0_rready <= 0; disable w_r0; end 
                    end 
                end 
            end
        end
    endtask

    // =========================================================================
    // 16-PHASE GRANDMASTER TORTURE SUITE
    // =========================================================================
    integer loop_var;

    initial begin
        m0_araddr=0; m0_arlen=0; m0_arsize=0; m0_arburst=0; m0_arvalid=0; m0_rready=0;
        m1_awaddr=0; m1_awvalid=0; m1_wdata=0; m1_wstrb=0; m1_wvalid=0; m1_bready=0; m1_araddr=0; m1_arvalid=0; m1_rready=0;
        m2_awaddr=0; m2_awvalid=0; m2_wdata=0; m2_wstrb=0; m2_wvalid=0; m2_bready=0; m2_araddr=0; m2_arvalid=0; m2_rready=0;
        rst_n = 0; #100 rst_n = 1; #20;

        $display("\n=========================================================");
        $display(" PHASE 1: BASIC RAM ACCESS");
        m1_write(32'h8000_0000, 32'hA1B2C3D4, 2'b00);
        m1_read_check(32'h8000_0000, 32'hA1B2C3D4, 2'b00);

        $display("\n=========================================================");
        $display(" PHASE 2: PERIPHERAL ACCESS & BOUNDARY");
        m2_write(32'h4000_003C, 32'hDEADBEEF, 2'b00);
        m2_read_check(32'h4000_003C, 32'hDEADBEEF, 2'b00);

        $display("\n=========================================================");
        $display(" PHASE 3: WRITE PROTECTION (ROM)");
        m1_write(32'h0000_2000, 32'h12345678, 2'b11);

        $display("\n=========================================================");
        $display(" PHASE 4: UNMAPPED ADDRESS");
        m2_read_check(32'h9000_0000, 32'h0000_0000, 2'b11);

        $display("\n=========================================================");
        $display(" PHASE 5: WRITE ARBITRATION CONFLICT");
        fork
            m1_write(32'h8000_1000, 32'h1111_1111, 2'b00);
            m2_write(32'h8000_1004, 32'h2222_2222, 2'b00);
        join
        m1_read_check(32'h8000_1000, 32'h1111_1111, 2'b00);
        m2_read_check(32'h8000_1004, 32'h2222_2222, 2'b00);

        $display("\n=========================================================");
        $display(" PHASE 6: READ/WRITE CONFLICT (SAME SLAVE)");
        m1_write(32'h8000_1008, 32'h3333_3333, 2'b00);
        fork
            m1_write(32'h8000_100C, 32'h4444_4444, 2'b00);
            m2_read_check(32'h8000_1008, 32'h3333_3333, 2'b00);
        join

        $display("\n=========================================================");
        $display(" PHASE 7: ROUTING INDEPENDENCE");
        fork
            m1_write(32'h8000_1010, 32'h5555_5555, 2'b00);
            m2_write(32'h4000_0010, 32'h6666_6666, 2'b00);
        join

        $display("\n=========================================================");
        $display(" PHASE 8: I-CACHE (M0) BURST READ");
        m1_write(32'h8000_2000, 32'hB00B_B00B, 2'b00);
        m0_burst_check(32'h8000_2000, 8'd3, 32'hB00B_B00B);

        $display("\n=========================================================");
        $display(" PHASE 9: FULL SYSTEM CONCURRENCY");
        fork
            m0_burst_check(32'h8000_2000, 8'd7, 32'hB00B_B00B);
            m1_read_check(32'h4000_0010, 32'h6666_6666, 2'b00);
            m2_write(32'h8000_2010, 32'h7777_7777, 2'b00);
        join

        $display("\n=========================================================");
        $display(" PHASE 10: BACK-TO-BACK SEQUENTIAL");
        for(loop_var=0; loop_var<5; loop_var=loop_var+1) begin
            m1_write(32'h8000_3000 + (loop_var*4), loop_var, 2'b00);
        end
        for(loop_var=0; loop_var<5; loop_var=loop_var+1) begin
            m2_read_check(32'h8000_3000 + (loop_var*4), loop_var, 2'b00);
        end

        $display("\n=========================================================");
        $display(" PHASE 11: LONG ENDURANCE TORTURE");
        for(loop_var=0; loop_var<25; loop_var=loop_var+1) begin
            m2_write(32'h8000_4000 + (loop_var*4), ~loop_var, 2'b00);
            m1_read_check(32'h8000_4000 + (loop_var*4), ~loop_var, 2'b00);
        end

        $display("\n=========================================================");
        $display(" PHASE 12: MAX LENGTH BURST STRESS (256 BEATS)");
        m1_write(32'h8000_5000, 32'h5555_AAAA, 2'b00); 
        m0_burst_check(32'h8000_5000, 8'd255, 32'h5555_AAAA);

        $display("\n=========================================================");
        $display(" PHASE 13: CROSS-MATRIX ROUTING");
        fork
            m1_write(32'h8000_6000, 32'h11223344, 2'b00);
            m2_write(32'h4000_0020, 32'h55667788, 2'b00);
        join
        fork
            m1_read_check(32'h4000_0020, 32'h55667788, 2'b00);
            m2_read_check(32'h8000_6000, 32'h11223344, 2'b00);
        join

        $display("\n=========================================================");
        $display(" PHASE 14: SPI FLASH POLLING");
        fork
            m1_read_check(32'h2000_0000, 32'hCCCC_3333, 2'b00);
            m2_read_check(32'h2000_0004, 32'hCCCC_3333, 2'b00);
        join

        $display("\n=========================================================");
        $display(" PHASE 15: PRODUCER-CONSUMER");
        for(loop_var=0; loop_var<5; loop_var=loop_var+1) begin
            m1_write(32'h8000_7000 + (loop_var*4), 32'hA0A0A0A0 + loop_var, 2'b00);
            m2_read_check(32'h8000_7000 + (loop_var*4), 32'hA0A0A0A0 + loop_var, 2'b00);
        end

        $display("\n=========================================================");
        $display(" PHASE 16: THE FINAL APOCALYPSE");
        fork
            m0_burst_check(32'h8000_0000, 8'd15, 32'hA1B2C3D4);
            m1_write(32'h4000_0038, 32'hCAFEBABE, 2'b00);
            m2_write(32'h8000_8000, 32'hBEEFBEEF, 2'b00);
            m1_read_check(32'h0000_1000, 32'hAAAA_0000, 2'b00);
            m2_read_check(32'h2000_0100, 32'hCCCC_3333, 2'b00);
        join

        #200;
        $display("\n=========================================================");
        $display(" FINAL VERIFICATION SCOREBOARD");
        $display("=========================================================");
        $display(" Total Tests Conducted : %0d", test_passed + test_failed);
        $display(" PASSED (NO HANGS)     : %0d", test_passed);
        $display(" FAILED (DEADLOCKS)    : %0d", test_failed);
        
        if (test_failed == 0) 
            $display("\n >>> SUCCESS! 16-PHASE PASSED! <<<\n");
        else 
            $display("\n >>> CRITICAL FAILURE DETECTED! <<<\n");
        
        $finish;
    end
endmodule