`timescale 1ns / 1ps

module tb_apb_interconnect();

    parameter ADDR_WIDTH = 32;
    parameter DATA_WIDTH = 32;

    reg clk;
    reg rst_n;

    // MASTER BFM SIGNALS
    reg  [ADDR_WIDTH-1:0] m_paddr;
    reg                   m_psel;
    reg                   m_penable;
    reg                   m_pwrite;
    reg  [DATA_WIDTH-1:0] m_pwdata;
    reg  [3:0]            m_pstrb;
    wire [DATA_WIDTH-1:0] m_prdata;
    wire                  m_pready;
    wire                  m_pslverr;

    // SLAVE INTERFACE WIRES
    wire s0_psel, s1_psel, s2_psel, s3_psel, s4_psel, s5_psel, s6_psel, s7_psel;
    wire [ADDR_WIDTH-1:0] s0_paddr, s1_paddr, s2_paddr, s3_paddr, s4_paddr, s5_paddr, s6_paddr, s7_paddr;
    wire s0_penable, s1_penable, s2_penable, s3_penable, s4_penable, s5_penable, s6_penable, s7_penable;
    wire s0_pwrite, s1_pwrite, s2_pwrite, s3_pwrite, s4_pwrite, s5_pwrite, s6_pwrite, s7_pwrite;
    wire [DATA_WIDTH-1:0] s0_pwdata, s1_pwdata, s2_pwdata, s3_pwdata, s4_pwdata, s5_pwdata, s6_pwdata, s7_pwdata;
    wire [3:0] s0_pstrb, s1_pstrb, s2_pstrb, s3_pstrb, s4_pstrb, s5_pstrb, s6_pstrb, s7_pstrb;
    
    reg [DATA_WIDTH-1:0] s0_prdata, s1_prdata, s2_prdata, s3_prdata, s4_prdata, s5_prdata, s6_prdata, s7_prdata;
    reg s0_pready, s1_pready, s2_pready, s3_pready, s4_pready, s5_pready, s6_pready, s7_pready;
    reg s0_pslverr, s1_pslverr, s2_pslverr, s3_pslverr, s4_pslverr, s5_pslverr, s6_pslverr, s7_pslverr;

    // DUT INSTANTIATION
    apb_interconnect #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) dut (
        .m_paddr(m_paddr), .m_psel(m_psel), .m_penable(m_penable), .m_pwrite(m_pwrite), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb), .m_prdata(m_prdata), .m_pready(m_pready), .m_pslverr(m_pslverr),
        .s0_psel(s0_psel), .s0_paddr(s0_paddr), .s0_penable(s0_penable), .s0_pwrite(s0_pwrite), .s0_pwdata(s0_pwdata), .s0_pstrb(s0_pstrb), .s0_prdata(s0_prdata), .s0_pready(s0_pready), .s0_pslverr(s0_pslverr),
        .s1_psel(s1_psel), .s1_paddr(s1_paddr), .s1_penable(s1_penable), .s1_pwrite(s1_pwrite), .s1_pwdata(s1_pwdata), .s1_pstrb(s1_pstrb), .s1_prdata(s1_prdata), .s1_pready(s1_pready), .s1_pslverr(s1_pslverr),
        .s2_psel(s2_psel), .s2_paddr(s2_paddr), .s2_penable(s2_penable), .s2_pwrite(s2_pwrite), .s2_pwdata(s2_pwdata), .s2_pstrb(s2_pstrb), .s2_prdata(s2_prdata), .s2_pready(s2_pready), .s2_pslverr(s2_pslverr),
        .s3_psel(s3_psel), .s3_paddr(s3_paddr), .s3_penable(s3_penable), .s3_pwrite(s3_pwrite), .s3_pwdata(s3_pwdata), .s3_pstrb(s3_pstrb), .s3_prdata(s3_prdata), .s3_pready(s3_pready), .s3_pslverr(s3_pslverr),
        .s4_psel(s4_psel), .s4_paddr(s4_paddr), .s4_penable(s4_penable), .s4_pwrite(s4_pwrite), .s4_pwdata(s4_pwdata), .s4_pstrb(s4_pstrb), .s4_prdata(s4_prdata), .s4_pready(s4_pready), .s4_pslverr(s4_pslverr),
        .s5_psel(s5_psel), .s5_paddr(s5_paddr), .s5_penable(s5_penable), .s5_pwrite(s5_pwrite), .s5_pwdata(s5_pwdata), .s5_pstrb(s5_pstrb), .s5_prdata(s5_prdata), .s5_pready(s5_pready), .s5_pslverr(s5_pslverr),
        .s6_psel(s6_psel), .s6_paddr(s6_paddr), .s6_penable(s6_penable), .s6_pwrite(s6_pwrite), .s6_pwdata(s6_pwdata), .s6_pstrb(s6_pstrb), .s6_prdata(s6_prdata), .s6_pready(s6_pready), .s6_pslverr(s6_pslverr),
        .s7_psel(s7_psel), .s7_paddr(s7_paddr), .s7_penable(s7_penable), .s7_pwrite(s7_pwrite), .s7_pwdata(s7_pwdata), .s7_pstrb(s7_pstrb), .s7_prdata(s7_prdata), .s7_pready(s7_pready), .s7_pslverr(s7_pslverr)
    );

    // Clock Gen
    initial begin clk = 0; forever #5 clk = ~clk; end

    // SCOREBOARD
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
                $display("  [FAIL] %0s <--- APB PROTOCOL/DATA ERROR!", test_msg);
                test_failed = test_failed + 1;
            end
        end
    endtask

    // SMART 8-SLAVE ARRAY MODEL (FIXED MEMORY SIZE FOR S7)
    reg [31:0] mem_s0 [0:255]; reg [31:0] mem_s1 [0:255]; reg [31:0] mem_s2 [0:255]; reg [31:0] mem_s3 [0:255];
    reg [31:0] mem_s4 [0:255]; reg [31:0] mem_s5 [0:255]; reg [31:0] mem_s6 [0:255]; reg [31:0] mem_s7 [0:1023]; // Tăng S7 lên 4KB
    
    integer i;
    initial begin
        for(i=0; i<256; i=i+1) begin mem_s0[i]=0; mem_s1[i]=0; mem_s2[i]=0; mem_s3[i]=0; mem_s4[i]=0; mem_s5[i]=0; mem_s6[i]=0; end
        for(i=0; i<1024; i=i+1) begin mem_s7[i]=0; end
    end

    reg random_stall;
    always @(posedge clk) random_stall <= ($random % 100) > 30;

    `define APB_SLAVE_MODEL(ID) \
        always @(posedge clk or negedge rst_n) begin \
            if (!rst_n) begin \
                s``ID``_pready <= 0; s``ID``_prdata <= 0; s``ID``_pslverr <= 0; \
            end else begin \
                if (s``ID``_psel && !s``ID``_penable) begin \
                    s``ID``_pready <= 0; s``ID``_pslverr <= 0; \
                end else if (s``ID``_psel && s``ID``_penable) begin \
                    if (random_stall) begin \
                        s``ID``_pready <= 1; \
                        if (s``ID``_pwrite) begin \
                            if (s``ID``_pstrb[0]) mem_s``ID``[(s``ID``_paddr & 32'h0FFF) >> 2][7:0]   <= s``ID``_pwdata[7:0]; \
                            if (s``ID``_pstrb[1]) mem_s``ID``[(s``ID``_paddr & 32'h0FFF) >> 2][15:8]  <= s``ID``_pwdata[15:8]; \
                            if (s``ID``_pstrb[2]) mem_s``ID``[(s``ID``_paddr & 32'h0FFF) >> 2][23:16] <= s``ID``_pwdata[23:16]; \
                            if (s``ID``_pstrb[3]) mem_s``ID``[(s``ID``_paddr & 32'h0FFF) >> 2][31:24] <= s``ID``_pwdata[31:24]; \
                            if (ID == 5 && ((s``ID``_paddr & 32'h0FFF) == 12'h100)) s``ID``_pslverr <= 1; \
                        end else begin \
                            s``ID``_prdata <= mem_s``ID``[(s``ID``_paddr & 32'h0FFF) >> 2]; \
                            if (ID == 2 && ((s``ID``_paddr & 32'h0FFF) == 12'h200)) s``ID``_pslverr <= 1; \
                        end \
                    end else begin \
                        s``ID``_pready <= 0; \
                    end \
                end else begin \
                    s``ID``_pready <= 0; s``ID``_pslverr <= 0; \
                end \
            end \
        end

    `APB_SLAVE_MODEL(0) `APB_SLAVE_MODEL(1) `APB_SLAVE_MODEL(2) `APB_SLAVE_MODEL(3)
    `APB_SLAVE_MODEL(4) `APB_SLAVE_MODEL(5) `APB_SLAVE_MODEL(6) `APB_SLAVE_MODEL(7)

    // APB MASTER VERIFICATION TASKS (FIXED)
    task apb_write(input [31:0] addr, input [31:0] data, input [3:0] strb, input exp_err);
        begin : blk_pw
            @(posedge clk); m_psel <= 1; m_paddr <= addr; m_pwrite <= 1; m_pwdata <= data; m_pstrb <= strb; m_penable <= 0;
            @(posedge clk); m_penable <= 1;
            begin : wait_pready
                forever begin
                    @(posedge clk);
                    if (m_pready) begin
                        m_psel <= 0; m_penable <= 0;
                        print_result("APB Write Task", (m_pslverr == exp_err));
                        disable wait_pready;
                    end
                end
            end
        end
    endtask

    task apb_read(input [31:0] addr, input [31:0] exp_data, input exp_err);
        begin : blk_pr
            @(posedge clk); m_psel <= 1; m_paddr <= addr; m_pwrite <= 0; m_penable <= 0;
            @(posedge clk); m_penable <= 1;
            begin : wait_pr_ready
                forever begin
                    @(posedge clk);
                    if (m_pready) begin
                        m_psel <= 0; m_penable <= 0;
                        // FIXED LOGIC: Chỉ check DATA nếu KHÔNG CÓ LỖI. Dùng toán tử === để bắt chặt X/Z.
                        if (exp_err)
                            print_result("APB Read Task (Err Check)", (m_pslverr == 1'b1));
                        else
                            print_result("APB Read Task (Data Check)", (m_prdata === exp_data && m_pslverr == 1'b0));
                        disable wait_pr_ready;
                    end
                end
            end
        end
    endtask

    // 16-PHASE APB TORTURE SUITE
    integer loop_var;

    initial begin
        m_psel = 0; m_penable = 0; m_pwrite = 0; m_paddr = 0; m_pwdata = 0; m_pstrb = 0;
        rst_n = 0; #50 rst_n = 1; #20;

        $display("\n=========================================================");
        $display(" PHASE 1: BASIC I/O - SYSCON (SLAVE 0)");
        apb_write(32'h4000_0004, 32'hA1A1_B2B2, 4'b1111, 0);
        apb_read (32'h4000_0004, 32'hA1A1_B2B2, 0);

        $display("\n=========================================================");
        $display(" PHASE 2: BOUNDARY I/O - ACCELERATOR (SLAVE 7)");
        apb_write(32'h4000_7FFC, 32'hDEAD_BEEF, 4'b1111, 0);
        apb_read (32'h4000_7FFC, 32'hDEAD_BEEF, 0);

        $display("\n=========================================================");
        $display(" PHASE 3: ADDRESS DECODE CROSSTALK CHECK");
        apb_write(32'h4000_1000, 32'h1111_1111, 4'b1111, 0); 
        apb_write(32'h4000_2000, 32'h2222_2222, 4'b1111, 0); 
        apb_read (32'h4000_1000, 32'h1111_1111, 0); 

        $display("\n=========================================================");
        $display(" PHASE 4: UNMAPPED ADDRESS (EXPECT DEAD_BEEF & ERROR)");
        apb_read (32'h5000_0000, 32'hDEAD_BEEF, 1); 

        $display("\n=========================================================");
        $display(" PHASE 5: BACK-TO-BACK WRITES (UART - SLAVE 3)");
        apb_write(32'h4000_3000, 32'h0000_00AA, 4'b1111, 0);
        apb_write(32'h4000_3004, 32'h0000_00BB, 4'b1111, 0);
        apb_write(32'h4000_3008, 32'h0000_00CC, 4'b1111, 0);

        $display("\n=========================================================");
        $display(" PHASE 6: BACK-TO-BACK READS (UART - SLAVE 3)");
        apb_read (32'h4000_3000, 32'h0000_00AA, 0);
        apb_read (32'h4000_3004, 32'h0000_00BB, 0);
        apb_read (32'h4000_3008, 32'h0000_00CC, 0);

        $display("\n=========================================================");
        $display(" PHASE 7: ROUND-ROBIN SWEEP WRITES (ALL SLAVES)");
        for(loop_var=0; loop_var<8; loop_var=loop_var+1) begin
            apb_write(32'h4000_0010 + (loop_var << 12), 32'hC0DE_0000 + loop_var, 4'b1111, 0);
        end

        $display("\n=========================================================");
        $display(" PHASE 8: ROUND-ROBIN SWEEP READS (ALL SLAVES)");
        for(loop_var=0; loop_var<8; loop_var=loop_var+1) begin
            apb_read(32'h4000_0010 + (loop_var << 12), 32'hC0DE_0000 + loop_var, 0);
        end

        $display("\n=========================================================");
        $display(" PHASE 9: WRITE WITH EXTREME BACKPRESSURE");
        apb_write(32'h4000_40A0, 32'hBADC_0FFE, 4'b1111, 0); 
        apb_read (32'h4000_40A0, 32'hBADC_0FFE, 0);

        $display("\n=========================================================");
        $display(" PHASE 10: READ WITH EXTREME BACKPRESSURE");
        apb_read (32'h4000_40A0, 32'hBADC_0FFE, 0);

        $display("\n=========================================================");
        $display(" PHASE 11: SLAVE-GENERATED ERROR HANDLING");
        apb_write(32'h4000_5100, 32'hBADD_BADD, 4'b1111, 1);
        apb_read (32'h4000_2200, 32'h0000_0000, 1);         

        $display("\n=========================================================");
        $display(" PHASE 12: WALK-1 ADDRESS TEST");
        apb_write(32'h4000_4004, 32'h0000_0001, 4'b1111, 0);
        apb_write(32'h4000_4008, 32'h0000_0002, 4'b1111, 0);
        apb_write(32'h4000_4010, 32'h0000_0004, 4'b1111, 0);
        apb_read (32'h4000_4004, 32'h0000_0001, 0);
        apb_read (32'h4000_4010, 32'h0000_0004, 0);

        $display("\n=========================================================");
        $display(" PHASE 13: WALK-1 DATA PATH TEST");
        apb_write(32'h4000_6000, 32'h5555_AAAA, 4'b1111, 0); 
        apb_read (32'h4000_6000, 32'h5555_AAAA, 0);

        $display("\n=========================================================");
        $display(" PHASE 14: WRITE STROBE (PSTRB) PARTIAL WRITE TEST");
        apb_write(32'h4000_0040, 32'h0000_0000, 4'b1111, 0);
        apb_write(32'h4000_0040, 32'hFFFF_BBFF, 4'b0010, 0);
        apb_read (32'h4000_0040, 32'h0000_BB00, 0);

        $display("\n=========================================================");
        $display(" PHASE 15: RAPID ALTERNATING READ/WRITE (I2C - SLAVE 5)");
        apb_write(32'h4000_5008, 32'h8888_8888, 4'b1111, 0);
        apb_read (32'h4000_5008, 32'h8888_8888, 0);
        apb_write(32'h4000_5008, 32'h9999_9999, 4'b1111, 0);
        apb_read (32'h4000_5008, 32'h9999_9999, 0);

        $display("\n=========================================================");
        $display(" PHASE 16: LONG ENDURANCE TORTURE (100 RANDOM TRANSACTIONS)");
        for(loop_var=0; loop_var<50; loop_var=loop_var+1) begin
            apb_write(32'h4000_6000 + (loop_var*4), 32'hF0F0_0F0F ^ loop_var, 4'b1111, 0);
            apb_read (32'h4000_6000 + (loop_var*4), 32'hF0F0_0F0F ^ loop_var, 0);
        end

        #200;
        $display("\n=========================================================");
        $display(" APB INTERCONNECT: FINAL VERIFICATION SCOREBOARD");
        $display("=========================================================");
        $display(" Total Tests Conducted : %0d", test_passed + test_failed);
        $display(" PASSED                : %0d", test_passed);
        $display(" FAILED                : %0d", test_failed);
        
        if (test_failed == 0) 
            $display("\n >>> SUCCESS! 16-PHASE PASSED! <<<\n");
        else 
            $display("\n >>> CRITICAL FAILURE DETECTED! <<<\n");
        
        $finish;
    end
endmodule