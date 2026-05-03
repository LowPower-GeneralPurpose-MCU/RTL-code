`timescale 1ns / 1ps

module tb_axi_spi_flash;

    // Parameters
    parameter ADDR_WIDTH = 32;
    parameter DATA_WIDTH = 32;
    
    // Clock and Reset
    reg clk;
    reg rst_n;

    // AXI Write Channel
    reg  [ADDR_WIDTH-1:0] s_axi_awaddr;
    reg                   s_axi_awvalid;
    wire                  s_axi_awready;
    reg  [DATA_WIDTH-1:0] s_axi_wdata;
    reg  [3:0]            s_axi_wstrb;
    reg                   s_axi_wvalid;
    wire                  s_axi_wready;
    wire [1:0]            s_axi_bresp;
    wire                  s_axi_bvalid;
    reg                   s_axi_bready;

    // AXI Read Channel
    reg  [ADDR_WIDTH-1:0] s_axi_araddr;
    reg                   s_axi_arvalid;
    wire                  s_axi_arready;
    wire [DATA_WIDTH-1:0] s_axi_rdata;
    wire [1:0]            s_axi_rresp;
    wire                  s_axi_rvalid;
    reg                   s_axi_rready;

    // SPI Interface
    wire spi_cs_n;
    wire spi_sck;
    wire spi_mosi;
    wire spi_miso;

    // Testbench Variables
    integer error_count;
    integer i;
    reg [31:0] read_data;
    reg [1:0]  read_resp;
    
    // System RAM (Simulating destination for bootloader)
    reg [31:0] sys_ram [0:15];
    reg [31:0] expected_word;

    // -----------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------
    axi_spi_flash #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .s_axi_awaddr(s_axi_awaddr),
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
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

    // -----------------------------------------------------------------
    // Clock Generation
    // -----------------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz Clock
    end

    // -----------------------------------------------------------------
    // External SPI Flash Memory Model (Behavioral)
    // -----------------------------------------------------------------
    reg [7:0]  flash_mem [0:255];
    reg [23:0] flash_addr;
    reg [7:0]  flash_cmd;
    reg [31:0] flash_data_out;
    integer    spi_bit_idx;
    reg        spi_miso_reg;

    // Initialize Flash Memory 
    initial begin
        flash_addr = 24'd0; // Prevent initial 'x' state propagation
        for (i = 0; i < 256; i = i + 1) begin
            flash_mem[i] = i; 
        end
    end

    // Fix Delta-Cycle Race: Delay MOSI slightly so the slave samples 
    // the correct bit before the DUT shifts it out on the same clock edge.
    wire spi_mosi_del;
    assign #1 spi_mosi_del = spi_mosi;

    // SPI Slave FSM: Capture MOSI on posedge SCK
    always @(posedge spi_sck or posedge spi_cs_n) begin
        if (spi_cs_n) begin
            spi_bit_idx <= 63;
        end else begin
            // Capture Command (bits 63:56)
            if (spi_bit_idx >= 56) begin
                flash_cmd[spi_bit_idx - 56] <= spi_mosi_del;
            end 
            // Capture Address (bits 55:32)
            else if (spi_bit_idx >= 32) begin
                flash_addr[spi_bit_idx - 32] <= spi_mosi_del;
                
                // Fetch data when address is complete
                // Fix: Manually concatenate the last bit (spi_mosi_del) since 
                // flash_addr[0] won't be updated until the end of this time step.
                if (spi_bit_idx == 32) begin
                    flash_data_out <= {
                        flash_mem[{flash_addr[23:1], spi_mosi_del}], 
                        flash_mem[{flash_addr[23:1], spi_mosi_del} + 1], 
                        flash_mem[{flash_addr[23:1], spi_mosi_del} + 2], 
                        flash_mem[{flash_addr[23:1], spi_mosi_del} + 3]
                    };
                end
            end
            
            if (spi_bit_idx > 0)
                spi_bit_idx <= spi_bit_idx - 1;
        end
    end

    // SPI Slave Output: Drive MISO on negedge SCK
    always @(negedge spi_sck or posedge spi_cs_n) begin
        if (spi_cs_n) begin
            spi_miso_reg <= 1'b0;
        end else begin
            if (spi_bit_idx < 32) begin
                spi_miso_reg <= flash_data_out[spi_bit_idx];
            end else begin
                spi_miso_reg <= 1'b0;
            end
        end
    end

    assign spi_miso = spi_miso_reg;

    // -----------------------------------------------------------------
    // AXI Bus Functional Model (BFM) Tasks
    // -----------------------------------------------------------------
    task axi_read;
        input  [31:0] addr;
        output [31:0] data_out;
        output [1:0]  resp_out;
        begin
            @(posedge clk);
            s_axi_araddr  <= addr;
            s_axi_arvalid <= 1'b1;
            
            // Wait for AR handshake
            @(posedge clk);
            while (s_axi_arready === 1'b0) @(posedge clk);
            s_axi_arvalid <= 1'b0;
            s_axi_rready  <= 1'b1;
            
            // Wait for R handshake
            while (s_axi_rvalid === 1'b0) @(posedge clk);
            data_out = s_axi_rdata;
            resp_out = s_axi_rresp;
            
            @(posedge clk);
            s_axi_rready <= 1'b0;
        end
    endtask

    task axi_write;
        input  [31:0] addr;
        input  [31:0] data;
        output [1:0]  resp_out;
        begin
            @(posedge clk);
            s_axi_awaddr  <= addr;
            s_axi_awvalid <= 1'b1;
            s_axi_wdata   <= data;
            s_axi_wvalid  <= 1'b1;
            s_axi_wstrb   <= 4'hF;
            s_axi_bready  <= 1'b1;
            
            // Wait for AW/W handshake
            @(posedge clk);
            while (s_axi_awready === 1'b0 || s_axi_wready === 1'b0) @(posedge clk);
            s_axi_awvalid <= 1'b0;
            s_axi_wvalid  <= 1'b0;
            
            // Wait for B phase response
            while (s_axi_bvalid === 1'b0) @(posedge clk);
            resp_out = s_axi_bresp;
            
            @(posedge clk);
            s_axi_bready <= 1'b0;
        end
    endtask

    // -----------------------------------------------------------------
    // Main Test Sequence (5 Phases)
    // -----------------------------------------------------------------
    initial begin
        // Init signals
        rst_n = 0;
        s_axi_awaddr = 0; s_axi_awvalid = 0; s_axi_wdata = 0;
        s_axi_wstrb = 0; s_axi_wvalid = 0; s_axi_bready = 0;
        s_axi_araddr = 0; s_axi_arvalid = 0; s_axi_rready = 0;
        error_count = 0;

        $display("=========================================================");
        $display("          AXI TO SPI FLASH BRIDGE TESTBENCH              ");
        $display("=========================================================");

        // -------------------------------------------------------------
        // PHASE 1: Reset & Initialization
        // -------------------------------------------------------------
        $display("\n[PHASE 1] Reset & Initialization Sequence...");
        #50;
        rst_n = 1;
        #50;
        if (spi_cs_n !== 1'b1 || spi_sck !== 1'b0) begin
            $display("  --> [FAIL] SPI Bus not in IDLE state after reset.");
            error_count = error_count + 1;
        end else begin
            $display("  --> [PASS] Reset successful. SPI Bus in IDLE state.");
        end

        // -------------------------------------------------------------
        // PHASE 2: Error Handling (Write Channel Rejection)
        // -------------------------------------------------------------
        $display("\n[PHASE 2] Testing AXI Write Rejection (SLVERR)...");
        axi_write(32'h0000_1000, 32'hDEADBEEF, read_resp);
        if (read_resp === 2'b10) begin
            $display("  --> [PASS] Write channel safely rejected with SLVERR.");
        end else begin
            $display("  --> [FAIL] Expected SLVERR (2'b10), got %b", read_resp);
            error_count = error_count + 1;
        end

        // -------------------------------------------------------------
        // PHASE 3: Single Word Fetch (Boot ROM read test)
        // -------------------------------------------------------------
        $display("\n[PHASE 3] Single AXI Read (Flash to System RAM)...");
        axi_read(32'h0000_0000, read_data, read_resp);
        sys_ram[0] = read_data; // Store to RAM
        
        // Expected data at 0: 0x00010203 (Based on initialization block)
        if (read_data === 32'h00010203 && read_resp === 2'b00) begin
            $display("  --> [PASS] Data Read Valid: 0x%08X", read_data);
        end else begin
            $display("  --> [FAIL] Data Read Mismatch. Expected 0x00010203, got 0x%08X", read_data);
            error_count = error_count + 1;
        end

        // -------------------------------------------------------------
        // PHASE 4: Bootloader Sequence (Sequential Block Copy to RAM)
        // -------------------------------------------------------------
        $display("\n[PHASE 4] Executing Bootloader Sequence (Loading 15 Words to RAM)...");
        for (i = 1; i < 16; i = i + 1) begin
            // Increment address by 4 bytes (Word aligned)
            axi_read(i * 4, read_data, read_resp);
            sys_ram[i] = read_data; 
        end
        $display("  --> [PASS] Bootloader loop completed. Transferred 64 bytes total.");

        // -------------------------------------------------------------
        // PHASE 5: Data Integrity Verification (Checksum/Array Compare)
        // -------------------------------------------------------------
        $display("\n[PHASE 5] Verifying System RAM against SPI Flash Source...");
        for (i = 0; i < 16; i = i + 1) begin
            // Reconstruct expected 32-bit word from byte-addressable flash
            expected_word = {flash_mem[i*4], flash_mem[i*4+1], flash_mem[i*4+2], flash_mem[i*4+3]};
            
            if (sys_ram[i] !== expected_word) begin
                $display("  --> [FAIL] Addr offset 0x%02X: RAM=0x%08X | Expected=0x%08X", 
                          i*4, sys_ram[i], expected_word);
                error_count = error_count + 1;
            end
        end

        if (error_count == 0)
            $display("  --> [PASS] System RAM integrity verification perfectly matched!");

        // -------------------------------------------------------------
        // FINAL VERDICT
        // -------------------------------------------------------------
        $display("\n=========================================================");
        if (error_count == 0) begin
            $display("   TEST RESULT: [ PASS ] - NO ERRORS DETECTED  ");
        end else begin
            $display("   TEST RESULT: [ FAIL ] - DETECTED %0d ERRORS          ", error_count);
        end
        $display("=========================================================\n");

        $finish;
    end

endmodule