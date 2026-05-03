module axi_to_apb_bridge #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- AXI4-Lite Slave Interface ---
    input  wire [ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  wire [2:0]             s_axi_awprot,
    input  wire                   s_axi_awvalid,
    output reg                    s_axi_awready,
    input  wire [DATA_WIDTH-1:0]  s_axi_wdata,
    input  wire [3:0]             s_axi_wstrb,
    input  wire                   s_axi_wvalid,
    output reg                    s_axi_wready,
    output reg  [1:0]             s_axi_bresp,
    output reg                    s_axi_bvalid,
    input  wire                   s_axi_bready,
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire [2:0]             s_axi_arprot,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    output reg  [DATA_WIDTH-1:0]  s_axi_rdata,
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready,

    // --- APB4 Master Interface ---
    output reg  [ADDR_WIDTH-1:0]  m_apb_paddr,
    output reg  [2:0]             m_apb_pprot,
    output reg                    m_apb_psel,
    output reg                    m_apb_penable,
    output reg                    m_apb_pwrite,
    output reg  [DATA_WIDTH-1:0]  m_apb_pwdata,
    output reg  [3:0]             m_apb_pstrb,
    input  wire                   m_apb_pready,
    input  wire [DATA_WIDTH-1:0]  m_apb_prdata,
    input  wire                   m_apb_pslverr
);

    localparam ST_IDLE   = 2'b00;
    localparam ST_SETUP  = 2'b01;
    localparam ST_ACCESS = 2'b10;

    reg [1:0] state;
    reg       is_write_txn;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            s_axi_awready <= 0; s_axi_wready <= 0; s_axi_bvalid <= 0;
            s_axi_arready <= 0; s_axi_rvalid <= 0;
            m_apb_psel <= 0; m_apb_penable <= 0;
        end else begin
            // Xóa Valid signals khi Master đã nhận
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 0;
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 0;

            case (state)
                ST_IDLE: begin
                    m_apb_penable <= 1'b0;
                    // Ưu tiên Write
                    if (s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                        state         <= ST_SETUP;
                        is_write_txn  <= 1'b1;
                        s_axi_awready <= 1'b1;
                        s_axi_wready  <= 1'b1;
                        
                        m_apb_psel    <= 1'b1;
                        m_apb_paddr   <= s_axi_awaddr;
                        m_apb_pwrite  <= 1'b1;
                        m_apb_pwdata  <= s_axi_wdata;
                        m_apb_pstrb   <= s_axi_wstrb;
                        m_apb_pprot   <= s_axi_awprot;
                    end 
                    else if (s_axi_arvalid && !s_axi_rvalid) begin
                        state         <= ST_SETUP;
                        is_write_txn  <= 1'b0;
                        s_axi_arready <= 1'b1;
                        
                        m_apb_psel    <= 1'b1;
                        m_apb_paddr   <= s_axi_araddr;
                        m_apb_pwrite  <= 1'b0;
                        m_apb_pprot   <= s_axi_arprot;
                    end
                end

                ST_SETUP: begin
                    s_axi_awready <= 1'b0;
                    s_axi_wready  <= 1'b0;
                    s_axi_arready <= 1'b0;
                    
                    m_apb_penable <= 1'b1; // Chuyển sang Access Phase
                    state         <= ST_ACCESS;
                end

                ST_ACCESS: begin
                    if (m_apb_pready) begin
                        m_apb_psel    <= 1'b0;
                        m_apb_penable <= 1'b0;
                        
                        if (is_write_txn) begin
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= m_apb_pslverr ? 2'b10 : 2'b00;
                        end else begin
                            s_axi_rvalid <= 1'b1;
                            s_axi_rdata  <= m_apb_prdata;
                            s_axi_rresp  <= m_apb_pslverr ? 2'b10 : 2'b00;
                        end
                        state <= ST_IDLE;
                    end
                end
            endcase
        end
    end
endmodule