`timescale 1ns / 1ps

module apb_gpio #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32
)(
    // --- Giao tiếp APB ---
    input  wire                  pclk,
    input  wire                  presetn,
    input  wire                  psel,
    input  wire                  penable,
    input  wire                  pwrite,
    input  wire [ADDR_WIDTH-1:0] paddr,
    input  wire [DATA_WIDTH-1:0] pwdata,
    output reg                   pready,
    output reg  [DATA_WIDTH-1:0] prdata,
    output reg                   pslverr,

    // --- Chân vật lý GPIO ---
    input  wire [31:0]           gpio_in,  // Tín hiệu vào từ Pad
    output reg  [31:0]           gpio_out, // Tín hiệu ra Pad
    output reg  [31:0]           gpio_dir, // Hướng: 1 là Output, 0 là Input
    output wire                  gpio_irq  // Tín hiệu ngắt gửi tới PLIC
);

    // --- Register Map ---
    reg [31:0] reg_int_en;
    reg [31:0] reg_int_type;
    reg [31:0] reg_int_polarity;
    reg [31:0] reg_int_bothedge;
    reg [31:0] reg_int_stat;

    // --- Mạch đồng bộ tín hiệu vào (Tránh Metastability) ---
    reg [31:0] sync_f1, sync_f2, sync_f3;
    
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            sync_f1 <= 32'd0;
            sync_f2 <= 32'd0;
            sync_f3 <= 32'd0;
        end else begin
            sync_f1 <= gpio_in;
            sync_f2 <= sync_f1; // Tín hiệu đã đồng bộ
            sync_f3 <= sync_f2; // Trì hoãn 1 chu kỳ để dò cạnh
        end
    end

    // --- Logic phát hiện sự kiện cơ bản ---
    wire [31:0] rising_edge  =  sync_f2 & ~sync_f3; // Đang là 1, trước đó là 0
    wire [31:0] falling_edge = ~sync_f2 &  sync_f3; // Đang là 0, trước đó là 1
    wire [31:0] high_level   =  sync_f2;
    wire [31:0] low_level    = ~sync_f2;

    // --- Mạch tổ hợp tạo trigger ngắt theo cấu hình ---
    wire [31:0] edge_irq;
    wire [31:0] level_irq;
    wire [31:0] irq_trigger;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_irq_logic
            // Xử lý loại Cạnh (Edge)
            assign edge_irq[i]  = reg_int_bothedge[i] ? (rising_edge[i] | falling_edge[i]) :
                                  reg_int_polarity[i] ? rising_edge[i] : falling_edge[i];
            
            // Xử lý loại Mức (Level)
            assign level_irq[i] = reg_int_polarity[i] ? high_level[i] : low_level[i];

            // Chọn giữa Edge hoặc Level
            assign irq_trigger[i] = reg_int_type[i] ? level_irq[i] : edge_irq[i];
        end
    endgenerate

    // --- Tín hiệu xuất ra PLIC ---
    // Chỉ báo ngắt khi bit đó có trạng thái ngắt = 1 VÀ được Enable = 1
    assign gpio_irq = |(reg_int_stat & reg_int_en);

    // --- Giao tiếp Đọc/Ghi APB ---
    wire apb_write = psel && penable && pwrite;
    wire apb_read  = psel && !penable && !pwrite;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            gpio_out         <= 32'd0;
            gpio_dir         <= 32'd0;
            reg_int_en       <= 32'd0;
            reg_int_type     <= 32'd0;
            reg_int_polarity <= 32'd0;
            reg_int_bothedge <= 32'd0;
            reg_int_stat     <= 32'd0;
            
            pready           <= 1'b0;
            prdata           <= 32'd0;
            pslverr          <= 1'b0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 1'b0;

            // Xử lý ghi nhận ngắt: 
            // - Chỉ ghi nhận nếu chân đó đang là Input (~gpio_dir)
            // - Có thể bị xóa (Clear) bởi CPU thông qua lệnh Write-1-to-Clear
            if (apb_write && paddr[11:0] == 12'h01C) begin
                // CPU đang ghi vào thanh ghi INT_STAT để xóa ngắt (Ghi 1 vào bit nào thì xóa bit đó)
                reg_int_stat <= (reg_int_stat & ~pwdata) | (irq_trigger & ~gpio_dir);
            end else begin
                // Bình thường: Tự động set bit lên 1 nếu có trigger
                reg_int_stat <= reg_int_stat | (irq_trigger & ~gpio_dir);
            end

            // Xử lý lệnh Ghi (Write) cho các thanh ghi khác
            if (apb_write) begin
                case (paddr[11:0])
                    12'h004: gpio_out         <= pwdata;
                    12'h008: gpio_dir         <= pwdata;
                    12'h00C: reg_int_en       <= pwdata;
                    12'h010: reg_int_type     <= pwdata;
                    12'h014: reg_int_polarity <= pwdata;
                    12'h018: reg_int_bothedge <= pwdata;
                    // 12'h01C được xử lý ở khối logic phía trên
                    12'h01C: ; // Dummy case để không rơi vào default
                    default: pslverr          <= 1'b1;
                endcase
            end
            
            // Xử lý lệnh Đọc (Read)
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= sync_f2;          // Đọc dữ liệu đầu vào thực tế
                    12'h004: prdata <= gpio_out;         // Đọc lại giá trị đang xuất
                    12'h008: prdata <= gpio_dir;         
                    12'h00C: prdata <= reg_int_en;
                    12'h010: prdata <= reg_int_type;
                    12'h014: prdata <= reg_int_polarity;
                    12'h018: prdata <= reg_int_bothedge;
                    12'h01C: prdata <= reg_int_stat;
                    default: begin prdata <= 32'd0; pslverr <= 1'b1; end
                endcase
            end
        end
    end

endmodule