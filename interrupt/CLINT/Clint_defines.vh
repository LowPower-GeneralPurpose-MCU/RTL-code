// ============================================================
//  clint_defines.vh  —  Hằng số dùng chung, thuần Verilog
//  `include "clint_defines.vh" ở đầu mỗi module
// ============================================================
`ifndef CLINT_DEFINES_VH
`define CLINT_DEFINES_VH

//  Bus widths 
`define CLINT_DATA_W    32      // Data bus cố định 32-bit
`define CLINT_STRB_W     4      // Byte strobe = DATA_W/8

//  Memory map offsets (byte, từ CLINT base) 
//   msip[i]     : 0x000000 + i*4    (0x0000 – 0x3FFF)
//   mtimecmp[i] : 0x004000 + i*8    (0x4000 – 0xBFF7)
//   mtime lo    : 0x00BFF8
//   mtime hi    : 0x00BFFC
`define CLINT_MSIP_BASE      26'h000000
`define CLINT_MTIMECMP_BASE  26'h004000
`define CLINT_MTIME_LO       26'h00BFF8
`define CLINT_MTIME_HI       26'h00BFFC
`define CLINT_ADDR_MAX       26'h00BFFF

//  AXI4 Response codes 
`define AXI_OKAY    2'b00
`define AXI_EXOKAY  2'b01
`define AXI_SLVERR  2'b10
`define AXI_DECERR  2'b11

//  AXI4 Burst types 
`define AXI_BURST_FIXED  2'b00
`define AXI_BURST_INCR   2'b01
`define AXI_BURST_WRAP   2'b10

//  Reset value cho mtimecmp (không trigger interrupt ngay) 
`define CLINT_MTIMECMP_RST  64'hFFFF_FFFF_FFFF_FFFF

`endif