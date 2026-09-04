package CPUSTC.predict

import chisel3._
import chisel3.util.{HasBlackBoxInline, log2Ceil}

private class BpuSdpRamBlackBox(width: Int, depth: Int)
    extends BlackBox(Map("RAM_WIDTH" -> width, "RAM_DEPTH" -> depth))
    with HasBlackBoxInline {

    val io = IO(new Bundle {
        val clock = Input(Clock())
        val ren   = Input(Bool())
        val raddr = Input(UInt(log2Ceil(depth).W))
        val rdata = Output(UInt(width.W))
        val wen   = Input(Bool())
        val waddr = Input(UInt(log2Ceil(depth).W))
        val wdata = Input(UInt(width.W))
    })

    setInline(
        "BpuSdpRamBlackBox.sv",
        s"""
           |module BpuSdpRamBlackBox #(parameter RAM_WIDTH = 59, parameter RAM_DEPTH = 16) (
           |  input  wire                           clock,
           |  input  wire                           ren,
           |  input  wire [$$clog2(RAM_DEPTH)-1:0] raddr,
           |  output reg  [RAM_WIDTH-1:0]           rdata,
           |  input  wire                           wen,
           |  input  wire [$$clog2(RAM_DEPTH)-1:0] waddr,
           |  input  wire [RAM_WIDTH-1:0]           wdata
           |);
           |  (* ram_style = "block" *) reg [RAM_WIDTH-1:0] mem [0:RAM_DEPTH-1];
           |
           |  always @(posedge clock) begin
           |    if (ren)
           |      rdata <= mem[raddr];
           |    if (wen)
           |      mem[waddr] <= wdata;
           |  end
           |endmodule
           |""".stripMargin
    )
}

class BpuSdpRam(width: Int, depth: Int, useBlackBox: Boolean) extends Module {
    require(width > 0)
    require(depth > 1)

    val io = IO(new Bundle {
        val ren   = Input(Bool())
        val raddr = Input(UInt(log2Ceil(depth).W))
        val rdata = Output(UInt(width.W))
        val wen   = Input(Bool())
        val waddr = Input(UInt(log2Ceil(depth).W))
        val wdata = Input(UInt(width.W))
    })

    if (useBlackBox) {
        val ram = Module(new BpuSdpRamBlackBox(width, depth))
        ram.io.clock := clock
        ram.io.ren   := io.ren
        ram.io.raddr := io.raddr
        ram.io.wen   := io.wen
        ram.io.waddr := io.waddr
        ram.io.wdata := io.wdata
        io.rdata := ram.io.rdata
    } else {
        val ram = SyncReadMem(depth, UInt(width.W), SyncReadMem.ReadFirst)
        io.rdata := ram.read(io.raddr, io.ren)
        when(io.wen) {
            ram.write(io.waddr, io.wdata)
        }
    }
}
