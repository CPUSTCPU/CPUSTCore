package CPUSTC.predict

import chisel3._
import chisel3.util.{HasBlackBoxInline, log2Ceil}

private class RasAsyncRamBlackBox(width: Int, depth: Int)
    extends BlackBox(Map("RAM_WIDTH" -> width, "RAM_DEPTH" -> depth))
    with HasBlackBoxInline {

    val io = IO(new Bundle {
        val clock = Input(Clock())
        val raddr = Input(UInt(log2Ceil(depth).W))
        val rdata = Output(UInt(width.W))
        val wen   = Input(Bool())
        val waddr = Input(UInt(log2Ceil(depth).W))
        val wdata = Input(UInt(width.W))
    })

    setInline(
        "RasAsyncRamBlackBox.sv",
        s"""
           |module RasAsyncRamBlackBox #(
           |  parameter RAM_WIDTH = 30,
           |  parameter RAM_DEPTH = 16
           |) (
           |  input  wire                           clock,
           |  input  wire [$$clog2(RAM_DEPTH)-1:0] raddr,
           |  output wire [RAM_WIDTH-1:0]           rdata,
           |  input  wire                           wen,
           |  input  wire [$$clog2(RAM_DEPTH)-1:0] waddr,
           |  input  wire [RAM_WIDTH-1:0]           wdata
           |);
           |  (* ram_style = "distributed" *) reg [RAM_WIDTH-1:0] mem [0:RAM_DEPTH-1];
           |
           |  always @(posedge clock) begin
           |    if (wen)
           |      mem[waddr] <= wdata;
           |  end
           |
           |  assign rdata = mem[raddr];
           |endmodule
           |""".stripMargin
    )
}

class RasAsyncRam(width: Int, depth: Int, useBlackBox: Boolean)
    extends Module {
    require(width > 0)
    require(depth > 1)

    val io = IO(new Bundle {
        val raddr = Input(UInt(log2Ceil(depth).W))
        val rdata = Output(UInt(width.W))
        val wen   = Input(Bool())
        val waddr = Input(UInt(log2Ceil(depth).W))
        val wdata = Input(UInt(width.W))
    })

    if (useBlackBox) {
        val ram = Module(new RasAsyncRamBlackBox(width, depth))
        ram.io.clock := clock
        ram.io.raddr := io.raddr
        ram.io.wen := io.wen
        ram.io.waddr := io.waddr
        ram.io.wdata := io.wdata
        io.rdata := ram.io.rdata
    } else {
        val ram = Mem(depth, UInt(width.W))
        when(io.wen) {
            ram.write(io.waddr, io.wdata)
        }
        io.rdata := ram.read(io.raddr)
    }
}
