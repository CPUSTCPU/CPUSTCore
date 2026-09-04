package CPUSTC.backend.execute.fu

import chisel3._
import chisel3.util._

import CPUSTC.config.Memory._
import CPUSTC.config.RegisterFile.dataWidth

class AddressGenerationUnitIO extends Bundle {
    val base    = Input(UInt(dataWidth.W))
    val offset  = Input(UInt(dataWidth.W))
    val memType = Input(UInt(MEM_TYPE_SZ.W))

    val vaddr          = Output(UInt(dataWidth.W))
    val sizeMask       = Output(UInt(dataBytes.W))
    val addrMisaligned = Output(Bool())
}

class AddressGenerationUnit extends Module {
    require(dataWidth == 32)
    require(dataBytes == 4)

    val io = IO(new AddressGenerationUnitIO)

    val vaddr = io.base + io.offset
    io.vaddr := vaddr

    val memTypeLegal =
        io.memType === MEM_BYTE ||
        io.memType === MEM_HALF ||
        io.memType === MEM_WORD

    val rawMisaligned = MuxLookup(io.memType, true.B)(Seq(
        MEM_BYTE -> false.B,
        MEM_HALF -> vaddr(0),
        MEM_WORD -> vaddr(1, 0).orR
    ))

    val rawMask = MuxLookup(io.memType, 0.U(dataBytes.W))(Seq(
        MEM_BYTE -> "b0001".U(dataBytes.W),
        MEM_HALF -> "b0011".U(dataBytes.W),
        MEM_WORD -> "b1111".U(dataBytes.W)
    ))

    io.addrMisaligned := rawMisaligned
    io.sizeMask := Mux(
        memTypeLegal && !rawMisaligned,
        rawMask,
        0.U
    )
}
