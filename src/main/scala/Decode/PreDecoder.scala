package CPUSTC.decode

import chisel3._
import chisel3.util._

import CPUSTC.config.Consts._
import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.isa.Instructions._

trait PreDecodeTable {
    val default = List[BitPat](N, N, N)
    val table:Array[(BitPat, List[BitPat])] = Array[(BitPat, List[BitPat])](
        ////                      is br?
        ////                      |  is bl?
        ////                      |  |  is jirl?
        ////                      |  |  |
        ////                      |  |  |
        JIRL              -> List(N, N, Y),
        B                 -> List(N, Y, N),
        BL                -> List(N, Y, N),
        BEQ               -> List(Y, N, N),
        BNE               -> List(Y, N, N),
        BLT               -> List(Y, N, N),
        BLTU              -> List(Y, N, N),
        BGE               -> List(Y, N, N),
        BGEU              -> List(Y, N, N)
    )
}

class PreDecodeSignals extends Bundle {
    val isRet       = Bool()
    val isCall      = Bool()
    val staticTaken = Bool()
    val target      = UInt(32.W)
    val cfiType     = UInt(CFI_SZ.W)
}

class PreDecoder extends Module with PreDecodeTable {
    val io = IO(new Bundle{
        val instr   = Input(UInt(32.W))
        val pc      = Input(UInt(32.W))
        val out     = Output(new PreDecodeSignals)
    })

    val preDecodeSignals = DecodeTool(io.instr, default, table)

    val isBr          = preDecodeSignals(0)(0)
    val isBl          = preDecodeSignals(1)(0)
    val isJirl        = preDecodeSignals(2)(0)

    io.out.isRet := (isJirl && io.instr(4,0) === 0.U && io.instr(9,5) === 1.U && io.instr(25,10) === 0.U)

    io.out.isCall := (isBl && io.instr(26)) || (isJirl && io.instr(4, 0) === 1.U)

    // BTFNT fallback for a conditional branch that misses in the BTB.
    io.out.staticTaken := isBr && io.instr(25)

    io.out.target := (
        Mux(isBr,
            Cat(Fill(14, io.instr(25)), io.instr(25, 10), 0.U(2.W)),
            Cat(Fill(4, io.instr(9)), io.instr(9, 0), io.instr(25, 10), 0.U(2.W))
        ).asSInt + io.pc.asSInt).asUInt

    io.out.cfiType := Mux(isBr,   CFI_BR,
                      Mux(isBl,   CFI_BL,
                      Mux(isJirl, CFI_JIRL,
                                  CFI_X)))
}
