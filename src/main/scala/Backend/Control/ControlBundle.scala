package CPUSTC.backend.control

import chisel3._
import chisel3.util._

import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.Fetch._
import CPUSTC.config.RegisterFile._
import CPUSTC.frontend.FtqPtr
import CPUSTC.backend.rob.RobPtr

object RedirectKind {
    val width = 2

    val BRANCH = 0.U(width.W)
    val HARD   = 1.U(width.W)
}

class PipelineRedirect extends Bundle {
    val kind   = UInt(RedirectKind.width.W)
    val target = UInt(dataWidth.W)

    val robPtr = new RobPtr

    val ftqPtr    = new FtqPtr
    val ftqOffset = UInt(log2Ceil(nfch).W)

    val cfiType     = UInt(CFI_SZ.W)
    val actualTaken = Bool()
    val isCall      = Bool()
    val isRet       = Bool()
}
