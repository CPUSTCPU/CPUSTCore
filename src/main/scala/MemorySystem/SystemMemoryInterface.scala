package CPUSTC.memory

import chisel3._
import chisel3.util._

import CPUSTC.backend.{TLBIDX, TLBEHI, TLBELO, TLBRDResult, TLBSrchResult}
import CPUSTC.config.Execute.FU_OP_SZ
import CPUSTC.config.RegisterFile.dataWidth
import CPUSTC.backend.rob.RobPtr

/** Registered command crossing from the ROB-head privileged slow path into
  * the memory subsystem. Only one command is currently outstanding.
  */
class SysMemCmd(tlbIndexBits: Int = 5, epochBits: Int = 4) extends Bundle {
  val op     = UInt(FU_OP_SZ.W)
  val robPtr = new RobPtr
  val epoch  = UInt(epochBits.W)
  val pc     = UInt(dataWidth.W)

  val vaddr = UInt(dataWidth.W)
  val data  = UInt(dataWidth.W)
  val auxOp = UInt(5.W)

  val tlbidx  = new TLBIDX(tlbIndexBits)
  val tlbehi  = new TLBEHI
  val tlbelo0 = new TLBELO
  val tlbelo1 = new TLBELO
  val asid    = UInt(10.W)
  val inTlbRefill = Bool()
}

class SysMemResp(tlbIndexBits: Int = 5, epochBits: Int = 4) extends Bundle {
  val robPtr = new RobPtr
  val epoch  = UInt(epochBits.W)
  val data   = UInt(dataWidth.W)

  val exceptionValid = Bool()
  val exceptionCause = UInt(8.W)
  val badvValid       = Bool()
  val badv            = UInt(dataWidth.W)

  val tlbSearch = Valid(new TLBSrchResult(tlbIndexBits))
  val tlbRead   = Valid(new TLBRDResult)
  val tlbFill   = Valid(UInt(tlbIndexBits.W))
}

class TlbFillDebugEvent(tlbIndexBits: Int = 5) extends Bundle {
  val robPtr = new RobPtr
  val index = UInt(tlbIndexBits.W)
}
