package CPUSTC.backend

import chisel3._
import chisel3.util._

import CPUSTC.config.Consts._
import CPUSTC.config.ExpCode

class CRMD extends Bundle {
  val r0   = UInt(23.W)
  val datm = UInt(2.W)
  val datf = UInt(2.W)
  val pg   = UInt(1.W)
  val da   = UInt(1.W)
  val ie   = UInt(1.W)
  val plv  = UInt(2.W)
}

class PRMD extends Bundle {
  val r0   = UInt(29.W)
  val pie  = UInt(1.W)
  val pplv = UInt(2.W)
}

class EUEN extends Bundle {
  val r0  = UInt(31.W)
  val fpe = UInt(1.W)
}

class ECFG extends Bundle {
  val r0_0     = UInt(19.W)
  val lie12_11 = UInt(2.W)
  val r0_1     = UInt(1.W)
  val lie9_0   = UInt(10.W)
}

class ESTAT extends Bundle {
  val r0_0     = UInt(1.W)
  val esubcode = UInt(9.W)
  val ecode    = UInt(6.W)
  val r0_1     = UInt(3.W)
  val is_12    = UInt(1.W)
  val is_11    = UInt(1.W)
  val r0_2     = UInt(1.W)
  val is9_2    = UInt(8.W)
  val is1_0    = UInt(2.W)
}

class ERA extends Bundle {
  val pc = UInt(32.W)
}

class BADV extends Bundle {
  val vaddr = UInt(32.W)
}

class EENTRY extends Bundle {
  val va = UInt(26.W)
  val r0 = UInt(6.W)
}

class CPUID extends Bundle {
  val r0     = UInt(23.W)
  val coreid = UInt(9.W)
}

class SAVE extends Bundle {
  val data = UInt(32.W)
}

class LLBCTL extends Bundle {
  val r0    = UInt(29.W)
  val klo   = UInt(1.W)
  val wcllb = UInt(1.W)
  val rollb = UInt(1.W)
}

class TLBIDX(indexBits: Int) extends Bundle {
  require(indexBits >= 1 && indexBits <= 16)

  val ne    = UInt(1.W)
  val r0_0  = UInt(1.W)
  val ps    = UInt(6.W)
  val r0_1  = UInt(8.W)
  val r0_2  = UInt((16 - indexBits).W)
  val index = UInt(indexBits.W)
}

class TLBEHI extends Bundle {
  val vppn = UInt(19.W)
  val r0   = UInt(13.W)
}

class TLBELO extends Bundle {
  val r0_0 = UInt(4.W)
  val ppn  = UInt(20.W)
  val r0_1 = UInt(1.W)
  val g    = UInt(1.W)
  val mat  = UInt(2.W)
  val plv  = UInt(2.W)
  val d    = UInt(1.W)
  val v    = UInt(1.W)
}

class ASID extends Bundle {
  val r0_0     = UInt(8.W)
  val asidbits = UInt(8.W)
  val r0_1     = UInt(6.W)
  val asid     = UInt(10.W)
}

class PGDL extends Bundle {
  val base = UInt(20.W)
  val r0   = UInt(12.W)
}

class PGDH extends Bundle {
  val base = UInt(20.W)
  val r0   = UInt(12.W)
}

class PGD extends Bundle {
  val base = UInt(20.W)
  val r0   = UInt(12.W)
}

class TLBRENTRY extends Bundle {
  val pa = UInt(26.W)
  val r0 = UInt(6.W)
}

class DMW extends Bundle {
  val vseg = UInt(3.W)
  val r0_0 = UInt(1.W)
  val pseg = UInt(3.W)
  val r0_1 = UInt(19.W)
  val mat  = UInt(2.W)
  val plv3 = UInt(1.W)
  val r0_2 = UInt(2.W)
  val plv0 = UInt(1.W)
}

class TID extends Bundle {
  val tid = UInt(32.W)
}

class TCFG(timerBits: Int) extends Bundle {
  require(timerBits >= 3 && timerBits <= 32)

  val r0       = UInt((32 - timerBits).W)
  val initval  = UInt((timerBits - 2).W)
  val periodic = UInt(1.W)
  val en       = UInt(1.W)
}

class TVAL(timerBits: Int) extends Bundle {
  require(timerBits >= 3 && timerBits <= 32)

  val r0      = UInt((32 - timerBits).W)
  val timeval = UInt(timerBits.W)
}

class CTAG extends Bundle {
  val ctag = UInt(32.W)
}

class CSRReg(tlbIndexBits: Int, timerBits: Int) extends Bundle {
  val crmd  = new CRMD
  val prmd  = new PRMD
  val euen  = new EUEN
  val ecfg  = new ECFG
  val estat = new ESTAT

  val era    = new ERA
  val badv   = new BADV
  val eentry = new EENTRY

  val tlbidx  = new TLBIDX(tlbIndexBits)
  val tlbehi  = new TLBEHI
  val tlbelo0 = new TLBELO
  val tlbelo1 = new TLBELO
  val asid    = new ASID
  val pgdl    = new PGDL
  val pgdh    = new PGDH

  val cpuid = new CPUID
  val save0 = new SAVE
  val save1 = new SAVE
  val save2 = new SAVE
  val save3 = new SAVE

  val tid  = new TID
  val tcfg = new TCFG(timerBits)
  val tval = new TVAL(timerBits)

  val ctag      = new CTAG
  val llbctl    = new LLBCTL
  val tlbrentry = new TLBRENTRY
  val dmw0      = new DMW
  val dmw1      = new DMW
}

class CSRReadReq extends Bundle {
  val addr = UInt(14.W)
}

class CSRReadResp extends Bundle {
  val data = UInt(32.W)
  val legal = Bool()
}

class CSRWriteInfo extends Bundle {
  val addr = UInt(14.W)
  val data = UInt(32.W)
  val mask = UInt(32.W)
}

class CSRExceptionInfo extends Bundle {
  val err_pc    = UInt(32.W)
  val instr     = UInt(32.W)
  val ecode     = UInt(6.W)
  val esubcode  = UInt(9.W)
  val badvValid = Bool()
  val badv      = UInt(32.W)
}

/** Simulation/debug-only architectural CSR snapshot.  Consumers must treat
  * this as observation data; it never participates in the functional path.
  */
class CSRDebugState extends Bundle {
  val crmd       = UInt(32.W)
  val prmd       = UInt(32.W)
  val euen       = UInt(32.W)
  val ecfg       = UInt(32.W)
  val estat      = UInt(32.W)
  val era        = UInt(32.W)
  val badv       = UInt(32.W)
  val eentry     = UInt(32.W)
  val tlbidx     = UInt(32.W)
  val tlbehi     = UInt(32.W)
  val tlbelo0    = UInt(32.W)
  val tlbelo1    = UInt(32.W)
  val asid       = UInt(32.W)
  val pgdl       = UInt(32.W)
  val pgdh       = UInt(32.W)
  val save0      = UInt(32.W)
  val save1      = UInt(32.W)
  val save2      = UInt(32.W)
  val save3      = UInt(32.W)
  val tid        = UInt(32.W)
  val tcfg       = UInt(32.W)
  val tval       = UInt(32.W)
  val ticlr      = UInt(32.W)
  val llbctl     = UInt(32.W)
  val tlbrentry  = UInt(32.W)
  val dmw0       = UInt(32.W)
  val dmw1       = UInt(32.W)
}

class CSRExeIO extends Bundle {
  val kill = Input(Bool())
  val req  = Flipped(Decoupled(new CSRReadReq))
  val resp = Decoupled(new CSRReadResp)
}

class CSRCommitIO extends Bundle {
  val write     = Input(Valid(new CSRWriteInfo))
  val exception = Input(Valid(new CSRExceptionInfo))
  val ertn      = Input(Bool())
  val idle      = Input(Bool())
}

class CSRFrontendIO extends Bundle {
  val flush   = Output(Bool())
  val jumpEn  = Output(Bool())
  val jumpTgt = Output(UInt(32.W))
  val idle    = Output(Bool())
}

class CSRInterruptIO extends Bundle {
  val hwi     = Input(UInt(8.W))
  val pending = Output(Bool())
}

class TLBSrchResult(tlbIndexBits: Int) extends Bundle {
  val found = Bool()
  val index = UInt(tlbIndexBits.W)
}

class TLBRDResult extends Bundle {
  val entryValid = Bool()
  val ps         = UInt(6.W)
  val vppn       = UInt(19.W)
  val asid       = UInt(10.W)
  val elo0       = new TLBELO
  val elo1       = new TLBELO
}

class CSRMMUIO(tlbIndexBits: Int) extends Bundle {
  val crmd = Output(new CRMD)
  val asid = Output(new ASID)
  val pgdl = Output(new PGDL)
  val pgdh = Output(new PGDH)
  val pgd  = Output(new PGD)
  val dmw0 = Output(new DMW)
  val dmw1 = Output(new DMW)
  val fastAddressMap = Output(new AddressFastMap)
  val inTlbRefill = Output(Bool())

  val tlbidx  = Output(new TLBIDX(tlbIndexBits))
  val tlbehi  = Output(new TLBEHI)
  val tlbelo0 = Output(new TLBELO)
  val tlbelo1 = Output(new TLBELO)

  val tlbsrch = Input(Valid(new TLBSrchResult(tlbIndexBits)))
  val tlbrd   = Input(Valid(new TLBRDResult))
}

class CSRLLBitIO extends Bundle {
  val value = Input(Bool())
  val clear = Output(Bool())
  val keep  = Output(Bool())
}

class CSRIO(tlbIndexBits: Int) extends Bundle {
  val exe      = new CSRExeIO
  val cmt      = new CSRCommitIO
  val frontend = new CSRFrontendIO
  val intr     = new CSRInterruptIO
  val mmu      = new CSRMMUIO(tlbIndexBits)
  val llbit    = new CSRLLBitIO
  val debugState = Output(new CSRDebugState)
  val debugErtn = Output(Bool())
  val debugInterrupt = Output(UInt(11.W))
}

class CSRFile(tlbIndexBits: Int = 5, timerBits: Int = 32) extends Module {
  require(tlbIndexBits >= 1 && tlbIndexBits <= 16)
  require(timerBits >= 3 && timerBits <= 32)

  val io = IO(new CSRIO(tlbIndexBits))

  private val csrRst = WireInit(0.U.asTypeOf(new CSRReg(tlbIndexBits, timerBits)))
  csrRst.crmd.da       := 1.U
  csrRst.asid.asidbits := 10.U

  private val csrRegNxt = Wire(new CSRReg(tlbIndexBits, timerBits))
  private val csrReg    = RegNext(csrRegNxt, init = csrRst)
  csrRegNxt := csrReg

  // Keep this shadow exactly aligned with csrReg, including writes,
  // exception entry and ERTN. RegisterRead then sees only a local VSEG lookup
  // instead of the live CRMD/DMW classification network.
  private val fastAddressMapRst =
    AddressFastMap.build(csrRst.crmd, csrRst.dmw0, csrRst.dmw1)
  private val fastAddressMapReg = RegNext(
    AddressFastMap.build(csrRegNxt.crmd, csrRegNxt.dmw0, csrRegNxt.dmw1),
    init = fastAddressMapRst
  )

  private def masked(oldValue: UInt, data: UInt, mask: UInt): UInt =
    (oldValue & ~mask) | (data & mask)

  private val estatRead = WireDefault(csrReg.estat)
  estatRead.is9_2 := io.intr.hwi
  estatRead.is_12 := 0.U

  private val llbctlRead = WireDefault(csrReg.llbctl)
  llbctlRead.wcllb := 0.U
  llbctlRead.rollb := io.llbit.value

  private val pgdRead = Wire(new PGD)
  pgdRead.base := Mux(csrReg.badv.vaddr(31), csrReg.pgdh.base, csrReg.pgdl.base)
  pgdRead.r0   := 0.U

  private def readCSR(addr: UInt): UInt = MuxLookup(addr, 0.U(32.W))(Seq(
    CSR_CRMD      -> csrReg.crmd.asUInt,
    CSR_PRMD      -> csrReg.prmd.asUInt,
    CSR_EUEN      -> csrReg.euen.asUInt,
    CSR_ECFG      -> csrReg.ecfg.asUInt,
    CSR_ESTAT     -> estatRead.asUInt,
    CSR_ERA       -> csrReg.era.asUInt,
    CSR_BADV      -> csrReg.badv.asUInt,
    CSR_EENTRY    -> csrReg.eentry.asUInt,
    CSR_TLBIDX    -> csrReg.tlbidx.asUInt,
    CSR_TLBEHI    -> csrReg.tlbehi.asUInt,
    CSR_TLBELO0   -> csrReg.tlbelo0.asUInt,
    CSR_TLBELO1   -> csrReg.tlbelo1.asUInt,
    CSR_ASID      -> csrReg.asid.asUInt,
    CSR_PGDL      -> csrReg.pgdl.asUInt,
    CSR_PGDH      -> csrReg.pgdh.asUInt,
    CSR_PGD       -> pgdRead.asUInt,
    CSR_CPUID     -> csrReg.cpuid.asUInt,
    CSR_SAVE0     -> csrReg.save0.asUInt,
    CSR_SAVE1     -> csrReg.save1.asUInt,
    CSR_SAVE2     -> csrReg.save2.asUInt,
    CSR_SAVE3     -> csrReg.save3.asUInt,
    CSR_TID       -> csrReg.tid.asUInt,
    CSR_TCFG      -> csrReg.tcfg.asUInt,
    CSR_TVAL      -> csrReg.tval.asUInt,
    CSR_TICLR     -> 0.U(32.W),
    CSR_LLBCTL    -> llbctlRead.asUInt,
    CSR_TLBRENTRY -> csrReg.tlbrentry.asUInt,
    CSR_CTAG      -> csrReg.ctag.asUInt,
    CSR_DMW0      -> csrReg.dmw0.asUInt,
    CSR_DMW1      -> csrReg.dmw1.asUInt
  ))

  private def csrImplemented(addr: UInt): Bool = Seq(
    CSR_CRMD, CSR_PRMD, CSR_EUEN, CSR_ECFG, CSR_ESTAT, CSR_ERA,
    CSR_BADV, CSR_EENTRY, CSR_TLBIDX, CSR_TLBEHI, CSR_TLBELO0,
    CSR_TLBELO1, CSR_ASID, CSR_PGDL, CSR_PGDH, CSR_PGD, CSR_CPUID,
    CSR_SAVE0, CSR_SAVE1, CSR_SAVE2, CSR_SAVE3, CSR_TID, CSR_TCFG,
    CSR_TVAL, CSR_TICLR, CSR_LLBCTL, CSR_TLBRENTRY, CSR_CTAG,
    CSR_DMW0, CSR_DMW1
  ).map(addr === _).reduce(_ || _)

  private val respValid = RegInit(false.B)
  private val respData  = Reg(UInt(32.W))
  private val respLegal = RegInit(false.B)

  io.exe.req.ready      := !respValid || io.exe.resp.ready
  io.exe.resp.valid     := respValid
  io.exe.resp.bits.data := respData
  io.exe.resp.bits.legal := respLegal

  when(io.exe.kill) {
    respValid := false.B
    respLegal := false.B
  }.elsewhen(io.exe.req.ready) {
    respValid := io.exe.req.valid
    when(io.exe.req.valid) {
      respData := readCSR(io.exe.req.bits.addr)
      respLegal := csrImplemented(io.exe.req.bits.addr)
    }
  }

  private val interruptStatus = Cat(
    0.U(1.W),
    csrReg.estat.is_11,
    0.U(1.W),
    io.intr.hwi,
    csrReg.estat.is1_0
  )
  io.intr.pending := csrReg.crmd.ie.asBool &&
    (interruptStatus & csrReg.ecfg.asUInt(12, 0)).orR

  private val idleReg = RegInit(false.B)
  when(io.intr.pending || io.cmt.exception.valid || io.cmt.ertn) {
    idleReg := false.B
  }.elsewhen(io.cmt.idle) {
    idleReg := true.B
  }

  io.frontend.flush   := false.B
  io.frontend.jumpEn  := false.B
  io.frontend.jumpTgt := 0.U
  io.frontend.idle    := idleReg
  io.llbit.clear      := false.B
  io.llbit.keep       := csrReg.llbctl.klo.asBool

  // TVAL=0 has two meanings for a one-shot timer: a freshly armed zero
  // interval must fire once, while an expired timer must remain stopped.
  // Keep that distinction outside the architectural CSR image.
  private val timerArmed = RegInit(false.B)
  when(csrReg.tcfg.en.asBool && timerArmed) {
    when(csrReg.tval.timeval <= 1.U) {
      csrRegNxt.estat.is_11 := 1.U
      when(csrReg.tcfg.periodic.asBool) {
        csrRegNxt.tval.timeval := Cat(csrReg.tcfg.initval, 0.U(2.W))
        timerArmed := true.B
      }.otherwise {
        csrRegNxt.tval.timeval := 0.U
        timerArmed := false.B
      }
    }.otherwise {
      csrRegNxt.tval.timeval := csrReg.tval.timeval - 1.U
    }
  }

  private val writeValid = io.cmt.write.valid &&
    !io.cmt.exception.valid && !io.cmt.ertn

  when(writeValid) {
    val w = io.cmt.write.bits
    // CSRUnit has already merged the instruction mask into w.data. Keep the
    // raw mask only for write-triggered side effects such as TCFG and TICLR.
    switch(w.addr) {
      is(CSR_CRMD) {
        csrRegNxt.crmd := masked(csrReg.crmd.asUInt, w.data, 0x1ff.U(32.W))
          .asTypeOf(new CRMD)
      }
      is(CSR_PRMD) {
        csrRegNxt.prmd := masked(csrReg.prmd.asUInt, w.data, 0x7.U(32.W))
          .asTypeOf(new PRMD)
      }
      is(CSR_EUEN) {
        csrRegNxt.euen := masked(csrReg.euen.asUInt, w.data, 0x1.U(32.W))
          .asTypeOf(new EUEN)
      }
      is(CSR_ECFG) {
        csrRegNxt.ecfg := masked(csrReg.ecfg.asUInt, w.data, 0x1bff.U(32.W))
          .asTypeOf(new ECFG)
      }
      is(CSR_ESTAT) {
        csrRegNxt.estat.is1_0 := w.data(1, 0)
      }
      is(CSR_ERA) {
        csrRegNxt.era := w.data.asTypeOf(new ERA)
      }
      is(CSR_BADV) {
        csrRegNxt.badv := w.data.asTypeOf(new BADV)
      }
      is(CSR_EENTRY) {
        csrRegNxt.eentry := masked(
          csrReg.eentry.asUInt,
          w.data,
          0xffffffc0L.U(32.W)
        ).asTypeOf(new EENTRY)
      }
      is(CSR_TLBIDX) {
        val indexMask = ((BigInt(1) << tlbIndexBits) - 1).U(32.W)
        val writableMask = 0xbf000000L.U(32.W) | indexMask
        csrRegNxt.tlbidx := masked(
          csrReg.tlbidx.asUInt,
          w.data,
          writableMask
        ).asTypeOf(new TLBIDX(tlbIndexBits))
      }
      is(CSR_TLBEHI) {
        csrRegNxt.tlbehi := masked(
          csrReg.tlbehi.asUInt,
          w.data,
          0xffffe000L.U(32.W)
        ).asTypeOf(new TLBEHI)
      }
      is(CSR_TLBELO0) {
        csrRegNxt.tlbelo0 := masked(
          csrReg.tlbelo0.asUInt,
          w.data,
          0x0fffff7fL.U(32.W)
        ).asTypeOf(new TLBELO)
      }
      is(CSR_TLBELO1) {
        csrRegNxt.tlbelo1 := masked(
          csrReg.tlbelo1.asUInt,
          w.data,
          0x0fffff7fL.U(32.W)
        ).asTypeOf(new TLBELO)
      }
      is(CSR_ASID) {
        csrRegNxt.asid.asid := w.data(9, 0)
      }
      is(CSR_PGDL) {
        csrRegNxt.pgdl := masked(
          csrReg.pgdl.asUInt,
          w.data,
          0xfffff000L.U(32.W)
        ).asTypeOf(new PGDL)
      }
      is(CSR_PGDH) {
        csrRegNxt.pgdh := masked(
          csrReg.pgdh.asUInt,
          w.data,
          0xfffff000L.U(32.W)
        ).asTypeOf(new PGDH)
      }
      is(CSR_SAVE0) {
        csrRegNxt.save0 := w.data.asTypeOf(new SAVE)
      }
      is(CSR_SAVE1) {
        csrRegNxt.save1 := w.data.asTypeOf(new SAVE)
      }
      is(CSR_SAVE2) {
        csrRegNxt.save2 := w.data.asTypeOf(new SAVE)
      }
      is(CSR_SAVE3) {
        csrRegNxt.save3 := w.data.asTypeOf(new SAVE)
      }
      is(CSR_TID) {
        csrRegNxt.tid := w.data.asTypeOf(new TID)
      }
      is(CSR_TCFG) {
        val timerMask = ((BigInt(1) << timerBits) - 1).U(32.W)
        val effectiveMask = w.mask & timerMask
        val nextTCFG = masked(csrReg.tcfg.asUInt, w.data, timerMask)
          .asTypeOf(new TCFG(timerBits))
        when(effectiveMask.orR) {
          csrRegNxt.tcfg := nextTCFG
          csrRegNxt.tval.timeval := Cat(nextTCFG.initval, 0.U(2.W))
          timerArmed := nextTCFG.en.asBool
        }
      }
      is(CSR_TICLR) {
        when(w.mask(0) && w.data(0)) {
          csrRegNxt.estat.is_11 := 0.U
        }
      }
      is(CSR_LLBCTL) {
        when(w.mask(2)) {
          csrRegNxt.llbctl.klo := w.data(2)
        }
        when(w.mask(1) && w.data(1)) {
          io.llbit.clear := true.B
        }
      }
      is(CSR_TLBRENTRY) {
        csrRegNxt.tlbrentry := masked(
          csrReg.tlbrentry.asUInt,
          w.data,
          0xffffffc0L.U(32.W)
        ).asTypeOf(new TLBRENTRY)
      }
      is(CSR_CTAG) {
        csrRegNxt.ctag := w.data.asTypeOf(new CTAG)
      }
      is(CSR_DMW0) {
        csrRegNxt.dmw0 := masked(
          csrReg.dmw0.asUInt,
          w.data,
          0xee000039L.U(32.W)
        ).asTypeOf(new DMW)
      }
      is(CSR_DMW1) {
        csrRegNxt.dmw1 := masked(
          csrReg.dmw1.asUInt,
          w.data,
          0xee000039L.U(32.W)
        ).asTypeOf(new DMW)
      }
    }
  }

  when(io.mmu.tlbsrch.valid && !io.cmt.exception.valid && !io.cmt.ertn) {
    csrRegNxt.tlbidx.ne := !io.mmu.tlbsrch.bits.found
    when(io.mmu.tlbsrch.bits.found) {
      csrRegNxt.tlbidx.index := io.mmu.tlbsrch.bits.index
    }
  }

  when(io.mmu.tlbrd.valid && !io.cmt.exception.valid && !io.cmt.ertn) {
    val result = io.mmu.tlbrd.bits
    csrRegNxt.tlbidx.ne := !result.entryValid
    when(result.entryValid) {
      csrRegNxt.tlbidx.ps    := result.ps
      csrRegNxt.tlbehi.vppn  := result.vppn
      csrRegNxt.asid.asid    := result.asid
      csrRegNxt.tlbelo0      := result.elo0
      csrRegNxt.tlbelo1      := result.elo1
      csrRegNxt.tlbelo0.r0_0 := 0.U
      csrRegNxt.tlbelo0.r0_1 := 0.U
      csrRegNxt.tlbelo1.r0_0 := 0.U
      csrRegNxt.tlbelo1.r0_1 := 0.U
    }.otherwise {
      csrRegNxt.tlbidx.ps    := 0.U
      csrRegNxt.tlbehi.vppn  := 0.U
      csrRegNxt.asid.asid    := 0.U
      csrRegNxt.tlbelo0      := 0.U.asTypeOf(new TLBELO)
      csrRegNxt.tlbelo1      := 0.U.asTypeOf(new TLBELO)
    }
  }

  assert(!(io.mmu.tlbsrch.valid && io.mmu.tlbrd.valid))

  when(io.cmt.ertn && !io.cmt.exception.valid) {
    csrRegNxt.crmd.plv := csrReg.prmd.pplv
    csrRegNxt.crmd.ie  := csrReg.prmd.pie
    when(csrReg.estat.ecode === ExpCode.TLBR) {
      csrRegNxt.crmd.da := 0.U
      csrRegNxt.crmd.pg := 1.U
    }

    io.frontend.flush   := true.B
    io.frontend.jumpEn  := true.B
    io.frontend.jumpTgt := csrReg.era.pc
    io.llbit.clear      := !csrReg.llbctl.klo.asBool
    csrRegNxt.llbctl.klo := 0.U
  }

  when(io.cmt.exception.valid) {
    val exception = io.cmt.exception.bits

    csrRegNxt.prmd.pplv := csrReg.crmd.plv
    csrRegNxt.prmd.pie  := csrReg.crmd.ie
    csrRegNxt.crmd.plv  := 0.U
    csrRegNxt.crmd.ie   := 0.U

    when(exception.ecode === ExpCode.TLBR) {
      csrRegNxt.crmd.da := 1.U
      csrRegNxt.crmd.pg := 0.U
    }

    csrRegNxt.era.pc         := exception.err_pc
    csrRegNxt.estat.ecode    := exception.ecode
    csrRegNxt.estat.esubcode := exception.esubcode

    when(exception.badvValid) {
      csrRegNxt.badv.vaddr := exception.badv
    }

    val tlbAddressException = exception.ecode === ExpCode.TLBR ||
      exception.ecode === ExpCode.PIL ||
      exception.ecode === ExpCode.PIS ||
      exception.ecode === ExpCode.PIF ||
      exception.ecode === ExpCode.PME ||
      exception.ecode === ExpCode.PPI
    when(tlbAddressException && exception.badvValid) {
      csrRegNxt.tlbehi.vppn := exception.badv(31, 13)
    }

    io.frontend.flush  := true.B
    io.frontend.jumpEn := true.B
    io.frontend.jumpTgt := Mux(
      exception.ecode === ExpCode.TLBR,
      csrReg.tlbrentry.asUInt,
      csrReg.eentry.asUInt
    )
  }

  when(io.cmt.exception.valid || io.cmt.ertn || io.exe.kill) {
    respValid := false.B
  }

  io.mmu.crmd    := csrReg.crmd
  io.mmu.asid    := csrReg.asid
  io.mmu.pgdl    := csrReg.pgdl
  io.mmu.pgdh    := csrReg.pgdh
  io.mmu.pgd     := pgdRead
  io.mmu.dmw0    := csrReg.dmw0
  io.mmu.dmw1    := csrReg.dmw1
  io.mmu.fastAddressMap := fastAddressMapReg
  io.mmu.inTlbRefill := csrReg.estat.ecode === ExpCode.TLBR
  io.mmu.tlbidx  := csrReg.tlbidx
  io.mmu.tlbehi  := csrReg.tlbehi
  io.mmu.tlbelo0 := csrReg.tlbelo0
  io.mmu.tlbelo1 := csrReg.tlbelo1

  // Export the post-update architectural image used at this cycle's commit
  // boundary.  Hardware interrupt and LLBit fields are overlaid because they
  // are not stored directly in CSRReg.
  private val debugEstat = WireDefault(csrRegNxt.estat)
  debugEstat.is9_2 := io.intr.hwi
  debugEstat.is_12 := 0.U
  private val debugLlbctl = WireDefault(csrRegNxt.llbctl)
  debugLlbctl.wcllb := 0.U
  debugLlbctl.rollb := io.llbit.value && !io.llbit.clear
  io.debugState.crmd := csrRegNxt.crmd.asUInt
  io.debugState.prmd := csrRegNxt.prmd.asUInt
  io.debugState.euen := csrRegNxt.euen.asUInt
  io.debugState.ecfg := csrRegNxt.ecfg.asUInt
  io.debugState.estat := debugEstat.asUInt
  io.debugState.era := csrRegNxt.era.asUInt
  io.debugState.badv := csrRegNxt.badv.asUInt
  io.debugState.eentry := csrRegNxt.eentry.asUInt
  io.debugState.tlbidx := csrRegNxt.tlbidx.asUInt
  io.debugState.tlbehi := csrRegNxt.tlbehi.asUInt
  io.debugState.tlbelo0 := csrRegNxt.tlbelo0.asUInt
  io.debugState.tlbelo1 := csrRegNxt.tlbelo1.asUInt
  io.debugState.asid := csrRegNxt.asid.asUInt
  io.debugState.pgdl := csrRegNxt.pgdl.asUInt
  io.debugState.pgdh := csrRegNxt.pgdh.asUInt
  io.debugState.save0 := csrRegNxt.save0.asUInt
  io.debugState.save1 := csrRegNxt.save1.asUInt
  io.debugState.save2 := csrRegNxt.save2.asUInt
  io.debugState.save3 := csrRegNxt.save3.asUInt
  io.debugState.tid := csrRegNxt.tid.asUInt
  io.debugState.tcfg := csrRegNxt.tcfg.asUInt
  io.debugState.tval := csrRegNxt.tval.asUInt
  io.debugState.ticlr := 0.U
  io.debugState.llbctl := debugLlbctl.asUInt
  io.debugState.tlbrentry := csrRegNxt.tlbrentry.asUInt
  io.debugState.dmw0 := csrRegNxt.dmw0.asUInt
  io.debugState.dmw1 := csrRegNxt.dmw1.asUInt
  io.debugErtn := io.cmt.ertn
  io.debugInterrupt := interruptStatus(12, 2)
}
