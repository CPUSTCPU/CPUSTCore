package CPUSTC.backend.execute.fu

import chisel3._
import chisel3.util._

import CPUSTC.backend.{
  AddressFastMap,
  AddressTranslationState,
  CSRExceptionInfo,
  CSRDebugState,
  CSRFile,
  TLBRDResult,
  TLBSrchResult
}
import CPUSTC.config.Consts.CSR_TID
import CPUSTC.config.ExpCode
import CPUSTC.config.{CsrOp, SystemOp}
import CPUSTC.config.Execute._
import CPUSTC.config.RegisterFile._
import CPUSTC.backend.execute.ExecuteResult
import CPUSTC.backend.rob.RobPtr
import CPUSTC.memory.{MemSysConfig, SysMemCmd, SysMemResp}

class CSRUnitReq extends Bundle {
  val system  = Bool()
  val op      = UInt(FU_OP_SZ.W)
  val robPtr  = new RobPtr
  val pc      = UInt(dataWidth.W)
  val csrAddr = UInt(14.W)
  val csrData = UInt(dataWidth.W)
  val csrMask = UInt(dataWidth.W)
  val sysImm  = UInt(dataWidth.W)
  val auxOp   = UInt(5.W)
  val pdest   = UInt(CPUSTC.config.RegisterFile.wpreg.W)
  val rfWen   = Bool()
  val brMask  = UInt(CPUSTC.config.RenameConfig.maxBrCount.W)
}

class CSRUnitIO extends Bundle {
  val flush = Input(Bool())

  val req  = Flipped(Decoupled(new CSRUnitReq))
  val resp = Decoupled(new ExecuteResult)

  val sysMemCmd    = Decoupled(new SysMemCmd)
  val sysMemResp   = Flipped(Decoupled(new SysMemResp))

  val headReq   = Output(Valid(new RobPtr))
  val headGrant = Input(Valid(new RobPtr))
  val commit    = Input(Valid(new RobPtr))
  val exception = Flipped(Valid(new CSRExceptionInfo))
  val hardwareInterrupt = Input(UInt(8.W))

  val interruptPending = Output(Bool())
  val archRedirect = Output(Valid(UInt(dataWidth.W)))
  val idle         = Output(Bool())
  val idleResumePc = Output(UInt(dataWidth.W))
  val busy         = Output(Bool())
  val commitDone   = Output(Bool())
  val addressState = Output(new AddressTranslationState)
  val fastAddressMap = Output(new AddressFastMap)
  val llbitValue   = Input(Bool())
  val llbitClear   = Output(Bool())
  val debugState = Output(new CSRDebugState)
  val debugErtn = Output(Bool())
  val debugInterrupt = Output(UInt(11.W))
}

class CSRUnit(memSysConfig: MemSysConfig = MemSysConfig()) extends Module {
  val io = IO(new CSRUnitIO)

  val csr = Module(new CSRFile)
  private val cpuConfig = new CpuConfig(memSysConfig)

  object State extends ChiselEnum {
    val Idle, WaitRead, WaitHeadGrant, SysSend, SysWait, Response, WaitCommit = Value
  }

  val state = RegInit(State.Idle)
  val pending = Reg(new CSRUnitReq)
  val pendingLegal = RegInit(false.B)

  val resultValid = RegInit(false.B)
  val resultBits  = Reg(new ExecuteResult)

  val redirectValid = RegInit(false.B)
  val redirectTarget = Reg(UInt(dataWidth.W))
  val commitDone = RegInit(false.B)
  val idleResumePc = RegInit(0.U(dataWidth.W))

  val sysEpoch = RegInit(0.U(4.W))
  val pendingTlbSearchValid = RegInit(false.B)
  val pendingTlbSearch = Reg(new TLBSrchResult(5))
  val pendingTlbReadValid = RegInit(false.B)
  val pendingTlbRead = Reg(new TLBRDResult)

  def needsRead(req: CSRUnitReq): Bool =
    !req.system && (
      req.op === CsrOp.READ ||
        req.op === CsrOp.WRITE ||
        req.op === CsrOp.XCHG ||
        req.op === CsrOp.CNTID
    )

  def changesArchitecturalState(req: CSRUnitReq): Bool =
    req.system ||
      req.op === CsrOp.WRITE ||
      req.op === CsrOp.XCHG ||
      req.op === CsrOp.ERTN ||
      req.op === CsrOp.IDLE

  def privilegedSystemOp(req: CSRUnitReq): Bool =
    req.op === SystemOp.TLBSRCH ||
      req.op === SystemOp.TLBRD ||
      req.op === SystemOp.TLBWR ||
      req.op === SystemOp.TLBFILL ||
      req.op === SystemOp.INVTLB ||
      (req.op === SystemOp.CACOP && req.auxOp(4, 3) =/= 2.U)

  def privilegedCsrOp(op: UInt): Bool =
    op === CsrOp.READ ||
      op === CsrOp.WRITE ||
      op === CsrOp.XCHG ||
      op === CsrOp.ERTN ||
      op === CsrOp.IDLE

  def systemRedirectRequired(op: UInt): Bool =
    op === SystemOp.SC ||
      op === SystemOp.TLBRD ||
      op === SystemOp.TLBWR ||
      op === SystemOp.TLBFILL ||
      op === SystemOp.INVTLB ||
      op === SystemOp.DBAR ||
      op === SystemOp.IBAR ||
      op === SystemOp.CACOP

  def systemMutatesTranslation(op: UInt): Bool =
    op === SystemOp.TLBRD ||
      op === SystemOp.TLBWR ||
      op === SystemOp.TLBFILL ||
      op === SystemOp.INVTLB

  val incomingCsrPrivilegeFault =
    !io.req.bits.system &&
      privilegedCsrOp(io.req.bits.op) &&
      csr.io.mmu.crmd.plv =/= 0.U

  csr.io.exe.kill := io.flush
  csr.io.exe.req.valid :=
    state === State.Idle &&
      io.req.valid &&
      needsRead(io.req.bits) &&
      !incomingCsrPrivilegeFault &&
      !io.flush
  csr.io.exe.req.bits.addr := Mux(
    io.req.bits.op === CsrOp.CNTID,
    CSR_TID,
    io.req.bits.csrAddr
  )
  csr.io.exe.resp.ready :=
    state === State.WaitRead && !resultValid && !io.flush

  io.req.ready :=
    state === State.Idle &&
      !resultValid &&
      !io.flush &&
      Mux(needsRead(io.req.bits), csr.io.exe.req.ready, true.B)

  io.resp.valid := resultValid && !io.flush
  io.resp.bits  := resultBits

  private def sameRobPtr(left: RobPtr, right: RobPtr): Bool =
    left.asUInt === right.asUInt

  val commitHit =
    state === State.WaitCommit &&
      io.commit.valid &&
      sameRobPtr(io.commit.bits, pending.robPtr)

  val sysMemRespMatchesPending =
    io.sysMemResp.bits.robPtr.asUInt === pending.robPtr.asUInt &&
      io.sysMemResp.bits.epoch === sysEpoch

  // Collapse the instruction-selected CSR merge once, after the registered
  // read response. CSRFile then only applies each CSR's architectural mask.
  val pendingWriteMask = Mux(
    pending.op === CsrOp.WRITE,
    Fill(dataWidth, 1.U(1.W)),
    pending.csrMask
  )

  csr.io.cmt.write.valid :=
    commitHit &&
      pendingLegal &&
      !pending.system &&
      (pending.op === CsrOp.WRITE || pending.op === CsrOp.XCHG)
  csr.io.cmt.write.bits.addr := pending.csrAddr
  csr.io.cmt.write.bits.data := pending.csrData
  csr.io.cmt.write.bits.mask := pendingWriteMask

  csr.io.cmt.exception := io.exception
  csr.io.cmt.ertn :=
    commitHit && pendingLegal && !pending.system && pending.op === CsrOp.ERTN
  csr.io.cmt.idle :=
    commitHit && pendingLegal && !pending.system && pending.op === CsrOp.IDLE

  csr.io.intr.hwi := io.hardwareInterrupt
  csr.io.mmu.tlbsrch.valid :=
    commitHit && pending.system && pendingTlbSearchValid
  csr.io.mmu.tlbsrch.bits := pendingTlbSearch
  csr.io.mmu.tlbrd.valid :=
    commitHit && pending.system && pendingTlbReadValid
  csr.io.mmu.tlbrd.bits := pendingTlbRead
  csr.io.llbit.value := io.llbitValue
  io.llbitClear := csr.io.llbit.clear

  io.sysMemCmd.valid := state === State.SysSend && !io.flush
  io.sysMemCmd.bits := 0.U.asTypeOf(io.sysMemCmd.bits)
  io.sysMemCmd.bits.op := pending.op
  io.sysMemCmd.bits.robPtr := pending.robPtr
  io.sysMemCmd.bits.epoch := sysEpoch
  io.sysMemCmd.bits.pc := pending.pc
  io.sysMemCmd.bits.vaddr := Mux(
    pending.op === SystemOp.INVTLB,
    pending.csrMask,
    pending.csrData + pending.sysImm
  )
  io.sysMemCmd.bits.data := pending.csrMask
  io.sysMemCmd.bits.auxOp := pending.auxOp
  io.sysMemCmd.bits.tlbidx := csr.io.mmu.tlbidx
  io.sysMemCmd.bits.tlbehi := csr.io.mmu.tlbehi
  io.sysMemCmd.bits.tlbelo0 := csr.io.mmu.tlbelo0
  io.sysMemCmd.bits.tlbelo1 := csr.io.mmu.tlbelo1
  io.sysMemCmd.bits.asid := Mux(
    pending.op === SystemOp.INVTLB,
    pending.csrData(9, 0),
    csr.io.mmu.asid.asid
  )
  io.sysMemCmd.bits.inTlbRefill := csr.io.mmu.inTlbRefill
  // A command that crossed into the memory subsystem before a flush cannot be
  // recalled.  Consume its eventual response, but only a response tagged with
  // the current ROB pointer and system epoch may complete the live operation.
  io.sysMemResp.ready :=
    !io.flush && (state === State.SysWait || state === State.Idle)

  io.headReq.valid := state === State.WaitHeadGrant && !io.flush
  io.headReq.bits := pending.robPtr

  io.interruptPending := csr.io.intr.pending
  io.archRedirect.valid := redirectValid
  io.archRedirect.bits  := redirectTarget
  io.idle := csr.io.frontend.idle
  io.idleResumePc := idleResumePc
  io.busy := state =/= State.Idle || resultValid
  io.commitDone := commitDone
  io.addressState.crmd := csr.io.mmu.crmd
  io.addressState.asid := csr.io.mmu.asid.asid
  io.addressState.dmw0 := csr.io.mmu.dmw0
  io.addressState.dmw1 := csr.io.mmu.dmw1
  io.fastAddressMap := csr.io.mmu.fastAddressMap
  io.debugState := csr.io.debugState
  io.debugErtn := csr.io.debugErtn
  io.debugInterrupt := csr.io.debugInterrupt

  redirectValid := false.B
  commitDone := false.B

  when(io.flush) {
    state := State.Idle
    sysEpoch := sysEpoch + 1.U
    resultValid := false.B
    pendingLegal := false.B
    pendingTlbSearchValid := false.B
    pendingTlbReadValid := false.B
    redirectValid := false.B
    commitDone := false.B
  }.otherwise {
    when(io.req.fire) {
      pending := io.req.bits
      pendingLegal := !needsRead(io.req.bits)
      pendingTlbSearchValid := false.B
      pendingTlbReadValid := false.B

      val privilegeFault = csr.io.mmu.crmd.plv =/= 0.U && Mux(
        io.req.bits.system,
        privilegedSystemOp(io.req.bits),
        privilegedCsrOp(io.req.bits.op)
      )

      when(privilegeFault) {
        pendingLegal := false.B
        resultValid := true.B
        resultBits := 0.U.asTypeOf(new ExecuteResult)
        resultBits.robPtr := io.req.bits.robPtr
        resultBits.pdest := io.req.bits.pdest
        resultBits.rfWen := io.req.bits.rfWen
        resultBits.brMask := io.req.bits.brMask
        resultBits.exceptionValid := true.B
        resultBits.exceptionCause := ExpCode.IPE
        state := State.Response
      }.elsewhen(io.req.bits.system) {
        state := State.WaitHeadGrant
      }.elsewhen(needsRead(io.req.bits)) {
        state := State.WaitRead
      }.otherwise {
        resultValid := true.B
        resultBits := 0.U.asTypeOf(new ExecuteResult)
        resultBits.robPtr := io.req.bits.robPtr
        resultBits.pdest  := io.req.bits.pdest
        resultBits.rfWen  := io.req.bits.rfWen
        resultBits.data   := Mux(
          io.req.bits.op === CsrOp.CPUCFG,
          cpuConfig.read(io.req.bits.csrData),
          0.U
        )
        resultBits.brMask := io.req.bits.brMask
        state := State.Response
      }
    }

    when(
      state === State.WaitHeadGrant &&
        io.headGrant.valid &&
        sameRobPtr(io.headGrant.bits, pending.robPtr)
    ) {
      state := State.SysSend
    }

    when(state === State.SysSend && io.sysMemCmd.fire) {
      state := State.SysWait
    }

    when(
      state === State.SysWait &&
        io.sysMemResp.fire &&
        sysMemRespMatchesPending
    ) {
      pendingTlbSearchValid := io.sysMemResp.bits.tlbSearch.valid
      pendingTlbSearch := io.sysMemResp.bits.tlbSearch.bits
      pendingTlbReadValid := io.sysMemResp.bits.tlbRead.valid
      pendingTlbRead := io.sysMemResp.bits.tlbRead.bits

      resultValid := true.B
      resultBits := 0.U.asTypeOf(new ExecuteResult)
      resultBits.robPtr := pending.robPtr
      resultBits.pdest := pending.pdest
      resultBits.rfWen := pending.rfWen
      resultBits.data := io.sysMemResp.bits.data
      resultBits.brMask := pending.brMask
      resultBits.exceptionValid := io.sysMemResp.bits.exceptionValid
      resultBits.exceptionCause := io.sysMemResp.bits.exceptionCause
      resultBits.exceptionBadvValid :=
        io.sysMemResp.bits.exceptionValid && io.sysMemResp.bits.badvValid
      resultBits.exceptionBadv := Mux(
        io.sysMemResp.bits.exceptionValid && io.sysMemResp.bits.badvValid,
        io.sysMemResp.bits.badv,
        0.U
      )
      pendingLegal := !io.sysMemResp.bits.exceptionValid
      state := State.Response
    }

    when(csr.io.exe.resp.fire) {
      pendingLegal := csr.io.exe.resp.bits.legal
      when(
        csr.io.exe.resp.bits.legal &&
          (pending.op === CsrOp.WRITE || pending.op === CsrOp.XCHG)
      ) {
        pending.csrData :=
          (csr.io.exe.resp.bits.data & ~pendingWriteMask) |
            (pending.csrData & pendingWriteMask)
      }
      resultValid := true.B
      resultBits := 0.U.asTypeOf(new ExecuteResult)
      resultBits.robPtr := pending.robPtr
      resultBits.pdest  := pending.pdest
      resultBits.rfWen  := pending.rfWen
      resultBits.data   := csr.io.exe.resp.bits.data
      resultBits.brMask := pending.brMask
      resultBits.exceptionValid := !csr.io.exe.resp.bits.legal
      resultBits.exceptionCause := Mux(
        csr.io.exe.resp.bits.legal,
        0.U,
        ExpCode.INE
      )
      state := State.Response
    }

    when(io.resp.fire) {
      resultValid := false.B
      state := Mux(
        changesArchitecturalState(pending) && pendingLegal,
        State.WaitCommit,
        State.Idle
      )
    }

    when(commitHit) {
      assert(pendingLegal)
      commitDone := true.B
      redirectValid := Mux(
        pending.system,
        systemRedirectRequired(pending.op),
        true.B
      )
      redirectTarget := Mux(
        !pending.system && pending.op === CsrOp.ERTN,
        csr.io.frontend.jumpTgt,
        pending.pc + 4.U
      )
      when(!pending.system && pending.op === CsrOp.IDLE) {
        idleResumePc := pending.pc + 4.U
      }
      when(pending.system && systemMutatesTranslation(pending.op)) {
        sysEpoch := sysEpoch + 1.U
      }
      pendingTlbSearchValid := false.B
      pendingTlbReadValid := false.B
      state := State.Idle
    }

    when(io.exception.valid) {
      redirectValid := true.B
      redirectTarget := csr.io.frontend.jumpTgt
    }
  }

  when(state === State.WaitCommit) {
    assert(changesArchitecturalState(pending))
    assert(pendingLegal)
  }

  when(io.commit.valid) {
    assert(state === State.WaitCommit)
    assert(sameRobPtr(io.commit.bits, pending.robPtr))
  }

  when(io.exception.valid) {
    assert(state === State.Idle)
    assert(!commitHit)
  }

  when(csr.io.cmt.ertn || io.exception.valid) {
    assert(csr.io.frontend.flush)
    assert(csr.io.frontend.jumpEn)
  }
}
