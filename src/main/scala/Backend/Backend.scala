package CPUSTC.backend

import chisel3._
import chisel3.util._

import CPUSTC.config.Commit._
import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.Decode._
import CPUSTC.config.Execute.{intPorts => intPortParams}
import CPUSTC.config.FunctionUnit._
import CPUSTC.config.EXEOp.opLL
import CPUSTC.config.Issue._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.control.{BranchUpdateDomain, PipelineControl}
import CPUSTC.backend.dispatch.{Dispatch, LsqAllocator}
import CPUSTC.backend.execute.{IntExecutePort, MemExecutePort}
import CPUSTC.backend.issue.{IntIssueQueue, MemIssueQueue}
import CPUSTC.backend.regfile.{PhysicalRegisterFile, RegisterRead}
import CPUSTC.backend.rename.Rename
import CPUSTC.backend.rob.{Rob, RobExceptionWriteback, RobPtr}
import CPUSTC.backend.writeback.Writeback
import CPUSTC.config.ExpCode
import CPUSTC.memory.MemSysConfig

class Backend(
    enableCommitDebug: Boolean = false,
    enablePerfCounters: Boolean = false,
    maxCommitPerCycle: Int = ncmt,
    memSysConfig: MemSysConfig = MemSysConfig()
) extends Module {
    val io = IO(new BackendIO(enableCommitDebug, enablePerfCounters))

    val rename     = Module(new Rename)
    val dispatch   = Module(new Dispatch)
    val lsqAlloc   = Module(new LsqAllocator)
    val rob        = Module(new Rob(
        maxCommitPerCycle = maxCommitPerCycle,
        enableDebug = false
    ))

    val intIq      = Module(new IntIssueQueue)
    val memIq      = Module(new MemIssueQueue)

    val regRead    = Module(new RegisterRead)
    val regFile    = Module(new PhysicalRegisterFile(
        forwardingWritePorts = nIntWb,
        forwardingReadPorts = intNissue * 2
    ))

    require(intPortParams.length == intNissue)

    val intPorts = intPortParams.map { params =>
        Module(new IntExecutePort(params, memSysConfig))
    }
    private val csrPortIndex = intPortParams.indexWhere(_.csr)
    require(csrPortIndex >= 0)
    val csrPort = intPorts(csrPortIndex)

    val memPorts = Seq.fill(memNissue) {
        Module(new MemExecutePort)
    }

    val control    = Module(new PipelineControl)
    val writeback  = Module(new Writeback)

    val architecturalRequestKill = WireDefault(
        VecInit.fill(memNissue)(false.B)
    )
    val csrCommitEvent = WireDefault(
        0.U.asTypeOf(Valid(new RobPtr))
    )
    val csrExceptionEvent = WireDefault(
        0.U.asTypeOf(Valid(new CSRExceptionInfo))
    )

    for (p <- 0 until intNissue) {
        intPorts(p).io.csrCommit := csrCommitEvent
        intPorts(p).io.sysHeadGrant := 0.U.asTypeOf(intPorts(p).io.sysHeadGrant)
        if (p != csrPortIndex) {
            intPorts(p).io.sysMemCmd.ready := false.B
            intPorts(p).io.sysMemResp.valid := false.B
            intPorts(p).io.sysMemResp.bits :=
                0.U.asTypeOf(intPorts(p).io.sysMemResp.bits)
        }
        intPorts(p).io.hardwareInterrupt := io.hardwareInterrupt
        intPorts(p).io.llbitValue := io.llbitValue
        if (intPortParams(p).csr) {
            intPorts(p).io.csrException := csrExceptionEvent
        } else {
            intPorts(p).io.csrException :=
                0.U.asTypeOf(intPorts(p).io.csrException)
        }
    }

    io.sysMemCmd.valid := csrPort.io.sysMemCmd.valid
    io.sysMemCmd.bits := csrPort.io.sysMemCmd.bits
    csrPort.io.sysMemCmd.ready := io.sysMemCmd.ready
    csrPort.io.sysMemResp.valid := io.sysMemResp.valid
    csrPort.io.sysMemResp.bits := io.sysMemResp.bits
    io.sysMemResp.ready := csrPort.io.sysMemResp.ready
    io.addressState := csrPort.io.csrAddressState
    io.llbitClear := csrPort.io.llbitClear
    io.csrDebugState := csrPort.io.csrDebugState
    io.csrDebugErtn := csrPort.io.csrDebugErtn
    io.csrDebugInterrupt := csrPort.io.csrDebugInterrupt
    for (port <- 0 until intNissue) {
        io.counterDebug(port) := intPorts(port).io.counterDebug
    }
    io.loadDebug := writeback.io.loadDebug

    rename.io.in <> io.decode
    dispatch.io.in <> rename.io.out

    dispatch.io.intIq <> intIq.io.dispatch
    dispatch.io.memIq <> memIq.io.dispatch
    dispatch.io.rob   <> rob.io.enq
    dispatch.io.lsq   <> lsqAlloc.io.dispatch
    dispatch.io.intResidentProducers := intIq.io.residentIntProducers
    dispatch.io.intAllocOH := intIq.io.dispatchAllocOH

    regRead.io.intIssue <> intIq.io.issue
    regRead.io.memIssue <> memIq.io.issue
    io.ftqPredictionReadReq := intIq.io.ftqPredictionReadReq
    regRead.io.ftqPredictionReadResp := io.ftqPredictionReadResp

    for (p <- 0 until intNissue) {
        intPorts(p).io.in <> regRead.io.intExecute(p)
    }

    for (p <- 0 until memNissue) {
        memPorts(p).io.in <> regRead.io.memExecute(p)
        memPorts(p).io.fastAddressMap := csrPort.io.csrFastAddressMap
    }
    regRead.io.fastAddressMap := csrPort.io.csrFastAddressMap

    regFile.io.readReq  := regRead.io.rfReadReq
    regRead.io.rfReadData := regFile.io.readData

    regFile.io.write := writeback.io.rfWrite
    regRead.io.bypass := writeback.io.bypass

    val issueWakeup = WireDefault(
        0.U.asTypeOf(Vec(nDataWb, Valid(new CPUSTC.backend.issue.IssueWakeup)))
    )
    val renameWakeup = WireDefault(
        0.U.asTypeOf(Vec(nDataWb, Valid(new CPUSTC.backend.rename.RenameWakeupInfo)))
    )

    require(nFastIntWb == 2)
    require(nIntWb == 3)

    val p2Input = regRead.io.intExecute(2)
    val p2FixedWakeEligible =
        (
            p2Input.bits.uop.ctrl.fuType === FU_ALU ||
            p2Input.bits.uop.ctrl.fuType === FU_MUL
        ) &&
        p2Input.bits.uop.reg.rfWen &&
        p2Input.bits.uop.reg.pdest =/= 0.U
    val p2FixedAccepted = p2Input.fire && p2FixedWakeEligible
    val p2RawFixedAccepted =
        p2Input.valid &&
        intPorts(2).io.recoveryIndependentReady &&
        p2FixedWakeEligible

    val p2AcceptedPdest = p2Input.bits.uop.reg.pdest

    for (w <- 0 until nDataWb) {
        issueWakeup(w) := writeback.io.issueWakeup(w)
        issueWakeup(w).bits.fast := false.B
        renameWakeup(w) := writeback.io.renameWakeup(w)
    }

    // P2 ALU/MUL have a fixed two-cycle latency after P2 accepts them. Wake
    // dependents at that acceptance point; their RegisterRead cycle then
    // coincides with the producer's normal writeback bypass. DIV remains on
    // the response-time wakeup path because its latency is variable.
    issueWakeup(2).valid :=
        p2FixedAccepted || writeback.io.issueWakeup(2).valid
    issueWakeup(2).bits.pdest := Mux(
        p2FixedAccepted,
        p2AcceptedPdest,
        writeback.io.issueWakeup(2).bits.pdest
    )
    issueWakeup(2).bits.fast := false.B

    val intIqWakeup = WireDefault(
        0.U.asTypeOf(Vec(nDataWb, Valid(new CPUSTC.backend.issue.IssueWakeup)))
    )
    for (p <- 0 until nIntWb) {
        intIqWakeup(p).valid := intPorts(p).io.rawWakeup.valid
        intIqWakeup(p).bits.pdest := intPorts(p).io.rawWakeup.bits
        intIqWakeup(p).bits.fast := false.B
    }
    intIqWakeup(2).valid :=
        p2RawFixedAccepted || intPorts(2).io.rawWakeup.valid
    intIqWakeup(2).bits.pdest := Mux(
        p2RawFixedAccepted,
        p2AcceptedPdest,
        intPorts(2).io.rawWakeup.bits
    )
    intIq.io.p2DeferredWake :=
        p2RawFixedAccepted && !p2FixedAccepted
    intIq.io.p2FixedWakeAccepted := p2FixedAccepted
    for (l <- 0 until nLoadWb) {
        intIqWakeup(nIntWb + l) := writeback.io.rawLoadWakeup(l)
    }

    renameWakeup(2).valid :=
        p2FixedAccepted || writeback.io.renameWakeup(2).valid
    renameWakeup(2).bits.pdest := Mux(
        p2FixedAccepted,
        p2AcceptedPdest,
        writeback.io.renameWakeup(2).bits.pdest
    )

    when(p2FixedAccepted) {
        assert(p2Input.bits.uop.reg.ldestValid)
        assert(p2AcceptedPdest =/= 0.U)
    }

    assert(
        writeback.io.intResult(2).ready,
        "P2 acceptance wakeup requires a non-backpressured integer writeback"
    )
    assert(
        writeback.io.intResult(1).ready,
        "P1 selection wakeup requires a non-backpressured integer writeback"
    )

    intIq.io.wakeup := intIqWakeup
    memIq.io.wakeup := issueWakeup
    memIq.io.intFastWakeup := intIq.io.fastWakeup
    memIq.io.intProducerFastWakeMask := intIq.io.producerFastWakeMask
    memIq.io.intProducerReleaseMask := intIq.io.producerReleaseMask
    intIq.io.loadPredWake := io.loadPredWake
    intIq.io.loadPredResolve := io.loadPredResolve
    memIq.io.loadPredWake := io.loadPredWake
    memIq.io.loadPredResolve := io.loadPredResolve
    rename.io.wakeup := renameWakeup

    // Keep the IQ-local fast wakeup combinational so resident and currently
    // dispatching consumers retain back-to-back issue.  Rename only needs the
    // event one cycle later; registering this branch removes issue selection
    // from the global BusyTable update cone.
    val delayedRenameWakeup = RegInit(
        0.U.asTypeOf(Vec(
            nFastIntWb,
            Valid(new CPUSTC.backend.rename.RenameWakeupInfo)
        ))
    )
    for (w <- 0 until nFastIntWb) {
        delayedRenameWakeup(w).valid := intIq.io.fastWakeup(w).valid
        delayedRenameWakeup(w).bits.pdest := intIq.io.fastWakeup(w).bits.pdest
        rename.io.earlyWakeup(w) := delayedRenameWakeup(w)
    }

    val fastForward = Wire(Vec(nFastIntWb, Valid(new CPUSTC.backend.execute.ExecuteForward)))
    for (w <- 0 until nFastIntWb) {
        fastForward(w) := intPorts(w).io.operandForward
    }

    for (p <- 0 until intNissue) {
        intPorts(p).io.fastForward := fastForward
    }

    for (p <- 0 until memNissue) {
        memPorts(p).io.fastForward := fastForward
    }

    for (p <- 0 until intNissue) {
        // CSR issue is already serialized by IntIssueQueue's full ROB-head
        // check. RegisterRead provides the normal two-entry backpressure, so a
        // buffered CSR must not remove P1's ALU capability combinationally.
        val fixedStatus = intPorts(p).io.status.supportedFuMask
        val fixedCaps = fixedStatus & (~FU_DIV).asUInt
        val divCaps = if (intPortParams(p).div) {
            Mux(
                (regRead.io.intBufferedFuMask(p) & FU_DIV).orR,
                0.U(FUC_SZ.W),
                intPorts(p).io.status.readyFuMask & FU_DIV
            )
        } else {
            0.U(FUC_SZ.W)
        }

        intIq.io.portCaps(p) := fixedCaps | divCaps
    }

    for (p <- 0 until memNissue) {
        memIq.io.portCaps(p) := FU_MEM
    }

    for (p <- 0 until intNissue) {
        writeback.io.intResult(p) <> intPorts(p).io.result
    }
    for (p <- 0 until nIntWb) {
        writeback.io.fastIntRawValid(p) := intPorts(p).io.robRawValid
    }

    val loadRequestFire = Wire(Vec(memNissue, Bool()))
    val loadRequestLdindex = Wire(Vec(memNissue, UInt(nldq.W)))
    val loadRequestLdindexHigh = Wire(Vec(memNissue, Bool()))

    for (p <- 0 until memNissue) {
        io.memRequest(p).valid :=
            memPorts(p).io.backendInst.valid && !architecturalRequestKill(p)
        io.memRequest(p).bits := memPorts(p).io.backendInst.bits
        memPorts(p).io.backendInst.ready := io.memRequest(p).ready

        // A direct cached Load has no architectural side effect.  Let an
        // in-flight request cross a same-cycle rare redirect; the registered
        // state flush and LDQ generation check discard its eventual result.
        // Keeping this connection direct also prevents CSR/replay recovery
        // from entering the cache-read valid/ready timing cone.
        io.directCachedLoad(p).valid := memPorts(p).io.directCachedLoad.valid
        io.directCachedLoad(p).bits := memPorts(p).io.directCachedLoad.bits
        memPorts(p).io.directCachedLoad.ready := io.directCachedLoad(p).ready

        val slowLoadFire =
            io.memRequest(p).fire && io.memRequest(p).bits.uop.isLD
        val directLoadFire = io.directCachedLoad(p).fire
        loadRequestFire(p) := slowLoadFire || directLoadFire
        loadRequestLdindex(p) := Mux(
            directLoadFire,
            io.directCachedLoad(p).bits.ldindex,
            io.memRequest(p).bits.ldindex
        )
        loadRequestLdindexHigh(p) := Mux(
            directLoadFire,
            io.directCachedLoad(p).bits.ldindexHigh,
            io.memRequest(p).bits.ldindexHigh
        )

        assert(!(io.memRequest(p).valid && io.directCachedLoad(p).valid))
    }

    val staAccepted = Wire(Vec(
        memNissue,
        Valid(new CPUSTC.backend.dispatch.StqPtr)
    ))
    for (p <- 0 until memNissue) {
        staAccepted(p).valid :=
            io.memRequest(p).fire && io.memRequest(p).bits.uop.isSTA
        staAccepted(p).bits.oh := io.memRequest(p).bits.sqindex
        staAccepted(p).bits.flag := io.memRequest(p).bits.sqindexHigh
    }
    memIq.io.staEarlyAccepted :=
        VecInit(memPorts.map(_.io.staDependencyReleaseEarly))
    for (p <- 0 until memNissue) {
        val executeRelease = memPorts(p).io.staDependencyRelease
        val translatedPageStaAccepted =
            staAccepted(p).valid && io.memRequest(p).bits.translationPending

        memIq.io.staAccepted(p).valid :=
            executeRelease.valid || translatedPageStaAccepted
        memIq.io.staAccepted(p).bits := Mux(
            translatedPageStaAccepted,
            staAccepted(p).bits,
            executeRelease.bits
        )
    }
    lsqAlloc.io.staAccepted := staAccepted

    writeback.io.loadResult <> io.loadResult

    val storeCompletePacked = Wire(Vec(
        nStoreComplete,
        Valid(new RobPtr)
    ))
    var remainingStoreComplete = VecInit(io.storeComplete.map(_.valid)).asUInt

    for (s <- 0 until nStoreComplete) {
        val selectOH = PriorityEncoderOH(remainingStoreComplete)
        val selected = Mux1H(selectOH, io.storeComplete.map(_.bits))

        storeCompletePacked(s).valid := selectOH.orR
        storeCompletePacked(s).bits := selected.robPtr

        remainingStoreComplete = remainingStoreComplete & (~selectOH).asUInt
    }

    writeback.io.storeComplete := storeCompletePacked

    rob.io.wb := writeback.io.robComplete
    writeback.io.lsqLive := lsqAlloc.io.writebackLiveState

    val blockRobHeadLoad = WireDefault(false.B)
    io.robHeadLoad.valid :=
        rob.io.status.headValid &&
            rob.io.status.headIsLoad &&
            !blockRobHeadLoad
    io.robHeadLoad.bits.robPtr := rob.io.status.headRobPtr
    io.robHeadLoad.bits.waiting := !rob.io.status.headComplete

    io.commit := rob.io.cmt
    io.storeCommit := rob.io.storeCommit
    rename.io.commit := rob.io.rnmCmt

    io.llCommit := 0.U.asTypeOf(io.llCommit)
    for (lane <- 0 until ncmt) {
        when(rob.io.cmt(lane).valid && rob.io.cmt(lane).bits.uop === opLL) {
            // Commit lanes are oldest to youngest; the last matching lane has
            // the architecturally newest reservation after a wide retirement.
            io.llCommit.valid := true.B
            io.llCommit.bits := rob.io.cmt(lane).bits.robPtr
        }
    }

    val csrCommitNow = rob.io.boundaryCommit
    val csrCommitEventValid = RegNext(csrCommitNow, false.B)
    // The payload is observable only with csrCommitEventValid. Sample it every
    // cycle so the rare boundary decision does not become a wide register CE.
    val csrCommitEventBits = RegNext(rob.io.status.headRobPtr)
    csrCommitEvent.valid := csrCommitEventValid
    csrCommitEvent.bits := csrCommitEventBits

    if (enableCommitDebug) {
        for (i <- 0 until ncmt) {
            io.commitData.get(i) := rob.io.cmt(i).bits.data
        }
    }

    for (i <- 0 until ncmt) {
        lsqAlloc.io.ldqRelease(i) :=
            rob.io.cmt(i).valid && rob.io.cmt(i).bits.isLoad
    }
    lsqAlloc.io.stqFreed := io.stqFreed
    lsqAlloc.io.stqCommitPtr := io.stqCommitPtr
    lsqAlloc.io.stqCommittedMask := io.stqCommittedMask

    val ftqRetireValidNow = rob.io.cmt.map(_.valid).reduce(_ || _)
    val ftqRetireBitsNow = WireDefault(
        0.U.asTypeOf(new CPUSTC.frontend.FtqRetire)
    )

    for (i <- 0 until ncmt) {
        when(rob.io.cmt(i).valid) {
            ftqRetireBitsNow.ptr    := rob.io.cmt(i).bits.ftqPtr
            ftqRetireBitsNow.offset := rob.io.cmt(i).bits.ftqOffset
        }
        when(rob.io.cmt(i).valid && rob.io.cmt(i).bits.ftqLast) {
            ftqRetireBitsNow.completed.valid := true.B
            ftqRetireBitsNow.completed.bits  := rob.io.cmt(i).bits.ftqPtr
        }
    }

    // ROB commit is already a registered event describing instructions that
    // retired in C0. Preserve it across a same-cycle architectural redirect.
    io.ftqRetire.valid := ftqRetireValidNow
    io.ftqRetire.bits  := ftqRetireBitsNow

    object TrapState extends ChiselEnum {
        val Idle, IntrArmed, IntrCapture, SyncQueued, IntrQueued, InFlight = Value
    }

    val trapState = RegInit(TrapState.Idle)
    val trapStateBusy = trapState =/= TrapState.Idle
    val trapInFlight = trapState === TrapState.InFlight
    // IntrArmed drains through a real retirement boundary. Once that boundary
    // has retired, stop a new uncached head from issuing while its PC is
    // captured for ERA; otherwise the new head could acquire an MMIO side
    // effect that the ensuing architectural flush would replay.
    blockRobHeadLoad :=
        trapStateBusy && trapState =/= TrapState.IntrArmed
    val csrCommitInFlight = RegInit(false.B)
    val synchronousTrap = rob.io.trap.valid

    val sysHeadGrantReg = RegInit(
        0.U.asTypeOf(Valid(new RobPtr))
    )
    csrPort.io.sysHeadGrant := sysHeadGrantReg

    val rawSysHeadGrant =
        csrPort.io.sysHeadReq.valid &&
        rob.io.status.headValid &&
        !rob.io.status.headComplete &&
        (csrPort.io.sysHeadReq.bits.asUInt ===
            rob.io.status.headRobPtr.asUInt) &&
        !synchronousTrap &&
        !trapStateBusy &&
        !csrCommitInFlight &&
        !io.hardRedirect.valid

    // CSRUnit already registers its redirect. Send that narrow event directly
    // to the frontend, but let only a one-bit local copy reach the wide
    // Backend state. This preserves the existing state-clear cycle while
    // removing the CSR target/valid cone from rename, dispatch and allocation.
    val csrRedirectAccepted = WireDefault(
        0.U.asTypeOf(Valid(UInt(CPUSTC.config.RegisterFile.dataWidth.W)))
    )
    csrRedirectAccepted.valid :=
        csrPort.io.csrArchRedirect.valid &&
        !io.hardRedirect.valid
    csrRedirectAccepted.bits := csrPort.io.csrArchRedirect.bits

    val rareRecoveryRedirect = csrRedirectAccepted
    for (port <- 0 until memNissue) {
        architecturalRequestKill(port) := csrRedirectAccepted.valid
    }

    val rareStateFlushReg = RegNext(rareRecoveryRedirect.valid, false.B)
    val immediateStateFlush = io.hardRedirect.valid
    val backendStateFlush = immediateStateFlush || rareStateFlushReg

    // Keep rare recovery off the global module-boundary flush net.  Every copy
    // samples the same event as rareStateFlushReg, so the visible flush cycle is
    // unchanged; the immediate hard redirect remains combinational by design.
    def rareFlushCopy(name: String): Bool = {
        val copy = RegNext(rareRecoveryRedirect.valid, false.B)
        copy.suggestName(name)
        dontTouch(copy)
        copy
    }
    val renameRareFlushReg = rareFlushCopy("renameRareFlushReg")
    val dispatchRareFlushReg = rareFlushCopy("dispatchRareFlushReg")
    val memoryRareFlushReg = rareFlushCopy("memoryRareFlushReg")
    val lsqAllocRareFlushReg = rareFlushCopy("lsqAllocRareFlushReg")
    val robRareFlushReg = rareFlushCopy("robRareFlushReg")
    val intIqRareFlushReg = rareFlushCopy("intIqRareFlushReg")
    val memIqRareFlushReg = rareFlushCopy("memIqRareFlushReg")
    val regReadRareFlushReg = rareFlushCopy("regReadRareFlushReg")
    val writebackRareFlushReg = rareFlushCopy("writebackRareFlushReg")
    val intPortRareFlushRegs = Seq.tabulate(intNissue) { port =>
        rareFlushCopy(s"intPort${port}RareFlushReg")
    }
    val memPortRareFlushRegs = Seq.tabulate(memNissue) { port =>
        rareFlushCopy(s"memPort${port}RareFlushReg")
    }

    control.io.resolve      := intPorts(0).io.branchResolve
    control.io.hardRedirect := io.hardRedirect
    control.io.archRedirect :=
        0.U.asTypeOf(control.io.archRedirect)
    control.io.squashPending := rareRecoveryRedirect.valid

    val rawControlFlush = control.io.fullFlush
    val rareRedirectBits = WireDefault(
        0.U.asTypeOf(new CPUSTC.backend.control.PipelineRedirect)
    )
    rareRedirectBits.kind := CPUSTC.backend.control.RedirectKind.HARD
    rareRedirectBits.target := rareRecoveryRedirect.bits

    io.redirect.valid := rareRecoveryRedirect.valid || control.io.redirect.valid
    io.redirect.bits := Mux(
        rareRecoveryRedirect.valid,
        rareRedirectBits,
        control.io.redirect.bits
    )
    io.fullFlush := backendStateFlush
    io.memoryStateFlush := immediateStateFlush || memoryRareFlushReg

    val fullFlush = backendStateFlush

    when(fullFlush) {
        sysHeadGrantReg.valid := false.B
    }.otherwise {
        sysHeadGrantReg.valid := rawSysHeadGrant
        when(rawSysHeadGrant) {
            sysHeadGrantReg.bits := csrPort.io.sysHeadReq.bits
        }
    }

    val branchUpdateCopies = control.io.branchUpdateCopies

    require(io.storeException.length == nStoreComplete)

    // Store address exceptions are rare and must retain SQ-generation and
    // recovery checks. Terminate the raw CPUSTC.memory event at an unconditional
    // register, then validate only registered state on this independent slow
    // path. Clean Store completion never enters this sidecar.
    val storeExceptionS0Valid = RegInit(
        VecInit.fill(nStoreComplete)(false.B)
    )
    val storeExceptionS0Bits = Reg(
        Vec(nStoreComplete, new CPUSTC.memory.StoreExceptionEvent)
    )
    val storeExceptionS0LiveValid = RegInit(0.U(nstq.W))
    val storeExceptionS0LiveHigh = RegInit(0.U(nstq.W))
    val storeExceptionS0RecoveryKill = RegInit(0.U(nstq.W))
    val storeExceptionS0Flush = RegInit(false.B)

    for (i <- 0 until nStoreComplete) {
        storeExceptionS0Valid(i) := io.storeException(i).valid
        storeExceptionS0Bits(i) := io.storeException(i).bits
    }
    storeExceptionS0LiveValid :=
        lsqAlloc.io.writebackLiveState.stqValidMask
    storeExceptionS0LiveHigh :=
        lsqAlloc.io.writebackLiveState.stqHighMask
    storeExceptionS0RecoveryKill := lsqAlloc.io.stqRecoveryKillMask
    storeExceptionS0Flush := fullFlush

    val storeExceptionS1Valid = RegInit(
        VecInit.fill(nStoreComplete)(false.B)
    )
    val storeExceptionS1Bits = Reg(
        Vec(nStoreComplete, new RobExceptionWriteback)
    )

    for (i <- 0 until nStoreComplete) {
        val event = storeExceptionS0Bits(i)
        val aliveAtCapture =
            CPUSTC.memory.MemoryPointerUtils.pointerAlive(
                event.sqindex,
                event.sqindexHigh,
                storeExceptionS0LiveValid,
                storeExceptionS0LiveHigh
            )
        val killedAtCapture =
            (event.sqindex & storeExceptionS0RecoveryKill).orR
        val killedNow =
            (event.sqindex & lsqAlloc.io.stqRecoveryKillMask).orR

        storeExceptionS1Valid(i) :=
            storeExceptionS0Valid(i) &&
                aliveAtCapture &&
                !killedAtCapture &&
                !storeExceptionS0Flush &&
                !killedNow &&
                !fullFlush
        storeExceptionS1Bits(i).robPtr := event.robPtr
        storeExceptionS1Bits(i).cause := event.cause
        storeExceptionS1Bits(i).badvValid := event.badvValid
        storeExceptionS1Bits(i).badv := event.badv

        rob.io.storeException(i).valid := storeExceptionS1Valid(i)
        rob.io.storeException(i).bits := storeExceptionS1Bits(i)

        when(storeExceptionS0Valid(i)) {
            assert(PopCount(event.sqindex) === 1.U)
            assert(event.cause =/= CPUSTC.config.MemoryException.EXC_NONE)
        }
        when(storeExceptionS1Valid(i)) {
            assert(storeExceptionS1Bits(i).cause =/=
                CPUSTC.config.MemoryException.EXC_NONE)
        }
    }

    assert(
        rawControlFlush === immediateStateFlush
    )
    assert(io.memoryStateFlush === io.fullFlush)
    val branchRecoveryFlush =
        control.io.redirect.valid && !rawControlFlush
    writeback.io.loadRecovery := branchRecoveryFlush
    assert(
        branchRecoveryFlush ===
            (control.io.branchUpdate.valid &&
                control.io.branchUpdate.bits.mispredictMask.orR)
    )
    when(csrRedirectAccepted.valid) {
        assert(!immediateStateFlush)
        assert(!rareStateFlushReg)
        assert(!rawControlFlush)
        assert(io.redirect.valid)
        assert(
            io.redirect.bits.kind === CPUSTC.backend.control.RedirectKind.HARD
        )
        assert(
            io.redirect.bits.target ===
                csrPort.io.csrArchRedirect.bits
        )
    }
    when(io.hardRedirect.valid) {
        assert(backendStateFlush)
        assert(control.io.redirect.bits.target === io.hardRedirect.bits)
    }
    val interruptEligible =
        csrPort.io.csrInterruptPending &&
        !csrPort.io.csrBusy

    // Raw ROB/CSR observations terminate at narrow sampling registers. The
    // exception payload and interrupt PC are intentionally written every cycle
    // so capture arbitration cannot be absorbed into their CE/SR pins.
    val synchronousTrapSeen = RegInit(false.B)
    val interruptEligibleSeen = RegInit(false.B)
    val interruptIdleBoundarySeen = RegInit(false.B)
    val interruptUseIdlePc = RegInit(false.B)
    val csrBusySeen = RegNext(csrPort.io.csrBusy, false.B)
    val synchronousTrapPayload = Reg(new CSRExceptionInfo)
    val interruptHeadPcSample = Reg(UInt(CPUSTC.config.RegisterFile.dataWidth.W))
    val interruptIdlePcSample = Reg(UInt(CPUSTC.config.RegisterFile.dataWidth.W))

    synchronousTrapPayload.err_pc := rob.io.trap.bits.pc
    synchronousTrapPayload.instr := rob.io.trap.bits.instr
    synchronousTrapPayload.ecode := rob.io.trap.bits.exceptionCause(5, 0)
    // The existing 8-bit cause keeps the common ECODE in bits 5:0 and carries
    // ADE's subcode in bits 7:6. Decode it only after the ROB trap boundary.
    synchronousTrapPayload.esubcode := Cat(
        0.U(7.W),
        rob.io.trap.bits.exceptionCause(7, 6)
    )
    synchronousTrapPayload.badvValid := rob.io.trap.bits.exceptionBadvValid
    synchronousTrapPayload.badv := rob.io.trap.bits.exceptionBadv
    interruptHeadPcSample := rob.io.status.headPc
    interruptIdlePcSample := csrPort.io.csrIdleResumePc
    dontTouch(synchronousTrapPayload)
    dontTouch(interruptHeadPcSample)
    dontTouch(interruptIdlePcSample)

    val trapCaptureAvailable =
        !csrCommitInFlight &&
        !csrBusySeen
    val launchSynchronousTrap = trapState === TrapState.SyncQueued
    val launchInterrupt = trapState === TrapState.IntrQueued
    val trapLaunchAvailable =
        !csrCommitInFlight &&
        !csrBusySeen
    val launchArchitecturalTrap =
        (launchSynchronousTrap || launchInterrupt) &&
        trapLaunchAvailable &&
        !fullFlush

    when(fullFlush) {
        trapState := TrapState.Idle
        synchronousTrapSeen := false.B
        interruptEligibleSeen := false.B
        interruptIdleBoundarySeen := false.B
        interruptUseIdlePc := false.B
    }.otherwise {
        synchronousTrapSeen := synchronousTrap
        interruptEligibleSeen := interruptEligible
        when(interruptEligible && csrPort.io.csrIdle) {
            interruptIdleBoundarySeen := true.B
        }

        switch(trapState) {
            is(TrapState.Idle) {
                // LoongArch requires an eligible interrupt to win when it and
                // a synchronous exception meet at the same precise boundary.
                // The exception instruction remains at the ROB head and is
                // retried after ERTN from the interrupt handler.
                when(interruptEligibleSeen && trapCaptureAvailable) {
                    trapState := TrapState.IntrArmed
                    interruptUseIdlePc := false.B
                }.elsewhen(synchronousTrapSeen && trapCaptureAvailable) {
                    trapState := TrapState.SyncQueued
                    interruptIdleBoundarySeen := false.B
                }.elsewhen(!interruptEligible && !interruptEligibleSeen) {
                    interruptIdleBoundarySeen := false.B
                }
            }
            is(TrapState.IntrArmed) {
                when(csrCommitInFlight || csrBusySeen) {
                    trapState := TrapState.Idle
                    interruptIdleBoundarySeen := false.B
                }.elsewhen(!interruptEligibleSeen) {
                    trapState := TrapState.Idle
                    interruptIdleBoundarySeen := false.B
                }.elsewhen(interruptIdleBoundarySeen) {
                    trapState := TrapState.IntrQueued
                    interruptUseIdlePc := true.B
                    interruptIdleBoundarySeen := false.B
                }.elsewhen(synchronousTrapSeen || rob.io.retireMask.orR) {
                    // headComplete is only an execution result. A completed
                    // MMIO load must actually retire before the interrupt can
                    // use the following instruction as its precise boundary.
                    trapState := TrapState.IntrCapture
                    interruptUseIdlePc := false.B
                }
            }
            is(TrapState.IntrCapture) {
                when(csrCommitInFlight || csrBusySeen) {
                    trapState := TrapState.Idle
                    interruptIdleBoundarySeen := false.B
                }.elsewhen(!interruptEligibleSeen) {
                    trapState := TrapState.Idle
                    interruptIdleBoundarySeen := false.B
                }.elsewhen(csrPort.io.csrIdle) {
                    trapState := TrapState.IntrQueued
                    interruptUseIdlePc := true.B
                    interruptIdleBoundarySeen := false.B
                }.elsewhen(rob.io.status.headValid) {
                    // Retirement is blocked in IntrCapture. Sampling for one
                    // full cycle therefore records the first unretired PC.
                    trapState := TrapState.IntrQueued
                    interruptUseIdlePc := false.B
                }
            }
            is(TrapState.SyncQueued) {
                when(trapLaunchAvailable) {
                    trapState := TrapState.InFlight
                }
            }
            is(TrapState.IntrQueued) {
                when(trapLaunchAvailable) {
                    trapState := TrapState.InFlight
                }
            }
            is(TrapState.InFlight) {
                trapState := TrapState.InFlight
            }
        }
    }

    csrExceptionEvent.valid := launchArchitecturalTrap
    csrExceptionEvent.bits := synchronousTrapPayload
    when(launchInterrupt) {
        csrExceptionEvent.bits.err_pc := Mux(
            interruptUseIdlePc,
            interruptIdlePcSample,
            interruptHeadPcSample
        )
        csrExceptionEvent.bits.instr := 0.U
        csrExceptionEvent.bits.ecode := ExpCode.INT
        csrExceptionEvent.bits.esubcode := 0.U
        csrExceptionEvent.bits.badvValid := false.B
        csrExceptionEvent.bits.badv := 0.U
    }
    io.exceptionTrace := csrExceptionEvent

    rob.io.commitBlock :=
        (trapStateBusy && trapState =/= TrapState.IntrArmed) ||
        csrCommitInFlight

    when(trapState === TrapState.IntrCapture) {
        assert(!rob.io.retireMask.orR)
        assert(!io.robHeadLoad.valid)
    }

    when(fullFlush || csrPort.io.csrCommitDone) {
        csrCommitInFlight := false.B
    }.elsewhen(csrCommitNow) {
        csrCommitInFlight := true.B
    }

    when(csrCommitNow) {
        assert(csrPort.io.csrBusy)
        assert(!csrCommitInFlight)
    }
    when(csrCommitEvent.valid) {
        assert(csrCommitInFlight)
        assert(csrPort.io.csrBusy)
    }
    when(csrPort.io.csrCommitDone) {
        assert(csrCommitInFlight)
    }
    when(csrCommitInFlight && !fullFlush) {
        assert(!rob.io.retireMask.orR)
    }

    // Give each IntIQ entry a physically independent copy of the registered
    // ROB head. This preserves the CSR eligibility cycle while avoiding one
    // long, high-fanout head-to-all-entry comparison network.
    val robHeadValidRegs = RegInit(VecInit.fill(intNiq)(false.B))
    val robHeadBitsRegs = Reg(Vec(intNiq, new RobPtr))
    val robHeadValidNext = rob.io.status.headValid && !fullFlush
    val robHeadGeneration = RegInit(false.B)
    val robHeadSnapshotChanged =
        (robHeadValidNext =/= robHeadValidRegs(0)) ||
            (robHeadValidNext && robHeadValidRegs(0) &&
                (rob.io.status.headRobPtr.asUInt =/=
                    robHeadBitsRegs(0).asUInt))

    when(robHeadSnapshotChanged) {
        robHeadGeneration := !robHeadGeneration
    }

    for (i <- 0 until intNiq) {
        robHeadValidRegs(i) := robHeadValidNext
        when(robHeadValidNext) {
            robHeadBitsRegs(i) := rob.io.status.headRobPtr
        }
        intIq.io.robHead(i).valid := robHeadValidRegs(i)
        intIq.io.robHead(i).bits  := robHeadBitsRegs(i)
    }
    intIq.io.robHeadGeneration := robHeadGeneration
    dontTouch(robHeadValidRegs)
    dontTouch(robHeadBitsRegs)

    for (i <- 1 until intNiq) {
        assert(robHeadValidRegs(i) === robHeadValidRegs(0))
        when(robHeadValidRegs(0)) {
            assert(robHeadBitsRegs(i).asUInt === robHeadBitsRegs(0).asUInt)
        }
    }

    io.branchRecoveryFlush := branchRecoveryFlush
    io.idle := csrPort.io.csrIdle
    io.lsqLive      := lsqAlloc.io.liveState
    io.stqHeadCurrent := lsqAlloc.io.stqHeadCurrent
    io.ldqFlushMask := lsqAlloc.io.ldqFlushMask
    io.stqFlushMask := lsqAlloc.io.stqFlushMask

    when(csrPort.io.csrArchRedirect.valid) {
        assert(trapInFlight || csrCommitInFlight)
    }

    assert(PopCount(VecInit(io.storeComplete.map(_.valid))) <= nStoreComplete.U)
    for (i <- io.storeComplete.indices; j <- i + 1 until io.storeComplete.length) {
        when(io.storeComplete(i).valid && io.storeComplete(j).valid) {
            assert(
                io.storeComplete(i).bits.robPtr.asUInt =/=
                    io.storeComplete(j).bits.robPtr.asUInt
            )
        }
    }

    rename.io.branchUpdate :=
        branchUpdateCopies(BranchUpdateDomain.Rename)
    lsqAlloc.io.branchUpdate :=
        branchUpdateCopies(BranchUpdateDomain.DispatchLsq)
    rob.io.branchUpdate :=
        branchUpdateCopies(BranchUpdateDomain.Rob)
    intIq.io.branchUpdate :=
        branchUpdateCopies(BranchUpdateDomain.IntIssue)
    memIq.io.branchUpdate :=
        branchUpdateCopies(BranchUpdateDomain.MemIssue)
    val intClusterBranchUpdate =
        branchUpdateCopies(BranchUpdateDomain.IntCluster)
    regRead.io.branchUpdate := intClusterBranchUpdate
    writeback.io.branchUpdate := intClusterBranchUpdate

    val dispatchBranchMispredict =
        branchUpdateCopies(BranchUpdateDomain.DispatchLsq).valid &&
        branchUpdateCopies(BranchUpdateDomain.DispatchLsq).bits.mispredictMask.orR
    dispatch.io.branchMispredict := dispatchBranchMispredict

    rename.io.flush    := immediateStateFlush || renameRareFlushReg
    dispatch.io.flush  := immediateStateFlush || dispatchRareFlushReg
    lsqAlloc.io.flush  := immediateStateFlush || lsqAllocRareFlushReg
    rob.io.flush       := immediateStateFlush || robRareFlushReg
    intIq.io.flush     := immediateStateFlush || intIqRareFlushReg
    memIq.io.flush     := immediateStateFlush || memIqRareFlushReg
    regRead.io.flush   := immediateStateFlush || regReadRareFlushReg
    writeback.io.flush := immediateStateFlush || writebackRareFlushReg

    for (p <- 0 until intNissue) {
        intPorts(p).io.flush :=
            immediateStateFlush || intPortRareFlushRegs(p)
        intPorts(p).io.branchUpdate := intClusterBranchUpdate
        intPorts(p).io.counterValue := io.counterValue
    }

    val branchRecovery =
        intClusterBranchUpdate.valid &&
        intClusterBranchUpdate.bits.mispredictMask.orR
    when(!branchRecovery) {
        assert(p2RawFixedAccepted === p2FixedAccepted)
        for (w <- 0 until nDataWb) {
            assert(intIqWakeup(w).valid === issueWakeup(w).valid)
            when(issueWakeup(w).valid) {
                assert(
                    intIqWakeup(w).bits.pdest === issueWakeup(w).bits.pdest
                )
            }
        }
    }

    for (p <- 0 until memNissue) {
        memPorts(p).io.flush :=
            immediateStateFlush || memPortRareFlushRegs(p)
        memPorts(p).io.branchUpdate :=
            branchUpdateCopies(BranchUpdateDomain.MemExecute)
    }

    when(rareRecoveryRedirect.valid || rareStateFlushReg) {
        assert(!rob.io.retireMask.orR)
        assert(!io.storeCommit.valid)
        assert(!VecInit(io.memRequest.map(_.fire)).asUInt.orR)
    }

    when(rename.io.flush || dispatch.io.flush) {
        assert(!dispatch.io.fire.asUInt.orR)
    }

    when(launchArchitecturalTrap) {
        assert(!rob.io.retireMask.orR)
        // A CSR that retired in the previous cycle reaches CSRUnit through
        // the delayed commit event in this cycle.  CSRUnit is still
        // combinationally busy while that final commit is being accepted,
        // but it is idle before the registered exception event is consumed.
        // Do not reject a precise trap in this legal handoff cycle.
        assert(
            !csrPort.io.csrBusy ||
            csrCommitEvent.valid ||
            csrPort.io.csrResultFire
        )
    }

    when(launchInterrupt) {
        assert(!launchSynchronousTrap)
    }

    if (enablePerfCounters) {
        val perf = io.perf.get
        val dispatchActive = rename.io.out.map(_.valid).reduce(_ || _)
        val dispatchFired  = dispatch.io.fire.asUInt.orR
        val needLsq = dispatch.io.lsq.req.map(_.valid).reduce(_ || _)
        val needIntIq = dispatch.io.intIq.reqCountOH =/= 1.U
        val needMemIq = dispatch.io.memIq.reqCountOH =/= 1.U

        val loadAllocatedValid = RegInit(VecInit.fill(nldq)(false.B))
        val loadAllocatedHigh  = Reg(Vec(nldq, Bool()))
        val loadAllocatedRobPtr = Reg(Vec(nldq, new CPUSTC.backend.rob.RobPtr))
        val loadIssuedValid = RegInit(VecInit.fill(nldq)(false.B))
        val loadIssuedHigh  = Reg(Vec(nldq, Bool()))
        val loadResultValid = RegInit(VecInit.fill(nldq)(false.B))
        val loadResultHigh  = Reg(Vec(nldq, Bool()))

        for (index <- 0 until nldq) {
            val allocatedHere = VecInit((0 until ndcd).map { lane =>
                dispatch.io.fire(lane) &&
                    dispatch.io.lsq.req(lane).bits.isLoad &&
                    lsqAlloc.io.dispatch.resp(lane).bits.ldqIdx.oh(index)
            })
            val issuedHere = VecInit((0 until memNissue).map { port =>
                loadRequestFire(port) && loadRequestLdindex(port)(index)
            })
            val resultHere = VecInit((0 until nLoadWb).map { port =>
                io.loadResult(port).valid &&
                    io.loadResult(port).bits.inst.uop.isLD &&
                    io.loadResult(port).bits.inst.ldindex(index)
            })

            val issueHigh = Mux1H(issuedHere, loadRequestLdindexHigh)
            val resultHigh = Mux1H(resultHere, io.loadResult.map(_.bits.inst.ldindexHigh))
            val allocatedHigh = Mux1H(
                allocatedHere,
                lsqAlloc.io.dispatch.resp.map(_.bits.ldqIdx.flag)
            )
            val allocatedRobPtr = Mux1H(allocatedHere, rob.io.enq.resp.map(_.bits))
            val slotAlive = lsqAlloc.io.liveState.ldqValidMask(index)

            when(fullFlush) {
                loadAllocatedValid(index) := false.B
                loadIssuedValid(index) := false.B
                loadResultValid(index) := false.B
            }.otherwise {
                when(!slotAlive) {
                    loadAllocatedValid(index) := false.B
                    loadIssuedValid(index) := false.B
                    loadResultValid(index) := false.B
                }
                when(allocatedHere.asUInt.orR) {
                    loadAllocatedValid(index) := true.B
                    loadAllocatedHigh(index)  := allocatedHigh
                    loadAllocatedRobPtr(index) := allocatedRobPtr
                    loadIssuedValid(index) := false.B
                    loadResultValid(index) := false.B
                }
                when(issuedHere.asUInt.orR) {
                    loadIssuedValid(index) := true.B
                    loadIssuedHigh(index)  := issueHigh
                    loadResultValid(index) := false.B
                }
                when(resultHere.asUInt.orR) {
                    loadIssuedValid(index) := true.B
                    loadIssuedHigh(index)  := resultHigh
                    loadResultValid(index) := true.B
                    loadResultHigh(index)  := resultHigh
                }
            }

            assert(PopCount(allocatedHere) <= 1.U)
            assert(PopCount(issuedHere) <= 1.U)
            assert(PopCount(resultHere) <= 1.U)
        }

        val robHeadWaiting = rob.io.status.headValid && !rob.io.status.headComplete
        val robHeadWaitingLoad = robHeadWaiting && rob.io.status.headIsLoad
        def sameRobPtr(a: CPUSTC.backend.rob.RobPtr, b: CPUSTC.backend.rob.RobPtr): Bool =
            a.qidx === b.qidx &&
                a.offset === b.offset &&
                a.high === b.high &&
                a.epoch === b.epoch

        val headLdqOH = VecInit((0 until nldq).map { index =>
            loadAllocatedValid(index) &&
                sameRobPtr(loadAllocatedRobPtr(index), rob.io.status.headRobPtr)
        }).asUInt
        val headLdqAlive = VecInit((0 until nldq).map { index =>
            headLdqOH(index) &&
                lsqAlloc.io.liveState.ldqValidMask(index) &&
                lsqAlloc.io.liveState.ldqHighMask(index) === loadAllocatedHigh(index)
        }).asUInt.orR
        val headLoadIssued = VecInit((0 until nldq).map { index =>
            headLdqOH(index) &&
                loadIssuedValid(index) &&
                loadIssuedHigh(index) === loadAllocatedHigh(index)
        }).asUInt.orR
        val headLoadResult = VecInit((0 until nldq).map { index =>
            headLdqOH(index) &&
                loadResultValid(index) &&
                loadResultHigh(index) === loadAllocatedHigh(index)
        }).asUInt.orR

        val headLoadPreIssue =
            robHeadWaitingLoad && headLdqAlive && !headLoadIssued && !headLoadResult
        val headLoadWaitResult =
            robHeadWaitingLoad && headLdqAlive && headLoadIssued && !headLoadResult
        val headLoadWaitRob =
            robHeadWaitingLoad && headLdqAlive && headLoadResult
        val headLoadUntracked = robHeadWaitingLoad && (
            !headLdqAlive ||
                (headLoadResult && !headLoadIssued)
        )

        perf.decodeValidCount := PopCount(VecInit(io.decode.map(_.valid)))
        perf.decodeFireCount  := PopCount(VecInit(io.decode.map(_.fire)))

        perf.renameBlockedOutput := rename.io.status.outputBlocked
        perf.renameBlockedFree   := rename.io.status.freeBlocked
        perf.renameBlockedTag    := rename.io.status.tagBlocked

        perf.dispatchValid     := dispatchActive
        perf.dispatchFireCount := PopCount(dispatch.io.fire)
        perf.dispatchRobBlocked := dispatchActive && !dispatchFired && !rob.io.enq.canAccept
        perf.dispatchLsqBlocked := dispatchActive && !dispatchFired &&
            needLsq && !lsqAlloc.io.dispatch.canAccept
        perf.dispatchIntBlocked := dispatchActive && !dispatchFired &&
            needIntIq && !intIq.io.dispatch.canAccept
        perf.dispatchMemBlocked := dispatchActive && !dispatchFired &&
            needMemIq && !memIq.io.dispatch.canAccept
        perf.robEmpty     := rob.io.empty
        perf.robFull      := rob.io.full
        perf.robOccupancy := rob.io.occupancy
        perf.robHeadWaitLoad   := robHeadWaiting && rob.io.status.headIsLoad
        perf.robHeadWaitStore  := robHeadWaiting && rob.io.status.headIsStore
        perf.robHeadWaitBranch := robHeadWaiting && rob.io.status.headIsBranch
        perf.robHeadWaitInt := robHeadWaiting &&
            !rob.io.status.headIsLoad &&
            !rob.io.status.headIsStore &&
            !rob.io.status.headIsBranch
        perf.robHeadLoadPreIssue   := headLoadPreIssue
        perf.robHeadLoadWaitResult := headLoadWaitResult
        perf.robHeadLoadWaitRob    := headLoadWaitRob
        perf.robHeadLoadUntracked  := headLoadUntracked
        perf.p0HandoffEvent := rob.io.status.p0Handoff.event
        perf.p0HandoffCommitBlocked := rob.io.status.p0Handoff.commitBlocked
        perf.p0HandoffNoNext := rob.io.status.p0Handoff.noNext
        perf.p0HandoffNextReady := rob.io.status.p0Handoff.nextReady
        perf.p0HandoffNextWaitInt := rob.io.status.p0Handoff.nextWaitInt
        perf.p0HandoffNextWaitLoad := rob.io.status.p0Handoff.nextWaitLoad
        perf.p0HandoffNextWaitStore := rob.io.status.p0Handoff.nextWaitStore
        perf.p0HandoffNextWaitBranch := rob.io.status.p0Handoff.nextWaitBranch
        perf.p0HandoffNextWaitOther := rob.io.status.p0Handoff.nextWaitOther
        perf.p0HandoffRetireOne := rob.io.status.p0Handoff.retireOne
        perf.p0HandoffRetireWide := rob.io.status.p0Handoff.retireWide
        perf.p0HandoffRetireDeferred := rob.io.status.p0Handoff.retireDeferred

        perf.loadPredWakeCount := PopCount(VecInit(io.loadPredWake.map(_.valid)))
        perf.loadPredSuccessCount := PopCount(VecInit(io.loadPredResolve.map { event =>
            event.valid && event.bits.success
        }))
        perf.loadPredCancelCount := PopCount(VecInit(io.loadPredResolve.map { event =>
            event.valid && !event.bits.success
        }))
        perf.intLoadPredIssueCount := intIq.io.loadPredIssueCount
        perf.memLoadPredIssueCount := memIq.io.loadPredIssueCount
        perf.loadToLoadPredIssueCount := memIq.io.loadPredLoadIssueCount

        perf.commitBlockedStore := false.B
        perf.commitBlockedControl := false.B

        perf.intIqOccupancy := PopCount(intIq.io.validMask)
        perf.intIqFull      := intIq.io.full
        perf.intIqNoReady   := intIq.io.validMask.orR && !intIq.io.canIssueMask.orR
        perf.intIssueCount  := PopCount(VecInit(intIq.io.issue.map(_.fire)))
        perf.intIssueStallCount := PopCount(VecInit(intIq.io.issue.map { port =>
            port.valid && !port.ready
        }))

        perf.memIqOccupancy := PopCount(memIq.io.validMask)
        perf.memIqFull      := memIq.io.full
        perf.memIqNoReady   := memIq.io.validMask.orR && !memIq.io.canIssueMask.orR
        perf.memIssueCount  := PopCount(VecInit(memIq.io.issue.map(_.fire)))
        perf.memIssueStallCount := PopCount(VecInit(memIq.io.issue.map { port =>
            port.valid && !port.ready
        }))

        perf.ldqOccupancy := PopCount(lsqAlloc.io.liveState.ldqValidMask)
        perf.stqOccupancy := PopCount(lsqAlloc.io.liveState.stqValidMask)
        perf.ldqFull      := lsqAlloc.io.liveState.ldqValidMask.andR
        perf.stqFull      := lsqAlloc.io.liveState.stqValidMask.andR

        perf.memRequestCount :=
            PopCount(VecInit(io.memRequest.map(_.fire))) +&
                PopCount(VecInit(io.directCachedLoad.map(_.fire)))
        perf.memRequestStallCount :=
            PopCount(VecInit(io.memRequest.map { port =>
                port.valid && !port.ready
            })) +& PopCount(VecInit(io.directCachedLoad.map { port =>
                port.valid && !port.ready
            }))
        perf.loadRequestCount := PopCount(loadRequestFire)
        perf.staRequestCount := PopCount(VecInit(io.memRequest.map { port =>
            port.fire && port.bits.uop.isSTA
        }))
        perf.stdRequestCount := PopCount(VecInit(io.memRequest.map { port =>
            port.fire && port.bits.uop.isSTD
        }))
        perf.loadResultCount := PopCount(VecInit(io.loadResult.map(_.valid)))

        for (port <- 0 until memNissue) {
            perf.loadStart(port).valid := loadRequestFire(port)
            perf.loadStart(port).bits.indexOH := loadRequestLdindex(port)
            perf.loadStart(port).bits.high := loadRequestLdindexHigh(port)

            perf.loadDone(port).valid := io.loadResult(port).valid
            perf.loadDone(port).bits.indexOH := io.loadResult(port).bits.inst.ldindex
            perf.loadDone(port).bits.high    := io.loadResult(port).bits.inst.ldindexHigh
        }

        val branchResolve = control.io.resolve
        val branchMispredict = branchResolve.valid && branchResolve.bits.mispredict
        val branchIsCond = branchResolve.bits.cfiType === CFI_BR
        val branchIsJirl = branchResolve.bits.cfiType === CFI_JIRL

        perf.branchResolve := branchResolve.valid
        perf.branchMispredict := branchMispredict
        perf.branchDirectionWrong := branchMispredict && branchResolve.bits.directionWrong
        perf.branchTargetWrong := branchMispredict && branchResolve.bits.targetWrong
        perf.branchCondResolve := branchResolve.valid && branchIsCond
        perf.branchCondMispredict := branchMispredict && branchIsCond
        perf.branchJirlResolve := branchResolve.valid && branchIsJirl
        perf.branchJirlMispredict := branchMispredict && branchIsJirl
        perf.branchActualTaken := branchResolve.valid && branchResolve.bits.actualTaken
        perf.branchPredTaken := branchResolve.valid && branchResolve.bits.predTaken
        perf.fullFlush := fullFlush

        when(robHeadWaitingLoad) {
            assert(PopCount(headLdqOH) === 1.U)
            assert(PopCount(VecInit(Seq(
                headLoadPreIssue,
                headLoadWaitResult,
                headLoadWaitRob,
                headLoadUntracked
            ))) === 1.U)
        }
    }
}
