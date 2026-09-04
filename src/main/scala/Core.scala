package CPUSTC

import chisel3._
import chisel3.util._

import CPUSTC.backend.{Backend, CSRDebugState, CSRExceptionInfo}
import CPUSTC.config.Commit._
import CPUSTC.config.Decode._
import CPUSTC.config.EXEOp._
import CPUSTC.config.Fetch._
import CPUSTC.config.JumpOp._
import CPUSTC.config.Issue._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.config.RegisterFile._
import CPUSTC.config.WritebackConfig._
import CPUSTC.backend.control.PipelineRedirect
import CPUSTC.decode.Decode
import CPUSTC.frontend.Frontend
import CPUSTC.predict.BPU
import CPUSTC.perf.CorePerformanceMonitor
import CPUSTC.backend.rob.RobCommitEntry
import CPUSTC.backend.execute.CounterDebugEvent
import CPUSTC.memory.{
    DcacheConfig,
    IcacheConfig,
    LoadQueueConfig,
    LoadStateTableConfig,
    MemSysConfig,
    MemorySubSystem,
    StoreQueueConfig,
    LoadDebugEvent,
    StoreCommitTrace,
    TlbFillDebugEvent
}
import CPUSTC.memory.external.AXIIO

class CPUSTCoreIO(enableCommitDebug: Boolean = false) extends Bundle {
    val hardRedirect = Flipped(Valid(UInt(dataWidth.W)))
    val hardwareInterrupt = Input(UInt(8.W))

    val axi = new AXIIO

    val commitTrace   = Output(Vec(ncmt, Valid(new RobCommitEntry)))
    val commitData = if (enableCommitDebug) {
        Some(Output(Vec(ncmt, UInt(dataWidth.W))))
    } else {
        None
    }
    val redirectTrace = Output(Valid(new PipelineRedirect))
    val exceptionTrace = Output(Valid(new CSRExceptionInfo))
    val csrDebugState = Output(new CSRDebugState)
    val csrDebugErtn = Output(Bool())
    val csrDebugInterrupt = Output(UInt(11.W))
    val llbitDebugClear = Output(Bool())
    val loadDebug = Output(Vec(nLoadWb, Valid(new LoadDebugEvent)))
    val storeDebug = Output(Valid(new StoreCommitTrace))
    val counterDebug = Output(Vec(intNissue, Valid(new CounterDebugEvent)))
    val tlbFillDebug = Output(Valid(new TlbFillDebugEvent))
    val npc           = Output(UInt(32.W))
}

class CPUSTCore(
    useBlackBoxRam: Boolean = true,
    enableCommitDebug: Boolean = false,
    enablePerfCounters: Boolean = false,
    maxCommitPerCycle: Int = ncmt,
    perfMeasurementPcs: Option[(BigInt, BigInt)] = None,
    perfMeasurementByTimer: Boolean = false,
    perfMeasurementPcInstrs: Option[((BigInt, BigInt), (BigInt, BigInt))] = None,
    memSysConfig: MemSysConfig = MemSysConfig()
) extends Module {
    require(maxCommitPerCycle > 0 && maxCommitPerCycle <= ncmt)
    require(
        perfMeasurementPcs.isEmpty ||
        (!perfMeasurementByTimer && perfMeasurementPcInstrs.isEmpty)
    )

    val io = IO(new CPUSTCoreIO(enableCommitDebug))

    require(nfch == IcacheConfig.nfetch)
    require(memNissue == LoadQueueConfig.EnqNum)
    require(nLoadWb == DcacheConfig.nPorts)
    require(nldq == LoadStateTableConfig.length)
    require(nstq == StoreQueueConfig.length)

    val frontend = Module(new Frontend(
        enablePerfCounters = enablePerfCounters,
        useBlackBoxRam = useBlackBoxRam
    ))
    val bpu      = Module(new BPU(useBlackBoxRam = useBlackBoxRam))
    val decode   = Module(new Decode)
    val backend  = Module(new Backend(
        enableCommitDebug = enableCommitDebug,
        enablePerfCounters = enablePerfCounters,
        maxCommitPerCycle = maxCommitPerCycle,
        memSysConfig = memSysConfig
    ))
    val memory   = Module(new MemorySubSystem(
        useBlackBoxRam = useBlackBoxRam,
        enablePerfCounters = enablePerfCounters,
        memSysConfig = memSysConfig
    ))

    val cycleCounter = RegInit(0.U(64.W))
    cycleCounter := cycleCounter + 1.U

    if (enablePerfCounters) {
        val perf = Module(new CorePerformanceMonitor(
            gatedMeasurement =
                perfMeasurementByTimer ||
                perfMeasurementPcs.nonEmpty ||
                perfMeasurementPcInstrs.nonEmpty
        ))
        perf.io.cycle := cycleCounter
        perf.io.commitCount := PopCount(VecInit(backend.io.commit.map(_.valid)))
        if (perfMeasurementByTimer || perfMeasurementPcInstrs.nonEmpty) {
            val timerReads = VecInit(backend.io.commit.map { commit =>
                perfMeasurementByTimer.B &&
                commit.valid &&
                commit.bits.uop === opRDCNTVLW
            })
            val timerMarker = timerReads.asUInt.orR
            val pcInstrStart = WireDefault(false.B)
            val pcInstrStop  = WireDefault(false.B)

            perfMeasurementPcInstrs.foreach {
                case ((startPc, startInstr), (stopPc, stopInstr)) =>
                    Seq(startPc, startInstr, stopPc, stopInstr).foreach { value =>
                        require(value >= 0 && value < (BigInt(1) << 32))
                    }
                    pcInstrStart := backend.io.commit.map { commit =>
                        commit.valid &&
                        commit.bits.pc === startPc.U(32.W) &&
                        commit.bits.instr === startInstr.U(32.W)
                    }.reduce(_ || _)
                    pcInstrStop := backend.io.commit.map { commit =>
                        commit.valid &&
                        commit.bits.pc === stopPc.U(32.W) &&
                        commit.bits.instr === stopInstr.U(32.W)
                    }.reduce(_ || _)
            }

            val measurementActive = RegInit(false.B)
            val measurementStart = pcInstrStart || (timerMarker && !measurementActive)
            val measurementStop  = pcInstrStop || (timerMarker && measurementActive)

            perf.io.measurementStart := measurementStart
            perf.io.measurementStop  := measurementStop

            when(measurementStart) {
                measurementActive := true.B
            }.elsewhen(measurementStop) {
                measurementActive := false.B
            }

            assert(PopCount(timerReads) <= 1.U)
            assert(!(measurementStart && measurementStop))
        } else {
            perfMeasurementPcs match {
                case Some((startPc, stopPc)) =>
                    require(startPc >= 0 && startPc < (BigInt(1) << 32))
                    require(stopPc >= 0 && stopPc < (BigInt(1) << 32))
                    perf.io.measurementStart := backend.io.commit.map { commit =>
                        commit.valid && commit.bits.pc === startPc.U(32.W)
                    }.reduce(_ || _)
                    perf.io.measurementStop := backend.io.commit.map { commit =>
                        commit.valid && commit.bits.pc === stopPc.U(32.W)
                    }.reduce(_ || _)
                case None =>
                    perf.io.measurementStart := false.B
                    perf.io.measurementStop := false.B
            }
        }
        perf.io.frontend := frontend.io.perf.get
        perf.io.backend  := backend.io.perf.get
        perf.io.memory   := memory.io.perf.get
    }

    frontend.io.redirect := backend.io.redirect
    frontend.io.idle := backend.io.idle

    decode.io.flush := backend.io.fullFlush || backend.io.branchRecoveryFlush
    decode.io.in <> frontend.io.ibuffer

    for (i <- 0 until ndcd) {
        backend.io.decode(i).valid := decode.io.out(i).valid
        backend.io.decode(i).bits  := decode.io.out(i).bits
        decode.io.outReady(i)      := backend.io.decode(i).ready
    }

    backend.io.hardRedirect := io.hardRedirect
    backend.io.hardwareInterrupt := io.hardwareInterrupt
    backend.io.counterValue := cycleCounter
    backend.io.llbitValue := memory.io.llbitValue

    frontend.io.ftqReadPtr := 0.U.asTypeOf(frontend.io.ftqReadPtr)
    frontend.io.ftqPredictionReadReq := backend.io.ftqPredictionReadReq
    backend.io.ftqPredictionReadResp := frontend.io.ftqPredictionReadResp

    frontend.io.ftqRetire := backend.io.ftqRetire

    memory.io.commitStore := backend.io.storeCommit

    io.commitTrace   := backend.io.commit
    if (enableCommitDebug) {
        io.commitData.get := backend.io.commitData.get
    }
    io.redirectTrace := backend.io.redirect
    io.exceptionTrace := backend.io.exceptionTrace
    io.csrDebugState := backend.io.csrDebugState
    io.csrDebugErtn := backend.io.csrDebugErtn
    io.csrDebugInterrupt := backend.io.csrDebugInterrupt
    io.llbitDebugClear := backend.io.llbitClear
    io.storeDebug := memory.io.storeCommitTrace
    io.counterDebug := backend.io.counterDebug
    io.tlbFillDebug := memory.io.tlbFillDebug
    io.loadDebug := backend.io.loadDebug
    io.npc           := frontend.io.npc

    bpu.io.ifu.req.valid := frontend.io.bpu.req.valid
    bpu.io.ifu.req.bits  := frontend.io.bpu.req.bits
    frontend.io.bpu.req.ready := bpu.io.ifu.req.ready

    frontend.io.bpu.resp.valid   := bpu.io.ifu.resp.valid
    frontend.io.bpu.resp.bits.pc := bpu.io.ifu.resp.bits.pc
    frontend.io.bpu.resp.bits.lookupId := bpu.io.ifu.resp.bits.lookupId
    frontend.io.bpu.resp.bits.history := bpu.io.ifu.resp.bits.history
    frontend.io.bpu.resp.bits.longHistory :=
        bpu.io.ifu.resp.bits.longHistory
    for (i <- 0 until nfch) {
        frontend.io.bpu.resp.bits.taken(i)     := bpu.io.ifu.resp.bits.taken(i)
        frontend.io.bpu.resp.bits.pretarget(i) := bpu.io.ifu.resp.bits.pretarget(i)
        frontend.io.bpu.resp.bits.predType(i)  := bpu.io.ifu.resp.bits.predType(i)
        frontend.io.bpu.resp.bits.btbMeta(i)   := bpu.io.ifu.resp.bits.btbMeta(i)
    }
    frontend.io.bpu.auxResp := bpu.io.ifu.auxResp
    bpu.io.ifu.kill := frontend.io.icache.flush

    bpu.io.pd.isBr         := frontend.io.bpuPredecode.brMask.asBools
    bpu.io.pd.jumpEn       := frontend.io.bpuPredecode.takenMask.asBools
    bpu.io.pd.baseIsBr     := frontend.io.bpuPredecode.baseBrMask.asBools
    bpu.io.pd.baseJumpEn   := frontend.io.bpuPredecode.baseTakenMask.asBools
    bpu.io.pd.predType     := frontend.io.bpuPredecode.rasType
    bpu.io.pd.pc           := frontend.io.bpuPredecode.rasPc
    bpu.io.pd.history      := frontend.io.bpuPredecode.history
    bpu.io.pd.longHistory  := frontend.io.bpuPredecode.longHistory
    bpu.io.pd.historyRepair := frontend.io.bpuPredecode.historyRepair
    bpu.io.pd.baseHistoryRepair :=
        frontend.io.bpuPredecode.baseHistoryRepair
    bpu.io.pd.deferredHistoryRepair :=
        frontend.io.bpuPredecode.deferredHistoryRepair
    bpu.io.pd.targetOnlyRepair := frontend.io.bpuPredecode.targetOnlyRepair
    bpu.io.pd.flush        := frontend.io.bpuPredecode.repair
    bpu.io.pdStall         := !frontend.io.bpuPredecode.valid

    bpu.io.cmt.train <> frontend.io.bpuTrain
    bpu.io.cmt.flush := backend.io.redirect.valid
    bpu.io.cmt.historyRecovery := frontend.io.bpuHistoryRecovery
    bpu.io.cmt.longHistoryRecovery := frontend.io.bpuLongHistoryRecovery

    memory.io.icache.req.valid   := frontend.io.icache.req.valid
    memory.io.icache.req.bits.pc := frontend.io.icache.req.bits.pc
    memory.io.icache.req.bits.mask := frontend.io.icache.req.bits.mask
    frontend.io.icache.req.ready := memory.io.icache.req.ready

    frontend.io.icache.resp.valid       := memory.io.icache.resp.valid
    frontend.io.icache.resp.bits.pc     := memory.io.icache.resp.bits.pc
    frontend.io.icache.resp.bits.instrs := memory.io.icache.resp.bits.instrs
    frontend.io.icache.resp.bits.exceptions := memory.io.icache.resp.bits.exceptionCauses
    frontend.io.icache.resp.bits.normal := memory.io.icache.resp.bits.normal
    frontend.io.icache.resp.bits.mask   := memory.io.icache.resp.bits.mask
    memory.io.icache.resp.ready         := frontend.io.icache.resp.ready
    frontend.io.icache.miss             := false.B

    for (i <- 0 until memNissue) {
        memory.io.backendInst(i) <> backend.io.memRequest(i)
        memory.io.directCachedLoad(i) <> backend.io.directCachedLoad(i)
    }
    memory.io.sysMemCmd.valid := backend.io.sysMemCmd.valid
    memory.io.sysMemCmd.bits := backend.io.sysMemCmd.bits
    backend.io.sysMemCmd.ready := memory.io.sysMemCmd.ready
    backend.io.sysMemResp.valid := memory.io.sysMemResp.valid
    backend.io.sysMemResp.bits := memory.io.sysMemResp.bits
    memory.io.sysMemResp.ready := backend.io.sysMemResp.ready
    memory.io.addressState := backend.io.addressState
    memory.io.llCommit := backend.io.llCommit
    memory.io.llbitClear := backend.io.llbitClear
    backend.io.loadResult <> memory.io.loadResult
    backend.io.loadPredWake := memory.io.loadPredWake
    backend.io.loadPredResolve := memory.io.loadPredResolve
    backend.io.storeComplete := memory.io.storeComplete
    backend.io.storeException := memory.io.storeException

    val stqFreedReg = RegNext(
        memory.io.sqFreedMask,
        0.U.asTypeOf(memory.io.sqFreedMask)
    )
    backend.io.stqFreed := stqFreedReg
    backend.io.stqCommitPtr.oh := memory.io.sqCommitPtrOH
    backend.io.stqCommitPtr.flag := memory.io.sqCommitPtrHigh
    backend.io.stqCommittedMask := memory.io.sqCommittedMask

    memory.io.sqHeadOH   := backend.io.stqHeadCurrent.oh
    memory.io.sqHeadHigh := backend.io.stqHeadCurrent.flag

    val memoryLoadPtrCtrlInit = WireDefault(
        0.U.asTypeOf(new CPUSTC.memory.backend.DispatchPtrCtrl)
    )
    memoryLoadPtrCtrlInit.nextHeadPtr := 1.U
    memoryLoadPtrCtrlInit.nextHeadSuffixMask :=
        Fill(LoadStateTableConfig.length, true.B)
    memoryLoadPtrCtrlInit.nextTailPtr := 1.U

    val memoryLoadPtrCtrl = RegInit(memoryLoadPtrCtrlInit)

    memoryLoadPtrCtrl.nextHeadPtr     := backend.io.lsqLive.ldqHead.oh
    memoryLoadPtrCtrl.nextHeadSuffixMask := VecInit.tabulate(
        LoadStateTableConfig.length
    ) { index =>
        backend.io.lsqLive.ldqHead.oh(index, 0).orR
    }.asUInt
    memoryLoadPtrCtrl.nextTailPtr     := backend.io.lsqLive.ldqTail.oh
    memoryLoadPtrCtrl.nextHeadPtrHigh := backend.io.lsqLive.ldqHead.flag
    memoryLoadPtrCtrl.nextTailPtrHigh := backend.io.lsqLive.ldqTail.flag
    memoryLoadPtrCtrl.flushMask := backend.io.ldqFlushMask
    memoryLoadPtrCtrl.redirect := backend.io.branchRecoveryFlush

    memory.io.loadPtrCtrl := memoryLoadPtrCtrl

    // Memory-side liveness is sampled at the clock boundary. Branch rollback
    // still reaches CPUSTC.memory before any redirected-path request can arrive,
    // while ROB commit ready cannot feed back through LSQ release masks.
    val memoryLsqLiveInit = WireDefault(0.U.asTypeOf(new CPUSTC.memory.MemoryLsqLiveState))
    memoryLsqLiveInit.stqTailOH := 1.U
    val memoryLsqLive = RegInit(memoryLsqLiveInit)
    memoryLsqLive.ldqValidMask := backend.io.lsqLive.ldqValidMask
    memoryLsqLive.ldqHighMask  := backend.io.lsqLive.ldqHighMask
    memoryLsqLive.stqValidMask := backend.io.lsqLive.stqValidMask
    memoryLsqLive.stqHighMask  := backend.io.lsqLive.stqHighMask
    memoryLsqLive.stqTailOH    := backend.io.lsqLive.stqTail.oh
    memoryLsqLive.stqTailHigh  := backend.io.lsqLive.stqTail.flag
    memory.io.lsqLive := memoryLsqLive
    val memorySqFlushMask = RegNext(backend.io.stqFlushMask, 0.U)
    memory.io.sqFlushMask := memorySqFlushMask
    memory.io.robHeadLoad := backend.io.robHeadLoad

    // Branch recovery and frontend repair remain same-cycle. Architectural
    // redirects are rare; register only that class before it enters the wide
    // ICache/MSHR/L2 cancellation network.
    val memoryHardRedirect = RegNext(
        backend.io.redirect.valid &&
            backend.io.redirect.bits.kind === CPUSTC.backend.control.RedirectKind.HARD,
        false.B
    )
    memory.io.icacheFlush :=
        backend.io.branchRecoveryFlush ||
            frontend.io.localICacheFlush ||
            memoryHardRedirect
    memory.io.icacheRedirect := backend.io.branchRecoveryFlush
    // Architectural recovery still has identical cycle semantics, but the
    // memory-local registered copy keeps the shared Backend flush source out of
    // the DCache-result-to-IQ wakeup cone.
    memory.io.backendFlush := backend.io.memoryStateFlush
    memory.io.loadRecovery := backend.io.branchRecoveryFlush

    io.axi <> memory.io.axi

}
