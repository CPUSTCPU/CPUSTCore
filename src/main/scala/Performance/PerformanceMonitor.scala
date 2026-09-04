package CPUSTC.perf

import chisel3._
import chisel3.util._

import CPUSTC.config.Commit._
import CPUSTC.config.Decode._
import CPUSTC.config.Fetch._
import CPUSTC.config.Issue._
import CPUSTC.config.LoadStoreQueue._
import CPUSTC.predict.AdvancedPredictorPerfEvents

class LoadPerfEvent extends Bundle {
    val indexOH = UInt(nldq.W)
    val high    = Bool()
}

class FrontendPerfEvents extends Bundle {
    val icacheReqFire   = Bool()
    val icacheReqStall  = Bool()
    val icacheRespFire  = Bool()
    val fetchBundleFire = Bool()
    val fetchInstrCount = UInt(log2Ceil(nfch + 1).W)

    val decodeValidCount = UInt(log2Ceil(ndcd + 1).W)
    val decodeFireCount  = UInt(log2Ceil(ndcd + 1).W)

    val ibufferEmpty = Bool()
    val ibufferFull  = Bool()
    val ftqFull      = Bool()

    val bpuReqFire    = Bool()
    val bpuResp       = Bool()
    val bpuHitCount   = UInt(log2Ceil(nfch + 1).W)
    val bpuTakenCount = UInt(log2Ceil(nfch + 1).W)

    val predictedRedirect = Bool()
    val earlyRedirect     = Bool()
    val lateRedirect      = Bool()
    val predecodeRepair    = Bool()
    val advancedPredictor  = new AdvancedPredictorPerfEvents
}

class BackendPerfEvents extends Bundle {
    val decodeValidCount = UInt(log2Ceil(ndcd + 1).W)
    val decodeFireCount  = UInt(log2Ceil(ndcd + 1).W)

    val renameBlockedOutput = Bool()
    val renameBlockedFree   = Bool()
    val renameBlockedTag    = Bool()

    val dispatchValid     = Bool()
    val dispatchFireCount = UInt(log2Ceil(ndcd + 1).W)
    val dispatchRobBlocked = Bool()
    val dispatchLsqBlocked = Bool()
    val dispatchIntBlocked = Bool()
    val dispatchMemBlocked = Bool()

    val robEmpty     = Bool()
    val robFull      = Bool()
    val robOccupancy = UInt(log2Ceil(nrob + 1).W)
    val robHeadWaitInt    = Bool()
    val robHeadWaitLoad   = Bool()
    val robHeadWaitStore  = Bool()
    val robHeadWaitBranch = Bool()
    val robHeadLoadPreIssue   = Bool()
    val robHeadLoadWaitResult = Bool()
    val robHeadLoadWaitRob    = Bool()
    val robHeadLoadUntracked  = Bool()
    val p0HandoffEvent = Bool()
    val p0HandoffCommitBlocked = Bool()
    val p0HandoffNoNext = Bool()
    val p0HandoffNextReady = Bool()
    val p0HandoffNextWaitInt = Bool()
    val p0HandoffNextWaitLoad = Bool()
    val p0HandoffNextWaitStore = Bool()
    val p0HandoffNextWaitBranch = Bool()
    val p0HandoffNextWaitOther = Bool()
    val p0HandoffRetireOne = Bool()
    val p0HandoffRetireWide = Bool()
    val p0HandoffRetireDeferred = Bool()
    val commitBlockedStore   = Bool()
    val commitBlockedControl = Bool()

    val loadPredWakeCount    = UInt(log2Ceil(memNissue + 1).W)
    val loadPredSuccessCount = UInt(log2Ceil(memNissue + 1).W)
    val loadPredCancelCount  = UInt(log2Ceil(memNissue + 1).W)
    val intLoadPredIssueCount = UInt(log2Ceil(intNissue + 1).W)
    val memLoadPredIssueCount = UInt(log2Ceil(memNissue + 1).W)
    val loadToLoadPredIssueCount = UInt(log2Ceil(memNissue + 1).W)

    val intIqOccupancy = UInt(log2Ceil(intNiq + 1).W)
    val intIqFull      = Bool()
    val intIqNoReady   = Bool()
    val intIssueCount  = UInt(log2Ceil(intNissue + 1).W)
    val intIssueStallCount = UInt(log2Ceil(intNissue + 1).W)

    val memIqOccupancy = UInt(log2Ceil(memNiq + 1).W)
    val memIqFull      = Bool()
    val memIqNoReady   = Bool()
    val memIssueCount  = UInt(log2Ceil(memNissue + 1).W)
    val memIssueStallCount = UInt(log2Ceil(memNissue + 1).W)

    val ldqOccupancy = UInt(log2Ceil(nldq + 1).W)
    val stqOccupancy = UInt(log2Ceil(nstq + 1).W)
    val ldqFull      = Bool()
    val stqFull      = Bool()

    val memRequestCount      = UInt(log2Ceil(memNissue + 1).W)
    val memRequestStallCount = UInt(log2Ceil(memNissue + 1).W)
    val loadRequestCount     = UInt(log2Ceil(memNissue + 1).W)
    val staRequestCount      = UInt(log2Ceil(memNissue + 1).W)
    val stdRequestCount      = UInt(log2Ceil(memNissue + 1).W)
    val loadResultCount      = UInt(log2Ceil(memNissue + 1).W)

    val loadStart = Vec(memNissue, Valid(new LoadPerfEvent))
    val loadDone  = Vec(memNissue, Valid(new LoadPerfEvent))

    val branchResolve    = Bool()
    val branchMispredict = Bool()
    val branchDirectionWrong = Bool()
    val branchTargetWrong    = Bool()
    val branchCondResolve    = Bool()
    val branchCondMispredict = Bool()
    val branchJirlResolve    = Bool()
    val branchJirlMispredict = Bool()
    val branchActualTaken    = Bool()
    val branchPredTaken      = Bool()
    val fullFlush        = Bool()
}

class ICachePerfEvents extends Bundle {
    val hit  = Bool()
    val miss = Bool()
}

class DCachePerfEvents extends Bundle {
    val loadHitCount        = UInt(log2Ceil(memNissue + 1).W)
    val loadNormalMissCount = UInt(log2Ceil(memNissue + 1).W)
    val loadMshrFullCount   = UInt(log2Ceil(memNissue + 1).W)
    val storeHitCount       = UInt(log2Ceil(memNissue + 1).W)
    val storeMissCount      = UInt(log2Ceil(memNissue + 1).W)
    val storeRetryCount     = UInt(log2Ceil(nstq + 1).W)
    val storeMshrFullRetryCount = Bool()
    val storeRefillRetryCount   = Bool()
    val storeEntranceRetryCount = Bool()
}

class MemoryPerfEvents extends Bundle {
    val icacheHit  = Bool()
    val icacheMiss = Bool()

    val dcacheLoadHitCount  = UInt(log2Ceil(memNissue + 1).W)
    val dcacheLoadMissCount = UInt(log2Ceil(memNissue + 1).W)
    val dcacheMshrFullCount = UInt(log2Ceil(memNissue + 1).W)
    val dcacheStoreHitCount = UInt(log2Ceil(memNissue + 1).W)
    val dcacheStoreMissCount = UInt(log2Ceil(memNissue + 1).W)
    val dcacheStoreRetryCount = UInt(log2Ceil(nstq + 1).W)
    val dcacheStoreMshrFullRetryCount = Bool()
    val dcacheStoreRefillRetryCount = Bool()
    val dcacheStoreEntranceRetryCount = Bool()

    val l2IReadHit = Bool()
    val l2IReadMiss = Bool()
    val l2DReadHit = Bool()
    val l2DReadMiss = Bool()
    val l2WriteHit = Bool()
    val l2WriteMiss = Bool()
    val l2DirtyWriteback = Bool()
    val l2UncacheRead = Bool()
    val l2UncacheWrite = Bool()
    val l2Busy = Bool()

    val loadIssueCount      = UInt(log2Ceil(memNissue + 1).W)
    val loadForwardCount    = UInt(log2Ceil(memNissue + 1).W)
    val storeDataReplayCount = UInt(log2Ceil(memNissue + 1).W)
    val partialOverlapReplayCount = UInt(log2Ceil(memNissue + 1).W)
    val waitStoreDataReplayCount = UInt(log2Ceil(memNissue + 1).W)
    val staleResultCount    = UInt(log2Ceil(memNissue + 1).W)

    val headLoadTracked = Bool()
    val headLoadMissing = Bool()
    val headLoadReadyIssued = Bool()
    val headLoadReadyWait = Bool()
    val headLoadExecuting = Bool()
    val headLoadExecutingAfterStoreReplay = Bool()
    val headLoadWaitMshrFull = Bool()
    val headLoadWaitStoreData = Bool()
    val headLoadComplete = Bool()
    val headLoadOther = Bool()
    val headLoadResultNow = Bool()

    val lstOccupancy   = UInt(log2Ceil(nldq + 1).W)
    val lstFull        = Bool()
    val storeBufferFull = Bool()
}

class CorePerformanceMonitorIO extends Bundle {
    val cycle       = Input(UInt(64.W))
    val commitCount = Input(UInt(log2Ceil(ncmt + 1).W))
    val measurementStart = Input(Bool())
    val measurementStop  = Input(Bool())

    val frontend = Input(new FrontendPerfEvents)
    val backend  = Input(new BackendPerfEvents)
    val memory   = Input(new MemoryPerfEvents)

    val report = Output(Bool())
    val totalCycles       = Output(UInt(64.W))
    val totalCommits      = Output(UInt(64.W))
    val totalMispredicts  = Output(UInt(64.W))
    val totalLoadSamples  = Output(UInt(64.W))
    val totalLoadLatency  = Output(UInt(64.W))
    val maxLoadLatency    = Output(UInt(64.W))
}

class CorePerformanceMonitor(
    reportInterval: Int = 10000,
    gatedMeasurement: Boolean = false
) extends Module {
    require(reportInterval > 0)

    val io = IO(new CorePerformanceMonitorIO)

    private val reportWidth = math.max(1, log2Ceil(reportInterval))
    val reportCounter = RegInit(0.U(reportWidth.W))
    val intervalReportNow = reportCounter === (reportInterval - 1).U
    val measurementActive = RegInit(false.B)
    val measurementStart = if (gatedMeasurement) io.measurementStart else false.B
    val measurementStop = if (gatedMeasurement) io.measurementStop else false.B
    val countEnable = if (gatedMeasurement) {
        measurementActive && !measurementStop
    } else {
        true.B
    }
    val reportNow = if (gatedMeasurement) measurementStop else intervalReportNow

    if (gatedMeasurement) {
        reportCounter := 0.U
        when(measurementStart) {
            assert(!measurementActive, "Performance measurement started twice")
            measurementActive := true.B
        }
        when(measurementStop) {
            assert(measurementActive, "Performance measurement stopped before start")
            measurementActive := false.B
        }
    } else {
        reportCounter := Mux(reportNow, 0.U, reportCounter + 1.U)
    }
    io.report := reportNow

    val totalCycles      = RegInit(0.U(64.W))
    val totalCommits     = RegInit(0.U(64.W))
    val totalMispredicts = RegInit(0.U(64.W))

    when(measurementStart) {
        totalCycles := 0.U
        totalCommits := 0.U
        totalMispredicts := 0.U
    }.elsewhen(countEnable) {
        totalCycles  := totalCycles + 1.U
        totalCommits := totalCommits + io.commitCount
        when(io.backend.branchMispredict) {
            totalMispredicts := totalMispredicts + 1.U
        }
    }

    io.totalCycles      := totalCycles
    io.totalCommits     := totalCommits
    io.totalMispredicts := totalMispredicts

    def windowCounter(amount: UInt): UInt = {
        val value = RegInit(0.U(64.W))
        val increment = Mux(countEnable, amount, 0.U)
        val next = Mux(measurementStart, 0.U, value + increment)
        value := Mux(reportNow, 0.U, next)
        next
    }

    def event(flag: Bool): UInt = flag.asUInt

    val measuredCycles = windowCounter(1.U)
    val commitInst = windowCounter(io.commitCount)
    val commitHist = (0 to ncmt).map { count =>
        windowCounter(event(io.commitCount === count.U))
    }

    val feReq       = windowCounter(event(io.frontend.icacheReqFire))
    val feReqStall  = windowCounter(event(io.frontend.icacheReqStall))
    val feResp      = windowCounter(event(io.frontend.icacheRespFire))
    val feBundles   = windowCounter(event(io.frontend.fetchBundleFire))
    val feInstrs    = windowCounter(io.frontend.fetchInstrCount)
    val feDecodeValid = windowCounter(io.frontend.decodeValidCount)
    val feDecodeFire  = windowCounter(io.frontend.decodeFireCount)
    val ibufEmpty   = windowCounter(event(io.frontend.ibufferEmpty))
    val ibufFull    = windowCounter(event(io.frontend.ibufferFull))
    val ftqFull     = windowCounter(event(io.frontend.ftqFull))
    val bpuReq      = windowCounter(event(io.frontend.bpuReqFire))
    val bpuResp     = windowCounter(event(io.frontend.bpuResp))
    val bpuHits     = windowCounter(io.frontend.bpuHitCount)
    val bpuTaken    = windowCounter(io.frontend.bpuTakenCount)
    val feRedirect  = windowCounter(event(io.frontend.predictedRedirect))
    val earlyRedirect = windowCounter(event(io.frontend.earlyRedirect))
    val lateRedirect = windowCounter(event(io.frontend.lateRedirect))
    val predecodeRepair = windowCounter(event(io.frontend.predecodeRepair))
    val miniEligible = windowCounter(event(
        io.frontend.advancedPredictor.miniEligible
    ))
    val miniDisagree = windowCounter(event(
        io.frontend.advancedPredictor.miniDisagree
    ))
    val miniRecover = windowCounter(event(
        io.frontend.advancedPredictor.miniRecover
    ))
    val miniHarm = windowCounter(event(
        io.frontend.advancedPredictor.miniHarm
    ))
    val miniWrong = windowCounter(event(
        io.frontend.advancedPredictor.miniWrong
    ))
    val miniProvider0 = windowCounter(event(
        io.frontend.advancedPredictor.miniEligible &&
            io.frontend.advancedPredictor.miniProvider === 1.U
    ))
    val miniProvider1 = windowCounter(event(
        io.frontend.advancedPredictor.miniEligible &&
            io.frontend.advancedPredictor.miniProvider === 2.U
    ))
    val miniProvider2 = windowCounter(event(
        io.frontend.advancedPredictor.miniEligible &&
            io.frontend.advancedPredictor.miniProvider === 3.U
    ))
    val miniProvider3 = windowCounter(event(
        io.frontend.advancedPredictor.miniEligible &&
            io.frontend.advancedPredictor.miniProvider === 4.U
    ))
    val loopEligible = windowCounter(event(
        io.frontend.advancedPredictor.loopEligible
    ))
    val loopDisagree = windowCounter(event(
        io.frontend.advancedPredictor.loopDisagree
    ))
    val loopRecover = windowCounter(event(
        io.frontend.advancedPredictor.loopRecover
    ))
    val loopHarm = windowCounter(event(
        io.frontend.advancedPredictor.loopHarm
    ))
    val loopWrong = windowCounter(event(
        io.frontend.advancedPredictor.loopWrong
    ))

    val renameOutBlocked = windowCounter(event(io.backend.renameBlockedOutput))
    val renameFreeBlocked = windowCounter(event(io.backend.renameBlockedFree))
    val renameTagBlocked = windowCounter(event(io.backend.renameBlockedTag))
    val dispatchValid = windowCounter(event(io.backend.dispatchValid))
    val dispatchFire  = windowCounter(io.backend.dispatchFireCount)
    val dispatchRobBlocked = windowCounter(event(io.backend.dispatchRobBlocked))
    val dispatchLsqBlocked = windowCounter(event(io.backend.dispatchLsqBlocked))
    val dispatchIntBlocked = windowCounter(event(io.backend.dispatchIntBlocked))
    val dispatchMemBlocked = windowCounter(event(io.backend.dispatchMemBlocked))
    val robEmpty = windowCounter(event(io.backend.robEmpty))
    val robFull  = windowCounter(event(io.backend.robFull))
    val robOccupancySum = windowCounter(io.backend.robOccupancy)
    val robHeadWaitInt = windowCounter(event(io.backend.robHeadWaitInt))
    val robHeadWaitLoad = windowCounter(event(io.backend.robHeadWaitLoad))
    val robHeadWaitStore = windowCounter(event(io.backend.robHeadWaitStore))
    val robHeadWaitBranch = windowCounter(event(io.backend.robHeadWaitBranch))
    val commitBlockedStore = windowCounter(event(io.backend.commitBlockedStore))
    val commitBlockedControl = windowCounter(event(io.backend.commitBlockedControl))
    val robHeadLoadPreIssue = windowCounter(event(io.backend.robHeadLoadPreIssue))
    val robHeadLoadWaitResult = windowCounter(event(io.backend.robHeadLoadWaitResult))
    val robHeadLoadWaitRob = windowCounter(event(io.backend.robHeadLoadWaitRob))
    val robHeadLoadUntracked = windowCounter(event(io.backend.robHeadLoadUntracked))
    val p0HandoffEvent = windowCounter(event(io.backend.p0HandoffEvent))
    val p0HandoffCommitBlocked =
        windowCounter(event(io.backend.p0HandoffCommitBlocked))
    val p0HandoffNoNext = windowCounter(event(io.backend.p0HandoffNoNext))
    val p0HandoffNextReady =
        windowCounter(event(io.backend.p0HandoffNextReady))
    val p0HandoffNextWaitInt =
        windowCounter(event(io.backend.p0HandoffNextWaitInt))
    val p0HandoffNextWaitLoad =
        windowCounter(event(io.backend.p0HandoffNextWaitLoad))
    val p0HandoffNextWaitStore =
        windowCounter(event(io.backend.p0HandoffNextWaitStore))
    val p0HandoffNextWaitBranch =
        windowCounter(event(io.backend.p0HandoffNextWaitBranch))
    val p0HandoffNextWaitOther =
        windowCounter(event(io.backend.p0HandoffNextWaitOther))
    val p0HandoffRetireOne =
        windowCounter(event(io.backend.p0HandoffRetireOne))
    val p0HandoffRetireWide =
        windowCounter(event(io.backend.p0HandoffRetireWide))
    val p0HandoffRetireDeferred =
        windowCounter(event(io.backend.p0HandoffRetireDeferred))
    val loadPredWake = windowCounter(io.backend.loadPredWakeCount)
    val loadPredSuccess = windowCounter(io.backend.loadPredSuccessCount)
    val loadPredCancel = windowCounter(io.backend.loadPredCancelCount)
    val intLoadPredIssue = windowCounter(io.backend.intLoadPredIssueCount)
    val memLoadPredIssue = windowCounter(io.backend.memLoadPredIssueCount)
    val loadToLoadPredIssue = windowCounter(io.backend.loadToLoadPredIssueCount)

    val intIqOccupancySum = windowCounter(io.backend.intIqOccupancy)
    val intIqFull = windowCounter(event(io.backend.intIqFull))
    val intIqNoReady = windowCounter(event(io.backend.intIqNoReady))
    val intIssue = windowCounter(io.backend.intIssueCount)
    val intIssueStall = windowCounter(io.backend.intIssueStallCount)
    val memIqOccupancySum = windowCounter(io.backend.memIqOccupancy)
    val memIqFull = windowCounter(event(io.backend.memIqFull))
    val memIqNoReady = windowCounter(event(io.backend.memIqNoReady))
    val memIssue = windowCounter(io.backend.memIssueCount)
    val memIssueStall = windowCounter(io.backend.memIssueStallCount)
    val ldqOccupancySum = windowCounter(io.backend.ldqOccupancy)
    val stqOccupancySum = windowCounter(io.backend.stqOccupancy)
    val ldqFull = windowCounter(event(io.backend.ldqFull))
    val stqFull = windowCounter(event(io.backend.stqFull))

    val memReq = windowCounter(io.backend.memRequestCount)
    val memReqStall = windowCounter(io.backend.memRequestStallCount)
    val loadReq = windowCounter(io.backend.loadRequestCount)
    val staReq = windowCounter(io.backend.staRequestCount)
    val stdReq = windowCounter(io.backend.stdRequestCount)
    val loadResult = windowCounter(io.backend.loadResultCount)

    val branchResolve = windowCounter(event(io.backend.branchResolve))
    val branchMispredict = windowCounter(event(io.backend.branchMispredict))
    val branchDirectionWrong = windowCounter(event(io.backend.branchDirectionWrong))
    val branchTargetWrong = windowCounter(event(io.backend.branchTargetWrong))
    val branchCondResolve = windowCounter(event(io.backend.branchCondResolve))
    val branchCondMispredict = windowCounter(event(io.backend.branchCondMispredict))
    val branchJirlResolve = windowCounter(event(io.backend.branchJirlResolve))
    val branchJirlMispredict = windowCounter(event(io.backend.branchJirlMispredict))
    val branchActualTaken = windowCounter(event(io.backend.branchActualTaken))
    val branchPredTaken = windowCounter(event(io.backend.branchPredTaken))
    val fullFlush = windowCounter(event(io.backend.fullFlush))

    val icacheHit = windowCounter(event(io.memory.icacheHit))
    val icacheMiss = windowCounter(event(io.memory.icacheMiss))
    val dcacheHit = windowCounter(io.memory.dcacheLoadHitCount)
    val dcacheMiss = windowCounter(io.memory.dcacheLoadMissCount)
    val dcacheMshrFull = windowCounter(io.memory.dcacheMshrFullCount)
    val dcacheStoreHit = windowCounter(io.memory.dcacheStoreHitCount)
    val dcacheStoreMiss = windowCounter(io.memory.dcacheStoreMissCount)
    val dcacheStoreRetry = windowCounter(io.memory.dcacheStoreRetryCount)
    val dcacheStoreMshrFullRetry = windowCounter(event(io.memory.dcacheStoreMshrFullRetryCount))
    val dcacheStoreRefillRetry = windowCounter(event(io.memory.dcacheStoreRefillRetryCount))
    val dcacheStoreEntranceRetry = windowCounter(event(io.memory.dcacheStoreEntranceRetryCount))
    val l2IReadHit = windowCounter(event(io.memory.l2IReadHit))
    val l2IReadMiss = windowCounter(event(io.memory.l2IReadMiss))
    val l2DReadHit = windowCounter(event(io.memory.l2DReadHit))
    val l2DReadMiss = windowCounter(event(io.memory.l2DReadMiss))
    val l2WriteHit = windowCounter(event(io.memory.l2WriteHit))
    val l2WriteMiss = windowCounter(event(io.memory.l2WriteMiss))
    val l2DirtyWriteback = windowCounter(event(io.memory.l2DirtyWriteback))
    val l2UncacheRead = windowCounter(event(io.memory.l2UncacheRead))
    val l2UncacheWrite = windowCounter(event(io.memory.l2UncacheWrite))
    val l2Busy = windowCounter(event(io.memory.l2Busy))
    val loadIssue = windowCounter(io.memory.loadIssueCount)
    val loadForward = windowCounter(io.memory.loadForwardCount)
    val storeDataReplay = windowCounter(io.memory.storeDataReplayCount)
    val partialOverlapReplay = windowCounter(io.memory.partialOverlapReplayCount)
    val waitStoreDataReplay = windowCounter(io.memory.waitStoreDataReplayCount)
    val staleResult = windowCounter(io.memory.staleResultCount)
    val headLoadTracked = windowCounter(event(io.memory.headLoadTracked))
    val headLoadMissing = windowCounter(event(io.memory.headLoadMissing))
    val headLoadReadyIssued = windowCounter(event(io.memory.headLoadReadyIssued))
    val headLoadReadyWait = windowCounter(event(io.memory.headLoadReadyWait))
    val headLoadExecuting = windowCounter(event(io.memory.headLoadExecuting))
    val headLoadExecutingAfterStoreReplay = windowCounter(
        event(io.memory.headLoadExecutingAfterStoreReplay)
    )
    val headLoadWaitMshrFull = windowCounter(event(io.memory.headLoadWaitMshrFull))
    val headLoadWaitStoreData = windowCounter(event(io.memory.headLoadWaitStoreData))
    val headLoadComplete = windowCounter(event(io.memory.headLoadComplete))
    val headLoadOther = windowCounter(event(io.memory.headLoadOther))
    val headLoadResultNow = windowCounter(event(io.memory.headLoadResultNow))
    val lstOccupancySum = windowCounter(io.memory.lstOccupancy)
    val lstFull = windowCounter(event(io.memory.lstFull))
    val storeBufferFull = windowCounter(event(io.memory.storeBufferFull))

    val loadStartCycle = Reg(Vec(nldq, UInt(64.W)))
    val loadStartHigh  = Reg(Vec(nldq, Bool()))
    val loadStartValid = RegInit(VecInit.fill(nldq)(false.B))

    val nextLoadStartCycle = WireInit(loadStartCycle)
    val nextLoadStartHigh  = WireInit(loadStartHigh)
    val nextLoadStartValid = WireInit(loadStartValid)

    val doneLatency = Wire(Vec(memNissue, UInt(64.W)))
    val doneTracked = Wire(Vec(memNissue, Bool()))

    for (port <- 0 until memNissue) {
        val done = io.backend.loadDone(port)
        val doneCycle = Mux1H(done.bits.indexOH, loadStartCycle)
        val doneHigh = Mux1H(done.bits.indexOH, loadStartHigh)
        val doneValid = (done.bits.indexOH & loadStartValid.asUInt).orR

        doneTracked(port) := countEnable && done.valid && doneValid && doneHigh === done.bits.high
        doneLatency(port) := Mux(doneTracked(port), io.cycle - doneCycle, 0.U)

        when(doneTracked(port)) {
            for (index <- 0 until nldq) {
                when(done.bits.indexOH(index)) {
                    nextLoadStartValid(index) := false.B
                }
            }
        }
    }

    for (port <- 0 until memNissue) {
        val start = io.backend.loadStart(port)
        when(countEnable && start.valid) {
            for (index <- 0 until nldq) {
                when(start.bits.indexOH(index)) {
                    nextLoadStartCycle(index) := io.cycle
                    nextLoadStartHigh(index)  := start.bits.high
                    nextLoadStartValid(index) := true.B
                }
            }
        }
    }

    when(measurementStart) {
        loadStartValid := VecInit.fill(nldq)(false.B)
    }.otherwise {
        loadStartCycle := nextLoadStartCycle
        loadStartHigh  := nextLoadStartHigh
        loadStartValid := nextLoadStartValid
    }

    val latencySamplesThisCycle = PopCount(doneTracked)
    val latencySumThisCycle = doneLatency.reduce(_ +& _)
    val latencyMaxThisCycle = doneLatency.reduce((a, b) => Mux(a > b, a, b))

    val totalLoadSamples = RegInit(0.U(64.W))
    val totalLoadLatency = RegInit(0.U(64.W))
    val maxLoadLatency   = RegInit(0.U(64.W))

    when(measurementStart) {
        totalLoadSamples := 0.U
        totalLoadLatency := 0.U
        maxLoadLatency := 0.U
    }.elsewhen(countEnable) {
        totalLoadSamples := totalLoadSamples + latencySamplesThisCycle
        totalLoadLatency := totalLoadLatency + latencySumThisCycle
        when(latencyMaxThisCycle > maxLoadLatency) {
            maxLoadLatency := latencyMaxThisCycle
        }
    }

    io.totalLoadSamples := totalLoadSamples
    io.totalLoadLatency := totalLoadLatency
    io.maxLoadLatency   := maxLoadLatency

    val loadLatencySamples = windowCounter(latencySamplesThisCycle)
    val loadLatencySum = windowCounter(latencySumThisCycle)
    val windowLoadLatencyMax = RegInit(0.U(64.W))
    val nextWindowLoadLatencyMax = Mux(
        countEnable && latencyMaxThisCycle > windowLoadLatencyMax,
        latencyMaxThisCycle,
        windowLoadLatencyMax
    )
    windowLoadLatencyMax := Mux(measurementStart || reportNow, 0.U, nextWindowLoadLatencyMax)

    when(measurementStart) {
        printf(p"[PERF][MEASURE] startCycle=${io.cycle}\n")
    }

    when(reportNow) {
        printf(p"[PERF][COMMIT] cycle=${io.cycle} measuredCycles=${measuredCycles} inst=${commitInst} c0=${commitHist(0)} c1=${commitHist(1)} c2=${commitHist(2)} c3=${commitHist(3)} robEmpty=${robEmpty} robFull=${robFull} robOccSum=${robOccupancySum} waitInt=${robHeadWaitInt} waitLd=${robHeadWaitLoad} waitSt=${robHeadWaitStore} waitBr=${robHeadWaitBranch} limitSt=${commitBlockedStore} limitBr=${commitBlockedControl}\n")
        printf(p"[PERF][LOADPIPE] preIssue=${robHeadLoadPreIssue} waitResult=${robHeadLoadWaitResult} waitRob=${robHeadLoadWaitRob} untracked=${robHeadLoadUntracked} predWake=${loadPredWake} predSuccess=${loadPredSuccess} predCancel=${loadPredCancel} predIntIssue=${intLoadPredIssue} predMemIssue=${memLoadPredIssue} predLoadIssue=${loadToLoadPredIssue}\n")
        printf(p"[PERF][P0HANDOFF] event=${p0HandoffEvent} blocked=${p0HandoffCommitBlocked} noNext=${p0HandoffNoNext} nextReady=${p0HandoffNextReady} nextInt=${p0HandoffNextWaitInt} nextLoad=${p0HandoffNextWaitLoad} nextStore=${p0HandoffNextWaitStore} nextBranch=${p0HandoffNextWaitBranch} nextOther=${p0HandoffNextWaitOther} c1One=${p0HandoffRetireOne} c1Wide=${p0HandoffRetireWide} c1Deferred=${p0HandoffRetireDeferred}\n")
        printf(p"[PERF][FRONTEND] req=${feReq} reqStall=${feReqStall} resp=${feResp} bundles=${feBundles} instr=${feInstrs} decValid=${feDecodeValid} decFire=${feDecodeFire} ibEmpty=${ibufEmpty} ibFull=${ibufFull} ftqFull=${ftqFull}\n")
        printf(p"[PERF][PREDICT] bpuReq=${bpuReq} bpuResp=${bpuResp} hitSlots=${bpuHits} takenSlots=${bpuTaken} takenRedirect=${feRedirect} early=${earlyRedirect} late=${lateRedirect} repair=${predecodeRepair} resolve=${branchResolve} mispredict=${branchMispredict} flush=${fullFlush}\n")
        printf(p"[PERF][ADVBP] miniEligible=${miniEligible} miniDisagree=${miniDisagree} miniRecover=${miniRecover} miniHarm=${miniHarm} miniWrong=${miniWrong} provider0=${miniProvider0} provider1=${miniProvider1} provider2=${miniProvider2} provider3=${miniProvider3} loopEligible=${loopEligible} loopDisagree=${loopDisagree} loopRecover=${loopRecover} loopHarm=${loopHarm} loopWrong=${loopWrong}\n")
        printf(p"[PERF][BRANCH] directionWrong=${branchDirectionWrong} targetWrong=${branchTargetWrong} condResolve=${branchCondResolve} condMispredict=${branchCondMispredict} jirlResolve=${branchJirlResolve} jirlMispredict=${branchJirlMispredict} actualTaken=${branchActualTaken} predTaken=${branchPredTaken}\n")
        printf(p"[PERF][DISPATCH] active=${dispatchValid} fire=${dispatchFire} renameOut=${renameOutBlocked} renameFree=${renameFreeBlocked} renameTag=${renameTagBlocked} rob=${dispatchRobBlocked} lsq=${dispatchLsqBlocked} intIq=${dispatchIntBlocked} memIq=${dispatchMemBlocked}\n")
        printf(p"[PERF][ISSUE] intOccSum=${intIqOccupancySum} intFull=${intIqFull} intNoReady=${intIqNoReady} intIssue=${intIssue} intStall=${intIssueStall} memOccSum=${memIqOccupancySum} memFull=${memIqFull} memNoReady=${memIqNoReady} memIssue=${memIssue} memStall=${memIssueStall}\n")
        printf(p"[PERF][LSQ] ldOccSum=${ldqOccupancySum} stOccSum=${stqOccupancySum} ldFull=${ldqFull} stFull=${stqFull} lstOccSum=${lstOccupancySum} lstFull=${lstFull} sbFull=${storeBufferFull}\n")
        printf(p"[PERF][MEM] req=${memReq} reqStall=${memReqStall} ld=${loadReq} sta=${staReq} std=${stdReq} result=${loadResult} issue=${loadIssue} dHit=${dcacheHit} dMiss=${dcacheMiss} mshrFull=${dcacheMshrFull} forward=${loadForward} replay=${storeDataReplay} partial=${partialOverlapReplay} waitStd=${waitStoreDataReplay} stale=${staleResult} loadSamples=${loadLatencySamples} loadLatencySum=${loadLatencySum} loadLatencyMax=${nextWindowLoadLatencyMax}\n")
        printf(p"[PERF][HEADLD] tracked=${headLoadTracked} missing=${headLoadMissing} readyIssue=${headLoadReadyIssued} readyWait=${headLoadReadyWait} executing=${headLoadExecuting} replayExec=${headLoadExecutingAfterStoreReplay} waitMshr=${headLoadWaitMshrFull} waitStd=${headLoadWaitStoreData} complete=${headLoadComplete} other=${headLoadOther} resultNow=${headLoadResultNow}\n")
        printf(p"[PERF][CACHE] iHit=${icacheHit} iMiss=${icacheMiss} dStoreHit=${dcacheStoreHit} dStoreMiss=${dcacheStoreMiss} stRetry=${dcacheStoreRetry} stMshrFull=${dcacheStoreMshrFullRetry} stRefill=${dcacheStoreRefillRetry} stEntrance=${dcacheStoreEntranceRetry}\n")
        printf(p"[PERF][L2] iHit=${l2IReadHit} iMiss=${l2IReadMiss} dHit=${l2DReadHit} dMiss=${l2DReadMiss} wbHit=${l2WriteHit} wbMiss=${l2WriteMiss} dirtyEvict=${l2DirtyWriteback} ucRead=${l2UncacheRead} ucWrite=${l2UncacheWrite} busy=${l2Busy}\n")
    }
}
