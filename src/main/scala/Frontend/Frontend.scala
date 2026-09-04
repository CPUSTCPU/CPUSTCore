package CPUSTC.frontend

import chisel3._
import chisel3.util._

import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.Decode._
import CPUSTC.config.Fetch._
import CPUSTC.config.Predict.GShare.historyLength
import CPUSTC.config.ExpCode
import CPUSTC.backend.control.{PipelineRedirect, RedirectKind}
import CPUSTC.perf.FrontendPerfEvents
import CPUSTC.predict.{
    AdvancedPredictorConfig,
    AdvancedPredictorPacketMeta,
    BPUTrainUpdate
}

class FrontendIO(enablePerfCounters: Boolean = false) extends Bundle {
    val redirect = Flipped(Valid(new PipelineRedirect))
    val idle = Input(Bool())

    val icache = new IFUToICacheIO
    val bpu    = new IFUToBPUIO
    val bpuPredecode = Output(new BpuPredecodeUpdate)

    val ibuffer = Vec(ndcd, Decoupled(new IBufferEntry))
    val ftqRetire = Flipped(Valid(new FtqRetire))
    val bpuTrain = Decoupled(new BPUTrainUpdate)
    val bpuHistoryRecovery = Output(Valid(UInt(historyLength.W)))
    val bpuLongHistoryRecovery = Output(
        Valid(UInt(AdvancedPredictorConfig.historyWidth.W))
    )
    val localICacheFlush = Output(Bool())
    val ftqPredictionReadReq = Flipped(Valid(new FtqPtr))
    val ftqPredictionReadResp = Output(Valid(new FtqPredictionRead))
    val ftqReadPtr   = Input(new FtqPtr)
    val ftqReadEntry = Output(new FTQEntry)
    val npc   = Output(UInt(32.W))

    val perf = if (enablePerfCounters) {
        Some(Output(new FrontendPerfEvents))
    } else {
        None
    }
}

class FetchPacketBufferEntry extends Bundle {
    val fetch = new FetchBundle
    val predictor = new AdvancedPredictorPacketMeta
    val fault = Bool()
}

class FetchPacketBuffer extends Module {
    val io = IO(new Bundle {
        val flush = Input(Bool())
        val enq = Flipped(Decoupled(new FetchPacketBufferEntry))
        val deq = Decoupled(new FetchPacketBufferEntry)
    })

    val valid = RegInit(false.B)
    val bits  = Reg(new FetchPacketBufferEntry)

    // The ICache response stage already holds the next packet while ready is
    // low, so one skid entry is enough to absorb a newly asserted stall. Keep
    // enqueue ready independent of dequeue ready to preserve the timing cut.
    io.enq.ready := !valid && !io.flush
    io.deq.valid := (valid || io.enq.valid) && !io.flush
    io.deq.bits := Mux(valid, bits, io.enq.bits)

    val enqFire = io.enq.fire
    val deqFire = io.deq.fire

    when(io.flush) {
        valid := false.B
    }.elsewhen(valid) {
        when(deqFire) {
            valid := false.B
        }
    }.otherwise {
        when(enqFire && !deqFire) {
            valid := true.B
            bits := io.enq.bits
        }
    }

    when(valid) {
        assert(!io.enq.ready)
    }
}

class Frontend(
    enablePerfCounters: Boolean = false,
    useBlackBoxRam: Boolean = false
) extends Module {
    val io = IO(new FrontendIO(enablePerfCounters))

    private val fetchBytes = nfch * 4
    private val fetchOffsetBits = log2Ceil(fetchBytes)
    private val fullFetchMask = ((BigInt(1) << nfch) - 1).U(nfch.W)

    private def alignFetchBlock(addr: UInt): UInt = {
        addr(31, fetchOffsetBits) ## 0.U(fetchOffsetBits.W)
    }

    private def genRedirectMask(addr: UInt): UInt = {
        val wordOffset = addr(fetchOffsetBits - 1, 2)
        MuxLookup(wordOffset, fullFetchMask)(
            (0 until nfch).map { i =>
                i.U -> ((((BigInt(1) << nfch) - 1) << i) &
                    ((BigInt(1) << nfch) - 1)).U(nfch.W)
            }
        )
    }

    val npc  = Module(new NPC)
    val ifu  = Module(new InstructionFetchUnit(useBlackBoxRam))
    val ibuf = Module(new InstructionBuffer(useBlackBoxRam))
    val ftq  = Module(new FetchTargetQueue(useBlackBoxRam))

    val pc = RegInit((0x1c000000L - 4 * nfch).U(32.W))

    npc.io.pc.pc := pc
    pc := npc.io.pc.npc

    val pipelineFlush = io.redirect.valid
    val redirectTargetPending = RegInit(false.B)
    val redirectIsHard =
        io.redirect.bits.kind === RedirectKind.HARD
    val redirectIsBranch =
        io.redirect.bits.kind === RedirectKind.BRANCH
    val redirectMisaligned =
        pipelineFlush && io.redirect.bits.target(1, 0).orR
    val alignedPipelineRedirect = pipelineFlush && !redirectMisaligned
    val sameCycleBranchRequest = alignedPipelineRedirect && redirectIsBranch

    val fetchFaultPending = RegInit(false.B)
    val fetchFaultActive  = RegInit(false.B)
    val fetchFaultAddr    = RegInit(0.U(32.W))

    npc.io.cmt.flush   := alignedPipelineRedirect
    npc.io.cmt.jumpEn  := true.B
    npc.io.cmt.jumpTgt := io.redirect.bits.target

    npc.io.fq.ready :=
        ifu.io.npc.ready &&
        !redirectTargetPending &&
        !fetchFaultActive &&
        !fetchFaultPending &&
        !io.idle
    npc.io.ic.miss  := ifu.io.npc.miss

    npc.io.pd.flush      := false.B
    npc.io.pd.pc         := 0.U
    npc.io.pd.jumpOffset := 0.U

    val fetchFire = ifu.io.fetch.valid && ifu.io.fetch.ready
    val rawEarlyRedirect = ifu.io.earlyRedirect.valid
    val lateRedirect     = ifu.io.lateRedirect.valid
    val earlyRedirect    = rawEarlyRedirect && !lateRedirect
    val frontendRedirect = earlyRedirect || lateRedirect
    val frontendRedirectTarget = Mux(
        lateRedirect,
        ifu.io.lateRedirect.bits,
        ifu.io.earlyRedirect.bits
    )

    when (redirectMisaligned) {
        redirectTargetPending := false.B
    }.elsewhen(pipelineFlush) {
        redirectTargetPending :=
            !sameCycleBranchRequest ||
                !(ifu.io.npc.valid && ifu.io.npc.ready)
    }.elsewhen(lateRedirect) {
        redirectTargetPending := true.B
    }.elsewhen(earlyRedirect) {
        // A ready IFU has already issued the target selected by this early
        // redirect. Only retain the target when that request was blocked.
        redirectTargetPending := !ifu.io.npc.ready
    }.elsewhen(redirectTargetPending && ifu.io.npc.ready) {
        redirectTargetPending := false.B
    }

    npc.io.pr.flush      := frontendRedirect
    npc.io.pr.pc         := Mux(lateRedirect, ifu.io.fetch.bits.pc, io.bpu.resp.bits.pc)
    npc.io.pr.jumpOffset := frontendRedirectTarget
    npc.io.pr.predType   := Mux(lateRedirect, ifu.io.fetch.bits.cfiType, 0.U)

    io.npc := npc.io.pc.npc
    io.localICacheFlush := ifu.io.localICacheFlush

    ifu.io.flush := pipelineFlush
    ifu.io.redirectRequest := sameCycleBranchRequest

    // A late predecode repair updates NPC state at the clock edge and flushes
    // the current IFU packet. It must not feed the same cycle's BPU request
    // address. Early BPU predictions retain their zero-bubble bypass.
    val holdRequest =
        redirectTargetPending ||
        fetchFaultActive ||
        fetchFaultPending ||
        ifu.io.npc.miss
    val baseRequestPc = Mux(holdRequest, pc, npc.io.pc.sequentialPc)
    val baseRequestMask = Mux(
        holdRequest,
        npc.io.pc.currentMask.asUInt,
        fullFetchMask
    )
    val earlyRequestPc = alignFetchBlock(ifu.io.earlyRedirect.bits)
    val earlyRequestMask = genRedirectMask(ifu.io.earlyRedirect.bits)
    val pipelineRequestPc = alignFetchBlock(io.redirect.bits.target)
    val pipelineRequestMask = genRedirectMask(io.redirect.bits.target)

    ifu.io.npc.valid :=
        !io.idle &&
        !fetchFaultActive &&
        !fetchFaultPending &&
        !redirectMisaligned
    ifu.io.npc.req.pc := Mux(
        pipelineFlush,
        pipelineRequestPc,
        Mux(rawEarlyRedirect, earlyRequestPc, baseRequestPc)
    )
    ifu.io.npc.req.mask := Mux(
        pipelineFlush,
        pipelineRequestMask,
        Mux(rawEarlyRedirect, earlyRequestMask, baseRequestMask)
    )

    io.icache <> ifu.io.icache
    io.bpu    <> ifu.io.bpu
    io.bpuPredecode := ifu.io.bpuPredecode

    io.ibuffer <> ibuf.io.deq

    val newEntry = Wire(new FetchBundle)
    newEntry := ifu.io.fetch.bits

    val faultEntry = WireDefault(0.U.asTypeOf(new FetchBundle))
    faultEntry.pc := fetchFaultAddr
    faultEntry.mask := 1.U
    faultEntry.exceptions(0).valid     := true.B
    faultEntry.exceptions(0).cause     := ExpCode.ADEF
    faultEntry.exceptions(0).badvValid := true.B
    faultEntry.exceptions(0).badv      := fetchFaultAddr

    val selectedEntry = Mux(fetchFaultPending, faultEntry, newEntry)
    val selectedValid =
        (fetchFaultPending || (ifu.io.fetch.valid && !fetchFaultActive)) &&
        !pipelineFlush

    // A fall-through skid buffer preserves the zero-stall fetch latency,
    // while its registered enqueue ready cuts FTQ/IBuffer backpressure before
    // it can pass through ICache/TLB and return to IFU request acceptance.
    val packetBuffer = Module(new FetchPacketBuffer)
    packetBuffer.io.flush := pipelineFlush
    packetBuffer.io.enq.valid := selectedValid
    packetBuffer.io.enq.bits.fetch := selectedEntry
    packetBuffer.io.enq.bits.predictor := Mux(
        fetchFaultPending,
        0.U.asTypeOf(new AdvancedPredictorPacketMeta),
        ifu.io.fetchPredictorMeta
    )
    packetBuffer.io.enq.bits.fault := fetchFaultPending

    val packetOutBits = WireDefault(packetBuffer.io.deq.bits.fetch)
    packetOutBits.ftqPtr := ftq.io.enqPtr

    val packetOutReady = ibuf.io.enq.ready && ftq.io.enq.ready
    packetBuffer.io.deq.ready := packetOutReady
    val packetOutFire = packetBuffer.io.deq.fire

    ifu.io.fetch.ready :=
        !fetchFaultPending &&
        !fetchFaultActive &&
        !pipelineFlush &&
        packetBuffer.io.enq.ready

    ibuf.io.enq.valid := packetBuffer.io.deq.valid && ftq.io.enq.ready
    ibuf.io.enq.bits  := packetOutBits

    ftq.io.enq.valid := packetBuffer.io.deq.valid && ibuf.io.enq.ready
    ftq.io.enq.bits  := packetOutBits
    ftq.io.enqPredictorMeta := packetBuffer.io.deq.bits.predictor

    val faultCaptureFire =
        packetBuffer.io.enq.fire && packetBuffer.io.enq.bits.fault
    val faultEnqFire = packetOutFire && packetBuffer.io.deq.bits.fault

    when(redirectMisaligned) {
        fetchFaultPending := true.B
        fetchFaultActive  := true.B
        fetchFaultAddr    := io.redirect.bits.target
    }.elsewhen(alignedPipelineRedirect) {
        fetchFaultPending := false.B
        fetchFaultActive  := false.B
    }.elsewhen(faultCaptureFire) {
        fetchFaultPending := false.B
    }

    ibuf.io.flush := pipelineFlush
    ftq.io.hardRedirect := io.redirect.valid && redirectIsHard
    ftq.io.branchRedirect.valid := io.redirect.valid && redirectIsBranch
    ftq.io.branchRedirect.bits.ptr := io.redirect.bits.ftqPtr
    ftq.io.branchRedirect.bits.offset := io.redirect.bits.ftqOffset
    ftq.io.branchRedirect.bits.cfiType := io.redirect.bits.cfiType
    ftq.io.branchRedirect.bits.actualTaken := io.redirect.bits.actualTaken
    ftq.io.branchRedirect.bits.actualTarget := io.redirect.bits.target
    ftq.io.branchRedirect.bits.isCall := io.redirect.bits.isCall
    ftq.io.branchRedirect.bits.isRet := io.redirect.bits.isRet
    ftq.io.retire := io.ftqRetire
    io.bpuTrain <> ftq.io.train
    io.bpuHistoryRecovery := ftq.io.historyRecovery
    io.bpuLongHistoryRecovery := ftq.io.longHistoryRecovery

    ftq.io.predictionReadReq := io.ftqPredictionReadReq
    io.ftqPredictionReadResp := ftq.io.predictionReadResp

    ftq.io.readPtr := io.ftqReadPtr
    io.ftqReadEntry := ftq.io.readEntry

    if (enablePerfCounters) {
        val perf = io.perf.get

        perf.icacheReqFire   := io.icache.req.fire
        perf.icacheReqStall  := io.icache.req.valid && !io.icache.req.ready
        perf.icacheRespFire  := io.icache.resp.fire
        perf.fetchBundleFire := packetOutFire
        perf.fetchInstrCount := Mux(packetOutFire, PopCount(packetOutBits.mask), 0.U)

        perf.decodeValidCount := PopCount(VecInit(io.ibuffer.map(_.valid)))
        perf.decodeFireCount  := PopCount(VecInit(io.ibuffer.map(_.fire)))

        perf.ibufferEmpty := ibuf.io.empty
        perf.ibufferFull  := ibuf.io.full
        perf.ftqFull      := ftq.io.full

        val liveBpuResp = io.bpu.resp.valid && !io.icache.flush

        perf.bpuReqFire    := io.bpu.req.fire
        perf.bpuResp       := liveBpuResp
        perf.bpuHitCount   := Mux(
            liveBpuResp,
            PopCount(VecInit(io.bpu.resp.bits.pretarget.map(_.valid))),
            0.U
        )
        perf.bpuTakenCount := Mux(
            liveBpuResp,
            PopCount(VecInit((0 until nfch).map { i =>
                io.bpu.resp.bits.taken(i) &&
                    io.bpu.resp.bits.pretarget(i).valid
            })),
            0.U
        )

        perf.predictedRedirect := frontendRedirect
        perf.earlyRedirect     := earlyRedirect
        perf.lateRedirect      := lateRedirect
        perf.predecodeRepair    := ifu.io.bpuPredecode.repair
        perf.advancedPredictor  := ftq.io.predictorPerf
    }

    when(io.redirect.valid) {
        assert(redirectIsHard || redirectIsBranch)
        assert(!(redirectIsHard && redirectIsBranch))
    }

    when(fetchFaultActive) {
        assert(!io.icache.req.valid)
        assert(!io.bpu.req.valid)
    }

    when(faultEnqFire) {
        assert(ftq.io.enq.fire)
        assert(ibuf.io.enq.bits.mask === 1.U)
        assert(ibuf.io.enq.bits.exceptions(0).valid)
        assert(ibuf.io.enq.bits.exceptions(0).cause === ExpCode.ADEF)
        assert(ibuf.io.enq.bits.exceptions(0).badv === fetchFaultAddr)
    }

    when(packetOutFire) {
        assert(ibuf.io.enq.fire && ftq.io.enq.fire)
    }
}
