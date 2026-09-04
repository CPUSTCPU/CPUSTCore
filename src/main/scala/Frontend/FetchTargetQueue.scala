package CPUSTC.frontend

import chisel3._
import chisel3.util._

import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.Fetch._
import CPUSTC.config.JumpOp._
import CPUSTC.config.MaskLower
import CPUSTC.config.Predict.GShare.historyLength
import CPUSTC.config.Predict.BTBMini.useRamBtb
import CPUSTC.predict.{
    AdvancedPredictorConfig,
    AdvancedPredictorPacketMeta,
    AdvancedPredictorPerfEvents,
    BPUTrainUpdate,
    BpuSdpRam,
    BtbPredictionMeta,
    GlobalHistory
}

class FtqPtr extends Bundle {
    val idx  = UInt(log2Ceil(nftq).W)
    val high = Bool()
}

class FtqRetire extends Bundle {
    val ptr       = new FtqPtr
    val offset    = UInt(log2Ceil(nfch).W)
    val completed = Valid(new FtqPtr)
}

class FtqBranchCorrection extends Bundle {
    val ptr          = new FtqPtr
    val offset       = UInt(log2Ceil(nfch).W)
    val cfiType      = UInt(CFI_SZ.W)
    val actualTaken  = Bool()
    val actualTarget = UInt(32.W)
    val isCall       = Bool()
    val isRet        = Bool()
}

class FTQEntry extends Bundle {
    val ftqPtr = new FtqPtr
    val pc     = UInt(32.W)
    val history = UInt(historyLength.W)

    val cfiIdx    = Valid(UInt(log2Ceil(nfch).W))
    val cfiType   = UInt(CFI_SZ.W)
    val cfiIsCall = Bool()
    val cfiIsRet  = Bool()

    val taken     = Bool()
    val pretarget = UInt(32.W)

    val target = UInt(32.W)
    val mask   = UInt(nfch.W)
    val brMask = UInt(nfch.W)
}

class FtqBaseEntry extends Bundle {
    val ptr           = new FtqPtr
    val pc            = UInt(32.W)
    val history       = UInt(historyLength.W)
    val mask          = UInt(nfch.W)
    val brMask        = UInt(nfch.W)
    val branchTargets = Vec(nfch, UInt(32.W))

    val cfiIdx       = Valid(UInt(log2Ceil(nfch).W))
    val cfiType      = UInt(CFI_SZ.W)
    val cfiTaken     = Bool()
    val cfiTarget    = UInt(32.W)
    val cfiIsCall    = Bool()
    val cfiIsRet     = Bool()

    val predHit   = UInt(nfch.W)
    val btbMeta   = Vec(nfch, new BtbPredictionMeta)
}

class FtqCorrectionEntry extends Bundle {
    val valid        = Bool()
    val ptr          = new FtqPtr
    val cfiIdx       = UInt(log2Ceil(nfch).W)
    val cfiType      = UInt(CFI_SZ.W)
    val actualTaken  = Bool()
    val actualTarget = UInt(32.W)
    val isCall       = Bool()
    val isRet        = Bool()
}

class FtqHardCutEntry extends Bundle {
    val valid      = Bool()
    val ptr        = new FtqPtr
    val lastOffset = UInt(log2Ceil(nfch).W)
}

class FtqPredictionEntry extends Bundle {
    val high   = Bool()
    val cfiIdx = Valid(UInt(log2Ceil(nfch).W))
    val target = UInt(32.W)
}

class FtqTrainSidecarEntry extends Bundle {
    val high     = Bool()
    val fullMask = UInt(nfch.W)
}

class FtqPredictorSidecarEntry extends Bundle {
    val high = Bool()
    val meta = new AdvancedPredictorPacketMeta
}

class FtqPredictionRead extends Bundle {
    val ptr    = new FtqPtr
    val cfiIdx = Valid(UInt(log2Ceil(nfch).W))
    val target = UInt(32.W)
}

class FetchTargetQueueIO extends Bundle {
    val hardRedirect = Input(Bool())
    val branchRedirect = Flipped(Valid(new FtqBranchCorrection))
    val retire = Flipped(Valid(new FtqRetire))

    val enq = Flipped(Decoupled(new FetchBundle))
    val enqPredictorMeta = Input(new AdvancedPredictorPacketMeta)
    val enqPtr = Output(new FtqPtr)

    val train = Decoupled(new BPUTrainUpdate)
    val historyRecovery = Output(Valid(UInt(historyLength.W)))
    val longHistoryRecovery = Output(
        Valid(UInt(AdvancedPredictorConfig.historyWidth.W))
    )
    val predictorPerf = Output(new AdvancedPredictorPerfEvents)

    val predictionReadReq = Flipped(Valid(new FtqPtr))
    val predictionReadResp = Output(Valid(new FtqPredictionRead))

    // Kept as a debug/readback interface. The production Core does not consume
    // it, so Vivado can trim the extra read port there.
    val readPtr = Input(new FtqPtr)
    val readEntry = Output(new FTQEntry)

    val empty = Output(Bool())
    val full  = Output(Bool())
}

class FetchTargetQueue(useBlackBoxRam: Boolean = false) extends Module {
    val io = IO(new FetchTargetQueueIO)

    private val ptrWidth = log2Ceil(nftq) + 1

    private def ptrValue(ptr: FtqPtr): UInt = Cat(ptr.high, ptr.idx)

    private def nextPtr(ptr: FtqPtr): FtqPtr = {
        val next = Wire(new FtqPtr)
        val wrap = ptr.idx === (nftq - 1).U
        next.idx  := Mux(wrap, 0.U, ptr.idx + 1.U)
        next.high := ptr.high ^ wrap
        next
    }

    private def samePtr(a: FtqPtr, b: FtqPtr): Bool =
        a.idx === b.idx && a.high === b.high

    private def distance(from: FtqPtr, to: FtqPtr): UInt =
        (ptrValue(to) - ptrValue(from))(ptrWidth - 1, 0)

    private def inWindow(ptr: FtqPtr, head: FtqPtr, tail: FtqPtr): Bool =
        distance(head, ptr) < distance(head, tail)

    private def buildTrainMask(
        base: FtqBaseEntry,
        correctionValid: Bool,
        correctionIdx: UInt,
        correctionType: UInt,
        hardCutValid: Bool,
        hardCutOffset: UInt
    ): UInt = {
        val cfiValid = Mux(correctionValid, true.B, base.cfiIdx.valid)
        val cfiIdx = Mux(correctionValid, correctionIdx, base.cfiIdx.bits)
        val cfiType = Mux(correctionValid, correctionType, base.cfiType)
        val cfiOH = UIntToOH(cfiIdx, nfch)
        val cfiPrefix = MaskLower(cfiOH)
        val hardCutPrefix = MaskLower(UIntToOH(hardCutOffset, nfch))
        val visibleMask = base.mask & Mux(
            hardCutValid,
            hardCutPrefix,
            Fill(nfch, 1.U(1.W))
        )
        val visibleCfi = cfiValid && (cfiOH & visibleMask).orR
        val visibleBranches = base.brMask & visibleMask & Mux(
            visibleCfi,
            cfiPrefix,
            Fill(nfch, 1.U(1.W))
        )
        val visibleJump = Mux(
            visibleCfi && cfiType =/= CFI_BR,
            cfiOH,
            0.U(nfch.W)
        )
        visibleBranches | visibleJump
    }

    private val baseMem = Mem(nftq, new FtqBaseEntry)
    private val correctionMem = Mem(nftq, new FtqCorrectionEntry)
    private val hardCutMem = Mem(nftq, new FtqHardCutEntry)
    private val predictionMem = SyncReadMem(nftq, new FtqPredictionEntry)
    private val trainSidecar = RegInit(VecInit(Seq.fill(nftq)(
        0.U.asTypeOf(new FtqTrainSidecarEntry)
    )))
    private val predictorSidecarWidth = (new FtqPredictorSidecarEntry).getWidth
    private val predictorSidecarRam = Module(new BpuSdpRam(
        predictorSidecarWidth,
        nftq,
        useBlackBoxRam
    ))
    private val longHistoryValid = RegInit(VecInit.fill(nftq)(false.B))
    private val longHistoryHigh = Reg(Vec(nftq, Bool()))
    private val longHistoryData = Reg(
        Vec(nftq, UInt(AdvancedPredictorConfig.historyWidth.W))
    )

    private val allocTail = RegInit(0.U.asTypeOf(new FtqPtr))
    private val trainHead = RegInit(0.U.asTypeOf(new FtqPtr))
    private val commitTarget = RegInit(0.U.asTypeOf(new FtqPtr))
    private val commitPending = RegInit(false.B)
    private val trainedMask = RegInit(0.U(nfch.W))
    private val headFullMask = RegInit(0.U(nfch.W))
    private val headCandidateMask = RegInit(0.U(nfch.W))
    private val allTrainSlots = ((BigInt(1) << nfch) - 1).U(nfch.W)
    private val headHardCutMask = RegInit(allTrainSlots)

    private val lastRetired = RegInit(0.U.asTypeOf(Valid(new FtqRetire)))

    private val headPredictorMeta = WireDefault(
        0.U.asTypeOf(new AdvancedPredictorPacketMeta)
    )

    private val trainBase = baseMem(trainHead.idx)
    private val trainCorrection = correctionMem(trainHead.idx)
    private val trainHardCut = hardCutMem(trainHead.idx)

    private val correctionValid =
        trainCorrection.valid && samePtr(trainCorrection.ptr, trainHead)
    private val hardCutValid =
        trainHardCut.valid && samePtr(trainHardCut.ptr, trainHead)

    private val effectiveCfiValid = Mux(
        correctionValid,
        true.B,
        trainBase.cfiIdx.valid
    )
    private val effectiveCfiIdx = Mux(
        correctionValid,
        trainCorrection.cfiIdx,
        trainBase.cfiIdx.bits
    )
    private val effectiveCfiType = Mux(
        correctionValid,
        trainCorrection.cfiType,
        trainBase.cfiType
    )
    private val effectiveTaken = Mux(
        correctionValid,
        trainCorrection.actualTaken,
        trainBase.cfiTaken
    )
    private val effectiveTarget = Mux(
        correctionValid,
        trainCorrection.actualTarget,
        trainBase.cfiTarget
    )
    private val effectiveIsCall = Mux(
        correctionValid,
        trainCorrection.isCall,
        trainBase.cfiIsCall
    )
    private val effectiveIsRet = Mux(
        correctionValid,
        trainCorrection.isRet,
        trainBase.cfiIsRet
    )

    private val cfiOH = UIntToOH(effectiveCfiIdx, nfch)
    private val cfiPrefix = MaskLower(cfiOH)
    private val hardCutPrefix = MaskLower(UIntToOH(trainHardCut.lastOffset, nfch))
    private val visibleMask = trainBase.mask & Mux(
        hardCutValid,
        hardCutPrefix,
        Fill(nfch, 1.U(1.W))
    )
    private val visibleCfi = effectiveCfiValid && (cfiOH & visibleMask).orR
    private val visibleBranches = trainBase.brMask & visibleMask & Mux(
        visibleCfi,
        cfiPrefix,
        Fill(nfch, 1.U(1.W))
    )
    private val visibleJump = Mux(
        visibleCfi && effectiveCfiType =/= CFI_BR,
        cfiOH,
        0.U(nfch.W)
    )
    private val decodedTrainMask = visibleBranches | visibleJump
    private val effectiveHeadFullMask = headFullMask & headHardCutMask
    private val effectiveHeadCandidateMask =
        headCandidateMask & headHardCutMask
    private val trainMask = effectiveHeadFullMask
    private val trainCandidates = decodedTrainMask & ~trainedMask
    private val trainOH = PriorityEncoderOH(effectiveHeadCandidateMask)
    private val trainSlot = OHToUInt(trainOH)
    private val candidateNonEmpty = effectiveHeadCandidateMask.orR
    private val remainingAfterFire = effectiveHeadCandidateMask & ~trainOH

    private val retireCompletesBundle =
        io.retire.valid && io.retire.bits.completed.valid
    private val activeCommitTarget = Mux(
        retireCompletesBundle,
        io.retire.bits.completed.bits,
        commitTarget
    )
    private val pendingNow = commitPending || retireCompletesBundle
    private val redirectActive = io.hardRedirect || io.branchRedirect.valid
    private val trainControlBlocked =
        io.hardRedirect || (
            io.branchRedirect.valid &&
            samePtr(io.branchRedirect.bits.ptr, trainHead)
        )

    io.train.valid :=
        pendingNow && candidateNonEmpty && !trainControlBlocked
    io.train.bits := 0.U.asTypeOf(new BPUTrainUpdate)
    io.train.bits.pc := trainBase.pc + (trainSlot << 2)
    io.train.bits.taken :=
        visibleCfi && effectiveCfiIdx === trainSlot && effectiveTaken
    io.train.bits.predType := Mux(
        trainBase.brMask(trainSlot),
        BR,
        Mux(effectiveIsCall, CALL, Mux(effectiveIsRet, RET, BR))
    )
    io.train.bits.target := Mux(
        visibleCfi && effectiveCfiIdx === trainSlot &&
            effectiveCfiType === CFI_JIRL,
        effectiveTarget,
        trainBase.branchTargets(trainSlot)
    )
    io.train.bits.predHit := trainBase.predHit(trainSlot)
    io.train.bits.mispredict :=
        correctionValid && effectiveCfiIdx === trainSlot
    io.train.bits.predictor := headPredictorMeta

    val firstTrainForBundle = trainedMask === 0.U
    io.train.bits.btb.valid := firstTrainForBundle
    io.train.bits.btb.bits.basePc := trainBase.pc
    io.train.bits.btb.bits.history := trainBase.history
    io.train.bits.btb.bits.trainMask := trainMask
    io.train.bits.btb.bits.takenMask := Mux(
        visibleCfi && effectiveTaken,
        cfiOH,
        0.U(nfch.W)
    )
    io.train.bits.btb.bits.predHit := trainBase.predHit & trainMask
    for (slot <- 0 until nfch) {
        val slotIsCfi = visibleCfi && effectiveCfiIdx === slot.U
        val slotIsBranch = visibleBranches(slot)

        // The direct-indexed RAM BTB has no replacement way. Reuse its
        // otherwise dead training bit for the exact decoded branch class,
        // without widening BtbPacketTrain or any FTQ storage.
        io.train.bits.btb.bits.writeWay(slot) := (if (useRamBtb) {
            trainBase.brMask(slot)
        } else {
            trainBase.btbMeta(slot).writeWay
        })
        io.train.bits.btb.bits.oldCtr(slot) := trainBase.btbMeta(slot).localCtr
        io.train.bits.btb.bits.oldPredictorCtr(slot) :=
            trainBase.btbMeta(slot).predictorCtr
        io.train.bits.btb.bits.bias(slot) := trainBase.btbMeta(slot).bias
        io.train.bits.btb.bits.oldChooseAgree(slot) :=
            trainBase.btbMeta(slot).chooseAgree
        io.train.bits.btb.bits.predType(slot) := Mux(
            slotIsBranch,
            BR,
            Mux(
                slotIsCfi && effectiveIsCall,
                CALL,
                Mux(slotIsCfi && effectiveIsRet, RET, BR)
            )
        )
        io.train.bits.btb.bits.target(slot) := Mux(
            slotIsCfi && effectiveCfiType === CFI_JIRL,
            effectiveTarget(31, 2),
            trainBase.branchTargets(slot)(31, 2)
        )
    }

    private val trainAdvance =
        pendingNow && !trainControlBlocked && (
            !candidateNonEmpty ||
            (io.train.fire && !remainingAfterFire.orR)
        )
    private val reachesCommitTarget = samePtr(trainHead, activeCommitTarget)
    private val trainHeadAfter = Mux(trainAdvance, nextPtr(trainHead), trainHead)
    private val pendingAfterTrain =
        pendingNow && !(trainAdvance && reachesCommitTarget)

    private val activeLastRetired = WireDefault(lastRetired)
    when(io.retire.valid) {
        activeLastRetired.valid := true.B
        activeLastRetired.bits  := io.retire.bits
    }

    // A current retire is guaranteed by the input protocol to name a live
    // entry. Keep its valid bit off the ring-distance compare used only by a
    // previously saved retire event.
    private val savedRetiredStillLive =
        lastRetired.valid &&
        inWindow(lastRetired.bits.ptr, trainHeadAfter, allocTail)
    private val retiredStillLive = Mux(
        io.retire.valid,
        true.B,
        savedRetiredStillLive
    )
    private val hardTail = Mux(
        retiredStillLive,
        nextPtr(activeLastRetired.bits.ptr),
        trainHeadAfter
    )

    private val liveCount = distance(trainHead, allocTail)
    private val fullNow = liveCount === nftq.U
    io.enq.ready := !redirectActive && (!fullNow || trainAdvance)
    io.enqPtr := allocTail
    io.empty := liveCount === 0.U
    io.full  := fullNow

    val enqBase = WireDefault(0.U.asTypeOf(new FtqBaseEntry))
    enqBase.ptr        := allocTail
    enqBase.pc         := io.enq.bits.pc
    enqBase.history    := io.enq.bits.history
    enqBase.mask       := io.enq.bits.mask
    enqBase.brMask     := io.enq.bits.brMask & io.enq.bits.mask
    enqBase.branchTargets := io.enq.bits.branchTargets
    enqBase.cfiIdx     := io.enq.bits.cfiIdx
    enqBase.cfiType    := io.enq.bits.cfiType
    enqBase.cfiTaken   := io.enq.bits.taken
    enqBase.cfiTarget  := io.enq.bits.target
    enqBase.cfiIsCall  := io.enq.bits.cfiIsCall
    enqBase.cfiIsRet   := io.enq.bits.cfiIsRet
    enqBase.predHit    := io.enq.bits.predHit
    enqBase.btbMeta    := io.enq.bits.btbMeta

    val enqTrainMask = buildTrainMask(
        enqBase,
        false.B,
        0.U(log2Ceil(nfch).W),
        CFI_X,
        false.B,
        0.U(log2Ceil(nfch).W)
    )

    when(io.enq.fire) {
        baseMem.write(allocTail.idx, enqBase)
    }

    val predictorSidecarWrite = Wire(
        new FtqPredictorSidecarEntry
    )
    predictorSidecarWrite.high := allocTail.high
    predictorSidecarWrite.meta := io.enqPredictorMeta

    predictorSidecarRam.io.ren := true.B
    predictorSidecarRam.io.raddr := trainHeadAfter.idx
    predictorSidecarRam.io.wen := io.enq.fire
    predictorSidecarRam.io.waddr := allocTail.idx
    predictorSidecarRam.io.wdata := predictorSidecarWrite.asUInt

    val predictorWriteMatchesRead =
        io.enq.fire && allocTail.idx === trainHeadAfter.idx
    val predictorWriteMatchesReadReg = RegNext(
        predictorWriteMatchesRead,
        false.B
    )
    val predictorForwardData = RegEnable(
        predictorSidecarWrite.asUInt,
        predictorWriteMatchesRead
    )
    val predictorSidecarRead = Mux(
        predictorWriteMatchesReadReg,
        predictorForwardData,
        predictorSidecarRam.io.rdata
    ).asTypeOf(new FtqPredictorSidecarEntry)
    val predictorSidecarMatchesHead =
        predictorSidecarRead.high === trainHead.high &&
            predictorSidecarRead.meta.valid
    when(predictorSidecarMatchesHead) {
        headPredictorMeta := predictorSidecarRead.meta
    }

    when(io.enq.fire) {
        longHistoryValid(allocTail.idx) := io.enqPredictorMeta.valid
        longHistoryHigh(allocTail.idx) := allocTail.high
        longHistoryData(allocTail.idx) :=
            io.enqPredictorMeta.longHistory
    }

    val predictionWrite = WireDefault(0.U.asTypeOf(new FtqPredictionEntry))
    predictionWrite.high   := allocTail.high
    predictionWrite.cfiIdx := io.enq.bits.cfiIdx
    predictionWrite.target := io.enq.bits.target

    when(io.enq.fire) {
        predictionMem.write(allocTail.idx, predictionWrite)
    }

    val predictionReadData = predictionMem.read(
        io.predictionReadReq.bits.idx,
        io.predictionReadReq.valid
    )
    // Hold the most recent response while the P0 issue register is stalled.
    // A new request can only be launched when that register can accept a new
    // uop, so replacing this state also replaces the associated uop.
    val predictionReadValid = RegInit(false.B)
    val predictionReadPtr = RegInit(0.U.asTypeOf(new FtqPtr))
    when(io.predictionReadReq.valid) {
        predictionReadValid := true.B
        predictionReadPtr := io.predictionReadReq.bits
    }

    // An issued JIRL names a live FTQ entry, while allocTail names the first
    // free entry. They cannot address the same physical slot when an enqueue
    // writes predictionMem, even across a full-ring generation turnover.
    val predictionResult = predictionReadData
    when(!reset.asBool && io.predictionReadReq.valid && io.enq.fire) {
        assert(
            io.predictionReadReq.bits.idx =/= allocTail.idx,
            "FTQ prediction read collided with allocation"
        )
    }

    // SyncReadMem data is only defined in the response cycle.  Use it
    // directly in that cycle, then retain it while the matching P0 uop waits.
    val predictionResultFresh = RegNext(
        io.predictionReadReq.valid,
        false.B
    )
    val predictionResultHeld = RegInit(
        0.U.asTypeOf(new FtqPredictionEntry)
    )
    when(predictionResultFresh) {
        predictionResultHeld := predictionResult
    }
    val predictionResultStable = Mux(
        predictionResultFresh,
        predictionResult,
        predictionResultHeld
    )

    io.predictionReadResp.valid :=
        predictionReadValid &&
        predictionResultStable.high === predictionReadPtr.high
    io.predictionReadResp.bits.ptr    := predictionReadPtr
    io.predictionReadResp.bits.cfiIdx := predictionResultStable.cfiIdx
    io.predictionReadResp.bits.target := predictionResultStable.target

    val correctionWrite = WireDefault(0.U.asTypeOf(new FtqCorrectionEntry))
    correctionWrite.valid        := io.branchRedirect.valid
    correctionWrite.ptr          := io.branchRedirect.bits.ptr
    correctionWrite.cfiIdx       := io.branchRedirect.bits.offset
    correctionWrite.cfiType      := io.branchRedirect.bits.cfiType
    correctionWrite.actualTaken  := io.branchRedirect.bits.actualTaken
    correctionWrite.actualTarget := io.branchRedirect.bits.actualTarget
    correctionWrite.isCall       := io.branchRedirect.bits.isCall
    correctionWrite.isRet        := io.branchRedirect.bits.isRet

    val correctionWriteEnable = io.branchRedirect.valid || io.enq.fire
    val correctionWriteAddr = Mux(
        io.branchRedirect.valid,
        io.branchRedirect.bits.ptr.idx,
        allocTail.idx
    )
    val correctionWriteData = Mux(
        io.branchRedirect.valid,
        correctionWrite,
        0.U.asTypeOf(new FtqCorrectionEntry)
    )
    when(correctionWriteEnable) {
        correctionMem.write(correctionWriteAddr, correctionWriteData)
    }

    val hardCutWrite = WireDefault(0.U.asTypeOf(new FtqHardCutEntry))
    hardCutWrite.valid      := io.hardRedirect && retiredStillLive
    hardCutWrite.ptr        := activeLastRetired.bits.ptr
    hardCutWrite.lastOffset := activeLastRetired.bits.offset

    val hardCutWriteEnable = (io.hardRedirect && retiredStillLive) || io.enq.fire
    val hardCutWriteAddr = Mux(
        io.hardRedirect && retiredStillLive,
        activeLastRetired.bits.ptr.idx,
        allocTail.idx
    )
    val hardCutWriteData = Mux(
        io.hardRedirect && retiredStillLive,
        hardCutWrite,
        0.U.asTypeOf(new FtqHardCutEntry)
    )
    when(hardCutWriteEnable) {
        hardCutMem.write(hardCutWriteAddr, hardCutWriteData)
    }

    // Keep only the narrow scheduling summary beside the wide FTQ payload.
    // Overlay writes rebuild this mask from the base entry so head control
    // never needs to read the three payload memories combinationally.
    val correctionSidecarBase = baseMem(io.branchRedirect.bits.ptr.idx)
    val correctionSidecarHardCut = hardCutMem(
        io.branchRedirect.bits.ptr.idx
    )
    val correctionIncomingHardCut =
        io.hardRedirect && retiredStillLive &&
        samePtr(activeLastRetired.bits.ptr, io.branchRedirect.bits.ptr)
    val correctionExistingHardCut =
        correctionSidecarHardCut.valid &&
        samePtr(correctionSidecarHardCut.ptr, io.branchRedirect.bits.ptr)
    val correctionHardCutValid =
        correctionIncomingHardCut || correctionExistingHardCut
    val correctionHardCutOffset = Mux(
        correctionIncomingHardCut,
        activeLastRetired.bits.offset,
        correctionSidecarHardCut.lastOffset
    )
    val correctionFullMask = buildTrainMask(
        correctionSidecarBase,
        true.B,
        io.branchRedirect.bits.offset,
        io.branchRedirect.bits.cfiType,
        correctionHardCutValid,
        correctionHardCutOffset
    )

    val hardCutPtr = activeLastRetired.bits.ptr
    val incomingHardCutPrefix = MaskLower(UIntToOH(
        activeLastRetired.bits.offset,
        nfch
    ))

    val trainedMaskAfterFire = Mux(
        io.train.fire,
        trainedMask | trainOH,
        trainedMask
    )
    val correctionCandidateMask = correctionFullMask & ~trainedMask

    when(io.enq.fire) {
        trainSidecar(allocTail.idx).high := allocTail.high
        trainSidecar(allocTail.idx).fullMask := enqTrainMask
    }
    when(io.branchRedirect.valid) {
        trainSidecar(io.branchRedirect.bits.ptr.idx).high :=
            io.branchRedirect.bits.ptr.high
        trainSidecar(io.branchRedirect.bits.ptr.idx).fullMask :=
            correctionFullMask
    }
    when(io.hardRedirect && retiredStillLive) {
        for (entry <- 0 until nftq) {
            when(hardCutPtr.idx === entry.U) {
                trainSidecar(entry).fullMask :=
                    trainSidecar(entry).fullMask & incomingHardCutPrefix
            }
        }
    }

    when(io.retire.valid) {
        lastRetired.valid := true.B
        lastRetired.bits  := io.retire.bits
    }

    val nextHeadSidecar = trainSidecar(trainHeadAfter.idx)
    val nextHeadStoredMask = Mux(
        nextHeadSidecar.high === trainHeadAfter.high,
        nextHeadSidecar.fullMask,
        0.U(nfch.W)
    )
    val nextHeadLoadFullMask = Wire(UInt(nfch.W))
    nextHeadLoadFullMask := nextHeadStoredMask
    when(io.enq.fire && samePtr(allocTail, trainHeadAfter)) {
        nextHeadLoadFullMask := enqTrainMask
    }
    when(
        io.branchRedirect.valid &&
        samePtr(io.branchRedirect.bits.ptr, trainHeadAfter)
    ) {
        nextHeadLoadFullMask := correctionFullMask
    }
    when(trainAdvance) {
        trainHead          := trainHeadAfter
        trainedMask        := 0.U
        headFullMask       := nextHeadLoadFullMask
        headCandidateMask  := nextHeadLoadFullMask
        headHardCutMask    := allTrainSlots
    }.otherwise {
        when(io.train.fire) {
            trainedMask        := trainedMaskAfterFire
            headCandidateMask  := headCandidateMask & ~trainOH
        }
        when(io.enq.fire && samePtr(allocTail, trainHead)) {
            trainedMask        := 0.U
            headFullMask       := enqTrainMask
            headCandidateMask  := enqTrainMask
            headHardCutMask    := allTrainSlots
        }
        when(
            io.branchRedirect.valid &&
            samePtr(io.branchRedirect.bits.ptr, trainHead)
        ) {
            headFullMask       := correctionFullMask
            headCandidateMask  := correctionCandidateMask
            headHardCutMask    := allTrainSlots
        }
        when(
            io.hardRedirect && retiredStillLive &&
            samePtr(hardCutPtr, trainHead)
        ) {
            headHardCutMask := headHardCutMask & incomingHardCutPrefix
        }
    }

    when(retireCompletesBundle) {
        commitTarget := io.retire.bits.completed.bits
    }
    commitPending := pendingAfterTrain

    when(io.enq.fire) {
        allocTail := nextPtr(allocTail)
    }

    when(io.branchRedirect.valid) {
        allocTail := nextPtr(io.branchRedirect.bits.ptr)
    }

    val recoveryBase = baseMem(io.branchRedirect.bits.ptr.idx)
    val recoveryOffsetOH = UIntToOH(io.branchRedirect.bits.offset, nfch)
    val recoveryBranchMask = recoveryBase.brMask & recoveryBase.mask &
        MaskLower(recoveryOffsetOH)
    val recoveryTakenMask = Mux(
        io.branchRedirect.bits.cfiType === CFI_BR &&
            io.branchRedirect.bits.actualTaken,
        recoveryOffsetOH,
        0.U(nfch.W)
    )
    io.historyRecovery.valid := io.branchRedirect.valid
    io.historyRecovery.bits := GlobalHistory.advance(
        recoveryBase.history,
        recoveryBranchMask,
        recoveryTakenMask
    )
    val recoveryLongHistoryValid =
        longHistoryValid(io.branchRedirect.bits.ptr.idx) &&
            longHistoryHigh(io.branchRedirect.bits.ptr.idx) ===
                io.branchRedirect.bits.ptr.high
    io.longHistoryRecovery.valid :=
        io.branchRedirect.valid && recoveryLongHistoryValid
    io.longHistoryRecovery.bits := GlobalHistory.advance(
        longHistoryData(io.branchRedirect.bits.ptr.idx),
        recoveryBranchMask,
        recoveryTakenMask
    )

    io.predictorPerf := 0.U.asTypeOf(new AdvancedPredictorPerfEvents)
    val predictorTrainConditional =
        io.train.fire && trainBase.brMask(trainSlot)
    val predictorActualTaken = io.train.bits.taken
    val predictorBaseTaken = headPredictorMeta.fastTaken(trainSlot)
    val predictorBaseWrong =
        predictorBaseTaken =/= predictorActualTaken

    val miniEligible =
        predictorTrainConditional && headPredictorMeta.valid &&
            headPredictorMeta.miniValid &&
            headPredictorMeta.miniProviderHit(trainSlot)
    val miniTaken = headPredictorMeta.miniCandidateTaken(trainSlot)
    val miniWrong = miniTaken =/= predictorActualTaken
    io.predictorPerf.miniEligible := miniEligible
    io.predictorPerf.miniDisagree :=
        miniEligible && miniTaken =/= predictorBaseTaken
    io.predictorPerf.miniRecover :=
        miniEligible && predictorBaseWrong && !miniWrong
    io.predictorPerf.miniHarm :=
        miniEligible && !predictorBaseWrong && miniWrong
    io.predictorPerf.miniWrong := miniEligible && miniWrong
    io.predictorPerf.miniProvider := Mux(
        miniEligible,
        headPredictorMeta.miniMeta.slots(trainSlot).provider,
        0.U
    )
    when(miniEligible) {
        assert(
            io.predictorPerf.miniDisagree ===
                (io.predictorPerf.miniRecover || io.predictorPerf.miniHarm)
        )
        assert(!(io.predictorPerf.miniRecover && io.predictorPerf.miniHarm))
    }

    when(io.hardRedirect) {
        allocTail := hardTail
        commitPending := retiredStillLive
        when(retiredStillLive) {
            commitTarget := activeLastRetired.bits.ptr
        }
        when(!retiredStillLive) {
            trainedMask     := 0.U
            headHardCutMask := 0.U
        }
    }

    val debugBase = baseMem(io.readPtr.idx)
    val debugCorrection = correctionMem(io.readPtr.idx)
    val debugCorrectionValid =
        debugCorrection.valid && samePtr(debugCorrection.ptr, io.readPtr)
    val debugInWindow = inWindow(io.readPtr, trainHead, allocTail)

    io.readEntry := 0.U.asTypeOf(new FTQEntry)
    when(debugInWindow && samePtr(debugBase.ptr, io.readPtr)) {
        io.readEntry.ftqPtr := debugBase.ptr
        io.readEntry.pc     := debugBase.pc
        io.readEntry.history := debugBase.history
        io.readEntry.mask   := debugBase.mask
        io.readEntry.brMask := debugBase.brMask & debugBase.mask
        io.readEntry.cfiIdx.valid := Mux(
            debugCorrectionValid,
            true.B,
            debugBase.cfiIdx.valid
        )
        io.readEntry.cfiIdx.bits := Mux(
            debugCorrectionValid,
            debugCorrection.cfiIdx,
            debugBase.cfiIdx.bits
        )
        io.readEntry.cfiType := Mux(
            debugCorrectionValid,
            debugCorrection.cfiType,
            debugBase.cfiType
        )
        io.readEntry.cfiIsCall := Mux(
            debugCorrectionValid,
            debugCorrection.isCall,
            debugBase.cfiIsCall
        )
        io.readEntry.cfiIsRet := Mux(
            debugCorrectionValid,
            debugCorrection.isRet,
            debugBase.cfiIsRet
        )
        io.readEntry.taken := Mux(
            debugCorrectionValid,
            debugCorrection.actualTaken,
            debugBase.cfiTaken
        )
        io.readEntry.target := Mux(
            debugCorrectionValid,
            debugCorrection.actualTarget,
            debugBase.cfiTarget
        )
        io.readEntry.pretarget := debugBase.cfiTarget
    }

    when(io.enq.fire) {
        assert(samePtr(io.enq.bits.ftqPtr, allocTail))
    }

    when(io.branchRedirect.valid) {
        assert(!io.enq.fire)
        assert(inWindow(io.branchRedirect.bits.ptr, trainHead, allocTail))
        assert(recoveryLongHistoryValid)
    }

    when(io.hardRedirect) {
        assert(!io.enq.fire)
        assert(!io.branchRedirect.valid)
        assert(!io.train.fire)
        assert(!trainAdvance)
    }

    val priorHardCut = hardCutMem(hardCutPtr.idx)
    when(io.hardRedirect && retiredStillLive) {
        assert(trainSidecar(hardCutPtr.idx).high === hardCutPtr.high)
        when(priorHardCut.valid && samePtr(priorHardCut.ptr, hardCutPtr)) {
            // A hard flush cannot retire more instructions from the retained
            // partial bundle. Intersecting the registered sidecar is exactly
            // equivalent to rebuilding the mask while this cut is monotonic.
            assert(activeLastRetired.bits.offset <= priorHardCut.lastOffset)
        }
        when(samePtr(hardCutPtr, trainHead)) {
            assert(
                trainSidecar(hardCutPtr.idx).fullMask ===
                    effectiveHeadFullMask
            )
        }
    }

    when(io.retire.valid) {
        assert(inWindow(io.retire.bits.ptr, trainHead, allocTail))
        when(io.retire.bits.completed.valid) {
            assert(inWindow(io.retire.bits.completed.bits, trainHead, allocTail))
        }
    }

    when(io.train.valid) {
        assert(pendingNow)
        assert(effectiveHeadCandidateMask.orR)
        assert(effectiveHeadFullMask === decodedTrainMask)
        assert(trainCandidates.orR)
        assert(trainCandidates === effectiveHeadCandidateMask)
        assert(samePtr(trainBase.ptr, trainHead))
        assert(PopCount(trainOH) === 1.U)
        when(trainBase.brMask(trainSlot)) {
            assert(headPredictorMeta.valid)
            assert(predictorSidecarRead.high === trainHead.high)
        }
    }

    assert(liveCount <= nftq.U)
}
