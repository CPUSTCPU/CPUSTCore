package CPUSTC.predict

import chisel3._
import chisel3.util._
import CPUSTC.config.Fetch._
import CPUSTC.config.JumpOp._
import CPUSTC.config.MaskLower
import CPUSTC.config.Predict.BTBMini.useRamBtb
import CPUSTC.config.Predict.BIM.{counterWidth => bimCounterWidth, enabled => bimEnabled}
import CPUSTC.config.Predict.GShare.{counterWidth => gshareCounterWidth, enabled => gshareEnabled, historyLength}
import CPUSTC.config.Predict.Agree.{
    chooserInitial,
    chooserWidth,
    counterWidth => agreeCounterWidth,
    enabled => agreeEnabled
}
import CPUSTC.frontend.IFUFetchReq

/* ---------------- BPU top-level external interfaces ---------------- */
class BPUResp extends Bundle {
    val pc        = UInt(32.W)
    val lookupId  = UInt(AdvancedPredictorConfig.lookupIdWidth.W)
    val taken     = Vec(nfch, Bool())
    val pretarget = Vec(nfch, Valid(UInt(32.W)))
    val predType  = Vec(nfch, UInt(2.W))
    val btbMeta   = Vec(nfch, new BtbPredictionMeta)
    val history   = UInt(historyLength.W)
    val longHistory = UInt(AdvancedPredictorConfig.historyWidth.W)
}

class IFUToBPUIO extends Bundle {
    val req  = Decoupled(new IFUFetchReq)
    val resp = Flipped(Valid(new BPUResp))
    val auxResp = Flipped(Valid(new AdvancedPredictorAuxResp))
    // Squash the packet currently inside the BPU pipe on redirect / flush.
    val kill = Output(Bool())
}

/* ---------------- BPU top-level interfaces ---------------- */
class BPUPreDecodeIO extends Bundle {
    val isBr         = Input(Vec(nfch, Bool()))
    val jumpEn       = Input(Vec(nfch, Bool()))
    val baseIsBr     = Input(Vec(nfch, Bool()))
    val baseJumpEn   = Input(Vec(nfch, Bool()))
    val predType     = Input(UInt(2.W))
    val pc           = Input(UInt(32.W))
    val history      = Input(UInt(historyLength.W))
    val longHistory  = Input(UInt(AdvancedPredictorConfig.historyWidth.W))
    val returnOffset = Output(UInt(32.W))
    val historyRepair = Input(Bool())
    val baseHistoryRepair = Input(Bool())
    val deferredHistoryRepair = Input(Bool())
    val targetOnlyRepair = Input(Bool())
    val flush        = Input(Bool())
}

class BPUTrainUpdate extends Bundle {
    val pc         = UInt(32.W)
    val target     = UInt(32.W)
    val taken      = Bool()
    val predType   = UInt(2.W)
    val predHit    = Bool()
    val mispredict = Bool()
    val btb         = Valid(new BtbPacketTrain)
    val predictor   = new AdvancedPredictorPacketMeta
}

class BPUCommitIO extends Bundle {
    val train = Flipped(Decoupled(new BPUTrainUpdate))
    val flush = Input(Bool())
    val historyRecovery = Input(Valid(UInt(historyLength.W)))
    val longHistoryRecovery = Input(
        Valid(UInt(AdvancedPredictorConfig.historyWidth.W))
    )
}
class BPUIO extends Bundle {
    val ifu     = Flipped(new IFUToBPUIO)
    val pd      = new BPUPreDecodeIO
    val cmt     = new BPUCommitIO
    val pdStall = Input(Bool())
}

/* -------- BPU: BTB-local direction + RAS, fixed 1-cycle response -------- */
class BPU(useBlackBoxRam: Boolean = false) extends Module {
    val io = IO(new BPUIO)

    val btbM: BTBMiniBase = if (useRamBtb) {
        Module(new BTBMini(useBlackBoxRam))
    } else {
        Module(new LegacyBTBMini)
    }
    val ras  = Module(new RAS(useBlackBoxRam))

    // Predictor training is deliberately separated from the FTQ/retirement
    // domain. The predictors consume this register every cycle, so the input
    // remains one-update-per-cycle without a combinational ready path.
    val trainValidReg = RegInit(false.B)
    val trainBitsReg = Reg(new BPUTrainUpdate)

    io.cmt.train.ready := btbM.io.bpu.initDone
    trainValidReg := io.cmt.train.fire
    when(io.cmt.train.fire) {
        trainBitsReg := io.cmt.train.bits
    }

    val trainPredType = Mux(trainValidReg, trainBitsReg.predType, NOP)

    val speculativeHistory = RegInit(0.U(historyLength.W))
    val committedHistory = RegInit(0.U(historyLength.W))
    val requestHistory = Wire(UInt(historyLength.W))
    val predictorLookupHistory = Wire(UInt(historyLength.W))
    val speculativeLongHistory = RegInit(
        0.U(AdvancedPredictorConfig.historyWidth.W)
    )
    val committedLongHistory = RegInit(
        0.U(AdvancedPredictorConfig.historyWidth.W)
    )
    val requestLongHistory = Wire(
        UInt(AdvancedPredictorConfig.historyWidth.W)
    )
    val predictorLookupLongHistory = Wire(
        UInt(AdvancedPredictorConfig.historyWidth.W)
    )

    /* stage 1: accept the request */
    io.ifu.req.ready := true.B          // fixed latency, one packet per cycle
    val s1Fire = io.ifu.req.fire
    val s1PC   = io.ifu.req.bits.pc
    val lookupIdCounter = RegInit(
        0.U(AdvancedPredictorConfig.lookupIdWidth.W)
    )
    val s1LookupId = lookupIdCounter
    when(s1Fire) {
        lookupIdCounter := lookupIdCounter + 1.U
    }

    btbM.io.bpu.pc := s1PC              // BTB async read: registered below

    /* stage 2: register and align the BTB outputs */
    // A backend redirect kills the old stage-2 packet but may replace it with
    // the redirect target in stage 1. Frontend repairs still perform a pure
    // kill because cmt.flush is false for those events.
    val s2Valid = RegNext(
        s1Fire && (!io.ifu.kill || io.cmt.flush),
        false.B
    )
    val s2PC       = RegEnable(s1PC,                 s1Fire)
    val s2LookupId = RegEnable(s1LookupId,           s1Fire)
    val s2Mask     = RegEnable(io.ifu.req.bits.mask,        s1Fire)
    val s2History  = RegEnable(requestHistory, s1Fire)
    val s2LongHistory = RegEnable(requestLongHistory, s1Fire)
    val s2RValidRaw = Wire(Vec(nfch, Bool()))
    val s2PredTypeRaw = Wire(Vec(nfch, UInt(2.W)))
    val s2JumpTgt = Wire(Vec(nfch, UInt(32.W)))
    val s2BtbJumpCandRaw = Wire(Vec(nfch, Bool()))
    val s2BtbMetaRaw = Wire(Vec(nfch, new BtbPredictionMeta))
    val s2RawPredType = Wire(Vec(nfch, UInt(2.W)))
    val s2RawBias = Wire(Vec(nfch, Bool()))
    val s2RawLocalCtrMsb = Wire(Vec(nfch, Bool()))
    val s2HistoryHitRaw = Wire(Vec(nfch, Bool()))
    val s2RawIsConditional = Wire(Vec(nfch, Bool()))

    if (useRamBtb) {
        s2RValidRaw    := btbM.io.bpu.rValid
        s2PredTypeRaw  := btbM.io.bpu.predType
        s2JumpTgt      := btbM.io.bpu.jumpTgt
        s2BtbJumpCandRaw := btbM.io.bpu.jumpCandidate
        s2BtbMetaRaw     := btbM.io.bpu.meta
        s2RawPredType    := btbM.io.bpu.rawPredType
        s2RawBias        := btbM.io.bpu.rawBias
        s2RawLocalCtrMsb := btbM.io.bpu.rawLocalCtrMsb
        s2HistoryHitRaw  := btbM.io.bpu.historyHit
        s2RawIsConditional := btbM.io.bpu.rawIsConditional
    } else {
        s2RValidRaw    := RegEnable(btbM.io.bpu.rValid, s1Fire)
        s2PredTypeRaw  := RegEnable(btbM.io.bpu.predType, s1Fire)
        s2JumpTgt      := RegEnable(btbM.io.bpu.jumpTgt, s1Fire)
        s2BtbJumpCandRaw := RegEnable(btbM.io.bpu.jumpCandidate, s1Fire)
        s2BtbMetaRaw     := RegEnable(btbM.io.bpu.meta, s1Fire)
        s2RawPredType    := RegEnable(btbM.io.bpu.rawPredType, s1Fire)
        s2RawBias        := RegEnable(btbM.io.bpu.rawBias, s1Fire)
        s2RawLocalCtrMsb := RegEnable(btbM.io.bpu.rawLocalCtrMsb, s1Fire)
        s2HistoryHitRaw  := RegEnable(btbM.io.bpu.historyHit, s1Fire)
        s2RawIsConditional := RegEnable(
            btbM.io.bpu.rawIsConditional,
            s1Fire
        )
    }
    val s2JumpCandRaw = Wire(Vec(nfch, Bool()))
    val s2BtbMeta = Wire(Vec(nfch, new BtbPredictionMeta))
    val s2FastDirection = Wire(Vec(nfch, Bool()))
    if (agreeEnabled) {
        val agree = Module(new Agree(useBlackBoxRam))
        agree.io.pc := s1PC
        agree.io.history := predictorLookupHistory
        agree.io.packetTrain.valid := trainValidReg && trainBitsReg.btb.valid
        agree.io.packetTrain.bits := trainBitsReg.btb.bits
        for (slot <- 0 until nfch) {
            val agreeDirection =
                agree.io.counters(slot)(agreeCounterWidth - 1) ===
                    s2BtbMetaRaw(slot).bias
            val rawAgreeDirection =
                agree.io.counters(slot)(agreeCounterWidth - 1) ===
                    s2RawBias(slot)
            s2JumpCandRaw(slot) := Mux(
                agree.io.ready &&
                    agree.io.chooseAgree(slot)(chooserWidth - 1),
                agreeDirection,
                s2BtbJumpCandRaw(slot)
            )
            s2FastDirection(slot) := Mux(
                agree.io.ready &&
                    agree.io.chooseAgree(slot)(chooserWidth - 1),
                rawAgreeDirection,
                s2RawLocalCtrMsb(slot)
            )
            s2BtbMeta(slot) := s2BtbMetaRaw(slot)
            s2BtbMeta(slot).predictorCtr := Mux(
                agree.io.ready,
                agree.io.counters(slot).pad(
                    s2BtbMetaRaw(slot).predictorCtr.getWidth
                ),
                s2BtbMetaRaw(slot).predictorCtr
            )
            s2BtbMeta(slot).chooseAgree := Mux(
                agree.io.ready,
                agree.io.chooseAgree(slot),
                chooserInitial.U(chooserWidth.W)
            )
        }
    } else if (gshareEnabled) {
        val gshare = Module(new GShare(useBlackBoxRam))
        gshare.io.pc := s1PC
        gshare.io.history := predictorLookupHistory
        gshare.io.packetTrain.valid := trainValidReg && trainBitsReg.btb.valid
        gshare.io.packetTrain.bits := trainBitsReg.btb.bits
        for (slot <- 0 until nfch) {
            s2FastDirection(slot) := Mux(
                gshare.io.ready,
                gshare.io.counters(slot)(gshareCounterWidth - 1),
                s2RawLocalCtrMsb(slot)
            )
            s2JumpCandRaw(slot) := Mux(
                gshare.io.ready,
                gshare.io.counters(slot)(gshareCounterWidth - 1),
                s2BtbJumpCandRaw(slot)
            )
            s2BtbMeta(slot) := s2BtbMetaRaw(slot)
            s2BtbMeta(slot).predictorCtr := Mux(
                gshare.io.ready,
                gshare.io.counters(slot),
                s2BtbMetaRaw(slot).predictorCtr
            )
        }
    } else if (bimEnabled) {
        val bim = Module(new BIM(useBlackBoxRam))
        bim.io.pc := s1PC
        bim.io.packetTrain.valid := trainValidReg && trainBitsReg.btb.valid
        bim.io.packetTrain.bits := trainBitsReg.btb.bits
        for (slot <- 0 until nfch) {
            s2FastDirection(slot) := Mux(
                bim.io.ready,
                bim.io.counters(slot)(bimCounterWidth - 1),
                s2RawLocalCtrMsb(slot)
            )
            s2JumpCandRaw(slot) := Mux(
                bim.io.ready,
                bim.io.counters(slot)(bimCounterWidth - 1),
                s2BtbJumpCandRaw(slot)
            )
            s2BtbMeta(slot) := s2BtbMetaRaw(slot)
            s2BtbMeta(slot).predictorCtr := Mux(
                bim.io.ready,
                bim.io.counters(slot),
                s2BtbMetaRaw(slot).predictorCtr
            )
        }
    } else {
        s2JumpCandRaw := s2BtbJumpCandRaw
        s2BtbMeta := s2BtbMetaRaw
        s2FastDirection := s2RawLocalCtrMsb
    }
    val s2RValid = VecInit((0 until nfch).map { i =>
        s2Mask(i) && s2RValidRaw(i)
    })
    val s2PredType = VecInit((0 until nfch).map { i =>
        Mux(s2Mask(i), s2PredTypeRaw(i), NOP)
    })
    val s2JumpCand = VecInit((0 until nfch).map { i =>
        s2Mask(i) && s2JumpCandRaw(i)
    })
    // live: the packet at stage 2 is still on the correct path
    val s2Live = s2Valid && !io.ifu.kill

    // MiniTAGE is a late correction and has one cycle of latency slack relative
    // to the ICache response. Start its BRAM lookup from the already registered
    // s2 request. This keeps the preceding BTB direction/history update out of
    // the current MiniTAGE address cone without changing the BTB/Agree fast path.
    val miniTage = Module(new MiniTage(useBlackBoxRam))
    miniTage.io.lookup.valid := s2Live
    miniTage.io.lookup.bits.pc := s2PC
    miniTage.io.lookup.bits.history := s2LongHistory

    val miniLookupAccepted = s2Live && miniTage.io.ready
    val miniBaseValid = RegNext(miniLookupAccepted, false.B)
    val miniBasePc = RegEnable(s2PC, miniLookupAccepted)
    val miniBaseLookupId = RegEnable(s2LookupId, miniLookupAccepted)
    val miniBaseTaken = RegEnable(
        s2FastDirection.asUInt,
        miniLookupAccepted
    )
    val miniBaseConditionalMask = RegEnable(
        s2RawIsConditional.asUInt & s2Mask,
        miniLookupAccepted
    )
    miniTage.io.base.valid := miniBaseValid
    miniTage.io.base.bits.pc := miniBasePc
    miniTage.io.base.bits.baseTaken := miniBaseTaken
    miniTage.io.base.bits.conditionalMask := miniBaseConditionalMask

    val miniTageChooser = Module(new MiniTageChooser(useBlackBoxRam))
    miniTageChooser.io.lookup.valid := s1Fire
    miniTageChooser.io.lookup.bits.packetPc := s1PC

    val trainSlot = trainBitsReg.pc(3, 2)
    val trainSlotOH = UIntToOH(trainSlot, nfch)
    val predictorTrainConditionalMask = VecInit((0 until nfch).map { slot =>
        trainBitsReg.btb.bits.trainMask(slot) &&
            trainBitsReg.btb.bits.isConditional(slot)
    }).asUInt
    val predictorTrainValid =
        trainValidReg && trainBitsReg.predictor.valid &&
            (predictorTrainConditionalMask & trainSlotOH).orR
    val predictorTrainPacketPc = Cat(trainBitsReg.pc(31, 4), 0.U(4.W))

    miniTage.io.train.valid :=
        predictorTrainValid && trainBitsReg.predictor.miniValid
    miniTage.io.train.bits.pc := predictorTrainPacketPc
    miniTage.io.train.bits.history := trainBitsReg.predictor.longHistory
    miniTage.io.train.bits.baseTaken :=
        trainBitsReg.predictor.miniBaseTaken
    miniTage.io.train.bits.trainMask := trainSlotOH
    miniTage.io.train.bits.takenMask := Mux(
        trainBitsReg.taken,
        trainSlotOH,
        0.U(nfch.W)
    )
    miniTage.io.train.bits.meta := trainBitsReg.predictor.miniMeta

    val chooserTrainDisagree =
        trainBitsReg.predictor.miniCandidateTaken(trainSlot) =/=
            trainBitsReg.predictor.fastTaken(trainSlot)
    miniTageChooser.io.train.valid :=
        predictorTrainValid && trainBitsReg.predictor.miniValid &&
            trainBitsReg.predictor.miniProviderHit(trainSlot) &&
            chooserTrainDisagree
    miniTageChooser.io.train.bits.packetPc := predictorTrainPacketPc
    miniTageChooser.io.train.bits.slot := trainSlot
    miniTageChooser.io.train.bits.miniCorrect :=
        trainBitsReg.predictor.miniCandidateTaken(trainSlot) ===
            trainBitsReg.taken

    val chooserS2Aligned = miniTageChooser.io.resp.valid
    val miniChooserValid = RegNext(s2Live && chooserS2Aligned, false.B)
    val miniChooserCounters = RegEnable(
        miniTageChooser.io.resp.bits.counters,
        s2Live && chooserS2Aligned
    )

    // MiniTAGE selection happens one cycle after the registered lookup. Delay
    // the tiny chooser row and packet identity by the same fixed stage. The IFU
    // can patch this result directly into any of its three live metadata slots.
    val miniAuxPacketValid = RegNext(miniBaseValid, false.B)
    val miniAuxLookupId = RegEnable(miniBaseLookupId, miniBaseValid)
    val miniAuxChooserValid = RegNext(
        miniBaseValid && miniChooserValid,
        false.B
    )
    val miniAuxChooserCounters = RegEnable(
        miniChooserCounters,
        miniBaseValid && miniChooserValid
    )
    val miniAuxResp = WireDefault(miniTage.io.resp.bits)
    for (slot <- 0 until nfch) {
        miniAuxResp.meta.slots(slot).chooserCounter := Mux(
            miniAuxChooserValid,
            miniAuxChooserCounters(slot),
            0.U
        )
    }

    // IFU pipeFlush already blocks fetch and clears every matching meta/aux
    // valid bit. Keeping kill out of this registered late-response valid avoids
    // a redirect -> aux -> predecode -> history feedback path.
    io.ifu.auxResp.valid := miniAuxPacketValid && miniTage.io.resp.valid
    io.ifu.auxResp.bits :=
        0.U.asTypeOf(new AdvancedPredictorAuxResp)
    io.ifu.auxResp.bits.lookupId := miniAuxLookupId
    io.ifu.auxResp.bits.pc := miniTage.io.resp.bits.pc
    io.ifu.auxResp.bits.miniValid := miniTage.io.resp.valid
    io.ifu.auxResp.bits.mini := miniAuxResp

    if (!useBlackBoxRam) {
        when(!reset.asBool) {
            assert(miniTage.io.resp.valid === miniAuxPacketValid)
        }
        when(!reset.asBool && miniTageChooser.io.resp.valid) {
            assert(miniTageChooser.io.resp.bits.packetPc === s2PC)
        }
    }

    val exactTaken = VecInit((0 until nfch).map { i =>
        s2Live && s2Mask(i) && s2RValidRaw(i) &&
            Mux(s2RawIsConditional(i), s2JumpCandRaw(i), true.B)
    })
    val historyTaken = VecInit((0 until nfch).map { i =>
        s2Live &&
            s2Mask(i) &&
            s2HistoryHitRaw(i) &&
            Mux(s2RawIsConditional(i), s2FastDirection(i), true.B)
    })
    // NOP plus taken denotes a generic unconditional B/JIRL in the history
    // metadata view. Presence is carried by taken, leaving BR exclusive to a
    // conditional branch without widening BPUResp.
    val historyPredType = VecInit((0 until nfch).map { i =>
        Mux(
            s2Live && s2Mask(i) && s2HistoryHitRaw(i),
            Mux(
                s2RawIsConditional(i),
                BR,
                Mux(s2RawPredType(i) === BR, NOP, s2RawPredType(i))
            ),
            NOP
        )
    })

    val predictedTakenMask = historyTaken.asUInt
    val firstTaken = PriorityEncoderOH(predictedTakenMask)
    val historyVisibleMask = Mux(
        predictedTakenMask.orR,
        MaskLower(firstTaken),
        Fill(nfch, 1.U(1.W))
    )
    val predictedBranchMask = VecInit((0 until nfch).map { i =>
        s2HistoryHitRaw(i) && s2RawIsConditional(i)
    }).asUInt & s2Mask & historyVisibleMask
    val predictedHistory = GlobalHistory.advance(
        s2History,
        predictedBranchMask,
        predictedTakenMask
    )
    val predictedLongHistory = GlobalHistory.advance(
        s2LongHistory,
        predictedBranchMask,
        predictedTakenMask
    )
    when(!reset.asBool) {
        for (i <- 0 until nfch) {
            when(s2RValid(i)) {
                assert(s2HistoryHitRaw(i))
                assert(historyTaken(i) === exactTaken(i))
            }
        }
    }

    val basePdHistory = GlobalHistory.advance(
        io.pd.history,
        io.pd.baseIsBr.asUInt,
        io.pd.baseJumpEn.asUInt
    )
    val basePdLongHistory = GlobalHistory.advance(
        io.pd.longHistory,
        io.pd.baseIsBr.asUInt,
        io.pd.baseJumpEn.asUInt
    )
    val deferredHistoryRepairValid = RegNext(
        io.pd.deferredHistoryRepair && !io.cmt.flush,
        false.B
    )
    val deferredHistory = RegEnable(
        io.pd.history,
        io.pd.deferredHistoryRepair
    )
    val deferredLongHistory = RegEnable(
        io.pd.longHistory,
        io.pd.deferredHistoryRepair
    )
    val deferredBrMask = RegEnable(
        io.pd.isBr.asUInt,
        io.pd.deferredHistoryRepair
    )
    val deferredTakenMask = RegEnable(
        io.pd.jumpEn.asUInt,
        io.pd.deferredHistoryRepair
    )
    val deferredPdHistory = GlobalHistory.advance(
        deferredHistory,
        deferredBrMask,
        deferredTakenMask
    )
    val deferredPdLongHistory = GlobalHistory.advance(
        deferredLongHistory,
        deferredBrMask,
        deferredTakenMask
    )
    val trainBranchMask = VecInit((0 until nfch).map { i =>
        trainBitsReg.btb.bits.isConditional(i)
    }).asUInt & trainBitsReg.btb.bits.trainMask
    val trainUpdatesHistory = trainValidReg && trainBitsReg.btb.valid
    val committedAfterTrain = Mux(
        trainUpdatesHistory,
        GlobalHistory.advance(
            trainBitsReg.btb.bits.history,
            trainBranchMask,
            trainBitsReg.btb.bits.takenMask
        ),
        committedHistory
    )
    val committedLongAfterTrain = Mux(
        trainUpdatesHistory,
        GlobalHistory.advance(
            Mux(
                trainBitsReg.predictor.valid,
                trainBitsReg.predictor.longHistory,
                committedLongHistory
            ),
            trainBranchMask,
            trainBitsReg.btb.bits.takenMask
        ),
        committedLongHistory
    )
    when(trainUpdatesHistory) {
        committedHistory := committedAfterTrain
        committedLongHistory := committedLongAfterTrain
    }

    val historyAfterPrediction = Mux(
        s2Live,
        predictedHistory,
        speculativeHistory
    )
    val longHistoryAfterPrediction = Mux(
        s2Live,
        predictedLongHistory,
        speculativeLongHistory
    )
    val historyAfterPredecode = Mux(
        deferredHistoryRepairValid,
        deferredPdHistory,
        Mux(
            io.pd.baseHistoryRepair,
            basePdHistory,
            historyAfterPrediction
        )
    )
    val longHistoryAfterPredecode = Mux(
        deferredHistoryRepairValid,
        deferredPdLongHistory,
        Mux(
            io.pd.baseHistoryRepair,
            basePdLongHistory,
            longHistoryAfterPrediction
        )
    )
    val flushHistory = Mux(
        io.cmt.historyRecovery.valid,
        io.cmt.historyRecovery.bits,
        committedAfterTrain
    )
    val flushLongHistory = Mux(
        io.cmt.longHistoryRecovery.valid,
        io.cmt.longHistoryRecovery.bits,
        committedLongAfterTrain
    )
    // Base metadata repairs must reach a same-cycle live lookup. A tagged
    // direction repair is consumed one cycle later, when lateRepairActive has
    // already blocked the dead request.
    predictorLookupHistory := Mux(
        io.cmt.flush,
        flushHistory,
        Mux(
            io.pd.baseHistoryRepair,
            basePdHistory,
            historyAfterPrediction
        )
    )
    predictorLookupLongHistory := Mux(
        io.cmt.flush,
        flushLongHistory,
        Mux(
            io.pd.baseHistoryRepair,
            basePdLongHistory,
            longHistoryAfterPrediction
        )
    )
    requestHistory := Mux(io.cmt.flush, flushHistory, historyAfterPredecode)
    requestLongHistory := Mux(
        io.cmt.flush,
        flushLongHistory,
        longHistoryAfterPredecode
    )
    speculativeHistory := requestHistory
    speculativeLongHistory := requestLongHistory

    /* RAS: speculative push / pop at stage 2, driven by s2-aligned predType */
    val exactFirstTaken = PriorityEncoderOH(exactTaken.asUInt)
    val s2CallMask = VecInit(s2PredType.map(_ === CALL)).asUInt
    val s2RetMask = VecInit(s2PredType.map(_ === RET)).asUInt
    ras.io.bpu.fetchCall := (exactFirstTaken & s2CallMask).orR
    ras.io.bpu.fetchRet := (exactFirstTaken & s2RetMask).orR
    ras.io.bpu.fetchReturnOffset :=
        s2PC(31, 2) + OHToUInt(exactFirstTaken).pad(30) + 1.U
    ras.io.btbM.predType := s2PredType // retained for the shared BTB/RAS interface
    btbM.io.ras.returnOffset := ras.io.btbM.returnOffset
    ras.io.bpu.fcStall := !s2Live        // a dead packet must not move the stack
    ras.io.bpu.pdStall := io.pdStall

    // A target-only repair already kills the following BPU cycle. Restore the
    // RAS in that registered bubble, while metadata repairs remain same-cycle.
    val targetOnlyRasRepair = io.pd.targetOnlyRepair
    val targetOnlyRasRepairPending = RegNext(targetOnlyRasRepair, false.B)

    /* response (fixed 1 cycle after req) */
    // A current-cycle kill prevents RAS updates, but does not feed back
    // into the registered lookup candidate. The IFU owns the same flush and
    // discards this response, which keeps late predecode repair out of the
    // next request's BTB address path.
    io.ifu.resp.valid      := s2Valid
    io.ifu.resp.bits.pc    := s2PC
    io.ifu.resp.bits.lookupId := s2LookupId
    io.ifu.resp.bits.taken := historyTaken
    io.ifu.resp.bits.predType := historyPredType
    io.ifu.resp.bits.btbMeta  := s2BtbMeta
    io.ifu.resp.bits.history  := s2History
    io.ifu.resp.bits.longHistory := s2LongHistory
    io.ifu.resp.bits.pretarget.zipWithIndex.foreach{ case (t, i) =>
        t.valid := s2RValid(i)
        // re-mux the RET target at stage 2 with the CURRENT stack top, so a
        // call in the immediately preceding packet is already visible
        t.bits  := Mux(s2PredType(i) === RET, ras.io.btbM.returnOffset, s2JumpTgt(i))
    }

    /* backend updates: top-level owns all external traffic */
    ras.io.bpu.pdPredType := io.pd.predType
    ras.io.bpu.pdPc       := io.pd.pc
    ras.io.bpu.pdFlush    :=
        io.pd.baseHistoryRepair || deferredHistoryRepairValid ||
            targetOnlyRasRepairPending
    io.pd.returnOffset := ras.io.bpu.pdReturnOffset

    btbM.io.bpu.updatepc       := trainBitsReg.pc
    btbM.io.bpu.updatejumpTgt  := trainBitsReg.target
    btbM.io.bpu.updatepredType := trainPredType
    btbM.io.bpu.updatejumpEn   := trainBitsReg.taken
    btbM.io.bpu.packetTrain.valid := trainValidReg && trainBitsReg.btb.valid
    btbM.io.bpu.packetTrain.bits  := trainBitsReg.btb.bits
    ras.io.bpu.cmtPredType := trainPredType
    ras.io.bpu.cmtPc       := trainBitsReg.pc
    ras.io.bpu.cmtFlush    := io.cmt.flush

    when(trainValidReg) {
        assert(trainBitsReg.predType =/= NOP)
    }
    when(predictorTrainValid) {
        assert(trainBitsReg.btb.bits.basePc === predictorTrainPacketPc)
    }
    when(io.pd.historyRepair) {
        assert(io.pd.flush)
    }
    when(io.pd.deferredHistoryRepair) {
        assert(io.pd.flush)
    }
    when(deferredHistoryRepairValid && !io.cmt.flush) {
        assert(io.ifu.kill)
    }
    when(io.cmt.longHistoryRecovery.valid) {
        assert(io.cmt.historyRecovery.valid)
    }
    when(s1Fire && io.ifu.kill) {
        assert(io.cmt.flush)
    }
    when(s1Fire) {
        // Deferred predecode repair always blocks request acceptance. For every
        // real request, the history captured in s2 is exactly the history that
        // the former same-cycle MiniTAGE lookup observed.
        assert(requestLongHistory === predictorLookupLongHistory)
    }
}
