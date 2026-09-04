package CPUSTC.frontend

import chisel3._
import chisel3.util._

import CPUSTC.config._
import CPUSTC.config.CtrlFlowInstr._
import CPUSTC.config.Fetch._
import CPUSTC.config.JumpOp._
import CPUSTC.config.Predict.GShare.historyLength
import CPUSTC.decode.PreDecoder
import CPUSTC.predict.{
    AdvancedPredictorAuxResp,
    AdvancedPredictorConfig,
    AdvancedPredictorPacketMeta,
    BtbPredictionMeta,
    MiniTageChooserConfig
}

class FetchException extends Bundle {
    val valid     = Bool()
    val cause     = UInt(8.W)
    val badvValid = Bool()
    val badv      = UInt(32.W)
}

class FetchBundle extends Bundle {
    val pc     = UInt(32.W)
    val history = UInt(historyLength.W)
    val instrs = Vec(nfch, Bits(32.W))
    val exceptions = Vec(nfch, new FetchException)

    val cfiIdx    = Valid(UInt(log2Ceil(nfch).W))
    val cfiType   = UInt(CFI_SZ.W)
    val cfiIsCall = Bool()
    val cfiIsRet  = Bool()

    val taken     = Bool()
    val pretarget = UInt(32.W)

    val target = UInt(32.W)
    val ftqPtr = new FtqPtr
    val mask   = UInt(nfch.W)
    val brMask = UInt(nfch.W)
    val predHit  = UInt(nfch.W)
    val predTaken = UInt(nfch.W)
    val btbMeta = Vec(nfch, new BtbPredictionMeta)
    val branchTargets = Vec(nfch, UInt(32.W))
}

class IFUFetchReq extends Bundle {
    val pc   = UInt(32.W)
    val mask = UInt(nfch.W)
}

class NPCToIFUIO extends Bundle {
    val valid = Output(Bool())
    val req   = Output(new IFUFetchReq)

    val ready = Input(Bool())
    val miss  = Input(Bool())
}

class ICacheResp extends Bundle {
    val pc     = UInt(32.W)
    val instrs = UInt((32 * nfch).W)
    val exceptions = Vec(nfch, UInt(8.W))
    val normal = Bool()
    val mask   = UInt(nfch.W)
}

class IFUToICacheIO extends Bundle {
    val req  = Decoupled(new IFUFetchReq)
    val resp = Flipped(Decoupled(new ICacheResp))

    val flush = Output(Bool())
    val miss  = Input(Bool())
}

class BPUResp extends Bundle {
    val pc      = UInt(32.W)
    val lookupId = UInt(AdvancedPredictorConfig.lookupIdWidth.W)
    val history = UInt(historyLength.W)
    val longHistory = UInt(AdvancedPredictorConfig.historyWidth.W)
    val taken   = Vec(nfch, Bool())
    val pretarget  = Vec(nfch, Valid(UInt(32.W)))
    val predType = Vec(nfch, UInt(2.W))
    val btbMeta = Vec(nfch, new BtbPredictionMeta)
}

class IFUToBPUIO extends Bundle {
    val req  = Decoupled(new IFUFetchReq)
    val resp = Flipped(Valid(new BPUResp))
    val auxResp = Flipped(Valid(new AdvancedPredictorAuxResp))
}

class IFUMeta extends Bundle {
    val pc        = UInt(32.W)
    val mask      = UInt(nfch.W)
    val predValid = Bool()
    val pred      = new BPUResp
    val predRedirected = Bool()
}

class BpuPredecodeUpdate extends Bundle {
    val valid     = Bool()
    val cfiMask   = UInt(nfch.W)
    val brMask    = UInt(nfch.W)
    val takenMask = UInt(nfch.W)
    val baseBrMask = UInt(nfch.W)
    val baseTakenMask = UInt(nfch.W)
    val history   = UInt(historyLength.W)
    val longHistory = UInt(AdvancedPredictorConfig.historyWidth.W)
    val rasType   = UInt(2.W)
    val rasPc     = UInt(32.W)
    val historyRepair = Bool()
    val baseHistoryRepair = Bool()
    val deferredHistoryRepair = Bool()
    val targetOnlyRepair = Bool()
    val repair    = Bool()
}

class InstructionFetchUnitIO extends Bundle {
    val flush = Input(Bool())
    val redirectRequest = Input(Bool())

    val npc = Flipped(new NPCToIFUIO)
    val icache = new IFUToICacheIO
    val bpu = new IFUToBPUIO
    val bpuPredecode = Output(new BpuPredecodeUpdate)

    val fetch = Decoupled(new FetchBundle)
    val fetchPredictorMeta = Output(new AdvancedPredictorPacketMeta)
    val earlyRedirect = Valid(UInt(32.W))
    val lateRedirect  = Valid(UInt(32.W))
    val localICacheFlush = Output(Bool())
}

class InstructionFetchUnit(useBlackBoxRam: Boolean = false) extends Module {
    val io = IO(new InstructionFetchUnitIO)

    val metaValid = RegInit(VecInit(Seq.fill(icacheLatency)(false.B)))
    val metaPipe  = RegInit(VecInit(Seq.fill(icacheLatency)(0.U.asTypeOf(new IFUMeta))))
    val predictorAuxValid = RegInit(
        VecInit(Seq.fill(icacheLatency)(false.B))
    )
    val predictorAuxPipe = Reg(
        Vec(icacheLatency, new AdvancedPredictorAuxResp)
    )

    val respMetaValid = metaValid(icacheLatency - 1)
    val respMeta      = metaPipe(icacheLatency - 1)

    val auxMatches = VecInit((0 until icacheLatency).map { i =>
        io.bpu.auxResp.valid && metaValid(i) && metaPipe(i).predValid &&
            io.bpu.auxResp.bits.lookupId === metaPipe(i).pred.lookupId
    })
    val patchedAuxValid = Wire(Vec(icacheLatency, Bool()))
    val patchedAuxPipe = Wire(
        Vec(icacheLatency, new AdvancedPredictorAuxResp)
    )
    for (i <- 0 until icacheLatency) {
        patchedAuxValid(i) := predictorAuxValid(i) && metaValid(i)
        patchedAuxPipe(i) := predictorAuxPipe(i)
        when(auxMatches(i)) {
            patchedAuxValid(i) := true.B
            patchedAuxPipe(i) := io.bpu.auxResp.bits
        }
    }
    val respPredictorAuxValid = patchedAuxValid(icacheLatency - 1)
    val respPredictorAux = patchedAuxPipe(icacheLatency - 1)

    val earlyRedirectRaw       = WireDefault(false.B)
    val earlyRedirectTarget    = WireDefault(0.U(32.W))
    val detectedLateRedirect   = WireDefault(false.B)
    val detectedRedirectTarget = WireDefault(0.U(32.W))
    val ghostHistoryRepair     = WireDefault(false.B)

    // Predecode repair is intentionally registered. This prevents an ICache
    // response from passing through target checking and back into the next BTB
    // lookup in one cycle. The current fetch packet remains valid; only its
    // younger requests are discarded when the registered repair is applied.
    val lateRepairValid  = RegInit(false.B)
    val lateRepairTarget = Reg(UInt(32.W))
    val lateRepairActive = lateRepairValid && !io.flush
    io.localICacheFlush := lateRepairValid

    val fetchValid = respMetaValid && io.icache.resp.valid && !io.flush && !lateRepairActive
    val fetchFire  = WireDefault(false.B)
    
    val pipeFlush = io.flush || lateRepairActive
    val acceptRedirectRequest = io.flush && io.redirectRequest

    val pipeReady = Wire(Vec(icacheLatency, Bool()))
    pipeReady(icacheLatency - 1) :=  pipeFlush || !metaValid(icacheLatency - 1) || fetchFire

    for (i <- (0 until icacheLatency - 1).reverse) {
        pipeReady(i) := pipeReady(i + 1) || !metaValid(i)
    }

    val pipeCanAccept = pipeReady(0) && (!pipeFlush || acceptRedirectRequest)

    // ICache and BPU observe the same fetch packet. Gate each valid only with
    // the peer's ready so either both requests fire or neither one does.
    io.icache.req.valid := io.npc.valid && pipeCanAccept && io.bpu.req.ready
    io.bpu.req.valid    := io.npc.valid && pipeCanAccept && io.icache.req.ready
    io.npc.ready := pipeCanAccept && io.icache.req.ready && io.bpu.req.ready

    val reqFire = io.icache.req.fire && io.bpu.req.fire

    io.icache.req.bits  := io.npc.req
    io.icache.resp.ready := fetchFire || pipeFlush
    io.icache.flush := pipeFlush

    io.bpu.req.bits  := io.npc.req

    io.npc.miss := io.icache.miss

    val stage0PredHit =
        metaValid(0) &&
        io.bpu.resp.valid &&
        io.bpu.resp.bits.pc === metaPipe(0).pc
    // History may use a partial-tag BTB match. Only an exact full-tag match is
    // allowed to redirect or consume a target.
    val earlyRedirectVec = VecInit((0 until nfch).map { i =>
        io.bpu.resp.bits.taken(i) &&
            io.bpu.resp.bits.pretarget(i).valid
    })
    val earlyRedirectIdx = PriorityEncoder(earlyRedirectVec.asUInt)

    // The BPU response belongs to metaPipe(0). Frontend redirects NPC
    // combinationally, so this cycle's request is the predicted target rather
    // than a younger sequential packet.
    earlyRedirectRaw :=
        stage0PredHit &&
        earlyRedirectVec.asUInt.orR &&
        !pipeFlush
    earlyRedirectTarget := io.bpu.resp.bits.pretarget(earlyRedirectIdx).bits

    // Keep the early prediction independent of a simultaneous late repair.
    // Frontend gives the repair priority for state recovery, while pipeFlush
    // prevents this cycle's speculative request from being accepted.
    io.earlyRedirect.valid := earlyRedirectRaw
    io.earlyRedirect.bits  := earlyRedirectTarget

    val stage0WithPred = WireDefault(metaPipe(0))
    when (stage0PredHit) {
        stage0WithPred.predValid := true.B
        stage0WithPred.pred      := io.bpu.resp.bits
        stage0WithPred.predRedirected := earlyRedirectRaw
    }

    when (pipeFlush) {
        for (i <- 0 until icacheLatency) {
            metaValid(i) := false.B
            predictorAuxValid(i) := false.B
        }
        when(acceptRedirectRequest && reqFire) {
            metaValid(0) := true.B
            metaPipe(0).pc := io.npc.req.pc
            metaPipe(0).mask := io.npc.req.mask
            metaPipe(0).predValid := false.B
            metaPipe(0).predRedirected := false.B
            predictorAuxValid(0) := false.B
        }
    }.otherwise {
        for (i <- (1 until icacheLatency).reverse) {
            when (pipeReady(i)) {
                metaValid(i) := metaValid(i - 1)
                predictorAuxValid(i) := patchedAuxValid(i - 1)
                when(patchedAuxValid(i - 1)) {
                    predictorAuxPipe(i) := patchedAuxPipe(i - 1)
                }
                if (i == 1) {
                    metaPipe(i) := stage0WithPred
                } else {
                    metaPipe(i) := metaPipe(i - 1)
                }
            }.elsewhen(auxMatches(i)) {
                predictorAuxValid(i) := true.B
                predictorAuxPipe(i) := io.bpu.auxResp.bits
            }
        }

        when (pipeReady(0)) {
            metaValid(0) := reqFire
            predictorAuxValid(0) := false.B
            when (reqFire) {
                metaPipe(0).pc   := io.npc.req.pc
                metaPipe(0).mask := io.npc.req.mask
                metaPipe(0).predValid := false.B
                metaPipe(0).predRedirected := false.B
            }
        }.elsewhen(stage0PredHit) {
            metaPipe(0) := stage0WithPred
        }
        when(!pipeReady(0) && auxMatches(0)) {
            predictorAuxValid(0) := true.B
            predictorAuxPipe(0) := io.bpu.auxResp.bits
        }
    }

    val pred = Mux(
        respMeta.predValid,
        respMeta.pred,
        0.U.asTypeOf(new BPUResp)
    )

    when (fetchValid) {
        assert(io.icache.resp.bits.pc === respMeta.pc, "IFU expects ICache response pc to match the pending fetch request")
    }
    io.fetch.bits  := 0.U.asTypeOf(new FetchBundle)
    io.fetch.bits.pc     := respMeta.pc
    io.fetch.bits.history := pred.history

    val instrs = VecInit.tabulate(nfch) { i =>
        io.icache.resp.bits.instrs(32 * (i + 1) - 1, 32 * i)
    }
    io.fetch.bits.instrs := instrs

    val respMask = respMeta.mask & io.icache.resp.bits.mask
    val rawFetchExceptionMask = VecInit((0 until nfch).map { i =>
        respMask(i) && io.icache.resp.bits.exceptions(i).orR
    }).asUInt
    val fetchExceptionPresent = rawFetchExceptionMask.orR
    val firstFetchExceptionOH = PriorityEncoderOH(rawFetchExceptionMask)
    val normalPacket = io.icache.resp.bits.normal

    val predecoders = Seq.fill(nfch) { Module(new PreDecoder) }

    for (i <- 0 until nfch) {
        predecoders(i).io.instr := instrs(i)
        predecoders(i).io.pc    := respMeta.pc + (i * 4).U
    }

    val predecode = VecInit(predecoders.map(_.io.out))
    val brVec = VecInit((0 until nfch).map { i =>
        respMask(i) && predecode(i).cfiType === CFI_BR
    })
    val baseCondPredTaken = VecInit((0 until nfch).map { i =>
        Mux(
            pred.pretarget(i).valid,
            pred.taken(i),
            predecode(i).staticTaken
        )
    })
    val miniDirectionValid = VecInit((0 until nfch).map { i =>
        respPredictorAuxValid && respPredictorAux.miniValid &&
            respPredictorAux.mini.providerHitMask(i) &&
            respPredictorAux.mini.meta.slots(i).chooserCounter(
                MiniTageChooserConfig.counterWidth - 1
            )
    })
    val effectiveCondPredTaken = if (AdvancedPredictorConfig.miniTageOverrideEnabled) {
        VecInit((0 until nfch).map { i =>
            Mux(
                miniDirectionValid(i),
                respPredictorAux.mini.candidateTaken(i),
                baseCondPredTaken(i)
            )
        })
    } else {
        baseCondPredTaken
    }
    val redirectVec = VecInit((0 until nfch).map { i =>
        respMask(i) && (
            predecode(i).cfiType === CFI_BL ||
            (predecode(i).cfiType === CFI_JIRL && pred.pretarget(i).valid) ||
            (predecode(i).cfiType === CFI_BR && effectiveCondPredTaken(i))
        )
    })
    val baseRedirectVec = VecInit((0 until nfch).map { i =>
        respMask(i) && (
            predecode(i).cfiType === CFI_BL ||
            (predecode(i).cfiType === CFI_JIRL && pred.pretarget(i).valid) ||
            (predecode(i).cfiType === CFI_BR && baseCondPredTaken(i))
        )
    })
    val redirectValid = redirectVec.asUInt.orR
    val redirectIdx = PriorityEncoder(redirectVec.asUInt)
    val baseRedirectValid = baseRedirectVec.asUInt.orR
    val baseRedirectIdx = PriorityEncoder(baseRedirectVec.asUInt)
    io.fetch.valid := fetchValid
    fetchFire := io.fetch.valid && io.fetch.ready
    val normalFetchFire = fetchFire && normalPacket
    val packetRedirectValid = redirectValid && normalPacket
    val advancedDirectionRepair = normalFetchFire && (
        redirectValid =/= baseRedirectValid ||
            (redirectValid && baseRedirectValid &&
                redirectIdx =/= baseRedirectIdx)
    )

    val selPredecode = predecode(redirectIdx)
    val selPredTarget = pred.pretarget(redirectIdx)
    val actualRedirectTarget = Mux(
        selPredecode.cfiType === CFI_JIRL && selPredTarget.valid,
        selPredTarget.bits,
        selPredecode.target
    )

    val predictedRedirectVec = VecInit((0 until nfch).map { i =>
        respMask(i) && pred.taken(i) && pred.pretarget(i).valid
    })
    val predictedRedirectValid = predictedRedirectVec.asUInt.orR
    val predictedRedirectIdx = PriorityEncoder(predictedRedirectVec.asUInt)
    val predictedRedirectTarget = pred.pretarget(predictedRedirectIdx).bits
    val predictedPathMatches =
        redirectValid &&
        predictedRedirectValid &&
        redirectIdx === predictedRedirectIdx &&
        actualRedirectTarget === predictedRedirectTarget
    val baseSelPredecode = predecode(baseRedirectIdx)
    val baseSelPredTarget = pred.pretarget(baseRedirectIdx)
    val baseActualRedirectTarget = Mux(
        baseSelPredecode.cfiType === CFI_JIRL && baseSelPredTarget.valid,
        baseSelPredTarget.bits,
        baseSelPredecode.target
    )
    val basePredictedPathMatches =
        baseRedirectValid &&
        predictedRedirectValid &&
        baseRedirectIdx === predictedRedirectIdx &&
        baseActualRedirectTarget === predictedRedirectTarget

    // Predecode either confirms the path already selected by the BPU or
    // repairs it. A false BTB hit returns to the next sequential fetch block.
    val needsLateRedirect = Mux(
        respMeta.predRedirected,
        !predictedPathMatches,
        redirectValid
    )
    val sequentialTarget = respMeta.pc + (nfch * 4).U
    detectedRedirectTarget := Mux(redirectValid, actualRedirectTarget, sequentialTarget)
    detectedLateRedirect :=
        (normalFetchFire && needsLateRedirect) || advancedDirectionRepair ||
            ghostHistoryRepair

    io.lateRedirect.valid := lateRepairActive
    io.lateRedirect.bits  := lateRepairTarget

    // Payload is meaningful only with lateRepairValid. Capture it freely so the
    // long predecode decision controls one valid FF instead of 32 payload CEs.
    lateRepairTarget := detectedRedirectTarget
    when(io.flush) {
        lateRepairValid := false.B
    }.otherwise {
        lateRepairValid := detectedLateRedirect
    }

    val redirectOH = UIntToMask(redirectIdx, nfch)
    val maskUntilRedirect = MaskLower(redirectOH)
    val baseRedirectOH = UIntToMask(baseRedirectIdx, nfch)
    val baseMaskUntilRedirect = MaskLower(baseRedirectOH)

    val normalEffectiveMask = Mux(
        redirectValid,
        respMask & maskUntilRedirect,
        respMask
    )
    val baseNormalEffectiveMask = Mux(
        baseRedirectValid,
        respMask & baseMaskUntilRedirect,
        respMask
    )
    val normalControlMask = Mux(normalPacket, normalEffectiveMask, 0.U(nfch.W))
    val baseNormalControlMask = Mux(
        normalPacket,
        baseNormalEffectiveMask,
        0.U(nfch.W)
    )
    val effectiveMask = Mux(
        fetchExceptionPresent,
        firstFetchExceptionOH,
        normalEffectiveMask
    )

    val actualPredTypes = VecInit((0 until nfch).map { i =>
        Mux(
            predecode(i).isCall,
            CALL,
            Mux(
                predecode(i).isRet,
                RET,
                Mux(predecode(i).cfiType === CFI_BR, BR, NOP)
            )
        )
    })
    val actualTakenMask = Mux(
        packetRedirectValid,
        UIntToOH(redirectIdx, nfch),
        0.U(nfch.W)
    )
    val baseActualTakenMask = Mux(
        baseRedirectValid && normalPacket,
        UIntToOH(baseRedirectIdx, nfch),
        0.U(nfch.W)
    )
    val actualCfiMask = VecInit(predecode.map(_.cfiType =/= CFI_X)).asUInt &
        normalControlMask

    val rasMask = VecInit(actualPredTypes.map { predType =>
        predType === CALL || predType === RET
    }).asUInt & normalControlMask
    val rasIdx = PriorityEncoder(rasMask)

    val predictedTakenMask = pred.taken.asUInt & respMask
    val predictedTaken = predictedTakenMask.orR
    val predictedTakenIdx = PriorityEncoder(predictedTakenMask)
    val predictedEffectiveMask = Mux(
        predictedTaken,
        respMask & MaskLower(UIntToMask(predictedTakenIdx, nfch)),
        respMask
    )
    val predictedTypeBits = VecInit((0 until nfch).map { i =>
        Mux(predictedEffectiveMask(i), pred.predType(i), NOP)
    }).asUInt
    val actualTypeBits = VecInit((0 until nfch).map { i =>
        Mux(normalControlMask(i), actualPredTypes(i), NOP)
    }).asUInt
    val baseActualTypeBits = VecInit((0 until nfch).map { i =>
        Mux(baseNormalControlMask(i), actualPredTypes(i), NOP)
    }).asUInt
    val normalizedPredictedTaken = Mux(
        predictedTaken,
        UIntToOH(predictedTakenIdx, nfch),
        0.U(nfch.W)
    )

    val metadataRepair = normalFetchFire && (
        predictedTypeBits =/= actualTypeBits ||
        normalizedPredictedTaken =/= actualTakenMask
    )
    val baseMetadataRepair = normalFetchFire && (
        predictedTypeBits =/= baseActualTypeBits ||
        normalizedPredictedTaken =/= baseActualTakenMask
    )
    val ghostMetadataMask = VecInit((0 until nfch).map { i =>
        !pred.pretarget(i).valid &&
            (pred.taken(i) || pred.predType(i) =/= NOP)
    })
    // A partial-tag collision may have advanced the speculative history even
    // when the architectural path is sequential. Keep the current packet, then
    // use the existing registered late-repair bubble to discard younger packets
    // and restart from the predecoded next PC.
    ghostHistoryRepair := normalFetchFire && ghostMetadataMask.asUInt.orR

    // A late tagged-predictor direction change always schedules the existing
    // registered late redirect. Keep its wide history repair off the current
    // cycle's feedback cone; the BPU applies this checkpoint in that bubble.
    val deferredHistoryRepair = advancedDirectionRepair
    val baseRepair = baseMetadataRepair ||
        (normalFetchFire && respMeta.predRedirected &&
            !basePredictedPathMatches) ||
        ghostHistoryRepair

    io.bpuPredecode.valid     := normalFetchFire
    io.bpuPredecode.cfiMask   := actualCfiMask
    io.bpuPredecode.brMask    := brVec.asUInt & normalControlMask
    io.bpuPredecode.takenMask := actualTakenMask
    io.bpuPredecode.baseBrMask := brVec.asUInt & baseNormalControlMask
    io.bpuPredecode.baseTakenMask := baseActualTakenMask
    io.bpuPredecode.history   := pred.history
    io.bpuPredecode.longHistory := pred.longHistory
    io.bpuPredecode.rasType   := Mux(rasMask.orR, actualPredTypes(rasIdx), NOP)
    io.bpuPredecode.rasPc     := respMeta.pc + (rasIdx << 2) + 4.U
    io.bpuPredecode.historyRepair := metadataRepair
    io.bpuPredecode.baseHistoryRepair := baseMetadataRepair
    io.bpuPredecode.deferredHistoryRepair := deferredHistoryRepair
    io.bpuPredecode.targetOnlyRepair := baseRepair && !baseMetadataRepair
    io.bpuPredecode.repair := metadataRepair ||
        (normalFetchFire && respMeta.predRedirected && !predictedPathMatches) ||
        ghostHistoryRepair

    io.fetchPredictorMeta :=
        0.U.asTypeOf(new AdvancedPredictorPacketMeta)
    io.fetchPredictorMeta.valid := respMeta.predValid
    io.fetchPredictorMeta.longHistory := pred.longHistory
    io.fetchPredictorMeta.fastTaken := baseCondPredTaken.asUInt
    io.fetchPredictorMeta.miniValid :=
        respPredictorAuxValid && respPredictorAux.miniValid
    io.fetchPredictorMeta.miniBaseTaken :=
        respPredictorAux.mini.baseTaken
    io.fetchPredictorMeta.miniCandidateTaken :=
        respPredictorAux.mini.candidateTaken
    io.fetchPredictorMeta.miniProviderHit :=
        respPredictorAux.mini.providerHitMask
    io.fetchPredictorMeta.miniMeta := respPredictorAux.mini.meta

    io.fetch.bits.mask   := effectiveMask
    io.fetch.bits.predHit :=
        VecInit(pred.pretarget.map(_.valid)).asUInt & normalControlMask
    val effectivePredTaken = VecInit((0 until nfch).map { i =>
        Mux(
            predecode(i).cfiType === CFI_BR,
            effectiveCondPredTaken(i),
            pred.taken(i)
        )
    })
    io.fetch.bits.predTaken := effectivePredTaken.asUInt & normalControlMask
    io.fetch.bits.btbMeta := pred.btbMeta
    for (i <- 0 until nfch) {
        io.fetch.bits.branchTargets(i) := predecode(i).target
        val selectedException = firstFetchExceptionOH(i)
        io.fetch.bits.exceptions(i).valid := selectedException
        io.fetch.bits.exceptions(i).cause := Mux(
            selectedException,
            io.icache.resp.bits.exceptions(i),
            0.U
        )
        io.fetch.bits.exceptions(i).badvValid := selectedException
        io.fetch.bits.exceptions(i).badv := Mux(
            selectedException,
            respMeta.pc + (i * 4).U,
            0.U
        )
    }

    io.fetch.bits.taken        := packetRedirectValid
    io.fetch.bits.cfiIdx.valid := packetRedirectValid
    io.fetch.bits.cfiIdx.bits  := redirectIdx
    io.fetch.bits.cfiType      := Mux(packetRedirectValid, selPredecode.cfiType, CFI_X)
    io.fetch.bits.cfiIsCall    := Mux(packetRedirectValid, selPredecode.isCall, false.B)
    io.fetch.bits.cfiIsRet     := Mux(packetRedirectValid, selPredecode.isRet, false.B)
    io.fetch.bits.pretarget    := Mux(packetRedirectValid && selPredTarget.valid, selPredTarget.bits, 0.U)
    io.fetch.bits.target       := Mux(packetRedirectValid, actualRedirectTarget, 0.U)
    io.fetch.bits.brMask       := brVec.asUInt & normalControlMask

    when(io.earlyRedirect.valid) {
        assert(stage0PredHit)
        assert(earlyRedirectVec.asUInt.orR)
    }
    when(io.redirectRequest) {
        assert(io.flush)
    }
    when(io.lateRedirect.valid) {
        assert(!io.icache.req.valid)
        assert(!io.bpu.req.valid)
        assert(!io.fetch.valid)
    }
    when(detectedLateRedirect) {
        assert(fetchFire)
        assert(!io.lateRedirect.valid)
    }
    when(normalFetchFire) {
        for (i <- 0 until nfch) {
            when(normalControlMask(i) && predecode(i).cfiType === CFI_BR) {
                assert(io.fetch.bits.predTaken(i) === effectiveCondPredTaken(i))
            }
        }
        assert(io.fetch.bits.taken === io.fetch.bits.cfiIdx.valid)
        when(io.fetch.bits.cfiIdx.valid) {
            assert(normalControlMask(io.fetch.bits.cfiIdx.bits))
        }
        when(deferredHistoryRepair) {
            assert(detectedLateRedirect)
            assert(actualTakenMask =/= baseActualTakenMask)
        }
    }
    if (!useBlackBoxRam) {
        assert(PopCount(auxMatches) <= 1.U)
        for (i <- 0 until icacheLatency) {
            when(auxMatches(i)) {
                assert(io.bpu.auxResp.bits.pc === metaPipe(i).pc)
            }
        }
        when(respPredictorAuxValid) {
            assert(respPredictorAux.pc === respMeta.pc)
        }
    }
    when(respPredictorAuxValid) {
        assert(respMeta.predValid)
        assert(respPredictorAux.lookupId === pred.lookupId)
    }
}
