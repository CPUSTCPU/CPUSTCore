package CPUSTC.predict

import chisel3._
import chisel3.util._

import CPUSTC.config.Fetch.nfch

object LoopPredictorConfig {
    val entries = 128
    val indexWidth = log2Ceil(entries)
    val packetOffsetWidth = log2Ceil(nfch) + 2
    val packetTagWidth = 32 - packetOffsetWidth
    val slotWidth = log2Ceil(nfch)
    val iterationWidth = 8
    val confidenceWidth = 2
    val ageWidth = 2
}

class LoopPredictorEntry extends Bundle {
    import LoopPredictorConfig._

    val valid = Bool()
    val packetTag = UInt(packetTagWidth.W)
    val slot = UInt(slotWidth.W)
    val loopDirection = Bool()
    val numIter = UInt(iterationWidth.W)
    val currentIter = UInt(iterationWidth.W)
    val confidence = UInt(confidenceWidth.W)
    val age = UInt(ageWidth.W)
}

class LoopPredictorMeta extends Bundle {
    import LoopPredictorConfig._

    val valid = Bool()
    val hit = Bool()
    val index = UInt(indexWidth.W)
    val packetTag = UInt(packetTagWidth.W)
    val entry = new LoopPredictorEntry
}

class LoopPredictorLookupReq extends Bundle {
    val packetPc = UInt(32.W)

    // currentIter is commit-time state. This may be asserted only when the
    // integration has proved that no older prediction for this loop remains
    // unretired. The bit is registered with the request, so no FTQ/commit
    // signal is returned combinationally into the lookup stage.
    val overrideSafe = Bool()
}

class LoopPredictorPrediction extends Bundle {
    import LoopPredictorConfig._

    val slot = UInt(slotWidth.W)
    val taken = Bool()
}

class LoopPredictorCommitUpdate extends Bundle {
    val pc = UInt(32.W)
    val taken = Bool()

    // Allocate is intended for a base-predictor miss at a suspected loop
    // exit. Allocation therefore initializes direction to !taken and waits
    // for one complete subsequent loop before producing a prediction.
    val allocate = Bool()
    val meta = new LoopPredictorMeta
}

class LoopPredictorIO extends Bundle {
    val lookup = Flipped(Valid(new LoopPredictorLookupReq))
    val flush = Input(Bool())
    val ready = Output(Bool())

    val meta = Output(new LoopPredictorMeta)
    val shadow = Output(Valid(new LoopPredictorPrediction))
    val overridePrediction = Output(Valid(new LoopPredictorPrediction))

    val commit = Flipped(Valid(new LoopPredictorCommitUpdate))
}

/**
  * Direct-mapped, one-candidate-per-fetch-packet loop predictor.
  *
  * The entry width is 52 bits with the current parameters. The lookup and
  * authoritative training copies each fit comfortably in one RAMB18. A full
  * packet tag is stored even though the index is folded; an index collision
  * can evict an entry but cannot create a false hit.
  *
  * numIter is the number of consecutive loopDirection outcomes before the exit.
  * currentIter is advanced by committed outcomes and is reset on the exit.
  * Thus currentIter == numIter predicts the exit direction, and all other
  * values predict loopDirection.
  *
  * Important short-loop restriction: several dynamic instances can be fetched
  * before the first one commits, so commit-time currentIter is stale for the
  * younger lookups. Shadow output deliberately exposes that behavior for
  * measurement. overridePrediction is additionally gated by overrideSafe,
  * which must come from registered integration-side in-flight tracking. Until
  * speculative iteration checkpoints and recovery exist, the safe first use
  * is shadow-only (overrideSafe tied low).
  */
class LoopPredictor(useBlackBoxRam: Boolean = false) extends Module {
    import LoopPredictorConfig._

    val io = IO(new LoopPredictorIO)

    require(entries == 128)
    require(isPow2(entries))
    require(isPow2(nfch))
    require(iterationWidth > 1)

    private val entryWidth = (new LoopPredictorEntry).getWidth
    private val maxIteration = ((1 << iterationWidth) - 1).U
    private val maxConfidence = ((1 << confidenceWidth) - 1).U
    private val maxAge = ((1 << ageWidth) - 1).U

    private def packetTag(pc: UInt): UInt = pc(31, packetOffsetWidth)

    private def packetIndex(tag: UInt): UInt = {
        val chunks = (0 until packetTagWidth by indexWidth).map { low =>
            val high = math.min(low + indexWidth - 1, packetTagWidth - 1)
            val chunk = tag(high, low)
            if (high - low + 1 == indexWidth) {
                chunk
            } else {
                chunk.pad(indexWidth)
            }
        }
        chunks.reduce(_ ^ _)
    }

    private def saturatingIncrement(value: UInt, maximum: UInt): UInt =
        Mux(value === maximum, value, value + 1.U)

    private def saturatingDecrement(value: UInt): UInt =
        Mux(value === 0.U, value, value - 1.U)

    val lookupRam = Module(
        new BpuSdpRam(entryWidth, entries, useBlackBoxRam)
    )
    val trainRam = Module(
        new BpuSdpRam(entryWidth, entries, useBlackBoxRam)
    )

    // BRAM contents have no reset. Scrub only the valid bit at startup by
    // writing a zero entry into each row.
    val clearActive = RegInit(true.B)
    val clearIndex = RegInit(0.U(indexWidth.W))
    when(clearActive) {
        when(clearIndex === (entries - 1).U) {
            clearActive := false.B
        }.otherwise {
            clearIndex := clearIndex + 1.U
        }
    }
    io.ready := !clearActive

    val lookupTag = packetTag(io.lookup.bits.packetPc)
    val lookupIndex = packetIndex(lookupTag)
    val lookupAccepted = io.lookup.valid && !clearActive

    lookupRam.io.ren := lookupAccepted
    lookupRam.io.raddr := lookupIndex

    val responseValid = RegInit(false.B)
    val responseTag = Reg(UInt(packetTagWidth.W))
    val responseIndex = Reg(UInt(indexWidth.W))
    val responseOverrideSafe = RegInit(false.B)

    when(io.flush) {
        responseValid := false.B
    }.otherwise {
        responseValid := lookupAccepted
    }
    when(lookupAccepted) {
        responseTag := lookupTag
        responseIndex := lookupIndex
        responseOverrideSafe := io.lookup.bits.overrideSafe
    }

    val commitTag = packetTag(io.commit.bits.pc)
    val commitIndex = packetIndex(commitTag)
    val commitSlot = io.commit.bits.pc(packetOffsetWidth - 1, 2)
    val commitAccepted =
        io.commit.valid && !clearActive && io.commit.bits.meta.valid

    // The training copy is authoritative. A commit first reads its current
    // row and performs the read-modify-write one cycle later. Prediction-time
    // metadata is retained only to identify a stale replacement attempt; it
    // never supplies counters or learned state to the RMW datapath.
    trainRam.io.ren := commitAccepted
    trainRam.io.raddr := commitIndex

    val trainRequestValid = RegNext(commitAccepted, false.B)
    val trainRequestTag = RegEnable(commitTag, commitAccepted)
    val trainRequestIndex = RegEnable(commitIndex, commitAccepted)
    val trainRequestSlot = RegEnable(commitSlot, commitAccepted)
    val trainRequestTaken = RegEnable(io.commit.bits.taken, commitAccepted)
    val trainRequestAllocate = RegEnable(
        io.commit.bits.allocate,
        commitAccepted
    )
    val trainRequestMetaEntryValid = RegEnable(
        io.commit.bits.meta.entry.valid,
        commitAccepted
    )
    val trainRequestMetaEntryTag = RegEnable(
        io.commit.bits.meta.entry.packetTag,
        commitAccepted
    )
    val trainRequestMetaEntrySlot = RegEnable(
        io.commit.bits.meta.entry.slot,
        commitAccepted
    )

    // A same-row request arriving while the preceding request writes must see
    // that write, independent of the RAM's read-during-write mode.
    val trainReadWriteCollisionReg = RegInit(false.B)
    val trainForwardedWriteData = Reg(UInt(entryWidth.W))
    val trainBaseData = Mux(
        trainReadWriteCollisionReg,
        trainForwardedWriteData,
        trainRam.io.rdata
    )
    val trainBase = trainBaseData.asTypeOf(new LoopPredictorEntry)

    val exactTrainHit = trainBase.valid &&
        trainBase.packetTag === trainRequestTag &&
        trainBase.slot === trainRequestSlot
    val canReplace = !trainBase.valid || trainBase.age === 0.U
    val metadataIdentityUnchanged =
        trainBase.valid === trainRequestMetaEntryValid &&
            (!trainBase.valid || (
                trainBase.packetTag === trainRequestMetaEntryTag &&
                    trainBase.slot === trainRequestMetaEntrySlot
            ))

    val trainEntry = WireDefault(trainBase)
    val trainWrite = WireDefault(false.B)

    when(trainRequestValid) {
        when(exactTrainHit) {
            trainWrite := true.B
            when(trainRequestTaken === trainBase.loopDirection) {
                trainEntry.currentIter := saturatingIncrement(
                    trainBase.currentIter,
                    maxIteration
                )

                // A continuation where the learned loop expected its exit is
                // already evidence that the trip count changed.
                when(
                    trainBase.numIter =/= 0.U &&
                        trainBase.currentIter === trainBase.numIter
                ) {
                    trainEntry.confidence :=
                        saturatingDecrement(trainBase.confidence)
                    trainEntry.age := saturatingDecrement(trainBase.age)
                }
            }.otherwise {
                val observedIterations = trainBase.currentIter
                trainEntry.currentIter := 0.U

                when(observedIterations === 0.U) {
                    trainEntry.confidence :=
                        saturatingDecrement(trainBase.confidence)
                    trainEntry.age := saturatingDecrement(trainBase.age)
                }.elsewhen(
                    trainBase.numIter =/= 0.U &&
                        observedIterations === trainBase.numIter
                ) {
                    trainEntry.confidence := saturatingIncrement(
                        trainBase.confidence,
                        maxConfidence
                    )
                    trainEntry.age := saturatingIncrement(trainBase.age, maxAge)
                }.otherwise {
                    trainEntry.numIter := observedIterations
                    trainEntry.confidence := 0.U
                    trainEntry.age := saturatingDecrement(trainBase.age)
                }
            }
        }.elsewhen(trainRequestAllocate && metadataIdentityUnchanged) {
            trainWrite := true.B
            when(canReplace) {
                trainEntry.valid := true.B
                trainEntry.packetTag := trainRequestTag
                trainEntry.slot := trainRequestSlot
                trainEntry.loopDirection := !trainRequestTaken
                trainEntry.numIter := 0.U
                trainEntry.currentIter := 0.U
                trainEntry.confidence := 0.U
                trainEntry.age := 0.U
            }.otherwise {
                // Repeated allocation pressure ages a useful colliding entry
                // before replacement rather than destroying it immediately.
                trainEntry.age := saturatingDecrement(trainBase.age)
            }
        }
    }

    val tableWriteEnable = clearActive || trainWrite
    val tableWriteAddress = Mux(
        clearActive,
        clearIndex,
        trainRequestIndex
    )
    val tableWriteData = Mux(
        clearActive,
        0.U(entryWidth.W),
        trainEntry.asUInt
    )

    lookupRam.io.wen := tableWriteEnable
    lookupRam.io.waddr := tableWriteAddress
    lookupRam.io.wdata := tableWriteData
    trainRam.io.wen := tableWriteEnable
    trainRam.io.waddr := tableWriteAddress
    trainRam.io.wdata := tableWriteData

    val trainReadWriteCollision = trainWrite && commitAccepted &&
        trainRequestIndex === commitIndex
    trainReadWriteCollisionReg := trainReadWriteCollision
    when(trainReadWriteCollision) {
        trainForwardedWriteData := trainEntry.asUInt
    }

    // Make lookup behavior independent of the FPGA's read-during-write mode.
    val writeMatchesRead = trainWrite && lookupAccepted &&
        trainRequestIndex === lookupIndex
    val writeMatchesReadReg = RegNext(writeMatchesRead, false.B)
    val forwardedWriteData = RegEnable(trainEntry.asUInt, writeMatchesRead)
    val responseData = Mux(
        writeMatchesReadReg,
        forwardedWriteData,
        lookupRam.io.rdata
    )
    val responseEntry = responseData.asTypeOf(new LoopPredictorEntry)
    val responseHit = responseValid && responseEntry.valid &&
        responseEntry.packetTag === responseTag

    io.meta.valid := responseValid
    io.meta.hit := responseHit
    io.meta.index := responseIndex
    io.meta.packetTag := responseTag
    io.meta.entry := responseEntry

    val hasLearnedCount = responseEntry.numIter =/= 0.U
    val predictsExit = responseEntry.currentIter === responseEntry.numIter
    val predictedTaken = Mux(
        predictsExit,
        !responseEntry.loopDirection,
        responseEntry.loopDirection
    )

    io.shadow.valid := responseHit && hasLearnedCount
    io.shadow.bits.slot := responseEntry.slot
    io.shadow.bits.taken := predictedTaken

    io.overridePrediction.valid := io.shadow.valid &&
        responseEntry.confidence === maxConfidence &&
        responseOverrideSafe
    io.overridePrediction.bits := io.shadow.bits

    when(io.commit.valid && !clearActive) {
        assert(io.commit.bits.meta.valid)
        assert(io.commit.bits.meta.index === commitIndex)
        assert(io.commit.bits.meta.packetTag === commitTag)
        assert(io.commit.bits.pc(1, 0) === 0.U)
    }
}
